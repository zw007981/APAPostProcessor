#include "esdf_constraint.h"

#include <cmath>
#include <stdexcept>

#include "../NMPC/vehicle_circle_geometry.h"

namespace apa_post_processor {
iLQREsdfConstraint::iLQREsdfConstraint(
    const ESDFMap& esdf_map, const VehicleFootprintModel& footprint_model,
    const iLQRConfig& config)
    : esdf_map_(esdf_map),
      config_(config),
      circle_local_centers_(vehicle_circle_geometry::ExtractLocalCircleCenters(
          footprint_model, CircleType::OUTER)),
      circle_radius_(footprint_model.getOuterRadius()) {
    // 配置错误会静默污染全部下游优化目标，必须在构造期显式拒绝
    if (!std::isfinite(config_.esdf_margin_safe) || config_.esdf_margin_safe < 0.0) {
        throw std::invalid_argument("margin_safe 必须为非负有限值");
    }
    if (!std::isfinite(config_.esdf_margin_comf) ||
        config_.esdf_margin_comf <= config_.esdf_margin_safe) {
        throw std::invalid_argument("margin_comf 必须大于 margin_safe");
    }
    if (!std::isfinite(config_.esdf_weight_safe) || config_.esdf_weight_safe < 0.0) {
        throw std::invalid_argument("weight_safe 必须为非负有限值");
    }
    if (!std::isfinite(config_.esdf_weight_comf) || config_.esdf_weight_comf < 0.0) {
        throw std::invalid_argument("weight_comf 必须为非负有限值");
    }
    if (config_.esdf_stride < 1) {
        throw std::invalid_argument("stride 必须 >= 1");
    }
    if (circle_local_centers_.empty()) {
        throw std::invalid_argument("外圆集合不能为空");
    }
    if (!std::isfinite(circle_radius_) || circle_radius_ <= 0.0) {
        throw std::invalid_argument("外圆半径必须为正有限值");
    }
}

iLQREsdfPoseCost iLQREsdfConstraint::evaluate(double x, double y,
                                            double theta) const {
    iLQREsdfPoseCost result;
    const double cos_theta = std::cos(theta);
    const double sin_theta = std::sin(theta);
    Eigen::Vector3d grad3 = Eigen::Vector3d::Zero();
    Eigen::Matrix3d hess3 = Eigen::Matrix3d::Zero();
    for (const auto& local_center : circle_local_centers_) {
        const double lx = local_center.x();
        const double ly = local_center.y();
        // 圆心世界坐标 P = (x,y) + R(θ)·p_local
        const double cx = x + cos_theta * lx - sin_theta * ly;
        const double cy = y + sin_theta * lx + cos_theta * ly;
        // 条件查询：活跃阈值为外圆半径 + margin_comf（车辆外圆模型与
        // 配置共同决定的安全值）；两个 margin 均不活跃的圆对代价/梯度/
        // Hessian 的贡献恒为零，梯度插值整体跳过，结果逐位不变
        const auto query = esdf_map_.getDistAndGradIfCloser(
            cx, cy, circle_radius_ + config_.esdf_margin_comf);
        // 防御：ESDF 查询返回非有限值（当前实现整数距离场下不可达，属
        // 保守侵入兜底，梯度取零与逐圆全量路径的防御分支一致）
        if (!std::isfinite(query.dist)) {
            const double c_safe = circle_radius_ + config_.esdf_margin_safe;
            const double c_comf = circle_radius_ + config_.esdf_margin_comf;
            result.cost +=
                config_.esdf_weight_safe * c_safe * c_safe * c_safe;
            result.cost +=
                config_.esdf_weight_comf * c_comf * c_comf * c_comf;
            continue;
        }
        const double c_safe = circle_radius_ + config_.esdf_margin_safe - query.dist;
        const double c_comf = circle_radius_ + config_.esdf_margin_comf - query.dist;
        if (c_safe <= 0.0 && c_comf <= 0.0) {
            continue;
        }
        // 活跃 ⟺ dist < r+margin_comf ⟹ 梯度必然已计算（阈值同源）；
        // 圆心随 θ 旋转的几何链式法则：dP/dθ = dR/dθ·p_local
        const auto& grad = query.grad;
        const double dcx_dtheta = -sin_theta * lx - cos_theta * ly;
        const double dcy_dtheta = cos_theta * lx - sin_theta * ly;
        const Eigen::Vector3d circle_grad(
            -grad.x(), -grad.y(),
            -(grad.x() * dcx_dtheta + grad.y() * dcy_dtheta));
        if (c_safe > 0.0) {
            result.cost +=
                config_.esdf_weight_safe * c_safe * c_safe * c_safe;
            grad3 += (3.0 * config_.esdf_weight_safe * c_safe * c_safe) *
                     circle_grad;
            hess3 += (6.0 * config_.esdf_weight_safe * c_safe) *
                     (circle_grad * circle_grad.transpose());
        }
        if (c_comf > 0.0) {
            result.cost +=
                config_.esdf_weight_comf * c_comf * c_comf * c_comf;
            grad3 += (3.0 * config_.esdf_weight_comf * c_comf * c_comf) *
                     circle_grad;
            hess3 += (6.0 * config_.esdf_weight_comf * c_comf) *
                     (circle_grad * circle_grad.transpose());
        }
    }
    // 散布到七维状态的 (x, y, θ) 行/块（布局索引 0/1/2 连续），其余恒零
    result.gradient(ILQR_IDX_X) = grad3.x();
    result.gradient(ILQR_IDX_Y) = grad3.y();
    result.gradient(ILQR_IDX_THETA) = grad3.z();
    result.hessian.topLeftCorner<3, 3>() = hess3;
    return result;
}

iLQREsdfCircleConstraint iLQREsdfConstraint::evaluateCircle(
    const Eigen::Vector2d& local_center, double cos_theta, double sin_theta,
    double x, double y) const {
    const double lx = local_center.x();
    const double ly = local_center.y();
    // 圆心世界坐标 P = (x,y) + R(θ)·p_local
    const double cx = x + cos_theta * lx - sin_theta * ly;
    const double cy = y + sin_theta * lx + cos_theta * ly;
    const auto [dist, grad] = esdf_map_.getDistAndGrad(cx, cy);
    iLQREsdfCircleConstraint result;
    // 防御：ESDF 查询返回非有限值（当前实现整数距离场下不可达，属
    if (!std::isfinite(dist)) {
        result.c_safe = circle_radius_ + config_.esdf_margin_safe;
        result.c_comf = circle_radius_ + config_.esdf_margin_comf;
        return result;
    }
    result.c_safe = circle_radius_ + config_.esdf_margin_safe - dist;
    result.c_comf = circle_radius_ + config_.esdf_margin_comf - dist;
    // 圆心随 θ 旋转的几何链式法则：dP/dθ = dR/dθ·p_local；
    // ∂C/∂(x,y,θ) = −∇d 经圆心位置的链式反传
    const double dcx_dtheta = -sin_theta * lx - cos_theta * ly;
    const double dcy_dtheta = cos_theta * lx - sin_theta * ly;
    result.grad = Eigen::Vector3d(
        -grad.x(), -grad.y(), -(grad.x() * dcx_dtheta + grad.y() * dcy_dtheta));
    return result;
}
}  // namespace apa_post_processor
