#include "minco_esdf_penalty.h"

#include <cmath>
#include <stdexcept>

#include "../NMPC/vehicle_circle_geometry.h"

namespace apa_post_processor {
MincoEsdfPenalty::MincoEsdfPenalty(const ESDFMap& esdf_map,
                               const VehicleFootprintModel& footprint_model,
                               const MincoConfig& config)
    : esdf_map_(esdf_map),
      config_(config),
      circle_local_centers_(vehicle_circle_geometry::ExtractLocalCircleCenters(
          footprint_model, CircleType::OUTER)),
      circle_radius_(footprint_model.getOuterRadius()) {
    // 配置错误会静默污染全部下游优化目标，必须在构造期显式拒绝
    if (!std::isfinite(config_.margin_safe) || config_.margin_safe < 0.0) {
        throw std::invalid_argument("margin_safe 必须为非负有限值");
    }
    if (!std::isfinite(config_.margin_comf) ||
        config_.margin_comf <= config_.margin_safe) {
        throw std::invalid_argument("margin_comf 必须大于 margin_safe");
    }
    if (!std::isfinite(config_.weight_safe) || config_.weight_safe < 0.0) {
        throw std::invalid_argument("weight_safe 必须为非负有限值");
    }
    if (!std::isfinite(config_.weight_comf) || config_.weight_comf < 0.0) {
        throw std::invalid_argument("weight_comf 必须为非负有限值");
    }
    if (circle_local_centers_.empty()) {
        throw std::invalid_argument("外圆集合不能为空");
    }
    if (!std::isfinite(circle_radius_) || circle_radius_ <= 0.0) {
        throw std::invalid_argument("外圆半径必须为正有限值");
    }
}

MincoEsdfPoseCost MincoEsdfPenalty::evaluate(double x, double y,
                                         double theta) const {
    MincoEsdfPoseCost result;
    const double cos_theta = std::cos(theta);
    const double sin_theta = std::sin(theta);
    for (const auto& local_center : circle_local_centers_) {
        accumulateCircle(local_center, cos_theta, sin_theta, x, y, result);
    }
    return result;
}

void MincoEsdfPenalty::accumulateCircle(const Eigen::Vector2d& local_center,
                                      double cos_theta, double sin_theta,
                                      double x, double y,
                                      MincoEsdfPoseCost& result) const {
    const double lx = local_center.x();
    const double ly = local_center.y();
    // 圆心世界坐标 P_k = (x,y) + R(θ)·p_k^local
    const double cx = x + cos_theta * lx - sin_theta * ly;
    const double cy = y + sin_theta * lx + cos_theta * ly;
    const auto [dist, grad] = esdf_map_.getDistAndGrad(cx, cy);
    const double c_safe = circle_radius_ + config_.margin_safe - dist;
    const double c_comf = circle_radius_ + config_.margin_comf - dist;
    // 两段三次外点罚共享同一个空间梯度方向（仅边界值不同），合并为一个
    // 梯度系数 ∂I_k/∂d = -(3·W_safe·max(0,C_safe)²
    //                  + 3·W_comf·max(0,C_comf)²)
    double factor = 0.0;
    if (c_safe > 0.0) {
        result.cost += config_.weight_safe * c_safe * c_safe * c_safe;
        factor += config_.weight_safe * 3.0 * c_safe * c_safe;
    }
    if (c_comf > 0.0) {
        result.cost += config_.weight_comf * c_comf * c_comf * c_comf;
        factor += config_.weight_comf * 3.0 * c_comf * c_comf;
    }
    if (factor <= 0.0) {
        return;
    }
    // 圆心随 θ 旋转的几何链式法则：dP_k/dθ = dR/dθ·p_k^local
    const double dcx_dtheta = -sin_theta * lx - cos_theta * ly;
    const double dcy_dtheta = cos_theta * lx - sin_theta * ly;
    // ∂I_k/∂(x,y,θ) = -factor·∇d 经圆心位置的链式反传
    result.gradient.x() -= factor * grad.x();
    result.gradient.y() -= factor * grad.y();
    result.gradient.z() -=
        factor * (grad.x() * dcx_dtheta + grad.y() * dcy_dtheta);
}
}  // namespace apa_post_processor
