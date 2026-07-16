#include "iterative_corridor_constraint.h"

#include <Eigen/Core>
#include <cmath>
#include <stdexcept>

#include "../../spatial/esdf_map.h"
#include "../../vehicle/vehicle_footprint_model.h"

namespace apa_post_processor {
IterativeCorridorConstraint::IterativeCorridorConstraint(
    const ESDFMap& esdf_map,
    const std::vector<Eigen::Vector2d>& circle_local_positions,
    double circle_radius, int global_start_idx, int constraints_per_step,
    double hard_margin)
    : esdf_map_(esdf_map),
      circle_local_positions_(circle_local_positions),
      circle_radius_(circle_radius),
      hard_margin_(hard_margin),
      global_start_idx_(global_start_idx),
      constraints_per_step_(constraints_per_step) {
    if (constraints_per_step_ <= 0) {
        throw std::invalid_argument(
            "IterativeCorridorConstraint: constraints_per_step must be "
            "positive, got " +
            std::to_string(constraints_per_step_));
    }
    if (global_start_idx_ < 0) {
        throw std::invalid_argument(
            "IterativeCorridorConstraint: global_start_idx must be "
            "non-negative, got " +
            std::to_string(global_start_idx_));
    }
    if (circle_local_positions_.empty()) {
        throw std::invalid_argument(
            "IterativeCorridorConstraint: circle_local_positions must be "
            "non-empty");
    }
    if (!(circle_radius_ > 0.0)) {
        throw std::invalid_argument(
            "IterativeCorridorConstraint: circle_radius must be positive, "
            "got " +
            std::to_string(circle_radius_));
    }
}

int IterativeCorridorConstraint::ng() const { return constraints_per_step_; }

int IterativeCorridorConstraint::localStepIndex(
    const stc_SQP::Vector& p) const {
    // p(0) = 局部步索引
    const int local_step = static_cast<int>(p(0));
    if (local_step < 0) {
        throw std::invalid_argument(
            "IterativeCorridorConstraint: negative local_step_idx=" +
            std::to_string(local_step));
    }
    return local_step;
}

bool IterativeCorridorConstraint::computeCircleConstraint(
    double x_pos, double y_pos, double theta,
    const Eigen::Vector2d& local_offset, double& dist, Eigen::Vector2d& grad,
    Eigen::Matrix<double, 1, 5>& a_row, double& g_val) const {
    const double lx = local_offset.x();
    const double ly = local_offset.y();
    const double cos_th = std::cos(theta);
    const double sin_th = std::sin(theta);

    // 世界系圆心坐标
    const double cx = x_pos + cos_th * lx - sin_th * ly;
    const double cy = y_pos + sin_th * lx + cos_th * ly;

    // 越界检查：ESDF 地图外区域没有安全保证
    if (!esdf_map_.inMap(cx, cy)) {
        g_val = 0.0;  // 正值会触发软约束 slack
        a_row.setZero();
        dist = 0.0;
        grad.setZero();
        return false;
    }

    // ESDF 查询
    const auto [d, g] = esdf_map_.getDistAndGrad(cx, cy);
    dist = d;
    grad = g;

    // ESDF 梯度退化检查
    if (!std::isfinite(dist) || !grad.allFinite() ||
        grad.norm() < kMinValidGradientNorm) {
        g_val = -1.0;
        a_row.setZero();
        return false;
    }

    // 圆心对 theta 的偏导数
    const double dcx_dtheta = -sin_th * lx - cos_th * ly;
    const double dcy_dtheta = cos_th * lx - sin_th * ly;

    // 超平面法向量 A^T = grad^T * J_C
    // J_C = [1 0 dcx_dtheta 0 0; 0 1 dcy_dtheta 0 0]
    a_row << grad.x(), grad.y(), grad.x() * dcx_dtheta + grad.y() * dcy_dtheta,
        0.0, 0.0;

    // 约束值 g = R - d_esdf：正值 → 外圆侵入障碍物
    g_val = circle_radius_ + hard_margin_ - dist;

    return true;
}

void IterativeCorridorConstraint::evaluateAndJacobian(
    const stc_SQP::Vector& x, const stc_SQP::Vector& u,
    const stc_SQP::Vector& p, stc_SQP::Vector& g, stc_SQP::Matrix& Cx,
    stc_SQP::Matrix& Cu) const {
    (void)u;
    const int ng_val = ng();
    g.resize(ng_val);
    Cx = stc_SQP::Matrix::Zero(ng_val, x.size());
    Cu.resize(ng_val, 2);
    Cu.setZero();

    const int local_step = localStepIndex(p);
    (void)local_step;

    const double x_pos = x(0);
    const double y_pos = x(1);
    const double theta = x(2);
    // §13.6：cos/sin 只算一次（原 computeCircleConstraint 每个圆都重算）
    const double cos_th = std::cos(theta);
    const double sin_th = std::sin(theta);

    const int n_circles = static_cast<int>(circle_local_positions_.size());
    const int n_constraints = std::min(constraints_per_step_, n_circles);

    for (int m = 0; m < n_constraints; ++m) {
        const double lx = circle_local_positions_[m].x();
        const double ly = circle_local_positions_[m].y();
        const double cx = x_pos + cos_th * lx - sin_th * ly;
        const double cy = y_pos + sin_th * lx + cos_th * ly;

        if (!esdf_map_.inMap(cx, cy)) {
            g(m) = -1.0;  // 越界：永远满足
            continue;
        }

        const auto [dist, grad] = esdf_map_.getDistAndGrad(cx, cy);
        if (!std::isfinite(dist) || !grad.allFinite() ||
            grad.norm() < kMinValidGradientNorm) {
            g(m) = -1.0;  // 梯度退化：永远满足
            continue;
        }

        const double dcx_dtheta = -sin_th * lx - cos_th * ly;
        const double dcy_dtheta = cos_th * lx - sin_th * ly;
        Cx(m, 0) = -grad.x();
        Cx(m, 1) = -grad.y();
        Cx(m, 2) = -(grad.x() * dcx_dtheta + grad.y() * dcy_dtheta);
        g(m) = circle_radius_ + hard_margin_ - dist;
    }
}

void IterativeCorridorConstraint::evaluate(const stc_SQP::Vector& x,
                                           const stc_SQP::Vector& u,
                                           const stc_SQP::Vector& p,
                                           stc_SQP::Vector& g) const {
    (void)u;
    const int ng_val = ng();
    g.resize(ng_val);

    const int local_step = localStepIndex(p);
    (void)local_step;

    const double x_pos = x(0);
    const double y_pos = x(1);
    const double theta = x(2);

    const int n_circles = static_cast<int>(circle_local_positions_.size());
    const int n_constraints = std::min(constraints_per_step_, n_circles);

    for (int m = 0; m < n_constraints; ++m) {
        double dist = 0.0;
        Eigen::Vector2d grad = Eigen::Vector2d::Zero();
        Eigen::Matrix<double, 1, 5> a_row;
        double g_val = 0.0;

        const bool valid = computeCircleConstraint(x_pos, y_pos, theta,
                                                   circle_local_positions_[m],
                                                   dist, grad, a_row, g_val);

        g(m) = valid ? g_val : -1.0;
    }
}

void IterativeCorridorConstraint::jacobian(const stc_SQP::Vector& x,
                                           const stc_SQP::Vector& u,
                                           const stc_SQP::Vector& p,
                                           stc_SQP::Matrix& Cx,
                                           stc_SQP::Matrix& Cu) const {
    (void)u;
    const int ng_val = ng();
    Cx = stc_SQP::Matrix::Zero(ng_val, x.size());
    Cu.resize(ng_val, 2);
    Cu.setZero();

    const int local_step = localStepIndex(p);
    (void)local_step;

    const double x_pos = x(0);
    const double y_pos = x(1);
    const double theta = x(2);

    const int n_circles = static_cast<int>(circle_local_positions_.size());
    const int n_constraints = std::min(constraints_per_step_, n_circles);

    for (int m = 0; m < n_constraints; ++m) {
        double dist = 0.0;
        Eigen::Vector2d grad = Eigen::Vector2d::Zero();
        Eigen::Matrix<double, 1, 5> a_row;
        double g_val = 0.0;

        const bool valid = computeCircleConstraint(x_pos, y_pos, theta,
                                                   circle_local_positions_[m],
                                                   dist, grad, a_row, g_val);

        if (valid) {
            Cx.row(m).head(5) = -a_row;
        } else {
            Cx.row(m).setZero();
        }
    }
}

std::shared_ptr<stc_SQP::Constraint> IterativeCorridorConstraint::clone()
    const {
    return std::make_shared<IterativeCorridorConstraint>(
        esdf_map_, circle_local_positions_, circle_radius_, global_start_idx_,
        constraints_per_step_, hard_margin_);
}
}  // namespace apa_post_processor
