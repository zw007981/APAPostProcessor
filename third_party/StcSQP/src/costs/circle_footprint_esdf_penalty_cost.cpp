#include "circle_footprint_esdf_penalty_cost.h"

#include <cmath>
#include <stdexcept>

namespace stc_SQP {
CircleFootprintEsdfPenaltyCost::CircleFootprintEsdfPenaltyCost(
    std::vector<Eigen::Vector2d> circle_local_positions, double circle_radius,
    double safety_margin, const EsdfMapInterface& map, double penalty_weight)
    : circle_local_positions_(std::move(circle_local_positions))
    , circle_radius_(circle_radius)
    , safety_margin_(safety_margin)
    , map_(map)
    , penalty_weight_(penalty_weight)
{
    if (circle_local_positions_.empty()) {
        throw std::invalid_argument(
            "CircleFootprintEsdfPenaltyCost: circle_local_positions cannot be empty");
    }
    for (const auto& local : circle_local_positions_) {
        if (!local.allFinite()) {
            throw std::invalid_argument(
                "CircleFootprintEsdfPenaltyCost: circle_local_positions must all be finite");
        }
    }
    if (!std::isfinite(circle_radius_) || circle_radius_ <= 0.0) {
        throw std::invalid_argument(
            "CircleFootprintEsdfPenaltyCost: circle_radius must be a positive finite number");
    }
    if (!std::isfinite(safety_margin_) || safety_margin_ < 0.0) {
        throw std::invalid_argument(
            "CircleFootprintEsdfPenaltyCost: safety_margin must be a non-negative finite number");
    }
    if (!std::isfinite(penalty_weight_) || penalty_weight_ < 0.0) {
        throw std::invalid_argument(
            "CircleFootprintEsdfPenaltyCost: penalty_weight must be a non-negative finite number");
    }
}

void CircleFootprintEsdfPenaltyCost::computeViolations(const Vector& x,
    std::vector<double>& violations, std::vector<Eigen::Vector3d>& violation_grads) const
{
    const double theta = x(2), c = std::cos(theta), s = std::sin(theta);
    const Eigen::Vector2d pos(x(0), x(1));
    const double required_clearance = circle_radius_ + safety_margin_;
    const std::size_t circle_num = circle_local_positions_.size();
    violations.assign(circle_num, 0.0);
    violation_grads.assign(circle_num, Eigen::Vector3d::Zero());
    for (std::size_t k = 0; k < circle_num; ++k) {
        const Eigen::Vector2d& local = circle_local_positions_[k];
        const Eigen::Vector2d circle_world(
            pos(0) + c * local(0) - s * local(1), pos(1) + s * local(0) + c * local(1));
        const EsdfSample sample = map_.queryDistance(circle_world);
        const double violation = required_clearance - sample.distance;
        if (violation <= 0.0) {
            continue;
        }
        violations[k] = violation;
        // 圆心世界坐标 circle_world = pos + R(theta) * local，
        // d(circle_world)/dtheta = dR/dtheta * local
        const double dcx_dtheta = -s * local(0) - c * local(1);
        const double dcy_dtheta = c * local(0) - s * local(1);
        // violation = required_clearance - distance，对(x, y, theta)求导即为
        // -1 乘以ESDF梯度经圆心位置的链式法则结果
        violation_grads[k] = Eigen::Vector3d(-sample.gradient(0), -sample.gradient(1),
            -(sample.gradient(0) * dcx_dtheta + sample.gradient(1) * dcy_dtheta));
    }
}

void CircleFootprintEsdfPenaltyCost::evaluate(const Vector& x, const Vector& u, double& cost) const
{
    validateInputDimensions(x, u);
    std::vector<double> violations;
    std::vector<Eigen::Vector3d> violation_grads;
    computeViolations(x, violations, violation_grads);
    cost = 0.0;
    for (double violation : violations) {
        cost += 0.5 * penalty_weight_ * violation * violation;
    }
}

void CircleFootprintEsdfPenaltyCost::gradient(const Vector& x, const Vector& u, Vector& q,
    Vector& r) const
{
    validateInputDimensions(x, u);
    std::vector<double> violations;
    std::vector<Eigen::Vector3d> violation_grads;
    computeViolations(x, violations, violation_grads);
    q = Vector::Zero(x.size());
    for (std::size_t k = 0; k < violations.size(); ++k) {
        q.head<3>() += penalty_weight_ * violations[k] * violation_grads[k];
    }
    r = Vector::Zero(u.size());
}

void CircleFootprintEsdfPenaltyCost::hessian(const Vector& x, const Vector& u, Matrix& Q,
    Matrix& R, Matrix& S) const
{
    validateInputDimensions(x, u);
    std::vector<double> violations;
    std::vector<Eigen::Vector3d> violation_grads;
    computeViolations(x, violations, violation_grads);
    Q = Matrix::Zero(x.size(), x.size());
    for (std::size_t k = 0; k < violations.size(); ++k) {
        if (violations[k] <= 0.0) {
            continue;
        }
        // Gauss-Newton近似：忽略ESDF场自身的二阶曲率，只保留梯度外积项，保证半正定
        Q.topLeftCorner<3, 3>() +=
            penalty_weight_ * (violation_grads[k] * violation_grads[k].transpose());
    }
    R = Matrix::Zero(u.size(), u.size());
    S = Matrix::Zero(u.size(), x.size());
}

void CircleFootprintEsdfPenaltyCost::validateInputDimensions(const Vector& x, const Vector& u) const
{
    (void)u;
    if (x.size() < 3) {
        throw std::invalid_argument(
            "CircleFootprintEsdfPenaltyCost: state x dimension must be at least 3");
    }
}
} // namespace stc_SQP
