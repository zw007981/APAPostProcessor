#include "circle_footprint_esdf_constraint.h"

#include <cmath>
#include <stdexcept>
#include <string>

#include "../util/constants.h"

namespace stc_SQP {
CircleFootprintEsdfConstraint::CircleFootprintEsdfConstraint(
    std::vector<Eigen::Vector2d> circle_local_positions, double circle_radius,
    double safety_margin)
    : circle_local_positions_(std::move(circle_local_positions))
    , circle_radius_(circle_radius)
    , safety_margin_(safety_margin)
{
    if (circle_local_positions_.empty()
        || static_cast<int>(circle_local_positions_.size()) > kMaxCircles) {
        throw std::invalid_argument(
            "CircleFootprintEsdfConstraint: circle_local_positions size must be in [1, "
            + std::to_string(kMaxCircles) + "]");
    }
    for (const auto& local : circle_local_positions_) {
        if (!local.allFinite()) {
            throw std::invalid_argument(
                "CircleFootprintEsdfConstraint: circle_local_positions must all be finite");
        }
    }
    if (!std::isfinite(circle_radius_) || circle_radius_ <= 0.0) {
        throw std::invalid_argument(
            "CircleFootprintEsdfConstraint: circle_radius must be a positive finite number");
    }
    if (!std::isfinite(safety_margin_) || safety_margin_ < 0.0) {
        throw std::invalid_argument(
            "CircleFootprintEsdfConstraint: safety_margin must be a non-negative finite number");
    }
}

int CircleFootprintEsdfConstraint::ng() const
{
    return static_cast<int>(circle_local_positions_.size());
}

void CircleFootprintEsdfConstraint::evaluate(const Vector& x, const Vector& u, const Vector& p,
    Vector& g) const
{
    validateInputDimensions(x, u);
    validateParameters(p);
    const int circle_num = ng();
    g.resize(circle_num);
    const double theta = x(2), c = std::cos(theta), s = std::sin(theta);
    const Eigen::Vector2d pos(x(0), x(1));
    // 碰撞裕度要求：圆心到最近障碍物的距离至少为 circle_radius_ + safety_margin_
    const double required_clearance = circle_radius_ + safety_margin_;
    for (int k = 0; k < circle_num; ++k) {
        const int base = kParamStart + k * kCircleStride;
        const double d0 = p(base + 0);
        const Eigen::Vector2d grad(p(base + 1), p(base + 2));
        const Eigen::Vector2d query_pos(p(base + 3), p(base + 4));
        const Eigen::Vector2d& local = circle_local_positions_[k];
        const Eigen::Vector2d circle_world(
            pos(0) + c * local(0) - s * local(1), pos(1) + s * local(0) + c * local(1));
        const double d_approx = d0 + grad.dot(circle_world - query_pos);
        g(k) = required_clearance - d_approx;
    }
}

void CircleFootprintEsdfConstraint::jacobian(const Vector& x, const Vector& u, const Vector& p,
    Matrix& Cx, Matrix& Cu) const
{
    validateInputDimensions(x, u);
    validateParameters(p);
    const int circle_num = ng();
    Cx = Matrix::Zero(circle_num, x.size());
    Cu = Matrix::Zero(circle_num, u.size());
    const double theta = x(2), c = std::cos(theta), s = std::sin(theta);
    for (int k = 0; k < circle_num; ++k) {
        const int base = kParamStart + k * kCircleStride;
        const Eigen::Vector2d grad(p(base + 1), p(base + 2));
        const Eigen::Vector2d& local = circle_local_positions_[k];
        // 圆心世界坐标 circle_world = pos + R(theta) * local，
        // d(circle_world)/dtheta = dR/dtheta * local
        const double dcx_dtheta = -s * local(0) - c * local(1);
        const double dcy_dtheta = c * local(0) - s * local(1);
        // g = required_clearance - d0 - grad·(circle_world - query_pos)，对x/y/theta求导
        Cx(k, 0) = -grad(0);
        Cx(k, 1) = -grad(1);
        Cx(k, 2) = -(grad(0) * dcx_dtheta + grad(1) * dcy_dtheta);
    }
}

std::shared_ptr<Constraint> CircleFootprintEsdfConstraint::clone() const
{
    return std::make_shared<CircleFootprintEsdfConstraint>(
        circle_local_positions_, circle_radius_, safety_margin_);
}

void CircleFootprintEsdfConstraint::packCircleSample(int circle_idx, double distance,
    const Eigen::Vector2d& gradient, const Eigen::Vector2d& query_pos, Vector& p)
{
    if (circle_idx < 0 || circle_idx >= kMaxCircles) {
        throw std::invalid_argument(
            "CircleFootprintEsdfConstraint::packCircleSample: circle_idx must be in [0, "
            + std::to_string(kMaxCircles) + ")");
    }
    if (p.size() != STAGE_PARAM_DIM) {
        throw std::invalid_argument(
            "CircleFootprintEsdfConstraint::packCircleSample: p dimension must be "
            + std::to_string(STAGE_PARAM_DIM));
    }
    if (!std::isfinite(distance) || !gradient.allFinite() || !query_pos.allFinite()) {
        throw std::invalid_argument(
            "CircleFootprintEsdfConstraint::packCircleSample: distance/gradient/query_pos must "
            "all be finite");
    }
    const int base = kParamStart + circle_idx * kCircleStride;
    p(base + 0) = distance;
    p(base + 1) = gradient(0);
    p(base + 2) = gradient(1);
    p(base + 3) = query_pos(0);
    p(base + 4) = query_pos(1);
}

void CircleFootprintEsdfConstraint::validateInputDimensions(const Vector& x, const Vector& u) const
{
    (void)u;
    if (x.size() < kStateXYThetaDim) {
        throw std::invalid_argument(
            "CircleFootprintEsdfConstraint: state x dimension must be at least "
            + std::to_string(kStateXYThetaDim));
    }
}

void CircleFootprintEsdfConstraint::validateParameters(const Vector& p)
{
    if (p.size() != STAGE_PARAM_DIM) {
        throw std::invalid_argument(
            "CircleFootprintEsdfConstraint: parameter p dimension must be "
            + std::to_string(STAGE_PARAM_DIM));
    }
}
} // namespace stc_SQP
