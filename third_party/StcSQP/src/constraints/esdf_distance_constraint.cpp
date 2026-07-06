#include "esdf_distance_constraint.h"

#include <cmath>
#include <stdexcept>
#include <string>

#include "../core/vehicle_geometry.h"
#include "../util/constants.h"

namespace stc_SQP {
EsdfDistanceConstraint::EsdfDistanceConstraint(double safety_margin)
    : safety_margin_(safety_margin)
{
    if (!std::isfinite(safety_margin_) || safety_margin_ < 0.0) {
        throw std::invalid_argument("EsdfDistanceConstraint: safety_margin must be a non-negative finite number");
    }
}

int EsdfDistanceConstraint::ng() const { return kNumCorners; }

void EsdfDistanceConstraint::evaluate(const Vector& x, const Vector& u, const Vector& p,
    Vector& g) const
{
    validateInputDimensions(x, u);
    validateParameters(p);
    g.resize(kNumCorners);
    const double theta = x(2), c = std::cos(theta), s = std::sin(theta);
    const Eigen::Vector2d pos(x(0), x(1));
    const auto corners_local = cornerLocalPositions();
    for (int k = 0; k < kNumCorners; ++k) {
        const int base = kParamStart + k * kCornerStride;
        const double d0 = p(base + 0);
        const Eigen::Vector2d grad(p(base + 1), p(base + 2));
        const Eigen::Vector2d query_pos(p(base + 3), p(base + 4));
        const Eigen::Vector2d& local = corners_local[k];
        const Eigen::Vector2d corner_world(
            pos(0) + c * local(0) - s * local(1), pos(1) + s * local(0) + c * local(1));
        const double d_approx = d0 + grad.dot(corner_world - query_pos);
        g(k) = safety_margin_ - d_approx;
    }
}

void EsdfDistanceConstraint::jacobian(const Vector& x, const Vector& u, const Vector& p,
    Matrix& Cx, Matrix& Cu) const
{
    validateInputDimensions(x, u);
    validateParameters(p);
    Cx = Matrix::Zero(kNumCorners, x.size());
    Cu = Matrix::Zero(kNumCorners, u.size());
    const double theta = x(2), c = std::cos(theta), s = std::sin(theta);
    const auto corners_local = cornerLocalPositions();
    for (int k = 0; k < kNumCorners; ++k) {
        const int base = kParamStart + k * kCornerStride;
        const Eigen::Vector2d grad(p(base + 1), p(base + 2));
        const Eigen::Vector2d& local = corners_local[k];
        // 角点世界坐标 corner_world = pos + R(theta) * local，
        // d(corner_world)/dtheta = dR/dtheta * local
        const double dcx_dtheta = -s * local(0) - c * local(1);
        const double dcy_dtheta = c * local(0) - s * local(1);
        // g = margin - d0 - grad·(corner_world - query_pos)，对 x/y/theta 求导
        Cx(k, 0) = -grad(0);
        Cx(k, 1) = -grad(1);
        Cx(k, 2) = -(grad(0) * dcx_dtheta + grad(1) * dcy_dtheta);
    }
}

std::shared_ptr<Constraint> EsdfDistanceConstraint::clone() const
{
    return std::make_shared<EsdfDistanceConstraint>(safety_margin_);
}

void EsdfDistanceConstraint::packCornerSample(int corner_idx, double distance,
    const Eigen::Vector2d& gradient, const Eigen::Vector2d& query_pos, Vector& p)
{
    if (corner_idx < 0 || corner_idx >= kNumCorners) {
        throw std::invalid_argument(
            "EsdfDistanceConstraint::packCornerSample: corner_idx must be in [0, "
            + std::to_string(kNumCorners) + ")");
    }
    if (p.size() != STAGE_PARAM_DIM) {
        throw std::invalid_argument(
            "EsdfDistanceConstraint::packCornerSample: p dimension must be "
            + std::to_string(STAGE_PARAM_DIM));
    }
    if (!std::isfinite(distance) || !gradient.allFinite() || !query_pos.allFinite()) {
        throw std::invalid_argument(
            "EsdfDistanceConstraint::packCornerSample: distance/gradient/query_pos must all be finite");
    }
    const int base = kParamStart + corner_idx * kCornerStride;
    p(base + 0) = distance;
    p(base + 1) = gradient(0);
    p(base + 2) = gradient(1);
    p(base + 3) = query_pos(0);
    p(base + 4) = query_pos(1);
}

std::array<Eigen::Vector2d, EsdfDistanceConstraint::kNumCorners>
EsdfDistanceConstraint::cornerLocalPositions()
{
    // 与 autogen/generate_corridor.py 中的 CORNERS_LOCAL 保持一致：FL, FR, RL, RR
    return { Eigen::Vector2d(vehicle_geometry::kLf, vehicle_geometry::kWidth / 2.0),
        Eigen::Vector2d(vehicle_geometry::kLf, -vehicle_geometry::kWidth / 2.0),
        Eigen::Vector2d(-vehicle_geometry::kLr, vehicle_geometry::kWidth / 2.0),
        Eigen::Vector2d(-vehicle_geometry::kLr, -vehicle_geometry::kWidth / 2.0) };
}

void EsdfDistanceConstraint::validateInputDimensions(const Vector& x, const Vector& u) const
{
    (void)u;
    if (x.size() < kStateXYThetaDim) {
        throw std::invalid_argument(
            "EsdfDistanceConstraint: state x dimension must be at least " + std::to_string(kStateXYThetaDim));
    }
}

void EsdfDistanceConstraint::validateParameters(const Vector& p)
{
    if (p.size() != STAGE_PARAM_DIM) {
        throw std::invalid_argument(
            "EsdfDistanceConstraint: parameter p dimension must be " + std::to_string(STAGE_PARAM_DIM));
    }
}
} // namespace stc_SQP
