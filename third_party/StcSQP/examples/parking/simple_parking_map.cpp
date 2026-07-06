#include "simple_parking_map.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

#include "util/geometry.h"

namespace stc_SQP {
void SimpleParkingMap::addWall(
    const HalfSpace& half_space, const Eigen::Vector2d& start, const Eigen::Vector2d& end)
{
    if (!isValidHalfSpace(half_space)) {
        throw std::invalid_argument(
            "SimpleParkingMap: invalid half_space: normal must be 2D, finite and non-zero, intercept must be finite");
    }
    if (!start.allFinite() || !end.allFinite()) {
        throw std::invalid_argument("SimpleParkingMap: wall endpoints contain non-finite values");
    }
    if ((end - start).squaredNorm() == 0.0) {
        throw std::invalid_argument("SimpleParkingMap: wall endpoints coincide, degenerate line segment");
    }
    if (std::abs(half_space.normal.dot(start) - half_space.intercept) > kBoundaryTolerance
        || std::abs(half_space.normal.dot(end) - half_space.intercept) > kBoundaryTolerance) {
        throw std::invalid_argument(
            "SimpleParkingMap: wall endpoints must lie on the half_space boundary, otherwise GJK distance and constraint semantics are inconsistent");
    }
    walls_.push_back({ half_space, start, end });
}

std::vector<HalfSpace> SimpleParkingMap::queryHalfSpaces(
    const Vector& pose, double selection_radius, int top_k) const
{
    if (pose.size() < 3) {
        throw std::invalid_argument("SimpleParkingMap: pose dimension must be >= 3");
    }
    if (!pose.head(3).allFinite()) {
        throw std::invalid_argument("SimpleParkingMap: pose first three dimensions contain non-finite values");
    }
    if (!std::isfinite(selection_radius) || selection_radius < 0.0) {
        throw std::invalid_argument("SimpleParkingMap: selection_radius must be a non-negative finite number");
    }
    if (top_k < 0) {
        throw std::invalid_argument("SimpleParkingMap: top_k must not be negative");
    }
    const std::array<Eigen::Vector2d, 4> vehicle_corners = computeVehicleCorners(pose);
    std::vector<std::pair<std::size_t, double>> candidates;
    candidates.reserve(walls_.size());
    for (std::size_t idx = 0; idx < walls_.size(); ++idx) {
        const double dist = gjkConvexDistance2d(
            vehicle_corners.data(), vehicle_corners.size(), walls_[idx].start, walls_[idx].end);
        if (dist <= selection_radius) {
            candidates.emplace_back(idx, dist);
        }
    }
    const int n = std::min(top_k, static_cast<int>(candidates.size()));
    if (n > 0) {
        if (n < static_cast<int>(candidates.size())) {
            std::nth_element(candidates.begin(), candidates.begin() + n, candidates.end(),
                [](const auto& lhs, const auto& rhs) { return lhs.second < rhs.second; });
        }
        std::sort(candidates.begin(), candidates.begin() + n,
            [](const auto& lhs, const auto& rhs) { return lhs.second < rhs.second; });
    }
    std::vector<HalfSpace> result;
    result.reserve(n);
    for (int i = 0; i < n; ++i) {
        result.push_back(walls_[candidates[i].first].half_space);
    }
    return result;
}

std::array<Eigen::Vector2d, 4> SimpleParkingMap::computeVehicleCorners(const Vector& pose) const
{
    const double x = pose(0);
    const double y = pose(1);
    const double theta = pose(2);
    const double c = std::cos(theta), s = std::sin(theta);
    const double half_w = kVehicleWidth / 2.0;
    // 车身坐标系下局部角点：FL, FR, RL, RR
    const Eigen::Vector2d local[4] = {
        Eigen::Vector2d(kVehicleLf, half_w),
        Eigen::Vector2d(kVehicleLf, -half_w),
        Eigen::Vector2d(-kVehicleLr, half_w),
        Eigen::Vector2d(-kVehicleLr, -half_w),
    };
    std::array<Eigen::Vector2d, 4> corners;
    for (int i = 0; i < 4; ++i) {
        const Eigen::Vector2d& p = local[i];
        corners[i].x() = x + c * p.x() - s * p.y();
        corners[i].y() = y + s * p.x() + c * p.y();
    }
    return corners;
}
} // namespace stc_SQP
