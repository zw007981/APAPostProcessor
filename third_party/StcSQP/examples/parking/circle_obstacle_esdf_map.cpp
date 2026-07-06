#include "circle_obstacle_esdf_map.h"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace stc_SQP {
void CircleObstacleEsdfMap::addObstacle(const Eigen::Vector2d& center, double radius)
{
    if (!center.allFinite()) {
        throw std::invalid_argument("CircleObstacleEsdfMap::addObstacle: center must be finite");
    }
    if (!std::isfinite(radius) || radius <= 0.0) {
        throw std::invalid_argument("CircleObstacleEsdfMap::addObstacle: radius must be a finite positive number");
    }
    obstacles_.push_back({ center, radius });
}

EsdfSample CircleObstacleEsdfMap::queryDistance(const Eigen::Vector2d& point) const
{
    if (obstacles_.empty()) {
        return { std::numeric_limits<double>::max(), Eigen::Vector2d::Zero() };
    }
    EsdfSample best { std::numeric_limits<double>::max(), Eigen::Vector2d::Zero() };
    for (const auto& obstacle : obstacles_) {
        const Eigen::Vector2d diff = point - obstacle.center;
        const double center_dist = diff.norm();
        const double distance = center_dist - obstacle.radius;
        if (distance < best.distance) {
            // 梯度为距离场沿远离圆心方向的单位向量；point 与圆心重合时退化为零梯度，
            // 此时距离本身也是有限负数（车辆中心恰好落在障碍物内部），线性化仍安全。
            const Eigen::Vector2d gradient = center_dist > kMinCenterDistance
                ? Eigen::Vector2d(diff / center_dist)
                : Eigen::Vector2d::Zero();
            best = { distance, gradient };
        }
    }
    return best;
}
} // namespace stc_SQP
