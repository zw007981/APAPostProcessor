#pragma once

#include <Eigen/Core>

#include <cstddef>
#include <vector>

namespace stc_SQP {
// 2D 凸多边形间的 GJK 轮廓距离。
// 输入为按顺序排列的凸多边形顶点（逆时针或顺时针均可），函数返回两多边形之间的最小距离。
// 若两多边形相交（含边界接触），返回 0。
// 本实现基于 Minkowski 差 + 凸包，适用于顶点数较少的凸形状（如车辆矩形 4 顶点对障碍物线段 2 顶点）。
double gjkConvexDistance2d(
    const std::vector<Eigen::Vector2d>& polygon_a,
    const std::vector<Eigen::Vector2d>& polygon_b);
// 凸多边形到有限线段的距离：直接实现，避免调用方构造 2 点 vector
double gjkConvexDistance2d(const std::vector<Eigen::Vector2d>& polygon_a,
    const Eigen::Vector2d& seg_start, const Eigen::Vector2d& seg_end);
// 凸多边形（以原始指针 + 顶点数传入）到有限线段的距离。
// 供固定顶点数（如车辆四角点）的调用方使用，避免 std::vector 堆分配。
double gjkConvexDistance2d(const Eigen::Vector2d* polygon_a, std::size_t num_points,
    const Eigen::Vector2d& seg_start, const Eigen::Vector2d& seg_end);
} // namespace stc_SQP
