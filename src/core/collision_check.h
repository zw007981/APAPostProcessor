#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>

#include "../spatial/esdf_map.h"
#include "../util/constants.h"
#include "../util/path.h"
#include "../vehicle/vehicle_footprint_model.h"
#include "NMPC/vehicle_circle_geometry.h"

namespace apa_post_processor {
// 碰撞/有限性/角度工具函数：PostProcessor 两条路径的质量门与对应单元测试
// 共享同一实现，避免生产门禁与测试门禁各自维护导致公式漂移。

// 计算 Path 全部采样点的最大碰撞深度 (m)：对每个采样点遍历车身外圆集合，
// 取 outer_radius - d_esdf 的最大值（<= 0 表示无碰撞）。
inline double ComputeMaxCollisionDepth(
    const Path& path, const ESDFMap& esdf_map,
    const VehicleFootprintModel& footprint_model) {
    double max_collision = 0.0;
    const double outer_radius = footprint_model.getOuterRadius();
    // 外圆局部坐标只与 footprint 模型有关，循环外一次性提取
    const auto local_centers =
        vehicle_circle_geometry::ExtractLocalCircleCenters(footprint_model,
                                                           CircleType::OUTER);
    for (std::size_t m = 0; m < path.numManeuvers(); ++m) {
        for (const auto& pt : path.getManeuvers()[m].points) {
            const double cos_theta = std::cos(pt.theta);
            const double sin_theta = std::sin(pt.theta);
            for (const auto& local : local_centers) {
                const double wx =
                    pt.x + local.x() * cos_theta - local.y() * sin_theta;
                const double wy =
                    pt.y + local.x() * sin_theta + local.y() * cos_theta;
                max_collision = std::max(
                    max_collision, outer_radius - esdf_map.getDist(wx, wy));
            }
        }
    }
    return max_collision;
}

// 检查 Path 全部采样点及其已设置派生量均为有限值（无 NaN/Inf 奇异）。
inline bool IsPathFinite(const Path& path) {
    for (std::size_t m = 0; m < path.numManeuvers(); ++m) {
        for (const auto& pt : path.getManeuvers()[m].points) {
            if (!std::isfinite(pt.x) || !std::isfinite(pt.y) ||
                !std::isfinite(pt.theta)) {
                return false;
            }
            if (pt.hasV() && !std::isfinite(pt.getV())) {
                return false;
            }
            if (pt.hasDelta() && !std::isfinite(pt.getDelta())) {
                return false;
            }
            if (pt.hasA() && !std::isfinite(pt.getA())) {
                return false;
            }
            if (pt.hasDeltaDot() && !std::isfinite(pt.getDeltaDot())) {
                return false;
            }
        }
    }
    return true;
}

// 角度归一化到 [-π, π]（终点航向误差计算等场景共用）
inline double NormalizeAngle(double angle) {
    while (angle > PI) {
        angle -= 2.0 * PI;
    }
    while (angle < -PI) {
        angle += 2.0 * PI;
    }
    return angle;
}
}  // namespace apa_post_processor
