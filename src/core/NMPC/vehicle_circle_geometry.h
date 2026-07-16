#pragma once

#include <Eigen/Core>
#include <vector>

#include "../../vehicle/vehicle_footprint_model.h"

namespace apa_post_processor {
namespace vehicle_circle_geometry {
// 从 VehicleFootprintModel 提取车身坐标系下的圆心局部坐标。碰撞安全使用 CircleType::OUTER。
std::vector<Eigen::Vector2d> ExtractLocalCircleCenters(
    const VehicleFootprintModel& footprint_model, CircleType type);
}  // namespace vehicle_circle_geometry
}  // namespace apa_post_processor
