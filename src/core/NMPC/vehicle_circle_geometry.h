#pragma once

#include <vector>

#include <Eigen/Core>

#include "../../vehicle/vehicle_footprint_model.h"

namespace apa_post_processor {
namespace vehicle_circle_geometry {
// 从VehicleFootprintModel提取车身坐标系下（后轴中心为原点、航向为0）的圆心局部坐标，
// 用于构造StcSQP的CircleFootprintEsdfConstraint。碰撞安全约束应使用CircleType::OUTER，
// 保证车身完全被圆覆盖；CircleType::INNER为内缩近似，不满足硬碰撞安全裕度语义。
// 注意：CircleFootprintEsdfConstraint::kMaxCircles限制了p向量能容纳的圆数量上限，
// 若footprint_model按较大的outer_row_num配置导致边界圆数量超出该上限，调用方需自行
// 减小outer_row_num或对返回结果做下采样。
std::vector<Eigen::Vector2d> ExtractLocalCircleCenters(
    const VehicleFootprintModel& footprint_model, CircleType type);
}  // namespace vehicle_circle_geometry
}  // namespace apa_post_processor
