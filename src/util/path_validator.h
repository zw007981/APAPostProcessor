#pragma once

#include <string>
#include <vector>

#include "../core/NMPC/nmpc_solver.h"
#include "../spatial/esdf_map.h"
#include "../vehicle/vehicle_footprint_model.h"
#include "path.h"

namespace apa_post_processor {

// 路径合法性验证配置
struct PathValidationConfig {
    // 最大碰撞深度 (m)
    double max_collision_depth = 0.02;
    // 终点位置误差上限 (m)
    double max_terminal_position_error = 0.05;
    // 终点航向误差上限 (°)
    double max_terminal_heading_error_deg = 3.0;
};

// 路径合法性验证结果
struct PathValidationResult {
    bool collision_safe = false;
    bool terminal_position_ok = false;
    bool terminal_heading_ok = false;
    // 三项全部通过
    bool all_passed = false;

    // 最大碰撞深度 (m)
    double max_intrusion_depth = 0.0;
    // 终点位置误差 (m)
    double terminal_position_error = 0.0;
    // 终点航向误差 (°)
    double terminal_heading_error_deg = 0.0;
    // 各门失败原因（空字符串表示通过）
    std::string collision_detail;
    std::string terminal_position_detail;
    std::string terminal_heading_detail;
};

// 验证路径合法性：碰撞安全 + 终点收敛（运动学可行性由SQP保证，不重复检查）
PathValidationResult ValidatePath(const Path& path, const TrajectoryPoint& goal,
                                  const ESDFMap& esdf_map,
                                  const VehicleFootprintModel& footprint_model,
                                  const PathValidationConfig& config = {});

// 将验证结果格式化为单行可读字符串
std::string FormatValidationResult(const PathValidationResult& result);

}  // namespace apa_post_processor
