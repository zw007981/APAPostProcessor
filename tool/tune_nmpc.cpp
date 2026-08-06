// NMPC 四数据集调参驱动工具：对所有数据集执行
// `PostProcessor::optimize`，输出机动段数/路径长度/终点误差/碰撞深度/
// 运动学可行性（梯形配点残差）/耗时六类指标的机器可读行（[TUNE] 前缀），
// 供回归基线记录与后续重构对照。
// 运行示例：
//   ./build/Release/apa_tune_nmpc
// 输出约定：每行一个数据集评价；legal=1 表示 validate 三门（碰撞/终点/
// 运动学）全部通过且幅值硬限复检通过（κ 与 |ω| 不超过车辆真值上限的
// 1.02 包络——与 DDP/ALM 同口径）。
#include <cmath>
#include <functional>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include "core/post_processor.h"
#include "spatial/esdf_map.h"
#include "spatial/grid_map.h"
#include "util/config_loader.h"
#include "util/data_loader.hpp"
#include "util/trajectory.h"
#include "vehicle/vehicle_footprint_model.h"
#include "vehicle/vehicle_params.h"

using namespace apa_post_processor;

namespace {
// 数据集描述：显示名 + 文件路径
struct DatasetCase {
    std::string name;
    std::string file;
};

// 四份真实数据集（与 DDP/ALM tune 工具顺序一致，便于跨算法对比）
std::vector<DatasetCase> BuildDatasets() {
    return {{"data3_mid_park", "data/mid_park/data3.json"},
            {"data1_rub_park", "data/rub_park/data1.json"},
            {"data7_rub_park", "data/rub_park/data7.json"},
            {"data6_long_park", "data/long_park/data6.json"}};
}

// 行驶速度（|v|>=0.05 m/s）下的最大 |κ|=|tanδ|/L；无行驶速度点时返回 0
double ComputeMaxDrivingKappa(const Trajectory& traj, double wheelbase) {
    double max_kappa = 0.0;
    for (const auto& pt : traj) {
        if (!pt.hasV() || !pt.hasDelta() || std::abs(pt.getV()) < 0.05) {
            continue;
        }
        max_kappa =
            std::max(max_kappa, std::abs(std::tan(pt.getDelta())) / wheelbase);
    }
    return max_kappa;
}

// 在单数据集上运行 NMPC，返回是否合法（validate 三门 + 幅值硬限）
bool RunSingle(const DatasetCase& dataset) {
    ::apa::post_processor::OptimizeRequest request;
    if (DataLoader::LoadProtoFromJsonFile(dataset.file, request) !=
        LoadResult::SUCCESS) {
        std::cout << "[TUNE] algorithm=nmpc dataset=" << dataset.name
                  << " LOAD_FAILED" << std::endl;
        return false;
    }
    const auto vehicle_params = VehicleParams::FromProto(request.vehicle());
    // 外圆行数从生产配置 data/nmpc_config.json 加载，与生产同源
    NMPCConfig nmpc_config;
    LoadBaseConfigOverrides("data/nmpc_config.json", &nmpc_config);
    const VehicleFootprintModel footprint_model(
        vehicle_params, /*heading_sample_num=*/233, /*inner_row_num=*/2,
        nmpc_config.outer_row_num);
    const auto grid_map = GridMap::FromProto(request.environment());
    const ESDFMap esdf_map(grid_map);
    const auto init_path = Path::FromProto(request.initial_path());
    const PostProcessor processor(vehicle_params, footprint_model, esdf_map);
    const auto result = processor.optimize(init_path, nmpc_config);
    if (!result.success || result.optimized_trajectory.empty()) {
        std::cout << "[TUNE] algorithm=nmpc dataset=" << dataset.name
                  << " success=0 msg=\"" << result.message << "\""
                  << std::endl;
        return false;
    }
    const auto& goal_pt = init_path.back();
    const TrajectoryPoint goal(goal_pt.x, goal_pt.y, goal_pt.theta);
    const auto validation =
        result.optimized_trajectory.validate(goal, esdf_map, footprint_model);
    // 行驶速度最大 |κ| 与相对车辆物理上限的比值；max|ω| 全轨迹包络
    const double max_kappa =
        ComputeMaxDrivingKappa(result.optimized_trajectory, vehicle_params.wheelbase);
    const double kappa_ratio = max_kappa / vehicle_params.max_kappa;
    double max_omega = 0.0;
    for (const auto& pt : result.optimized_trajectory) {
        if (pt.hasDeltaDot()) {
            max_omega = std::max(max_omega, std::abs(pt.getDeltaDot()));
        }
    }
    // 合法性口径与 DDP/ALM 对齐（三门 + 幅值硬限 1.02 包络）
    const double omega_limit = 1.021 * vehicle_params.max_steer_rate;
    const bool legal = validation.all_passed && kappa_ratio <= 1.0247 &&
                       max_omega <= omega_limit;
    std::cout << "[TUNE] algorithm=nmpc dataset=" << dataset.name
              << " success=1"
              << " maneuvers=" << init_path.numManeuvers() << "->"
              << result.final_maneuvers << " length=" << init_path.length()
              << "->" << result.final_length
              << " term_pos=" << validation.terminal_position_error
              << " term_head=" << validation.terminal_heading_error_deg
              << " collision=" << validation.max_intrusion_depth
              << " kin_pos=" << validation.max_kinematic_position_residual
              << " kin_head=" << validation.max_kinematic_heading_residual_deg
              << " kin_vel=" << validation.max_kinematic_velocity_residual
              << " kin_steer=" << validation.max_kinematic_steer_residual
              << " max_kappa=" << max_kappa << " kappa_ratio=" << kappa_ratio
              << " max_omega=" << max_omega
              << " legal=" << (legal ? 1 : 0)
              << " time_ms=" << result.total_time_ms << " msg=\""
              << result.message << "\"" << std::endl;
    return legal;
}
}  // namespace

int main() {
    const auto datasets = BuildDatasets();
    int illegal_count = 0;
    for (const auto& dataset : datasets) {
        if (!RunSingle(dataset)) {
            ++illegal_count;
        }
    }
    std::cout << "[TUNE-SUMMARY] algorithm=nmpc datasets=" << datasets.size()
              << " illegal_evals=" << illegal_count << std::endl;
    return illegal_count == 0 ? 0 : 1;
}
