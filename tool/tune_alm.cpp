// ALM 四数据集调参驱动工具：对给定的 AlmConfig 变体列表，逐变体 × 逐数据集
// 执行 `PostProcessor::optimizeAlm`，输出机动段数/路径长度/终点误差/碰撞深度/
// 运动学可行性（梯形配点残差）/耗时六类指标的机器可读行（[TUNE] 前缀），
// 供调参过程记录与回归判断。运行示例：
//   ./build/Release/apa_tune_alm
// 输出约定：每行一个 变体×数据集 评价；legal=1 表示 Trajectory::validate()
// 三门（碰撞/终点/运动学）全部通过且幅值硬限复检通过（κ 与 |ω| 不超过
// 车辆真值上限的 1.02 包络——与 DDP 侧 tune_ddp/生产校验门同一口径，
// 两条链路的「合法」定义由此统一）
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

// 调参变体：显示名 + 完整 AlmConfig（以默认值为底、按需覆盖）
struct TuneVariant {
    std::string name;
    AlmConfig config;
};

// 四份真实数据集（顺序固定，便于对比多轮输出）
std::vector<DatasetCase> BuildDatasets() {
    return {{"data3_mid_park", "data/mid_park/data3.json"},
            {"data1_rub_park", "data/rub_park/data1.json"},
            {"data7_rub_park", "data/rub_park/data7.json"},
            {"data6_long_park", "data/long_park/data6.json"}};
}

// 本批次扫描的变体列表（baseline 恒为第一个，作为全部对比的基准）。
// 当前发货状态仅保留最终默认参数的基线验证；历史扫描批次（jerk_s/
// gear_cusp/eps_time/melt_arc 共 25 组 + 段数压缩批次 1~7 的变体定义）
// 见 docs/ALM.md 第四章调参标定小节收录的完整记录文档，可按需恢复后
// 复跑。
// baseline 的通用 Config 基类字段（如 outer_row_num）从生产配置
// data/alm_config.json 加载，保证调参评价与生产环境同一碰撞模型
std::vector<TuneVariant> BuildVariants() {
    std::vector<TuneVariant> variants;
    AlmConfig baseline;
    if (!LoadBaseConfigOverrides("data/alm_config.json", &baseline)) {
        std::cout << "[TUNE] baseline: data/alm_config.json load failed, "
                     "use built-in defaults"
                  << std::endl;
    }
    variants.emplace_back(TuneVariant{"baseline", baseline});
    return variants;
}

// 行驶速度（|v|>=0.05 m/s，与运动学门低速跳过阈值一致）下的最大
// |κ|=|tanδ|/L；无行驶速度点时返回 0
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

// 在单数据集上评价一个变体，返回是否合法（validate 三门全过）
bool RunSingle(const TuneVariant& variant, const DatasetCase& dataset) {
    ::apa::post_processor::OptimizeRequest request;
    if (DataLoader::LoadProtoFromJsonFile(dataset.file, request) !=
        LoadResult::SUCCESS) {
        std::cout << "[TUNE] variant=" << variant.name
                  << " dataset=" << dataset.name << " LOAD_FAILED" << std::endl;
        return false;
    }
    const auto vehicle_params = VehicleParams::FromProto(request.vehicle());
    // 外圆行数与变体配置同源（Config::outer_row_num，由
    // data/alm_config.json 覆盖），保证调参与生产同一碰撞模型
    const VehicleFootprintModel footprint_model(
        vehicle_params, /*heading_sample_num=*/233, /*inner_row_num=*/2,
        variant.config.outer_row_num);
    const auto grid_map = GridMap::FromProto(request.environment());
    const ESDFMap esdf_map(grid_map);
    const auto init_path = Path::FromProto(request.initial_path());
    const PostProcessor processor(vehicle_params, footprint_model, esdf_map);
    const auto result = processor.optimizeAlm(init_path, variant.config);
    if (!result.success || result.optimized_trajectory.empty()) {
        std::cout << "[TUNE] variant=" << variant.name
                  << " dataset=" << dataset.name << " success=0 msg=\""
                  << result.message << "\"" << std::endl;
        return false;
    }
    const auto& goal_pt = init_path.back();
    const TrajectoryPoint goal(goal_pt.x, goal_pt.y, goal_pt.theta);
    const auto validation =
        result.optimized_trajectory.validate(goal, esdf_map, footprint_model);
    // 行驶速度最大 |κ| 与相对车辆物理上限的比值；max|ω| 全轨迹包络
    // （执行器在驻留转向中同样工作，与 tune_ddp 同一量测口径）
    const double max_kappa =
        ComputeMaxDrivingKappa(result.optimized_trajectory, vehicle_params.wheelbase);
    const double kappa_ratio = max_kappa / vehicle_params.max_kappa;
    double max_omega = 0.0;
    for (const auto& pt : result.optimized_trajectory) {
        if (pt.hasDeltaDot()) {
            max_omega = std::max(max_omega, std::abs(pt.getDeltaDot()));
        }
    }
    // 合法性口径与 DDP 链路统一（spec Q4：两条链路的「合法」必须同
    // 定义才有对比意义）：validate 三门 + 幅值硬限复检（δ 相对 2.1% ≈
    // κ 相对 2.47%、ω 相对 2.1%——AL 平衡包络 + 车辆余量的一致标定，
    // 与 tune_ddp/生产校验门同源）
    const double omega_limit = 1.021 * vehicle_params.max_steer_rate;
    // ω 超限取证：超限点数与最不利点的上下文（速度/转角/轨迹位置），
    // 区分「换挡尖点伪影」与「真实可行性缺陷」
    int omega_over_points = 0;
    double omega_worst_v = 0.0;
    double omega_worst_delta = 0.0;
    std::size_t omega_worst_index = 0;
    for (std::size_t i = 0; i < result.optimized_trajectory.size(); ++i) {
        const auto& pt = result.optimized_trajectory[i];
        if (!pt.hasDeltaDot() || std::abs(pt.getDeltaDot()) <= omega_limit) {
            continue;
        }
        ++omega_over_points;
        if (std::abs(pt.getDeltaDot()) >= max_omega) {
            omega_worst_index = i;
            omega_worst_v = pt.hasV() ? pt.getV() : 0.0;
            omega_worst_delta = pt.hasDelta() ? pt.getDelta() : 0.0;
        }
    }
    const bool legal = validation.all_passed && kappa_ratio <= 1.0247 &&
                       max_omega <= omega_limit;
    std::cout << "[TUNE] variant=" << variant.name
              << " dataset=" << dataset.name << " success=1"
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
              << " omega_over=" << omega_over_points
              << " omega_worst=(idx=" << omega_worst_index
              << ",v=" << omega_worst_v << ",delta=" << omega_worst_delta
              << ",size=" << result.optimized_trajectory.size() << ")"
              << " legal=" << (legal ? 1 : 0)
              << " time_ms=" << result.total_time_ms << " msg=\""
              << result.message << "\"" << std::endl;
    return legal;
}
}  // namespace

int main() {
    const auto datasets = BuildDatasets();
    const auto variants = BuildVariants();
    int illegal_count = 0;
    for (const auto& variant : variants) {
        for (const auto& dataset : datasets) {
            if (!RunSingle(variant, dataset)) {
                ++illegal_count;
            }
        }
    }
    std::cout << "[TUNE-SUMMARY] variants=" << variants.size()
              << " illegal_evals=" << illegal_count << std::endl;
    return illegal_count == 0 ? 0 : 1;
}
