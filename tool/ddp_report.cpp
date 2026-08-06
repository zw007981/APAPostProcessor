// 四数据集 DDP 可视化 + 结果报告（一次性工具，重构回归用）
// 运行：./build/Release/apa_tune_ddp
//   或  ./build/Release/apa_ddp_report
#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>

#include "core/post_processor.h"
#include "spatial/esdf_map.h"
#include "spatial/grid_map.h"
#include "util/config_loader.h"
#include "util/data_loader.hpp"
#include "util/logger.h"
#include "util/trajectory.h"
#include "util/visualizer.hpp"
#include "vehicle/vehicle_footprint_model.h"
#include "vehicle/vehicle_params.h"

using namespace apa_post_processor;

namespace {

struct DatasetEntry {
    std::string name;
    std::string file;
};

struct RunResult {
    bool legal{false};
    int level{0};
    int maneuvers_in{0};
    int maneuvers_out{0};
    double length_in{0.0};
    double length_out{0.0};
    double term_pos{0.0};
    double term_head{0.0};
    double collision{0.0};
    double kappa_ratio{0.0};
    double max_omega{0.0};
    double time_ms{0.0};
    std::string msg;
};

RunResult RunDdpOnDataset(const DatasetEntry& dataset) {
    RunResult r;
    ::apa::post_processor::OptimizeRequest request;
    if (DataLoader::LoadProtoFromJsonFile(dataset.file, request) !=
        LoadResult::SUCCESS) {
        std::cerr << "FAILED to load " << dataset.file << std::endl;
        return r;
    }
    const auto vehicle_params = VehicleParams::FromProto(request.vehicle());
    const VehicleFootprintModel footprint_model(vehicle_params, 233, 2, 2);
    const auto grid_map = GridMap::FromProto(request.environment());
    const ESDFMap esdf_map(grid_map);
    const auto init_path = Path::FromProto(request.initial_path());
    const PostProcessor processor(vehicle_params, footprint_model, esdf_map);

    // 加载 DDP 配置
    DdpConfig ddp_config;
    if (!LoadBaseConfigOverrides("data/ddp_config.json", &ddp_config)) {
        std::cerr << "WARN: ddp_config.json base overrides load failed"
                  << std::endl;
    }
    nlohmann::json details;
    if (DataLoader::LoadJsonFile("data/ddp_config.json", details) ==
        LoadResult::SUCCESS) {
        LoadDdpConfigOverrides(details, &ddp_config);
    }

    const auto result = processor.optimizeDdp(init_path, ddp_config);
    r.maneuvers_in = static_cast<int>(init_path.numManeuvers());
    r.length_in = init_path.length();
    r.time_ms = result.total_time_ms;
    r.msg = result.message;
    r.level = result.output_level == OutputLevel::kFullSuccess    ? 2
              : result.output_level == OutputLevel::kDegraded    ? 1
                                                                 : 0;

    if (!result.success || result.optimized_trajectory.empty()) {
        r.maneuvers_out = r.maneuvers_in;
        r.length_out = r.length_in;
        return r;
    }

    const auto& goal_pt = init_path.back();
    const TrajectoryPoint goal(goal_pt.x, goal_pt.y, goal_pt.theta);
    const auto validation =
        result.optimized_trajectory.validate(goal, esdf_map, footprint_model);
    r.maneuvers_out = result.final_maneuvers;
    r.length_out = result.final_length;
    r.term_pos = validation.terminal_position_error;
    r.term_head = validation.terminal_heading_error_deg;
    r.collision = validation.max_intrusion_depth;

    // 曲率包络
    double max_kappa = 0.0;
    for (const auto& pt : result.optimized_trajectory) {
        if (pt.hasV() && pt.hasDelta() && std::abs(pt.getV()) >= 0.05) {
            max_kappa = std::max(
                max_kappa,
                std::abs(std::tan(pt.getDelta())) / vehicle_params.wheelbase);
        }
    }
    r.kappa_ratio = max_kappa / vehicle_params.max_kappa;

    // 转角速率
    for (const auto& pt : result.optimized_trajectory) {
        if (pt.hasDeltaDot()) {
            r.max_omega = std::max(r.max_omega, std::abs(pt.getDeltaDot()));
        }
    }

    r.legal = validation.all_passed && r.kappa_ratio <= 1.0247 &&
              r.max_omega <= 1.021 * vehicle_params.max_steer_rate;

    // 生成可视化图
    const Trajectory init_traj(init_path, vehicle_params);
    auto viz = Visualizer("DDP - " + dataset.name, -1.0, 2.33);
    if (!init_traj.empty()) {
        viz.plotTrajectory(init_traj, vehicle_params, &footprint_model,
                           &esdf_map, &grid_map, false, true,
                           {{"color", visualizer::Pen::RED},
                            {"label", "Original A* Path"}});
    }
    if (!result.optimized_trajectory.empty()) {
        viz.plotTrajectory(result.optimized_trajectory, vehicle_params,
                           &footprint_model, &esdf_map, &grid_map, true, false,
                           {{"color", visualizer::Pen::GREEN},
                            {"label", "DDP Optimized"}});
    }
    viz.save(std::string(PROJECT_ROOT_DIR) + "/fig/test");

    return r;
}

}  // namespace

int main() {
    Logger::SetLogDirectory(std::string(PROJECT_ROOT_DIR) + "/build/log");
    Logger::SetConsoleOutputEnabled(false);

    const std::vector<DatasetEntry> datasets = {
        {"data3", "data/mid_park/data3.json"},
        {"data1", "data/rub_park/data1.json"},
        {"data7", "data/rub_park/data7.json"},
        {"data6", "data/long_park/data6.json"},
    };

    std::cout << "\n========== DDP 四数据集回归报告 ("
              << __DATE__ << ") ==========\n\n";
    std::cout << "| 数据集 | level | maneuvers | length (m) | term_pos (m) | "
                 "term_head(°) | coll(m) | κ_ratio | |ω|max | legal | "
                 "time(ms) |\n";
    std::cout << "|---|---|---|---|---|---|---|---|---|---|---|\n";

    for (const auto& ds : datasets) {
        const auto r = RunDdpOnDataset(ds);
        std::cout << "| " << ds.name << " | " << r.level << " | "
                  << r.maneuvers_in << "→" << r.maneuvers_out << " | "
                  << std::fixed << std::setprecision(2) << r.length_in << "→"
                  << r.length_out << " | " << std::scientific
                  << std::setprecision(1) << r.term_pos << " | "
                  << std::fixed << std::setprecision(3) << r.term_head
                  << " | " << std::setprecision(4) << r.collision << " | "
                  << std::setprecision(3) << r.kappa_ratio << " | "
                  << std::setprecision(3) << r.max_omega << " | "
                  << (r.legal ? "✅" : "❌") << " | " << std::setprecision(0)
                  << r.time_ms << " |\n";
        std::cout << "  → " << r.msg << "\n";
    }

    std::cout << "\n图片已保存至 fig/test/ddp_*.png\n";
    return 0;
}
