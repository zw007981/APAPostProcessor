#include "core/NMPC/nmpc_solver.h"
#include "spatial/esdf_map.h"
#include "spatial/grid_map.h"
#include "util/data_loader.hpp"
#include "util/logger.h"
#include "util/position.h"
#include "util/visualizer.hpp"
#include "vehicle/vehicle_footprint_model.h"
#include "vehicle/vehicle_params.h"

using namespace apa_post_processor;

int main() {
    Logger::SetLogDirectory("../log");
    LOG_INFO("Hello, APA Post Processor!");

    try {
        auto config_json = nlohmann::json();
        const auto config_file_path = std::string("data/config.json");
        const auto load_result =
            DataLoader::LoadJsonFile(config_file_path, config_json);
        const auto data_file_path =
            config_json["data_file_path"].get<std::string>();
        auto optimize_request = ::apa::post_processor::OptimizeRequest();
        (void)DataLoader::LoadProtoFromJsonFile(data_file_path,
                                                optimize_request);
        LOG_FMT_INFO(
            "Load config from {}, vehicle params: {}", config_file_path,
            VehicleParams::FromProto(optimize_request.vehicle()).toString());
        auto vehicle_params =
            VehicleParams::FromProto(optimize_request.vehicle());
        auto grid_map = GridMap::FromProto(optimize_request.environment());
        auto init_path = Path::FromProto(optimize_request.initial_path());

        // 构建NMPC求解所需的ESDF地图与车辆圆形分解模型（outer_row_num取较小值，
        // 确保外圆数量不超过CircleFootprintEsdfConstraint::kMaxCircles，见仓库记忆）
        const ESDFMap esdf_map(grid_map);
        const VehicleFootprintModel footprint_model(
            vehicle_params, /*heading_sample_num=*/233, /*inner_row_num=*/2,
            /*outer_row_num=*/2);
        const NmpcSolver nmpc_solver(vehicle_params, footprint_model);
        const auto nmpc_result =
            nmpc_solver.optimizeWithPruning(init_path, esdf_map);

        auto optimize_response = ::apa::post_processor::OptimizeResponse();
        optimize_response.set_success(nmpc_result.converged);
        optimize_response.set_optimization_time_ms(nmpc_result.solve_time_ms);
        const auto optimized_path = NmpcSolver::ToPath(nmpc_result);
        if (!optimized_path.empty()) {
            optimized_path.toProto(optimize_response.mutable_optimized_path());
            optimized_path.toProto(optimize_response.mutable_maneuvers());
            optimize_response.set_message(
                nmpc_result.converged
                    ? "NMPC optimize converged"
                    : "NMPC reached max_iter without full convergence, "
                      "returning last iterate");
        } else {
            optimize_response.set_message(
                "NMPC solve failed on the first iteration, no usable "
                "trajectory produced, see log for details");
        }
        LOG_FMT_INFO(
            "NMPC optimize converged={}, time_ms={}, prune_iterations={}, "
            "maneuvers={}->{}, length={:.3f}->{:.3f}, message={}",
            nmpc_result.converged, nmpc_result.solve_time_ms,
            nmpc_result.prune_iterations, init_path.numManeuvers(),
            nmpc_result.segment_steps.size(), init_path.length(),
            optimized_path.empty() ? 0.0 : optimized_path.length(),
            optimize_response.message());

        auto visualizer = Visualizer("PostProcessor");
        if (!optimized_path.empty()) {
            visualizer
                .plotPath(optimized_path, true,
                          Visualizer::Style{{"color", visualizer::Pen::BLUE},
                                            {"linewidth", "2"},
                                            {"label", "OptimizedPath"}})
                .plotPathDetails(
                    optimized_path,
                    Visualizer::Style{{"color", visualizer::Pen::BLUE},
                                      {"label", "OptimizedPath"}});
        }
        visualizer
            .plotPath(init_path, true,
                      Visualizer::Style{{"color", visualizer::Pen::RED},
                                        {"linewidth", "2"},
                                        {"label", "InitPath"}})
            .plotVehicle(init_path.front(), vehicle_params, false,
                         Visualizer::Style{{"color", visualizer::Pen::YELLOW},
                                           {"linewidth", "2"},
                                           {"label", "Start"}})
            .plotVehicle(init_path.back(), vehicle_params, false,
                         Visualizer::Style{{"color", visualizer::Pen::GREEN},
                                           {"linewidth", "2"},
                                           {"label", "Goal"}})
            // .plotGridMap(grid_map, [&esdf_map](double x, double y) {
            //     return esdf_map.getDist(x, y);
            // })
            .plotGridMap(grid_map)
            .plotPathDetails(init_path,
                             Visualizer::Style{{"color", visualizer::Pen::RED},
                                               {"label", "InitPath"}});
        visualizer.save("../fig");
    } catch (const std::exception& e) {
        LOG_FMT_ERROR("Exception caught in main: {}!!!", e.what());
        return 1;
    }
    return 0;
}