#include "core/NMPC/nmpc_solver.h"
#include "core/post_processor.h"
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

        // 构建 ESDF 地图与车辆圆形分解模型
        const ESDFMap esdf_map(grid_map);
        const VehicleFootprintModel footprint_model(
            vehicle_params, /*heading_sample_num=*/233, /*inner_row_num=*/2,
            /*outer_row_num=*/2);

        // PostProcessor 完整链路
        PreprocessingPipelineConfig pipeline_config;
        NmpcSolverConfig nmpc_config;
        AdaptiveRetryConfig retry_config;
        const PostProcessor post_processor(vehicle_params, footprint_model,
                                           esdf_map);
        const auto post_result = post_processor.optimize(
            init_path, pipeline_config, nmpc_config, retry_config);

        LOG_FMT_INFO(
            "PostProcessor result: success={}, maneuvers={}, length={:.3f}, "
            "time_ms={:.1f}, used_retry={}, message={}",
            post_result.success, post_result.final_maneuvers,
            post_result.final_length, post_result.total_time_ms,
            post_result.used_retry, post_result.message);

        const auto& optimized_path = post_result.optimized_path;

        auto visualizer = Visualizer("PostProcessor", -1.0, 2.33);
        // 预处理轨迹：不绘制扫过轮廓。
        if (!post_result.preprocessed_traj.empty()) {
            visualizer.plotTrajectory(
                post_result.preprocessed_traj, vehicle_params, &footprint_model,
                &esdf_map, &grid_map,
                /*draw_swept_area=*/false,
                {{"color", visualizer::Pen::RED}, {"label", "Preprocessed"}});
        }
        // NMPC 轨迹：绘制扫过轮廓。
        if (!post_result.nmpc_traj.empty()) {
            visualizer.plotTrajectory(post_result.nmpc_traj, vehicle_params,
                                      &footprint_model, &esdf_map, &grid_map,
                                      /*draw_swept_area=*/true,
                                      {{"color", visualizer::Pen::BLUE},
                                       {"label", "NMPC Optimized"}});
        }
        // 至少有一条轨迹才出图。
        if (!post_result.preprocessed_traj.empty() ||
            !post_result.nmpc_traj.empty()) {
            visualizer.save("../fig");
        }
    } catch (const std::exception& e) {
        LOG_FMT_ERROR("Exception caught in main: {}!!!", e.what());
        return 1;
    }
    return 0;
}