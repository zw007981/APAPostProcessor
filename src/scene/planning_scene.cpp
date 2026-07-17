#include "planning_scene.h"

#include "../util/data_loader.hpp"
#include "../util/logger.h"

namespace apa_post_processor {
PlanningScene::PlanningScene(std::unique_ptr<Config> config)
    : config_(std::move(config)) {}

bool PlanningScene::init(const std::string& config_file_path) {
    try {
        // 读取配置文件获取数据文件路径
        auto config_json = nlohmann::json();
        const auto load_result =
            DataLoader::LoadJsonFile(config_file_path, config_json);
        const auto data_file_path =
            config_json["data_file_path"].get<std::string>();
        // 从数据文件加载 protobuf
        auto optimize_request = ::apa::post_processor::OptimizeRequest();
        (void)DataLoader::LoadProtoFromJsonFile(data_file_path,
                                                optimize_request);
        // 提取车辆参数
        vehicle_params_ = VehicleParams::FromProto(optimize_request.vehicle());
        LOG_FMT_INFO("Load config from {}, vehicle params: {}",
                     config_file_path, vehicle_params_.toString());
        // 提取栅格地图并构建 ESDF
        grid_map_ = std::make_unique<GridMap>(
            GridMap::FromProto(optimize_request.environment()));
        esdf_map_ = std::make_unique<ESDFMap>(*grid_map_);
        // 初始路径
        init_path_ = Path::FromProto(optimize_request.initial_path());
        // 车辆圆形分解模型
        footprint_model_ = std::make_unique<VehicleFootprintModel>(
            vehicle_params_, /*heading_sample_num=*/233, /*inner_row_num=*/2,
            /*outer_row_num=*/2);
        // 加载算法配置详情文件（如果存在）覆盖默认配置
        if (config_json.contains("config_details_path")) {
            const auto details_path =
                config_json["config_details_path"].get<std::string>();
            loadConfigDetails(details_path);
            LOG_FMT_INFO("Config overridden from {}", details_path);
        }
        return true;
    } catch (const std::exception& e) {
        LOG_FMT_ERROR("PlanningScene::init failed: {}!!!", e.what());
        return false;
    }
}
}  // namespace apa_post_processor
