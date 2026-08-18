#include "planning_scene.h"

#include <iomanip>
#include <sstream>

#include "../util/data_loader.hpp"
#include "../util/logger.h"
#include "minco_planning_scene.h"
#include "ilqr_planning_scene.h"
#include "nmpc_planning_scene.h"

namespace apa_post_processor {
PlanningScene::PlanningScene(std::unique_ptr<Config> config)
    : config_(std::move(config)) {}

std::string PlanningScene::optimizeSummary() const {
    std::ostringstream oss;
    if (!last_result_.success) {
        // 未执行或失败：不输出优化后指标，注明原因与耗时
        oss << algorithmName() << " optimize failed or not executed: message=\""
            << last_result_.message << "\"";
        if (last_result_.total_time_ms > 0.0) {
            oss << ", time_ms=" << std::fixed << std::setprecision(1)
                << last_result_.total_time_ms;
        }
        return oss.str();
    }
    oss << algorithmName() << " optimize summary: length=" << std::fixed
        << std::setprecision(3) << init_path_.length() << "->"
        << last_result_.final_length
        << "m, maneuvers=" << init_path_.numManeuvers() << "->"
        << last_result_.final_maneuvers << ", time_ms=" << std::setprecision(1)
        << last_result_.total_time_ms;
    return oss.str();
}

void PlanningScene::printOptimizeSummary() const {
    if (!last_result_.success) {
        LOG_FMT_ERROR("{}", optimizeSummary());
        return;
    }
    LOG_FMT_INFO("{}", optimizeSummary());
}

std::unique_ptr<PlanningScene> PlanningScene::LoadFromFile(
    const std::string& config_file_path) {
    auto config_json = nlohmann::json();
    if (DataLoader::LoadJsonFile(config_file_path, config_json) !=
        LoadResult::SUCCESS) {
        LOG_FMT_ERROR("PlanningScene::LoadFromFile failed to load {}!!!",
                      config_file_path);
        return nullptr;
    }
    if (!config_json.contains("config_details_path")) {
        LOG_FMT_ERROR(
            "PlanningScene::LoadFromFile: {} misses config_details_path!!!",
            config_file_path);
        return nullptr;
    }
    // 按约定从算法配置详情 JSON 的 "algorithm" 字段路由到对应算法场景
    const auto details_path =
        config_json["config_details_path"].get<std::string>();
    auto details_json = nlohmann::json();
    if (DataLoader::LoadJsonFile(details_path, details_json) !=
            LoadResult::SUCCESS ||
        !details_json.contains("algorithm")) {
        LOG_FMT_ERROR(
            "PlanningScene::LoadFromFile: algorithm config details {} "
            "unreadable or misses algorithm field!!!",
            details_path);
        return nullptr;
    }
    const auto algorithm = details_json["algorithm"].get<std::string>();
    if (algorithm == "minco") {
        return MINCOPlanningScene::LoadFromFile(config_file_path);
    }
    if (algorithm == "nmpc") {
        return NMPCPlanningScene::LoadFromFile(config_file_path);
    }
    if (algorithm == "ilqr") {
        return ILQRPlanningScene::LoadFromFile(config_file_path);
    }
    LOG_FMT_ERROR(
        "PlanningScene::LoadFromFile: unknown algorithm \"{}\" in {}!!!",
        algorithm, details_path);
    return nullptr;
}

bool PlanningScene::init(const std::string& config_file_path) {
    try {
        // 读取配置文件获取数据文件路径
        auto config_json = nlohmann::json();
        const auto load_result =
            DataLoader::LoadJsonFile(config_file_path, config_json);
        (void)load_result;
        const auto data_file_path =
            config_json["data_file_path"].get<std::string>();
        // 先加载算法配置详情（footprint 模型依赖其中的 outer_row_num），
        // 再构建场景资源
        if (config_json.contains("config_details_path")) {
            const auto details_path =
                config_json["config_details_path"].get<std::string>();
            loadConfigDetails(details_path);
            LOG_FMT_INFO("Config overridden from {}", details_path);
        }
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
        // 车辆圆形分解模型：外圆行数取自算法配置（碰撞模型参数，影响
        // MINCO/NMPC 两条链路的 ESDF 惩罚与合法性校验）
        footprint_model_ = std::make_unique<VehicleFootprintModel>(
            vehicle_params_, /*heading_sample_num=*/233, /*inner_row_num=*/2,
            config_->outer_row_num);
        LOG_FMT_INFO(
            "Vehicle footprint model: heading_sample_num=233, "
            "inner_row_num=2, outer_row_num={}",
            config_->outer_row_num);
        return true;
    } catch (const std::exception& e) {
        LOG_FMT_ERROR("PlanningScene::init failed: {}!!!", e.what());
        return false;
    }
}
}  // namespace apa_post_processor
