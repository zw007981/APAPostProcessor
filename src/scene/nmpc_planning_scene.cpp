#include "nmpc_planning_scene.h"

#include "../core/NMPC/nmpc_config.h"
#include "../core/post_processor.h"
#include "../util/config_loader.h"
#include "../util/data_loader.hpp"
#include "../util/logger.h"

namespace apa_post_processor {
NMPCPlanningScene::NMPCPlanningScene()
    : PlanningScene(std::make_unique<NMPCConfig>()) {}

std::unique_ptr<NMPCPlanningScene> NMPCPlanningScene::LoadFromFile(
    const std::string& config_file_path) {
    auto scene = std::unique_ptr<NMPCPlanningScene>(new NMPCPlanningScene());
    scene->init(config_file_path);
    return scene;
}

void NMPCPlanningScene::loadConfigDetails(
    const std::string& config_details_path) {
    // 算法无关的基类字段覆盖项（outer_row_num 等）与 ALM 场景同一解析
    // 入口；proto 路由不识别这些字段，缺此会被静默忽略
    if (!LoadBaseConfigOverrides(config_details_path, &config())) {
        LOG_FMT_ERROR(
            "NMPC config details load failed: {}, keep default config!!!",
            config_details_path);
    }
    ::apa::post_processor::NMPCConfigProto proto;
    const auto load_result =
        DataLoader::LoadProtoFromJsonFile(config_details_path, proto);
    if (load_result == LoadResult::SUCCESS) {
        nmpcConfig().loadFromProto(proto);
    }
}

PostProcessorResult NMPCPlanningScene::optimize() {
    AdaptiveRetryConfig retry_config;
    const PostProcessor post_processor(vehicleParams(), footprintModel(),
                                       esdfMap());
    last_result_ =
        post_processor.optimize(initPath(), nmpcConfig(), retry_config);
    LOG_FMT_INFO(
        "PostProcessor result: success={}, maneuvers={}, length={:.3f}, "
        "time_ms={:.1f}, used_retry={}, message={}",
        last_result_.success, last_result_.final_maneuvers,
        last_result_.final_length, last_result_.total_time_ms,
        last_result_.used_retry, last_result_.message);
    // 从 intermediate_traces 中提取预处理轨迹
    preprocessed_traj_ = Trajectory{};
    for (const auto& [name, traj] : last_result_.intermediate_traces) {
        if (name == "preprocessed") {
            preprocessed_traj_ = traj;
            break;
        }
    }
    // 统一从 optimized_trajectory 读取（与 ALM/DDP 共用基类 optimized_traj_）
    optimized_traj_ = last_result_.optimized_trajectory;
    return last_result_;
}
}  // namespace apa_post_processor
