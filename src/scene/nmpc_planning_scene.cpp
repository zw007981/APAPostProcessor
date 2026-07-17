#include "nmpc_planning_scene.h"

#include "../core/post_processor.h"
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
    preprocessed_traj_ = last_result_.preprocessed_traj;
    nmpc_traj_ = last_result_.nmpc_traj;
    return last_result_;
}
}  // namespace apa_post_processor
