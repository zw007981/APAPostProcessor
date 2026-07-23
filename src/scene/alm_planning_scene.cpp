#include "alm_planning_scene.h"

#include "../core/post_processor.h"
#include "../util/config_loader.h"
#include "../util/logger.h"

namespace apa_post_processor {
ALMPlanningScene::TypedAlmConfig ALMPlanningScene::MakeAlmConfig() {
    auto typed = std::make_unique<AlmConfig>();
    // 在移动所有权之前取得类型化观察指针，全程无需任何下行转换
    AlmConfig* observer = typed.get();
    return {std::move(typed), observer};
}

ALMPlanningScene::ALMPlanningScene(TypedAlmConfig&& typed)
    : PlanningScene(std::move(typed.base_ptr)), alm_config_(typed.typed_ptr) {}

ALMPlanningScene::ALMPlanningScene() : ALMPlanningScene(MakeAlmConfig()) {}

std::unique_ptr<ALMPlanningScene> ALMPlanningScene::LoadFromFile(
    const std::string& config_file_path) {
    auto scene = std::unique_ptr<ALMPlanningScene>(new ALMPlanningScene());
    scene->init(config_file_path);
    return scene;
}

void ALMPlanningScene::loadConfigDetails(
    const std::string& config_details_path) {
    if (!LoadBaseConfigOverrides(config_details_path, alm_config_)) {
        LOG_FMT_ERROR(
            "ALM config details load failed: {}, keep default config!!!",
            config_details_path);
    }
}

PostProcessorResult ALMPlanningScene::optimize() {
    const PostProcessor post_processor(vehicleParams(), footprintModel(),
                                       esdfMap());
    last_result_ = post_processor.optimizeAlm(initPath(), almConfig());
    LOG_FMT_INFO(
        "ALM PostProcessor result: success={}, maneuvers={}, length={:.3f}, "
        "time_ms={:.1f}, message={}",
        last_result_.success, last_result_.final_maneuvers,
        last_result_.final_length, last_result_.total_time_ms,
        last_result_.message);
    alm_traj_ = last_result_.alm_traj;
    alm_preprocessed_traj_ = last_result_.alm_preprocessed_traj;
    return last_result_;
}
}  // namespace apa_post_processor
