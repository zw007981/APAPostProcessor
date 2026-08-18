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
    // 从 intermediate_traces 中提取预处理轨迹（"优化前"基线）
    alm_preprocessed_traj_ = Trajectory{};
    for (const auto& [name, traj] : last_result_.intermediate_traces) {
        if (name == "alm_preprocessed") {
            alm_preprocessed_traj_ = traj;
            break;
        }
    }
    // 统一从 optimized_trajectory 读取（与 NMPC/iLQR 共用基类 optimized_traj_）
    optimized_traj_ = last_result_.optimized_trajectory;
    return last_result_;
}
}  // namespace apa_post_processor
