#include "ddp_planning_scene.h"

#include "../core/post_processor.h"
#include "../util/config_loader.h"
#include "../util/data_loader.hpp"
#include "../util/logger.h"

namespace apa_post_processor {
DDPPlanningScene::TypedDdpConfig DDPPlanningScene::MakeDdpConfig() {
    auto typed = std::make_unique<DdpConfig>();
    // 在移动所有权之前取得类型化观察指针，全程无需任何下行转换
    DdpConfig* observer = typed.get();
    return {std::move(typed), observer};
}

DDPPlanningScene::DDPPlanningScene(TypedDdpConfig&& typed)
    : PlanningScene(std::move(typed.base_ptr)), ddp_config_(typed.typed_ptr) {}

DDPPlanningScene::DDPPlanningScene() : DDPPlanningScene(MakeDdpConfig()) {}

std::unique_ptr<DDPPlanningScene> DDPPlanningScene::LoadFromFile(
    const std::string& config_file_path) {
    auto scene = std::make_unique<DDPPlanningScene>();
    scene->init(config_file_path);
    return scene;
}

void DDPPlanningScene::loadConfigDetails(
    const std::string& config_details_path) {
    // 算法无关的基类字段覆盖项（outer_row_num 等）与 ALM/NMPC 场景同一解析
    // 入口，防止基类字段被静默忽略（Config 双源缺口的历史教训）
    if (!LoadBaseConfigOverrides(config_details_path, ddp_config_)) {
        LOG_FMT_ERROR(
            "DDP config details load failed: {}, keep default config!!!",
            config_details_path);
        return;
    }
    // DDP 专有字段覆盖项：仅覆盖显式出现的字段，加载后幅值边界经单一
    // 权威来源同步
    auto details = nlohmann::json();
    if (DataLoader::LoadJsonFile(config_details_path, details) !=
        LoadResult::SUCCESS) {
        LOG_FMT_ERROR(
            "DDP config details parse failed: {}, keep default config!!!",
            config_details_path);
        return;
    }
    LoadDdpConfigOverrides(details, ddp_config_);
}

PostProcessorResult DDPPlanningScene::optimize() {
    const PostProcessor post_processor(vehicleParams(), footprintModel(),
                                       esdfMap());
    last_result_ = post_processor.optimizeDdp(initPath(), ddpConfig());
    LOG_FMT_INFO(
        "DDP PostProcessor result: success={}, maneuvers={}, length={:.3f}, "
        "time_ms={:.1f}, message={}",
        last_result_.success, last_result_.final_maneuvers,
        last_result_.final_length, last_result_.total_time_ms,
        last_result_.message);
    // 统一从 optimized_trajectory 读取（与 NMPC/ALM 共用基类 optimized_traj_）
    optimized_traj_ = last_result_.optimized_trajectory;
    return last_result_;
}
}  // namespace apa_post_processor
