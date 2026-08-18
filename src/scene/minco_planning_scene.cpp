#include "minco_planning_scene.h"

#include "../core/post_processor.h"
#include "../util/config_loader.h"
#include "../util/data_loader.hpp"
#include "../util/logger.h"

namespace apa_post_processor {
MINCOPlanningScene::TypedMincoConfig MINCOPlanningScene::MakeMincoConfig() {
    auto typed = std::make_unique<MincoConfig>();
    // 在移动所有权之前取得类型化观察指针，全程无需任何下行转换
    MincoConfig* observer = typed.get();
    return {std::move(typed), observer};
}

MINCOPlanningScene::MINCOPlanningScene(TypedMincoConfig&& typed)
    : PlanningScene(std::move(typed.base_ptr)), minco_config_(typed.typed_ptr) {}

MINCOPlanningScene::MINCOPlanningScene() : MINCOPlanningScene(MakeMincoConfig()) {}

std::unique_ptr<MINCOPlanningScene> MINCOPlanningScene::LoadFromFile(
    const std::string& config_file_path) {
    auto scene = std::unique_ptr<MINCOPlanningScene>(new MINCOPlanningScene());
    scene->init(config_file_path);
    return scene;
}

void MINCOPlanningScene::loadConfigDetails(
    const std::string& config_details_path) {
    // 算法无关的基类字段覆盖项（outer_row_num 等）与 iLQR/NMPC 场景同一解析
    // 入口，防止基类字段被静默忽略（Config 双源缺口的历史教训）
    if (!LoadBaseConfigOverrides(config_details_path, minco_config_)) {
        LOG_FMT_ERROR(
            "MINCO config details load failed: {}, keep default config!!!",
            config_details_path);
        return;
    }
    // MINCO 专有字段覆盖项：仅覆盖显式出现的字段，未出现的保持默认值
    auto details = nlohmann::json();
    if (DataLoader::LoadJsonFile(config_details_path, details) !=
        LoadResult::SUCCESS) {
        LOG_FMT_ERROR(
            "MINCO config details parse failed: {}, keep default config!!!",
            config_details_path);
        return;
    }
    LoadMincoConfigOverrides(details, minco_config_);
}

PostProcessorResult MINCOPlanningScene::optimize() {
    const PostProcessor post_processor(vehicleParams(), footprintModel(),
                                       esdfMap());
    last_result_ = post_processor.optimizeMinco(initPath(), mincoConfig());
    LOG_FMT_INFO(
        "MINCO PostProcessor result: success={}, maneuvers={}, length={:.3f}, "
        "time_ms={:.1f}, message={}",
        last_result_.success, last_result_.final_maneuvers,
        last_result_.final_length, last_result_.total_time_ms,
        last_result_.message);
    // 从 intermediate_traces 中提取预处理轨迹（"优化前"基线）
    minco_preprocessed_traj_ = Trajectory{};
    for (const auto& [name, traj] : last_result_.intermediate_traces) {
        if (name == "minco_preprocessed") {
            minco_preprocessed_traj_ = traj;
            break;
        }
    }
    // 统一从 optimized_trajectory 读取（与 NMPC/iLQR 共用基类 optimized_traj_）
    optimized_traj_ = last_result_.optimized_trajectory;
    return last_result_;
}
}  // namespace apa_post_processor
