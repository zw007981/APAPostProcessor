#pragma once

#include "../core/NMPC/nmpc_config.h"
#include "../core/post_processor.h"
#include "planning_scene.h"

namespace apa_post_processor {
// NMPC 规划场景：构造时创建 NMPCConfig 并移交基类管理，实现完整优化链路。
class NMPCPlanningScene : public PlanningScene {
   public:
    NMPCPlanningScene();
    // 加载配置文件并构造 NMPCPlanningScene 实例
    static std::unique_ptr<NMPCPlanningScene> LoadFromFile(
        const std::string& config_file_path);
    // 执行 NMPC 优化链路
    PostProcessorResult optimize() override;
    // 从 proto JSON 文件加载 NMPC 配置详情
    void loadConfigDetails(const std::string& config_details_path) override;
    // 最近一次 NMPC 优化产出的轨迹（未执行 optimize() 或失败时为空）
    const Trajectory& optimizedTraj() const override { return optimized_traj_; }
    // 算法名（绘图标签与日志用）
    std::string algorithmName() const override { return "NMPC"; }
    // 带类型访问 NMPC 专有配置
    NMPCConfig& nmpcConfig() { return static_cast<NMPCConfig&>(config()); }
    const NMPCConfig& nmpcConfig() const {
        return static_cast<const NMPCConfig&>(config());
    }
};
}  // namespace apa_post_processor
