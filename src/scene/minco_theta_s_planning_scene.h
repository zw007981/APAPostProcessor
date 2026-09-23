#pragma once

#include "../core/post_processor.h"
#include "planning_scene.h"

namespace apa_post_processor {
// MINCO_THETA_S 规划场景：构造时创建 MincoThetaSConfig 并移交基类管理，实现完整优化链路。
// 注意：MINCO_THETA_S 场景不产出 NMPC 预处理轨迹，基类的 preprocessedTraj() 对本类
// 恒返回空轨迹；优化产出经 optimizedTraj()/mincoThetaSTraj() 访问，"优化前"对比
// 基线请使用 mincoThetaSPreprocessedTraj()。
class MincoThetaSPlanningScene : public PlanningScene {
   public:
    MincoThetaSPlanningScene();
    // 加载配置文件并构造 MincoThetaSPlanningScene 实例
    static std::unique_ptr<MincoThetaSPlanningScene> LoadFromFile(
        const std::string& config_file_path);
    // 执行 MINCO_THETA_S 优化链路
    PostProcessorResult optimize() override;
    // 从 MINCO_THETA_S 配置详情 JSON 加载配置覆盖项（当前映射通用 Config 基类字段，
    // MINCO_THETA_S 专有字段的映射随每算法一详情文件的约定扩展）
    void loadConfigDetails(const std::string& config_details_path) override;
    // 最近一次 MINCO_THETA_S 优化产出的轨迹（未执行 optimize() 或失败时为空）
    const Trajectory& optimizedTraj() const override { return optimized_traj_; }
    // 算法名（绘图标签与日志用）
    std::string algorithmName() const override { return "MINCO_THETA_S"; }
    // 带类型访问 MINCO_THETA_S 专有配置（构造期已固化类型，无下行转换）
    MincoThetaSConfig& mincoThetaSConfig() { return *minco_theta_s_config_; }
    const MincoThetaSConfig& mincoThetaSConfig() const { return *minco_theta_s_config_; }
    // 最近一次 MINCO_THETA_S 优化产出的轨迹（与 optimizedTraj() 同源，保留以兼容
    // 算法特定的旧调用方；Phase 3 起调用方应统一使用 optimizedTraj()）
    const Trajectory& mincoThetaSTraj() const { return optimized_traj_; }
    // 最近一次 MINCO_THETA_S 预处理粗优化产出的轨迹（"优化前"对比基线，与 mincoThetaSTraj()
    // 经同一套离散化管线产出）；预处理失败或未执行 optimize() 时为空
    const Trajectory& mincoThetaSPreprocessedTraj() const {
        return minco_theta_s_preprocessed_traj_;
    }

   protected:
    // 构造期由 unique_ptr<MincoThetaSConfig> 直接取得的类型化配置观察指针（非拥有，
    // 生命周期与基类持有的配置对象一致）
    MincoThetaSConfig* minco_theta_s_config_;
    // MINCO_THETA_S 预处理粗优化轨迹（采样点携带 θ-s 轨迹全局时刻，"优化前"对比基线）
    Trajectory minco_theta_s_preprocessed_traj_;

   private:
    // 委托构造的中间结构：基类持有配置所有权，本类保留类型化观察指针
    struct TypedMincoThetaSConfig {
        std::unique_ptr<Config> base_ptr;
        MincoThetaSConfig* typed_ptr;
    };
    explicit MincoThetaSPlanningScene(TypedMincoThetaSConfig&& typed);
    static TypedMincoThetaSConfig MakeMincoThetaSConfig();
};
}  // namespace apa_post_processor
