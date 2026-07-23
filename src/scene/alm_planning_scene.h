#pragma once

#include "../core/post_processor.h"
#include "planning_scene.h"

namespace apa_post_processor {
// ALM 规划场景：构造时创建 AlmConfig 并移交基类管理，实现完整优化链路。
// 注意：ALM 场景不产出 NMPC 预处理轨迹，基类的 preprocessedTraj() 对本类
// 恒返回空轨迹；优化产出经 optimizedTraj()/almTraj() 访问，"优化前"对比
// 基线请使用 almPreprocessedTraj()。
class ALMPlanningScene : public PlanningScene {
   public:
    ALMPlanningScene();
    // 加载配置文件并构造 ALMPlanningScene 实例
    static std::unique_ptr<ALMPlanningScene> LoadFromFile(
        const std::string& config_file_path);
    // 执行 ALM 优化链路
    PostProcessorResult optimize() override;
    // 从 ALM 配置详情 JSON 加载配置覆盖项（当前映射通用 Config 基类字段，
    // ALM 专有字段的映射随每算法一详情文件的约定扩展）
    void loadConfigDetails(const std::string& config_details_path) override;
    // 最近一次 ALM 优化产出的轨迹（未执行 optimize() 或失败时为空）
    const Trajectory& optimizedTraj() const override { return alm_traj_; }
    // 算法名（绘图标签与日志用）
    std::string algorithmName() const override { return "ALM"; }
    // 带类型访问 ALM 专有配置（构造期已固化类型，无下行转换）
    AlmConfig& almConfig() { return *alm_config_; }
    const AlmConfig& almConfig() const { return *alm_config_; }
    // 最近一次 ALM 优化产出的轨迹
    const Trajectory& almTraj() const { return alm_traj_; }
    // 最近一次 ALM 预处理粗优化产出的轨迹（"优化前"对比基线，与 almTraj()
    // 经同一套离散化管线产出）；预处理失败或未执行 optimize() 时为空
    const Trajectory& almPreprocessedTraj() const {
        return alm_preprocessed_traj_;
    }

   protected:
    // 构造期由 unique_ptr<AlmConfig> 直接取得的类型化配置观察指针（非拥有，
    // 生命周期与基类持有的配置对象一致）
    AlmConfig* alm_config_;
    // ALM 优化后轨迹（采样点携带 θ-s 轨迹全局时刻）
    Trajectory alm_traj_;
    // ALM 预处理粗优化轨迹（采样点携带 θ-s 轨迹全局时刻，"优化前"对比基线）
    Trajectory alm_preprocessed_traj_;

   private:
    // 委托构造的中间结构：基类持有配置所有权，本类保留类型化观察指针
    struct TypedAlmConfig {
        std::unique_ptr<Config> base_ptr;
        AlmConfig* typed_ptr;
    };
    explicit ALMPlanningScene(TypedAlmConfig&& typed);
    static TypedAlmConfig MakeAlmConfig();
};
}  // namespace apa_post_processor
