#pragma once

#include "../core/post_processor.h"
#include "planning_scene.h"

namespace apa_post_processor {
// DDP 规划场景：构造时创建 DdpConfig 并移交基类管理，实现完整优化链路。
// 注意：DDP 场景不产出 NMPC 预处理轨迹，基类的 preprocessedTraj() 对本类
// 恒返回空轨迹；优化产出经 optimizedTraj()/ddpTraj() 访问
class DDPPlanningScene : public PlanningScene {
   public:
    DDPPlanningScene();
    // 加载配置文件并构造 DDPPlanningScene 实例
    static std::unique_ptr<DDPPlanningScene> LoadFromFile(
        const std::string& config_file_path);
    // 执行 DDP 优化链路
    PostProcessorResult optimize() override;
    // 从 DDP 配置详情 JSON 加载配置覆盖项（通用 Config 基类字段与 DDP 专有
    // 字段，二者字段集互不重叠）
    void loadConfigDetails(const std::string& config_details_path) override;
    // 最近一次 DDP 优化产出的轨迹（未执行 optimize() 或失败时为空）
    const Trajectory& optimizedTraj() const override { return optimized_traj_; }
    // 算法名（绘图标签与日志用）
    std::string algorithmName() const override { return "DDP"; }
    // 带类型访问 DDP 专有配置（构造期已固化类型，无下行转换）
    DdpConfig& ddpConfig() { return *ddp_config_; }
    const DdpConfig& ddpConfig() const { return *ddp_config_; }
    // 最近一次 DDP 优化产出的轨迹（与 optimizedTraj() 同源，保留以兼容
    // 算法特定的旧调用方；Phase 3 起调用方应统一使用 optimizedTraj()）
    const Trajectory& ddpTraj() const { return optimized_traj_; }

   protected:
    // 构造期由 unique_ptr<DdpConfig> 直接取得的类型化配置观察指针（非拥有，
    // 生命周期与基类持有的配置对象一致）
    DdpConfig* ddp_config_;

   private:
    // 委托构造的中间结构：基类持有配置所有权，本类保留类型化观察指针
    struct TypedDdpConfig {
        std::unique_ptr<Config> base_ptr;
        DdpConfig* typed_ptr;
    };
    explicit DDPPlanningScene(TypedDdpConfig&& typed);
    static TypedDdpConfig MakeDdpConfig();
};
}  // namespace apa_post_processor
