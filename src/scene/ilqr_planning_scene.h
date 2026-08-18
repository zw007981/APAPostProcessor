#pragma once

#include "../core/post_processor.h"
#include "planning_scene.h"

namespace apa_post_processor {
// iLQR 规划场景：构造时创建 iLQRConfig 并移交基类管理，实现完整优化链路。
// 注意：iLQR 场景不产出 NMPC 预处理轨迹，基类的 preprocessedTraj() 对本类
// 恒返回空轨迹；优化产出经 optimizedTraj()/ilqrTraj() 访问
class ILQRPlanningScene : public PlanningScene {
   public:
    ILQRPlanningScene();
    // 加载配置文件并构造 ILQRPlanningScene 实例
    static std::unique_ptr<ILQRPlanningScene> LoadFromFile(
        const std::string& config_file_path);
    // 执行 iLQR 优化链路
    PostProcessorResult optimize() override;
    // 从 iLQR 配置详情 JSON 加载配置覆盖项（通用 Config 基类字段与 iLQR 专有
    // 字段，二者字段集互不重叠）
    void loadConfigDetails(const std::string& config_details_path) override;
    // 最近一次 iLQR 优化产出的轨迹（未执行 optimize() 或失败时为空）
    const Trajectory& optimizedTraj() const override { return optimized_traj_; }
    // 算法名（绘图标签与日志用）
    std::string algorithmName() const override { return "iLQR"; }
    // 带类型访问 iLQR 专有配置（构造期已固化类型，无下行转换）
    iLQRConfig& ilqrConfig() { return *ilqr_config_; }
    const iLQRConfig& ilqrConfig() const { return *ilqr_config_; }
    // 最近一次 iLQR 优化产出的轨迹（与 optimizedTraj() 同源，保留以兼容
    // 算法特定的旧调用方；Phase 3 起调用方应统一使用 optimizedTraj()）
    const Trajectory& ilqrTraj() const { return optimized_traj_; }

   protected:
    // 构造期由 unique_ptr<iLQRConfig> 直接取得的类型化配置观察指针（非拥有，
    // 生命周期与基类持有的配置对象一致）
    iLQRConfig* ilqr_config_;

   private:
    // 委托构造的中间结构：基类持有配置所有权，本类保留类型化观察指针
    struct TypediLQRConfig {
        std::unique_ptr<Config> base_ptr;
        iLQRConfig* typed_ptr;
    };
    explicit ILQRPlanningScene(TypediLQRConfig&& typed);
    static TypediLQRConfig MakeiLQRConfig();
};
}  // namespace apa_post_processor
