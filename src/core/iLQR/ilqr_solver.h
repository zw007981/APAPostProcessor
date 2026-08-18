#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "../../util/path.h"
#include "../../util/trajectory.h"
#include "../../vehicle/vehicle_footprint_model.h"
#include "../../vehicle/vehicle_params.h"
#include "../post_processor.h"
#include "apa_ilqr_solver.h"
#include "ilqr_diagnostics.h"
#include "ilqr_post_stage.h"

namespace apa_post_processor {

// iLQR 单次求解的完整产出，由 PostProcessor 映射为统一 PostProcessorResult
struct iLQRSolverResult {
    // 最终优化轨迹
    Trajectory optimized_trajectory;
    // 优化后几何路径
    Path optimized_path;
    // 是否成功
    bool success = false;
    // 状态消息
    std::string message;
    // 端到端耗时 (ms)
    double total_time_ms = 0.0;
    // 最终方向段数
    int final_maneuvers = 0;
    // 最终路径长度 (m)
    double final_length = 0.0;
    // 输出级别
    OutputLevel output_level{OutputLevel::kFallback};
};

// iLQR 完整链路编排器：参考构建 → 阶段一 → 后处理与阶段二。无状态，每次
// optimizeSinglePass() 按需构造组件
class iLQRSolver {
   public:
    // 构造时持有车辆参数、足迹模型与 ESDF 地图的只读引用
    iLQRSolver(const VehicleParams& vehicle_params,
              const VehicleFootprintModel& footprint_model,
              const ESDFMap& esdf_map);

    // 单遍完整链路，不含双候选择优与 RS 短接回退（那两层由 PostProcessor 调度）
    iLQRSolverResult optimizeSinglePass(const Path& init_path,
                                       const iLQRConfig& config) const;

   protected:
    // 车辆物理参数（外部持有，只读引用）
    const VehicleParams& vehicle_params_;
    // 车辆足迹模型（外部持有，只读引用）
    const VehicleFootprintModel& footprint_model_;
    // 符号距离场（外部持有，只读引用）
    const ESDFMap& esdf_map_;

    // ESDF 约束缓存：外圆圆心提取（ExtractLocalCircleCenters）有非平凡
    // 计算开销，而足迹模型在 iLQRSolver 生命周期内不变。缓存避免双候选
    // 场景重复提取
    mutable std::unique_ptr<iLQREsdfConstraint> esdf_cache_;
    // 上次缓存对应的 ESDF 配置，用于判断是否需要重建
    mutable iLQREsdfConstraintConfig last_esdf_config_;
};

}  // namespace apa_post_processor
