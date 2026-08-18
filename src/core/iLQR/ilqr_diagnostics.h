#pragma once

#include <string>
#include <vector>

#include "apa_ilqr_solver.h"
#include "ilqr_cost.h"
#include "ilqr_post_stage.h"
#include "ilqr_reference_builder.h"
#include "../post_processor.h"
#include "../../spatial/esdf_map.h"
#include "../../vehicle/vehicle_footprint_model.h"

namespace apa_post_processor {

// iLQR 链路诊断工具集：全部方法为静态，可在任意上下文中调用（求解器、
// 调参工具、审计工具等），与 iLQRSolver 解耦以支持独立单元测试。
class iLQRDiagnostics {
   public:
    // 从后处理状态码构建单行失败消息文本（纯字符串构建，可测试）
    static std::string BuildFailureMessage(const iLQRPostStageResult& post);

    // 量测外圆侵入深度：所有外圆圆心到最近障碍物的最大穿透值（>0=侵入）
    static double MeasureIntrusion(const iLQRAlignedVec<iLQRState>& states,
                                   const VehicleFootprintModel& footprint_model,
                                   const ESDFMap& esdf_map);

    // 定位最大幅值违反：区分 v/a/ω/δ 五种约束中哪种在哪个节点最严重，
    // 输出诊断日志（未收敛外层轮次的最后一公里归因）
    static void LogWorstAmplitudeViolation(
        const iLQRAlignedVec<iLQRState>& states, const iLQRCostConfig& cost);

    // 阶段一求解报告（含收敛/未收敛双路径的诊断差异）
    static void LogStageOneReport(const ApaILQRStageOneResult& stage_one,
                                  const iLQRConfig& config,
                                  const VehicleFootprintModel& footprint_model,
                                  const ESDFMap& esdf_map);

    // 后处理阶段报告：状态码、换挡数对比、接缝数与越界查询统计
    static void LogPostStageReport(
        const iLQRPostStageResult& post,
        const ApaILQRStageOneResult& stage_one, const ESDFMap& esdf_map);

    // 失败诊断全量转储：失败消息 + 阶段二明细 + 逐接缝报告
    static void LogFailureDiagnostics(
        const iLQRPostStageResult& post,
        const VehicleFootprintModel& footprint_model, const ESDFMap& esdf_map);

   private:
    // 阶段二侵入诊断日志（由 LogFailureDiagnostics 内部分派）
    static void LogStageTwoIntrusion(
        const iLQRPostStageResult& post,
        const VehicleFootprintModel& footprint_model, const ESDFMap& esdf_map);
};

}  // namespace apa_post_processor
