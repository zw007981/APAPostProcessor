#include "ilqr_solver.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>

#include "../../util/logger.h"
#include "bicycle_dynamics.h"
#include "ilqr_cost.h"
#include "ilqr_diagnostics.h"
#include "ilqr_post_stage.h"
#include "ilqr_reference_builder.h"
#include "esdf_constraint.h"

namespace apa_post_processor {
iLQRSolver::iLQRSolver(const VehicleParams& vehicle_params,
                     const VehicleFootprintModel& footprint_model,
                     const ESDFMap& esdf_map)
    : vehicle_params_(vehicle_params),
      footprint_model_(footprint_model),
      esdf_map_(esdf_map) {}

iLQRSolverResult iLQRSolver::optimizeSinglePass(
    const Path& init_path, const iLQRConfig& ilqr_config) const {
    iLQRSolverResult result;
    const auto t_start = std::chrono::steady_clock::now();

    try {
        if (init_path.empty()) {
            result.message = "iLQR input path is empty";
        } else {
            // 配置副本：按车辆真值收紧幅值边界（只收不放，自动同步到全部消费方）
            iLQRConfig config = ilqr_config;
            config.clampToVehicleParams(vehicle_params_);

            // 越界 ESDF
            // 查询计数复位（恢复场使小幅越界成为合法探测，单次汇总即可）
            esdf_map_.resetOutOfMapQueryCount();

            // 0) RS
            // 换挡点短接（默认关闭）：唯一能突破"换挡点由前端钉死"的几何前处理
            Path solver_input = init_path;
            if (config.rs_cap_ratio > 0.0) {
                const Path shortcut = ShortcutShiftPoints(
                    solver_input, esdf_map_, footprint_model_,
                    vehicle_params_.wheelbase, config.reference_delta_max,
                    config);
                LOG_FMT_INFO(
                    "iLQR RS shortcut: maneuvers {} -> {}, length {:.3f} -> "
                    "{:.3f}",
                    solver_input.numManeuvers(), shortcut.numManeuvers(),
                    solver_input.length(), shortcut.length());
                solver_input = std::move(shortcut);
            }

            // 1) 前端参考构建：等弧长重采样 + 初值提取 + 打靶节点布设
            const iLQRReferenceBuilder reference_builder(config,
                                                        vehicle_params_);
            iLQRReference reference = reference_builder.build(solver_input);

            // 2) 求解组件装配：ESDF 约束按配置缓存（外圆圆心提取有非平凡开销）
            const bool esdf_config_changed =
                !esdf_cache_ ||
                last_esdf_config_.esdf_margin_safe != config.esdf_margin_safe ||
                last_esdf_config_.esdf_margin_comf != config.esdf_margin_comf ||
                last_esdf_config_.esdf_weight_safe != config.esdf_weight_safe ||
                last_esdf_config_.esdf_weight_comf != config.esdf_weight_comf ||
                last_esdf_config_.esdf_stride != config.esdf_stride;
            if (esdf_config_changed) {
                esdf_cache_ = std::make_unique<iLQREsdfConstraint>(
                    esdf_map_, footprint_model_, config);
                last_esdf_config_ = config;
            }
            const BicycleDynamics dynamics(vehicle_params_.wheelbase);
            const iLQRCostEvaluator cost_evaluator(config,
                                                  esdf_cache_.get());
            ApaILQRSolver solver(config, &dynamics, &cost_evaluator);

            // 3) 阶段一全局软化求解（任何出口均带最后可用轨迹，anytime 性质）
            ApaILQRStageOneResult stage_one = solver.solveStageOne(reference);
            iLQRDiagnostics::LogStageOneReport(stage_one, config,
                                              footprint_model_, esdf_map_);

            // 4) 后处理与阶段二门控精化：游程分析/修剪/重解/驻留/校验
            const auto& goal_pt = init_path.back();
            const TrajectoryPoint goal(goal_pt.x, goal_pt.y, goal_pt.theta);
            iLQRPostStage post_stage(config, &reference_builder, &solver,
                                    vehicle_params_);
            auto post = post_stage.run(init_path, reference, stage_one, goal,
                                       esdf_map_, footprint_model_);
            iLQRDiagnostics::LogPostStageReport(post, stage_one, esdf_map_);

            const bool post_succeeded =
                post.status == iLQRPostStageStatus::SUCCESS ||
                post.status == iLQRPostStageStatus::SUCCESS_STAGE_ONE_ONLY;

            if (!post_succeeded) {
                // 失败不产出半成品，message 携带结构化诊断（失败阶段 + 失败项 +
                // 量测值/阈值）
                result.message = iLQRDiagnostics::BuildFailureMessage(post);
                iLQRDiagnostics::LogFailureDiagnostics(post, footprint_model_,
                                                      esdf_map_);
            } else {
                Path optimized;
                for (const auto& pt : post.trajectory.points()) {
                    optimized.addPoint(pt);
                }
                optimized.finalize();
                result.optimized_trajectory = std::move(post.trajectory);
                result.optimized_path = std::move(optimized);
                result.success = true;
                // 降级输出（阶段一候选）如实反映，不伪装成阶段二完全成功
                if (post.status == iLQRPostStageStatus::SUCCESS_STAGE_ONE_ONLY) {
                    result.message =
                        std::string(
                            "iLQR converged with stage-one candidate "
                            "(stage-two unavailable: ") +
                        post.diagnostics.degraded_reason + ")";
                    LOG_FMT_WARN("iLQR post stage degraded output: {}",
                                 result.message);
                } else {
                    result.message = "iLQR converged";
                }
                // 质量指标全量日志转储（记录不否决）
                for (const auto& check : post.diagnostics.metric_checks) {
                    LOG_FMT_INFO(
                        "iLQR post metric: {}={:.4f} (threshold {:.4f}, {})",
                        check.name, check.measured, check.threshold,
                        check.passed ? "pass" : "EXCEEDED");
                }
            }
        }
    } catch (const std::exception& e) {
        LOG_FMT_WARN("optimizeiLQR exception: {}", e.what());
        result.message = std::string("iLQR attempt failed: ") + e.what();
    }

    // 统一收尾
    result.total_time_ms = std::chrono::duration<double, std::milli>(
                               std::chrono::steady_clock::now() - t_start)
                               .count();
    result.output_level =
        result.success
            ? (result.message.find("stage-one candidate") != std::string::npos
                   ? OutputLevel::kDegraded
                   : OutputLevel::kFullSuccess)
            : OutputLevel::kFallback;
    result.final_maneuvers =
        result.optimized_trajectory.empty()
            ? static_cast<int>(result.optimized_path.numManeuvers())
            : result.optimized_trajectory.countDirectionRuns();
    result.final_length =
        result.optimized_path.empty() ? 0.0 : result.optimized_path.length();
    return result;
}
}  // namespace apa_post_processor
