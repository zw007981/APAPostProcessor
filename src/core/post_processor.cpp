#include "post_processor.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>

#include "../util/topology_cleaner.h"
#include "NMPC/vehicle_circle_geometry.h"

namespace apa_post_processor {
namespace {
// 对 Path 中所有点做碰撞深度诊断（纯日志，不做门禁），返回最大碰撞深度(m)。
double ComputeMaxCollisionDepth(const Path& path, const ESDFMap& esdf_map,
                                const VehicleFootprintModel& footprint_model) {
    double max_collision = 0.0;
    const double R = footprint_model.getOuterRadius();
    for (std::size_t m = 0; m < path.numManeuvers(); ++m) {
        for (const auto& pt : path.getManeuvers()[m].points) {
            const double c = std::cos(pt.theta), s = std::sin(pt.theta);
            for (const auto& local :
                 vehicle_circle_geometry::ExtractLocalCircleCenters(
                     footprint_model, CircleType::OUTER)) {
                const double wx = pt.x + local.x() * c - local.y() * s;
                const double wy = pt.y + local.x() * s + local.y() * c;
                const double d = esdf_map.getDist(wx, wy);
                max_collision = std::max(max_collision, R - d);
            }
        }
    }
    return max_collision;
}
}  // namespace
PostProcessor::PostProcessor(const VehicleParams& vehicle_params,
                             const VehicleFootprintModel& footprint_model,
                             const ESDFMap& esdf_map)
    : vehicle_params_(vehicle_params),
      footprint_model_(footprint_model),
      esdf_map_(esdf_map) {}

PostProcessorResult PostProcessor::optimize(
    const Path& init_path, const PreprocessingPipelineConfig& pipeline_config,
    const NmpcSolverConfig& nmpc_config,
    const AdaptiveRetryConfig& retry_config) const {
    (void)retry_config;
    auto attempt = runSingleAttempt(init_path, pipeline_config, nmpc_config);
    PostProcessorResult result;
    result.optimized_path = std::move(attempt.optimized_path);
    result.success = !result.optimized_path.empty();
    result.used_retry = false;
    result.message = attempt.message;
    result.total_time_ms = attempt.time_ms;
    result.final_maneuvers = result.optimized_path.numManeuvers();
    result.final_length = result.optimized_path.length();
    result.preprocessed_traj = std::move(attempt.preprocessed_traj);
    result.nmpc_traj = std::move(attempt.nmpc_traj);
    return result;
}

PostProcessor::AttemptResult PostProcessor::runSingleAttempt(
    const Path& init_path, const PreprocessingPipelineConfig& pipeline_config,
    const NmpcSolverConfig& nmpc_config) const {
    AttemptResult result;
    const auto t_start = std::chrono::steady_clock::now();

    // 单次全链路求解：预处理 → OCP → NMPC → 碰撞检查 → 拓扑清洗
    auto solveFullPipeline =
        [&](const Path& input_path,
            std::vector<TrajectoryPoint>* preprocessed_out = nullptr,
            std::vector<TrajectoryPoint>* nmpc_out = nullptr) -> Path {
        try {
            const PreprocessingPipeline pipeline(
                pipeline_config, vehicle_params_, footprint_model_, esdf_map_);
            const auto pipe_result = pipeline.run(input_path);
            if (!pipe_result.success) {
                return Path{};
            }
            // 填充预处理轨迹点（带时间戳）
            if (preprocessed_out) {
                preprocessed_out->clear();
                preprocessed_out->reserve(pipe_result.z_ref.size());
                double t = 0.0;
                for (std::size_t i = 0; i < pipe_result.z_ref.size(); ++i) {
                    TrajectoryPoint tp = pipe_result.z_ref[i];
                    if (i > 0 && i - 1 < pipe_result.delta_t.size()) {
                        t += pipe_result.delta_t[i - 1];
                    }
                    tp.setT(t);
                    preprocessed_out->push_back(tp);
                }
            }
            const PreprocessingToOcpConverter converter(
                vehicle_params_, nmpc_config.path_to_ocp_config);
            const auto conv = converter.convert(input_path, pipe_result);
            auto local_nmpc_config = nmpc_config;
            local_nmpc_config.static_corridor_C = conv.static_corridor_C;
            local_nmpc_config.static_corridor_d = conv.static_corridor_d;
            const NmpcSolver nmpc_solver(vehicle_params_, footprint_model_,
                                         local_nmpc_config);
            const auto nmpc_result =
                nmpc_solver.optimize(conv.ocp, conv.init_guess, esdf_map_);
            // 填充 NMPC 轨迹点（带时间戳，与预处理共用时间网格）。
            if (nmpc_out && !nmpc_result.trajectory.x.empty()) {
                nmpc_out->clear();
                const std::size_t total_states =
                    nmpc_result.trajectory.x.size();
                nmpc_out->reserve(total_states);
                int global_k = 0;
                int global_u = 0;
                double t = 0.0;
                for (std::size_t seg_idx = 0;
                     seg_idx < nmpc_result.segment_steps.size(); ++seg_idx) {
                    const int step_num = nmpc_result.segment_steps[seg_idx];
                    for (int i = 0; i <= step_num; ++i) {
                        const auto& state =
                            nmpc_result.trajectory
                                .x[static_cast<std::size_t>(global_k + i)];
                        TrajectoryPoint tp(state(0), state(1), state(2));
                        tp.setV(state(3));
                        tp.setDelta(state(4));
                        if (i < step_num) {
                            const auto& control =
                                nmpc_result.trajectory
                                    .u[static_cast<std::size_t>(global_u + i)];
                            tp.setA(control(0));
                            tp.setDeltaDot(control(1));
                        }
                        if (i > 0) {
                            // 从 pipe_result.delta_t 取时间步长
                            const std::size_t dt_idx =
                                static_cast<std::size_t>(global_k + i - 1);
                            if (dt_idx < pipe_result.delta_t.size()) {
                                t += pipe_result.delta_t[dt_idx];
                            }
                        }
                        tp.setT(t);
                        nmpc_out->push_back(tp);
                    }
                    global_k += step_num;
                    global_u += step_num;
                }
            }
            if (nmpc_result.trajectory.x.empty()) {
                // NMPC 失败但预处理成功 → 回退到预处理轨迹
                Path fallback;
                for (const auto& z : pipe_result.z_ref) {
                    fallback.addPoint(z);
                }
                fallback.finalize();
                result.message =
                    "NMPC solve failed, falling back to preprocessing";
                result.nmpc_had_output = false;
                return fallback;
            }
            // 碰撞安全检查
            double max_collision = 0.0;
            const double R = footprint_model_.getOuterRadius();
            for (const auto& x : nmpc_result.trajectory.x) {
                const double c = std::cos(x(2)), s = std::sin(x(2));
                for (const auto& local :
                     vehicle_circle_geometry::ExtractLocalCircleCenters(
                         footprint_model_, CircleType::OUTER)) {
                    const double wx = x(0) + local.x() * c - local.y() * s;
                    const double wy = x(1) + local.x() * s + local.y() * c;
                    const double d = esdf_map_.getDist(wx, wy);
                    max_collision = std::max(max_collision, R - d);
                }
            }
            constexpr double kMaxAcceptableCollision = 0.02;
            if (max_collision > kMaxAcceptableCollision) {
                LOG_FMT_WARN(
                    "NMPC collision depth {:.4f}m exceeds threshold, "
                    "falling back",
                    max_collision);
                // 碰撞超标 → 回退到预处理轨迹
                Path fallback;
                for (const auto& z : pipe_result.z_ref) {
                    fallback.addPoint(z);
                }
                fallback.finalize();
                result.message = "NMPC collision depth " +
                                 std::to_string(max_collision) +
                                 "m, falling back to preprocessing";
                result.nmpc_had_output = false;
                return fallback;
            }
            // ToPath + 拓扑清洗
            Path optimized = NmpcSolver::ToPath(nmpc_result);
            const std::size_t before = optimized.numManeuvers();
            auto& maneuvers = optimized.getManeuvers();
            TopologyCleanupConfig cleanup_config;
            ClassifyAndResetManeuvers(maneuvers, cleanup_config);
            int pivot_count = 0;
            for (const auto& m : maneuvers) {
                if (m.direction == Direction::PIVOT) {
                    ++pivot_count;
                }
            }
            optimized = ReconstructPath(maneuvers);
            const double diag_collision = ComputeMaxCollisionDepth(
                optimized, esdf_map_, footprint_model_);
            LOG_FMT_INFO(
                "Topology Cleanup: maneuvers {}→{}, PIVOT={}, "
                "collision_depth={:.4f}m",
                before, optimized.numManeuvers(), pivot_count, diag_collision);
            // 记录 NMPC 收敛状态到 message（供外层使用）
            result.nmpc_converged = nmpc_result.converged;
            result.nmpc_had_output = true;
            result.message = nmpc_result.converged
                                 ? "NMPC converged"
                                 : "NMPC did not fully converge, using last "
                                   "iterate";
            return optimized;
        } catch (const std::exception& e) {
            LOG_FMT_WARN("solveFullPipeline exception: {}", e.what());
            return Path{};
        }
    };

    try {
        // 单次全链路求解：Milestone 023 三次重构起彻底删除了预处理层"剪枝
        // 重试"循环（RemoveShortestManeuver 及其四重门禁判定），改由 NMPC
        // 内生的 J_smooth/J_target 等代价机制承担段数削减压力，不再需要
        // 预处理层盲拼接重试验证（详见 docs/NMPC.md 6.7 节）。
        Path best_path = solveFullPipeline(init_path, &result.preprocessed_traj,
                                           &result.nmpc_traj);
        if (best_path.empty()) {
            result.message = "Initial NMPC solve failed";
            const auto t_end = std::chrono::steady_clock::now();
            result.time_ms =
                std::chrono::duration<double, std::milli>(t_end - t_start)
                    .count();
            return result;
        }
        result.optimized_path = std::move(best_path);
        result.nmpc_had_output = true;
    } catch (const std::exception& e) {
        result.message = std::string("Attempt failed: ") + e.what();
        result.nmpc_had_output = false;
    }

    const auto t_end = std::chrono::steady_clock::now();
    result.time_ms =
        std::chrono::duration<double, std::milli>(t_end - t_start).count();
    return result;
}

PreprocessingPipelineConfig PostProcessor::applyRetryConfig(
    const PreprocessingPipelineConfig& base_config,
    const AdaptiveRetryConfig& retry_config, int retry_idx) {
    auto config = base_config;
    const std::size_t idx = static_cast<std::size_t>(retry_idx);
    const double dense_mult =
        idx < retry_config.dense_step_dist_multipliers.size()
            ? retry_config.dense_step_dist_multipliers[idx]
            : retry_config.dense_step_dist_multipliers.back();
    const double nominal_mult =
        idx < retry_config.nominal_step_s_multipliers.size()
            ? retry_config.nominal_step_s_multipliers[idx]
            : retry_config.nominal_step_s_multipliers.back();
    const bool use_corridor =
        idx < retry_config.use_static_corridor_flags.size()
            ? retry_config.use_static_corridor_flags[idx]
            : retry_config.use_static_corridor_flags.back();
    if (std::isfinite(dense_mult) && dense_mult > 0.0) {
        config.bspline.dense_step_dist *= dense_mult;
    }
    if (std::isfinite(nominal_mult) && nominal_mult > 0.0) {
        config.resampler.nominal_step_s *= nominal_mult;
    }
    config.use_static_corridor = use_corridor;
    return config;
}
}  // namespace apa_post_processor
