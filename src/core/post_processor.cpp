#include "post_processor.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>

#include "../preprocessing/preprocessing_pipeline.h"
#include "../util/topology_cleaner.h"
#include "ALM/alm_steer_padding.h"
#include "ALM/alm_trajectory_sampler.h"
#include "NMPC/vehicle_circle_geometry.h"
#include "collision_check.h"

namespace apa_post_processor {
namespace {
// 优化结果允许的最大碰撞深度 (m)：NMPC 与 ALM 两条路径共用同一质量门
constexpr double kMaxAcceptableCollisionDepth = 0.02;
// 从 NMPCConfig 构造 PreprocessingPipelineConfig（过渡期辅助函数）
PreprocessingPipelineConfig BuildPipelineConfig(const NMPCConfig& nmpc_config) {
    PreprocessingPipelineConfig cfg;
    cfg.bspline = nmpc_config.bspline;
    cfg.speed = nmpc_config.speed;
    cfg.diff_flat = nmpc_config.diff_flat;
    cfg.resampler = nmpc_config.resampler;
    cfg.corridor = nmpc_config.corridor;
    cfg.use_static_corridor = nmpc_config.use_static_corridor;
    cfg.collision_safety_margin = nmpc_config.collision_safety_margin;
    cfg.enable_debug_output = nmpc_config.enable_debug_output;
    return cfg;
}
// 把离散化产出的 Maneuver 序列展平为 Trajectory（保持原有顺序，逐点携带
// v/a/delta/delta_dot，未携带时间戳），供可视化与下游消费
Trajectory FlattenManeuvers(const std::vector<Maneuver>& maneuvers) {
    std::size_t total_points = 0;
    for (const auto& maneuver : maneuvers) {
        total_points += maneuver.points.size();
    }
    Trajectory trajectory;
    trajectory.reserve(total_points);
    for (const auto& maneuver : maneuvers) {
        for (const auto& pt : maneuver.points) {
            trajectory.push_back(pt);
        }
    }
    return trajectory;
}
}  // namespace
PostProcessor::PostProcessor(const VehicleParams& vehicle_params,
                             const VehicleFootprintModel& footprint_model,
                             const ESDFMap& esdf_map)
    : vehicle_params_(vehicle_params),
      footprint_model_(footprint_model),
      esdf_map_(esdf_map) {}

PostProcessorResult PostProcessor::optimize(
    const Path& init_path, const NMPCConfig& nmpc_config,
    const AdaptiveRetryConfig& retry_config) const {
    (void)retry_config;
    auto attempt = runSingleAttempt(init_path, nmpc_config);
    PostProcessorResult result;
    result.optimized_path = std::move(attempt.optimized_path);
    result.success = !result.optimized_path.empty();
    result.used_retry = false;
    result.message = attempt.message;
    result.total_time_ms = attempt.time_ms;
    // 物理方向段数（v 变号）为默认口径；轨迹缺失（回退预处理路径）时
    // 回退为 Path 机动段标签数
    result.final_maneuvers =
        attempt.nmpc_traj.empty()
            ? static_cast<int>(result.optimized_path.numManeuvers())
            : attempt.nmpc_traj.countDirectionRuns();
    result.final_length = result.optimized_path.length();
    result.preprocessed_traj = std::move(attempt.preprocessed_traj);
    result.nmpc_traj = std::move(attempt.nmpc_traj);
    return result;
}

PostProcessor::AttemptResult PostProcessor::runSingleAttempt(
    const Path& init_path, const NMPCConfig& nmpc_config) const {
    AttemptResult result;
    const auto t_start = std::chrono::steady_clock::now();

    // 单次全链路求解：预处理 → OCP → NMPC → 碰撞检查 → 拓扑清洗
    const auto pipeline_config = BuildPipelineConfig(nmpc_config);
    auto solveFullPipeline = [&](const Path& input_path,
                                 Trajectory* preprocessed_out = nullptr,
                                 Trajectory* nmpc_out = nullptr) -> Path {
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
                        // 状态增广模型（7 维）中 a/delta_dot 是状态分量，
                        // 每个点都有真值；5 维模型中来自控制序列
                        // （末点无对应控制量）
                        if (state.size() >= 7) {
                            tp.setA(state(5));
                            tp.setDeltaDot(state(6));
                        } else if (i < step_num) {
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
            if (max_collision > kMaxAcceptableCollisionDepth) {
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

NMPCConfig PostProcessor::applyRetryConfig(
    const NMPCConfig& base_config, const AdaptiveRetryConfig& retry_config,
    int retry_idx) {
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

BicycleKinematicsConfig PostProcessor::DeriveKinematicsConfig(
    const VehicleParams& vehicle_params, double max_velocity) {
    BicycleKinematicsConfig config;
    config.wheelbase = vehicle_params.wheelbase;
    config.max_velocity = max_velocity;
    // 加速度上限取双向较小值，保证正向加速与倒车减速都不越界
    config.max_acceleration =
        std::min(vehicle_params.max_accel, std::abs(vehicle_params.max_decel));
    config.max_steer_angle = vehicle_params.max_steer_angle;
    config.max_steer_rate = vehicle_params.max_steer_rate;
    return config;
}

PostProcessorResult PostProcessor::optimizeAlm(
    const Path& init_path, const AlmConfig& alm_config) const {
    PostProcessorResult result;
    const auto t_start = std::chrono::steady_clock::now();
    // 统一收尾：填充耗时与结果统计后返回（失败分支 optimized_path 为空）
    const auto finish = [&result, t_start]() {
        result.total_time_ms = std::chrono::duration<double, std::milli>(
                                   std::chrono::steady_clock::now() - t_start)
                                   .count();
        // 物理方向段数（v 变号）为默认口径——多项式段内过冲按实际换挡
        // 如实计入；轨迹缺失（失败分支）时回退为 Path 机动段标签数
        result.final_maneuvers =
            result.alm_traj.empty()
                ? static_cast<int>(result.optimized_path.numManeuvers())
                : result.alm_traj.countDirectionRuns();
        result.final_length = result.optimized_path.length();
        return result;
    };
    try {
        // 空路径无法提取起点锚点，直接显式失败
        if (init_path.empty()) {
            result.message = "ALM input path is empty";
            return finish();
        }
        // 世界坐标还原积分的锚点：初始路径首点
        const Eigen::Vector2d start_position{init_path.front().x,
                                             init_path.front().y};
        // 运动学配置由车辆物理参数派生，全链路（预处理/主求解/融化采样）
        // 共用同一份，避免跨阶段数值不自洽
        const auto kinematics_config =
            DeriveKinematicsConfig(vehicle_params_, alm_config.max_velocity);
        const BicycleKinematicsExtractor kinematics(kinematics_config);
        // 1) 前端解析与分段：换挡打断 + 空间等距降采样
        const AlmManeuverSegmenter segmenter(alm_config.segmenter);
        const auto estimates = segmenter.segment(init_path);
        // 2) 预处理粗优化：把初值拉近前端路径并满足运动学约束
        const AlmPreprocessor preprocessor(alm_config.preprocessor,
                                           kinematics_config);
        const auto pre_result =
            preprocessor.preprocess(estimates, start_position);
        if (!pre_result.success) {
            result.message = "ALM preprocessing failed (max endpoint error " +
                             std::to_string(pre_result.max_endpoint_error) +
                             " m)";
            return finish();
        }
        // 预处理粗优化轨迹同步离散化，作为"优化前/优化后"对比的优化前基线：
        // 与最终输出共用同一套离散化工具、同一采样密度与同一运动学提取器，
        // 保证两条曲线只相差"主优化 + 机动融化"这一步
        result.alm_preprocessed_traj = FlattenManeuvers(SampleMincoTrajectory(
            pre_result.trajectory, estimates, start_position, kinematics,
            alm_config.melter.samples_per_segment));
        // 3) PHR-ALM 主优化：内层 L-BFGS + 外层乘子/惩罚权重更新
        const AlmSolver solver(alm_config.solver, kinematics_config,
                               alm_config.esdf_penalty);
        const auto solve_result = solver.solve(
            estimates, pre_result, start_position, esdf_map_, footprint_model_);
        if (solve_result.trajectory.numSegments() == 0) {
            // 内层首轮即失败或轨迹重建失败：无任何可用轨迹，显式失败
            result.message = "ALM solve failed: no valid trajectory produced";
            return finish();
        }
        // 4) 机动融化与拓扑修剪：剔除废段、同向合并，产出采样 Path
        const AlmManeuverMelter melter(alm_config.melter);
        const auto melt_result = melter.meltAndPrune(
            solve_result.trajectory, estimates, start_position, kinematics);
        Path optimized = melt_result.path;
        if (optimized.empty()) {
            result.message = "ALM melt produced empty path";
            return finish();
        }
        // 4.5) 停驻窗口"停-打轮-走"合法化改写：净 Δθ 很小的换挡停驻窗口
        // （θ̇≠0 且 δ 翻转的伪影）改写为 v=0、θ 冻结、δ 按 ≤δ̇_max 有界
        // 过渡；净 Δθ 超阈值的真实 pivot 窗口保持原样（阿克曼车辆无原地
        // 转向能力，合法执行需多点掉头，登记为已知边界）
        AlmSteerPaddingConfig padding_config;
        padding_config.max_steer_angle = vehicle_params_.max_steer_angle;
        padding_config.max_steer_rate = vehicle_params_.max_steer_rate;
        const auto padding_stats =
            ApplySteerPadding(optimized.getManeuvers(), padding_config);
        if (padding_stats.windows_legalized > 0 ||
            padding_stats.windows_skipped > 0) {
            LOG_FMT_INFO("ALM steer padding: legalized={}, skipped={}",
                         padding_stats.windows_legalized,
                         padding_stats.windows_skipped);
        }
        // 改写后重算曲率（停驻窗口 θ 已被冻结，原估计值已过期）
        optimized.finalize();
        // 5) 质量门：无奇异（全部采样点有限）→ 碰撞深度 → 终点精度
        if (!IsPathFinite(optimized)) {
            result.message = "ALM output rejected: non-finite samples";
            return finish();
        }
        const double max_collision =
            ComputeMaxCollisionDepth(optimized, esdf_map_, footprint_model_);
        if (max_collision > kMaxAcceptableCollisionDepth) {
            LOG_FMT_WARN(
                "ALM collision depth {:.4f}m exceeds threshold, rejecting",
                max_collision);
            result.message = "ALM collision depth " +
                             std::to_string(max_collision) +
                             " m exceeds threshold, rejecting";
            return finish();
        }
        // 终点精度以采样后路径末点相对初始路径末点（即停车目标位姿）度量；
        // 阈值复用主求解器的双指标收敛判据，与 ALM 收敛定义保持一致
        const double terminal_pos_err =
            std::hypot(optimized.back().x - init_path.back().x,
                       optimized.back().y - init_path.back().y);
        const double terminal_head_err_deg =
            std::abs(NormalizeAngle(optimized.back().theta -
                                    init_path.back().theta)) *
            RAD2DEG;
        if (terminal_pos_err > alm_config.solver.terminal_position_tolerance ||
            terminal_head_err_deg >
                alm_config.solver.terminal_heading_tolerance_deg) {
            result.message = "ALM terminal error " +
                             std::to_string(terminal_pos_err) + " m / " +
                             std::to_string(terminal_head_err_deg) +
                             " deg exceeds tolerance, rejecting";
            return finish();
        }
        // 填充 ALM 轨迹视图（与输出 Path 同序采样点，携带 v/a/delta/delta_dot，
        // 未携带时间戳），供可视化与下游消费
        result.alm_traj = FlattenManeuvers(optimized.getManeuvers());
        result.optimized_path = std::move(optimized);
        result.success = true;
        result.message =
            solve_result.status == AlmSolverStatus::CONVERGED
                ? "ALM converged"
                : "ALM did not fully converge after " +
                      std::to_string(solve_result.outer_iterations) +
                      " outer iterations, using last iterate (terminal "
                      "error " +
                      std::to_string(terminal_pos_err) + " m / " +
                      std::to_string(terminal_head_err_deg) + " deg)";
    } catch (const std::exception& e) {
        LOG_FMT_WARN("optimizeAlm exception: {}", e.what());
        result.message = std::string("ALM attempt failed: ") + e.what();
    }
    return finish();
}
}  // namespace apa_post_processor
