#include "post_processor.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <optional>
#include <stdexcept>
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
// 按节按键覆盖：仅显式出现的字段被写入（缺失保持原值）
template <typename T>
void LoadJsonFieldIfPresent(const nlohmann::json& section, const char* key,
                            T* out) {
    if (section.contains(key)) {
        *out = section[key].get<T>();
    }
}
// 路径段曲率 κ = wrap(Δθ)/ds 的最大绝对值（跳过零位移段）——与参考
// 构建器 δ 反解同一口径，仅供曲率投影的日志诊断
double MaxSegmentKappaForLog(const Path& path) {
    std::vector<Pose> points;
    points.reserve(path.size());
    path.forEach([&points](const TrajectoryPoint& point) {
        points.emplace_back(point.x, point.y, point.theta);
    });
    double max_kappa = 0.0;
    for (std::size_t i = 1; i < points.size(); ++i) {
        const double ds = std::hypot(points[i].x - points[i - 1].x,
                                     points[i].y - points[i - 1].y);
        if (ds <= 1e-9) {
            continue;
        }
        max_kappa = std::max(
            max_kappa,
            std::abs(WrapAngle(points[i].theta - points[i - 1].theta)) / ds);
    }
    return max_kappa;
}
// 取 JSON 节：缺失时返回空对象（后续 contains 恒 false，不产生任何覆盖）
const nlohmann::json& JsonSectionOrEmpty(const nlohmann::json& details,
                                         const char* name) {
    static const nlohmann::json kEmpty = nlohmann::json::object();
    const auto it = details.find(name);
    if (it == details.end()) {
        return kEmpty;
    }
    return *it;
}
// DDP 后处理状态码的可读名称（失败诊断消息用）
const char* DdpPostStageStatusName(DdpPostStageStatus status) {
    switch (status) {
        case DdpPostStageStatus::SUCCESS:
            return "SUCCESS";
        case DdpPostStageStatus::SUCCESS_STAGE_ONE_ONLY:
            return "SUCCESS_STAGE_ONE_ONLY";
        case DdpPostStageStatus::STAGE_ONE_NOT_CONVERGED:
            return "STAGE_ONE_NOT_CONVERGED";
        case DdpPostStageStatus::PIVOT_DETECTED:
            return "PIVOT_DETECTED";
        case DdpPostStageStatus::PRUNED_PATH_DEGENERATE:
            return "PRUNED_PATH_DEGENERATE";
        case DdpPostStageStatus::STAGE_TWO_NOT_CONVERGED:
            return "STAGE_TWO_NOT_CONVERGED";
        case DdpPostStageStatus::VALIDATION_FAILED:
            return "VALIDATION_FAILED";
    }
    return "UNKNOWN";
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

void LoadDdpConfigOverrides(const nlohmann::json& details, DdpConfig* config) {
    if (config == nullptr) {
        throw std::invalid_argument(
            "LoadDdpConfigOverrides received null config!!!");
    }
    // 参考构建节：状态幅值边界（v_max/a_max/delta_max/omega_max）的唯一
    // 权威来源，加载完成后统一同步进 cost/post_stage 的同源字段
    const auto& reference = JsonSectionOrEmpty(details, "reference");
    LoadJsonFieldIfPresent(reference, "sample_dist",
                           &config->reference.sample_dist);
    LoadJsonFieldIfPresent(reference, "dt", &config->reference.dt);
    LoadJsonFieldIfPresent(reference, "shooting_interval",
                           &config->reference.shooting_interval);
    LoadJsonFieldIfPresent(reference, "v_max", &config->reference.v_max);
    LoadJsonFieldIfPresent(reference, "a_max", &config->reference.a_max);
    LoadJsonFieldIfPresent(reference, "delta_max",
                           &config->reference.delta_max);
    LoadJsonFieldIfPresent(reference, "omega_max",
                           &config->reference.omega_max);
    // 求解编排节（阶段二门控调度参数）
    const auto& solver = JsonSectionOrEmpty(details, "solver");
    LoadJsonFieldIfPresent(solver, "stage_two_max_outer_iterations",
                           &config->solver.stage_two_max_outer_iterations);
    LoadJsonFieldIfPresent(solver, "gating_mu_initial",
                           &config->solver.gating_mu_initial);
    LoadJsonFieldIfPresent(solver, "seed_mu_cap_ratio",
                           &config->solver.seed_mu_cap_ratio);
    LoadJsonFieldIfPresent(solver, "gating_mu_max",
                           &config->solver.gating_mu_max);
    LoadJsonFieldIfPresent(solver, "gating_tol", &config->solver.gating_tol);
    // 内层 MS-iLQR 节：steer_accel_max 是 eta_max 的唯一权威来源
    const auto& inner = JsonSectionOrEmpty(solver, "inner");
    LoadJsonFieldIfPresent(inner, "jerk_max", &config->solver.inner.jerk_max);
    LoadJsonFieldIfPresent(inner, "steer_accel_max",
                           &config->solver.inner.steer_accel_max);
    LoadJsonFieldIfPresent(inner, "max_iterations",
                           &config->solver.inner.max_iterations);
    LoadJsonFieldIfPresent(inner, "cost_change_tol",
                           &config->solver.inner.cost_change_tol);
    LoadJsonFieldIfPresent(inner, "gradient_tol",
                           &config->solver.inner.gradient_tol);
    LoadJsonFieldIfPresent(inner, "reg_initial",
                           &config->solver.inner.reg_initial);
    LoadJsonFieldIfPresent(inner, "reg_min", &config->solver.inner.reg_min);
    LoadJsonFieldIfPresent(inner, "reg_max", &config->solver.inner.reg_max);
    LoadJsonFieldIfPresent(inner, "reg_increase",
                           &config->solver.inner.reg_increase);
    LoadJsonFieldIfPresent(inner, "reg_decrease",
                           &config->solver.inner.reg_decrease);
    LoadJsonFieldIfPresent(inner, "armijo_gamma",
                           &config->solver.inner.armijo_gamma);
    LoadJsonFieldIfPresent(inner, "backtrack_beta",
                           &config->solver.inner.backtrack_beta);
    LoadJsonFieldIfPresent(inner, "max_backtracks",
                           &config->solver.inner.max_backtracks);
    LoadJsonFieldIfPresent(inner, "merit_mu0", &config->solver.inner.merit_mu0);
    LoadJsonFieldIfPresent(inner, "merit_rho", &config->solver.inner.merit_rho);
    LoadJsonFieldIfPresent(inner, "merit_kappa_d",
                           &config->solver.inner.merit_kappa_d);
    LoadJsonFieldIfPresent(inner, "merit_mu_max",
                           &config->solver.inner.merit_mu_max);
    LoadJsonFieldIfPresent(inner, "inter_segment_weight",
                           &config->solver.inner.inter_segment_weight);
    LoadJsonFieldIfPresent(inner, "domain_guard_margin",
                           &config->solver.inner.domain_guard_margin);
    // 外层 AL 节
    const auto& outer = JsonSectionOrEmpty(solver, "outer");
    LoadJsonFieldIfPresent(outer, "max_outer_iterations",
                           &config->solver.outer.max_outer_iterations);
    LoadJsonFieldIfPresent(outer, "terminal_position_tol",
                           &config->solver.outer.terminal_position_tol);
    LoadJsonFieldIfPresent(outer, "terminal_heading_tol_deg",
                           &config->solver.outer.terminal_heading_tol_deg);
    LoadJsonFieldIfPresent(outer, "inequality_tol",
                           &config->solver.outer.inequality_tol);
    LoadJsonFieldIfPresent(outer, "defect_tol",
                           &config->solver.outer.defect_tol);
    LoadJsonFieldIfPresent(outer, "mu_min", &config->solver.outer.mu_min);
    LoadJsonFieldIfPresent(outer, "mu_max", &config->solver.outer.mu_max);
    LoadJsonFieldIfPresent(outer, "first_round_mu",
                           &config->solver.outer.first_round_mu);
    LoadJsonFieldIfPresent(outer, "amplitude_mu_initial",
                           &config->solver.outer.amplitude_mu_initial);
    LoadJsonFieldIfPresent(outer, "epsilon_mu",
                           &config->solver.outer.epsilon_mu);
    LoadJsonFieldIfPresent(outer, "mu_gate_kappa",
                           &config->solver.outer.mu_gate_kappa);
    LoadJsonFieldIfPresent(outer, "mu_growth_factor",
                           &config->solver.outer.mu_growth_factor);
    LoadJsonFieldIfPresent(outer, "amplitude_mu_max",
                           &config->solver.outer.amplitude_mu_max);
    LoadJsonFieldIfPresent(outer, "anneal_gamma",
                           &config->solver.outer.anneal_gamma);
    LoadJsonFieldIfPresent(outer, "anneal_hold_rounds",
                           &config->solver.outer.anneal_hold_rounds);
    LoadJsonFieldIfPresent(outer, "anneal_freeze_length_growth",
                           &config->solver.outer.anneal_freeze_length_growth);
    LoadJsonFieldIfPresent(
        outer, "anneal_freeze_lateral_deviation",
        &config->solver.outer.anneal_freeze_lateral_deviation);
    LoadJsonFieldIfPresent(outer, "anneal_freeze_defect",
                           &config->solver.outer.anneal_freeze_defect);
    LoadJsonFieldIfPresent(
        outer, "anneal_freeze_ref_length_ratio",
        &config->solver.outer.anneal_freeze_ref_length_ratio);
    LoadJsonFieldIfPresent(outer, "shift_beta_initial",
                           &config->solver.outer.shift_beta_initial);
    LoadJsonFieldIfPresent(outer, "shift_beta_final",
                           &config->solver.outer.shift_beta_final);
    LoadJsonFieldIfPresent(outer, "shift_beta_gamma",
                           &config->solver.outer.shift_beta_gamma);
    LoadJsonFieldIfPresent(outer, "melt_crit_threshold",
                           &config->solver.outer.melt_crit_threshold);
    LoadJsonFieldIfPresent(outer, "candidate_anneal_gamma",
                           &config->solver.outer.candidate_anneal_gamma);
    // 代价节：不接收幅值边界键（由 reference 节统一供给）
    const auto& cost = JsonSectionOrEmpty(solver, "cost");
    LoadJsonFieldIfPresent(cost, "weight_jerk",
                           &config->solver.cost.weight_jerk);
    LoadJsonFieldIfPresent(cost, "weight_steer_accel",
                           &config->solver.cost.weight_steer_accel);
    LoadJsonFieldIfPresent(cost, "weight_ref_base",
                           &config->solver.cost.weight_ref_base);
    LoadJsonFieldIfPresent(cost, "weight_theta",
                           &config->solver.cost.weight_theta);
    LoadJsonFieldIfPresent(cost, "weight_shift",
                           &config->solver.cost.weight_shift);
    LoadJsonFieldIfPresent(cost, "shift_beta", &config->solver.cost.shift_beta);
    LoadJsonFieldIfPresent(cost, "weight_curvature",
                           &config->solver.cost.weight_curvature);
    LoadJsonFieldIfPresent(cost, "weight_velocity",
                           &config->solver.cost.weight_velocity);
    LoadJsonFieldIfPresent(cost, "weight_delta_guard",
                           &config->solver.cost.weight_delta_guard);
    LoadJsonFieldIfPresent(cost, "delta_guard",
                           &config->solver.cost.delta_guard);
    // ESDF 双 margin 惩罚节
    const auto& esdf = JsonSectionOrEmpty(details, "esdf");
    LoadJsonFieldIfPresent(esdf, "margin_safe", &config->esdf.margin_safe);
    LoadJsonFieldIfPresent(esdf, "margin_comf", &config->esdf.margin_comf);
    LoadJsonFieldIfPresent(esdf, "weight_safe", &config->esdf.weight_safe);
    LoadJsonFieldIfPresent(esdf, "weight_comf", &config->esdf.weight_comf);
    LoadJsonFieldIfPresent(esdf, "stride", &config->esdf.stride);
    // 阶段二 ESDF 独立标定节（L6.3a，缺席 = 关闭、阶段二与阶段一共用同一
    // 惩罚）：出现时以阶段一 esdf 节为底播种、仅覆盖显式列出的字段——
    // 阶段二已有合法热启动解，可以且应当比阶段一更保守（对拉假设见
    // spec.md L6.3）
    if (details.contains("esdf_stage_two")) {
        config->esdf_stage_two = config->esdf;
        const auto& esdf_s2 = JsonSectionOrEmpty(details, "esdf_stage_two");
        LoadJsonFieldIfPresent(esdf_s2, "margin_safe",
                               &config->esdf_stage_two.margin_safe);
        LoadJsonFieldIfPresent(esdf_s2, "margin_comf",
                               &config->esdf_stage_two.margin_comf);
        LoadJsonFieldIfPresent(esdf_s2, "weight_safe",
                               &config->esdf_stage_two.weight_safe);
        LoadJsonFieldIfPresent(esdf_s2, "weight_comf",
                               &config->esdf_stage_two.weight_comf);
        LoadJsonFieldIfPresent(esdf_s2, "stride",
                               &config->esdf_stage_two.stride);
        config->esdf_stage_two_enabled = true;
    }
    // 后处理节：不接收 omega_max/eta_max 键（由 reference/inner 节统一
    // 供给）；cleanup/validation 两个嵌套配置不在 JSON 映射范围内
    const auto& post_stage = JsonSectionOrEmpty(details, "post_stage");
    LoadJsonFieldIfPresent(post_stage, "epsilon_v",
                           &config->post_stage.epsilon_v);
    LoadJsonFieldIfPresent(post_stage, "v_dwell", &config->post_stage.v_dwell);
    LoadJsonFieldIfPresent(post_stage, "shift_delay",
                           &config->post_stage.shift_delay);
    LoadJsonFieldIfPresent(post_stage, "kappa_pad",
                           &config->post_stage.kappa_pad);
    LoadJsonFieldIfPresent(post_stage, "seam_speed_tol",
                           &config->post_stage.seam_speed_tol);
    LoadJsonFieldIfPresent(post_stage, "dwell_omega_tol",
                           &config->post_stage.dwell_omega_tol);
    LoadJsonFieldIfPresent(post_stage, "amplitude_check_tol",
                           &config->post_stage.amplitude_check_tol);
    LoadJsonFieldIfPresent(post_stage, "amplitude_check_rel_tol",
                           &config->post_stage.amplitude_check_rel_tol);
    LoadJsonFieldIfPresent(post_stage, "control_overshoot_tol",
                           &config->post_stage.control_overshoot_tol);
    LoadJsonFieldIfPresent(post_stage, "stage_two_min_tracking_weight",
                           &config->post_stage.stage_two_min_tracking_weight);
    // cusp 几何预剪枝节（参考构建前的冗余折返剔除，默认关闭）
    const auto& cusp_prune = JsonSectionOrEmpty(details, "cusp_prune");
    LoadJsonFieldIfPresent(cusp_prune, "max_prune_arc",
                           &config->cusp_prune.max_prune_arc);
    LoadJsonFieldIfPresent(cusp_prune, "overlap_ratio",
                           &config->cusp_prune.overlap_ratio);
    LoadJsonFieldIfPresent(cusp_prune, "collision_margin",
                           &config->cusp_prune.collision_margin);
    // 参考保形曲率投影节（参考构建前的超限弧段压回，默认关闭）
    const auto& curvature_projection =
        JsonSectionOrEmpty(details, "curvature_projection");
    LoadJsonFieldIfPresent(curvature_projection, "cap_ratio",
                           &config->curvature_projection.cap_ratio);
    // 融化开/关双候选择优（顶层字段，默认 false = 关闭）
    LoadJsonFieldIfPresent(details, "dual_candidate_select",
                           &config->dual_candidate_select);
    // margin 延续救援（顶层字段，默认 0 = 关闭）
    LoadJsonFieldIfPresent(details, "rescue_margin_safe",
                           &config->rescue_margin_safe);
    // 参考重锚触发阈值（顶层字段，默认 0 = 关闭）
    LoadJsonFieldIfPresent(details, "reanchor_intrusion_threshold",
                           &config->reanchor_intrusion_threshold);
    // 幅值边界由权威来源同步进全部消费方（JSON 层单一来源的最终兑现）
    config->synchronizeAmplitudeBounds();
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

bool PostProcessor::PreferControlCandidate(const PostProcessorResult& melt,
                                           const PostProcessorResult& control) {
    // 成功优先：融化候选失败时选对照（哪怕对照也失败——其诊断不更差）
    if (!melt.success) {
        return true;
    }
    if (!control.success) {
        return false;
    }
    // maneuver 数少优先（本 Milestone 的主目标维度）
    if (control.final_maneuvers != melt.final_maneuvers) {
        return control.final_maneuvers < melt.final_maneuvers;
    }
    // 长度短优先；完全持平保持融化候选（现状语义）
    return control.final_length < melt.final_length;
}

PostProcessorResult PostProcessor::optimizeDdp(
    const Path& init_path, const DdpConfig& ddp_config) const {
    // 默认单遍（现状语义）；启用双候选后同一输入跑两遍完整链路择优
    if (!ddp_config.dual_candidate_select) {
        return optimizeDdpSinglePass(init_path, ddp_config);
    }
    // 候选一：配置原样（融化开）；候选二：退火率 ≈1（融化机制实际关闭，
    // 与调参矩阵的 nomelt_control 同一语义）
    PostProcessorResult melt = optimizeDdpSinglePass(init_path, ddp_config);
    DdpConfig control_config = ddp_config;
    control_config.solver.outer.anneal_gamma = 0.999;
    PostProcessorResult control =
        optimizeDdpSinglePass(init_path, control_config);
    const bool pick_control = PreferControlCandidate(melt, control);
    LOG_FMT_INFO(
        "DDP dual candidate: melt(success={}, maneuvers={}, length={:.3f}) "
        "vs control(success={}, maneuvers={}, length={:.3f}) -> pick {}",
        melt.success, melt.final_maneuvers, melt.final_length, control.success,
        control.final_maneuvers, control.final_length,
        pick_control ? "control" : "melt");
    // 耗时如实合并：返回结果是双遍编排的产物
    PostProcessorResult& chosen = pick_control ? control : melt;
    chosen.total_time_ms = melt.total_time_ms + control.total_time_ms;
    return chosen;
}

PostProcessorResult PostProcessor::optimizeDdpSinglePass(
    const Path& init_path, const DdpConfig& ddp_config) const {
    PostProcessorResult result;
    const auto t_start = std::chrono::steady_clock::now();
    // 统一收尾：填充耗时与结果统计后返回（失败分支 optimized_path 为空）
    const auto finish = [&result, t_start]() {
        result.total_time_ms = std::chrono::duration<double, std::milli>(
                                   std::chrono::steady_clock::now() - t_start)
                                   .count();
        // 物理方向段数（v 变号）为默认口径；轨迹缺失（失败/回退分支）时
        // 回退为 Path 机动段标签数
        result.final_maneuvers =
            result.ddp_traj.empty()
                ? static_cast<int>(result.optimized_path.numManeuvers())
                : result.ddp_traj.countDirectionRuns();
        // 失败/回退分支 optimized_path 为空：显式守卫取 0，不依赖
        // Path::length() 对空路径行为的实现细节
        result.final_length = result.optimized_path.empty()
                                  ? 0.0
                                  : result.optimized_path.length();
        return result;
    };
    try {
        // 空路径无法构建参考，直接显式失败
        if (init_path.empty()) {
            result.message = "DDP input path is empty";
            return finish();
        }
        // 局部配置副本：先按车辆物理参数收紧幅值边界（只准收紧不准放宽，
        // 并注入曲率正则的轴距系数）、再同步一次（调用方可能绕过
        // synchronizeAmplitudeBounds 直接改写字段），不触碰调用方配置
        DdpConfig config = ddp_config;
        config.clampToVehicleParams(vehicle_params_);
        config.synchronizeAmplitudeBounds();
        // L8.5：越界 ESDF 查询计数复位（逐条告警已移除——恢复场使小幅
        // 越界成为合法探测；计数在本次求解结束后单次汇总）
        esdf_map_.resetOutOfMapQueryCount();
        // 0) cusp 几何预剪枝（默认关闭）：剔除冗余折返段并安全重连，
        // 让求解器从更少的 cusp 出发；ESDF 检查不通过的剔除在剪枝函数
        // 内部已逐个回滚，此处拿到的剪枝结果保证同伦类安全。终点位姿
        // 由末段保护保证不变；原始 A* 路径仍是回退出口的唯一兜底
        Path solver_input = init_path;
        if (config.cusp_prune.max_prune_arc > 0.0) {
            solver_input = PruneRedundantCusps(
                init_path, esdf_map_, footprint_model_, config.cusp_prune);
            LOG_FMT_INFO(
                "DDP cusp prune: maneuvers {} -> {}, length {:.3f} -> {:.3f}",
                init_path.numManeuvers(), solver_input.numManeuvers(),
                init_path.length(), solver_input.length());
        }
        // 0.5) 参考保形曲率投影（默认关闭）：把隐含曲率超车辆上限的弧段
        // 压回可行域（θ 差分口径钳制 + maneuver 内航向守恒摊派），让
        // 初值 δ 不再越限、AL 幅值约束不再从首轮即激活。只改 θ 不改
        // 位置——端点/换挡点/同伦类自动保持；整段贴限无可摊容量的
        // maneuver 在投影函数内部已逐段回滚
        if (config.curvature_projection.cap_ratio > 0.0) {
            const Path projected = ProjectReferenceCurvature(
                solver_input, vehicle_params_.wheelbase,
                config.reference.delta_max, config.curvature_projection);
            LOG_FMT_INFO(
                "DDP curvature projection: max |kappa| {:.4f} -> "
                "{:.4f} (cap {:.4f})",
                MaxSegmentKappaForLog(solver_input),
                MaxSegmentKappaForLog(projected),
                config.curvature_projection.cap_ratio *
                    std::tan(config.reference.delta_max) /
                    vehicle_params_.wheelbase);
            solver_input = std::move(projected);
        }
        // 1) 前端参考构建：等弧长重采样 + 初值提取 + 打靶节点布设
        // （退化输入抛 std::invalid_argument，由外层 catch 转显式失败）
        const DdpReferenceBuilder reference_builder(config.reference,
                                                    vehicle_params_);
        DdpReference reference = reference_builder.build(solver_input);
        // 2) 求解组件装配：动力学/ESDF 惩罚/代价求值层与求解器共用同一份
        // 配置副本（幅值边界已经单一来源同步，乘子更新与终止判据不失配）
        const BicycleDynamics dynamics(vehicle_params_.wheelbase);
        const DdpEsdfConstraint esdf_constraint(esdf_map_, footprint_model_,
                                                config.esdf);
        const DdpCostEvaluator cost_evaluator(config.solver.cost,
                                              &esdf_constraint);
        ApaDdpSolver solver(config.solver, &dynamics, &cost_evaluator);
        // 阶段二 ESDF 独立标定（L6.3a，默认关闭）：启用时为阶段二门控精化
        // 单独装配惩罚/求值层与求解器；optional 保证条件构造且生命周期
        // 覆盖 post_stage.run 全程
        std::optional<DdpEsdfConstraint> stage_two_constraint;
        std::optional<DdpCostEvaluator> stage_two_evaluator;
        std::optional<ApaDdpSolver> stage_two_solver;
        if (config.esdf_stage_two_enabled) {
            stage_two_constraint.emplace(esdf_map_, footprint_model_,
                                         config.esdf_stage_two);
            stage_two_evaluator.emplace(config.solver.cost,
                                        &*stage_two_constraint);
            stage_two_solver.emplace(config.solver, &dynamics,
                                     &*stage_two_evaluator);
        }
        // 3) 阶段一全局软化求解（任何出口均带最后可用轨迹与结构化报告）
        ApaDdpStageOneResult stage_one = solver.solveStageOne(reference);
        // 3r) ESDF margin 延续救援（默认关闭）：默认 margin 下阶段一未收敛
        // 时，以外移的 ESDF 罚边界（rescue_margin_safe）冷启动重解一次——
        // 罚激活边界前移把求解拉出「δ 奇异区逃逸 → μ 螺旋」的失败盆地；
        // 收敛后再以默认 margin 热启动精化一次（延续法的收口步骤），
        // 精化收敛则取之、否则取救援解本身。健康数据集默认路径已收敛，
        // 永不进入本分支——对它们零副作用
        if (stage_one.report.status != ApaDdpStatus::CONVERGED &&
            config.rescue_margin_safe > 0.0) {
            DdpEsdfConstraintConfig rescue_esdf = config.esdf;
            rescue_esdf.margin_safe = config.rescue_margin_safe;
            const DdpEsdfConstraint rescue_constraint(
                esdf_map_, footprint_model_, rescue_esdf);
            const DdpCostEvaluator rescue_evaluator(config.solver.cost,
                                                    &rescue_constraint);
            ApaDdpSolver rescue_solver(config.solver, &dynamics,
                                       &rescue_evaluator);
            auto rescue_one = rescue_solver.solveStageOne(reference);
            LOG_FMT_INFO(
                "DDP rescue stage one (margin_safe={:.3f}): status={}, "
                "outer={}, term_err={:.4f} m/{:.3f} deg, ineq={:.4f}, "
                "defect={:.2e}",
                config.rescue_margin_safe,
                static_cast<int>(rescue_one.report.status),
                rescue_one.report.outer_iterations,
                rescue_one.report.terminal_position_error,
                rescue_one.report.terminal_heading_error_deg,
                rescue_one.report.max_amplitude_violation,
                rescue_one.report.defect_norm_inf);
            if (rescue_one.report.status == ApaDdpStatus::CONVERGED) {
                // 默认 margin 热启动精化（延续收口）：救援解的宽松间隙
                // 平衡被拉回默认边界
                auto refined = solver.solveStageOne(
                    reference, rescue_one.states, rescue_one.controls);
                LOG_FMT_INFO(
                    "DDP refined stage one (default margin, warm): status={}, "
                    "outer={}, term_err={:.4f} m/{:.3f} deg, ineq={:.4f}, "
                    "defect={:.2e}",
                    static_cast<int>(refined.report.status),
                    refined.report.outer_iterations,
                    refined.report.terminal_position_error,
                    refined.report.terminal_heading_error_deg,
                    refined.report.max_amplitude_violation,
                    refined.report.defect_norm_inf);
                stage_one = refined.report.status == ApaDdpStatus::CONVERGED
                                ? std::move(refined)
                                : std::move(rescue_one);
            }
        }
        LOG_FMT_INFO(
            "DDP stage one: status={}, outer={}, inner_total={}, restarts={}, "
            "term_err={:.4f} m/{:.3f} deg, ineq={:.4f}, defect={:.2e}, "
            "mu_final={:.1f}",
            static_cast<int>(stage_one.report.status),
            stage_one.report.outer_iterations,
            stage_one.report.total_inner_iterations,
            stage_one.report.inner_restarts,
            stage_one.report.terminal_position_error,
            stage_one.report.terminal_heading_error_deg,
            stage_one.report.max_amplitude_violation,
            stage_one.report.defect_norm_inf, stage_one.report.mu_final);
        // 阶段一解的碰撞侵入量测（与 Trajectory::validate 同一口径：外圆
        // 半径 − ESDF 距离的全圆全点最大值）：阶段一的 ESDF 项是纯代价、
        // 不门控收敛，其平衡侵入会原样遗传给阶段二——终检碰撞超标时先
        // 据此区分"阶段一遗留"与"阶段二引入"，再决定调哪一侧参数
        const auto measure_stage_one_intrusion = [&]() {
            const double outer_radius = footprint_model_.getOuterRadius();
            const auto outer_circles =
                vehicle_circle_geometry::ExtractLocalCircleCenters(
                    footprint_model_, CircleType::OUTER);
            double intrusion = 0.0;
            for (const auto& state : stage_one.states) {
                const double cos_theta = std::cos(state(DDP_IDX_THETA));
                const double sin_theta = std::sin(state(DDP_IDX_THETA));
                for (const auto& local : outer_circles) {
                    const double wx = state(DDP_IDX_X) + local.x() * cos_theta -
                                      local.y() * sin_theta;
                    const double wy = state(DDP_IDX_Y) + local.x() * sin_theta +
                                      local.y() * cos_theta;
                    intrusion = std::max(
                        intrusion, outer_radius - esdf_map_.getDist(wx, wy));
                }
            }
            return intrusion;
        };
        if (stage_one.report.status == ApaDdpStatus::CONVERGED) {
            const double stage_one_intrusion = measure_stage_one_intrusion();
            LOG_FMT_INFO("DDP stage one collision intrusion: {:.4f} m",
                         stage_one_intrusion);
            // 参考重锚（默认关闭）：阶段一解侵入超阈时，把解重建为新参考
            // 并热启动重解一次——原参考超物理上限时跟踪项把解往不可达
            // 急弯上拉、ESDF 平衡残余落在侵入侧；重锚后参考已是「较可行」
            // 的上一轮解，跟踪压力解除。只重锚一轮（多轮实测可发散）；
            // 重锚后仍不收敛则保留原解（任何出口都不比原解更差）
            if (config.reanchor_intrusion_threshold > 0.0 &&
                stage_one_intrusion > config.reanchor_intrusion_threshold) {
                DdpPostStage reanchor_post_stage(config.post_stage,
                                                 &reference_builder, &solver,
                                                 vehicle_params_);
                const auto runs =
                    reanchor_post_stage.analyzeSignRuns(stage_one.states);
                auto maneuvers =
                    reanchor_post_stage.buildManeuvers(stage_one.states, runs);
                if (reanchor_post_stage.pruneManeuvers(&maneuvers)) {
                    const Path anchor_path = ReconstructPath(maneuvers);
                    // 退化防御：重锚路径无法构建参考（过短/单点）或重解
                    // 抛异常时，放弃重锚、保留原阶段一解（任何出口都不比
                    // 原解更差）
                    try {
                        DdpReference new_reference =
                            reference_builder.build(anchor_path);
                        DdpAlignedVec<DdpState> reanchor_warm_states;
                        DdpAlignedVec<DdpControl> reanchor_warm_controls;
                        reanchor_post_stage.buildStageTwoWarmStart(
                            anchor_path, new_reference, &reanchor_warm_states,
                            &reanchor_warm_controls);
                        auto reanchored = solver.solveStageOne(
                            new_reference, reanchor_warm_states,
                            reanchor_warm_controls);
                        LOG_FMT_INFO(
                            "DDP reanchored stage one: status={}, outer={}, "
                            "term_err={:.4f} m/{:.3f} deg, ineq={:.4f}, "
                            "defect={:.2e}",
                            static_cast<int>(reanchored.report.status),
                            reanchored.report.outer_iterations,
                            reanchored.report.terminal_position_error,
                            reanchored.report.terminal_heading_error_deg,
                            reanchored.report.max_amplitude_violation,
                            reanchored.report.defect_norm_inf);
                        if (reanchored.report.status ==
                            ApaDdpStatus::CONVERGED) {
                            stage_one = std::move(reanchored);
                            reference = std::move(new_reference);
                            LOG_FMT_INFO(
                                "DDP reanchored stage one collision "
                                "intrusion: {:.4f} m (before reanchor: "
                                "{:.4f} m)",
                                measure_stage_one_intrusion(),
                                stage_one_intrusion);
                        }
                    } catch (const std::exception& e) {
                        LOG_FMT_WARN(
                            "DDP reanchor skipped (anchor path degenerate "
                            "or rebuild failed): {}",
                            e.what());
                    }
                }
            }
        }
        // 未收敛时逐轮转储外层历史：收敛趋势（终点误差/违反度/缺陷/μ 调度
        // 与内层状态逐轮演变）是定位"哪一类判据卡住收敛"的唯一证据，
        // 仅终态量测不足以区分预算不足与结构性发散（成功路径不转储，
        // 避免正常生产日志膨胀）
        if (stage_one.report.status != ApaDdpStatus::CONVERGED) {
            for (const auto& round : stage_one.report.history) {
                LOG_FMT_WARN(
                    "DDP stage one round {}: w_ref={:.3f}, mu={:.1f}, "
                    "term_err={:.4f} m/{:.3f} deg, ineq={:.4f}, "
                    "defect={:.2e}, inner_status={}, inner_iter={}",
                    round.outer_index, round.tracking_weight, round.mu,
                    round.terminal_position_error,
                    round.terminal_heading_error_deg,
                    round.max_amplitude_violation, round.defect_norm_inf,
                    static_cast<int>(round.inner_status),
                    round.inner_iterations);
            }
            // 定位最大幅值违反的约束类型与节点：五种幅值约束（v²/a²/ω²
            // 平方形态 + δ 双侧线性形态）共用同一 max 量测，不展开分量
            // 就无法区分"哪个物理量、在哪个节点"顽固违反——这是调参时
            // 判断该收紧哪一侧调度的最小必要信息
            const auto& states = stage_one.states;
            const auto& cost = config.solver.cost;
            double worst_violation = 0.0;
            std::size_t worst_index = 0;
            std::string worst_name;
            double worst_state_value = 0.0;
            for (std::size_t k = 0; k < states.size(); ++k) {
                const auto& x = states[k];
                const std::array<std::pair<const char*, double>, 5> checks{{
                    {"v",
                     x(DDP_IDX_V) * x(DDP_IDX_V) - cost.v_max * cost.v_max},
                    {"a",
                     x(DDP_IDX_A) * x(DDP_IDX_A) - cost.a_max * cost.a_max},
                    {"omega", x(DDP_IDX_OMEGA) * x(DDP_IDX_OMEGA) -
                                  cost.omega_max * cost.omega_max},
                    {"delta+", x(DDP_IDX_DELTA) - cost.delta_max},
                    {"delta-", -x(DDP_IDX_DELTA) - cost.delta_max},
                }};
                for (const auto& [name, g] : checks) {
                    if (g > worst_violation) {
                        worst_violation = g;
                        worst_index = k;
                        worst_name = name;
                    }
                }
            }
            if (worst_violation > 0.0) {
                const auto& x = states[worst_index];
                worst_state_value =
                    worst_name[0] == 'v'
                        ? x(DDP_IDX_V)
                        : (worst_name[0] == 'a'
                               ? x(DDP_IDX_A)
                               : (worst_name[0] == 'o' ? x(DDP_IDX_OMEGA)
                                                       : x(DDP_IDX_DELTA)));
                LOG_FMT_WARN(
                    "DDP stage one worst amplitude: constraint={}, "
                    "node={}/{}, violation={:.4f}, state_value={:.4f}, "
                    "v={:.4f}, pose=({:.3f}, {:.3f}, {:.3f})",
                    worst_name, worst_index, states.size(), worst_violation,
                    worst_state_value, x(DDP_IDX_V), x(DDP_IDX_X), x(DDP_IDX_Y),
                    x(DDP_IDX_THETA));
            }
        }
        // 4) 后处理与阶段二门控精化（游程分析/修剪/重解/驻留/校验/回退六
        // 步全部由其内部编排），终点目标取初始路径末点位姿（停车目标）
        DdpPostStage post_stage(config.post_stage, &reference_builder, &solver,
                                vehicle_params_);
        if (stage_two_solver.has_value()) {
            post_stage.setStageTwoSolver(&*stage_two_solver);
        }
        const auto& goal_pt = init_path.back();
        const TrajectoryPoint goal(goal_pt.x, goal_pt.y, goal_pt.theta);
        auto post = post_stage.run(init_path, reference, stage_one, goal,
                                   esdf_map_, footprint_model_);
        LOG_FMT_INFO("DDP post stage: status={}, maneuvers {}->{}, seams={}",
                     static_cast<int>(post.status),
                     post.diagnostics.input_maneuver_count,
                     post.diagnostics.output_maneuver_count,
                     post.diagnostics.seams.size());
        // L8.4/L8.5 分项归因单次汇总：越界查询计数（恢复场覆盖的探测量）
        // 与定义域守卫拒绝数（被拦住的越界试探量）——两者合并区分
        // 「出界已消失 / 出界被拦住 / 出界仍在发生」
        LOG_FMT_INFO(
            "DDP domain diagnostics: out_of_map_queries={}, "
            "domain_guard_rejections={}",
            esdf_map_.outOfMapQueryCount(),
            stage_one.report.domain_guard_rejections);
        const bool post_succeeded =
            post.status == DdpPostStageStatus::SUCCESS ||
            post.status == DdpPostStageStatus::SUCCESS_STAGE_ONE_ONLY;
        if (!post_succeeded) {
            // 回退不产出半成品（optimized_path/ddp_traj 均为空），但必须
            // 带结构化诊断：失败阶段 + 失败项 + 量化值/阈值（+ 降级原因）
            result.message =
                std::string("DDP post stage ") +
                DdpPostStageStatusName(post.status) + ": " +
                post.diagnostics.failed_check +
                " measured=" + std::to_string(post.diagnostics.measured_value) +
                " threshold=" + std::to_string(post.diagnostics.threshold);
            if (!post.diagnostics.degraded_reason.empty()) {
                result.message +=
                    ", degraded_reason=" + post.diagnostics.degraded_reason;
            }
            result.message += ", fell back to original path";
            LOG_FMT_WARN("{}", result.message);
            // 阶段二已执行时补充其终态量测：联合终止判据四个分项与三类
            // 门控违反度——回退诊断必须足以定位"哪一类判据卡住了收敛"，
            // 否则调参只能盲扫（阶段二未执行到时该结构为空，跳过）
            if (post.stage_two.has_value()) {
                const auto& stage_two_report = post.stage_two->report;
                LOG_FMT_WARN(
                    "DDP stage two diagnostics: status={}, outer={}, "
                    "term_err={:.4f} m/{:.3f} deg, ineq={:.4f}, "
                    "defect={:.2e}, sign_viol={:.4f}, dwell_viol={:.4f}, "
                    "seam_speed={:.4f}, gating_ok={}",
                    static_cast<int>(stage_two_report.status),
                    stage_two_report.outer_iterations,
                    stage_two_report.terminal_position_error,
                    stage_two_report.terminal_heading_error_deg,
                    stage_two_report.max_amplitude_violation,
                    stage_two_report.defect_norm_inf,
                    post.stage_two->max_sign_violation,
                    post.stage_two->max_dwell_violation,
                    post.stage_two->max_seam_speed, post.stage_two->gating_ok);
                // 阶段二输出的碰撞侵入量测（驻留插入只做时间拉伸、不改
                // 空间剖面，与终检同一口径）：区分碰撞是阶段二门控牵引
                // 引入还是修剪参考几何引入
                const double outer_radius = footprint_model_.getOuterRadius();
                const auto outer_circles =
                    vehicle_circle_geometry::ExtractLocalCircleCenters(
                        footprint_model_, CircleType::OUTER);
                double stage_two_intrusion = 0.0;
                std::size_t worst_node = 0;
                double worst_wx = 0.0;
                double worst_wy = 0.0;
                for (std::size_t k = 0; k < post.stage_two->states.size();
                     ++k) {
                    const auto& state = post.stage_two->states[k];
                    const double cos_theta = std::cos(state(DDP_IDX_THETA));
                    const double sin_theta = std::sin(state(DDP_IDX_THETA));
                    for (const auto& local : outer_circles) {
                        const double wx = state(DDP_IDX_X) +
                                          local.x() * cos_theta -
                                          local.y() * sin_theta;
                        const double wy = state(DDP_IDX_Y) +
                                          local.x() * sin_theta +
                                          local.y() * cos_theta;
                        const double intrusion =
                            outer_radius - esdf_map_.getDist(wx, wy);
                        if (intrusion > stage_two_intrusion) {
                            stage_two_intrusion = intrusion;
                            worst_node = k;
                            worst_wx = wx;
                            worst_wy = wy;
                        }
                    }
                }
                LOG_FMT_WARN(
                    "DDP stage two collision intrusion: {:.4f} m "
                    "(node={}, circle_center=({:.3f}, {:.3f}))",
                    stage_two_intrusion, worst_node, worst_wx, worst_wy);
                // 未收敛时逐轮转储外层历史（理由同阶段一转储）：精化阶段
                // 的收敛趋势是区分"预算不足"与"门控/对偶调度失衡"的证据
                if (stage_two_report.status != ApaDdpStatus::CONVERGED) {
                    for (const auto& round : stage_two_report.history) {
                        LOG_FMT_WARN(
                            "DDP stage two round {}: w_ref={:.3f}, mu={:.1f}, "
                            "term_err={:.4f} m/{:.3f} deg, ineq={:.4f}, "
                            "defect={:.2e}, inner_status={}, inner_iter={}",
                            round.outer_index, round.tracking_weight, round.mu,
                            round.terminal_position_error,
                            round.terminal_heading_error_deg,
                            round.max_amplitude_violation,
                            round.defect_norm_inf,
                            static_cast<int>(round.inner_status),
                            round.inner_iterations);
                    }
                }
            }
            // 逐接缝驻留报告转储：窗口定宽/驻留完整性类失败的定位依赖
            // 接缝级量测（转向需求/计划驻留/窗内速度帽/窗端 ω），仅总量
            // 无法区分是哪个接缝、哪一侧窗口不足
            for (const auto& seam : post.diagnostics.seams) {
                LOG_FMT_WARN(
                    "DDP seam report: index={}, delta_delta={:.3f}, "
                    "t_resteer={:.2f}, t_dwell={:.2f}, dwell={:.2f}, "
                    "seam_speed={:.4f}, window_max_speed={:.4f}, "
                    "window_end_omega={:.4f}",
                    seam.seam_index, seam.delta_delta, seam.t_resteer,
                    seam.t_dwell, seam.dwell_duration, seam.seam_speed,
                    seam.window_max_speed, seam.window_end_omega);
            }
            return finish();
        }
        // 5) 成功：最终轨迹转 Path（几何口径），ddp_traj 承载含驻留与时间
        // 戳的完整轨迹（供可视化与下游消费）。降级输出（阶段一候选）的
        // 消息必须如实反映实际走的是哪一级，不得伪装成阶段二完全成功
        Path optimized;
        for (const auto& pt : post.trajectory.points()) {
            optimized.addPoint(pt);
        }
        optimized.finalize();
        result.ddp_traj = std::move(post.trajectory);
        result.optimized_path = std::move(optimized);
        result.success = true;
        if (post.status == DdpPostStageStatus::SUCCESS_STAGE_ONE_ONLY) {
            result.message = std::string(
                                 "DDP converged with stage-one candidate "
                                 "(stage-two unavailable: ") +
                             post.diagnostics.degraded_reason + ")";
            LOG_FMT_WARN("DDP post stage degraded output: {}", result.message);
        } else {
            result.message = "DDP converged";
        }
        // 质量指标随日志全量转储（记录不否决）：控制盒过冲/接缝与驻留
        // 完整性子项/maneuver 数不增/长度比，供方案比较与调参消费
        for (const auto& check : post.diagnostics.metric_checks) {
            LOG_FMT_INFO("DDP post metric: {}={:.4f} (threshold {:.4f}, {})",
                         check.name, check.measured, check.threshold,
                         check.passed ? "pass" : "EXCEEDED");
        }
    } catch (const std::exception& e) {
        LOG_FMT_WARN("optimizeDdp exception: {}", e.what());
        result.message = std::string("DDP attempt failed: ") + e.what();
    }
    return finish();
}
}  // namespace apa_post_processor
