#include "post_processor.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <string>

#include "../preprocessing/preprocessing_pipeline.h"
#include "../util/topology_cleaner.h"
#include "MINCO/minco_esdf_penalty.h"
#include "MINCO/minco_maneuver_melter.h"
#include "MINCO/minco_maneuver_segmenter.h"
#include "MINCO/minco_preprocessor.h"
#include "MINCO/minco_solver.h"
#include "MINCO/minco_steer_padding.h"
#include "MINCO/minco_trajectory_sampler.h"
#include "iLQR/ilqr_solver.h"
#include "NMPC/vehicle_circle_geometry.h"
#include "collision_check.h"

namespace apa_post_processor {
namespace {
// 优化结果允许的最大碰撞深度 (m)：NMPC 与 MINCO 两条路径共用同一质量门
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
// 按弧长比例线性插值两点间的位姿与运动学量（t ∈ [0,1]）；航向角走最短
// 角差。κ 不在此处插值，由后续 finalize() 统一按 Δθ/Δs 重算
TrajectoryPoint InterpolatePoint(const TrajectoryPoint& a,
                                 const TrajectoryPoint& b, double t) {
    TrajectoryPoint p(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y),
                      a.theta + t * NormalizeAngle(b.theta - a.theta));
    if (a.hasV() && b.hasV()) {
        p.setV(a.getV() + t * (b.getV() - a.getV()));
    }
    if (a.hasA() && b.hasA()) {
        p.setA(a.getA() + t * (b.getA() - a.getA()));
    }
    if (a.hasDelta() && b.hasDelta()) {
        p.setDelta(a.getDelta() + t * (b.getDelta() - a.getDelta()));
    }
    if (a.hasDeltaDot() && b.hasDeltaDot()) {
        p.setDeltaDot(a.getDeltaDot() + t * (b.getDeltaDot() - a.getDeltaDot()));
    }
    if (a.hasT() && b.hasT()) {
        p.setT(a.getT() + t * (b.getT() - a.getT()));
    }
    return p;
}
// 输出等距重采样：MINCO 轨迹按时间等分采样，点距随速度变化（低速蠕动区
// 1~2cm、高速区可达 10cm）。沿弧长以 DELTA_DIST 为间隔对整条路径重采样：
// 过密区抽稀、过疏区插值，使输出点距统一到约 5cm（与前端路径一致）；段
// 首尾锚点始终保留。插值点运动学量按弧长比例线性插值
void ResampleToUniformSpacing(std::vector<Maneuver>* maneuvers) {
    if (maneuvers == nullptr) {
        return;
    }
    for (auto& m : *maneuvers) {
        auto& pts = m.points;
        if (pts.size() < 3) {
            continue;
        }
        std::vector<TrajectoryPoint> out;
        out.reserve(pts.size());
        out.push_back(pts.front());
        double carry = 0.0;             // 距上一个发射点的累计弧长
        double next_s = DELTA_DIST;     // 下一个发射点的目标弧长
        for (std::size_t i = 1; i < pts.size(); ++i) {
            const auto& prev = pts[i - 1];
            const auto& cur = pts[i];
            const double seg_len =
                std::hypot(cur.x - prev.x, cur.y - prev.y);
            if (seg_len < 1e-12) {
                continue;  // 重合点：跳过
            }
            const double seg_end = carry + seg_len;
            // 沿当前弦按 DELTA_DIST 步长发射插值点
            while (next_s <= seg_end + 1e-9) {
                const double t =
                    std::clamp((next_s - carry) / seg_len, 0.0, 1.0);
                out.push_back(InterpolatePoint(prev, cur, t));
                next_s += DELTA_DIST;
            }
            carry = seg_end;
        }
        // 段末锚点总是保留（换挡/终点锚点）
        if (std::hypot(pts.back().x - out.back().x,
                       pts.back().y - out.back().y) > 1e-9) {
            out.push_back(pts.back());
        }
        pts = std::move(out);
    }
}
// 按节按键覆盖：仅显式出现的字段被写入（缺失保持原值）
template <typename T>
void LoadJsonFieldIfPresent(const nlohmann::json& section, const char* key,
                            T* out) {
    if (section.contains(key)) {
        *out = section[key].get<T>();
    }
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
    // 先填充输出轨迹再统计（final_maneuvers 依赖轨迹内容）
    result.algorithm = "nmpc";
    result.optimized_trajectory = std::move(attempt.nmpc_traj);
    result.output_level = result.success ? OutputLevel::kFullSuccess
                                         : OutputLevel::kFallback;
    if (!attempt.preprocessed_traj.empty()) {
        result.intermediate_traces.emplace_back(
            "preprocessed", std::move(attempt.preprocessed_traj));
    }
    // 物理方向段数（v 变号）为默认口径；轨迹缺失（回退预处理路径）时
    // 回退为 Path 机动段标签数
    result.final_maneuvers =
        result.optimized_trajectory.empty()
            ? static_cast<int>(result.optimized_path.numManeuvers())
            : result.optimized_trajectory.countDirectionRuns();
    result.final_length = result.optimized_path.length();
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

void LoadiLQRConfigOverrides(const nlohmann::json& details, iLQRConfig* config) {
    if (config == nullptr) {
        throw std::invalid_argument(
            "LoadiLQRConfigOverrides received null config!!!");
    }
    // 参考构建节：状态幅值边界（v_max/a_max/delta_max/omega_max）的唯一
    // 权威来源，加载完成后统一同步进 cost/post_stage 的同源字段
    const auto& reference = JsonSectionOrEmpty(details, "reference");
    LoadJsonFieldIfPresent(reference, "sample_dist",
                           &config->reference_sample_dist);
    LoadJsonFieldIfPresent(reference, "dt", &config->reference_dt);
    LoadJsonFieldIfPresent(reference, "shooting_interval",
                           &config->reference_shooting_interval);
    LoadJsonFieldIfPresent(reference, "v_max", &config->reference_v_max);
    LoadJsonFieldIfPresent(reference, "a_max", &config->reference_a_max);
    LoadJsonFieldIfPresent(reference, "delta_max",
                           &config->reference_delta_max);
    LoadJsonFieldIfPresent(reference, "omega_max",
                           &config->reference_omega_max);
    // 求解编排节（阶段二门控调度参数）
    const auto& solver = JsonSectionOrEmpty(details, "solver");
    LoadJsonFieldIfPresent(solver, "stage_two_max_outer_iterations",
                           &config->stage_two_max_outer_iterations);
    LoadJsonFieldIfPresent(solver, "gating_mu_initial",
                           &config->gating_mu_initial);
    LoadJsonFieldIfPresent(solver, "gating_mu_max",
                           &config->gating_mu_max);
    LoadJsonFieldIfPresent(solver, "gating_tol", &config->gating_tol);
    // 内层 MS-iLQR 节：steer_accel_max 是 eta_max 的唯一权威来源
    const auto& inner = JsonSectionOrEmpty(solver, "inner");
    LoadJsonFieldIfPresent(inner, "jerk_max", &config->inner_jerk_max);
    LoadJsonFieldIfPresent(inner, "steer_accel_max",
                           &config->inner_steer_accel_max);
    LoadJsonFieldIfPresent(inner, "max_iterations",
                           &config->inner_max_iterations);
    LoadJsonFieldIfPresent(inner, "cost_change_tol",
                           &config->inner_cost_change_tol);
    LoadJsonFieldIfPresent(inner, "gradient_tol",
                           &config->inner_gradient_tol);
    LoadJsonFieldIfPresent(inner, "convergence_defect_tol",
                           &config->inner_convergence_defect_tol);
    LoadJsonFieldIfPresent(inner, "reg_initial",
                           &config->inner_reg_initial);
    LoadJsonFieldIfPresent(inner, "reg_min", &config->inner_reg_min);
    LoadJsonFieldIfPresent(inner, "reg_max", &config->inner_reg_max);
    LoadJsonFieldIfPresent(inner, "reg_increase",
                           &config->inner_reg_increase);
    LoadJsonFieldIfPresent(inner, "reg_decrease",
                           &config->inner_reg_decrease);
    LoadJsonFieldIfPresent(inner, "armijo_gamma",
                           &config->inner_armijo_gamma);
    LoadJsonFieldIfPresent(inner, "backtrack_beta",
                           &config->inner_backtrack_beta);
    LoadJsonFieldIfPresent(inner, "max_backtracks",
                           &config->inner_max_backtracks);
    LoadJsonFieldIfPresent(inner, "merit_mu0", &config->inner_merit_mu0);
    LoadJsonFieldIfPresent(inner, "merit_mu_max",
                           &config->inner_merit_mu_max);
    LoadJsonFieldIfPresent(inner, "merit_mu_al_ratio",
                           &config->inner_merit_mu_al_ratio);
    LoadJsonFieldIfPresent(inner, "domain_guard_margin",
                           &config->inner_domain_guard_margin);
    LoadJsonFieldIfPresent(inner, "use_virtual_control",
                           &config->inner_use_virtual_control);
    // 外层 AL 节
    const auto& outer = JsonSectionOrEmpty(solver, "outer");
    LoadJsonFieldIfPresent(outer, "max_outer_iterations",
                           &config->outer_max_outer_iterations);
    LoadJsonFieldIfPresent(outer, "terminal_position_tol",
                           &config->outer_terminal_position_tol);
    LoadJsonFieldIfPresent(outer, "terminal_heading_tol_deg",
                           &config->outer_terminal_heading_tol_deg);
    LoadJsonFieldIfPresent(outer, "inequality_tol",
                           &config->outer_inequality_tol);
    LoadJsonFieldIfPresent(outer, "defect_tol",
                           &config->outer_defect_tol);
    LoadJsonFieldIfPresent(outer, "mu_min", &config->outer_mu_min);
    LoadJsonFieldIfPresent(outer, "mu_max", &config->outer_mu_max);
    LoadJsonFieldIfPresent(outer, "first_round_mu",
                           &config->outer_first_round_mu);
    LoadJsonFieldIfPresent(outer, "amplitude_mu_initial",
                           &config->outer_amplitude_mu_initial);
    LoadJsonFieldIfPresent(outer, "epsilon_mu",
                           &config->outer_epsilon_mu);
    LoadJsonFieldIfPresent(outer, "mu_gate_kappa",
                           &config->outer_mu_gate_kappa);
    LoadJsonFieldIfPresent(outer, "mu_growth_factor",
                           &config->outer_mu_growth_factor);
    LoadJsonFieldIfPresent(outer, "anneal_gamma",
                           &config->outer_anneal_gamma);
    LoadJsonFieldIfPresent(
        outer, "amplitude_mu_per_element_stage_two",
        &config->outer_amplitude_mu_per_element_stage_two);
    LoadJsonFieldIfPresent(outer, "esdf_scale_growth",
                           &config->outer_esdf_scale_growth);
    LoadJsonFieldIfPresent(outer, "esdf_scale_max",
                           &config->outer_esdf_scale_max);
    // 代价节：不接收幅值边界键（由 reference 节统一供给）
    const auto& cost = JsonSectionOrEmpty(solver, "cost");
    LoadJsonFieldIfPresent(cost, "weight_jerk",
                           &config->cost_weight_jerk);
    LoadJsonFieldIfPresent(cost, "weight_steer_accel",
                           &config->cost_weight_steer_accel);
    LoadJsonFieldIfPresent(cost, "weight_ref_base",
                           &config->cost_weight_ref_base);
    LoadJsonFieldIfPresent(cost, "weight_theta",
                           &config->cost_weight_theta);
    // ESDF 双 margin 惩罚节
    const auto& esdf = JsonSectionOrEmpty(details, "esdf");
    LoadJsonFieldIfPresent(esdf, "margin_safe", &config->esdf_margin_safe);
    LoadJsonFieldIfPresent(esdf, "margin_comf", &config->esdf_margin_comf);
    LoadJsonFieldIfPresent(esdf, "weight_safe", &config->esdf_weight_safe);
    LoadJsonFieldIfPresent(esdf, "weight_comf", &config->esdf_weight_comf);
    LoadJsonFieldIfPresent(esdf, "stride", &config->esdf_stride);
    // 后处理节：不接收 omega_max/eta_max 键（由 reference/inner 节统一
    // 供给）；cleanup/validation 两个嵌套配置不在 JSON 映射范围内
    const auto& post_stage = JsonSectionOrEmpty(details, "post_stage");
    LoadJsonFieldIfPresent(post_stage, "epsilon_v",
                           &config->post_epsilon_v);
    LoadJsonFieldIfPresent(post_stage, "v_dwell", &config->post_v_dwell);
    LoadJsonFieldIfPresent(post_stage, "shift_delay",
                           &config->post_shift_delay);
    LoadJsonFieldIfPresent(post_stage, "kappa_pad",
                           &config->post_kappa_pad);
    LoadJsonFieldIfPresent(post_stage, "seam_speed_tol",
                           &config->post_seam_speed_tol);
    LoadJsonFieldIfPresent(post_stage, "dwell_omega_tol",
                           &config->post_dwell_omega_tol);
    LoadJsonFieldIfPresent(post_stage, "amplitude_check_tol",
                           &config->post_amplitude_check_tol);
    LoadJsonFieldIfPresent(post_stage, "amplitude_check_rel_tol",
                           &config->post_amplitude_check_rel_tol);
    LoadJsonFieldIfPresent(post_stage, "control_overshoot_tol",
                           &config->post_control_overshoot_tol);
    LoadJsonFieldIfPresent(post_stage, "stage_two_min_tracking_weight",
                           &config->post_stage_two_min_tracking_weight);
    LoadJsonFieldIfPresent(
        post_stage, "skip_stage_two_when_weight_exhausted",
        &config->post_skip_stage_two_when_weight_exhausted);
    // Reeds-Shepp 换挡点短接节（参考构建前的同伦类重选，默认关闭；
    // 编排为 maneuver 边界节点的动态规划全局择优）
    const auto& rs_shortcut = JsonSectionOrEmpty(details, "rs_shortcut");
    LoadJsonFieldIfPresent(rs_shortcut, "cap_ratio",
                           &config->rs_cap_ratio);
    LoadJsonFieldIfPresent(rs_shortcut, "collision_margin",
                           &config->rs_collision_margin);
    LoadJsonFieldIfPresent(rs_shortcut, "max_length_growth",
                           &config->rs_max_length_growth);
    LoadJsonFieldIfPresent(rs_shortcut, "sample_dist",
                           &config->rs_sample_dist);
    LoadJsonFieldIfPresent(rs_shortcut, "segment_fixed_cost",
                           &config->rs_segment_fixed_cost);
    LoadJsonFieldIfPresent(rs_shortcut, "short_segment_weight",
                           &config->rs_short_segment_weight);
    LoadJsonFieldIfPresent(rs_shortcut, "short_segment_length",
                           &config->rs_short_segment_length);
    LoadJsonFieldIfPresent(rs_shortcut, "rs_timing_csv",
                           &config->rs_timing_csv);
    LoadJsonFieldIfPresent(rs_shortcut, "rs_timing_tag",
                           &config->rs_timing_tag);
    // 融化开/关双候选择优（顶层字段，默认 false = 关闭）
    LoadJsonFieldIfPresent(details, "dual_candidate_select",
                           &config->dual_candidate_select);
    // 幅值边界由权威来源同步进全部消费方（JSON 层单一来源的最终兑现）
    config->synchronizeAmplitudeBounds();
}

// MINCO 专有字段覆盖项：仅覆盖显式出现的字段，未出现的保持构造默认值
void LoadMincoConfigOverrides(const nlohmann::json& details,
                              MincoConfig* config) {
    if (config == nullptr) {
        throw std::invalid_argument(
            "LoadMincoConfigOverrides received null config!!!");
    }
    // ESDF 双重安全惩罚节（与 MincoConfig 扁平字段同名）
    const auto& esdf = JsonSectionOrEmpty(details, "esdf");
    LoadJsonFieldIfPresent(esdf, "margin_safe", &config->margin_safe);
    LoadJsonFieldIfPresent(esdf, "margin_comf", &config->margin_comf);
    LoadJsonFieldIfPresent(esdf, "weight_safe", &config->weight_safe);
    LoadJsonFieldIfPresent(esdf, "weight_comf", &config->weight_comf);
}

MincoConfig PostProcessor::DeriveKinematicsConfig(
    const VehicleParams& vehicle_params, const MincoConfig& minco_config) {
    MincoConfig config = minco_config;
    config.wheelbase = vehicle_params.wheelbase;
    // max_velocity 是 MINCO 路径独立配置量，保持 minco_config 原值不覆盖
    // 加速度上限取双向较小值，保证正向加速与倒车减速都不越界
    config.max_acceleration =
        std::min(vehicle_params.max_accel, std::abs(vehicle_params.max_decel));
    config.max_steer_angle = vehicle_params.max_steer_angle;
    config.max_steer_rate = vehicle_params.max_steer_rate;
    return config;
}

PostProcessorResult PostProcessor::optimizeMinco(
    const Path& init_path, const MincoConfig& minco_config) const {
    PostProcessorResult result;
    const auto t_start = std::chrono::steady_clock::now();
    // 统一收尾：填充耗时与结果统计后返回（失败分支 optimized_path 为空）
    const auto finish = [&result, t_start]() {
        result.total_time_ms = std::chrono::duration<double, std::milli>(
                                   std::chrono::steady_clock::now() - t_start)
                                   .count();
        // 先填充输出标签再统计
        result.algorithm = "minco";
        result.output_level = result.success ? OutputLevel::kFullSuccess
                                             : OutputLevel::kFallback;
        // 物理方向段数（v 变号）为默认口径——多项式段内过冲按实际换挡
        // 如实计入；轨迹缺失（失败分支）时回退为 Path 机动段标签数
        result.final_maneuvers =
            result.optimized_trajectory.empty()
                ? static_cast<int>(result.optimized_path.numManeuvers())
                : result.optimized_trajectory.countDirectionRuns();
        result.final_length = result.optimized_path.length();
        return result;
    };
    try {
        // 空路径无法提取起点锚点，直接显式失败
        if (init_path.empty()) {
            result.message = "MINCO input path is empty";
            return finish();
        }
        // 世界坐标还原积分的锚点：初始路径首点
        const Eigen::Vector2d start_position{init_path.front().x,
                                             init_path.front().y};
        // 运动学配置由车辆物理参数派生，全链路（预处理/主求解/融化采样）
        // 共用同一份，避免跨阶段数值不自洽
        // 由车辆物理参数派生运动学字段并覆盖到完整配置副本，全链路
        // （预处理/主求解/融化采样）共用同一份，避免跨阶段数值不自洽
        const MincoConfig derived =
            DeriveKinematicsConfig(vehicle_params_, minco_config);
        const BicycleKinematicsExtractor kinematics(derived);
        // 1) 前端解析与分段：换挡打断 + 空间等距降采样
        const MincoManeuverSegmenter segmenter(derived);
        const auto estimates = segmenter.segment(init_path);
        // 2) 预处理粗优化：把初值拉近前端路径并满足运动学约束
        const MincoPreprocessor preprocessor(derived);
        const auto pre_result =
            preprocessor.preprocess(estimates, start_position);
        if (!pre_result.success) {
            result.message = "MINCO preprocessing failed (max endpoint error " +
                             std::to_string(pre_result.max_endpoint_error) +
                             " m)";
            return finish();
        }
        // 预处理粗优化轨迹同步离散化，作为"优化前/优化后"对比的优化前基线：
        // 与最终输出共用同一套离散化工具、同一采样密度与同一运动学提取器，
        // 保证两条曲线只相差"主优化 + 机动融化"这一步
        result.intermediate_traces.emplace_back(
            "minco_preprocessed",
            FlattenManeuvers(SampleMincoTrajectory(
                pre_result.trajectory, estimates, start_position, kinematics,
                derived.samples_per_segment)));
        // 3) PHR-ALM 主优化：内层 L-BFGS + 外层乘子/惩罚权重更新
        const MincoSolver solver(derived);
        const auto solve_result = solver.solve(
            estimates, pre_result, start_position, esdf_map_, footprint_model_);
        if (solve_result.trajectory.numSegments() == 0) {
            // 内层首轮即失败或轨迹重建失败：无任何可用轨迹，显式失败
            result.message = "MINCO solve failed: no valid trajectory produced";
            return finish();
        }
        // 4) 机动融化与拓扑修剪：剔除废段、同向合并，产出采样 Path
        const MincoManeuverMelter melter(derived);
        const auto melt_result = melter.meltAndPrune(
            solve_result.trajectory, estimates, start_position, kinematics);
        Path optimized = melt_result.path;
        if (optimized.empty()) {
            result.message = "MINCO melt produced empty path";
            return finish();
        }
        // 4.2) 按 v 符号游程重切机动段：忠实于实际运动方向（几何特征），
        // 与求解器 estimate 标签解耦；段内方向反转处切分为独立 maneuver
        // （如揉库段内部的退-进-退微调），使输出 Path 段数与轨迹的
        // "物理方向段数"（countDirectionRuns）口径一致
        ResegmentByVelocityDirection(&optimized, derived.v_epsilon);
        // 4.5) 停驻窗口"停-打轮-走"合法化改写：净 Δθ 很小的换挡停驻窗口
        // （θ̇≠0 且 δ 翻转的伪影）改写为 v=0、θ 冻结、δ 按 ≤δ̇_max 有界
        // 过渡；净 Δθ 超阈值的真实 pivot 窗口保持原样（阿克曼车辆无原地
        // 转向能力，合法执行需多点掉头，登记为已知边界）
        MincoConfig padding_config = derived;
        padding_config.pad_steer_angle = vehicle_params_.max_steer_angle;
        padding_config.pad_steer_rate = vehicle_params_.max_steer_rate;
        const auto padding_stats =
            ApplySteerPadding(optimized.getManeuvers(), padding_config);
        if (padding_stats.windows_legalized > 0 ||
            padding_stats.windows_skipped > 0) {
            LOG_FMT_INFO("MINCO steer padding: legalized={}, skipped={}",
                         padding_stats.windows_legalized,
                         padding_stats.windows_skipped);
        }
        // 改写后重算曲率（停驻窗口 θ 已被冻结，原估计值已过期）
        optimized.finalize();
        // 5) 质量门：无奇异（全部采样点有限）→ 碰撞深度 → 终点精度
        if (!IsPathFinite(optimized)) {
            result.message = "MINCO output rejected: non-finite samples";
            return finish();
        }
        const double max_collision =
            ComputeMaxCollisionDepth(optimized, esdf_map_, footprint_model_);
        if (max_collision > kMaxAcceptableCollisionDepth) {
            LOG_FMT_WARN(
                "MINCO collision depth {:.4f}m exceeds threshold, rejecting",
                max_collision);
            result.message = "MINCO collision depth " +
                             std::to_string(max_collision) +
                             " m exceeds threshold, rejecting";
            return finish();
        }
        // 终点精度以采样后路径末点相对初始路径末点（即停车目标位姿）度量；
        // 阈值复用主求解器的双指标收敛判据，与 MINCO 收敛定义保持一致
        const double terminal_pos_err =
            std::hypot(optimized.back().x - init_path.back().x,
                       optimized.back().y - init_path.back().y);
        const double terminal_head_err_deg =
            std::abs(NormalizeAngle(optimized.back().theta -
                                    init_path.back().theta)) *
            RAD2DEG;
        if (terminal_pos_err > derived.terminal_position_tolerance ||
            terminal_head_err_deg > derived.terminal_heading_tolerance_deg) {
            result.message = "MINCO terminal error " +
                             std::to_string(terminal_pos_err) + " m / " +
                             std::to_string(terminal_head_err_deg) +
                             " deg exceeds tolerance, rejecting";
            return finish();
        }
        // 5.5) 输出等距重采样：MINCO 按时间等分采样，低速蠕动区点距仅
        // 1~2cm，几何曲率 κ=Δθ/Δs 被放大；按前端路径等距间隔 DELTA_DIST
        // 对整条路径抽稀过密点，恢复输出点距（输出给控制的路径间隔与前端一致）
        ResampleToUniformSpacing(&optimized.getManeuvers());
        optimized.finalize();
        // 填充 MINCO 轨迹视图（与输出 Path 同序采样点，携带 v/a/delta/delta_dot，
        // 未携带时间戳），供可视化与下游消费
        result.optimized_trajectory = FlattenManeuvers(optimized.getManeuvers());
        result.optimized_path = std::move(optimized);
        result.success = true;
        result.message =
            solve_result.status == MincoSolverStatus::CONVERGED
                ? "MINCO converged"
                : "MINCO did not fully converge after " +
                      std::to_string(solve_result.outer_iterations) +
                      " outer iterations, using last iterate (terminal "
                      "error " +
                      std::to_string(terminal_pos_err) + " m / " +
                      std::to_string(terminal_head_err_deg) + " deg)";
    } catch (const std::exception& e) {
        LOG_FMT_WARN("optimizeMinco exception: {}", e.what());
        result.message = std::string("MINCO attempt failed: ") + e.what();
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

PostProcessorResult PostProcessor::optimizeiLQR(
    const Path& init_path, const iLQRConfig& ilqr_config) const {
    PostProcessorResult result = optimizeiLQRDualCandidate(init_path, ilqr_config);
    // RS 短接失败回退：短接是一次**组合式不连续**的同伦类重选——
    // 转弯半径微变就可能选中完全不同的拼接对，产出几何上合法但
    // 求解器启动不了的参考，因此它天然不具备单调性。不能靠调参避开
    // 这个风险（那只是在现有数据集上过拟合），而是用失败重试把它堆回
    // 单调：短接不成则以未短接参考重跑一遍，保证不会比关闭短接更差
    if (ilqr_config.rs_cap_ratio > 0.0 && !result.success) {
        iLQRConfig no_shortcut_config = ilqr_config;
        no_shortcut_config.rs_cap_ratio = 0.0;
        PostProcessorResult fallback =
            optimizeiLQRDualCandidate(init_path, no_shortcut_config);
        LOG_FMT_INFO(
            "iLQR RS shortcut retry: shortcut failed, no-shortcut success={}",
            fallback.success);
        // 耗时如实合并：返回结果是两遍完整链路的产物
        fallback.total_time_ms += result.total_time_ms;
        return fallback;
    }
    return result;
}

PostProcessorResult PostProcessor::optimizeiLQRDualCandidate(
    const Path& init_path, const iLQRConfig& ilqr_config) const {
    // 默认单遍（现状语义）；启用双候选后同一输入跑两遍完整链路择优
    if (!ilqr_config.dual_candidate_select) {
        return optimizeiLQRSinglePass(init_path, ilqr_config);
    }
    // RS 短接对两个候选是完全相同的纯函数（同输入路径、同 rs_shortcut
    // 配置、同幅值钳制），提升到编排层只算一次；下发配置的 cap_ratio
    // 置零，防止 SinglePass 内部重复短接（clampToVehicleParams 幂等，
    // SinglePass 内二次钳制无副作用）
    Path shared_input = init_path;
    iLQRConfig shared_config = ilqr_config;
    if (ilqr_config.rs_cap_ratio > 0.0) {
        shared_config.clampToVehicleParams(vehicle_params_);
        Path shortcut = ShortcutShiftPoints(
            init_path, esdf_map_, footprint_model_, vehicle_params_.wheelbase,
            shared_config.reference_delta_max, shared_config);
        LOG_FMT_INFO(
            "iLQR RS shortcut: maneuvers {} -> {}, length {:.3f} -> {:.3f}",
            init_path.numManeuvers(), shortcut.numManeuvers(),
            init_path.length(), shortcut.length());
        shared_input = std::move(shortcut);
        shared_config.rs_cap_ratio = 0.0;
    }
    // 候选一：配置原样（融化开）；候选二：退火率 ≈1（融化机制实际关闭，
    // 与调参矩阵的 nomelt_control 同一语义）
    PostProcessorResult melt = optimizeiLQRSinglePass(shared_input, shared_config);
    iLQRConfig control_config = shared_config;
    control_config.outer_anneal_gamma = 0.999;
    PostProcessorResult control =
        optimizeiLQRSinglePass(shared_input, control_config);
    const bool pick_control = PreferControlCandidate(melt, control);
    LOG_FMT_INFO(
        "iLQR dual candidate: melt(success={}, maneuvers={}, length={:.3f}) "
        "vs control(success={}, maneuvers={}, length={:.3f}) -> pick {}",
        melt.success, melt.final_maneuvers, melt.final_length, control.success,
        control.final_maneuvers, control.final_length,
        pick_control ? "control" : "melt");
    // 耗时如实合并：返回结果是双遍编排的产物
    PostProcessorResult& chosen = pick_control ? control : melt;
    chosen.total_time_ms = melt.total_time_ms + control.total_time_ms;
    return chosen;
}

PostProcessorResult PostProcessor::optimizeiLQRSinglePass(
    const Path& init_path, const iLQRConfig& ilqr_config) const {
    const iLQRSolver solver(vehicle_params_, footprint_model_, esdf_map_);
    const auto ilqr_result = solver.optimizeSinglePass(init_path, ilqr_config);

    PostProcessorResult result;
    result.algorithm = "ilqr";
    result.optimized_trajectory = ilqr_result.optimized_trajectory;
    result.optimized_path = ilqr_result.optimized_path;
    result.success = ilqr_result.success;
    result.message = ilqr_result.message;
    result.total_time_ms = ilqr_result.total_time_ms;
    result.final_maneuvers = ilqr_result.final_maneuvers;
    result.final_length = ilqr_result.final_length;
    result.output_level = ilqr_result.output_level;
    return result;
}
}  // namespace apa_post_processor
