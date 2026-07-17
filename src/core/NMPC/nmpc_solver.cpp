#include "nmpc_solver.h"

#include <costs/circle_footprint_esdf_penalty_cost.h>
#include <costs/composite_cost.h>
#include <costs/quadratic_tracking.h>
#include <qp/hpipm_solver.h>

#include <chrono>
#include <cmath>
#include <math/math_util.hpp>
#include <memory>
#include <stdexcept>
#include <strategies/strategy_common.hpp>
#include <string>
#include <tuple>

#include "apa_esdf_map_adapter.h"
#include "iterative_corridor_constraint.h"
#include "position_trust_region_constraint.h"
#include "static_corridor_linear_constraint.h"
#include "theta_trust_region_constraint.h"
#include "vehicle_circle_geometry.h"

namespace {
void SetStcSqpLogSink() {
    static bool initialized = false;
    if (initialized) {
        return;
    }
    initialized = true;
    stc_SQP::Logger::SetSink(
        [](stc_SQP::LogLevel level, const std::string& msg) {
            switch (level) {
                case stc_SQP::LogLevel::ERROR:
                    LOG_FMT_ERROR("StcSQP: {}", msg);
                    break;
                case stc_SQP::LogLevel::WARN:
                    LOG_FMT_WARN("StcSQP: {}", msg);
                    break;
                case stc_SQP::LogLevel::INFO:
                    LOG_FMT_INFO("StcSQP: {}", msg);
                    break;
                default:
                    LOG_FMT_DEBUG("StcSQP: {}", msg);
                    break;
            }
        });
}
}  // namespace

namespace apa_post_processor {
NmpcSolver::NmpcSolver(const VehicleParams& vehicle_params,
                       const VehicleFootprintModel& footprint_model,
                       NMPCConfig config)
    : vehicle_params_(vehicle_params),
      circle_local_positions_(
          vehicle_circle_geometry::ExtractLocalCircleCenters(
              footprint_model, CircleType::OUTER)),
      circle_radius_(footprint_model.getOuterRadius()),
      config_(std::move(config)),
      converter_(vehicle_params_, config_.path_to_ocp_config) {
    if (circle_local_positions_.empty()) {
        throw std::invalid_argument(
            "NmpcSolver: footprint_model produced zero outer circles!!!");
    }
    SetStcSqpLogSink();
}

NmpcSolver::Result NmpcSolver::optimize(const Path& initial_path,
                                        const ESDFMap& esdf_map) const {
    const auto start_time = std::chrono::steady_clock::now();
    auto conv = converter_.convert(initial_path);
    auto result = solveOcp(conv.ocp, conv.init_guess, esdf_map);
    const auto end_time = std::chrono::steady_clock::now();
    result.solve_time_ms =
        std::chrono::duration<double, std::milli>(end_time - start_time)
            .count();
    // 打印耗时供基线对比。
    LOG_FMT_INFO(
        "NmpcSolver::optimize(Path): solve_time_ms={:.3f}, converged={}",
        result.solve_time_ms, result.converged);
    return result;
}

NmpcSolver::Result NmpcSolver::optimize(const stc_SQP::MultiStageOCP& ocp,
                                        const stc_SQP::Trajectory& init_guess,
                                        const ESDFMap& esdf_map) const {
    const auto start_time = std::chrono::steady_clock::now();
    auto result = solveOcp(ocp, init_guess, esdf_map);
    const auto end_time = std::chrono::steady_clock::now();
    result.solve_time_ms =
        std::chrono::duration<double, std::milli>(end_time - start_time)
            .count();
    // 打印耗时供基线对比。
    LOG_FMT_INFO(
        "NmpcSolver::optimize(OCP): solve_time_ms={:.3f}, converged={}",
        result.solve_time_ms, result.converged);
    return result;
}

NmpcSolver::Result NmpcSolver::solveOcp(const stc_SQP::MultiStageOCP& ocp,
                                        const stc_SQP::Trajectory& init_guess,
                                        const ESDFMap& esdf_map) const {
    const int circle_num = static_cast<int>(circle_local_positions_.size());
    const int total_steps = ocp.totalSteps();
    const int nx = ocp.nx();
    const int nu = ocp.nu();
    const int cond_n = stc_SQP::strategy_internal::computeCondN(
        total_steps, config_.hpipm_block_size, config_.short_n_threshold);
    const bool use_omp = stc_SQP::strategy_internal::computeUseOmp(
        total_steps, config_.short_n_threshold);

    // 迭代走廊是安全下界始终生效；静态舒适走廊按需叠加为 HPIPM 软约束。
    stc_SQP::MultiStageOCP mutable_ocp = ocp;
    const bool has_static_corridor =
        config_.static_corridor_C.has_value() &&
        config_.static_corridor_d.has_value() &&
        config_.static_corridor_C->rows() > 0 &&
        config_.static_corridor_C->rows() == config_.static_corridor_d->size();
    bool use_static_corridor = false;
    int static_constraints_per_step = 0;
    if (has_static_corridor) {
        if (config_.static_corridor_C->rows() % total_steps != 0) {
            LOG_FMT_WARN(
                "NmpcSolver: static comfort corridor rows {} not divisible by "
                "total_steps {}, skip comfort constraint",
                config_.static_corridor_C->rows(), total_steps);
        } else {
            static_constraints_per_step =
                config_.static_corridor_C->rows() / total_steps;
            if (static_constraints_per_step <= 0) {
                LOG_FMT_WARN(
                    "NmpcSolver: static constraints_per_step is 0, skip "
                    "comfort constraint");
            } else {
                use_static_corridor = true;
            }
        }
    }
    const bool inject_comfort_corridor =
        use_static_corridor && config_.use_static_corridor_soft_constraint;

    // 航向跟踪与位置跟踪统一使用软代价。
    const bool use_theta_tracking = config_.theta_tracking_weight > 0.0;
    const bool use_position_tracking = config_.position_tracking_weight > 0.0;
    const int num_segments = static_cast<int>(mutable_ocp.segments().size());
    int global_step_offset = 0;
    int seg_idx = 0;
    for (auto& segment : mutable_ocp.segments()) {
        const bool is_last_segment = (seg_idx == num_segments - 1);
        if (use_theta_tracking) {
            // 航向跟踪：复用 ThetaTrustRegionConstraint 的 ng=2 约束形式，作为
            // HPIPM 软约束注入。
            segment.constraints.push_back(
                std::make_shared<ThetaTrustRegionConstraint>(
                    config_.max_theta_deviation_from_ref));
        }
        if (use_position_tracking) {
            // 位置跟踪：复用 PositionTrustRegionConstraint 的 ng=4
            // 约束形式，作为 HPIPM 软约束注入。
            segment.constraints.push_back(
                std::make_shared<PositionTrustRegionConstraint>(
                    config_.max_position_deviation_from_ref));
        }
        // 迭代走廊：每轮 SQP 重新线性化 ESDF，始终注入提供碰撞安全下界。
        segment.constraints.push_back(
            std::make_shared<IterativeCorridorConstraint>(
                esdf_map, circle_local_positions_, circle_radius_,
                global_step_offset, circle_num, config_.corridor_hard_margin));
        if (inject_comfort_corridor) {
            // 静态舒适走廊：预处理冻结线性化的舒适度软约束。
            segment.constraints.push_back(
                std::make_shared<StaticCorridorLinearConstraint>(
                    *config_.static_corridor_C, *config_.static_corridor_d,
                    global_step_offset, static_constraints_per_step, segment.N,
                    /*skip_last_step=*/is_last_segment &&
                        config_.skip_last_step_corridor));
        }
        global_step_offset += segment.N;
        ++seg_idx;
    }

    // ESDF 代价提供软引导梯度，迭代走廊提供安全下界，两者协同不互斥。
    // 适配器必须在 SQP 求解期间保持存活。
    std::unique_ptr<ApaEsdfMapAdapter> esdf_adapter;
    if (config_.esdf_penalty_weight > 0.0) {
        esdf_adapter = std::make_unique<ApaEsdfMapAdapter>(esdf_map);
        auto esdf_penalty_cost =
            std::make_shared<stc_SQP::CircleFootprintEsdfPenaltyCost>(
                circle_local_positions_, circle_radius_,
                config_.esdf_safety_margin, *esdf_adapter,
                config_.esdf_penalty_weight);
        for (auto& segment : mutable_ocp.segments()) {
            auto composite = std::make_shared<stc_SQP::CompositeCost>(
                std::vector<std::shared_ptr<stc_SQP::CostTerm>>{
                    segment.cost, esdf_penalty_cost->clone()});
            segment.cost = std::move(composite);
        }
    }

    int ng_max = 0;
    int ns_per_step = 0;
    std::vector<int> soft_constraint_idxs;
    // Zu（二次项）与 zu（一次项）分开维护；迭代走廊使用独立字段。
    std::vector<double> soft_Zu_weights;
    std::vector<double> soft_zu_weights;
    // 按注入顺序动态累加行偏移量，必须与上方约束注入循环顺序一致。
    {
        int offset = 0;
        if (use_theta_tracking) {
            // 航向跟踪 2 行全部注册为软约束。
            for (int j = 0; j < 2; ++j) {
                soft_constraint_idxs.push_back(offset + j);
                soft_Zu_weights.push_back(config_.theta_tracking_weight);
                soft_zu_weights.push_back(config_.theta_tracking_weight);
            }
            offset += 2;
        }
        if (use_position_tracking) {
            // 位置跟踪 4 行全部注册为软约束。
            for (int j = 0; j < 4; ++j) {
                soft_constraint_idxs.push_back(offset + j);
                soft_Zu_weights.push_back(config_.position_tracking_weight);
                soft_zu_weights.push_back(config_.position_tracking_weight);
            }
            offset += 4;
        }
        // 迭代走廊：始终注入，全部行软化，Zu/zu 使用解耦后的独立字段。
        for (int j = 0; j < circle_num; ++j) {
            soft_constraint_idxs.push_back(offset + j);
            soft_Zu_weights.push_back(config_.corridor_soft_quadratic_weight);
            soft_zu_weights.push_back(config_.corridor_soft_linear_weight);
        }
        offset += circle_num;
        if (inject_comfort_corridor) {
            // 静态舒适走廊：Zu=zu 耦合的额外软约束。
            for (int j = 0; j < static_constraints_per_step; ++j) {
                soft_constraint_idxs.push_back(offset + j);
                soft_Zu_weights.push_back(config_.static_corridor_soft_weight);
                soft_zu_weights.push_back(config_.static_corridor_soft_weight);
            }
        }
        ns_per_step = static_cast<int>(soft_constraint_idxs.size());
    }
    // ng_max 由 mutable_ocp 各段实际 ng() 之和求最大值得出。
    ng_max = stc_SQP::strategy_internal::computeOcpNgMax(mutable_ocp);

    auto qp_solver = std::make_unique<stc_SQP::HPIPMQPSolver>(
        total_steps, nx, nu, nx, nu, ng_max, ns_per_step, cond_n);
    qp_solver->setTolerance(config_.hpipm_tol);
    stc_SQP::SQPSolver solver(std::move(qp_solver));
    solver.options().use_omp = use_omp;
    solver.options().max_iter = config_.max_iter;
    solver.options().use_line_search = config_.use_line_search;
    solver.options().hessian_regularization =
        config_.sqp_hessian_regularization;

    // QPData 对象池：按 (N, nx, nu, ng_max) 缓存，避免反复分配。
    {
        auto cache_key = std::make_tuple(total_steps, nx, nu, ng_max);
        auto it = qp_data_cache_.find(cache_key);
        if (it != qp_data_cache_.end()) {
            solver.setExternalQPData(std::move(it->second));
            qp_data_cache_.erase(it);
        }
    }

    // HPIPM 原生软约束：按各自权重独立软化。
    stc_SQP::SoftConstraintConfig soft_cfg;
    soft_cfg.ns = ns_per_step;
    soft_cfg.idxs = soft_constraint_idxs;
    soft_cfg.Zl.resize(ns_per_step);
    soft_cfg.Zu.resize(ns_per_step);
    soft_cfg.zl.resize(ns_per_step);
    soft_cfg.zu.resize(ns_per_step);
    for (int j = 0; j < ns_per_step; ++j) {
        soft_cfg.Zl[j] = 0.0;
        soft_cfg.zl[j] = 0.0;
        soft_cfg.Zu[j] = soft_Zu_weights[j];
        soft_cfg.zu[j] = soft_zu_weights[j];
    }
    solver.options().soft_constraint_config = std::move(soft_cfg);

    Result result;
    try {
        LOG_FMT_INFO(
            "NmpcSolver: start SQP solve, total_steps={}, ng_max={}, ns={}, "
            "comfort_corridor_injected={}",
            total_steps, ng_max, ns_per_step, inject_comfort_corridor);
        result.converged =
            solver.solve(mutable_ocp, init_guess, result.trajectory);
        LOG_FMT_INFO(
            "NmpcSolver: SQP solve finished, converged={}, trajectory_size={}",
            result.converged, result.trajectory.x.size());
    } catch (const std::exception& e) {
        LOG_FMT_ERROR("NmpcSolver: SQP solve threw exception: {}", e.what());
        result.converged = false;
    }
    // 将 QPData 归还对象池
    {
        auto cached = solver.takeQPData();
        if (cached) {
            auto cache_key = std::make_tuple(total_steps, nx, nu, ng_max);
            qp_data_cache_[cache_key] = std::move(cached);
        }
    }
    result.segment_steps.reserve(mutable_ocp.segments().size());
    result.segment_v_signs.reserve(mutable_ocp.segments().size());
    for (const auto& segment : mutable_ocp.segments()) {
        result.segment_steps.push_back(segment.N);
        result.segment_v_signs.push_back(segment.v_sign);
    }

    // 诊断日志
    if (!result.trajectory.x.empty()) {
        std::string seg_info =
            "segment_count=" + std::to_string(result.segment_steps.size());
        int global_k = 0;
        for (std::size_t i = 0; i < result.segment_steps.size(); ++i) {
            const int N = result.segment_steps[i];
            double arc = 0.0;
            for (int j = 0; j < N; ++j) {
                const auto& p0 =
                    result.trajectory.x[static_cast<std::size_t>(global_k + j)];
                const auto& p1 =
                    result.trajectory
                        .x[static_cast<std::size_t>(global_k + j + 1)];
                arc += std::hypot(p1(0) - p0(0), p1(1) - p0(1));
            }
            seg_info += " [" + std::to_string(i) +
                        "]sign=" + std::to_string(result.segment_v_signs[i]) +
                        " N=" + std::to_string(N) +
                        " arc=" + std::to_string(arc);
            global_k += N;
        }

        // 碰撞深度（纯诊断）
        double max_collision_depth = 0.0;
        for (const auto& x : result.trajectory.x) {
            const double px = x(0), py = x(1), theta = x(2);
            const double c = std::cos(theta), s = std::sin(theta);
            for (const auto& local : circle_local_positions_) {
                const double wx = px + local.x() * c - local.y() * s;
                const double wy = py + local.x() * s + local.y() * c;
                const double dist = esdf_map.getDist(wx, wy);
                const double depth = circle_radius_ - dist;
                if (depth > max_collision_depth) max_collision_depth = depth;
            }
        }

        // 长度变化（纯诊断）
        double init_len = 0.0, opt_len = 0.0;
        for (std::size_t i = 1; i < init_guess.x.size(); ++i)
            init_len += std::hypot(init_guess.x[i](0) - init_guess.x[i - 1](0),
                                   init_guess.x[i](1) - init_guess.x[i - 1](1));
        for (std::size_t i = 1; i < result.trajectory.x.size(); ++i)
            opt_len += std::hypot(
                result.trajectory.x[i](0) - result.trajectory.x[i - 1](0),
                result.trajectory.x[i](1) - result.trajectory.x[i - 1](1));
        const double len_pct =
            init_len > 0 ? 100.0 * (opt_len - init_len) / init_len : 0.0;

        LOG_FMT_INFO(
            "NmpcSolver: {} collision_depth={:.4f} len={:.3f}->{:.3f} "
            "({:+.1f}%) converged={}",
            seg_info, max_collision_depth, init_len, opt_len, len_pct,
            result.converged);

        // 终点收敛质量检查
        if (!result.trajectory.x.empty()) {
            const auto& term_state = result.trajectory.x.back();
            const double term_pos_err =
                std::hypot(term_state(0) - init_guess.x.back()(0),
                           term_state(1) - init_guess.x.back()(1));
            const double term_head_err_rad = std::abs(std::remainder(
                term_state(2) - init_guess.x.back()(2), 2.0 * M_PI));
            const double term_head_err_deg = term_head_err_rad * 180.0 / M_PI;
            LOG_FMT_INFO(
                "NmpcSolver terminal: pos_err={:.4f}m head_err={:.2f}deg",
                term_pos_err, term_head_err_deg);
            // 质量门：超出阈值标记不收敛
            if (config_.terminal_position_error_threshold > 0 &&
                term_pos_err > config_.terminal_position_error_threshold) {
                result.converged = false;
                LOG_FMT_WARN(
                    "NmpcSolver terminal position error {:.4f}m exceeds "
                    "threshold {:.4f}m",
                    term_pos_err, config_.terminal_position_error_threshold);
            }
            if (config_.terminal_heading_error_threshold_deg > 0 &&
                term_head_err_deg >
                    config_.terminal_heading_error_threshold_deg) {
                result.converged = false;
                LOG_FMT_WARN(
                    "NmpcSolver terminal heading error {:.2f}deg exceeds "
                    "threshold {:.2f}deg",
                    term_head_err_deg,
                    config_.terminal_heading_error_threshold_deg);
            }
        }
    }

    return result;
}

Path NmpcSolver::ToPath(const Result& result) {
    Path path;
    // trajectory 为空表示第0次迭代即失败，返回空 Path。
    if (result.trajectory.x.empty()) {
        return path;
    }
    auto& maneuvers = path.getManeuvers();
    int global_k = 0;
    int global_u = 0;
    for (std::size_t seg_idx = 0; seg_idx < result.segment_steps.size();
         ++seg_idx) {
        const int step_num = result.segment_steps[seg_idx];
        const Direction direction = (result.segment_v_signs[seg_idx] > 0.0)
                                        ? Direction::FORWARD
                                        : Direction::BACKWARD;
        std::vector<TrajectoryPoint> points;
        points.reserve(static_cast<std::size_t>(step_num) + 1);
        for (int i = 0; i <= step_num; ++i) {
            const auto& state =
                result.trajectory.x[static_cast<std::size_t>(global_k + i)];
            TrajectoryPoint point(state(0), state(1), state(2));
            // 回填状态量（v/delta）与控制量（a/delta_dot）。
            // 控制序列比状态序列少一个，末尾点无对应控制量为预期行为。
            point.setV(state(3));
            point.setDelta(state(4));
            if (i < step_num) {
                const auto& control =
                    result.trajectory.u[static_cast<std::size_t>(global_u + i)];
                point.setA(control(0));
                point.setDeltaDot(control(1));
            }
            points.emplace_back(std::move(point));
        }
        maneuvers.emplace_back(std::move(points), direction);
        global_k += step_num;
        global_u += step_num;
    }
    return path;
}

// QPData 对象池（13.3）：静态缓存跨 NmpcSolver 实例共享，key 为 (N, nx, nu,
// ng_max)。 避免每次 solveOcp() 调用内 SQPSolver::solve() 重新 new/delete
// 一整块对齐内存池。
std::map<std::tuple<int, int, int, int>, std::unique_ptr<stc_SQP::QPData>>
    NmpcSolver::qp_data_cache_;

}  // namespace apa_post_processor
