#include "nmpc_solver.h"

#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <costs/circle_footprint_esdf_penalty_cost.h>
#include <costs/composite_cost.h>
#include <qp/hpipm_solver.h>
#include <strategies/strategy_common.hpp>

#include "apa_esdf_map_adapter.h"
#include "vehicle_circle_geometry.h"

namespace apa_post_processor {
NmpcSolver::NmpcSolver(const VehicleParams& vehicle_params,
                       const VehicleFootprintModel& footprint_model,
                       NmpcSolverConfig config)
    : vehicle_params_(vehicle_params),
      circle_local_positions_(vehicle_circle_geometry::ExtractLocalCircleCenters(
          footprint_model, CircleType::OUTER)),
      circle_radius_(footprint_model.getOuterRadius()),
      config_(std::move(config)),
      converter_(vehicle_params_, config_.path_to_ocp_config) {
    if (circle_local_positions_.empty()) {
        throw std::invalid_argument(
            "NmpcSolver: footprint_model produced zero outer circles!!!");
    }
}

NmpcSolver::Result NmpcSolver::optimize(const Path& initial_path,
                                        const ESDFMap& esdf_map) const {
    const auto start_time = std::chrono::steady_clock::now();
    auto conv = converter_.convert(initial_path);
    // 把每段原有的跟踪代价与圆形分解ESDF碰撞惩罚代价组合为CompositeCost；碰撞惩罚代价
    // 直接持有ESDF地图适配器的引用，每次SQP迭代重新查询，不需要额外的参数注入步骤（
    // 见NmpcSolverConfig::esdf_penalty_weight注释）。
    const ApaEsdfMapAdapter map_adapter(esdf_map);
    for (auto& segment : conv.ocp.segments()) {
        auto esdf_penalty_cost = std::make_shared<stc_SQP::CircleFootprintEsdfPenaltyCost>(
            circle_local_positions_, circle_radius_, config_.esdf_safety_margin, map_adapter,
            config_.esdf_penalty_weight);
        segment.cost = std::make_shared<stc_SQP::CompositeCost>(
            std::vector<std::shared_ptr<stc_SQP::CostTerm>>{segment.cost, esdf_penalty_cost});
    }

    const int nx = conv.ocp.nx();
    const int nu = conv.ocp.nu();
    const int total_steps = conv.ocp.totalSteps();
    // 复用AutoAdaptiveStrategy的判定逻辑（是否启用partial condensing/OpenMP并行），
    // 但不经过该类本身，以便自定义SQPSolverOptions与HPIPM容差（见头文件注释）
    const int cond_n = stc_SQP::strategy_internal::computeCondN(
        total_steps, config_.hpipm_block_size, config_.short_n_threshold);
    const bool use_omp = stc_SQP::strategy_internal::computeUseOmp(
        total_steps, config_.short_n_threshold);

    // 碰撞已改为软代价，不再使用一般约束，ng恒为0
    auto qp_solver = std::make_unique<stc_SQP::HPIPMQPSolver>(
        total_steps, nx, nu, nx, nu, /*ng=*/0, /*ns=*/0, cond_n);
    qp_solver->setTolerance(config_.hpipm_tol);
    stc_SQP::SQPSolver solver(std::move(qp_solver));
    solver.options().use_omp = use_omp;
    solver.options().max_iter = config_.max_iter;
    solver.options().use_line_search = config_.use_line_search;

    Result result;
    result.converged = solver.solve(conv.ocp, conv.init_guess, result.trajectory);
    // 记录分段结构（每段步数与方向），供ToPath()把trajectory还原为Maneuver序列
    result.segment_steps.reserve(conv.ocp.segments().size());
    result.segment_v_signs.reserve(conv.ocp.segments().size());
    for (const auto& segment : conv.ocp.segments()) {
        result.segment_steps.push_back(segment.N);
        result.segment_v_signs.push_back(segment.v_sign);
    }
    const auto end_time = std::chrono::steady_clock::now();
    result.solve_time_ms =
        std::chrono::duration<double, std::milli>(end_time - start_time).count();
    return result;
}

Path NmpcSolver::ToPath(const Result& result) {
    Path path;
    // trajectory为空表示solve()在第0次迭代就失败（见头文件注释），此时没有可还原的轨迹，
    // 直接返回空Path，调用方应结合result.converged自行判断是否要回退到原始初始路径。
    if (result.trajectory.x.empty()) {
        return path;
    }
    auto& maneuvers = path.getManeuvers();
    int global_k = 0;
    int global_u = 0;
    for (std::size_t seg_idx = 0; seg_idx < result.segment_steps.size(); ++seg_idx) {
        const int step_num = result.segment_steps[seg_idx];
        const Direction direction =
            (result.segment_v_signs[seg_idx] > 0.0) ? Direction::FORWARD : Direction::BACKWARD;
        std::vector<PathPoint> points;
        points.reserve(static_cast<std::size_t>(step_num) + 1);
        for (int i = 0; i <= step_num; ++i) {
            const auto& state = result.trajectory.x[static_cast<std::size_t>(global_k + i)];
            PathPoint point(state(0), state(1), state(2));
            // 回填NMPC优化产出的状态量（v/delta）与控制量（a/delta_dot）。
            // 控制序列比状态序列少一个，每段最后一个点没有对应的控制量，
            // 因此保持 hasA()/hasDeltaDot() 为 false，属于预期行为。
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

NmpcSolver::Result NmpcSolver::optimizeWithPruning(const Path& initial_path,
                                                   const ESDFMap& esdf_map,
                                                   const PruningConfig& pruning_config) const {
    const Pose& goal_pose = initial_path.back();
    Result best_result = optimize(initial_path, esdf_map);
    double best_total_length = ToPath(best_result).length();
    for (int iter = 0; iter < pruning_config.max_prune_iterations; ++iter) {
        auto pruned_path =
            pruneShortestSegment(best_result, pruning_config.min_segment_arc_length);
        if (!pruned_path.has_value()) {
            break;
        }
        Result candidate = optimize(*pruned_path, esdf_map);
        if (candidate.trajectory.x.empty()) {
            // 裁剪后引入的跳变导致求解失败（未产出可用轨迹），回退到裁剪前的结果并停止
            break;
        }
        const auto& candidate_final = candidate.trajectory.x.back();
        const double goal_deviation = std::hypot(candidate_final(0) - goal_pose.x,
            candidate_final(1) - goal_pose.y);
        if (goal_deviation > pruning_config.max_terminal_deviation) {
            // 裁剪后剩余机动段结构已无法运动学可行地到达原目标（如强行合并为单一方向后
            // 转弯半径不够），终点明显偏离原始Path终点，回退到裁剪前的结果并停止
            break;
        }
        // 距离-段数等效权衡：裁剪掉的机动段数每一个都“值”maneuver_length_equivalent米，
        // 只有当总弧长增量不超过这个等效收益时才接受本轮裁剪，否则认为得不偿失
        const double candidate_total_length = ToPath(candidate).length();
        const int maneuvers_removed = static_cast<int>(best_result.segment_steps.size())
            - static_cast<int>(candidate.segment_steps.size());
        const double allowed_increase =
            maneuvers_removed * pruning_config.maneuver_length_equivalent;
        if (candidate_total_length - best_total_length > allowed_increase) {
            break;
        }
        candidate.prune_iterations = best_result.prune_iterations + 1;
        best_result = std::move(candidate);
        best_total_length = candidate_total_length;
    }
    return best_result;
}

std::optional<Path> NmpcSolver::pruneShortestSegment(const Result& result,
                                                     double min_arc_length) {
    if (result.trajectory.x.empty() || result.segment_steps.size() <= 1) {
        return std::nullopt;
    }
    // 逐段计算弧长（对已求解轨迹的相邻状态点求欧氏距离累加），记录每段在全局轨迹中的起始下标
    std::vector<int> segment_starts(result.segment_steps.size());
    int shortest_idx = -1;
    double shortest_arc_length = min_arc_length;
    int global_k = 0;
    for (std::size_t seg = 0; seg < result.segment_steps.size(); ++seg) {
        segment_starts[seg] = global_k;
        const int step_num = result.segment_steps[seg];
        double arc_length = 0.0;
        for (int i = 0; i < step_num; ++i) {
            const auto& p0 = result.trajectory.x[static_cast<std::size_t>(global_k + i)];
            const auto& p1 = result.trajectory.x[static_cast<std::size_t>(global_k + i + 1)];
            arc_length += std::hypot(p1(0) - p0(0), p1(1) - p0(1));
        }
        if (arc_length < shortest_arc_length) {
            shortest_arc_length = arc_length;
            shortest_idx = static_cast<int>(seg);
        }
        global_k += step_num;
    }
    if (shortest_idx < 0) {
        return std::nullopt;
    }
    // 跳过弧长最短的机动段，其余段原样保留；由于Maneuver方向严格交替，被跳过段两侧的
    // 相邻段必然同号，若相邻段在裁剪后紧挨在一起，直接拼接点序列合并为一段（不去重，
    // 因为跳过段两端并非同一物理点，存在真实的位置跳变，交由重新求解时的弧长插值消化）
    Path pruned_path;
    auto& maneuvers = pruned_path.getManeuvers();
    for (std::size_t seg = 0; seg < result.segment_steps.size(); ++seg) {
        if (static_cast<int>(seg) == shortest_idx) {
            continue;
        }
        const int start = segment_starts[seg];
        const int step_num = result.segment_steps[seg];
        const Direction direction =
            (result.segment_v_signs[seg] > 0.0) ? Direction::FORWARD : Direction::BACKWARD;
        std::vector<PathPoint> points;
        points.reserve(static_cast<std::size_t>(step_num) + 1);
        for (int i = 0; i <= step_num; ++i) {
            const auto& state = result.trajectory.x[static_cast<std::size_t>(start + i)];
            PathPoint point(state(0), state(1), state(2));
            point.setV(state(3));
            point.setDelta(state(4));
            if (i < step_num) {
                const auto& control =
                    result.trajectory.u[static_cast<std::size_t>(start + i)];
                point.setA(control(0));
                point.setDeltaDot(control(1));
            }
            points.emplace_back(std::move(point));
        }
        if (!maneuvers.empty() && maneuvers.back().direction == direction) {
            auto& prev_points = maneuvers.back().points;
            prev_points.insert(prev_points.end(), points.begin(), points.end());
        } else {
            maneuvers.emplace_back(std::move(points), direction);
        }
    }
    if (maneuvers.empty()) {
        return std::nullopt;
    }
    return pruned_path;
}
}  // namespace apa_post_processor

