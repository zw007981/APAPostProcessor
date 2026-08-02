#include "alm_solver.h"

#include <LBFGS.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace apa_post_processor {
Eigen::VectorXd AlmSolverProblem::initialGuess() const {
    const int num_segments = numSegments();
    Eigen::VectorXd guess(variableCount());
    for (int m = 0; m + 1 < num_segments; ++m) {
        guess[2 * m] = initial_waypoints[m].x();
        guess[2 * m + 1] = initial_waypoints[m].y();
    }
    for (int i = 0; i < num_segments; ++i) {
        guess[2 * (num_segments - 1) + i] =
            MincoTrajectory::DurationToTau(initial_durations[i]);
    }
    guess[variableCount() - 1] = initial_final_arc_length;
    return guess;
}

AlmSolver::AlmSolver(AlmSolverConfig config,
                     BicycleKinematicsConfig kinematics_config,
                     AlmEsdfPenaltyConfig esdf_config)
    : config_(config),
      kinematics_(kinematics_config),
      esdf_config_(esdf_config) {
    // 收敛判据与外层预算：正有限值
    if (!(config_.terminal_position_tolerance > 0.0) ||
        !std::isfinite(config_.terminal_position_tolerance) ||
        !(config_.terminal_heading_tolerance_deg > 0.0) ||
        !std::isfinite(config_.terminal_heading_tolerance_deg) ||
        config_.max_outer_iterations < 1) {
        throw std::invalid_argument("AlmSolverConfig 收敛判据/外层预算非法");
    }
    // ρ 标定与更新参数：0 < ρ_min <= ρ_max，安全阀/首轮权重为正，γ 非负
    if (!(config_.rho_min > 0.0) || !(config_.rho_max >= config_.rho_min) ||
        !std::isfinite(config_.rho_max) || !(config_.epsilon_rho > 0.0) ||
        !std::isfinite(config_.epsilon_rho) ||
        !(config_.first_round_rho > 0.0) ||
        !std::isfinite(config_.first_round_rho) ||
        config_.rho_increase_factor < 0.0 ||
        !std::isfinite(config_.rho_increase_factor)) {
        throw std::invalid_argument("AlmSolverConfig ρ 标定/更新参数非法");
    }
    // 门控阈值仅在门控启用时校验（κ∈(0,1)）
    if (config_.use_rho_increase_gate &&
        !(config_.rho_gate_kappa > 0.0 && config_.rho_gate_kappa < 1.0)) {
        throw std::invalid_argument("AlmSolverConfig 门控阈值必须落在 (0,1)");
    }
    // 权重类字段：非负且有限
    if (config_.weight_jerk_s < 0.0 || !std::isfinite(config_.weight_jerk_s) ||
        config_.weight_jerk_theta < 0.0 ||
        !std::isfinite(config_.weight_jerk_theta) ||
        config_.epsilon_time < 0.0 || !std::isfinite(config_.epsilon_time) ||
        config_.weight_velocity < 0.0 ||
        !std::isfinite(config_.weight_velocity) ||
        config_.weight_acceleration < 0.0 ||
        !std::isfinite(config_.weight_acceleration) ||
        config_.weight_steer_angle < 0.0 ||
        !std::isfinite(config_.weight_steer_angle) ||
        config_.weight_steer_rate < 0.0 ||
        !std::isfinite(config_.weight_steer_rate) ||
        config_.weight_gear_cusp < 0.0 ||
        !std::isfinite(config_.weight_gear_cusp) ||
        config_.weight_gear_cusp_theta < 0.0 ||
        !std::isfinite(config_.weight_gear_cusp_theta) ||
        config_.weight_duration_balance < 0.0 ||
        !std::isfinite(config_.weight_duration_balance)) {
        throw std::invalid_argument("AlmSolverConfig 权重必须非负有限");
    }
    // 段时长平衡系数：0 < ε_low < ε_upp 才有非空可行带
    if (!(config_.duration_balance_lower > 0.0) ||
        !(config_.duration_balance_upper > config_.duration_balance_lower)) {
        throw std::invalid_argument("段时长平衡系数必须满足 0 < ε_low < ε_upp");
    }
    // 采样配置：梯形至少 2 点；辛普森子区间必须为偶数且至少 2
    if (config_.physics_samples_per_segment < 2 ||
        config_.simpson_subintervals < 2 ||
        config_.simpson_subintervals % 2 != 0) {
        throw std::invalid_argument(
            "物理采样数须 >= 2，辛普森子区间须为 >= 2 的偶数");
    }
    // L-BFGS 参数（与预处理器同一套校验约定）
    if (config_.lbfgs_max_iterations <= 0 || config_.lbfgs_epsilon <= 0.0 ||
        config_.lbfgs_epsilon_rel < 0.0 || config_.lbfgs_m <= 0 ||
        config_.lbfgs_max_linesearch <= 0 ||
        config_.lbfgs_linesearch_algo < 1 ||
        config_.lbfgs_linesearch_algo > 3 || config_.lbfgs_ftol <= 0.0 ||
        config_.lbfgs_ftol >= 0.5 ||
        config_.lbfgs_wolfe <= config_.lbfgs_ftol ||
        config_.lbfgs_wolfe >= 1.0) {
        throw std::invalid_argument("AlmSolverConfig L-BFGS 参数非法");
    }
}

AlmSolverResult AlmSolver::solve(
    const std::vector<AlmManeuverEstimate>& estimates,
    const AlmPreprocessorResult& preprocessor_result,
    const Eigen::Vector2d& start_position, const ESDFMap& esdf_map,
    const VehicleFootprintModel& footprint_model) const {
    const AlmSolverProblem problem =
        buildProblem(estimates, preprocessor_result, start_position);
    const AlmEsdfPenalty esdf_penalty(esdf_map, footprint_model, esdf_config_);
    AlmSolverResult result;
    result.history.reserve(config_.max_outer_iterations);
    Eigen::VectorXd x = problem.initialGuess();
    // λ^0 = 0（纯软惩罚启动，不预设先验乘子）；首轮 ρ 取临时权重，首轮收敛
    // 后即被 2.5 节公式标定的 ρ^0 替换
    AlmMultiplierState multipliers;
    multipliers.rho = config_.first_round_rho;
    LBFGSpp::LBFGSParam<double> param;
    param.epsilon = config_.lbfgs_epsilon;
    param.epsilon_rel = config_.lbfgs_epsilon_rel;
    param.max_iterations = config_.lbfgs_max_iterations;
    param.m = config_.lbfgs_m;
    param.max_linesearch = config_.lbfgs_max_linesearch;
    param.linesearch = config_.lbfgs_linesearch_algo;
    param.ftol = config_.lbfgs_ftol;
    param.wolfe = config_.lbfgs_wolfe;
    double prev_violation_norm = std::numeric_limits<double>::infinity();
    MincoTrajectory final_trajectory;
    for (int outer = 0; outer < config_.max_outer_iterations; ++outer) {
        // 本轮实际使用的乘子快照（记录进 history，供更新公式单步验证）
        const AlmMultiplierState round_multipliers = multipliers;
        const Eigen::VectorXd x_before = x;
        auto objective = [&](const Eigen::VectorXd& vars,
                             Eigen::VectorXd& grad) -> double {
            return evaluateCostAndGradient(
                problem, esdf_penalty, round_multipliers, vars, &grad, nullptr);
        };
        // 内层：L-BFGS 无约束求解（从上一轮外层迭代的解热启动）
        try {
            LBFGSpp::LBFGSSolver<double, LBFGSpp::LineSearchBacktracking> lbfgs(
                param);
            double inner_cost = 0.0;
            result.total_lbfgs_iterations +=
                lbfgs.minimize(objective, x, inner_cost);
        } catch (const std::exception&) {
            // 内层失败：恢复到上一轮解（首轮恢复到预处理初值），显式标记
            // 失败并返回最后一次成功迭代的轨迹，不继续外层更新
            x = x_before;
            result.status = AlmSolverStatus::INNER_LBFGS_FAILED;
            try {
                final_trajectory = buildTrajectory(problem, x);
                const TerminalMetrics metrics =
                    computeTerminalMetrics(problem, final_trajectory);
                result.terminal_position_error = metrics.position_error;
                result.terminal_heading_error_deg = metrics.heading_error_deg;
                result.trajectory = final_trajectory;
            } catch (const std::exception&) {
                result.trajectory = MincoTrajectory{};
            }
            result.outer_iterations = static_cast<int>(result.history.size());
            return result;
        }
        try {
            final_trajectory = buildTrajectory(problem, x);
        } catch (const std::exception&) {
            // K(T) 奇异等极端数值情况：显式失败，不返回半成品轨迹
            result.status = AlmSolverStatus::TRAJECTORY_BUILD_FAILED;
            result.trajectory = MincoTrajectory{};
            result.outer_iterations = static_cast<int>(result.history.size());
            return result;
        }
        // 外层指标：复用与代价函数同一组辛普森节点，保证先离散后求导一致
        AlmCostBreakdown breakdown;
        Eigen::VectorXd grad_scratch(x.size());
        evaluateCostAndGradient(problem, esdf_penalty, round_multipliers, x,
                                &grad_scratch, &breakdown);
        const TerminalMetrics metrics =
            computeTerminalMetrics(problem, final_trajectory);
        // 首轮收敛后按 2.5 节公式标定 ρ^0 并替换临时权重（首轮求解本身
        // 仍使用 first_round_rho，标定值从首轮更新起生效）
        if (outer == 0) {
            result.rho_initial_calibrated = Clip(
                breakdown.j_s_prime / std::max(metrics.violation.squaredNorm(),
                                               config_.epsilon_rho),
                config_.rho_min, config_.rho_max);
            multipliers.rho = result.rho_initial_calibrated;
        }
        AlmOuterIterationRecord record;
        record.outer_index = outer;
        record.lambda_x = round_multipliers.lambda_x;
        record.lambda_y = round_multipliers.lambda_y;
        record.rho = round_multipliers.rho;
        record.terminal_violation = metrics.violation;
        record.position_error = metrics.position_error;
        record.heading_error_deg = metrics.heading_error_deg;
        record.j_s_prime = breakdown.j_s_prime;
        record.alm_terminal = breakdown.alm_terminal;
        record.esdf_penalty = breakdown.esdf_penalty;
        record.total_cost = breakdown.total;
        record.lbfgs_iterations = result.total_lbfgs_iterations;
        result.history.push_back(record);
        // 结果字段随每轮刷新，循环任意出口都保留最后一轮解
        result.terminal_position_error = metrics.position_error;
        result.terminal_heading_error_deg = metrics.heading_error_deg;
        result.lambda_x = round_multipliers.lambda_x;
        result.lambda_y = round_multipliers.lambda_y;
        result.rho = round_multipliers.rho;
        result.final_cost = breakdown.total;
        result.final_j_s_prime = breakdown.j_s_prime;
        result.trajectory = final_trajectory;
        // 双指标收敛判据：位置与朝向同时满足才宣告 ALM 收敛
        if (metrics.position_error <= config_.terminal_position_tolerance &&
            metrics.heading_error_deg <=
                config_.terminal_heading_tolerance_deg) {
            result.success = true;
            result.status = AlmSolverStatus::CONVERGED;
            break;
        }
        if (outer + 1 >= config_.max_outer_iterations) {
            break;
        }
        // 外层更新：λ^{k+1} = λ^k + ρ^k·C_f^k（首轮使用刚标定的 ρ^0）
        multipliers.lambda_x += multipliers.rho * metrics.violation.x();
        multipliers.lambda_y += multipliers.rho * metrics.violation.y();
        // ρ 更新：原论文无条件递增 ρ^{k+1}=min((1+γ)ρ^k, ρ_max)；门控启用时
        // 仅当误差未充分减小才提升（首轮无上一轮可比较，无条件提升）
        const double violation_norm = metrics.violation.norm();
        const bool increase_rho =
            !config_.use_rho_increase_gate || outer == 0 ||
            violation_norm > config_.rho_gate_kappa * prev_violation_norm;
        if (increase_rho) {
            multipliers.rho =
                std::min((1.0 + config_.rho_increase_factor) * multipliers.rho,
                         config_.rho_max);
        }
        prev_violation_norm = violation_norm;
    }
    const int num_segments = problem.numSegments();
    result.waypoints.reserve(num_segments > 0 ? num_segments - 1 : 0);
    for (int m = 0; m + 1 < num_segments; ++m) {
        result.waypoints.emplace_back(x[2 * m], x[2 * m + 1]);
    }
    result.durations.reserve(num_segments);
    for (int i = 0; i < num_segments; ++i) {
        result.durations.push_back(result.trajectory.duration(i));
    }
    result.final_arc_length = x[x.size() - 1];
    result.outer_iterations = static_cast<int>(result.history.size());
    return result;
}

AlmSolverProblem AlmSolver::buildProblem(
    const std::vector<AlmManeuverEstimate>& estimates,
    const AlmPreprocessorResult& preprocessor_result,
    const Eigen::Vector2d& start_position) const {
    if (estimates.empty()) {
        throw std::invalid_argument("初值估计不能为空");
    }
    if (!start_position.allFinite()) {
        throw std::invalid_argument("起点世界坐标必须为有限值");
    }
    AlmSolverProblem problem;
    problem.start_position = start_position;
    int total_segments = 0;
    for (const auto& estimate : estimates) {
        if (estimate.segments.empty()) {
            throw std::invalid_argument("每个 Maneuver 至少需要一个微观段");
        }
        if (!std::isfinite(estimate.start_theta) ||
            !std::isfinite(estimate.start_arc_length)) {
            throw std::invalid_argument("Maneuver 起点朝向/弧长必须为有限值");
        }
        total_segments += static_cast<int>(estimate.segments.size());
    }
    // 预处理结果与估计的结构性一致校验：段数/航点数匹配、时长为正有限、
    // 航点与终点弧长有限
    if (static_cast<int>(preprocessor_result.durations.size()) !=
            total_segments ||
        static_cast<int>(preprocessor_result.waypoints.size()) + 1 !=
            total_segments) {
        throw std::invalid_argument("预处理结果与初值估计的段数不一致");
    }
    for (const double duration : preprocessor_result.durations) {
        if (!std::isfinite(duration) || duration <= 0.0) {
            throw std::invalid_argument("预处理结果的段时长必须为正有限值");
        }
    }
    for (const auto& waypoint : preprocessor_result.waypoints) {
        if (!waypoint.allFinite()) {
            throw std::invalid_argument("预处理结果的内部航点必须为有限值");
        }
    }
    if (!std::isfinite(preprocessor_result.final_arc_length)) {
        throw std::invalid_argument("预处理结果的终点弧长必须为有限值");
    }
    // 起点边界：车辆从静止起步（速度/加速度均为 0）
    problem.start.theta = {estimates.front().start_theta, 0.0, 0.0};
    problem.start.s = {estimates.front().start_arc_length, 0.0, 0.0};
    // 终点边界与目标位姿：θ 取目标朝向（硬边界精确满足），s 的位置分量
    // 仅占位（s_f 是独立决策变量）；终点停车（速度/加速度为 0）
    const AlmSegmentEstimate& last_segment = estimates.back().segments.back();
    if (!last_segment.desired_position.allFinite() ||
        !std::isfinite(last_segment.theta)) {
        throw std::invalid_argument("目标位姿必须为有限值");
    }
    problem.target_position = last_segment.desired_position;
    problem.target_theta = last_segment.theta;
    problem.end.theta = {problem.target_theta, 0.0, 0.0};
    problem.end.s = {0.0, 0.0, 0.0};
    int global_index = 0;
    for (std::size_t m = 0; m < estimates.size(); ++m) {
        for (const auto& segment : estimates[m].segments) {
            if (!std::isfinite(segment.theta) ||
                !std::isfinite(segment.arc_length)) {
                throw std::invalid_argument(
                    "微观段估计的朝向/弧长必须为有限值");
            }
            ++global_index;
        }
        // 每个 Maneuver 末尾（非全局末尾）都是一个换挡点：物理上进入/离开
        // 换挡（含 PIVOT 原地转向）车辆必须停车，统一施加 ṡ² 软惩罚
        if (m + 1 < estimates.size()) {
            problem.cusp_segment_indices.push_back(global_index - 1);
        }
    }
    // 内层初值完全来自预处理结果（两阶段流程的第二阶段）
    problem.initial_waypoints = preprocessor_result.waypoints;
    problem.initial_durations = preprocessor_result.durations;
    problem.initial_final_arc_length = preprocessor_result.final_arc_length;
    return problem;
}

double AlmSolver::evaluateCostAndGradient(const AlmSolverProblem& problem,
                                          const AlmEsdfPenalty& esdf_penalty,
                                          const AlmMultiplierState& multipliers,
                                          const Eigen::VectorXd& x,
                                          Eigen::VectorXd* gradient,
                                          AlmCostBreakdown* breakdown) const {
    const int num_segments = problem.numSegments();
    const MincoTrajectory trajectory = buildTrajectory(problem, x);
    // ∂J/∂c（θ/s 两个维度，6xM）与 ∂J/∂T（M 维，仅显式部分）的累加器
    MincoTrajectory::CoeffMatrix dJ_dc_theta =
        MincoTrajectory::CoeffMatrix::Zero(MincoTrajectory::COEFFS_PER_SEG,
                                           num_segments);
    MincoTrajectory::CoeffMatrix dJ_dc_s = MincoTrajectory::CoeffMatrix::Zero(
        MincoTrajectory::COEFFS_PER_SEG, num_segments);
    Eigen::VectorXd dJ_dT = Eigen::VectorXd::Zero(num_segments);
    double j_s_prime = 0.0;
    double alm_terminal = 0.0;
    double esdf_penalty_cost = 0.0;
    // ---- 基础平滑目标 J_0：跃度积分闭式解 ----
    // 每段 ∫σ⃛²dt = Q(c)/T⁵，Q 为归一化系数的解析二次型（设计文档 1.3 节
    // 注明该积分存在闭式解，无需数值求积）：Q = 36c₃² + 192c₄² + 720c₅² +
    // 144c₃c₄ + 240c₃c₅ + 720c₄c₅；显式时长梯度 ∂/∂T = -5·J_i/T
    for (int i = 0; i < num_segments; ++i) {
        const double duration_i = trajectory.duration(i);
        const double inv_t5 = 1.0 / std::pow(duration_i, 5);
        for (int dim = 0; dim < 2; ++dim) {
            const auto& coeffs =
                (dim == 0) ? trajectory.coeffsTheta() : trajectory.coeffsS();
            const double weight =
                (dim == 0) ? config_.weight_jerk_theta : config_.weight_jerk_s;
            auto& dJ_dc = (dim == 0) ? dJ_dc_theta : dJ_dc_s;
            const double c3 = coeffs(3, i);
            const double c4 = coeffs(4, i);
            const double c5 = coeffs(5, i);
            const double q = 36.0 * c3 * c3 + 192.0 * c4 * c4 +
                             720.0 * c5 * c5 + 144.0 * c3 * c4 +
                             240.0 * c3 * c5 + 720.0 * c4 * c5;
            const double segment_cost = weight * q * inv_t5;
            j_s_prime += segment_cost;
            dJ_dc(3, i) +=
                weight * inv_t5 * (72.0 * c3 + 144.0 * c4 + 240.0 * c5);
            dJ_dc(4, i) +=
                weight * inv_t5 * (144.0 * c3 + 384.0 * c4 + 720.0 * c5);
            dJ_dc(5, i) +=
                weight * inv_t5 * (240.0 * c3 + 720.0 * c4 + 1440.0 * c5);
            dJ_dT(i) -= 5.0 * segment_cost / duration_i;
        }
    }
    // ---- 时间正则：ε_T·ΣT_i（J_0 的组成部分）----
    for (int i = 0; i < num_segments; ++i) {
        j_s_prime += config_.epsilon_time * trajectory.duration(i);
        dJ_dT(i) += config_.epsilon_time;
    }
    // ---- 物理约束惩罚：每段梯形积分，梯度经基函数行回传到多项式系数 ----
    const int num_physics = config_.physics_samples_per_segment;
    for (int i = 0; i < num_segments; ++i) {
        const double duration_i = trajectory.duration(i);
        for (int j = 0; j < num_physics; ++j) {
            const double tau = static_cast<double>(j) / (num_physics - 1);
            const double local_time = tau * duration_i;
            const double trapezoid =
                (j == 0 || j == num_physics - 1) ? 0.5 : 1.0;
            // 积分权重 ∝ T（dt = T·dτ）
            const double weight = duration_i * trapezoid / (num_physics - 1);
            ThetaSSample sample;
            sample.theta = trajectory.evaluateSegment(i, local_time, 0).x();
            sample.theta_dot = trajectory.evaluateSegment(i, local_time, 1).x();
            sample.theta_ddot =
                trajectory.evaluateSegment(i, local_time, 2).x();
            sample.s = trajectory.evaluateSegment(i, local_time, 0).y();
            sample.s_dot = trajectory.evaluateSegment(i, local_time, 1).y();
            sample.s_ddot = trajectory.evaluateSegment(i, local_time, 2).y();
            const PhysicalConstraintPenalties penalties =
                kinematics_.evaluatePenalties(sample);
            const double penalty_value =
                config_.weight_velocity * penalties.velocity.penalty +
                config_.weight_acceleration * penalties.acceleration.penalty +
                config_.weight_steer_angle * penalties.steer_angle.penalty +
                config_.weight_steer_rate * penalties.steer_rate.penalty;
            j_s_prime += weight * penalty_value;
            const double g_theta_dot =
                config_.weight_velocity *
                    penalties.velocity.gradient.d_theta_dot +
                config_.weight_acceleration *
                    penalties.acceleration.gradient.d_theta_dot +
                config_.weight_steer_angle *
                    penalties.steer_angle.gradient.d_theta_dot +
                config_.weight_steer_rate *
                    penalties.steer_rate.gradient.d_theta_dot;
            const double g_theta_ddot =
                config_.weight_velocity *
                    penalties.velocity.gradient.d_theta_ddot +
                config_.weight_acceleration *
                    penalties.acceleration.gradient.d_theta_ddot +
                config_.weight_steer_angle *
                    penalties.steer_angle.gradient.d_theta_ddot +
                config_.weight_steer_rate *
                    penalties.steer_rate.gradient.d_theta_ddot;
            const double g_s_dot =
                config_.weight_velocity * penalties.velocity.gradient.d_s_dot +
                config_.weight_acceleration *
                    penalties.acceleration.gradient.d_s_dot +
                config_.weight_steer_angle *
                    penalties.steer_angle.gradient.d_s_dot +
                config_.weight_steer_rate *
                    penalties.steer_rate.gradient.d_s_dot;
            const double g_s_ddot =
                config_.weight_velocity * penalties.velocity.gradient.d_s_ddot +
                config_.weight_acceleration *
                    penalties.acceleration.gradient.d_s_ddot +
                config_.weight_steer_angle *
                    penalties.steer_angle.gradient.d_s_ddot +
                config_.weight_steer_rate *
                    penalties.steer_rate.gradient.d_s_ddot;
            // ∂D^k/∂c_i = 实时间导数基函数行
            const auto basis_d1 =
                MincoTrajectory::DerivativeBasisRow(tau, 1, duration_i);
            const auto basis_d2 =
                MincoTrajectory::DerivativeBasisRow(tau, 2, duration_i);
            dJ_dc_theta.col(i) +=
                weight * (g_theta_dot * basis_d1.transpose() +
                          g_theta_ddot * basis_d2.transpose());
            dJ_dc_s.col(i) += weight * (g_s_dot * basis_d1.transpose() +
                                        g_s_ddot * basis_d2.transpose());
            // 显式时长梯度：积分权重 ∝T 贡献 weight/T·P；实时间导数
            // D^k∝T^{-k} 贡献 weight·Σ g_k·(-k/T)·D^k
            dJ_dT(i) +=
                weight * penalty_value / duration_i -
                weight / duration_i *
                    (g_theta_dot * sample.theta_dot +
                     2.0 * g_theta_ddot * sample.theta_ddot +
                     g_s_dot * sample.s_dot + 2.0 * g_s_ddot * sample.s_ddot);
        }
    }
    // ---- 段时长平衡约束：C_low=ε_low·mean(T)-T_i，C_upp=T_i-ε_upp·mean(T)
    double mean_duration = 0.0;
    for (int i = 0; i < num_segments; ++i) {
        mean_duration += trajectory.duration(i);
    }
    mean_duration /= num_segments;
    for (int i = 0; i < num_segments; ++i) {
        const double c_low = config_.duration_balance_lower * mean_duration -
                             trajectory.duration(i);
        if (c_low > 0.0) {
            j_s_prime +=
                config_.weight_duration_balance * c_low * c_low * c_low;
            // ∂C_low/∂T_j = ε_low/M - δ_ij
            const double grad_factor =
                3.0 * config_.weight_duration_balance * c_low * c_low;
            dJ_dT(i) -= grad_factor;
            for (int j = 0; j < num_segments; ++j) {
                dJ_dT(j) +=
                    grad_factor * config_.duration_balance_lower / num_segments;
            }
        }
        const double c_upp = trajectory.duration(i) -
                             config_.duration_balance_upper * mean_duration;
        if (c_upp > 0.0) {
            j_s_prime +=
                config_.weight_duration_balance * c_upp * c_upp * c_upp;
            // ∂C_upp/∂T_j = δ_ij - ε_upp/M
            const double grad_factor =
                3.0 * config_.weight_duration_balance * c_upp * c_upp;
            dJ_dT(i) += grad_factor;
            for (int j = 0; j < num_segments; ++j) {
                dJ_dT(j) -=
                    grad_factor * config_.duration_balance_upper / num_segments;
            }
        }
    }
    // ---- 换挡点 ṡ² 软惩罚（点态，无积分权重）----
    for (const int cusp_index : problem.cusp_segment_indices) {
        const double duration_g = trajectory.duration(cusp_index);
        const double s_dot_end =
            trajectory.evaluateSegment(cusp_index, duration_g, 1).y();
        j_s_prime += config_.weight_gear_cusp * s_dot_end * s_dot_end;
        const double g_s_dot = 2.0 * config_.weight_gear_cusp * s_dot_end;
        dJ_dc_s.col(cusp_index) +=
            g_s_dot *
            MincoTrajectory::DerivativeBasisRow(1.0, 1, duration_g).transpose();
        // 点态惩罚的显式时长梯度仅含导数缩放项 g·(-1/T)·ṡ
        dJ_dT(cusp_index) -= g_s_dot * s_dot_end / duration_g;
    }
    // ---- 换挡点 θ̇² 软惩罚（点态，与 ṡ² 同位置同结构）----
    for (const int cusp_index : problem.cusp_segment_indices) {
        const double duration_g = trajectory.duration(cusp_index);
        const double theta_dot_end =
            trajectory.evaluateSegment(cusp_index, duration_g, 1).x();
        j_s_prime +=
            config_.weight_gear_cusp_theta * theta_dot_end * theta_dot_end;
        const double g_theta_dot =
            2.0 * config_.weight_gear_cusp_theta * theta_dot_end;
        dJ_dc_theta.col(cusp_index) +=
            g_theta_dot *
            MincoTrajectory::DerivativeBasisRow(1.0, 1, duration_g).transpose();
        // 点态惩罚的显式时长梯度仅含导数缩放项 g·(-1/T)·θ̇
        dJ_dT(cusp_index) -= g_theta_dot * theta_dot_end / duration_g;
    }
    // ---- 空间项：PHR-ALM 终端 + ESDF 双重惩罚（共享同一组辛普森节点）----
    // 先离散后求导：代价与梯度共用同一组固定求积节点（由
    // computeSimpsonNodeData 统一产出，与终点指标计算共享），保证严格一致
    const int num_simpson = config_.simpson_subintervals;
    const std::vector<double> simpson_unit_weights =
        SimpsonUnitWeights(num_simpson);
    const SimpsonNodeData simpson_data =
        computeSimpsonNodeData(trajectory, problem.start_position);
    Eigen::Vector2d end_position = problem.start_position;
    for (int i = 0; i < num_segments; ++i) {
        end_position += simpson_data.segment_displacements[i];
    }
    // PHR-ALM 终端项：(ρ/2)·||C_f + λ/ρ||²，对末端位置的梯度为 ρ·C_f + λ
    const Eigen::Vector2d violation = end_position - problem.target_position;
    const double augmented_x =
        violation.x() + multipliers.lambda_x / multipliers.rho;
    const double augmented_y =
        violation.y() + multipliers.lambda_y / multipliers.rho;
    alm_terminal = 0.5 * multipliers.rho *
                   (augmented_x * augmented_x + augmented_y * augmented_y);
    // 位置相关项的后缀梯度：末端位置依赖全部节点，故以 ALM 终端梯度初始化
    // 后缀，再按节点逆序累加各节点的 ESDF 梯度（节点 (i,j) 的位置只依赖
    // 全局序不晚于它的节点）
    Eigen::Vector2d suffix_gradient(
        multipliers.rho * violation.x() + multipliers.lambda_x,
        multipliers.rho * violation.y() + multipliers.lambda_y);
    for (int i = num_segments - 1; i >= 0; --i) {
        const double duration_i = trajectory.duration(i);
        double segment_esdf_cost = 0.0;
        for (int j = num_simpson; j >= 0; --j) {
            const double tau = static_cast<double>(j) / num_simpson;
            // ESDF 惩罚按设计文档 2.4 节在时间轴上梯形离散积分（权重 ∝T）
            const double weight_trapezoid =
                duration_i / num_simpson *
                ((j == 0 || j == num_simpson) ? 0.5 : 1.0);
            const Eigen::Vector2d& position = simpson_data.node_positions[i][j];
            const double theta = simpson_data.node_theta[i][j];
            const double s_dot = simpson_data.node_s_dot[i][j];
            const AlmEsdfPoseCost pose_cost =
                esdf_penalty.evaluate(position.x(), position.y(), theta);
            segment_esdf_cost += weight_trapezoid * pose_cost.cost;
            suffix_gradient += weight_trapezoid * pose_cost.gradient.head<2>();
            // 节点 (i,j) 的位置偏导接收后缀梯度：∂P/∂θ = w_s·ṡ·(-sinθ, cosθ)，
            // ∂P/∂ṡ = w_s·(cosθ, sinθ)（w_s 为该节点的辛普森积分权重）；θ 另
            // 有 ESDF 惩罚随外圆旋转的直接梯度项
            const double weight_simpson =
                duration_i / (3.0 * num_simpson) * simpson_unit_weights[j];
            const double cos_theta = std::cos(theta);
            const double sin_theta = std::sin(theta);
            const double g_theta = weight_simpson * s_dot *
                                       (-sin_theta * suffix_gradient.x() +
                                        cos_theta * suffix_gradient.y()) +
                                   weight_trapezoid * pose_cost.gradient.z();
            const double g_s_dot =
                weight_simpson * (cos_theta * suffix_gradient.x() +
                                  sin_theta * suffix_gradient.y());
            dJ_dc_theta.col(i) += g_theta * MincoTrajectory::DerivativeBasisRow(
                                                tau, 0, duration_i)
                                                .transpose();
            dJ_dc_s.col(i) += g_s_dot * MincoTrajectory::DerivativeBasisRow(
                                            tau, 1, duration_i)
                                            .transpose();
        }
        esdf_penalty_cost += segment_esdf_cost;
        j_s_prime += segment_esdf_cost;
        // 显式时长梯度：ESDF 梯形权重 ∝T 贡献 cost/T；辛普森位置中权重 ∝T
        // 与 ṡ∝1/T 精确抵消、θ 节点值与 T 无关，无其余显式项
        dJ_dT(i) += segment_esdf_cost / duration_i;
    }
    // ---- 伴随反传：∂J/∂b = K(T)^{-T}·∂J/∂c，θ/s 两维共享同一分解 ----
    const MincoTrajectory::CoeffMatrix adjoint_theta =
        trajectory.solveAdjoint(dJ_dc_theta);
    const MincoTrajectory::CoeffMatrix adjoint_s =
        trajectory.solveAdjoint(dJ_dc_s);
    gradient->setZero(problem.variableCount());
    // 内部航点 d_m 在 b 中出现两次：段 m 末端位置（第 3 行）与段 m+1 起点
    // 位置（第 2 行）
    for (int m = 0; m + 1 < num_segments; ++m) {
        (*gradient)[2 * m] = adjoint_theta(3, m) + adjoint_theta(2, m + 1);
        (*gradient)[2 * m + 1] = adjoint_s(3, m) + adjoint_s(2, m + 1);
    }
    // 终点弧长 s_f 固定位于 b 的第 6M-3（0 基）个位置，即末段块第 3 行
    (*gradient)[problem.variableCount() - 1] = adjoint_s(3, num_segments - 1);
    // 时间变量：显式部分 + 经 K(T) 系统的隐式部分。K 的行按块组织（与
    // MincoTrajectory::AssembleK 的布局一一对应），每行对 T_i 的偏导为
    // -k/T_i 倍该行的带符号行值，故 ∂J/∂T_i = Σ (k/T_i)·adj_row·(±D^k)
    for (int i = 0; i < num_segments; ++i) {
        const double duration_i = trajectory.duration(i);
        double dT = dJ_dT(i);
        // 累加一条 K 行对 T_i 的隐式贡献：row 为块内行号，order 为该行的
        // 导数阶数 k，tau 为求值点（0=段首，1=段末），sign 为行符号，
        // block_row 为块行下标（对应伴随矩阵的列）
        auto accumulate_k_term = [&](int row, int order, double tau,
                                     double sign, int block_row) {
            const double local_time = tau * duration_i;
            const double value_theta =
                sign * trajectory.evaluateSegment(i, local_time, order).x();
            const double value_s =
                sign * trajectory.evaluateSegment(i, local_time, order).y();
            dT += (static_cast<double>(order) / duration_i) *
                  (adjoint_theta(row, block_row) * value_theta +
                   adjoint_s(row, block_row) * value_s);
        };
        if (i == 0) {
            // 块行 0：起点速度/加速度行（k=1,2，正号）
            accumulate_k_term(1, 1, 0.0, 1.0, 0);
            accumulate_k_term(2, 2, 0.0, 1.0, 0);
        } else {
            // 块行 i（i>0）第 0/1 行：本段起点 3/4 阶导数（负号）
            accumulate_k_term(0, 3, 0.0, -1.0, i);
            accumulate_k_term(1, 4, 0.0, -1.0, i);
            // 上块（块行 i-1 第 4/5 行）：本段起点 1/2 阶导数（负号）
            accumulate_k_term(4, 1, 0.0, -1.0, i - 1);
            accumulate_k_term(5, 2, 0.0, -1.0, i - 1);
        }
        // 块行 i 第 4/5 行：本段末端 1/2 阶导数（正号）
        accumulate_k_term(4, 1, 1.0, 1.0, i);
        accumulate_k_term(5, 2, 1.0, 1.0, i);
        if (i + 1 < num_segments) {
            // 下块（块行 i+1 第 0/1 行）：本段末端 3/4 阶导数（正号）
            accumulate_k_term(0, 3, 1.0, 1.0, i + 1);
            accumulate_k_term(1, 4, 1.0, 1.0, i + 1);
        }
        (*gradient)[2 * (num_segments - 1) + i] =
            dT * MincoTrajectory::TauToDurationDerivative(
                     x[2 * (num_segments - 1) + i]);
    }
    const double total = j_s_prime + alm_terminal;
    if (breakdown != nullptr) {
        breakdown->total = total;
        breakdown->j_s_prime = j_s_prime;
        breakdown->alm_terminal = alm_terminal;
        breakdown->esdf_penalty = esdf_penalty_cost;
    }
    return total;
}

MincoTrajectory AlmSolver::buildTrajectory(const AlmSolverProblem& problem,
                                           const Eigen::VectorXd& x) const {
    const int num_segments = problem.numSegments();
    if (x.size() != problem.variableCount()) {
        throw std::invalid_argument("决策变量维数与问题规模不匹配");
    }
    std::vector<Eigen::Vector2d> waypoints(num_segments > 0 ? num_segments - 1
                                                            : 0);
    for (int m = 0; m + 1 < num_segments; ++m) {
        waypoints[m] = Eigen::Vector2d(x[2 * m], x[2 * m + 1]);
    }
    std::vector<double> durations(num_segments);
    for (int i = 0; i < num_segments; ++i) {
        durations[i] =
            MincoTrajectory::TauToDuration(x[2 * (num_segments - 1) + i]);
    }
    MincoBoundaryCondition2d end = problem.end;
    end.s.pos = x[x.size() - 1];
    MincoTrajectory trajectory;
    trajectory.setTrajectory(problem.start, end, waypoints, durations);
    return trajectory;
}

AlmSolver::SimpsonNodeData AlmSolver::computeSimpsonNodeData(
    const MincoTrajectory& trajectory,
    const Eigen::Vector2d& start_position) const {
    const int num_segments = trajectory.numSegments();
    const int num_simpson = config_.simpson_subintervals;
    const std::vector<double> simpson_unit_weights =
        SimpsonUnitWeights(num_simpson);
    SimpsonNodeData data;
    data.node_theta.resize(num_segments);
    data.node_s_dot.resize(num_segments);
    data.node_positions.resize(num_segments);
    data.segment_displacements.assign(num_segments, Eigen::Vector2d::Zero());
    // 逐节点世界坐标从起点世界坐标开始累计；段位移保持相对量（不含起点偏移）
    Eigen::Vector2d running_position = start_position;
    for (int i = 0; i < num_segments; ++i) {
        const double duration_i = trajectory.duration(i);
        data.node_theta[i].resize(num_simpson + 1);
        data.node_s_dot[i].resize(num_simpson + 1);
        data.node_positions[i].resize(num_simpson + 1);
        Eigen::Vector2d segment_position = running_position;
        for (int j = 0; j <= num_simpson; ++j) {
            const double tau = static_cast<double>(j) / num_simpson;
            const double local_time = tau * duration_i;
            const double theta =
                trajectory.evaluateSegment(i, local_time, 0).x();
            const double s_dot =
                trajectory.evaluateSegment(i, local_time, 1).y();
            const double weight =
                duration_i / (3.0 * num_simpson) * simpson_unit_weights[j];
            // 逐节点世界坐标定义为辛普森累计部分和：代价函数与梯度反传共享
            // 同一离散映射，保证先离散后求导的严格一致
            segment_position +=
                weight * s_dot *
                Eigen::Vector2d(std::cos(theta), std::sin(theta));
            data.node_theta[i][j] = theta;
            data.node_s_dot[i][j] = s_dot;
            data.node_positions[i][j] = segment_position;
            data.segment_displacements[i] +=
                weight * s_dot *
                Eigen::Vector2d(std::cos(theta), std::sin(theta));
        }
        running_position += data.segment_displacements[i];
    }
    return data;
}

AlmSolver::TerminalMetrics AlmSolver::computeTerminalMetrics(
    const AlmSolverProblem& problem, const MincoTrajectory& trajectory) const {
    const SimpsonNodeData simpson_data =
        computeSimpsonNodeData(trajectory, problem.start_position);
    TerminalMetrics metrics;
    metrics.end_position = problem.start_position;
    for (int i = 0; i < trajectory.numSegments(); ++i) {
        metrics.end_position += simpson_data.segment_displacements[i];
    }
    metrics.violation = metrics.end_position - problem.target_position;
    metrics.position_error = metrics.violation.norm();
    // 终点朝向是 K(T) 的硬边界条件，本指标同时承担求解精度校验职责
    const double end_theta =
        trajectory.evaluate(trajectory.totalDuration(), 0).x();
    metrics.heading_error_deg =
        std::abs(NormalizeAngle(end_theta - problem.target_theta)) *
        (180.0 / std::acos(-1.0));
    return metrics;
}

std::vector<double> AlmSolver::SimpsonUnitWeights(int num_subintervals) {
    std::vector<double> weights(num_subintervals + 1, 2.0);
    weights.front() = 1.0;
    weights.back() = 1.0;
    for (int j = 1; j < num_subintervals; ++j) {
        weights[j] = (j % 2 == 1) ? 4.0 : 2.0;
    }
    return weights;
}

double AlmSolver::NormalizeAngle(double angle) {
    const double pi = std::acos(-1.0);
    angle = std::fmod(angle + pi, 2.0 * pi);
    if (angle < 0.0) {
        angle += 2.0 * pi;
    }
    return angle - pi;
}

double AlmSolver::Clip(double value, double lower, double upper) {
    return std::min(std::max(value, lower), upper);
}
}  // namespace apa_post_processor
