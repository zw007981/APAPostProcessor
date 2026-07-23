#include "alm_preprocessor.h"

#include <LBFGS.h>

#include <cmath>
#include <stdexcept>

namespace apa_post_processor {
Eigen::VectorXd AlmPreprocessorProblem::initialGuess() const {
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

AlmPreprocessor::AlmPreprocessor(AlmPreprocessorConfig config,
                                 BicycleKinematicsConfig kinematics_config)
    : config_(config), kinematics_(kinematics_config) {
    // 权重类字段：非负且有限
    if (config_.weight_endpoint_track < 0.0 ||
        !std::isfinite(config_.weight_endpoint_track) ||
        config_.weight_velocity < 0.0 ||
        !std::isfinite(config_.weight_velocity) ||
        config_.weight_acceleration < 0.0 ||
        !std::isfinite(config_.weight_acceleration) ||
        config_.weight_steer_angle < 0.0 ||
        !std::isfinite(config_.weight_steer_angle) ||
        config_.weight_steer_rate < 0.0 ||
        !std::isfinite(config_.weight_steer_rate) ||
        config_.weight_duration_balance < 0.0 ||
        !std::isfinite(config_.weight_duration_balance) ||
        config_.weight_gear_cusp < 0.0 ||
        !std::isfinite(config_.weight_gear_cusp) ||
        config_.weight_gear_cusp_theta < 0.0 ||
        !std::isfinite(config_.weight_gear_cusp_theta) ||
        config_.epsilon_time < 0.0 || !std::isfinite(config_.epsilon_time)) {
        throw std::invalid_argument("AlmPreprocessorConfig 权重必须非负有限");
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
    if (!(config_.convergence_position_tolerance > 0.0) ||
        !std::isfinite(config_.convergence_position_tolerance)) {
        throw std::invalid_argument("收敛位置容差必须为正有限值");
    }
    // L-BFGS 参数（与 BSplineSmoother 同一套校验约定）
    if (config_.lbfgs_max_iterations <= 0 || config_.lbfgs_epsilon <= 0.0 ||
        config_.lbfgs_epsilon_rel < 0.0 || config_.lbfgs_m <= 0 ||
        config_.lbfgs_max_linesearch <= 0 ||
        config_.lbfgs_linesearch_algo < 1 ||
        config_.lbfgs_linesearch_algo > 3 || config_.lbfgs_ftol <= 0.0 ||
        config_.lbfgs_ftol >= 0.5 ||
        config_.lbfgs_wolfe <= config_.lbfgs_ftol ||
        config_.lbfgs_wolfe >= 1.0) {
        throw std::invalid_argument("AlmPreprocessorConfig L-BFGS 参数非法");
    }
}

AlmPreprocessorResult AlmPreprocessor::preprocess(
    const std::vector<AlmManeuverEstimate>& estimates,
    const Eigen::Vector2d& start_position) const {
    const AlmPreprocessorProblem problem =
        buildProblem(estimates, start_position);
    AlmPreprocessorResult result;
    Eigen::VectorXd x = problem.initialGuess();
    auto objective = [this, &problem](const Eigen::VectorXd& vars,
                                      Eigen::VectorXd& grad) -> double {
        return evaluateCostAndGradient(problem, vars, &grad);
    };
    LBFGSpp::LBFGSParam<double> param;
    param.epsilon = config_.lbfgs_epsilon;
    param.epsilon_rel = config_.lbfgs_epsilon_rel;
    param.max_iterations = config_.lbfgs_max_iterations;
    param.m = config_.lbfgs_m;
    param.max_linesearch = config_.lbfgs_max_linesearch;
    param.linesearch = config_.lbfgs_linesearch_algo;
    param.ftol = config_.lbfgs_ftol;
    param.wolfe = config_.lbfgs_wolfe;
    double final_cost = 0.0;
    // 按线搜索算法分支：Armijo(1) / Wolfe(2) / Strong Wolfe(3) 使用
    // LineSearchBacktracking（与 BSplineSmoother 同一套接入方式）
    try {
        LBFGSpp::LBFGSSolver<double, LBFGSpp::LineSearchBacktracking> solver(
            param);
        result.lbfgs_iterations = solver.minimize(objective, x, final_cost);
        result.optimizer_converged = true;
    } catch (const std::exception&) {
        // 优化器失败：显式标记不收敛，仍尝试在最终迭代点重建轨迹以报告指标，
        // 但 success 保持 false，不静默返回看似正常的结果
        result.optimizer_converged = false;
    }
    result.final_cost = final_cost;
    // 终态重建：K(T) 奇异等极端情况按失败处理（不返回半成品轨迹）
    try {
        result.trajectory = buildTrajectory(problem, x);
    } catch (const std::exception&) {
        result.trajectory = MincoTrajectory{};
        result.success = false;
        return result;
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
    result.segment_end_positions =
        computeSegmentEndPositions(problem, result.trajectory);
    for (int m = 0; m < num_segments; ++m) {
        result.max_endpoint_error = std::max(
            result.max_endpoint_error,
            (result.segment_end_positions[m] - problem.track_positions[m])
                .norm());
    }
    for (const int cusp_index : problem.cusp_segment_indices) {
        result.max_cusp_speed = std::max(
            result.max_cusp_speed,
            std::abs(result.trajectory
                         .evaluateSegment(cusp_index,
                                          result.durations[cusp_index], 1)
                         .y()));
    }
    result.success =
        result.optimizer_converged &&
        result.max_endpoint_error <= config_.convergence_position_tolerance;
    return result;
}

AlmPreprocessorProblem AlmPreprocessor::buildProblem(
    const std::vector<AlmManeuverEstimate>& estimates,
    const Eigen::Vector2d& start_position) const {
    if (estimates.empty()) {
        throw std::invalid_argument("初值估计不能为空");
    }
    if (!start_position.allFinite()) {
        throw std::invalid_argument("起点世界坐标必须为有限值");
    }
    AlmPreprocessorProblem problem;
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
    problem.track_positions.reserve(total_segments);
    problem.initial_durations.reserve(total_segments);
    problem.initial_waypoints.reserve(total_segments - 1);
    problem.cusp_segment_indices.reserve(estimates.size() - 1);
    // 起点边界：车辆从静止起步（速度/加速度均为 0）
    problem.start.theta = {estimates.front().start_theta, 0.0, 0.0};
    problem.start.s = {estimates.front().start_arc_length, 0.0, 0.0};
    // 终点边界：θ 取前端估计的最终朝向（硬边界精确满足），s 的位置分量仅
    // 占位（s_f 是独立决策变量，求值时覆盖）；终点停车（速度/加速度为 0）
    problem.end.theta = {estimates.back().segments.back().theta, 0.0, 0.0};
    problem.end.s = {0.0, 0.0, 0.0};
    int global_index = 0;
    for (std::size_t m = 0; m < estimates.size(); ++m) {
        for (const auto& segment : estimates[m].segments) {
            if (!segment.desired_position.allFinite() ||
                !std::isfinite(segment.theta) ||
                !std::isfinite(segment.arc_length) ||
                !std::isfinite(segment.duration) || segment.duration <= 0.0) {
                throw std::invalid_argument(
                    "微观段估计的期望位置/朝向/弧长必须有限，时长必须为正");
            }
            problem.track_positions.push_back(segment.desired_position);
            problem.initial_durations.push_back(segment.duration);
            // 全局最后一段的终点不是内部航点（由终点边界与 s_f 决定）
            if (global_index + 1 < total_segments) {
                problem.initial_waypoints.emplace_back(segment.theta,
                                                       segment.arc_length);
            }
            ++global_index;
        }
        // 每个 Maneuver 末尾（非全局末尾）都是一个换挡点：物理上进入/离开
        // 换挡（含 PIVOT 原地转向）车辆必须停车，统一施加 ṡ² 软惩罚
        if (m + 1 < estimates.size()) {
            problem.cusp_segment_indices.push_back(global_index - 1);
        }
    }
    problem.initial_final_arc_length =
        estimates.back().segments.back().arc_length;
    return problem;
}

double AlmPreprocessor::evaluateCostAndGradient(
    const AlmPreprocessorProblem& problem, const Eigen::VectorXd& x,
    Eigen::VectorXd* gradient) const {
    const int num_segments = problem.numSegments();
    const MincoTrajectory trajectory = buildTrajectory(problem, x);
    // ∂J/∂c（θ/s 两个维度，6xM）与 ∂J/∂T（M 维，仅显式部分）的累加器
    MincoTrajectory::CoeffMatrix dJ_dc_theta =
        MincoTrajectory::CoeffMatrix::Zero(MincoTrajectory::COEFFS_PER_SEG,
                                           num_segments);
    MincoTrajectory::CoeffMatrix dJ_dc_s = MincoTrajectory::CoeffMatrix::Zero(
        MincoTrajectory::COEFFS_PER_SEG, num_segments);
    Eigen::VectorXd dJ_dT = Eigen::VectorXd::Zero(num_segments);
    double cost = 0.0;
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
            cost += weight * penalty_value;
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
    // ---- 逐段终点跟踪：辛普森积分还原世界坐标，位置误差梯度按段后缀和 ----
    // 先离散后求导：代价与梯度共用同一组固定求积节点（由
    // computeSimpsonNodeData 统一产出，与结果指标计算共享），保证严格一致
    const int num_simpson = config_.simpson_subintervals;
    const std::vector<double> simpson_unit_weights =
        SimpsonUnitWeights(num_simpson);
    const SimpsonNodeData simpson_data = computeSimpsonNodeData(trajectory);
    std::vector<Eigen::Vector2d> end_positions(num_segments);
    Eigen::Vector2d running_position = problem.start_position;
    for (int i = 0; i < num_segments; ++i) {
        running_position += simpson_data.segment_displacements[i];
        end_positions[i] = running_position;
    }
    for (int m = 0; m < num_segments; ++m) {
        cost += config_.weight_endpoint_track *
                (end_positions[m] - problem.track_positions[m]).squaredNorm();
    }
    // 段 i 内节点的位置误差梯度系数 G_i = Σ_{m>=i} 2·w·(p_f^m - p_w0^m)
    Eigen::Vector2d suffix_gradient = Eigen::Vector2d::Zero();
    for (int i = num_segments - 1; i >= 0; --i) {
        const double duration_i = trajectory.duration(i);
        suffix_gradient += 2.0 * config_.weight_endpoint_track *
                           (end_positions[i] - problem.track_positions[i]);
        for (int j = 0; j <= num_simpson; ++j) {
            const double tau = static_cast<double>(j) / num_simpson;
            const double weight =
                duration_i / (3.0 * num_simpson) * simpson_unit_weights[j];
            const double theta = simpson_data.node_theta[i][j];
            const double s_dot = simpson_data.node_s_dot[i][j];
            const double g_theta = weight * s_dot *
                                   (-std::sin(theta) * suffix_gradient.x() +
                                    std::cos(theta) * suffix_gradient.y());
            const double g_s_dot =
                weight * (std::cos(theta) * suffix_gradient.x() +
                          std::sin(theta) * suffix_gradient.y());
            dJ_dc_theta.col(i) += g_theta * MincoTrajectory::DerivativeBasisRow(
                                                tau, 0, duration_i)
                                                .transpose();
            dJ_dc_s.col(i) += g_s_dot * MincoTrajectory::DerivativeBasisRow(
                                            tau, 1, duration_i)
                                            .transpose();
            // 显式时长梯度恒为 0：辛普森权重 ∝T 与 ṡ∝1/T 精确抵消，
            // θ 节点值在固定系数下与 T 无关
        }
    }
    // ---- 段时长平衡约束：C_low=ε_low·mean(T)-T_i，C_upp=T_i-ε_upp·mean(T)
    // ----
    double mean_duration = 0.0;
    for (int i = 0; i < num_segments; ++i) {
        mean_duration += trajectory.duration(i);
    }
    mean_duration /= num_segments;
    for (int i = 0; i < num_segments; ++i) {
        const double c_low = config_.duration_balance_lower * mean_duration -
                             trajectory.duration(i);
        if (c_low > 0.0) {
            cost += config_.weight_duration_balance * c_low * c_low * c_low;
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
            cost += config_.weight_duration_balance * c_upp * c_upp * c_upp;
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
    // ---- 时间正则：ε_T·ΣT_i，消除 T 方向的平坦退化 ----
    for (int i = 0; i < num_segments; ++i) {
        cost += config_.epsilon_time * trajectory.duration(i);
        dJ_dT(i) += config_.epsilon_time;
    }
    // ---- 换挡点 ṡ² 软惩罚（点态，无积分权重）----
    for (const int cusp_index : problem.cusp_segment_indices) {
        const double duration_g = trajectory.duration(cusp_index);
        const double s_dot_end =
            trajectory.evaluateSegment(cusp_index, duration_g, 1).y();
        cost += config_.weight_gear_cusp * s_dot_end * s_dot_end;
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
        cost += config_.weight_gear_cusp_theta * theta_dot_end * theta_dot_end;
        const double g_theta_dot =
            2.0 * config_.weight_gear_cusp_theta * theta_dot_end;
        dJ_dc_theta.col(cusp_index) +=
            g_theta_dot *
            MincoTrajectory::DerivativeBasisRow(1.0, 1, duration_g).transpose();
        // 点态惩罚的显式时长梯度仅含导数缩放项 g·(-1/T)·θ̇
        dJ_dT(cusp_index) -= g_theta_dot * theta_dot_end / duration_g;
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
    return cost;
}

MincoTrajectory AlmPreprocessor::buildTrajectory(
    const AlmPreprocessorProblem& problem, const Eigen::VectorXd& x) const {
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

std::vector<Eigen::Vector2d> AlmPreprocessor::computeSegmentEndPositions(
    const AlmPreprocessorProblem& problem,
    const MincoTrajectory& trajectory) const {
    const SimpsonNodeData simpson_data = computeSimpsonNodeData(trajectory);
    std::vector<Eigen::Vector2d> positions;
    positions.reserve(problem.numSegments());
    Eigen::Vector2d running_position = problem.start_position;
    for (int i = 0; i < problem.numSegments(); ++i) {
        running_position += simpson_data.segment_displacements[i];
        positions.push_back(running_position);
    }
    return positions;
}

AlmPreprocessor::SimpsonNodeData AlmPreprocessor::computeSimpsonNodeData(
    const MincoTrajectory& trajectory) const {
    const int num_segments = trajectory.numSegments();
    const int num_simpson = config_.simpson_subintervals;
    const std::vector<double> simpson_unit_weights =
        SimpsonUnitWeights(num_simpson);
    SimpsonNodeData data;
    data.node_theta.resize(num_segments);
    data.node_s_dot.resize(num_segments);
    data.segment_displacements.assign(num_segments, Eigen::Vector2d::Zero());
    for (int i = 0; i < num_segments; ++i) {
        const double duration_i = trajectory.duration(i);
        data.node_theta[i].resize(num_simpson + 1);
        data.node_s_dot[i].resize(num_simpson + 1);
        for (int j = 0; j <= num_simpson; ++j) {
            const double tau = static_cast<double>(j) / num_simpson;
            const double local_time = tau * duration_i;
            data.node_theta[i][j] =
                trajectory.evaluateSegment(i, local_time, 0).x();
            data.node_s_dot[i][j] =
                trajectory.evaluateSegment(i, local_time, 1).y();
            const double weight =
                duration_i / (3.0 * num_simpson) * simpson_unit_weights[j];
            data.segment_displacements[i] +=
                weight * data.node_s_dot[i][j] *
                Eigen::Vector2d(std::cos(data.node_theta[i][j]),
                                std::sin(data.node_theta[i][j]));
        }
    }
    return data;
}

std::vector<double> AlmPreprocessor::SimpsonUnitWeights(int num_subintervals) {
    std::vector<double> weights(num_subintervals + 1, 2.0);
    weights.front() = 1.0;
    weights.back() = 1.0;
    for (int j = 1; j < num_subintervals; ++j) {
        weights[j] = (j % 2 == 1) ? 4.0 : 2.0;
    }
    return weights;
}
}  // namespace apa_post_processor
