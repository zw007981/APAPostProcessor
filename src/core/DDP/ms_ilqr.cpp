#include "ms_ilqr.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace apa_post_processor {
MsIlqrSolver::MsIlqrSolver(MsIlqrConfig config, const BicycleDynamics* dynamics,
                           const DdpCostEvaluator* cost_evaluator)
    : config_(config), dynamics_(dynamics), cost_evaluator_(cost_evaluator) {
    if (dynamics_ == nullptr || cost_evaluator_ == nullptr) {
        throw std::invalid_argument("MsIlqrSolver: 动力学与代价求值层必须非空");
    }
    if (!(config_.jerk_max > 0.0) || !(config_.steer_accel_max > 0.0) ||
        !std::isfinite(config_.jerk_max) ||
        !std::isfinite(config_.steer_accel_max)) {
        throw std::invalid_argument("MsIlqrSolver: 控制盒幅值必须为正有限");
    }
    if (config_.max_iterations <= 0 || !(config_.cost_change_tol > 0.0) ||
        !(config_.gradient_tol > 0.0)) {
        throw std::invalid_argument("MsIlqrSolver: 迭代上限与收敛容差必须为正");
    }
    if (!(config_.reg_min > 0.0) || !(config_.reg_max >= config_.reg_min) ||
        !(config_.reg_initial >= config_.reg_min &&
          config_.reg_initial <= config_.reg_max) ||
        !(config_.reg_increase > 1.0) ||
        !(config_.reg_decrease > 0.0 && config_.reg_decrease < 1.0)) {
        throw std::invalid_argument(
            "MsIlqrSolver: 正则化参数必须满足 0<min<=initial<=max、inc>1、"
            "0<dec<1");
    }
    if (!(config_.armijo_gamma > 0.0 && config_.armijo_gamma < 1.0) ||
        !(config_.backtrack_beta > 0.0 && config_.backtrack_beta < 1.0) ||
        config_.max_backtracks <= 0) {
        throw std::invalid_argument(
            "MsIlqrSolver: 线搜索参数必须满足 0<γ<1、0<β<1、回溯上限为正");
    }
    if (!(config_.merit_mu0 > 0.0) ||
        !(config_.merit_rho > 0.0 && config_.merit_rho < 1.0) ||
        !(config_.merit_kappa_d >= 0.0) ||
        !(config_.inter_segment_weight >= 0.0)) {
        throw std::invalid_argument("MsIlqrSolver: merit/段间惩罚参数非法");
    }
    rho_reg_ = config_.reg_initial;
    merit_mu_ = config_.merit_mu0;
}

MsIlqrResult MsIlqrSolver::solve(
    const DdpReference& reference, const DdpCostMultiplierState& multipliers,
    const DdpCostInput& cost_input,
    const DdpAlignedVec<DdpState>& initial_states,
    const DdpAlignedVec<DdpControl>& initial_controls) {
    const std::size_t num_poses = reference.poses.size();
    if (num_poses < 2) {
        throw std::invalid_argument("MsIlqrSolver: 参考位姿数量必须 >= 2");
    }
    const std::size_t num_steps = num_poses - 1;
    if (initial_states.size() != num_poses) {
        throw std::invalid_argument("MsIlqrSolver: 状态初值数量必须为 N+1");
    }
    if (initial_controls.size() != num_steps) {
        throw std::invalid_argument("MsIlqrSolver: 控制初值数量必须为 N");
    }
    if (!std::isfinite(reference.dt) || reference.dt <= 0.0) {
        throw std::invalid_argument("MsIlqrSolver: 参考轨迹 dt 必须为正有限值");
    }
    prepareWorkspace(num_steps);
    setShootingLookup(reference.shooting_nodes);
    setNominalTrajectory(reference, initial_states, initial_controls);
    evaluateNominal(reference, multipliers, cost_input);
    computeJacobians(reference);
    backward_pass_count_ = 0;
    linear_rollout_count_ = 0;
    nonlinear_rollout_count_ = 0;
    qp_factorization_count_ = 0;
    history_.clear();
    MsIlqrResult result;
    result.initial_cost = total_cost_;
    result.initial_defect_norm = defect_norm_;
    for (int iter = 1; iter <= config_.max_iterations; ++iter) {
        const double cost_prev = total_cost_;
        double accepted_alpha = 0.0;
        double accepted_cost = 0.0;
        int passes_this_iter = 0;
        int trials_this_iter = 0;
        // 正则化重试循环：QP 未收敛或线搜索耗尽都不能简单放弃本轮，必须
        // 增大 ρ_reg 并重跑整个后向传递（仅缩小步长无法脱困）；ρ_reg 超
        // 上限才判定本轮内层失败并上报
        while (true) {
            ++passes_this_iter;
            if (!backwardPass()) {
                if (!increaseReg()) {
                    result.status = MsIlqrStatus::REGULARIZATION_OVERFLOW;
                    result.iterations = iter - 1;
                    result.final_cost = total_cost_;
                    result.final_defect_norm = defect_norm_;
                    return result;
                }
                continue;
            }
            linearRollout();
            updateMeritMu();
            const double merit_prev = total_cost_ + merit_mu_ * defect_norm_;
            const std::int64_t trials_before = nonlinear_rollout_count_;
            if (lineSearch(reference, multipliers, cost_input, merit_prev,
                           &accepted_alpha, &accepted_cost)) {
                trials_this_iter =
                    static_cast<int>(nonlinear_rollout_count_ - trials_before);
                break;
            }
            if (!increaseReg()) {
                result.status = MsIlqrStatus::REGULARIZATION_OVERFLOW;
                result.iterations = iter - 1;
                result.final_cost = total_cost_;
                result.final_defect_norm = defect_norm_;
                return result;
            }
        }
        decreaseReg();
        acceptCandidate(accepted_alpha);
        computeJacobians(reference);
        MsIlqrIterationRecord record;
        record.iteration = iter;
        record.cost = total_cost_;
        record.defect_norm = defect_norm_;
        record.merit = total_cost_ + merit_mu_ * defect_norm_;
        record.merit_mu = merit_mu_;
        record.alpha = accepted_alpha;
        record.ec1 = ec1_;
        record.ec2 = ec2_;
        record.backward_passes = passes_this_iter;
        record.line_search_trials = trials_this_iter;
        history_.push_back(record);
        result.iterations = iter;
        const double rel_change = std::abs(cost_prev - total_cost_) /
                                  std::max(std::abs(cost_prev), 1e-12);
        if (rel_change < config_.cost_change_tol) {
            result.status = MsIlqrStatus::CONVERGED_COST;
            break;
        }
        if (max_qu_norm_ < config_.gradient_tol) {
            result.status = MsIlqrStatus::CONVERGED_GRADIENT;
            break;
        }
    }
    result.final_cost = total_cost_;
    result.final_defect_norm = defect_norm_;
    return result;
}

void MsIlqrSolver::prepareWorkspace(std::size_t num_steps) {
    num_steps_ = num_steps;
    const std::size_t nodes = num_steps + 1;
    is_shooting_.assign(nodes, false);
    states_.resize(nodes);
    cand_states_.resize(nodes);
    defects_.resize(nodes);
    controls_.resize(num_steps);
    cand_controls_.resize(num_steps);
    jac_A_.resize(num_steps);
    jac_B_.resize(num_steps);
    q_x_.resize(num_steps);
    q_u_.resize(num_steps);
    q_uu_.resize(num_steps);
    value_S_.resize(nodes);
    value_s_.resize(nodes);
    down_S_.resize(num_steps);
    down_s_.resize(num_steps);
    dx_lin_.resize(nodes);
    du_lin_.resize(num_steps);
    // 首轮 QP 初始点/钳制集只在（重）分配时清零；N 不变时保留跨 solve
    // 热启动数据（错误活动集会被 QP 内部梯度重判自动释放，不影响正确性；
    // 代价是若新问题的活动集与热启动猜测差异较大，个别步可能多做 1~2
    // 次分解才识别出正确活动集——纯性能影响，通常远小于冷启动全链）
    if (feedforward_.size() != num_steps) {
        feedforward_.assign(num_steps, DdpControl::Zero());
        clamped_.assign(num_steps,
                        std::array<bool, DDP_CONTROL_DIM>{false, false});
        has_clamped_history_ = false;
    }
    gain_K_.resize(num_steps);
    history_.reserve(static_cast<std::size_t>(config_.max_iterations));
}

void MsIlqrSolver::setShootingLookup(
    const std::vector<std::size_t>& shooting_nodes) {
    is_shooting_.assign(num_steps_ + 1, false);
    for (const std::size_t node : shooting_nodes) {
        if (node > num_steps_) {
            throw std::invalid_argument("MsIlqrSolver: 打靶节点索引越界");
        }
        is_shooting_[node] = true;
    }
}

void MsIlqrSolver::setNominalTrajectory(
    const DdpReference& reference,
    const DdpAlignedVec<DdpState>& initial_states,
    const DdpAlignedVec<DdpControl>& initial_controls) {
    dt_ = reference.dt;
    states_[0] = initial_states[0];
    defects_[0].setZero();
    // 初始名义建立：零反馈开环积分，滚动状态被动力学覆写、打靶状态直接
    // 注入初值，缺陷仅在打靶节点非零
    for (std::size_t k = 0; k < num_steps_; ++k) {
        controls_[k] = initial_controls[k];
        const DdpState integral =
            dynamics_->step(states_[k], controls_[k], dt_);
        if (is_shooting_[k + 1]) {
            defects_[k + 1] = integral - initial_states[k + 1];
            states_[k + 1] = initial_states[k + 1];
        } else {
            states_[k + 1] = integral;
            defects_[k + 1].setZero();
        }
    }
    defect_norm_ = DefectNorm(defects_);
}

void MsIlqrSolver::evaluateNominal(const DdpReference& reference,
                                   const DdpCostMultiplierState& multipliers,
                                   const DdpCostInput& cost_input) {
    cost_eval_ = cost_evaluator_->evaluate(reference, states_, controls_,
                                           multipliers, cost_input);
    total_cost_ = cost_eval_.total_cost;
}

void MsIlqrSolver::computeJacobians(const DdpReference& reference) {
    dt_ = reference.dt;
    for (std::size_t k = 0; k < num_steps_; ++k) {
        dynamics_->jacobians(states_[k], controls_[k], dt_, &jac_A_[k],
                             &jac_B_[k]);
    }
}

bool MsIlqrSolver::backwardPass() {
    ++backward_pass_count_;
    const auto& stages = cost_eval_.stages;
    const DdpStateHessian qd_mat =
        config_.inter_segment_weight * DdpStateHessian::Identity();
    DdpStateHessian value_hessian = stages[num_steps_].lxx;
    DdpState value_gradient = stages[num_steps_].lx;
    value_S_[num_steps_] = value_hessian;
    value_s_[num_steps_] = value_gradient;
    dv1_ = 0.0;
    dv2_ = 0.0;
    max_qu_norm_ = 0.0;
    for (std::size_t k = num_steps_; k-- > 0;) {
        // 可选段间惩罚：回推经过打靶节点 k+1 时先注入再装配（默认关闭）
        if (config_.inter_segment_weight > 0.0 && is_shooting_[k + 1]) {
            value_gradient -= config_.inter_segment_weight * defects_[k + 1];
            value_hessian += qd_mat;
        }
        down_S_[k] = value_hessian;
        down_s_[k] = value_gradient;
        const auto& stage = stages[k];
        const DdpStateJacobian& jac_a = jac_A_[k];
        const DdpControlJacobian& jac_b = jac_B_[k];
        // 缺陷修正：ẑ = s' + S'·d[k+1]（右端索引；非打靶节点 d 恒为零）
        const DdpState z = value_gradient + value_hessian * defects_[k + 1];
        const DdpState q_x = stage.lx + jac_a.transpose() * z;
        const DdpControl q_u = stage.lu + jac_b.transpose() * z;
        const DdpStateHessian q_xx =
            stage.lxx + jac_a.transpose() * value_hessian * jac_a;
        const DdpControlHessian q_uu =
            stage.luu + jac_b.transpose() * value_hessian * jac_b +
            rho_reg_ * DdpControlHessian::Identity();
        const DdpControlStateHessian q_ux =
            stage.lux + jac_b.transpose() * value_hessian * jac_a;
        // 盒约束 QP（H 已含正则化）：回推顺序上一步的钳制集热启动，
        // 分解每步必然重做
        BoxQpSolver<>::Problem problem;
        problem.hessian = q_uu;
        problem.gradient = q_u;
        problem.lower =
            DdpControl(-config_.jerk_max, -config_.steer_accel_max) -
            controls_[k];
        problem.upper = DdpControl(config_.jerk_max, config_.steer_accel_max) -
                        controls_[k];
        problem.initial = feedforward_[k];
        if (k + 1 < num_steps_) {
            problem.warm_start = true;
            problem.initial_clamped = clamped_[k + 1];
        } else if (has_clamped_history_) {
            problem.warm_start = true;
            problem.initial_clamped = clamped_[k];
        }
        const BoxQpSolver<>::Result qp_result = qp_.solve(problem);
        if (qp_result.status != BoxQpSolver<>::Status::CONVERGED) {
            return false;
        }
        qp_factorization_count_ += qp_result.factorizations;
        feedforward_[k] = qp_result.x;
        gain_K_[k] = -BoxQpSolver<>::SolveFreeExpanded(qp_result, q_ux);
        clamped_[k] = qp_result.clamped;
        q_x_[k] = q_x;
        q_u_[k] = q_u;
        q_uu_[k] = q_uu;
        // 价值回传（K/δũ 形式：任意活动集下均良定义，Q_uu 取含正则化版本）
        DdpStateHessian next_hessian =
            q_xx + gain_K_[k].transpose() * q_uu * gain_K_[k] +
            gain_K_[k].transpose() * q_ux + q_ux.transpose() * gain_K_[k];
        next_hessian = 0.5 * (next_hessian + next_hessian.transpose());
        const DdpState next_gradient =
            q_x + gain_K_[k].transpose() * q_uu * feedforward_[k] +
            gain_K_[k].transpose() * q_u + q_ux.transpose() * feedforward_[k];
        dv1_ += feedforward_[k].dot(q_u);
        dv2_ += (feedforward_[k].transpose() * q_uu * feedforward_[k]).value();
        max_qu_norm_ = std::max(max_qu_norm_, q_u.cwiseAbs().maxCoeff());
        value_S_[k] = next_hessian;
        value_s_[k] = next_gradient;
        value_hessian = next_hessian;
        value_gradient = next_gradient;
    }
    has_clamped_history_ = true;
    return true;
}

void MsIlqrSolver::linearRollout() {
    ++linear_rollout_count_;
    const auto& stages = cost_eval_.stages;
    dx_lin_[0].setZero();
    ec1_ = 0.0;
    ec2_ = 0.0;
    // α=1 的线性化方向传播：δu^l = δũ + K·δx^l，δx^l⁺ = A·δx^l + B·δu^l + d'
    // （打靶节点缺陷项进入方向；非打靶节点 d 恒为零）
    for (std::size_t k = 0; k < num_steps_; ++k) {
        du_lin_[k] = feedforward_[k] + gain_K_[k] * dx_lin_[k];
        dx_lin_[k + 1] =
            jac_A_[k] * dx_lin_[k] + jac_B_[k] * du_lin_[k] + defects_[k + 1];
        const auto& stage = stages[k];
        ec1_ += stage.lx.dot(dx_lin_[k]) + stage.lu.dot(du_lin_[k]);
        ec2_ +=
            (dx_lin_[k].transpose() * stage.lxx * dx_lin_[k]).value() +
            (2.0 * du_lin_[k].transpose() * stage.lux * dx_lin_[k]).value() +
            (du_lin_[k].transpose() * stage.luu * du_lin_[k]).value();
    }
    const auto& terminal = stages[num_steps_];
    ec1_ += terminal.lx.dot(dx_lin_[num_steps_]);
    ec2_ +=
        (dx_lin_[num_steps_].transpose() * terminal.lxx * dx_lin_[num_steps_])
            .value();
}

void MsIlqrSolver::updateMeritMu() {
    // 缺陷接近归零时不刷新权重（κ_d 门控）
    if (defect_norm_ <= config_.merit_kappa_d) {
        return;
    }
    // 上游求值异常（NaN/Inf）时保持既有权重，避免污染 µ_m
    if (!std::isfinite(defect_norm_) || !std::isfinite(ec1_) ||
        !std::isfinite(ec2_)) {
        return;
    }
    const double threshold =
        config_.merit_mu0 + std::abs(expectedChange(1.0)) /
                                ((1.0 - config_.merit_rho) * defect_norm_);
    merit_mu_ = std::max({merit_mu_, threshold, config_.merit_mu0});
}

double MsIlqrSolver::nonlinearRollout(double alpha,
                                      const DdpReference& reference,
                                      const DdpCostMultiplierState& multipliers,
                                      const DdpCostInput& cost_input) {
    ++nonlinear_rollout_count_;
    cand_states_[0] = states_[0];
    // 控制闭环更新 + 段内真实积分；打靶节点带缺陷缩放项（非打靶节点
    // d 恒为零，退化为纯动力学积分）
    for (std::size_t k = 0; k < num_steps_; ++k) {
        cand_controls_[k] = controls_[k] + alpha * feedforward_[k] +
                            gain_K_[k] * (cand_states_[k] - states_[k]);
        cand_states_[k + 1] =
            dynamics_->step(cand_states_[k], cand_controls_[k], dt_) +
            (alpha - 1.0) * defects_[k + 1];
    }
    // 新缺陷 d' = (1-α)·d̄ 精确成立（与状态更新公式逐位一致）
    cand_defect_norm_ = (1.0 - alpha) * defect_norm_;
    cand_eval_ = cost_evaluator_->evaluate(
        reference, cand_states_, cand_controls_, multipliers, cost_input);
    cand_cost_ = cand_eval_.total_cost;
    return cand_cost_;
}

bool MsIlqrSolver::lineSearch(const DdpReference& reference,
                              const DdpCostMultiplierState& multipliers,
                              const DdpCostInput& cost_input, double merit_prev,
                              double* accepted_alpha, double* accepted_cost) {
    double alpha = 1.0;
    for (int trial = 0; trial < config_.max_backtracks; ++trial) {
        const double cand_cost =
            nonlinearRollout(alpha, reference, multipliers, cost_input);
        const double cand_merit = cand_cost + merit_mu_ * cand_defect_norm_;
        const double expected =
            expectedChange(alpha) - alpha * merit_mu_ * defect_norm_;
        if (std::isfinite(cand_merit) &&
            cand_merit <= merit_prev + config_.armijo_gamma * expected) {
            *accepted_alpha = alpha;
            *accepted_cost = cand_cost;
            return true;
        }
        alpha *= config_.backtrack_beta;
    }
    return false;
}

void MsIlqrSolver::acceptCandidate(double alpha) {
    for (std::size_t i = 0; i <= num_steps_; ++i) {
        states_[i] = cand_states_[i];
        defects_[i] *= (1.0 - alpha);
    }
    for (std::size_t k = 0; k < num_steps_; ++k) {
        controls_[k] = cand_controls_[k];
    }
    defect_norm_ = DefectNorm(defects_);
    // 候选代价求值结果直接移入名义缓存，本轮无需重复全轨迹求值
    cost_eval_ = std::move(cand_eval_);
    total_cost_ = cand_cost_;
}

bool MsIlqrSolver::increaseReg() {
    if (rho_reg_ >= config_.reg_max) {
        return false;
    }
    rho_reg_ = std::min(rho_reg_ * config_.reg_increase, config_.reg_max);
    return true;
}

void MsIlqrSolver::decreaseReg() {
    rho_reg_ = std::max(rho_reg_ * config_.reg_decrease, config_.reg_min);
}

double MsIlqrSolver::DefectNorm(const DdpAlignedVec<DdpState>& defects) {
    double squared = 0.0;
    for (const auto& defect : defects) {
        squared += defect.squaredNorm();
    }
    return std::sqrt(squared);
}
}  // namespace apa_post_processor
