#include "ms_ilqr.h"

#include <algorithm>
#include <cmath>
#include <limits>
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
    if (!(config_.convergence_defect_tol > 0.0)) {
        throw std::invalid_argument("MsIlqrSolver: 收敛可行性容差必须为正");
    }
    if (!(config_.merit_mu0 > 0.0) || !(config_.merit_mu_al_ratio >= 0.0) ||
        !std::isfinite(config_.merit_mu_al_ratio)) {
        throw std::invalid_argument("MsIlqrSolver: merit 参数非法");
    }
    // µ_m 上限必须不小于 µ₀（否则地板与上限冲突、语义不自洽）
    if (!(config_.merit_mu_max >= config_.merit_mu0) ||
        !std::isfinite(config_.merit_mu_max)) {
        throw std::invalid_argument(
            "MsIlqrSolver: merit_mu_max 必须为有限值且不小于 merit_mu0");
    }
    // 定义域守卫 margin：0 = 关闭；启用时必须为非负有限值
    if (!(config_.domain_guard_margin >= 0.0) ||
        !std::isfinite(config_.domain_guard_margin)) {
        throw std::invalid_argument(
            "MsIlqrSolver: domain_guard_margin 必须为非负有限值");
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
    domain_guard_rejections_ = 0;
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
        while (true) {
            ++passes_this_iter;
            if (!backwardPass()) {
                if (!increaseReg()) {
                    result.status = MsIlqrStatus::REGULARIZATION_OVERFLOW;
                    result.iterations = iter - 1;
                    result.final_cost = total_cost_;
                    result.final_defect_norm = defect_norm_;
                    result.domain_guard_rejections = domain_guard_rejections_;
                    return result;
                }
                continue;
            }
            linearRollout();
            const double merit_prev = total_cost_ + merit_mu_ * defect_norm_;
            const std::int64_t trials_before = nonlinear_rollout_count_;
            if (lineSearch(reference, multipliers, cost_input, merit_prev,
                           &accepted_alpha, &accepted_cost)) {
                trials_this_iter =
                    static_cast<int>(nonlinear_rollout_count_ - trials_before);
                break;
            }
            // 线搜索被拒：升 ρ_reg 后重跑整个回推（QP 随 Hessian 变化
            // 必须重做分解——与 QP 失败路径同一约定）
            if (!increaseReg()) {
                result.status = MsIlqrStatus::REGULARIZATION_OVERFLOW;
                result.iterations = iter - 1;
                result.final_cost = total_cost_;
                result.final_defect_norm = defect_norm_;
                result.domain_guard_rejections = domain_guard_rejections_;
                return result;
            }
        }
        // 接受：ρ_reg 按既有 LM 调度收缩
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
        // 收敛出口的可行性守卫：两个尺度盲判据达标时，打靶缺陷还必须
        const double rel_change = std::abs(cost_prev - total_cost_) /
                                  std::max(std::abs(cost_prev), 1e-12);
        if (rel_change < config_.cost_change_tol && convergenceAllowed()) {
            result.status = MsIlqrStatus::CONVERGED_COST;
            break;
        }
        if (max_qu_norm_ < config_.gradient_tol && convergenceAllowed()) {
            result.status = MsIlqrStatus::CONVERGED_GRADIENT;
            break;
        }
    }
    result.final_cost = total_cost_;
    result.final_defect_norm = defect_norm_;
    result.domain_guard_rejections = domain_guard_rejections_;
    return result;
}

void MsIlqrSolver::prepareWorkspace(std::size_t num_steps) {
    // 步数不变时跳过全部 resize/assign，仅清空历史（热启动主路径）
    if (num_steps_ == num_steps) {
        is_shooting_.assign(is_shooting_.size(), false);
        history_.clear();
        return;
    }
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
    dx_lin_.resize(nodes);
    du_lin_.resize(num_steps);
    // 首轮 QP 初始点/钳制集只在（重）分配时清零；N 不变时保留跨 solve
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

void MsIlqrSolver::syncStepDt(const DdpReference& reference) {
    step_dt_.resize(num_steps_);
    for (std::size_t k = 0; k < num_steps_; ++k) {
        step_dt_[k] = reference.stepDt(k);
    }
}

void MsIlqrSolver::setNominalTrajectory(
    const DdpReference& reference,
    const DdpAlignedVec<DdpState>& initial_states,
    const DdpAlignedVec<DdpControl>& initial_controls) {
    syncStepDt(reference);
    states_[0] = initial_states[0];
    defects_[0].setZero();
    // 初始名义建立：零反馈开环积分，滚动状态被动力学覆写、打靶状态直接
    // 注入初值，缺陷仅在打靶节点非零
    for (std::size_t k = 0; k < num_steps_; ++k) {
        controls_[k] = initial_controls[k];
        const DdpState integral =
            dynamics_->step(states_[k], controls_[k], step_dt_[k]);
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
    syncStepDt(reference);
    for (std::size_t k = 0; k < num_steps_; ++k) {
        dynamics_->jacobians(states_[k], controls_[k], step_dt_[k], &jac_A_[k],
                             &jac_B_[k]);
    }
}

bool MsIlqrSolver::backwardPass() {
    ++backward_pass_count_;
    const auto& stages = cost_eval_.stages;
    DdpStateHessian value_hessian = stages[num_steps_].lxx;
    DdpState value_gradient = stages[num_steps_].lx;
    value_S_[num_steps_] = value_hessian;
    value_s_[num_steps_] = value_gradient;
    dv1_ = 0.0;
    dv2_ = 0.0;
    max_qu_norm_ = 0.0;
    for (std::size_t k = num_steps_; k-- > 0;) {
        const auto& stage = stages[k];
        const DdpStateJacobian& jac_a = jac_A_[k];
        const DdpControlJacobian& jac_b = jac_B_[k];
        // 缺陷修正：ẑ = s' + S'·d[k+1]（右端索引；非打靶节点 d 恒为零，
        // 直接复用 s' 跳过与零矩阵的乘积——打靶间隔 25，96% 的阶段受益）
        const DdpState z =
            is_shooting_[k + 1]
                ? value_gradient + value_hessian * defects_[k + 1]
                : value_gradient;
        const DdpState q_x = stage.lx + jac_a.transpose() * z;
        const DdpControl q_u = stage.lu + jac_b.transpose() * z;
        DdpStateHessian q_xx =
            stage.lxx + jac_a.transpose() * value_hessian * jac_a;
        DdpControlHessian q_uu =
            stage.luu + jac_b.transpose() * value_hessian * jac_b;
        DdpControlStateHessian q_ux =
            stage.lux + jac_b.transpose() * value_hessian * jac_a;
        q_uu += rho_reg_ * DdpControlHessian::Identity();
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

bool MsIlqrSolver::convergenceAllowed() const {
    double defect_inf = 0.0;
    for (const auto& defect : defects_) {
        defect_inf = std::max(defect_inf, defect.cwiseAbs().maxCoeff());
    }
    return defect_inf <= config_.convergence_defect_tol;
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
            dynamics_->step(cand_states_[k], cand_controls_[k], step_dt_[k]) +
            (alpha - 1.0) * defects_[k + 1];
    }
    // 新缺陷 d' = (1-α)·d̄ 精确成立（与状态更新公式逐位一致）
    cand_defect_norm_ = (1.0 - alpha) * defect_norm_;
    // L8.3 定义域守卫：AL 幅值约束只覆盖 v/a/δ/ω、Box-QP 只约束控制，
    if (config_.domain_guard_margin > 0.0 &&
        cost_evaluator_->esdfConstraint() != nullptr) {
        const ESDFMap& map = cost_evaluator_->esdfConstraint()->esdfMap();
        const double margin = config_.domain_guard_margin;
        const double min_x = map.getOrigin().x - margin;
        const double min_y = map.getOrigin().y - margin;
        const double max_x =
            map.getOrigin().x +
            static_cast<double>(map.getWidth()) * map.getResolution() + margin;
        const double max_y =
            map.getOrigin().y +
            static_cast<double>(map.getHeight()) * map.getResolution() + margin;
        for (const auto& state : cand_states_) {
            const double x = state(DDP_IDX_X);
            const double y = state(DDP_IDX_Y);
            if (!(x >= min_x && x <= max_x && y >= min_y && y <= max_y)) {
                ++domain_guard_rejections_;
                return std::numeric_limits<double>::infinity();
            }
        }
    }
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
