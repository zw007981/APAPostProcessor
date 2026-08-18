#include "ms_ilqr.h"

#include <Eigen/Cholesky>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace apa_post_processor {
template <bool UseVirtualControl>
MsIlqrSolverT<UseVirtualControl>::MsIlqrSolverT(
    MsIlqrConfig config, const BicycleDynamics* dynamics,
    const iLQRCostEvaluator* cost_evaluator)
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

template <bool UseVirtualControl>
MsIlqrResult MsIlqrSolverT<UseVirtualControl>::solve(
    const iLQRReference& reference, const iLQRCostMultiplierState& multipliers,
    const iLQRCostInput& cost_input,
    const iLQRAlignedVec<iLQRState>& initial_states,
    const iLQRAlignedVec<iLQRControl>& initial_controls,
    const iLQRAlignedVec<iLQRState>* initial_virtual_controls) {
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
    // 虚拟控制初始化：非空直接热启动，空则反解自初值轨迹（首轮 rollout
    // 恰好复现初值、初始缺陷恒零——ALTRO 不可行初始化的 w⁰ 构造）
    if constexpr (UseVirtualControl) {
        if (initial_virtual_controls != nullptr) {
            if (initial_virtual_controls->size() != num_steps) {
                throw std::invalid_argument(
                    "MsIlqrSolver: 虚拟控制初值数量必须为 N");
            }
            virtual_controls_ = *initial_virtual_controls;
        } else {
            virtual_controls_ =
                computeVirtualControls(reference, initial_states,
                                       initial_controls);
        }
    }
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

template <bool UseVirtualControl>
void MsIlqrSolverT<UseVirtualControl>::prepareWorkspace(std::size_t num_steps) {
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
    virtual_controls_.resize(num_steps);
    cand_virtual_controls_.resize(num_steps);
    virtual_feedforward_.resize(num_steps);
    virtual_gain_.resize(num_steps);
    dw_lin_.resize(num_steps);
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
        feedforward_.assign(num_steps, iLQRControl::Zero());
        clamped_.assign(num_steps,
                        std::array<bool, ILQR_CONTROL_DIM>{false, false});
        has_clamped_history_ = false;
    }
    gain_K_.resize(num_steps);
    history_.reserve(static_cast<std::size_t>(config_.max_iterations));
}

template <bool UseVirtualControl>
void MsIlqrSolverT<UseVirtualControl>::setShootingLookup(
    const std::vector<std::size_t>& shooting_nodes) {
    is_shooting_.assign(num_steps_ + 1, false);
    for (const std::size_t node : shooting_nodes) {
        if (node > num_steps_) {
            throw std::invalid_argument("MsIlqrSolver: 打靶节点索引越界");
        }
        is_shooting_[node] = true;
    }
}

template <bool UseVirtualControl>
void MsIlqrSolverT<UseVirtualControl>::syncStepDt(const iLQRReference& reference) {
    step_dt_.resize(num_steps_);
    for (std::size_t k = 0; k < num_steps_; ++k) {
        step_dt_[k] = reference.stepDt(k);
    }
}

template <bool UseVirtualControl>
void MsIlqrSolverT<UseVirtualControl>::setNominalTrajectory(
    const iLQRReference& reference,
    const iLQRAlignedVec<iLQRState>& initial_states,
    const iLQRAlignedVec<iLQRControl>& initial_controls) {
    syncStepDt(reference);
    states_[0] = initial_states[0];
    defects_[0].setZero();
    // 初始名义建立：零反馈开环积分，滚动状态被动力学覆写、打靶状态直接
    // 注入初值，缺陷仅在打靶节点非零
    for (std::size_t k = 0; k < num_steps_; ++k) {
        controls_[k] = initial_controls[k];
        iLQRState integral =
            dynamics_->step(states_[k], controls_[k], step_dt_[k]);
        if constexpr (UseVirtualControl) {
            integral += virtual_controls_[k];
        }
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

template <bool UseVirtualControl>
void MsIlqrSolverT<UseVirtualControl>::evaluateNominal(const iLQRReference& reference,
                                   const iLQRCostMultiplierState& multipliers,
                                   const iLQRCostInput& cost_input) {
    cost_eval_ = cost_evaluator_->evaluate(reference, states_, controls_,
                                           multipliers, cost_input);
    total_cost_ = cost_eval_.total_cost;
    if constexpr (UseVirtualControl) {
        // 增广代价并入 w 软代价 ½·R_inf·‖w‖²·dt：与 merit 链
        // （q_w 梯度 / EC 一、二阶项）同源，保证线搜索自洽
        double w_cost = 0.0;
        for (std::size_t k = 0; k < num_steps_; ++k) {
            w_cost += 0.5 * config_.virtual_control_weight * step_dt_[k] *
                      virtual_controls_[k].squaredNorm();
        }
        total_cost_ += w_cost;
    }
}

template <bool UseVirtualControl>
void MsIlqrSolverT<UseVirtualControl>::computeJacobians(const iLQRReference& reference) {
    syncStepDt(reference);
    for (std::size_t k = 0; k < num_steps_; ++k) {
        dynamics_->jacobians(states_[k], controls_[k], step_dt_[k], &jac_A_[k],
                             &jac_B_[k]);
    }
}

template <bool UseVirtualControl>
iLQRStateHessian MsIlqrSolverT<UseVirtualControl>::UpdateValueHessian(
    const iLQRStateHessian& s, const iLQRStateJacobian& a) {
    // 阶段一：t = S·A——按 A 各列的非零元组合 S 的列。列 0/1 为纯单位列；
    // 列 2~6 各含 3~5 个非零元（几何/积分项，非零集合与 jacobians() 一致）；
    // 列组合是连续内存的标量乘加，编译器可展开为 FMA 链并向量化。
    iLQRStateHessian t;
    t.col(ILQR_IDX_X) = s.col(ILQR_IDX_X);
    t.col(ILQR_IDX_Y) = s.col(ILQR_IDX_Y);
    t.col(ILQR_IDX_THETA) = a(ILQR_IDX_X, ILQR_IDX_THETA) * s.col(ILQR_IDX_X) +
                           a(ILQR_IDX_Y, ILQR_IDX_THETA) * s.col(ILQR_IDX_Y) +
                           s.col(ILQR_IDX_THETA);
    t.col(ILQR_IDX_V) = a(ILQR_IDX_X, ILQR_IDX_V) * s.col(ILQR_IDX_X) +
                       a(ILQR_IDX_Y, ILQR_IDX_V) * s.col(ILQR_IDX_Y) +
                       a(ILQR_IDX_THETA, ILQR_IDX_V) * s.col(ILQR_IDX_THETA) +
                       s.col(ILQR_IDX_V);
    t.col(ILQR_IDX_A) = a(ILQR_IDX_X, ILQR_IDX_A) * s.col(ILQR_IDX_X) +
                       a(ILQR_IDX_Y, ILQR_IDX_A) * s.col(ILQR_IDX_Y) +
                       a(ILQR_IDX_THETA, ILQR_IDX_A) * s.col(ILQR_IDX_THETA) +
                       a(ILQR_IDX_V, ILQR_IDX_A) * s.col(ILQR_IDX_V) +
                       s.col(ILQR_IDX_A);
    t.col(ILQR_IDX_DELTA) = a(ILQR_IDX_X, ILQR_IDX_DELTA) * s.col(ILQR_IDX_X) +
                           a(ILQR_IDX_Y, ILQR_IDX_DELTA) * s.col(ILQR_IDX_Y) +
                           a(ILQR_IDX_THETA, ILQR_IDX_DELTA) *
                               s.col(ILQR_IDX_THETA) +
                           s.col(ILQR_IDX_DELTA);
    t.col(ILQR_IDX_OMEGA) = a(ILQR_IDX_X, ILQR_IDX_OMEGA) * s.col(ILQR_IDX_X) +
                           a(ILQR_IDX_Y, ILQR_IDX_OMEGA) * s.col(ILQR_IDX_Y) +
                           a(ILQR_IDX_THETA, ILQR_IDX_OMEGA) *
                               s.col(ILQR_IDX_THETA) +
                           a(ILQR_IDX_DELTA, ILQR_IDX_OMEGA) *
                               s.col(ILQR_IDX_DELTA) +
                           s.col(ILQR_IDX_OMEGA);
    // 阶段二：out = Aᵀ·t——按 A 各行的非零元组合 t 的行（与列组合同形，
    // 求值顺序与阶段一对偶）。下游 next_hessian 恒做对称化，本内核
    // 无需保证输出逐位对称。
    iLQRStateHessian out;
    out.row(ILQR_IDX_X) = t.row(ILQR_IDX_X);
    out.row(ILQR_IDX_Y) = t.row(ILQR_IDX_Y);
    out.row(ILQR_IDX_THETA) = a(ILQR_IDX_X, ILQR_IDX_THETA) * t.row(ILQR_IDX_X) +
                             a(ILQR_IDX_Y, ILQR_IDX_THETA) * t.row(ILQR_IDX_Y) +
                             t.row(ILQR_IDX_THETA);
    out.row(ILQR_IDX_V) = a(ILQR_IDX_X, ILQR_IDX_V) * t.row(ILQR_IDX_X) +
                         a(ILQR_IDX_Y, ILQR_IDX_V) * t.row(ILQR_IDX_Y) +
                         a(ILQR_IDX_THETA, ILQR_IDX_V) * t.row(ILQR_IDX_THETA) +
                         t.row(ILQR_IDX_V);
    out.row(ILQR_IDX_A) = a(ILQR_IDX_X, ILQR_IDX_A) * t.row(ILQR_IDX_X) +
                         a(ILQR_IDX_Y, ILQR_IDX_A) * t.row(ILQR_IDX_Y) +
                         a(ILQR_IDX_THETA, ILQR_IDX_A) * t.row(ILQR_IDX_THETA) +
                         a(ILQR_IDX_V, ILQR_IDX_A) * t.row(ILQR_IDX_V) +
                         t.row(ILQR_IDX_A);
    out.row(ILQR_IDX_DELTA) = a(ILQR_IDX_X, ILQR_IDX_DELTA) * t.row(ILQR_IDX_X) +
                             a(ILQR_IDX_Y, ILQR_IDX_DELTA) * t.row(ILQR_IDX_Y) +
                             a(ILQR_IDX_THETA, ILQR_IDX_DELTA) *
                                 t.row(ILQR_IDX_THETA) +
                             t.row(ILQR_IDX_DELTA);
    out.row(ILQR_IDX_OMEGA) = a(ILQR_IDX_X, ILQR_IDX_OMEGA) * t.row(ILQR_IDX_X) +
                             a(ILQR_IDX_Y, ILQR_IDX_OMEGA) * t.row(ILQR_IDX_Y) +
                             a(ILQR_IDX_THETA, ILQR_IDX_OMEGA) *
                                 t.row(ILQR_IDX_THETA) +
                             a(ILQR_IDX_DELTA, ILQR_IDX_OMEGA) *
                                 t.row(ILQR_IDX_DELTA) +
                             t.row(ILQR_IDX_OMEGA);
    return out;
}

template <bool UseVirtualControl>
bool MsIlqrSolverT<UseVirtualControl>::backwardPass() {
    ++backward_pass_count_;
    const auto& stages = cost_eval_.stages;
    iLQRStateHessian value_hessian = stages[num_steps_].lxx;
    iLQRState value_gradient = stages[num_steps_].lx;
    value_S_[num_steps_] = value_hessian;
    value_s_[num_steps_] = value_gradient;
    dv1_ = 0.0;
    dv2_ = 0.0;
    max_qu_norm_ = 0.0;
    for (std::size_t k = num_steps_; k-- > 0;) {
        const auto& stage = stages[k];
        const iLQRStateJacobian& jac_a = jac_A_[k];
        const iLQRControlJacobian& jac_b = jac_B_[k];
        // 缺陷修正：ẑ = s' + S'·d[k+1]（右端索引；非打靶节点 d 恒为零，
        // 直接复用 s' 跳过与零矩阵的乘积——打靶间隔 25，96% 的阶段受益）
        const iLQRState z =
            is_shooting_[k + 1]
                ? value_gradient + value_hessian * defects_[k + 1]
                : value_gradient;
        const iLQRState q_x = stage.lx + jac_a.transpose() * z;
        const iLQRControl q_u = stage.lu + jac_b.transpose() * z;
        iLQRStateHessian q_xx = stage.lxx + UpdateValueHessian(value_hessian,
                                                               jac_a);
        iLQRControlHessian q_uu =
            stage.luu + jac_b.transpose() * value_hessian * jac_b;
        iLQRControlStateHessian q_ux =
            stage.lux + jac_b.transpose() * value_hessian * jac_a;
        q_uu += rho_reg_ * iLQRControlHessian::Identity();
        // 虚拟控制增广：x⁺=f(x,u)+w ⟹ ∂x⁺/∂w=I。w 为无约束输入，
        // 对 W=Q_ww 做 Schur 消元（先解 7×7 自由子问题）后再走既有
        // 2 维 box-QP——消元保留价值函数最优值，box-QP 与关闭路径
        // 完全同构
        iLQRStateHessian q_xx_eff = q_xx;
        iLQRControlHessian q_uu_eff = q_uu;
        iLQRControlStateHessian q_ux_eff = q_ux;
        iLQRControl q_u_eff = q_u;
        iLQRState q_x_eff = q_x;
        // w 消元的中间量（仅开关开启时赋值与消费）
        Eigen::Matrix<double, ILQR_STATE_DIM, ILQR_CONTROL_DIM> t_u;
        if constexpr (UseVirtualControl) {
            const double w_weight_dt =
                config_.virtual_control_weight * step_dt_[k];
            const iLQRState q_w =
                w_weight_dt * virtual_controls_[k] + z;
            iLQRStateHessian w_hessian = value_hessian;
            w_hessian.diagonal().array() += w_weight_dt;
            Eigen::LLT<iLQRStateHessian> w_llt(w_hessian);
            if (w_llt.info() != Eigen::Success) {
                return false;
            }
            const iLQRControlStateHessian q_uw =
                jac_b.transpose() * value_hessian;
            const iLQRStateHessian q_wx = value_hessian * jac_a;
            t_u = w_llt.solve(q_uw.transpose());
            const iLQRState t_w = w_llt.solve(q_w);
            const iLQRStateHessian t_x = w_llt.solve(q_wx);
            q_xx_eff = q_xx - q_wx.transpose() * t_x;
            q_uu_eff = q_uu - q_uw * t_u;
            q_ux_eff = q_ux - q_uw * t_x;
            q_u_eff = q_u - q_uw * t_w;
            q_x_eff = q_x - q_wx.transpose() * t_w;
            // w 的闭环响应（前馈部分在 QP 求解后补 k 项）：δw =
            // −W⁻¹(Q_w+Q_wu·δu+Q_wx·δx)，δu=k+K·δx
            virtual_feedforward_[k] = -t_w;
            virtual_gain_[k] = -t_x;
        }
        // 盒约束 QP（H 已含正则化）：回推顺序上一步的钳制集热启动，
        // 分解每步必然重做
        BoxQpSolver<>::Problem problem;
        if constexpr (UseVirtualControl) {
            problem.hessian = q_uu_eff;
            problem.gradient = q_u_eff;
        } else {
            problem.hessian = q_uu;
            problem.gradient = q_u;
        }
        problem.lower =
            iLQRControl(-config_.jerk_max, -config_.steer_accel_max) -
            controls_[k];
        problem.upper = iLQRControl(config_.jerk_max, config_.steer_accel_max) -
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
        if constexpr (UseVirtualControl) {
            gain_K_[k] =
                -BoxQpSolver<>::SolveFreeExpanded(qp_result, q_ux_eff);
        } else {
            gain_K_[k] =
                -BoxQpSolver<>::SolveFreeExpanded(qp_result, q_ux);
        }
        clamped_[k] = qp_result.clamped;
        q_x_[k] = q_x;
        q_u_[k] = q_u;
        q_uu_[k] = q_uu;
        if constexpr (UseVirtualControl) {
            // w 前馈补 QP 项：w_ff = −W⁻¹(Q_w + Q_wu·k)
            virtual_feedforward_[k].noalias() -= t_u * feedforward_[k];
            // w 增益补 QP 项：K_w = −W⁻¹(Q_wu·K + Q_wx)
            virtual_gain_[k].noalias() -= t_u * gain_K_[k];
        }
        // 价值回传（K/δũ 形式：任意活动集下均良定义，Q_uu 取含正则化版本；
        // 增广路径消费 Schur 消元后的有效量，保证价值函数含 w 的影响）
        iLQRStateHessian next_hessian;
        iLQRState next_gradient;
        if constexpr (UseVirtualControl) {
            next_hessian =
                q_xx_eff + gain_K_[k].transpose() * q_uu_eff * gain_K_[k] +
                gain_K_[k].transpose() * q_ux_eff +
                q_ux_eff.transpose() * gain_K_[k];
            next_hessian = 0.5 * (next_hessian + next_hessian.transpose());
            next_gradient =
                q_x_eff + gain_K_[k].transpose() * q_uu_eff * feedforward_[k] +
                gain_K_[k].transpose() * q_u_eff +
                q_ux_eff.transpose() * feedforward_[k];
            dv1_ += feedforward_[k].dot(q_u_eff);
            dv2_ +=
                (feedforward_[k].transpose() * q_uu_eff * feedforward_[k])
                    .value();
            max_qu_norm_ =
                std::max(max_qu_norm_, q_u_eff.cwiseAbs().maxCoeff());
        } else {
            next_hessian =
                q_xx + gain_K_[k].transpose() * q_uu * gain_K_[k] +
                gain_K_[k].transpose() * q_ux + q_ux.transpose() * gain_K_[k];
            next_hessian = 0.5 * (next_hessian + next_hessian.transpose());
            next_gradient =
                q_x + gain_K_[k].transpose() * q_uu * feedforward_[k] +
                gain_K_[k].transpose() * q_u + q_ux.transpose() * feedforward_[k];
            dv1_ += feedforward_[k].dot(q_u);
            dv2_ +=
                (feedforward_[k].transpose() * q_uu * feedforward_[k]).value();
            max_qu_norm_ = std::max(max_qu_norm_, q_u.cwiseAbs().maxCoeff());
        }
        value_S_[k] = next_hessian;
        value_s_[k] = next_gradient;
        value_hessian = next_hessian;
        value_gradient = next_gradient;
    }
    has_clamped_history_ = true;
    return true;
}

template <bool UseVirtualControl>
void MsIlqrSolverT<UseVirtualControl>::linearRollout() {
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
        if constexpr (UseVirtualControl) {
            iLQRState dw = virtual_feedforward_[k] + virtual_gain_[k] * dx_lin_[k];
            dw_lin_[k] = dw;
            dx_lin_[k + 1] += dw;
        }
        const auto& stage = stages[k];
        ec1_ += stage.lx.dot(dx_lin_[k]) + stage.lu.dot(du_lin_[k]);
        ec2_ +=
            (dx_lin_[k].transpose() * stage.lxx * dx_lin_[k]).value() +
            (2.0 * du_lin_[k].transpose() * stage.lux * dx_lin_[k]).value() +
            (du_lin_[k].transpose() * stage.luu * du_lin_[k]).value();
        // 虚拟控制软代价对 EC 的闭式贡献（名义 w 的一阶/二阶项）
        if constexpr (UseVirtualControl) {
            const double w_weight_dt =
                config_.virtual_control_weight * step_dt_[k];
            ec1_ += w_weight_dt * virtual_controls_[k].dot(dw_lin_[k]);
            ec2_ += w_weight_dt * dw_lin_[k].squaredNorm();
        }
    }
    const auto& terminal = stages[num_steps_];
    ec1_ += terminal.lx.dot(dx_lin_[num_steps_]);
    ec2_ +=
        (dx_lin_[num_steps_].transpose() * terminal.lxx * dx_lin_[num_steps_])
            .value();
}

template <bool UseVirtualControl>
bool MsIlqrSolverT<UseVirtualControl>::convergenceAllowed() const {
    double defect_inf = 0.0;
    for (const auto& defect : defects_) {
        defect_inf = std::max(defect_inf, defect.cwiseAbs().maxCoeff());
    }
    return defect_inf <= config_.convergence_defect_tol;
}

template <bool UseVirtualControl>
double MsIlqrSolverT<UseVirtualControl>::nonlinearRollout(double alpha,
                                      const iLQRReference& reference,
                                      const iLQRCostMultiplierState& multipliers,
                                      const iLQRCostInput& cost_input,
                                      double merit_reject_threshold) {
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
        if constexpr (UseVirtualControl) {
            const iLQRState cand_w =
                virtual_controls_[k] + alpha * virtual_feedforward_[k] +
                virtual_gain_[k] * (cand_states_[k] - states_[k]);
            cand_virtual_controls_[k] = cand_w;
            cand_states_[k + 1] += cand_w;
        }
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
            const double x = state(ILQR_IDX_X);
            const double y = state(ILQR_IDX_Y);
            if (!(x >= min_x && x <= max_x && y >= min_y && y <= max_y)) {
                ++domain_guard_rejections_;
                return std::numeric_limits<double>::infinity();
            }
        }
    }
    // 线搜索早停：merit = cost + µ_m·‖d‖，把 merit 接受阈值折算为
    // 代价阈值交给求值层——ESDF 代价恒非负 ⟹ 廉价小计是完整代价的
    // 下界，小计超阈即整段跳过 ESDF（拒绝决策与全量求值逐位一致）
    iLQRCostInput screened_input = cost_input;
    screened_input.screen_cost_threshold =
        merit_reject_threshold - merit_mu_ * cand_defect_norm_;
    cand_eval_ = cost_evaluator_->evaluate(
        reference, cand_states_, cand_controls_, multipliers, screened_input);
    cand_cost_ = cand_eval_.total_cost;
    if constexpr (UseVirtualControl) {
        // 候选增广代价并入 w 软代价（与名义侧同源）
        double w_cost = 0.0;
        for (std::size_t k = 0; k < num_steps_; ++k) {
            w_cost += 0.5 * config_.virtual_control_weight * step_dt_[k] *
                      cand_virtual_controls_[k].squaredNorm();
        }
        cand_cost_ += w_cost;
    }
    return cand_cost_;
}

template <bool UseVirtualControl>
bool MsIlqrSolverT<UseVirtualControl>::lineSearch(const iLQRReference& reference,
                              const iLQRCostMultiplierState& multipliers,
                              const iLQRCostInput& cost_input, double merit_prev,
                              double* accepted_alpha, double* accepted_cost) {
    double alpha = 1.0;
    for (int trial = 0; trial < config_.max_backtracks; ++trial) {
        // EC(α) 闭式量先算好，才能把 merit 接受阈值折算为代价早停
        // 阈值传给 rollout（次序换位不改变数值：所用成员均不随试错变）
        const double expected =
            expectedChange(alpha) - alpha * merit_mu_ * defect_norm_;
        const double merit_threshold =
            merit_prev + config_.armijo_gamma * expected;
        const double cand_cost =
            nonlinearRollout(alpha, reference, multipliers, cost_input,
                             merit_threshold);
        double cand_merit = cand_cost + merit_mu_ * cand_defect_norm_;
        // 刀刃防御：筛选比较式（廉价小计 > 折算阈值）与原始 merit
        // 比较式（代价 + µ_m·‖d‖ ≤ 阈值）在 1 ulp 边界可能分歧——
        // 一旦出现「筛选拒绝而原始式会接受」的迹象，用全量求值重判，
        // 保证接受/拒绝决策与未加筛选的原始实现逐位等价
        if (cand_eval_.esdf_screened_out && cand_merit <= merit_threshold) {
            cand_eval_ = cost_evaluator_->evaluate(
                reference, cand_states_, cand_controls_, multipliers,
                cost_input);
            cand_cost_ = cand_eval_.total_cost;
            cand_merit = cand_cost_ + merit_mu_ * cand_defect_norm_;
        }
        if (std::isfinite(cand_merit) && cand_merit <= merit_threshold) {
            *accepted_alpha = alpha;
            *accepted_cost = cand_cost_;
            return true;
        }
        alpha *= config_.backtrack_beta;
    }
    return false;
}

template <bool UseVirtualControl>
void MsIlqrSolverT<UseVirtualControl>::acceptCandidate(double alpha) {
    for (std::size_t i = 0; i <= num_steps_; ++i) {
        states_[i] = cand_states_[i];
        defects_[i] *= (1.0 - alpha);
    }
    for (std::size_t k = 0; k < num_steps_; ++k) {
        controls_[k] = cand_controls_[k];
    }
    if constexpr (UseVirtualControl) {
        virtual_controls_ = cand_virtual_controls_;
    }
    defect_norm_ = DefectNorm(defects_);
    // 候选代价求值结果直接移入名义缓存，本轮无需重复全轨迹求值
    cost_eval_ = std::move(cand_eval_);
    total_cost_ = cand_cost_;
}

template <bool UseVirtualControl>
bool MsIlqrSolverT<UseVirtualControl>::increaseReg() {
    if (rho_reg_ >= config_.reg_max) {
        return false;
    }
    rho_reg_ = std::min(rho_reg_ * config_.reg_increase, config_.reg_max);
    return true;
}

template <bool UseVirtualControl>
void MsIlqrSolverT<UseVirtualControl>::decreaseReg() {
    rho_reg_ = std::max(rho_reg_ * config_.reg_decrease, config_.reg_min);
}

template <bool UseVirtualControl>
double MsIlqrSolverT<UseVirtualControl>::DefectNorm(const iLQRAlignedVec<iLQRState>& defects) {
    double squared = 0.0;
    for (const auto& defect : defects) {
        squared += defect.squaredNorm();
    }
    return std::sqrt(squared);
}

template <bool UseVirtualControl>
iLQRAlignedVec<iLQRState> MsIlqrSolverT<UseVirtualControl>::computeVirtualControls(
    const iLQRReference& reference,
    const iLQRAlignedVec<iLQRState>& initial_states,
    const iLQRAlignedVec<iLQRControl>& initial_controls) const {
    const std::size_t num_steps = initial_controls.size();
    iLQRAlignedVec<iLQRState> w;
    w.reserve(num_steps);
    for (std::size_t k = 0; k < num_steps; ++k) {
        const iLQRState integral =
            dynamics_->step(initial_states[k], initial_controls[k],
                            reference.stepDt(k));
        w.push_back(initial_states[k + 1] - integral);
    }
    return w;
}

template class MsIlqrSolverT<false>;
template class MsIlqrSolverT<true>;
}  // namespace apa_post_processor
