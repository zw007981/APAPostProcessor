#include "ddp_cost.h"

#include <cmath>
#include <stdexcept>

namespace apa_post_processor {
DdpCostMultiplierState DdpCostMultiplierState::MakeZero(std::size_t num_steps) {
    DdpCostMultiplierState state;
    state.amplitude_lambda = Eigen::VectorXd::Zero(
        static_cast<Eigen::Index>(DDP_AMPLITUDE_CONSTRAINT_DIM * num_steps));
    state.amplitude_mu = Eigen::VectorXd::Zero(
        static_cast<Eigen::Index>(DDP_AMPLITUDE_CONSTRAINT_DIM * num_steps));
    state.terminal_lambda.setZero();
    state.terminal_mu.setZero();
    return state;
}

DdpCostMultiplierState DdpCostMultiplierState::MakeStageTwoZero(
    std::size_t num_steps, std::size_t num_seams) {
    auto state = MakeZero(num_steps);
    const auto nodes = static_cast<Eigen::Index>(num_steps + 1);
    const auto seams = static_cast<Eigen::Index>(num_seams);
    state.gating_sign_lambda = Eigen::VectorXd::Zero(nodes);
    state.gating_sign_mu = Eigen::VectorXd::Zero(nodes);
    state.gating_seam_lambda = Eigen::VectorXd::Zero(seams);
    state.gating_seam_mu = Eigen::VectorXd::Zero(seams);
    state.gating_dwell_lambda = Eigen::VectorXd::Zero(nodes);
    state.gating_dwell_mu = Eigen::VectorXd::Zero(nodes);
    return state;
}

DdpCostEvaluator::DdpCostEvaluator(DdpCostConfig config,
                                   const DdpEsdfConstraint* esdf_constraint)
    : config_(config), esdf_constraint_(esdf_constraint) {
    // 配置错误会静默污染全部下游求解目标，必须在构造期显式拒绝
    if (!std::isfinite(config_.weight_jerk) || config_.weight_jerk < 0.0 ||
        !std::isfinite(config_.weight_steer_accel) ||
        config_.weight_steer_accel < 0.0 ||
        !std::isfinite(config_.weight_ref_base) ||
        config_.weight_ref_base < 0.0 || !std::isfinite(config_.weight_theta) ||
        config_.weight_theta < 0.0) {
        throw std::invalid_argument("代价权重必须为非负有限值");
    }
    if (!std::isfinite(config_.v_max) || config_.v_max <= 0.0 ||
        !std::isfinite(config_.a_max) || config_.a_max <= 0.0 ||
        !std::isfinite(config_.omega_max) || config_.omega_max <= 0.0 ||
        !std::isfinite(config_.delta_max) || config_.delta_max <= 0.0) {
        throw std::invalid_argument("幅值边界必须为正有限值");
    }
}

DdpCostEvaluation DdpCostEvaluator::evaluate(
    const DdpReference& reference, const DdpAlignedVec<DdpState>& states,
    const DdpAlignedVec<DdpControl>& controls,
    const DdpCostMultiplierState& multipliers,
    const DdpCostInput& input) const {
    const std::size_t num_poses = reference.poses.size();
    if (num_poses < 2) {
        throw std::invalid_argument("参考位姿数量必须 >= 2");
    }
    const std::size_t num_steps = num_poses - 1;
    if (states.size() != num_poses) {
        throw std::invalid_argument("状态数量必须为 N+1");
    }
    if (controls.size() != num_steps) {
        throw std::invalid_argument("控制数量必须为 N");
    }
    const auto amplitude_size =
        static_cast<Eigen::Index>(DDP_AMPLITUDE_CONSTRAINT_DIM * num_steps);
    if (multipliers.amplitude_lambda.size() != amplitude_size ||
        multipliers.amplitude_mu.size() != amplitude_size) {
        throw std::invalid_argument("幅值乘子尺寸必须为 5N");
    }
    if (input.anneal_exempt_mask != nullptr &&
        input.anneal_exempt_mask->size() != num_poses) {
        throw std::invalid_argument("豁免掩码尺寸必须为 N+1");
    }
    if (!std::isfinite(reference.dt) || reference.dt <= 0.0) {
        throw std::invalid_argument("参考轨迹 dt 必须为正有限值");
    }
    if (!std::isfinite(input.tracking_weight)) {
        throw std::invalid_argument("跟踪权重必须为有限值");
    }
    // 门控一致性校验（"阶段一禁止启用"的断言防御）：门控计划与门控乘子
    const bool has_plan = input.gating_plan != nullptr;
    const bool has_gating_multipliers =
        multipliers.gating_sign_lambda.size() > 0 ||
        multipliers.gating_sign_mu.size() > 0 ||
        multipliers.gating_seam_lambda.size() > 0 ||
        multipliers.gating_seam_mu.size() > 0 ||
        multipliers.gating_dwell_lambda.size() > 0 ||
        multipliers.gating_dwell_mu.size() > 0;
    if (has_plan != has_gating_multipliers) {
        throw std::logic_error(
            "DdpCostEvaluator: 门控计划与门控乘子必须同在场（阶段一禁止"
            "启用门控项）");
    }
    if (has_plan) {
        const auto nodes = static_cast<Eigen::Index>(num_poses);
        const auto seams =
            static_cast<Eigen::Index>(input.gating_plan->seam_indices.size());
        if (input.gating_plan->sign_gate.size() != num_poses ||
            input.gating_plan->seam_lookup.size() != num_poses ||
            input.gating_plan->dwell_v_cap.size() != num_poses) {
            throw std::invalid_argument("门控计划尺寸必须为 N+1");
        }
        if (multipliers.gating_sign_lambda.size() != nodes ||
            multipliers.gating_sign_mu.size() != nodes ||
            multipliers.gating_dwell_lambda.size() != nodes ||
            multipliers.gating_dwell_mu.size() != nodes ||
            multipliers.gating_seam_lambda.size() != seams ||
            multipliers.gating_seam_mu.size() != seams) {
            throw std::invalid_argument("门控乘子尺寸必须与计划一致");
        }
    }
    if (!(input.esdf_scale > 0.0) || !std::isfinite(input.esdf_scale)) {
        throw std::invalid_argument(
            "DdpCostEvaluator: esdf_scale 必须为正有限值");
    }
    DdpCostEvaluation result;
    result.stages.resize(num_poses);
    for (std::size_t k = 0; k < num_steps; ++k) {
        evaluateRunningStage(k, reference, states, controls, multipliers, input,
                             &result.stages[k]);
        result.total_cost += result.stages[k].totalCost();
    }
    evaluateTerminalStage(reference, states, multipliers, input.esdf_scale,
                          &result.stages[num_steps]);
    result.total_cost += result.stages[num_steps].totalCost();
    return result;
}

void DdpCostEvaluator::evaluateRunningStage(
    std::size_t k, const DdpReference& reference,
    const DdpAlignedVec<DdpState>& states,
    const DdpAlignedVec<DdpControl>& controls,
    const DdpCostMultiplierState& multipliers, const DdpCostInput& input,
    DdpStageCostDerivatives* out) const {
    // 显式清零：防御调用方复用输出缓冲的场景（当前 evaluate 总是给零值阶段）
    *out = DdpStageCostDerivatives{};
    const DdpState& x = states[k];
    const DdpControl& u = controls[k];
    const double dt = reference.stepDt(k);
    // 平滑主项 ½(w_j·j² + w_η·η²)·dt：融化机制的核心驱动
    const double jerk = u(DDP_IDX_JERK);
    const double eta = u(DDP_IDX_ETA);
    out->cost_smooth += 0.5 *
                        (config_.weight_jerk * jerk * jerk +
                         config_.weight_steer_accel * eta * eta) *
                        dt;
    out->lu(DDP_IDX_JERK) += config_.weight_jerk * jerk * dt;
    out->lu(DDP_IDX_ETA) += config_.weight_steer_accel * eta * dt;
    out->luu(DDP_IDX_JERK, DDP_IDX_JERK) += config_.weight_jerk * dt;
    out->luu(DDP_IDX_ETA, DDP_IDX_ETA) += config_.weight_steer_accel * dt;
    // 退火跟踪项：权重选择优先级「豁免点恒用 w_ref,0 > 普通点用 w_ref(r)」
    const auto& mask = input.anneal_exempt_mask;
    const double w_ref = (mask != nullptr && (*mask)[k])
                             ? config_.weight_ref_base
                             : input.tracking_weight;
    const Pose& ref = reference.poses[k];
    const double err_x = x(DDP_IDX_X) - ref.x;
    const double err_y = x(DDP_IDX_Y) - ref.y;
    const double err_theta = WrapAngle(x(DDP_IDX_THETA) - ref.theta);
    out->cost_tracking +=
        0.5 * w_ref * (err_x * err_x + err_y * err_y) * dt +
        0.5 * config_.weight_theta * err_theta * err_theta * dt;
    out->lx(DDP_IDX_X) += w_ref * err_x * dt;
    out->lx(DDP_IDX_Y) += w_ref * err_y * dt;
    out->lx(DDP_IDX_THETA) += config_.weight_theta * err_theta * dt;
    out->lxx(DDP_IDX_X, DDP_IDX_X) += w_ref * dt;
    out->lxx(DDP_IDX_Y, DDP_IDX_Y) += w_ref * dt;
    out->lxx(DDP_IDX_THETA, DDP_IDX_THETA) += config_.weight_theta * dt;
    accumulateAmplitudeConstraints(k, x, multipliers, out);
    // 阶段二门控（仅计划在场时累加；阶段一 gating_plan 恒为 nullptr）
    if (input.gating_plan != nullptr) {
        accumulateGatingConstraints(k, x, multipliers, *input.gating_plan, out);
    }
    // ESDF 双 margin 惩罚：按 stride 时间轴抽样
    if (esdf_constraint_ != nullptr && esdf_constraint_->isSampled(k)) {
        const auto esdf = esdf_constraint_->evaluate(x(DDP_IDX_X), x(DDP_IDX_Y),
                                                     x(DDP_IDX_THETA));
        out->cost_esdf += input.esdf_scale * esdf.cost;
        out->lx += input.esdf_scale * esdf.gradient;
        out->lxx += input.esdf_scale * esdf.hessian;
    }
}

void DdpCostEvaluator::evaluateTerminalStage(
    const DdpReference& reference, const DdpAlignedVec<DdpState>& states,
    const DdpCostMultiplierState& multipliers, double esdf_scale,
    DdpStageCostDerivatives* out) const {
    // 显式清零：防御调用方复用输出缓冲的场景（当前 evaluate 总是给零值阶段）
    *out = DdpStageCostDerivatives{};
    const DdpState& x = states.back();
    const Pose& goal = reference.poses.back();
    // 终点 AL 等式：c=[x−xg, y−yg, wrap(θ−θg), v, a]，λᵀc + ½μc²；
    // 残差对状态的雅可比是常量选择矩阵，GN 形 Hessian 即精确 diag(μ)
    Eigen::Matrix<double, DDP_TERMINAL_CONSTRAINT_DIM, 1> c;
    c << x(DDP_IDX_X) - goal.x, x(DDP_IDX_Y) - goal.y,
        WrapAngle(x(DDP_IDX_THETA) - goal.theta), x(DDP_IDX_V), x(DDP_IDX_A);
    out->cost_terminal += multipliers.terminal_lambda.dot(c) +
                          0.5 * multipliers.terminal_mu.cwiseProduct(c).dot(c);
    const auto scale =
        multipliers.terminal_lambda + multipliers.terminal_mu.cwiseProduct(c);
    constexpr int kRows[DDP_TERMINAL_CONSTRAINT_DIM] = {
        DDP_IDX_X, DDP_IDX_Y, DDP_IDX_THETA, DDP_IDX_V, DDP_IDX_A};
    for (int i = 0; i < DDP_TERMINAL_CONSTRAINT_DIM; ++i) {
        out->lx(kRows[i]) += scale(i);
        out->lxx(kRows[i], kRows[i]) += multipliers.terminal_mu(i);
    }
    // 终端阶段恒评估 ESDF（终点避障不抽样）
    if (esdf_constraint_ != nullptr) {
        const auto esdf = esdf_constraint_->evaluate(x(DDP_IDX_X), x(DDP_IDX_Y),
                                                     x(DDP_IDX_THETA));
        out->cost_esdf += esdf_scale * esdf.cost;
        out->lx += esdf_scale * esdf.gradient;
        out->lxx += esdf_scale * esdf.hessian;
    }
}

void DdpCostEvaluator::accumulateAmplitudeConstraints(
    std::size_t k, const DdpState& x, const DdpCostMultiplierState& multipliers,
    DdpStageCostDerivatives* out) const {
    const auto base =
        static_cast<Eigen::Index>(DDP_AMPLITUDE_CONSTRAINT_DIM * k);
    const double v = x(DDP_IDX_V);
    const double a = x(DDP_IDX_A);
    const double delta = x(DDP_IDX_DELTA);
    const double omega = x(DDP_IDX_OMEGA);
    AccumulateAmplitudeInequality(
        v * v - config_.v_max * config_.v_max, 2.0 * v, DDP_IDX_V,
        multipliers.amplitude_lambda(base + DDP_AMP_V),
        multipliers.amplitude_mu(base + DDP_AMP_V), out);
    AccumulateAmplitudeInequality(
        a * a - config_.a_max * config_.a_max, 2.0 * a, DDP_IDX_A,
        multipliers.amplitude_lambda(base + DDP_AMP_A),
        multipliers.amplitude_mu(base + DDP_AMP_A), out);
    AccumulateAmplitudeInequality(
        omega * omega - config_.omega_max * config_.omega_max, 2.0 * omega,
        DDP_IDX_OMEGA, multipliers.amplitude_lambda(base + DDP_AMP_OMEGA),
        multipliers.amplitude_mu(base + DDP_AMP_OMEGA), out);
    AccumulateAmplitudeInequality(
        delta - config_.delta_max, 1.0, DDP_IDX_DELTA,
        multipliers.amplitude_lambda(base + DDP_AMP_DELTA_POS),
        multipliers.amplitude_mu(base + DDP_AMP_DELTA_POS), out);
    AccumulateAmplitudeInequality(
        -delta - config_.delta_max, -1.0, DDP_IDX_DELTA,
        multipliers.amplitude_lambda(base + DDP_AMP_DELTA_NEG),
        multipliers.amplitude_mu(base + DDP_AMP_DELTA_NEG), out);
}

void DdpCostEvaluator::AccumulateAmplitudeInequality(
    double g_value, double g_grad, int state_index, double lambda, double mu,
    DdpStageCostDerivatives* out) {
    // 门控：未违反且乘子为零 → 完全退出代价（值与导数恒零）；
    // g=0 且 λ=0 时同样不激活，代价/梯度在边界处连续
    if (g_value <= 0.0 && lambda <= 0.0) {
        return;
    }
    out->cost_amplitude += lambda * g_value + 0.5 * mu * g_value * g_value;
    out->lx(state_index) += (lambda + mu * g_value) * g_grad;
    out->lxx(state_index, state_index) += mu * g_grad * g_grad;
}

void DdpCostEvaluator::accumulateGatingConstraints(
    std::size_t k, const DdpState& x, const DdpCostMultiplierState& multipliers,
    const DdpGatingPlan& plan, DdpStageCostDerivatives* out) const {
    const auto idx = static_cast<Eigen::Index>(k);
    const double v = x(DDP_IDX_V);
    // 段内符号门 −s·v ≤ 0（AL 不等式）：残差/梯度对 v 线性，GN Hessian
    // 恒为 μ（与幅值不等式同一套门控累加约定：未违反且 λ=0 时完全退出）
    const int sign = plan.sign_gate[k];
    if (sign != 0) {
        const double g = SignGateResidual(sign, v);
        const double lambda = multipliers.gating_sign_lambda(idx);
        const double mu = multipliers.gating_sign_mu(idx);
        if (g > 0.0 || lambda > 0.0) {
            const double g_grad = -static_cast<double>(sign);
            out->cost_gating += lambda * g + 0.5 * mu * g * g;
            out->lx(DDP_IDX_V) += (lambda + mu * g) * g_grad;
            out->lxx(DDP_IDX_V, DDP_IDX_V) += mu * g_grad * g_grad;
        }
    }
    // 接缝零速等式 v = 0（AL 等式）：λ 符号自由，恒激活（等式无门控退出）
    const int seam = plan.seam_lookup[k];
    if (seam >= 0) {
        const double c = SeamResidual(v);
        const double lambda = multipliers.gating_seam_lambda(seam);
        const double mu = multipliers.gating_seam_mu(seam);
        out->cost_gating += lambda * c + 0.5 * mu * c * c;
        out->lx(DDP_IDX_V) += lambda + mu * c;
        out->lxx(DDP_IDX_V, DDP_IDX_V) += mu;
    }
    // 驻留速度帽 |v|−cap ≤ 0（AL 不等式）：活动区 |v|>cap 处处光滑且
    const double cap = plan.dwell_v_cap[k];
    if (cap > 0.0) {
        const double g = DwellResidual(v, cap);
        const double lambda = multipliers.gating_dwell_lambda(idx);
        const double mu = multipliers.gating_dwell_mu(idx);
        if (g > 0.0 || lambda > 0.0) {
            const double g_grad = (v > 0.0) ? 1.0 : (v < 0.0) ? -1.0 : 0.0;
            out->cost_gating += lambda * g + 0.5 * mu * g * g;
            out->lx(DDP_IDX_V) += (lambda + mu * g) * g_grad;
            out->lxx(DDP_IDX_V, DDP_IDX_V) += mu * g_grad * g_grad;
        }
    }
}
}  // namespace apa_post_processor
