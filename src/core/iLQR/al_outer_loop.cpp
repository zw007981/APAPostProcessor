#include "al_outer_loop.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <stdexcept>

#include "../../util/constants.h"

namespace apa_post_processor {
AlOuterLoop::AlOuterLoop(AlOuterLoopConfig config, iLQRCostConfig cost_config)
    : config_(config), cost_config_(cost_config) {
    // 调度参数校验：错误值会静默污染外层行为，构造期显式拒绝
    if (config_.max_outer_iterations <= 0) {
        throw std::invalid_argument("AlOuterLoop: 外层迭代上限必须为正");
    }
    if (!(config_.terminal_position_tol > 0.0) ||
        !(config_.terminal_heading_tol_deg > 0.0) ||
        !(config_.inequality_tol > 0.0) || !(config_.defect_tol > 0.0)) {
        throw std::invalid_argument("AlOuterLoop: 终止判据容差必须为正");
    }
    if (!(config_.mu_min > 0.0) || !std::isfinite(config_.mu_max) ||
        !(config_.mu_max >= config_.mu_min)) {
        throw std::invalid_argument(
            "AlOuterLoop: 罚权重 clip 区间必须满足 0<μ_min<=μ_max");
    }
    if (!(config_.first_round_mu > 0.0) || !(config_.epsilon_mu > 0.0) ||
        !(config_.amplitude_mu_initial > 0.0) ||
        !std::isfinite(config_.amplitude_mu_initial)) {
        throw std::invalid_argument(
            "AlOuterLoop: 首轮罚权重/幅值初始罚权重与 ε_μ 必须为正有限");
    }
    if (!(config_.mu_gate_kappa > 0.0 && config_.mu_gate_kappa < 1.0) ||
        !(config_.mu_growth_factor > 1.0) ||
        !(config_.anneal_gamma > 0.0 && config_.anneal_gamma < 1.0)) {
        throw std::invalid_argument(
            "AlOuterLoop: 门控/增长/退火参数必须满足 0<κ<1、φ>1、0<γ<1");
    }
    // ESDF 逐轮量级调度：增长率与上限都不得小于 1（小于 1 等于随轮次
    // 削弱避障，与本机制的意图相反），且必须为有限值
    if (!(config_.esdf_scale_growth >= 1.0) ||
        !(config_.esdf_scale_max >= 1.0) ||
        !std::isfinite(config_.esdf_scale_growth) ||
        !std::isfinite(config_.esdf_scale_max)) {
        throw std::invalid_argument(
            "AlOuterLoop: ESDF 逐轮量级增长率/上限必须为 >=1 的有限值");
    }
    // 幅值边界进入 g 公式（量测与代价求值层同口径），w_ref,0 进入退火基准
    if (!(cost_config_.v_max > 0.0) || !(cost_config_.a_max > 0.0) ||
        !(cost_config_.omega_max > 0.0) || !(cost_config_.delta_max > 0.0) ||
        !(cost_config_.weight_ref_base >= 0.0) ||
        !std::isfinite(cost_config_.weight_ref_base)) {
        throw std::invalid_argument(
            "AlOuterLoop: 代价配置幅值边界必须为正、w_ref,0 非负有限");
    }
    // 归一化尺度一次算出（5N 循环内不再逐元素分派）
    for (int i = 0; i < ILQR_AMPLITUDE_CONSTRAINT_DIM; ++i) {
        amplitude_scales_[static_cast<std::size_t>(i)] =
            AmplitudeScale(cost_config_, i);
    }
    reset();
}

void AlOuterLoop::reset() {
    round_ = 0;
    mu_terminal_ = config_.first_round_mu;
    mu_amplitude_ = config_.amplitude_mu_initial;
    // 逐元素模式的状态在下一次更新/初始化乘子时按 5N 尺寸重建
    amplitude_mu_vec_.resize(0);
    prev_element_violation_.resize(0);
    mu_calibrated_ = config_.first_round_mu;
    mu_calibrated_flag_ = false;
    prev_terminal_violation_ = -1.0;
    prev_amplitude_violation_ = -1.0;
    mu_increase_count_ = 0;
    last_terminal_growth_ = false;
    last_amplitude_growth_ = false;
}

double AlOuterLoop::trackingWeight() const {
    // 纯几何退火：w_ref(r) = w_ref,0·γ^r，r 为当前外层轮次（豁免点
    // 不衰减由代价层的退火豁免掩码保证）
    return cost_config_.weight_ref_base *
           std::pow(config_.anneal_gamma, static_cast<double>(round_));
}

double AlOuterLoop::esdfScale() const {
    // 跟 AL 轮次 round_：本因子随罚权重增长节奏同步放大（μ 调度同样
    // 挂在 round_ 上），维持 ESDF 与 AL 罚的交换比不随轮次单边倾斜
    return std::min(
        config_.esdf_scale_max,
        std::pow(config_.esdf_scale_growth, static_cast<double>(round_)));
}

std::vector<bool> AlOuterLoop::makeAnnealExemptMask(
    const iLQRReference& reference) const {
    const std::size_t num_poses = reference.poses.size();
    std::vector<bool> mask(num_poses, false);
    if (reference.maneuvers.empty()) {
        // 无 maneuver 元数据（合成参考）：无法识别首末段，不豁免任何点
        return mask;
    }
    if (mask.empty()) {
        // 位姿为空而元数据非空已属数据完整性错误，直接返回空掩码
        // （避免下方 size()-1 下溢），debug 下由断言进一步拦截
        return mask;
    }
    // 首/末 maneuver 覆盖区间（含与相邻段共享的边界点）恒豁免；
    const auto mark = [&mask](const iLQRReferenceManeuver& maneuver) {
        assert(maneuver.begin_index < mask.size() &&
               maneuver.end_index < mask.size());
        const std::size_t end = std::min(maneuver.end_index, mask.size() - 1);
        for (std::size_t k = maneuver.begin_index; k <= end; ++k) {
            mask[k] = true;
        }
    };
    mark(reference.maneuvers.front());
    mark(reference.maneuvers.back());
    return mask;
}

iLQRCostMultiplierState AlOuterLoop::makeInitialMultipliers(
    std::size_t num_steps) const {
    auto multipliers = iLQRCostMultiplierState::MakeZero(num_steps);
    multipliers.amplitude_mu.setConstant(mu_amplitude_);
    multipliers.terminal_mu.setConstant(mu_terminal_);
    return multipliers;
}

double AlOuterLoop::muAmplitude() const {
    // 逐元素模式：返回全元素最大值（病态条件的驱动量——最大元素决定
    // 增广 Hessian 的最坏条件数）；标量广播模式：返回标量值
    if (config_.amplitude_mu_per_element && amplitude_mu_vec_.size() > 0) {
        return amplitude_mu_vec_.maxCoeff();
    }
    return mu_amplitude_;
}

AlConstraintSnapshot AlOuterLoop::measure(
    const iLQRReference& reference, const iLQRAlignedVec<iLQRState>& states,
    const iLQRAlignedVec<iLQRState>& defects) const {
    const std::size_t num_poses = reference.poses.size();
    if (num_poses < 2) {
        throw std::invalid_argument("AlOuterLoop: 参考位姿数量必须 >= 2");
    }
    if (states.size() != num_poses || defects.size() != num_poses) {
        throw std::invalid_argument(
            "AlOuterLoop: 状态/缺陷数量必须与参考位姿一致（N+1）");
    }
    const std::size_t num_steps = num_poses - 1;
    AlConstraintSnapshot snapshot;
    // 幅值不等式残差：与代价求值层同一组公式（v²/a²/ω² 平方形态 +
    snapshot.amplitude_g.resize(
        static_cast<Eigen::Index>(ILQR_AMPLITUDE_CONSTRAINT_DIM * num_steps));
    double violation_sq = 0.0;
    double max_violation = 0.0;
    for (std::size_t k = 0; k < num_steps; ++k) {
        const iLQRState& x = states[k];
        const auto base =
            static_cast<Eigen::Index>(ILQR_AMPLITUDE_CONSTRAINT_DIM * k);
        const double v = x(ILQR_IDX_V);
        const double a = x(ILQR_IDX_A);
        const double delta = x(ILQR_IDX_DELTA);
        const double omega = x(ILQR_IDX_OMEGA);
        snapshot.amplitude_g(base + ILQR_AMP_V) =
            v * v - cost_config_.v_max * cost_config_.v_max;
        snapshot.amplitude_g(base + ILQR_AMP_A) =
            a * a - cost_config_.a_max * cost_config_.a_max;
        snapshot.amplitude_g(base + ILQR_AMP_OMEGA) =
            omega * omega - cost_config_.omega_max * cost_config_.omega_max;
        snapshot.amplitude_g(base + ILQR_AMP_DELTA_POS) =
            delta - cost_config_.delta_max;
        snapshot.amplitude_g(base + ILQR_AMP_DELTA_NEG) =
            -delta - cost_config_.delta_max;
        for (int i = 0; i < ILQR_AMPLITUDE_CONSTRAINT_DIM; ++i) {
            // 量纲归一化：各约束残差除以其自然尺度（构造期预计算），
            const double active = std::max(
                0.0, snapshot.amplitude_g(base + i) /
                         amplitude_scales_[static_cast<std::size_t>(i)]);
            max_violation = std::max(max_violation, active);
            violation_sq += active * active;
        }
    }
    // 终点等式残差：c = [x−xg, y−yg, wrap(θ−θg), v, a]，
    // 角度 wrap 与初值提取/跟踪代价同一实现
    const iLQRState& x_final = states.back();
    const Pose& goal = reference.poses.back();
    snapshot.terminal_c << x_final(ILQR_IDX_X) - goal.x,
        x_final(ILQR_IDX_Y) - goal.y,
        WrapAngle(x_final(ILQR_IDX_THETA) - goal.theta), x_final(ILQR_IDX_V),
        x_final(ILQR_IDX_A);
    snapshot.terminal_position_error =
        std::hypot(snapshot.terminal_c(0), snapshot.terminal_c(1));
    snapshot.terminal_heading_error_deg =
        std::abs(snapshot.terminal_c(2)) * 180.0 / PI;
    // 终点组门控比较量：‖c‖（原始量纲——只与本组历史比较，任意自洽
    snapshot.terminal_violation_norm = snapshot.terminal_c.norm();
    const double terminal_normalized_sq =
        std::pow(snapshot.terminal_c(0) / config_.terminal_position_tol, 2.0) +
        std::pow(snapshot.terminal_c(1) / config_.terminal_position_tol, 2.0) +
        std::pow(snapshot.terminal_c(2) /
                     (config_.terminal_heading_tol_deg * PI / 180.0),
                 2.0) +
        std::pow(snapshot.terminal_c(3) / cost_config_.v_max, 2.0) +
        std::pow(snapshot.terminal_c(4) / cost_config_.a_max, 2.0);
    snapshot.max_amplitude_violation = max_violation;
    snapshot.amplitude_violation_norm = std::sqrt(violation_sq);
    // 打靶缺陷 ‖d‖∞（全节点全分量绝对值最大）
    double defect_inf = 0.0;
    for (const auto& defect : defects) {
        defect_inf = std::max(defect_inf, defect.cwiseAbs().maxCoeff());
    }
    snapshot.defect_norm_inf = defect_inf;
    // 联合违反度范数（诊断量：收敛分析日志消费）：归一化终点范数 +
    snapshot.violation_norm = std::sqrt(terminal_normalized_sq + violation_sq);
    return snapshot;
}

AlTerminationCheck AlOuterLoop::checkTermination(
    const AlConstraintSnapshot& snapshot) const {
    AlTerminationCheck check;
    check.terminal_ok =
        snapshot.terminal_position_error <= config_.terminal_position_tol &&
        snapshot.terminal_heading_error_deg <= config_.terminal_heading_tol_deg;
    check.inequality_ok =
        snapshot.max_amplitude_violation <= config_.inequality_tol;
    check.defect_ok = snapshot.defect_norm_inf <= config_.defect_tol;
    return check;
}

bool AlOuterLoop::update(const AlConstraintSnapshot& snapshot, double base_cost,
                         iLQRCostMultiplierState* multipliers) {
    if (multipliers == nullptr ||
        multipliers->amplitude_lambda.size() != snapshot.amplitude_g.size() ||
        multipliers->amplitude_mu.size() != snapshot.amplitude_g.size()) {
        throw std::invalid_argument(
            "AlOuterLoop: 乘子尺寸必须与约束残差快照一致（5N）");
    }
    // 首轮先标定终点 μ⁰：J_s′/max(‖c‖², ε_μ) clip 进 [μ_min, μ_max]，
    if (!mu_calibrated_flag_) {
        mu_terminal_ =
            Clip(base_cost / std::max(snapshot.terminal_c.squaredNorm(),
                                      config_.epsilon_mu),
                 config_.mu_min, config_.mu_max);
        mu_calibrated_ = mu_terminal_;
        mu_calibrated_flag_ = true;
    }
    // 乘子更新（Hestenes-Powell）：终点等式 λ ← λ+μ_term·c（符号自由）；
    multipliers->terminal_lambda += mu_terminal_ * snapshot.terminal_c;
    if (config_.amplitude_mu_per_element && amplitude_mu_vec_.size() > 0) {
        multipliers->amplitude_lambda =
            (multipliers->amplitude_lambda +
             amplitude_mu_vec_.cwiseProduct(snapshot.amplitude_g))
                .cwiseMax(0.0);
    } else {
        multipliers->amplitude_lambda = (multipliers->amplitude_lambda +
                                         mu_amplitude_ * snapshot.amplitude_g)
                                            .cwiseMax(0.0);
    }
    // 门控 μ 增长（分组独立门控）：终点组仅当终点违反度（归一化）未
    const double terminal_violation = snapshot.terminal_violation_norm;
    const double amplitude_violation = snapshot.amplitude_violation_norm;
    // 边界语义说明：取严格 `>`，即违反度恰好下降到 κ·上轮（含 ±ULP 级
    const bool increase_terminal =
        prev_terminal_violation_ >= 0.0 &&
        terminal_violation > config_.mu_gate_kappa * prev_terminal_violation_;
    const bool increase_amplitude =
        prev_amplitude_violation_ >= 0.0 &&
        amplitude_violation > config_.mu_gate_kappa * prev_amplitude_violation_;
    if (increase_terminal) {
        mu_terminal_ =
            std::min(config_.mu_growth_factor * mu_terminal_, config_.mu_max);
    }
    bool amplitude_grew = false;
    if (config_.amplitude_mu_per_element) {
        // 逐元素门控：仅当本元素的归一化违反度未充分下降才提升其 μ_j
        const auto n = snapshot.amplitude_g.size();
        if (amplitude_mu_vec_.size() != n) {
            amplitude_mu_vec_.setConstant(n, config_.amplitude_mu_initial);
            prev_element_violation_.setConstant(n, -1.0);
        }
        for (Eigen::Index i = 0; i < n; ++i) {
            const int constraint_index =
                static_cast<int>(i % ILQR_AMPLITUDE_CONSTRAINT_DIM);
            const double active =
                std::max(0.0, snapshot.amplitude_g(i) /
                                  amplitude_scales_[static_cast<std::size_t>(
                                      constraint_index)]);
            if (prev_element_violation_(i) >= 0.0 &&
                active > config_.mu_gate_kappa * prev_element_violation_(i)) {
                amplitude_mu_vec_(i) =
                    std::min(config_.mu_growth_factor * amplitude_mu_vec_(i),
                             config_.mu_max);
                amplitude_grew = true;
            }
            prev_element_violation_(i) = active;
        }
        multipliers->amplitude_mu = amplitude_mu_vec_;
    } else {
        if (increase_amplitude) {
            mu_amplitude_ = std::min(config_.mu_growth_factor * mu_amplitude_,
                                     config_.mu_max);
            amplitude_grew = true;
        }
        multipliers->amplitude_mu.setConstant(mu_amplitude_);
    }
    if (increase_terminal || amplitude_grew) {
        ++mu_increase_count_;
    }
    last_terminal_growth_ = increase_terminal;
    last_amplitude_growth_ = amplitude_grew;
    prev_terminal_violation_ = terminal_violation;
    prev_amplitude_violation_ = amplitude_violation;
    // μ 写回乘子状态（下一轮内层即以新罚权重求解）
    multipliers->terminal_mu.setConstant(mu_terminal_);
    ++round_;
    return increase_terminal || amplitude_grew;
}
}  // namespace apa_post_processor
