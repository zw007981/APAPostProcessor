#include "al_outer_loop.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <stdexcept>

#include "../../util/constants.h"

namespace apa_post_processor {
AlOuterLoop::AlOuterLoop(AlOuterLoopConfig config, DdpCostConfig cost_config)
    : config_(config), cost_config_(cost_config) {
    // 调度参数错误会静默污染全部外层行为（罚权重病态增长、退火失效、
    // 判据失真），必须在构造期显式拒绝
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
    // 幅值边界进入 g 公式（量测与代价求值层同口径），w_ref,0 进入退火基准
    if (!(cost_config_.v_max > 0.0) || !(cost_config_.a_max > 0.0) ||
        !(cost_config_.omega_max > 0.0) || !(cost_config_.delta_max > 0.0) ||
        !(cost_config_.weight_ref_base >= 0.0) ||
        !std::isfinite(cost_config_.weight_ref_base)) {
        throw std::invalid_argument(
            "AlOuterLoop: 代价配置幅值边界必须为正、w_ref,0 非负有限");
    }
    reset();
}

void AlOuterLoop::reset() {
    round_ = 0;
    mu_terminal_ = config_.first_round_mu;
    mu_amplitude_ = config_.amplitude_mu_initial;
    mu_calibrated_ = config_.first_round_mu;
    mu_calibrated_flag_ = false;
    prev_terminal_violation_ = -1.0;
    prev_amplitude_violation_ = -1.0;
    mu_increase_count_ = 0;
}

double AlOuterLoop::trackingWeight() const {
    return cost_config_.weight_ref_base *
           std::pow(config_.anneal_gamma, static_cast<double>(round_));
}

std::vector<bool> AlOuterLoop::makeAnnealExemptMask(
    const DdpReference& reference) const {
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
    // 元数据索引越界属前端数据完整性错误，debug 下显式拦截（release
    // 下按区间裁剪兜底，避免非法内存访问）
    const auto mark = [&mask](const DdpReferenceManeuver& maneuver) {
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

DdpCostMultiplierState AlOuterLoop::makeInitialMultipliers(
    std::size_t num_steps) const {
    auto multipliers = DdpCostMultiplierState::MakeZero(num_steps);
    multipliers.amplitude_mu.setConstant(mu_amplitude_);
    multipliers.terminal_mu.setConstant(mu_terminal_);
    return multipliers;
}

AlConstraintSnapshot AlOuterLoop::measure(
    const DdpReference& reference, const DdpAlignedVec<DdpState>& states,
    const DdpAlignedVec<DdpState>& defects) const {
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
    // δ 双侧线性形态），同一布局（阶段 k 的 5 个约束位于 [5k, 5k+5)）
    snapshot.amplitude_g.resize(
        static_cast<Eigen::Index>(DDP_AMPLITUDE_CONSTRAINT_DIM * num_steps));
    double violation_sq = 0.0;
    double max_violation = 0.0;
    for (std::size_t k = 0; k < num_steps; ++k) {
        const DdpState& x = states[k];
        const auto base =
            static_cast<Eigen::Index>(DDP_AMPLITUDE_CONSTRAINT_DIM * k);
        const double v = x(DDP_IDX_V);
        const double a = x(DDP_IDX_A);
        const double delta = x(DDP_IDX_DELTA);
        const double omega = x(DDP_IDX_OMEGA);
        snapshot.amplitude_g(base + DDP_AMP_V) =
            v * v - cost_config_.v_max * cost_config_.v_max;
        snapshot.amplitude_g(base + DDP_AMP_A) =
            a * a - cost_config_.a_max * cost_config_.a_max;
        snapshot.amplitude_g(base + DDP_AMP_OMEGA) =
            omega * omega - cost_config_.omega_max * cost_config_.omega_max;
        snapshot.amplitude_g(base + DDP_AMP_DELTA_POS) =
            delta - cost_config_.delta_max;
        snapshot.amplitude_g(base + DDP_AMP_DELTA_NEG) =
            -delta - cost_config_.delta_max;
        for (int i = 0; i < DDP_AMPLITUDE_CONSTRAINT_DIM; ++i) {
            const double active = std::max(0.0, snapshot.amplitude_g(base + i));
            max_violation = std::max(max_violation, active);
            violation_sq += active * active;
        }
    }
    // 终点等式残差：c = [x−xg, y−yg, wrap(θ−θg), v, a]，
    // 角度 wrap 与初值提取/跟踪代价同一实现
    const DdpState& x_final = states.back();
    const Pose& goal = reference.poses.back();
    snapshot.terminal_c << x_final(DDP_IDX_X) - goal.x,
        x_final(DDP_IDX_Y) - goal.y,
        WrapAngle(x_final(DDP_IDX_THETA) - goal.theta), x_final(DDP_IDX_V),
        x_final(DDP_IDX_A);
    snapshot.terminal_position_error =
        std::hypot(snapshot.terminal_c(0), snapshot.terminal_c(1));
    snapshot.terminal_heading_error_deg =
        std::abs(snapshot.terminal_c(2)) * 180.0 / PI;
    snapshot.max_amplitude_violation = max_violation;
    // 打靶缺陷 ‖d‖∞（全节点全分量绝对值最大）
    double defect_inf = 0.0;
    for (const auto& defect : defects) {
        defect_inf = std::max(defect_inf, defect.cwiseAbs().maxCoeff());
    }
    snapshot.defect_norm_inf = defect_inf;
    // 联合违反度范数：终点残差与激活不等式违反同度量（缺陷由 merit
    // 机制内建处理，不参与 AL 门控比较）
    snapshot.violation_norm =
        std::sqrt(snapshot.terminal_c.squaredNorm() + violation_sq);
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
                         DdpCostMultiplierState* multipliers) {
    if (multipliers == nullptr ||
        multipliers->amplitude_lambda.size() != snapshot.amplitude_g.size() ||
        multipliers->amplitude_mu.size() != snapshot.amplitude_g.size()) {
        throw std::invalid_argument(
            "AlOuterLoop: 乘子尺寸必须与约束残差快照一致（5N）");
    }
    // 首轮先标定终点 μ⁰：J_s′/max(‖c‖², ε_μ) clip 进 [μ_min, μ_max]，
    // 替换首轮的临时终点权重（标定值从首轮更新起生效）；幅值罚权重
    // 不参与标定（物理边界量级已知，靠 λ 累积与分组门控渐硬即可）
    if (!mu_calibrated_flag_) {
        mu_terminal_ =
            Clip(base_cost / std::max(snapshot.terminal_c.squaredNorm(),
                                      config_.epsilon_mu),
                 config_.mu_min, config_.mu_max);
        mu_calibrated_ = mu_terminal_;
        mu_calibrated_flag_ = true;
    }
    // 乘子更新（Hestenes-Powell）：终点等式 λ ← λ+μ_term·c（符号自由）；
    // 幅值不等式 λ ← max(0, λ+μ_amp·g)（投影恒非负，约束持续可行时
    // 乘子自然回落）
    multipliers->terminal_lambda += mu_terminal_ * snapshot.terminal_c;
    multipliers->amplitude_lambda =
        (multipliers->amplitude_lambda + mu_amplitude_ * snapshot.amplitude_g)
            .cwiseMax(0.0);
    // 门控 μ 增长（分组独立门控）：终点组仅当 ‖c‖ 未充分下降
    // （> κ·上轮）才提升 μ_term，幅值组仅当激活违反范数未充分下降才
    // 提升 μ_amp——两组违反度量级与硬化动态不同（终点残差需快速钉死
    // 双指标、物理边界靠 λ 累积渐硬），捆绑增长会让单点顽固违反
    // （如换挡区 δ）把终端 μ 一并推入病态。标定轮（首轮）只标定不
    // 增长——μ⁰ 刚按违反度量级标定，立即 φ 倍增长与标定自相矛盾
    // （实测会把次轮内层直接压入病态），门控自次轮起生效
    const double terminal_violation = snapshot.terminal_c.norm();
    double amplitude_sq = 0.0;
    for (Eigen::Index i = 0; i < snapshot.amplitude_g.size(); ++i) {
        amplitude_sq += std::pow(std::max(0.0, snapshot.amplitude_g(i)), 2.0);
    }
    const double amplitude_violation = std::sqrt(amplitude_sq);
    // 边界语义说明：取严格 `>`，即违反度恰好下降到 κ·上轮（含 ±ULP 级
    // 抖动）时判为"已充分下降"、本轮不增长。门控阈值 κ 本身是工程启发
    // 常数，边界落点无论判向哪一侧，效果都只是一轮 μ 是否提升——判
    // "充分"则推迟一轮硬化、判"不足"则提前一轮硬化，下一轮量测会
    // 自动修正排程方向，无累积误差；故不引入额外绝对容差（反而会把
    // 排程行为绑定到违反度的具体量级上）
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
    if (increase_amplitude) {
        mu_amplitude_ =
            std::min(config_.mu_growth_factor * mu_amplitude_, config_.mu_max);
    }
    if (increase_terminal || increase_amplitude) {
        ++mu_increase_count_;
    }
    prev_terminal_violation_ = terminal_violation;
    prev_amplitude_violation_ = amplitude_violation;
    // μ 写回乘子状态（下一轮内层即以新罚权重求解）
    multipliers->amplitude_mu.setConstant(mu_amplitude_);
    multipliers->terminal_mu.setConstant(mu_terminal_);
    ++round_;
    return increase_terminal || increase_amplitude;
}
}  // namespace apa_post_processor
