#pragma once

#include <Eigen/Core>
#include <array>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <vector>

#include "ilqr_config.h"
#include "ilqr_cost.h"
#include "ilqr_reference_builder.h"

namespace apa_post_processor {
// 约束残差快照：λ 更新消费原始残差，聚合违反度一律归一化（两类量纲不可混用）
struct AlConstraintSnapshot {
    // 原始物理量纲，供 λ 更新
    Eigen::VectorXd amplitude_g;
    // 原始物理量纲
    Eigen::Matrix<double, ILQR_TERMINAL_CONSTRAINT_DIM, 1> terminal_c;
    // 归一化，供终止判据
    double max_amplitude_violation{0.0};
    // 原始 ‖c‖，供终点组门控环比
    double terminal_violation_norm{0.0};
    // 归一化，供幅值组门控环比
    double amplitude_violation_norm{0.0};
    // 终点位置误差 (m)
    double terminal_position_error{0.0};
    // 终点朝向误差 (deg)
    double terminal_heading_error_deg{0.0};
    // 打靶缺陷 ‖d‖∞
    double defect_norm_inf{0.0};
    // 归一化诊断量，仅日志消费
    double violation_norm{0.0};
};
struct AlTerminationCheck {
    // 终点达标
    bool terminal_ok{false};
    // 不等式达标
    bool inequality_ok{false};
    // 缺陷达标
    bool defect_ok{false};
    bool converged() const { return terminal_ok && inequality_ok && defect_ok; }
};
// AL 外层状态机：乘子更新、门控 μ 增长、退火调度、联合终止判据
class AlOuterLoop {
   public:
    // 构造时校验调度参数合法性（容差/罚权重区间/增长退火参数）
    AlOuterLoop(const iLQRConfig& config);
    // 当前外层轮次（从 0 起）
    int round() const { return round_; }
    // 当前轮次的几何跟踪权重 w_ref(r) = w_ref,0·γ^r
    double trackingWeight() const;
    // 当前轮次的 ESDF 惩罚放大因子（与 μ 同步增长）
    double esdfScale() const;
    // 首/末 maneuver 恒豁免退火（承载起点状态与终点语义）
    std::vector<bool> makeAnnealExemptMask(const iLQRReference& reference) const;
    // 初始乘子构造：λ 全零、μ 填初始值
    iLQRCostMultiplierState makeInitialMultipliers(std::size_t num_steps) const;
    // 约束违反度量：幅值不等式（归一化）+ 终点等式 + 打靶缺陷
    AlConstraintSnapshot measure(const iLQRReference& reference,
                                 const iLQRAlignedVec<iLQRState>& states,
                                 const iLQRAlignedVec<iLQRState>& defects) const;
    // 终止判据：终点/不等式/缺陷三项同时达标
    AlTerminationCheck checkTermination(
        const AlConstraintSnapshot& snapshot) const;
    // μ⁰ = clip(J_s′/max(‖c‖², ε_μ), μ_min, μ_max)；首轮只标定不增长
    // 返回是否有任一组 μ 提升
    bool update(const AlConstraintSnapshot& snapshot, double base_cost,
                iLQRCostMultiplierState* multipliers);
    // 终点组当前罚权重
    double mu() const { return mu_terminal_; }
    // 幅值组当前罚权重（逐元素模式取最大元素，标量模式取标量值）
    double muAmplitude() const;
    // 上一轮更新中终点组是否触发增长（提前退出判据消费）
    bool wantedTerminalGrowth() const { return last_terminal_growth_; }
    // 上一轮更新中幅值组是否触发增长（提前退出判据消费）
    bool wantedAmplitudeGrowth() const { return last_amplitude_growth_; }
    // 标定后的 μ⁰（首轮后生效）
    double calibratedMu() const { return mu_calibrated_; }
    // 累计 μ 增长次数
    std::int64_t mu_increase_count() const { return mu_increase_count_; }
    // 复位到首轮状态：轮次归零、μ 归初值、清除标定标志与历史违反度
    void reset();

   protected:
    static constexpr double Clip(double value, double lo, double hi) {
        return std::min(std::max(value, lo), hi);
    }
    // 归一化尺度：v²/a²/ω² 除以 2×上限²，δ 除以上限——同 inequality_tol
    // 对各量一视同仁
    static constexpr double AmplitudeScale(const iLQRConfig& config,
                                           int constraint_index) {
        switch (constraint_index) {
            case ILQR_AMP_V:
                return 2.0 * config.cost_v_max * config.cost_v_max;
            case ILQR_AMP_A:
                return 2.0 * config.cost_a_max * config.cost_a_max;
            case ILQR_AMP_OMEGA:
                return 2.0 * config.cost_omega_max * config.cost_omega_max;
            case ILQR_AMP_DELTA_POS:
            case ILQR_AMP_DELTA_NEG:
                return config.cost_delta_max;
            default:
                throw std::logic_error(
                    "AlOuterLoop: 未登记的幅值约束索引（扩展约束维度时"
                    "必须同步补充归一化尺度）");
        }
    }

   protected:
    // 配置
    iLQRConfig config_;
    std::array<double, ILQR_AMPLITUDE_CONSTRAINT_DIM> amplitude_scales_{};
    // 当前外层轮次
    int round_{0};
    // 终点罚权重
    double mu_terminal_{0.0};
    // 幅值罚权重
    double mu_amplitude_{0.0};
    // 逐元素门控启用时维护
    Eigen::VectorXd amplitude_mu_vec_;
    // 上一轮逐元素违反度
    Eigen::VectorXd prev_element_violation_;
    // 标定 μ⁰
    double mu_calibrated_{0.0};
    // μ⁰ 已标定标志
    bool mu_calibrated_flag_{false};
    // <0 = 尚无历史
    double prev_terminal_violation_{-1.0};
    // 上一轮幅值违反度
    double prev_amplitude_violation_{-1.0};
    // μ 增长计数
    std::int64_t mu_increase_count_{0};
    // 上一轮更新中终点组是否触发增长（门控判定结果）
    bool last_terminal_growth_{false};
    // 上一轮更新中幅值组是否触发增长（门控判定结果）
    bool last_amplitude_growth_{false};
};
}  // namespace apa_post_processor
