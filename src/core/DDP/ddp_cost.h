#pragma once

#include <Eigen/Core>
#include <cstddef>
#include <vector>

#include "bicycle_dynamics.h"
#include "ddp_reference_builder.h"
#include "esdf_constraint.h"

namespace apa_post_processor {
using DdpControlHessian =
    Eigen::Matrix<double, DDP_CONTROL_DIM, DDP_CONTROL_DIM>;
using DdpControlStateHessian =
    Eigen::Matrix<double, DDP_CONTROL_DIM, DDP_STATE_DIM>;
// 每步 5 个幅值不等式：v²/a²/ω² 平方形态 + δ 双侧线性（v² 形态对符号中性，保 v
// 过零自由）
inline constexpr int DDP_AMPLITUDE_CONSTRAINT_DIM = 5;
inline constexpr int DDP_AMP_V = 0;
inline constexpr int DDP_AMP_A = 1;
inline constexpr int DDP_AMP_OMEGA = 2;
inline constexpr int DDP_AMP_DELTA_POS = 3;
inline constexpr int DDP_AMP_DELTA_NEG = 4;
// 终点等式 5 维 [x, y, θ, v, a]；δ_N/ω_N 不约束
inline constexpr int DDP_TERMINAL_CONSTRAINT_DIM = 5;
// 阶段二门控计划：三类门控只作用于运行阶段，严禁在阶段一施加（会摧毁融化机制）
struct DdpGatingPlan {
    // 逐节点 −s·v ≤ 0，0=无门
    std::vector<int> sign_gate;
    // 接缝零速等式节点索引
    std::vector<std::size_t> seam_indices;
    // O(1) 接缝查表
    std::vector<int> seam_lookup;
    // 驻留速度帽 g=|v|−cap ≤ 0
    std::vector<double> dwell_v_cap;
};
struct DdpCostConfig {
    // 跃度权重
    double weight_jerk{1.0};
    // 转角加加速度权重
    double weight_steer_accel{1.0};
    // 跟踪位置权重基准值
    double weight_ref_base{10.0};
    // 跟踪朝向权重
    double weight_theta{5.0};
    // 速度上限 (m/s)
    double v_max{1.5};
    // 加速度上限 (m/s²)
    double a_max{1.0};
    // 转角速率上限 (rad/s)
    double omega_max{0.5};
    // 前轮转角上限 (rad)
    double delta_max{0.55};
};
// AL 乘子状态：外层 AL 维护，本层只消费
struct DdpCostMultiplierState {
    static DdpCostMultiplierState MakeZero(std::size_t num_steps);
    static DdpCostMultiplierState MakeStageTwoZero(std::size_t num_steps,
                                                   std::size_t num_seams);
    // 幅值 λ (5N)
    Eigen::VectorXd amplitude_lambda;
    // 幅值 μ
    Eigen::VectorXd amplitude_mu;
    // 终点 λ
    Eigen::Matrix<double, DDP_TERMINAL_CONSTRAINT_DIM, 1> terminal_lambda;
    // 终点 μ
    Eigen::Matrix<double, DDP_TERMINAL_CONSTRAINT_DIM, 1> terminal_mu;
    // 阶段二门控（默认空 = 阶段一模式）
    Eigen::VectorXd gating_sign_lambda;
    // 符号门 μ
    Eigen::VectorXd gating_sign_mu;
    // 接缝 λ
    Eigen::VectorXd gating_seam_lambda;
    // 接缝 μ
    Eigen::VectorXd gating_seam_mu;
    // 驻留帽 λ
    Eigen::VectorXd gating_dwell_lambda;
    // 驻留帽 μ
    Eigen::VectorXd gating_dwell_mu;
};
// 单次全轨迹求值的外部输入
struct DdpCostInput {
    // 跟踪权重
    double tracking_weight{0.0};
    // 退火豁免点
    const std::vector<bool>* anneal_exempt_mask{nullptr};
    // 阶段二模式
    const DdpGatingPlan* gating_plan{nullptr};
    // 与 AL 罚同步增长，保持避障/约束交换比恒定
    double esdf_scale{1.0};
};
// dt 因子约定：平滑/跟踪项含 dt，AL/ESDF 为点态量不乘 dt（否则 AL 约束被 dt
// 稀释）
struct DdpStageCostDerivatives {
    // 平滑代价
    double cost_smooth{0.0};
    // 跟踪代价
    double cost_tracking{0.0};
    // 幅值 AL 代价
    double cost_amplitude{0.0};
    // ESDF 代价
    double cost_esdf{0.0};
    // 终点 AL 代价
    double cost_terminal{0.0};
    // 门控代价
    double cost_gating{0.0};
    double totalCost() const {
        return cost_smooth + cost_tracking + cost_amplitude + cost_esdf +
               cost_terminal + cost_gating;
    }
    DdpState lx{DdpState::Zero()};
    DdpControl lu{DdpControl::Zero()};
    DdpStateHessian lxx{DdpStateHessian::Zero()};
    DdpControlHessian luu{DdpControlHessian::Zero()};
    DdpControlStateHessian lux{DdpControlStateHessian::Zero()};
};
struct DdpCostEvaluation {
    DdpAlignedVec<DdpStageCostDerivatives> stages;
    // 全轨迹总代价
    double total_cost{0.0};
};
// 代价与约束统一求值层：平滑 + 跟踪 + 幅值 AL + ESDF + 终点 AL + 阶段二门控
class DdpCostEvaluator {
   public:
    // esdf_constraint 可为 nullptr（无地图场景）
    DdpCostEvaluator(DdpCostConfig config,
                     const DdpEsdfConstraint* esdf_constraint);
    // 全轨迹求值
    // 两者均不在场即阶段一模式，门控项恒为零
    DdpCostEvaluation evaluate(const DdpReference& reference,
                               const DdpAlignedVec<DdpState>& states,
                               const DdpAlignedVec<DdpControl>& controls,
                               const DdpCostMultiplierState& multipliers,
                               const DdpCostInput& input) const;
    // 门控残差公式的唯一来源（外层量测/乘子更新必须复用，禁止另写副本）：
    // 符号门不等式 −s·v ≤ 0 的残差
    static double SignGateResidual(int sign, double v) { return -sign * v; }
    // 接缝零速等式 v = 0 的残差
    static double SeamResidual(double v) { return v; }
    // 驻留帽不等式 |v|−cap ≤ 0 的残差（活动区梯度模恒为 1、对符号中性）
    static double DwellResidual(double v, double cap) {
        return std::abs(v) - cap;
    }
    // ESDF 惩罚组件只读访问（可为空：无地图场景只评估运动学项；
    // L8.3 定义域守卫据此判定守卫是否可用）
    const DdpEsdfConstraint* esdfConstraint() const { return esdf_constraint_; }

   protected:
    // 运行阶段 k（0..N-1）：平滑 + 跟踪 + 幅值 AL + ESDF
    void evaluateRunningStage(std::size_t k, const DdpReference& reference,
                              const DdpAlignedVec<DdpState>& states,
                              const DdpAlignedVec<DdpControl>& controls,
                              const DdpCostMultiplierState& multipliers,
                              const DdpCostInput& input,
                              DdpStageCostDerivatives* out) const;
    // 终端阶段 N：终点 AL 等式 + ESDF（恒评估，不做时间轴抽样）
    void evaluateTerminalStage(const DdpReference& reference,
                               const DdpAlignedVec<DdpState>& states,
                               const DdpCostMultiplierState& multipliers,
                               double esdf_scale,
                               DdpStageCostDerivatives* out) const;
    // 幅值 AL 五项累加（v/a/ω 平方形态 + δ 双侧线性形态）
    void accumulateAmplitudeConstraints(
        std::size_t k, const DdpState& x,
        const DdpCostMultiplierState& multipliers,
        DdpStageCostDerivatives* out) const;
    // 阶段二门控累加（仅运行阶段、仅 gating_plan 在场时调用）：
    void accumulateGatingConstraints(std::size_t k, const DdpState& x,
                                     const DdpCostMultiplierState& multipliers,
                                     const DdpGatingPlan& plan,
                                     DdpStageCostDerivatives* out) const;
    // 单个 AL 不等式的门控累加：g>0 或 λ>0 时计入 λg+½μg²、
    // 梯度 (λ+μg)·∂g 与 GN Hessian μ·∂g²，否则不产生任何贡献
    static void AccumulateAmplitudeInequality(double g_value, double g_grad,
                                              int state_index, double lambda,
                                              double mu,
                                              DdpStageCostDerivatives* out);

   protected:
    // 配置
    DdpCostConfig config_;
    // ESDF 双 margin 惩罚（不持有所有权，可为空）
    const DdpEsdfConstraint* esdf_constraint_;
};
}  // namespace apa_post_processor
