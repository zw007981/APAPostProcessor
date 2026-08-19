#pragma once

#include <Eigen/Core>
#include <cstddef>
#include <limits>
#include <vector>

#include "bicycle_dynamics.h"
#include "ilqr_config.h"
#include "ilqr_reference_builder.h"
#include "esdf_constraint.h"

namespace apa_post_processor {
using iLQRControlHessian =
    Eigen::Matrix<double, ILQR_CONTROL_DIM, ILQR_CONTROL_DIM>;
using iLQRControlStateHessian =
    Eigen::Matrix<double, ILQR_CONTROL_DIM, ILQR_STATE_DIM>;
// 每步 5 个幅值不等式：v²/a²/ω² 平方形态 + δ 双侧线性（v² 形态对符号中性，保 v
// 过零自由）
inline constexpr int ILQR_AMPLITUDE_CONSTRAINT_DIM = 5;
inline constexpr int ILQR_AMP_V = 0;
inline constexpr int ILQR_AMP_A = 1;
inline constexpr int ILQR_AMP_OMEGA = 2;
inline constexpr int ILQR_AMP_DELTA_POS = 3;
inline constexpr int ILQR_AMP_DELTA_NEG = 4;
// 终点等式 5 维 [x, y, θ, v, a]；δ_N/ω_N 不约束
inline constexpr int ILQR_TERMINAL_CONSTRAINT_DIM = 5;
// 阶段二门控计划：三类门控只作用于运行阶段，严禁在阶段一施加（会摧毁融化机制）
struct iLQRGatingPlan {
    // 逐节点 −s·v ≤ 0，0=无门
    std::vector<int> sign_gate;
    // 接缝零速等式节点索引
    std::vector<std::size_t> seam_indices;
    // O(1) 接缝查表
    std::vector<int> seam_lookup;
    // 驻留速度帽 g=|v|−cap ≤ 0
    std::vector<double> dwell_v_cap;
};
// AL 乘子状态：外层 AL 维护，本层只消费
struct iLQRCostMultiplierState {
    static iLQRCostMultiplierState MakeZero(std::size_t num_steps);
    static iLQRCostMultiplierState MakeStageTwoZero(std::size_t num_steps,
                                                   std::size_t num_seams);
    // 幅值 λ (5N)
    Eigen::VectorXd amplitude_lambda;
    // 幅值 μ
    Eigen::VectorXd amplitude_mu;
    // 终点 λ
    Eigen::Matrix<double, ILQR_TERMINAL_CONSTRAINT_DIM, 1> terminal_lambda;
    // 终点 μ
    Eigen::Matrix<double, ILQR_TERMINAL_CONSTRAINT_DIM, 1> terminal_mu;
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
struct iLQRCostInput {
    // 跟踪权重
    double tracking_weight{0.0};
    // 退火豁免点
    const std::vector<bool>* anneal_exempt_mask{nullptr};
    // 阶段二模式
    const iLQRGatingPlan* gating_plan{nullptr};
    // 与 AL 罚同步增长，保持避障/约束交换比恒定
    double esdf_scale{1.0};
    // 线搜索早停阈值：不含 ESDF 的廉价小计（完整代价的下界）超过
    // 该值即跳过 ESDF 求值提前返回；默认 +inf 不筛选，行为不变
    double screen_cost_threshold{std::numeric_limits<double>::infinity()};
};
// dt 因子约定：平滑/跟踪项含 dt，AL/ESDF 为点态量不乘 dt（否则 AL 约束被 dt
// 稀释）
struct iLQRStageCostDerivatives {
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
    iLQRState lx{iLQRState::Zero()};
    iLQRControl lu{iLQRControl::Zero()};
    iLQRStateHessian lxx{iLQRStateHessian::Zero()};
    iLQRControlHessian luu{iLQRControlHessian::Zero()};
    iLQRControlStateHessian lux{iLQRControlStateHessian::Zero()};
};
struct iLQRCostEvaluation {
    iLQRAlignedVec<iLQRStageCostDerivatives> stages;
    // 全轨迹总代价
    double total_cost{0.0};
    // 廉价小计超阈、ESDF 求值被跳过时置真（此时 total_cost 仅为不含
    // ESDF 的下界，调用方须按拒绝路径处理或全量重判）；默认 false
    bool esdf_screened_out{false};
};
// 代价与约束统一求值层：平滑 + 跟踪 + 幅值 AL + ESDF + 终点 AL + 阶段二门控
class iLQRCostEvaluator {
   public:
    // esdf_constraint 可为 nullptr（无地图场景）
    iLQRCostEvaluator(const iLQRConfig& config,
                     const iLQREsdfConstraint* esdf_constraint);
    // 全轨迹求值；screen_cost_threshold 供线搜索早停：廉价小计
    // （不含 ESDF，恒 ≤ 完整代价）超阈即跳过 ESDF 求值提前返回，
    // 默认 +inf 时行为与全量求值一致
    iLQRCostEvaluation evaluate(const iLQRReference& reference,
                               const iLQRAlignedVec<iLQRState>& states,
                               const iLQRAlignedVec<iLQRControl>& controls,
                               const iLQRCostMultiplierState& multipliers,
                               const iLQRCostInput& input) const;
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
    const iLQREsdfConstraint* esdfConstraint() const { return esdf_constraint_; }

   protected:
    // 运行阶段 k（0..N-1）：平滑 + 跟踪 + 幅值 AL + 阶段二门控
    // （ESDF 项由 evaluate 阶段 B 单独补入，见线搜索早停设计）
    void evaluateRunningStage(std::size_t k, const iLQRReference& reference,
                              const iLQRAlignedVec<iLQRState>& states,
                              const iLQRAlignedVec<iLQRControl>& controls,
                              const iLQRCostMultiplierState& multipliers,
                              const iLQRCostInput& input,
                              iLQRStageCostDerivatives* out) const;
    // 终端阶段 N：终点 AL 等式（ESDF 由 evaluate 阶段 B 单独补入）
    void evaluateTerminalStage(const iLQRReference& reference,
                               const iLQRAlignedVec<iLQRState>& states,
                               const iLQRCostMultiplierState& multipliers,
                               iLQRStageCostDerivatives* out) const;
    // ESDF 双 margin 惩罚单阶段补入（运行/终端阶段共用）：与廉价项
    // 累加顺序固定为「先廉价后 ESDF」，与全量求值路径逐位一致
    void accumulateEsdfStage(const iLQRState& x, double esdf_scale,
                             iLQRStageCostDerivatives* out) const;
    // 幅值 AL 五项累加（v/a/ω 平方形态 + δ 双侧线性形态）
    void accumulateAmplitudeConstraints(
        std::size_t k, const iLQRState& x,
        const iLQRCostMultiplierState& multipliers,
        iLQRStageCostDerivatives* out) const;
    // 阶段二门控累加（仅运行阶段、仅 gating_plan 在场时调用）：
    void accumulateGatingConstraints(std::size_t k, const iLQRState& x,
                                     const iLQRCostMultiplierState& multipliers,
                                     const iLQRGatingPlan& plan,
                                     iLQRStageCostDerivatives* out) const;
    // 单个 AL 不等式的门控累加：g>0 或 λ>0 时计入 λg+½μg²、
    // 梯度 (λ+μg)·∂g 与 GN Hessian μ·∂g²，否则不产生任何贡献
    static void AccumulateAmplitudeInequality(double g_value, double g_grad,
                                              int state_index, double lambda,
                                              double mu,
                                              iLQRStageCostDerivatives* out);

   protected:
    // 配置
    iLQRConfig config_;
    // ESDF 双 margin 惩罚（不持有所有权，可为空）
    const iLQREsdfConstraint* esdf_constraint_;
};
}  // namespace apa_post_processor
