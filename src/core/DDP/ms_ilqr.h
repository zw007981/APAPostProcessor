#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "bicycle_dynamics.h"
#include "box_qp.h"
#include "ddp_cost.h"
#include "ddp_reference_builder.h"

namespace apa_post_processor {
// MS-iLQR 内层配置：控制盒、迭代/收敛、LM 正则化、线搜索与 merit 罚参数
struct MsIlqrConfig {
    // 跃度上限 (m/s³)
    double jerk_max{1.5};
    // 转角加加速度上限 (rad/s²)
    double steer_accel_max{1.0};
    // 迭代上限
    int max_iterations{50};
    // 代价变化容差
    double cost_change_tol{1e-6};
    // 梯度范数容差
    double gradient_tol{1e-8};
    // 收敛的可行性守卫：缺陷 ‖d‖∞ 必须降到本容差内才允许判收敛（防大 μ
    // 下尺度盲误判）
    double convergence_defect_tol{1e-3};
    // 正则化初值
    double reg_initial{1e-4};
    // 正则化下界
    double reg_min{1e-9};
    // 正则化上界
    double reg_max{1e9};
    // 正则化增大倍率
    double reg_increase{10.0};
    // 正则化缩小倍率
    double reg_decrease{0.5};
    // Armijo γ
    double armijo_gamma{0.1};
    // 回溯衰减 β
    double backtrack_beta{0.5};
    // 回溯次数上限
    int max_backtracks{50};
    // merit μ₀
    double merit_mu0{10.0};
    // merit-AL 量级挂钩比率 c：保持缺陷修复与代价下降的交换比恒定（0=关闭）
    double merit_mu_al_ratio{0.0};
    // merit μ 上限
    double merit_mu_max{1e9};
    // 前向 rollout 定义域守卫：地图外扩 margin
    // 米，越界候选直接判失败回溯（0=关闭）
    double domain_guard_margin{2.0};
};
// 内层求解状态：失败语义（正则化溢出/迭代超限）经状态码上报外层，
// 供回退逻辑消费；仅输入契约违例抛异常
enum class MsIlqrStatus {
    CONVERGED_COST,          // 相对代价变化低于阈值
    CONVERGED_GRADIENT,      // 控制梯度范数低于阈值
    MAX_ITERATIONS,          // 达内层迭代上限，按未收敛上报
    REGULARIZATION_OVERFLOW  // ρ_reg 超上限，内层失败上报
};
// 单轮接受迭代的诊断记录：供收敛分析、单元测试对拍与外层日志使用
struct MsIlqrIterationRecord {
    // 迭代序号（从 1 起）
    int iteration{0};
    // 接受后的总代价 J / 缺陷 L2 范数 / merit 值 M = J + µ_m‖d‖
    double cost{0.0};
    // 缺陷 L2 范数
    double defect_norm{0.0};
    // merit 罚 μ_m
    double merit{0.0};
    // 本轮使用的 µ_m 与接受步长 α
    double merit_mu{0.0};
    // 线搜索步长
    double alpha{0.0};
    // 本轮被接受方向对应的 EC 系数（EC(α) = α·EC₁ + ½α²·EC₂）
    double ec1{0.0};
    // EC 二阶
    double ec2{0.0};
    // 本轮后向传递次数（正则化重试 >1 表明发生过 QP 失败或线搜索拒绝）
    int backward_passes{0};
    // 本轮线搜索的非线性 rollout 次数
    int line_search_trials{0};
};
// 内层求解结果汇总
struct MsIlqrResult {
    // 求解状态
    MsIlqrStatus status{MsIlqrStatus::MAX_ITERATIONS};
    // 实际执行的迭代轮数
    int iterations{0};
    // 求解前后的总代价与缺陷 L2 范数（诊断）
    double initial_cost{0.0};
    // 终态基础代价
    double final_cost{0.0};
    // 初始缺陷范数
    double initial_defect_norm{0.0};
    // 最终缺陷范数
    double final_defect_norm{0.0};
    // 定义域守卫（L8.3）本次求解拒绝的试探候选数（L8.4 分项归因：
    // 「出界已被拦住」与「出界不再发生」的区分证据）
    std::int64_t domain_guard_rejections{0};
};
// MS-iLQR 内层求解器（Gauss-Newton）：缺陷感知回推 + box-QP + 非线性 rollout +
// L₂ merit 线搜索
class MsIlqrSolver {
   public:
    // 构造时校验配置与控制盒/线搜索/正则化参数合法性
    MsIlqrSolver(MsIlqrConfig config, const BicycleDynamics* dynamics,
                 const DdpCostEvaluator* cost_evaluator);
    // µ_m 与 ρ_reg 跨 solve
    // 调用保持（外层轮次间自然热启动），计数器与历史每次清空
    // 输入契约：states/controls 尺寸必须为 N+1/N，dt 为正有限
    MsIlqrResult solve(const DdpReference& reference,
                       const DdpCostMultiplierState& multipliers,
                       const DdpCostInput& cost_input,
                       const DdpAlignedVec<DdpState>& initial_states,
                       const DdpAlignedVec<DdpControl>& initial_controls);
    // 最后一次接受迭代的名义状态序列（N+1 个）
    const DdpAlignedVec<DdpState>& states() const { return states_; }
    // 最后一次接受迭代的名义控制序列（N 个）
    const DdpAlignedVec<DdpControl>& controls() const { return controls_; }
    // 最后一次接受迭代的打靶缺陷序列（N+1 个）
    const DdpAlignedVec<DdpState>& defects() const { return defects_; }
    // 当前缺陷 L2 范数
    double defectNorm() const { return defect_norm_; }
    // 当前全轨迹总代价（增广后）
    double totalCost() const { return total_cost_; }
    // 当前 merit 罚 µ_m
    double meritMu() const { return merit_mu_; }
    // 地板抬升：µ_m = min(max(µ_m, floor), merit_mu_max)，棘轮只升不降
    void raiseMeritMuFloor(double floor) {
        if (!std::isfinite(floor)) {
            return;
        }
        merit_mu_ = std::min(std::max(merit_mu_, floor), config_.merit_mu_max);
    }
    // 当前 LM 正则化 ρ_reg
    double rhoReg() const { return rho_reg_; }
    // 本轮 solve 的逐接受迭代诊断历史（不含被拒绝迭代）
    const std::vector<MsIlqrIterationRecord>& history() const {
        return history_;
    }

   protected:
    void prepareWorkspace(std::size_t num_steps);
    void setShootingLookup(const std::vector<std::size_t>& shooting_nodes);
    void setNominalTrajectory(
        const DdpReference& reference,
        const DdpAlignedVec<DdpState>& initial_states,
        const DdpAlignedVec<DdpControl>& initial_controls);
    void evaluateNominal(const DdpReference& reference,
                         const DdpCostMultiplierState& multipliers,
                         const DdpCostInput& cost_input);
    void computeJacobians(const DdpReference& reference);
    void syncStepDt(const DdpReference& reference);
    // 缺陷感知 Riccati 回推：每步 box-QP 求 δũ +
    // K，钳制行恒为零。任一步失败返回 false
    bool backwardPass();
    // 线性 rollout（每轮一次）：缓存 EC₁/EC₂ 为成员，此后 EC(α) 闭式求值
    void linearRollout();
    // 预期代价变化闭式模型 EC(α) = α·EC₁ + ½α²·EC₂
    double expectedChange(double alpha) const {
        return alpha * ec1_ + 0.5 * alpha * alpha * ec2_;
    }
    // 收敛出口的可行性守卫：打靶缺陷 ‖d‖∞ 是否已降到容差内——
    // CONVERGED_COST/CONVERGED_GRADIENT 两个出口仅在守卫通过时允许触发
    bool convergenceAllowed() const;
    // 非线性 rollout（仅线搜索内逐候选 α 调用）：控制闭环更新
    double nonlinearRollout(double alpha, const DdpReference& reference,
                            const DdpCostMultiplierState& multipliers,
                            const DdpCostInput& cost_input);
    // merit 线搜索：α 自 1 起回溯，接受判据
    // M' <= M + γ·(EC(α) - α·µ_m‖d‖)；接受时经输出参数带回 α 与候选代价
    bool lineSearch(const DdpReference& reference,
                    const DdpCostMultiplierState& multipliers,
                    const DdpCostInput& cost_input, double merit_prev,
                    double* accepted_alpha, double* accepted_cost);
    // 接受候选轨迹：名义状态/控制/缺陷/代价整体替换（缺陷按 1-α 缩放，
    // 候选代价求值结果直接移入名义缓存，避免每轮重复全轨迹求值）
    void acceptCandidate(double alpha);
    // ρ_reg 增大（false=超上限）/ 接受后缩小
    bool increaseReg();
    void decreaseReg();
    static double DefectNorm(const DdpAlignedVec<DdpState>& defects);

   protected:
    // 配置
    MsIlqrConfig config_;
    // 动力学模型（不持有所有权）
    const BicycleDynamics* dynamics_;
    // 代价求值层（不持有所有权）
    const DdpCostEvaluator* cost_evaluator_;
    // 步数 N
    std::size_t num_steps_{0};
    // 逐步时长 dt[k]，未填充时退化为均匀网格
    std::vector<double> step_dt_;
    // 打靶标志（第 k 阶段是否为打靶节点）
    std::vector<bool> is_shooting_;
    // 名义状态序列（N+1 个，当前接受迭代点）
    DdpAlignedVec<DdpState> states_;
    // 名义控制序列（N 个）
    DdpAlignedVec<DdpControl> controls_;
    // 打靶缺陷序列（N+1 个）
    DdpAlignedVec<DdpState> defects_;
    // 候选状态序列（线搜索试探用）
    DdpAlignedVec<DdpState> cand_states_;
    // 候选控制序列（线搜索试探用）
    DdpAlignedVec<DdpControl> cand_controls_;
    // 候选代价
    double cand_cost_{0.0};
    // 候选缺陷范数
    double cand_defect_norm_{0.0};
    // 动力学雅可比 A = ∂f/∂x（N 个，逐步）
    DdpAlignedVec<DdpStateJacobian> jac_A_;
    // 动力学雅可比 B = ∂f/∂u（N 个，逐步）
    DdpAlignedVec<DdpControlJacobian> jac_B_;
    // 当前名义轨迹的全轨迹代价求值结果
    DdpCostEvaluation cost_eval_;
    // 候选轨迹的全轨迹代价求值结果
    DdpCostEvaluation cand_eval_;
    // Riccati 回推产出：前馈控制修正 δũ（N 个）
    DdpAlignedVec<DdpControl> feedforward_;
    // Riccati 回推产出：状态反馈增益 K（N 个）
    DdpAlignedVec<DdpControlStateHessian> gain_K_;
    // 回推中间量：代价对状态的梯度 Q_x（N 个）
    DdpAlignedVec<DdpState> q_x_;
    // 回推中间量：代价对控制的梯度 Q_u（N 个）
    DdpAlignedVec<DdpControl> q_u_;
    // 回推中间量：代价对控制的 Hessian Q_uu（N 个）
    DdpAlignedVec<DdpControlHessian> q_uu_;
    // 回推中间量：值函数 Hessian S（N+1 个）
    DdpAlignedVec<DdpStateHessian> value_S_;
    // 回推中间量：值函数梯度 s（N+1 个）
    DdpAlignedVec<DdpState> value_s_;
    // 钳制集（每步控制维活动集，供热启动跨 step 复用）
    std::vector<std::array<bool, DDP_CONTROL_DIM>> clamped_;
    // 钳制历史有效标志（步数不变时保留，N 变化时清空）
    bool has_clamped_history_{false};
    // 线性 rollout 产出：状态摄动序列 dx（N+1 个）
    DdpAlignedVec<DdpState> dx_lin_;
    // 线性 rollout 产出：控制摄动序列 du（N 个）
    DdpAlignedVec<DdpControl> du_lin_;
    // EC 一阶系数：EC(α)=α·EC₁+½α²·EC₂ 的线性项
    double ec1_{0.0};
    // EC 二阶系数
    double ec2_{0.0};
    // ΔV 一阶系数（二次展开的线性项）
    double dv1_{0.0};
    // ΔV 二阶系数（二次展开的 Hessian 项）
    double dv2_{0.0};
    // 最大 ‖Q_u‖∞（控制梯度收敛判据的驱动量）
    double max_qu_norm_{0.0};
    // 全轨迹总代价（增广后）
    double total_cost_{0.0};
    // 缺陷 L2 范数
    double defect_norm_{0.0};
    // 当前 merit μ_m（跨外层轮次保持）
    double merit_mu_{0.0};
    // 当前 LM 正则化 ρ_reg（跨外层轮次保持）
    double rho_reg_{0.0};
    // 盒约束 QP 求解器（无状态，逐步复用）
    BoxQpSolver<> qp_;
    // 计数器（每次 solve 清零，供单元测试断言双 rollout 调用次数与
    // 正则化变更后的全量重分解；统一 64 位，长时多次 solve 累积不溢出）
    std::int64_t backward_pass_count_{0};
    // 线性 rollout 计数
    std::int64_t linear_rollout_count_{0};
    // 非线性 rollout 计数
    std::int64_t nonlinear_rollout_count_{0};
    // QP 分解计数
    std::int64_t qp_factorization_count_{0};
    // 定义域守卫拒绝的试探候选数（每次 solve 清零）
    std::int64_t domain_guard_rejections_{0};
    // 逐轮诊断历史（仅接受迭代）
    std::vector<MsIlqrIterationRecord> history_;
};
}  // namespace apa_post_processor
