#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "bicycle_dynamics.h"
#include "box_qp.h"
#include "ilqr_cost.h"
#include "ilqr_reference_builder.h"

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
    // true=ALTRO 式虚拟控制增广（不可行状态轨迹初始化）：动力学
    // 改写为 x⁺=f(x,u)+w，初始 w 反解自初值轨迹使首轮 rollout 恰好
    // 复现初值、初始缺陷恒零；默认 false 保持打靶缺陷注入路径
    bool use_virtual_control{false};
    // 虚拟控制二次软代价权重 R_inf：½·R_inf·‖w‖²·dt 驱动 w→0
    double virtual_control_weight{1e4};
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
// 内层求解器编排接口：关闭/开启两个编译期实例化的公共抽象。
// 开关选择发生在编排层（纯调度、无浮点核心），虚调用不会改变任何
// 浮点密集函数的机器码；若把开关分支放进求解器内部的浮点函数，
// 分支会改变其编译布局与 FMA 收缩模式，导致默认关闭路径的输出
// 相对基线 ulp 漂移（多次二分实测证据）
class MsIlqrSolverInterface {
   public:
    virtual ~MsIlqrSolverInterface() = default;
    // 内层求解（参数契约见 MsIlqrSolverT::solve）
    virtual MsIlqrResult solve(
        const iLQRReference& reference,
        const iLQRCostMultiplierState& multipliers,
        const iLQRCostInput& cost_input,
        const iLQRAlignedVec<iLQRState>& initial_states,
        const iLQRAlignedVec<iLQRControl>& initial_controls,
        const iLQRAlignedVec<iLQRState>* initial_virtual_controls = nullptr) = 0;
    // 最后一次接受迭代的名义状态序列（N+1 个）
    virtual const iLQRAlignedVec<iLQRState>& states() const = 0;
    // 最后一次接受迭代的名义控制序列（N 个）
    virtual const iLQRAlignedVec<iLQRControl>& controls() const = 0;
    // 最后一次接受迭代的虚拟控制序列（N 个，关闭实例为空）
    virtual const iLQRAlignedVec<iLQRState>& virtualControls() const = 0;
    // 最后一次接受迭代的打靶缺陷序列（N+1 个）
    virtual const iLQRAlignedVec<iLQRState>& defects() const = 0;
    // 地板抬升：µ_m = min(max(µ_m, floor), merit_mu_max)，棘轮只升不降
    virtual void raiseMeritMuFloor(double floor) = 0;
    // 历次迭代审计记录（供编排层/测试诊断消费）
    virtual const std::vector<MsIlqrIterationRecord>& history() const = 0;
    // 正则化 ρ_reg 当前值（审计诊断消费）
    virtual double rhoReg() const = 0;
    // merit 罚 µ_m 当前值（审计诊断消费）
    virtual double meritMu() const = 0;
};
// MS-iLQR 内层求解器（Gauss-Newton）：缺陷感知回推 + box-QP + 非线性 rollout +
// L₂ merit 线搜索。UseVirtualControl=true 为 ALTRO 式虚拟控制增广实例；
// 全部开关分支用编译期 if constexpr，false 实例的机器码与基线逐字符
// 一致（运行时分支会改变浮点密集函数的编译布局，见接口类注释）
template <bool UseVirtualControl>
class MsIlqrSolverT : public MsIlqrSolverInterface {
   public:
    // 构造时校验配置与控制盒/线搜索/正则化参数合法性
    MsIlqrSolverT(MsIlqrConfig config, const BicycleDynamics* dynamics,
                  const iLQRCostEvaluator* cost_evaluator);
    // µ_m 与 ρ_reg 跨 solve
    // 调用保持（外层轮次间自然热启动），计数器与历史每次清空
    // 输入契约：states/controls 尺寸必须为 N+1/N，dt 为正有限；
    // initial_virtual_controls 仅开关开启时消费：非空直接热启动，
    // 空则反解自初值轨迹（ALTRO 不可行初始化，初始缺陷恒零）
    // 内层求解调度器：按开关零分支转发到关闭/开启两个独立实现。
    // 开关分支绝不允许出现在任何浮点密集函数内——真实分支会改变
    // 该函数的编译布局（栈帧/寄存器分配）与 FMA 收缩模式，导致
    // 默认关闭路径的输出相对基线 ulp 漂移（多次二分实测证据）
    MsIlqrResult solve(const iLQRReference& reference,
                       const iLQRCostMultiplierState& multipliers,
                       const iLQRCostInput& cost_input,
                       const iLQRAlignedVec<iLQRState>& initial_states,
                       const iLQRAlignedVec<iLQRControl>& initial_controls,
                       const iLQRAlignedVec<iLQRState>*
                           initial_virtual_controls = nullptr) override;
    // 最后一次接受迭代的名义状态序列（N+1 个）
    const iLQRAlignedVec<iLQRState>& states() const override { return states_; }
    // 最后一次接受迭代的名义控制序列（N 个）
    const iLQRAlignedVec<iLQRControl>& controls() const override { return controls_; }
    // 最后一次接受迭代的虚拟控制序列（N 个，关闭实例为空）
    const iLQRAlignedVec<iLQRState>& virtualControls() const override {
        return virtual_controls_;
    }
    // 最后一次接受迭代的打靶缺陷序列（N+1 个）
    const iLQRAlignedVec<iLQRState>& defects() const override { return defects_; }
    // 当前缺陷 L2 范数
    double defectNorm() const { return defect_norm_; }
    // 当前全轨迹总代价（增广后）
    double totalCost() const { return total_cost_; }
    // 当前 merit 罚 µ_m
    double meritMu() const override { return merit_mu_; }
    // 地板抬升：µ_m = min(max(µ_m, floor), merit_mu_max)，棘轮只升不降
    void raiseMeritMuFloor(double floor) override {
        if (!std::isfinite(floor)) {
            return;
        }
        merit_mu_ = std::min(std::max(merit_mu_, floor), config_.merit_mu_max);
    }
    // 当前 LM 正则化 ρ_reg
    double rhoReg() const override { return rho_reg_; }
    // 本轮 solve 的逐接受迭代诊断历史（不含被拒绝迭代）
    const std::vector<MsIlqrIterationRecord>& history() const override {
        return history_;
    }

   protected:
    void prepareWorkspace(std::size_t num_steps);
    void setShootingLookup(const std::vector<std::size_t>& shooting_nodes);
    void setNominalTrajectory(
        const iLQRReference& reference,
        const iLQRAlignedVec<iLQRState>& initial_states,
        const iLQRAlignedVec<iLQRControl>& initial_controls);
    void evaluateNominal(const iLQRReference& reference,
                         const iLQRCostMultiplierState& multipliers,
                         const iLQRCostInput& cost_input);
    void computeJacobians(const iLQRReference& reference);
    void syncStepDt(const iLQRReference& reference);
    // 缺陷感知 Riccati 回推：每步 box-QP 求 δũ +
    // K，钳制行恒为零。任一步失败返回 false。本函数即关闭虚拟控制
    // 的生产默认实现（与基线逐字符一致）：增广相关分支若混入浮点
    // 密集函数会改变其编译布局与 FMA 收缩模式，导致输出 ulp 漂移，
    // 因此开启路径走独立的 backwardPassWithVirtualControl
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
    // 非线性 rollout（仅线搜索内逐候选 α 调用）：控制闭环更新 +
    // 候选代价求值；merit_reject_threshold 为 Armijo 接受阈值
    // （merit_prev + γ·EC(α)），求值层据此做 ESDF 早停筛选——
    // 廉价小计超阈即跳过 ESDF，拒绝决策与全量求值逐位一致
    double nonlinearRollout(double alpha, const iLQRReference& reference,
                            const iLQRCostMultiplierState& multipliers,
                            const iLQRCostInput& cost_input,
                            double merit_reject_threshold);
    // merit 线搜索：α 自 1 起回溯，接受判据
    // M' <= M + γ·(EC(α) - α·µ_m‖d‖)；接受时经输出参数带回 α 与候选代价
    bool lineSearch(const iLQRReference& reference,
                    const iLQRCostMultiplierState& multipliers,
                    const iLQRCostInput& cost_input, double merit_prev,
                    double* accepted_alpha, double* accepted_cost);
    // 接受候选轨迹：名义状态/控制/缺陷/代价整体替换（缺陷按 1-α 缩放，
    // 候选代价求值结果直接移入名义缓存，避免每轮重复全轨迹求值）
    void acceptCandidate(double alpha);
    // ρ_reg 增大（false=超上限）/ 接受后缩小
    bool increaseReg();
    void decreaseReg();
    static double DefectNorm(const iLQRAlignedVec<iLQRState>& defects);
    // 结构化价值回传内核：out = Aᵀ·S·A。A 的非零结构由
    // bicycle_dynamics.cpp 的 jacobians() 解析式决定（对角恒 1、
    // 行 0~2 几何项、行 3 列 4 = dt、行 5 列 6 = dt、其余恒零），
    // 内核按该结构编译期展开列/行组合（连续内存可向量化），跳过
    // 全部零项贡献；与 Eigen 稠密乘积逐元素差在 1e-13 量级（求值
    // 结合顺序不同，人工裁决重新基线后启用）
    static iLQRStateHessian UpdateValueHessian(const iLQRStateHessian& s,
                                              const iLQRStateJacobian& a);
    // 虚拟控制反解：w_k = x_{k+1} − f(x_k, u_k)，使首轮 rollout 恰好
    // 复现初值轨迹（ALTRO 不可行初始化的 w⁰ 构造）
    iLQRAlignedVec<iLQRState> computeVirtualControls(
        const iLQRReference& reference,
        const iLQRAlignedVec<iLQRState>& initial_states,
        const iLQRAlignedVec<iLQRControl>& initial_controls) const;

   protected:
    // 配置
    MsIlqrConfig config_;
    // 动力学模型（不持有所有权）
    const BicycleDynamics* dynamics_;
    // 代价求值层（不持有所有权）
    const iLQRCostEvaluator* cost_evaluator_;
    // 步数 N
    std::size_t num_steps_{0};
    // 逐步时长 dt[k]，未填充时退化为均匀网格
    std::vector<double> step_dt_;
    // 打靶标志（第 k 阶段是否为打靶节点）
    std::vector<bool> is_shooting_;
    // 名义状态序列（N+1 个，当前接受迭代点）
    iLQRAlignedVec<iLQRState> states_;
    // 名义控制序列（N 个）
    iLQRAlignedVec<iLQRControl> controls_;
    // 名义虚拟控制序列（N 个，开关关闭时为空）
    iLQRAlignedVec<iLQRState> virtual_controls_;
    // 候选虚拟控制序列（线搜索试探用）
    iLQRAlignedVec<iLQRState> cand_virtual_controls_;
    // 打靶缺陷序列（N+1 个）
    iLQRAlignedVec<iLQRState> defects_;
    // 候选状态序列（线搜索试探用）
    iLQRAlignedVec<iLQRState> cand_states_;
    // 候选控制序列（线搜索试探用）
    iLQRAlignedVec<iLQRControl> cand_controls_;
    // 候选代价
    double cand_cost_{0.0};
    // 候选缺陷范数
    double cand_defect_norm_{0.0};
    // 动力学雅可比 A = ∂f/∂x（N 个，逐步）
    iLQRAlignedVec<iLQRStateJacobian> jac_A_;
    // 动力学雅可比 B = ∂f/∂u（N 个，逐步）
    iLQRAlignedVec<iLQRControlJacobian> jac_B_;
    // 当前名义轨迹的全轨迹代价求值结果
    iLQRCostEvaluation cost_eval_;
    // 候选轨迹的全轨迹代价求值结果
    iLQRCostEvaluation cand_eval_;
    // Riccati 回推产出：前馈控制修正 δũ（N 个）
    iLQRAlignedVec<iLQRControl> feedforward_;
    // Riccati 回推产出：状态反馈增益 K（N 个）
    iLQRAlignedVec<iLQRControlStateHessian> gain_K_;
    // Riccati 回推产出：虚拟控制前馈修正（N 个，开关关闭时为空）
    iLQRAlignedVec<iLQRState> virtual_feedforward_;
    // Riccati 回推产出：虚拟控制状态反馈增益 K_w（N 个）
    iLQRAlignedVec<iLQRStateHessian> virtual_gain_;
    // 线性 rollout 产出：虚拟控制摄动序列 dw（N 个）
    iLQRAlignedVec<iLQRState> dw_lin_;
    // 回推中间量：代价对状态的梯度 Q_x（N 个）
    iLQRAlignedVec<iLQRState> q_x_;
    // 回推中间量：代价对控制的梯度 Q_u（N 个）
    iLQRAlignedVec<iLQRControl> q_u_;
    // 回推中间量：代价对控制的 Hessian Q_uu（N 个）
    iLQRAlignedVec<iLQRControlHessian> q_uu_;
    // 回推中间量：值函数 Hessian S（N+1 个）
    iLQRAlignedVec<iLQRStateHessian> value_S_;
    // 回推中间量：值函数梯度 s（N+1 个）
    iLQRAlignedVec<iLQRState> value_s_;
    // 钳制集（每步控制维活动集，供热启动跨 step 复用）
    std::vector<std::array<bool, ILQR_CONTROL_DIM>> clamped_;
    // 钳制历史有效标志（步数不变时保留，N 变化时清空）
    bool has_clamped_history_{false};
    // 线性 rollout 产出：状态摄动序列 dx（N+1 个）
    iLQRAlignedVec<iLQRState> dx_lin_;
    // 线性 rollout 产出：控制摄动序列 du（N 个）
    iLQRAlignedVec<iLQRControl> du_lin_;
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
// 生产默认实例：虚拟控制关闭（与基线机器码逐字符一致）
using MsIlqrSolver = MsIlqrSolverT<false>;
// ALTRO 式虚拟控制增广实例（实验/对照用）
using MsIlqrSolverVirtualControl = MsIlqrSolverT<true>;
}  // namespace apa_post_processor
