#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "bicycle_dynamics.h"
#include "box_qp.h"
#include "ddp_cost.h"
#include "ddp_reference_builder.h"

namespace apa_post_processor {
// MS-iLQR 内层求解器配置：控制盒、迭代/收敛、LM 正则化、线搜索与 merit
// 自适应罚参数。默认值取 DDP.md 2.5 节参数表与 Unified MS-DDP 论文/官方
// 实现的建议值，最终标定属端到端调参
struct MsIlqrConfig {
    // 纵向跃度盒约束幅值 |j| <= jerk_max (m/s³)
    double jerk_max{1.5};
    // 前轮转角加加速度盒约束幅值 |η| <= steer_accel_max (rad/s²)
    double steer_accel_max{1.0};
    // 内层迭代上限（超限按未收敛上报外层）
    int max_iterations{50};
    // 收敛判据一：相对代价变化 |ΔJ|/|J| 阈值
    double cost_change_tol{1e-6};
    // 收敛判据二：控制梯度范数 max_k ‖Q_u‖∞ 阈值（无约束驻点信号；
    // 盒约束激活处 Q_u 不必为零，该判据自然不会误触发）
    double gradient_tol{1e-8};
    // LM 正则化初值/下界/上界：ρ_reg 超上限判定本轮内层失败并上报
    double reg_initial{1e-4};
    double reg_min{1e-9};
    double reg_max{1e9};
    // 拒绝后增大倍率 / 接受后缩小倍率（Levenberg-Marquardt 启发式）
    double reg_increase{10.0};
    double reg_decrease{0.5};
    // Armijo 充分下降系数 γ 与回溯衰减 β
    double armijo_gamma{0.1};
    double backtrack_beta{0.5};
    // 回溯次数上限（α 下界 = β^max_backtracks）
    int max_backtracks{50};
    // merit 罚参数 µ_m 的安全余量 µ₀：自适应阈值 = µ₀+|EC(1)|/((1-ρ)‖d‖)，
    // 且 µ_m 恒不小于 µ₀；µ_m 与 AL 外层的 µ 是完全独立的两套参数
    double merit_mu0{10.0};
    // 自适应规则的安全因子 ρ（分母 1-ρ 提供余量）
    double merit_rho{0.5};
    // µ_m 更新的缺陷范数门限 κ_d：仅当 ‖d‖₂ > κ_d 时刷新权重，
    // 避免缺陷接近归零后权重被无意义地刷新
    double merit_kappa_d{1e-6};
    // 可选段间惩罚权重 w_d（Q_d = w_d·I₇，默认 0 = 关闭）：回推经过打靶
    // 节点时注入 s[i] -= Q_d·d[i]、S[i] += Q_d，促使段从左侧也向缺陷靠拢
    double inter_segment_weight{0.0};
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
    double defect_norm{0.0};
    double merit{0.0};
    // 本轮使用的 µ_m 与接受步长 α
    double merit_mu{0.0};
    double alpha{0.0};
    // 本轮被接受方向对应的 EC 系数（EC(α) = α·EC₁ + ½α²·EC₂）
    double ec1{0.0};
    double ec2{0.0};
    // 本轮后向传递次数（正则化重试 >1 表明发生过 QP 失败或线搜索拒绝）
    int backward_passes{0};
    // 本轮线搜索的非线性 rollout 次数
    int line_search_trials{0};
};
// 内层求解结果汇总
struct MsIlqrResult {
    MsIlqrStatus status{MsIlqrStatus::MAX_ITERATIONS};
    // 实际执行的迭代轮数
    int iterations{0};
    // 求解前后的总代价与缺陷 L2 范数（诊断）
    double initial_cost{0.0};
    double final_cost{0.0};
    double initial_defect_norm{0.0};
    double final_defect_norm{0.0};
};
// MS-iLQR 内层求解器（Gauss-Newton，丢弃动力学二阶张量项）：
// 缺陷感知 Riccati 回推（右端索引约定 d[i] ≡ f(x[i-1],u[i-1]) - x[i]，
// i=1..N，d[0]=0）+ 每步盒约束 QP（活动集沿回推顺序热启动）+
// 非线性 rollout 与 L₂ merit 线搜索。索引约定：状态/缺陷数组长 N+1，
// 控制/雅可比/增益数组长 N；缺陷仅在打靶节点非零（滚动状态恒被动力学
// 覆写，其缺陷恒为零）。µ_m 与 AL 罚权重 µ 完全独立（独立变量、独立
// 更新逻辑）。全部轨迹/矩阵按 SoA 以对齐分配器容器存储，Workspace 在
// 首次 solve 时按 N 预分配，回推/滚动热循环内零堆分配。
class MsIlqrSolver {
   public:
    // 构造校验：动力学/代价求值层必须非空，盒边界/容差/正则化上下界/
    // 线搜索与 merit 参数必须为正且自洽，非法输入抛 std::invalid_argument
    MsIlqrSolver(MsIlqrConfig config, const BicycleDynamics* dynamics,
                 const DdpCostEvaluator* cost_evaluator);
    // 求解一次内层问题（固定 AL 乘子）：以初值序列建立名义轨迹
    // （打靶状态直接注入、滚动状态由动力学覆写），迭代至收敛/上限/失败。
    // initial_states 尺寸必须为 N+1、initial_controls 为 N（N = 参考位姿
    // 数 - 1）；维度/取值不符抛 std::invalid_argument。µ_m 与 ρ_reg 跨
    // solve 调用保持（外层 AL 轮次间自然热启动），计数器与历史每次清空
    MsIlqrResult solve(const DdpReference& reference,
                       const DdpCostMultiplierState& multipliers,
                       const DdpCostInput& cost_input,
                       const DdpAlignedVec<DdpState>& initial_states,
                       const DdpAlignedVec<DdpControl>& initial_controls);
    // 当前名义轨迹（尺寸 N+1 / N）
    const DdpAlignedVec<DdpState>& states() const { return states_; }
    const DdpAlignedVec<DdpControl>& controls() const { return controls_; }
    // 当前缺陷序列 d[i]（尺寸 N+1，d[0]=0，仅打靶节点非零）
    const DdpAlignedVec<DdpState>& defects() const { return defects_; }
    // 缺陷 L2 范数（聚合向量意义下的 2-范数）
    double defectNorm() const { return defect_norm_; }
    // 当前名义总代价 J
    double totalCost() const { return total_cost_; }
    // 当前 merit 罚参数 µ_m（与 AL 的 µ 无关）
    double meritMu() const { return merit_mu_; }
    // 当前 LM 正则化 ρ_reg
    double rhoReg() const { return rho_reg_; }
    // 逐轮诊断历史（仅记录被接受的迭代）
    const std::vector<MsIlqrIterationRecord>& history() const {
        return history_;
    }

   protected:
    // 按 N 预分配全部工作区（N 变化时重建；热循环外的准备动作）
    void prepareWorkspace(std::size_t num_steps);
    // 打靶节点集 -> 逐节点 bool 查表；节点索引越界抛 std::invalid_argument
    void setShootingLookup(const std::vector<std::size_t>& shooting_nodes);
    // 建立初始名义轨迹：滚动状态由动力学覆写、打靶状态直接注入初值，
    // 同步计算初始缺陷（非打靶节点恒为零）与缺陷范数
    void setNominalTrajectory(
        const DdpReference& reference,
        const DdpAlignedVec<DdpState>& initial_states,
        const DdpAlignedVec<DdpControl>& initial_controls);
    // 名义代价与 GN 导数求值（含 AL 增广项，由代价求值层统一折叠进
    // 各阶段导数，内层不区分基础代价与 AL 项）
    void evaluateNominal(const DdpReference& reference,
                         const DdpCostMultiplierState& multipliers,
                         const DdpCostInput& cost_input);
    // 逐阶段解析雅可比 A_k/B_k
    void computeJacobians(const DdpReference& reference);
    // 缺陷感知 Riccati 回推：标准 Q 量装配 + q_x/q_u 的 S'd 修正
    // （ẑ = s' + S'·d[k+1]，Q_x = ℓ_x + Aᵀẑ，Q_u = ℓ_u + Bᵀẑ），每步
    // box-QP 求 δũ 与自由子空间增益 K（钳制行恒为零），随后按 K/δũ 形式
    // 回传 S/s。可选段间惩罚在经过打靶节点时先注入 s -= Q_d·d、S += Q_d。
    // 任一步 QP 未收敛返回 false（调用方增大 ρ_reg 后重跑整个回推，
    // 不得复用旧分解；活动集经成员缓存热启动但分解必然重做）
    bool backwardPass();
    // 线性 rollout（每轮后向传递后恰好一次）：沿 A_k/B_k 传播 α=1 的
    // 搜索方向 δx^l/δu^l（含打靶节点缺陷项），并缓存 EC₁/EC₂ 为成员，
    // 此后整条 Armijo 回溯链上 EC(α) 均为闭式求值，严禁重复线性传播
    void linearRollout();
    // 预期代价变化闭式模型 EC(α) = α·EC₁ + ½α²·EC₂
    double expectedChange(double alpha) const {
        return alpha * ec1_ + 0.5 * alpha * alpha * ec2_;
    }
    // 自适应 µ_m 更新：仅当 ‖d‖₂ > κ_d 时按 µ₀+|EC(1)|/((1-ρ)‖d‖) 抬升
    // （取绝对值与只升不降的棘轮跟随官方实现，Nocedal Chp18.3 的动机；
    // 与 AL 的 µ 更新无任何共享变量/逻辑）
    void updateMeritMu();
    // 非线性 rollout（仅线搜索内逐候选 α 调用）：控制闭环更新
    // u' = ū + α·δũ + K(x' - x̄)，段内真实动力学积分，打靶节点状态按
    // x' = x_int + (α-1)·d̄ 收缩缺陷（α=1 时缺陷精确归零），新缺陷
    // d' = (1-α)·d̄ 精确成立。返回候选轨迹总代价
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
    // ρ_reg 增大（返回 false 表示已超上限，内层失败）/ 接受后缩小
    bool increaseReg();
    void decreaseReg();
    // 缺陷 L2 范数：sqrt(Σ_i ‖d[i]‖²)
    static double DefectNorm(const DdpAlignedVec<DdpState>& defects);

   protected:
    MsIlqrConfig config_;
    // 动力学与代价求值层（不持有所有权，必须非空）
    const BicycleDynamics* dynamics_;
    const DdpCostEvaluator* cost_evaluator_;
    // 步数 N 与固定步长 dt
    std::size_t num_steps_{0};
    double dt_{0.0};
    // 逐节点打靶标志（尺寸 N+1）
    std::vector<bool> is_shooting_;
    // 名义轨迹与缺陷（SoA，尺寸 N+1 / N / N+1）
    DdpAlignedVec<DdpState> states_;
    DdpAlignedVec<DdpControl> controls_;
    DdpAlignedVec<DdpState> defects_;
    // 候选轨迹（线搜索试用，尺寸 N+1 / N）及其总代价/缺陷范数缓存
    DdpAlignedVec<DdpState> cand_states_;
    DdpAlignedVec<DdpControl> cand_controls_;
    double cand_cost_{0.0};
    double cand_defect_norm_{0.0};
    // 逐阶段雅可比（尺寸 N）
    DdpAlignedVec<DdpStateJacobian> jac_A_;
    DdpAlignedVec<DdpControlJacobian> jac_B_;
    // 名义轨迹代价分解与 GN 导数（stages 尺寸 N+1）
    DdpCostEvaluation cost_eval_;
    // 候选轨迹代价求值结果（接受后移入 cost_eval_）
    DdpCostEvaluation cand_eval_;
    // 回推产物：前馈 δũ、反馈增益 K（钳制行恒为零）、逐阶段 Q 量
    // （供价值回传与白盒对拍），尺寸均为 N
    DdpAlignedVec<DdpControl> feedforward_;
    DdpAlignedVec<DdpControlStateHessian> gain_K_;
    DdpAlignedVec<DdpState> q_x_;
    DdpAlignedVec<DdpControl> q_u_;
    DdpAlignedVec<DdpControlHessian> q_uu_;
    // 逐节点价值梯度/Hessian 与逐步实际使用的下游值（段间惩罚注入后），
    // 尺寸 N+1 / N / N，供白盒对拍缺陷修正与 Q_d 注入位置
    DdpAlignedVec<DdpStateHessian> value_S_;
    DdpAlignedVec<DdpState> value_s_;
    DdpAlignedVec<DdpStateHessian> down_S_;
    DdpAlignedVec<DdpState> down_s_;
    // 逐步钳制集缓存（跨回推热启动；分解不跨次复用）与历史有效标志
    std::vector<std::array<bool, DDP_CONTROL_DIM>> clamped_;
    bool has_clamped_history_{false};
    // 线性 rollout 缓存的方向（尺寸 N+1 / N）
    DdpAlignedVec<DdpState> dx_lin_;
    DdpAlignedVec<DdpControl> du_lin_;
    // 本轮 EC 闭式系数缓存
    double ec1_{0.0};
    double ec2_{0.0};
    // 价值回传累积的 ΔV 一阶/二阶系数（诊断）
    double dv1_{0.0};
    double dv2_{0.0};
    // 本轮回推的最大控制梯度范数 max_k ‖Q_u‖∞（收敛判据二）
    double max_qu_norm_{0.0};
    // 当前状态量：总代价、缺陷范数、µ_m、ρ_reg
    double total_cost_{0.0};
    double defect_norm_{0.0};
    double merit_mu_{0.0};
    double rho_reg_{0.0};
    // 盒约束 QP 求解器（无状态，逐步复用）
    BoxQpSolver<> qp_;
    // 计数器（每次 solve 清零，供单元测试断言双 rollout 调用次数与
    // 正则化变更后的全量重分解；统一 64 位，长时多次 solve 累积不溢出）
    std::int64_t backward_pass_count_{0};
    std::int64_t linear_rollout_count_{0};
    std::int64_t nonlinear_rollout_count_{0};
    std::int64_t qp_factorization_count_{0};
    // 逐轮诊断历史（仅接受迭代）
    std::vector<MsIlqrIterationRecord> history_;
};
}  // namespace apa_post_processor
