#pragma once

#include <Eigen/Core>
#include <cstddef>
#include <vector>

#include "bicycle_dynamics.h"
#include "ddp_reference_builder.h"
#include "esdf_constraint.h"

namespace apa_post_processor {
// 控制二阶导数（2×2）与控制-状态交叉导数（2×7）定长类型，栈上分配
using DdpControlHessian =
    Eigen::Matrix<double, DDP_CONTROL_DIM, DDP_CONTROL_DIM>;
using DdpControlStateHessian =
    Eigen::Matrix<double, DDP_CONTROL_DIM, DDP_STATE_DIM>;
// 状态幅值 AL 不等式约束：每运行阶段 5 个（全部 ≤0 形式），布局如下
inline constexpr int DDP_AMPLITUDE_CONSTRAINT_DIM = 5;
// v² − v_max² ≤ 0：平方形态对符号中性、在 v=0 处光滑，保住 v 过零自由
// （融化前提），严禁改写成盒约束（会直接禁掉倒车/钉死过零）
inline constexpr int DDP_AMP_V = 0;
// a² − a_max² ≤ 0
inline constexpr int DDP_AMP_A = 1;
// ω² − ω_max² ≤ 0
inline constexpr int DDP_AMP_OMEGA = 2;
// δ − δ_max ≤ 0（线性形态）
inline constexpr int DDP_AMP_DELTA_POS = 3;
// −δ − δ_max ≤ 0（线性形态）
inline constexpr int DDP_AMP_DELTA_NEG = 4;
// 终点 AL 等式约束：5 个，布局 [x, y, θ(wrap), v, a]；
// δ_N/ω_N 不约束（停稳后前轮转角无物理要求，留给下游回正逻辑）
inline constexpr int DDP_TERMINAL_CONSTRAINT_DIM = 5;
// 阶段二门控约束计划：由后处理模块按修剪后 maneuver 序列生成，代价求值层
// 仅消费。三类门控全部只作用于运行阶段（0..N-1）；终端节点由终点等式
// 接管，其符号门/驻留帽条目不产生任何贡献。严禁在阶段一施加——它们本质
// 上是把结构性换挡边界请回局部，提前施加会立即摧毁融化机制
struct DdpGatingPlan {
    // 逐节点符号门（尺寸 N+1）：+1/-1 = 该节点施加 −s_m·v ≤ 0（段内方向
    // 保持），0 = 无门（接缝点由零速等式接管，避免边界归属歧义）
    std::vector<int> sign_gate;
    // 接缝零速等式 v=0 作用的节点索引（升序无重复）
    std::vector<std::size_t> seam_indices;
    // 逐节点接缝查表（尺寸 N+1）：接缝序号（0..S-1）或 -1（非接缝）；
    // 求值热路径据此 O(1) 判定当前阶段是否为接缝，禁止逐步二分查找
    std::vector<int> seam_lookup;
    // 逐节点驻留速度帽（尺寸 N+1）：>0 = 该节点施加 |v|≤cap（以
    // g=|v|−cap 形式施加：活动区 |v|>cap 处处光滑且梯度模恒为 1，
    // 对符号中性；纽结 v=0 位于非活动区内部，不产生数值问题）
    std::vector<double> dwell_v_cap;
};
// 阶段一代价/约束求值层配置：代价权重与幅值边界（默认值取 DDP.md 2.5 节
// 参数表建议初值，最终标定属端到端调参）
struct DdpCostConfig {
    // 平滑主项权重：纵向跃度 j（基准权重，其余权重相对它标定）
    double weight_jerk{1.0};
    // 平滑主项权重：前轮转角加加速度 η
    double weight_steer_accel{1.0};
    // 跟踪项位置权重基准值 w_ref,0：首轮权重与豁免退火点的恒定权重
    double weight_ref_base{10.0};
    // 跟踪项航向权重 w_θ
    double weight_theta{5.0};
    // 可选换挡代理权重 w_g：默认 0 = 关闭（退火兜底手段，启用需显式记录）
    double weight_shift{0.0};
    // 换挡代理 σ_β 门宽 (m/s)：越小门越陡，随外层退火收窄
    double shift_beta{0.1};
    // 曲率正则项权重 w_κ（默认 0 = 关闭）：ℓ_κ = w_κ·(tanδ/L)²（积分型
    // 代价，含 dt 因子）。平滑项（w_j·j² + w_η·η²）对 δ 量级零梯度——
    // 「满打方向盘匀速转」与「大半径缓转」在平滑项下完全等价，退火后
    // 目标函数中不存在任何抑制大曲率的力，解贴边界是最优解而非病态；
    // 本项的梯度 2w·tanδ·sec²δ/L² 以三阶极点发散（严格快于动力学项
    // tanδ/L 的二阶），构成 δ 奇异区的真正内屏障；其精确 Hessian
    // (2w/L²)(sec⁴δ + 2tan²δ·sec²δ) 处处为正，天然 PSD 无需投影
    double weight_curvature{0.0};
    // 弧长惩罚权重 w_v（默认 0 = 关闭）：ℓ_v = w_v·v²（积分型代价、
    // 含 dt 因子）。固定 dt 网格下总时长是常数，∫v²dt 是弧长的同向凸
    // 代理——平滑项对「a 恒定 + ω 恒定」恰为零，跟踪退火后目标函数中
    // 不存在任何反对轨迹跑飞（绕大圈）的项；本项 Hessian 2w_v>0 天然
    // PSD，与 ESDF 舒适罚（作用于离障距离）和已否决的第 8 状态方案
    // （状态维 7→8）均不同源
    double weight_velocity{0.0};
    // 轴距 (m)：曲率正则的 κ=tanδ/L 系数来源。默认 0 = 未注入——由
    // DdpConfig::clampToVehicleParams 按 VehicleParams 注入；weight_curvature
    // 非零而轴距未注入时求值层构造显式拒绝（防 0 除/语义漂移）
    double wheelbase{0.0};
    // δ 奇异区护栏权重 w_δg（默认 0 = 关闭）：对 |δ|>δ_guard 的区域施加
    // 光滑铰链罚 w·max(0,|δ|−δ_guard)²。换挡区 v≈0 时 tanδ 允许经 ±π/2
    // 奇异区「原地免费转头」——AL 幅值不等式在 g<0 且 λ=0 时零梯度（对
    // 该逃逸通道完全不可见），λ 累积的内屏障要到越界发生之后才建立；
    // 铰链罚从首轮内层第 0 次迭代起就提供持续的回推梯度。δ_guard 取在
    // δ_max 与 π/2 之间（0.7 rad）：健康解（|δ|≤δ_max+平衡容差）永不
    // 激活，行为逐位不变
    double weight_delta_guard{0.0};
    // δ 奇异区护栏的激活阈值 (rad)：必须大于 δ_max 且小于 π/2
    double delta_guard{0.7};
    // 纵向速度幅值上限 (m/s)（平方形态约束边界）
    double v_max{1.5};
    // 纵向加速度幅值上限 (m/s²)
    double a_max{1.0};
    // 前轮转角速度幅值上限 (rad/s)
    double omega_max{0.5};
    // 前轮转角幅值上限 (rad)
    double delta_max{0.55};
};
// AL 乘子/罚权重状态：由外层 AL 循环维护与更新，本层只消费不修改
struct DdpCostMultiplierState {
    // 创建全零乘子状态（首轮外层迭代：λ=0、μ 由调用方按自适应标定填入）；
    // 门控乘子向量为空 = 阶段一模式（门控项恒不生效）
    static DdpCostMultiplierState MakeZero(std::size_t num_steps);
    // 创建阶段二乘子状态：幅值/终点全零同 MakeZero，门控乘子按网格/接缝
    // 规模分配并清零（λ=0、μ=0，由阶段二编排填入初始罚权重）
    static DdpCostMultiplierState MakeStageTwoZero(std::size_t num_steps,
                                                   std::size_t num_seams);
    // 幅值不等式乘子 λ ≥ 0（尺寸 = 5·N，阶段 k 的 5 个约束位于 [5k, 5k+5)）
    Eigen::VectorXd amplitude_lambda;
    // 幅值不等式罚权重 μ > 0（尺寸 = 5·N，布局同上）
    Eigen::VectorXd amplitude_mu;
    // 终点等式乘子 λ（5 维，符号不限）
    Eigen::Matrix<double, DDP_TERMINAL_CONSTRAINT_DIM, 1> terminal_lambda;
    // 终点等式罚权重 μ > 0（5 维）
    Eigen::Matrix<double, DDP_TERMINAL_CONSTRAINT_DIM, 1> terminal_mu;
    // 以下六个门控乘子向量仅阶段二使用（默认空 = 阶段一模式）：
    // 符号门不等式乘子 λ ≥ 0 与罚权重 μ > 0（尺寸 N+1）
    Eigen::VectorXd gating_sign_lambda;
    Eigen::VectorXd gating_sign_mu;
    // 接缝零速等式乘子 λ（符号不限）与罚权重 μ > 0（尺寸 = 接缝数）
    Eigen::VectorXd gating_seam_lambda;
    Eigen::VectorXd gating_seam_mu;
    // 驻留帽不等式乘子 λ ≥ 0 与罚权重 μ > 0（尺寸 N+1）
    Eigen::VectorXd gating_dwell_lambda;
    Eigen::VectorXd gating_dwell_mu;
};
// 单次全轨迹求值的外部输入：退火权重与豁免掩码由外层调度器供给
struct DdpCostInput {
    // 当前退火后的跟踪权重 w_ref(r)（掩码豁免点恒用 w_ref,0）
    double tracking_weight{0.0};
    // 豁免退火的按点掩码（true = 该点恒用 w_ref,0）；nullptr = 无豁免点
    const std::vector<bool>* anneal_exempt_mask{nullptr};
    // 换挡代理 σ_β 的逐轮门宽 (m/s)：>0 时覆盖 DdpCostConfig::shift_beta
    // 静态值（外层 β 退火调度的落点）；=0 回退配置静态值（默认行为）。
    // 换挡代理只在阶段一生效——门控计划在场（阶段二）时代理整体关闭：
    // 阶段二的换挡位置已被符号门/接缝零速等式钉死，代理继续惩罚保留的
    // 换挡只会与门控等式对拉
    double shift_beta{0.0};
    // 候选待融段的按点掩码（true = 该点跟踪权重取 candidate_tracking_weight
    // 而非 tracking_weight）：由外层按融化平衡式临界比生成（低临界比内部
    // maneuver），深退火把「是否值得保留」的裁决权交还平滑项；权重选择
    // 优先级：豁免掩码（w_ref,0）> 候选掩码 > 普通退火权重。
    // nullptr = 无候选段（全部按普通退火）
    const std::vector<bool>* melt_candidate_mask{nullptr};
    // 候选段的逐轮跟踪权重（仅掩码在场时消费）
    double candidate_tracking_weight{0.0};
    // 阶段二门控计划（nullptr = 阶段一模式，门控项恒零）；
    // 与乘子门控向量必须同在场（见 evaluate 的一致性校验）
    const DdpGatingPlan* gating_plan{nullptr};
};
// 单阶段代价分解值与 Gauss-Newton 导数（供内层回推直接消费）。
// dt 因子约定：平滑/跟踪项（积分型代价）的代价与导数已含 dt 因子，
// ESDF 惩罚/幅值 AL/终点 AL 为逐阶段点态量、不乘 dt——回推累积时
// 须区分处理，不得对所有项统一乘 dt（否则 AL 约束被 dt 倍稀释，
// 约束渐硬机制失效、融化行为退化）
struct DdpStageCostDerivatives {
    // 平滑主项 ½(w_j·j² + w_η·η²)·dt
    double cost_smooth{0.0};
    // 退火跟踪项 ½w_ref,k·‖p−p_ref‖²·dt + ½w_θ·wrap(θ−θ_ref)²·dt
    double cost_tracking{0.0};
    // 可选换挡代理项（默认关闭）
    double cost_shift{0.0};
    // 状态幅值 AL 增广项（激活约束的 λg + ½μg²）
    double cost_amplitude{0.0};
    // δ 奇异区护栏惩罚（光滑铰链，默认关闭时恒为零）
    double cost_delta_guard{0.0};
    // 曲率正则项（默认关闭时恒为零；积分型代价、含 dt 因子）
    double cost_curvature{0.0};
    // 弧长惩罚项（默认关闭时恒为零；积分型代价、含 dt 因子）
    double cost_velocity{0.0};
    // ESDF 双 margin 惩罚（仅抽样阶段非零）
    double cost_esdf{0.0};
    // 终点 AL 等式增广项（仅末阶段非零）
    double cost_terminal{0.0};
    // 阶段二门控增广项（符号门/接缝等式/驻留帽；阶段一恒为零）
    double cost_gating{0.0};
    // 该阶段总代价
    double totalCost() const {
        return cost_smooth + cost_tracking + cost_shift + cost_amplitude +
               cost_delta_guard + cost_curvature + cost_velocity + cost_esdf +
               cost_terminal + cost_gating;
    }
    // GN 导数 ℓ_x（7 维）
    DdpState lx{DdpState::Zero()};
    // GN 导数 ℓ_u（2 维）
    DdpControl lu{DdpControl::Zero()};
    // GN 导数 ℓ_xx（7×7）：幅值/ESDF 项按 Gauss-Newton 丢弃约束二阶项
    DdpStateHessian lxx{DdpStateHessian::Zero()};
    // GN 导数 ℓ_uu（2×2）
    DdpControlHessian luu{DdpControlHessian::Zero()};
    // GN 导数 ℓ_ux（2×7）
    DdpControlStateHessian lux{DdpControlStateHessian::Zero()};
};
// 全轨迹求值结果：N+1 个阶段（0..N-1 运行阶段，N 终端阶段）
struct DdpCostEvaluation {
    // 逐阶段代价分解与导数
    DdpAlignedVec<DdpStageCostDerivatives> stages;
    // 全轨迹总代价（各阶段累加）
    double total_cost{0.0};
};
// 阶段一/阶段二代价与约束统一求值层（设计文档 DDP.md 2.3/2.4 节）：平滑主项 +
// 退火跟踪（角度 wrap 与初值提取/终端约束同一实现）+ 可选换挡代理 +
// 状态幅值 AL 不等式 + ESDF 双 margin 惩罚 + 终点 AL 等式 + 阶段二门控
// （符号门/接缝零速/驻留帽，仅经 DdpCostInput 显式挂接计划时生效），输出
// 逐阶段代价分解与 GN 导数。约定：平滑/跟踪项含 dt 因子；AL 增广项与 ESDF
// 惩罚为逐阶段点态量不乘 dt；终端阶段恒评估 ESDF（终点避障不抽样）。
class DdpCostEvaluator {
   public:
    // esdf_constraint 可为 nullptr（无地图场景只评估运动学项）；
    // 构造校验配置：权重非负有限、shift_beta>0、幅值边界为正有限，
    // 非法输入抛 std::invalid_argument
    DdpCostEvaluator(DdpCostConfig config,
                     const DdpEsdfConstraint* esdf_constraint);
    // 全轨迹求值：reference 提供参考位姿/dt/终点目标（终点取末位姿）；
    // states 尺寸必须为 N+1、controls 为 N、幅值乘子为 5N、
    // 豁免掩码非空时为 N+1、tracking_weight 必须有限；
    // 维度/取值不符抛 std::invalid_argument。
    // 门控一致性（"阶段一禁止启用"的断言防御）：门控计划与门控乘子
    // 必须同在场且尺寸匹配——仅一方在场属配置错误，抛 std::logic_error；
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
    // 运行阶段 k（0..N-1）：平滑 + 跟踪 + 换挡代理 + 幅值 AL + ESDF
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
                               DdpStageCostDerivatives* out) const;
    // 换挡代理累加：σ_β 双门乘积经 σ(−z)=1−σ(z) 化简为
    // ℓ = w_g·(A + D − 2AD)（A=σ(v)、D=σ(v⁺)，v⁺=v+a·dt 为显式一步预测、
    // 刻意不代入动力学链以保持纯状态代价），控制导数恒零；门区精确二阶导
    // 不定（σ_β 二阶导变号，负曲率注入 Riccati 会破坏 GN 假设），(v,a)
    // 2×2 块统一做负特征值截断的 PSD 投影；门宽 beta 由调用方按外层退火
    // 调度逐轮供给
    void accumulateShiftProxy(const DdpState& x, double dt, double beta,
                              DdpStageCostDerivatives* out) const;
    // δ 奇异区护栏累加：w·max(0,|δ|−δ_guard)² 的光滑铰链（C¹，活动区
    // 梯度模 2w·(|δ|−δ_guard)），GN Hessian 恒 2w；控制导数恒零
    void accumulateDeltaGuard(const DdpState& x,
                              DdpStageCostDerivatives* out) const;
    // 曲率正则累加：w·(tanδ/L)²（积分型、含 dt），梯度 2w·tanδ·sec²δ/L²
    // 以三阶极点发散（δ 奇异区内屏障）；精确 Hessian 处处为正（天然 PSD，
    // 无需投影）；控制导数恒零
    void accumulateCurvaturePenalty(const DdpState& x, double dt,
                                    DdpStageCostDerivatives* out) const;
    // 幅值 AL 五项累加（v/a/ω 平方形态 + δ 双侧线性形态）
    void accumulateAmplitudeConstraints(
        std::size_t k, const DdpState& x,
        const DdpCostMultiplierState& multipliers,
        DdpStageCostDerivatives* out) const;
    // 阶段二门控累加（仅运行阶段、仅 gating_plan 在场时调用）：
    // 符号门 −s·v≤0（AL 不等式）、接缝 v=0（AL 等式）、驻留帽
    // |v|−cap≤0（AL 不等式），全部只依赖 v，控制导数恒零
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
    DdpCostConfig config_;
    // ESDF 双 margin 惩罚（不持有所有权，可为空）
    const DdpEsdfConstraint* esdf_constraint_;
};
}  // namespace apa_post_processor
