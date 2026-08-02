#pragma once

#include <memory>
#include <vector>

#include "al_outer_loop.h"
#include "bicycle_dynamics.h"
#include "ddp_cost.h"
#include "ddp_reference_builder.h"
#include "ms_ilqr.h"

namespace apa_post_processor {
// 阶段一求解编排配置：聚合内层 MS-iLQR、外层 AL 与代价求值三层配置。
// 注意：本结构的默认构造会覆写 inner 的若干默认值（merit_kappa_d /
// merit_mu0 / cost_change_tol，理由见构造函数注释），三层并非彼此独立、
// 各取其 struct 默认值；cost 必须与代价求值层构造所用配置一致（幅值
// 边界同时进入外层违反度量测与代价惩罚，不一致会使乘子更新与终止判据
// 失配）
struct ApaDdpSolverConfig {
    // 阶段一编排默认标定（端到端调参阶段可再修订）：
    // - merit μ_m 的自适应规则在真实数据上会随 ‖d‖ 减小而爆炸
    //   （μ₀+|EC(1)|/((1−ρ)‖d‖) 的棘轮实测冲到 8e4 量级），放行
    //   "以任意代价歼灭残余缺陷"的破坏性步骤（轨迹被拖穿障碍、终点
    //   被打飞）；编排层默认把 κ_d 取为天文数字使该规则永不触发，
    //   μ_m 钉住在 μ₀=100（缺陷修复与代价下降的交换比固定、有界）；
    // - 内层相对代价判据收紧到 1e-9：AL 罚权重增大后总代价量级很大，
    //   默认 1e-6 会在只前进了微步时被误报为收敛（尺度盲）
    ApaDdpSolverConfig() {
        inner.merit_kappa_d = 1e9;
        inner.merit_mu0 = 100.0;
        inner.cost_change_tol = 1e-9;
    }
    MsIlqrConfig inner;
    AlOuterLoopConfig outer;
    DdpCostConfig cost;
    // 以下为阶段二门控精化的配置项（不影响阶段一行为）：
    // 阶段二外层迭代上限（热启动精化通常数轮即收敛，预算远小于阶段一；
    // 真实长视窗数据集（N≈400~700、多接缝）实测 8 轮预算不足——data3/
    // data7 在 10~11 轮收敛，取 16 留有余量）
    int stage_two_max_outer_iterations{16};
    // 门控乘子的初始罚权重与增长上限（符号门/接缝等式/驻留帽共用一套
    // 调度）。初值取 10 的考虑：门控约束局域在接缝邻域（点数少、热启动
    // 已接近可行），弱启动时 λ 累积驱动的慢速下降会被门控判为"充分
    // 下降"而抑制 μ 增长，驻留帽边沿残差收敛过慢（实测 8 轮仍超阈）
    double gating_mu_initial{10.0};
    double gating_mu_max{1e6};
    // 门控违反度容差（阶段二联合终止判据）：符号门/驻留帽线性违反度与
    // 接缝 |v| 的上限
    double gating_tol{1e-2};
    // 对偶热启动种子 μ 的量级自适应上限 κ（0 = 关闭，抬升行为不变）：
    // 种子量级正常时照常抬升（μ⁰ 标定下限抬到种子水平是已收敛终端平衡
    // 的必要条件）；种子已达 μ_max 的病态情形（阶段一终端 μ 被门控一路
    // 推到上限）截断为 κ·μ̂——μ̂ 是阶段二自身按 ALTRO clip 公式在热启动
    // 轨迹上量测的标定初值（不经种子抬升 μ_min 的版本）。实测依据：
    // 种子=μ_max=1e6 时阶段二内层从第 0 轮即强病态（线搜索拒绝一切
    // 移动、打靶缺陷降不动），而完全移除抬升会让正常量级种子的场景
    // 不收敛——只截断病态情形、保留正常抬升
    double seed_mu_cap_ratio{0.0};
};
// 阶段一求解状态：失败语义经状态码上报（供后处理/回退逻辑消费）
enum class ApaDdpStatus {
    CONVERGED,  // 联合判据达标（终点双指标 + 不等式 + 缺陷）
    MAX_OUTER_ITERATIONS,  // 外层迭代耗尽仍未达标（附结构化诊断）
    INNER_SOLVER_FAILED  // 内层致命失败（ρ_reg 溢出；输出最后可用轨迹）
};
// 单轮外层诊断记录：收敛分析、单元测试对拍与调参日志共用
struct ApaDdpOuterRecord {
    // 外层轮次（从 0 起）
    int outer_index{0};
    // 本轮跟踪权重 w_ref(r) 与罚权重 μ
    double tracking_weight{0.0};
    double mu{0.0};
    // 本轮更新是否提升了 μ（门控增长断言）
    bool mu_increased{false};
    // 基础代价 J_s′（零乘子求值）与内层收敛后的 AL 增广总代价
    double base_cost{0.0};
    double augmented_cost{0.0};
    // 本轮量测：终点双指标 / 状态不等式违反度 / 缺陷范数
    double terminal_position_error{0.0};
    double terminal_heading_error_deg{0.0};
    double max_amplitude_violation{0.0};
    double defect_norm_inf{0.0};
    // 内层求解状态与实际迭代数
    MsIlqrStatus inner_status{MsIlqrStatus::MAX_ITERATIONS};
    int inner_iterations{0};
};
// 阶段一求解报告：状态码 + 逐轮诊断 + 终态指标 + 分项达标标志。
// 未收敛时逐项给出哪类判据未达标与最终违反度，供失败回退记录诊断日志
struct ApaDdpReport {
    ApaDdpStatus status{ApaDdpStatus::MAX_OUTER_ITERATIONS};
    // 逐轮外层诊断（每轮一条）
    std::vector<ApaDdpOuterRecord> history;
    // 实际外层轮数与累计内层迭代数
    int outer_iterations{0};
    int total_inner_iterations{0};
    // 累计定义域守卫（L8.3）拒绝的试探候选数（L8.4 分项归因：
    // 「出界被拦住」的直接计数证据）
    std::int64_t domain_guard_rejections{0};
    // 内层溢出后的冷重启重试次数（box-QP 陈旧活动集僵局的兜底计数）
    int inner_restarts{0};
    // 首轮标定的 μ⁰ 与最终罚权重
    double mu_initial_calibrated{0.0};
    double mu_final{0.0};
    // 终态量测指标（任一出口均保留最后一轮量测）
    double terminal_position_error{0.0};
    double terminal_heading_error_deg{0.0};
    double max_amplitude_violation{0.0};
    double defect_norm_inf{0.0};
    // 终态基础代价 J_s′（零乘子求值）
    double final_cost{0.0};
    // 分项达标标志（失败诊断：哪类判据未达标）
    bool terminal_ok{false};
    bool inequality_ok{false};
    bool defect_ok{false};
};
// 阶段一求解输出：优化后状态/控制轨迹与求解报告
struct ApaDdpStageOneResult {
    // 优化后状态轨迹（尺寸 N+1）与控制序列（尺寸 N）
    DdpAlignedVec<DdpState> states;
    DdpAlignedVec<DdpControl> controls;
    ApaDdpReport report;
    // 终态 AL 乘子状态（供阶段二对偶热启动：阶段一建立的终端/幅值平衡
    // 由 λ/μ 承载，丢弃它们会把已收敛的平衡重新打开，实测终端误差被
    // 拖大一个量级、收敛轮数翻倍）；未求解时为全空向量
    DdpCostMultiplierState final_multipliers;
};
// 阶段二求解输出：在阶段一报告结构（status 为阶段一判据与门控判据的
// 联合结论）基础上附加门控终态量测，供后处理校验与失败诊断消费
struct ApaDdpStageTwoResult {
    // 优化后状态轨迹（尺寸 N+1）与控制序列（尺寸 N）
    DdpAlignedVec<DdpState> states;
    DdpAlignedVec<DdpControl> controls;
    ApaDdpReport report;
    // 门控终态量测（线性形式）：符号门 max(0,−s·v)、驻留帽 max(0,|v|−cap)、
    // 接缝 max|v|
    double max_sign_violation{0.0};
    double max_dwell_violation{0.0};
    double max_seam_speed{0.0};
    // 门控分项达标标志（失败诊断）
    bool gating_ok{false};
};
// APA-DDP 求解编排入口：把前端参考数据、MS-iLQR 内层与 AL 外层调度串成
// 完整的阶段一求解（全局软化：不施加任何符号门控/接缝静止窗，换挡相关
// 机制只属后续门控精化阶段）。外层轮次间以收敛轨迹热启动内层（μ_m 与
// ρ_reg 随内层实例自然跨轮保持），热启动前对 δ 做物理边界投影——换挡区
// v≈0 时 tanδ 允许经 ±π/2 奇异区"原地转头"（AL 不等式在 g<0 时不可见
// 该漏洞），投影 + λ 累积内屏障防止轨迹卡死在奇异盆地。内层迭代超限
// 不致命（沿用当前迭代点继续外层更新）；内层 ρ_reg 溢出时先以冷重启
// 实例重试本轮一次（溢出部分源于陈旧 QP 活动集热启动与衰减到地板的
// ρ_reg，全新实例在同一起点重解通常可通过），重试仍失败才判定致命
// 失败并保留最后可用轨迹（anytime 性质）。失败出口输出完整结构化
// 诊断，不返回未初始化数据。
class ApaDdpSolver {
   public:
    // 构造校验：动力学/代价求值层必须非空（不持有所有权）；
    // 三层配置的合法性由各自组件构造时校验，非法输入抛 std::invalid_argument
    ApaDdpSolver(ApaDdpSolverConfig config, const BicycleDynamics* dynamics,
                 const DdpCostEvaluator* cost_evaluator);
    // 阶段一全局软化求解：reference 必须自洽（位姿 ≥ 2、状态初值 N+1、
    // 控制初值 N），不符抛 std::invalid_argument；任何出口（含失败）
    // 均返回最后可用轨迹与完整求解报告
    ApaDdpStageOneResult solveStageOne(const DdpReference& reference);
    // 阶段一热启动重载（margin 延续/救援重试等场景）：首轮以调用方给定
    // 的轨迹启动（典型为另一次求解的收敛解），而非参考自带的前端初值；
    // warm_states 尺寸 N+1、warm_controls 尺寸 N，不符抛
    // std::invalid_argument
    ApaDdpStageOneResult solveStageOne(
        const DdpReference& reference,
        const DdpAlignedVec<DdpState>& warm_states,
        const DdpAlignedVec<DdpControl>& warm_controls);
    // 阶段二门控精化重解（设计文档 2.6 节第 3 步，必须重解、禁止直接
    // 拼接修剪结果）：以修剪后轨迹热启动，施加符号门/接缝零速/驻留帽
    // 三类门控 AL 约束，外层调度与阶段一同源（自适应 μ⁰ 标定 + 分组
    // 门控增长），联合终止判据在阶段一判据上追加门控违反度。
    // tracking_weight 为阶段二恒定的跟踪权重（不再退火——典型取阶段一
    // 末轮的退火后权重，由调用方给定；必须非负有限）。dual_seed 为阶段一
    // 终态乘子（对偶热启动：终端 λ/μ 直接续接，μ⁰ 标定下限随之抬升到
    // 种子水平；nullptr = 对偶冷启动）。gating_plan 尺寸必须为 N+1
    // （接缝数任意），warm_states 尺寸 N+1、warm_controls 尺寸 N，不符
    // 抛 std::invalid_argument；失败出口同样保留最后可用轨迹与完整诊断
    ApaDdpStageTwoResult solveStageTwo(
        const DdpReference& reference, const DdpGatingPlan& gating_plan,
        const DdpAlignedVec<DdpState>& warm_states,
        const DdpAlignedVec<DdpControl>& warm_controls, double tracking_weight,
        const DdpCostMultiplierState* dual_seed = nullptr);
    // 配置只读访问（后处理校验需要幅值/盒约束边界同源）
    const ApaDdpSolverConfig& config() const { return config_; }

   protected:
    // 阶段二门控残差快照：在收敛轨迹上量取，供乘子更新、门控增长与
    // 联合终止判据共用；残差公式与代价求值层的静态助手严格同一来源
    struct GatingSnapshot {
        // 符号门残差（尺寸 N+1，无门节点为 0）与线性违反度 max(0,−s·v)
        Eigen::VectorXd sign_g;
        double max_sign_violation{0.0};
        // 接缝等式残差（尺寸 S）与 max|v|
        Eigen::VectorXd seam_c;
        double max_seam_speed{0.0};
        // 驻留帽残差（尺寸 N+1，无窗节点为 0）与线性违反度 max(0,|v|−cap)
        Eigen::VectorXd dwell_g;
        double max_dwell_violation{0.0};
        // 联合违反度范数（门控 μ 增长的比较量，惩罚形态残差聚合）
        double violation_norm{0.0};
    };
    // 阶段二门控残差量测：states 尺寸必须为 N+1
    GatingSnapshot measureGating(const DdpGatingPlan& plan,
                                 const DdpAlignedVec<DdpState>& states) const;
    // 阶段二门控乘子更新（Hestenes-Powell + 单组门控 μ 增长，与外层
    // 幅值组同一排程哲学）：λ 更新用惩罚形态残差，μ 仅当联合违反度
    // 未充分下降（>κ·上轮）才按 φ 倍提升（首轮只记录不增长）；
    // 返回本轮 μ 是否提升
    bool updateGating(const GatingSnapshot& snapshot,
                      DdpCostMultiplierState* multipliers, double* gating_mu,
                      double* prev_violation) const;

   protected:
    ApaDdpSolverConfig config_;
    // 动力学与代价求值层（不持有所有权，必须非空）
    const BicycleDynamics* dynamics_;
    const DdpCostEvaluator* cost_evaluator_;
    // 外层调度状态机与内层求解器；内层按 RAII 持有以便溢出后冷重启
    AlOuterLoop outer_loop_;
    std::unique_ptr<MsIlqrSolver> inner_solver_;
};
}  // namespace apa_post_processor
