#pragma once

#include <Eigen/Core>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "ddp_cost.h"
#include "ddp_reference_builder.h"

namespace apa_post_processor {
// AL 外层循环配置：乘子/罚权重更新、跟踪权重退火与联合终止判据的全部
// 调度参数。默认值取设计文档 2.5 节参数表与 ALM 侧同构机制的既有约定，
// 最终标定属端到端调参
struct AlOuterLoopConfig {
    // 外层迭代上限（超限按未收敛退出，由编排层输出结构化诊断）
    int max_outer_iterations{20};
    // 终点双指标：位置容差 (m) 与朝向容差 (deg)
    double terminal_position_tol{0.05};
    double terminal_heading_tol_deg{1.5};
    // 状态不等式违反度容差：max(0,g) 全维最大值的工程容差（平方/线性
    // 形态混合量纲，量级参照速度/转角余量）
    double inequality_tol{1e-2};
    // 打靶缺陷 ‖d‖∞ 容差（缺陷归零判据）
    double defect_tol{1e-3};
    // 罚权重自适应标定的 clip 区间 [μ_min, μ_max]
    double mu_min{1e2};
    double mu_max{1e6};
    // 首轮内层的临时终点罚权重（λ=0 纯软惩罚轮；首轮收敛后即被标定的
    // μ⁰ 替换，沿用 ALM 侧 first_round_rho 的启动约定）
    double first_round_mu{1.0};
    // 幅值不等式的初始罚权重：与终点首轮权重同量级（弱启动）即可——
    // 物理边界违反靠 λ 累积与分组门控渐硬压回；强启动会使首轮内层在
    // 初值滚动的速度斜坡惩罚下直接进入病态（实测反例）。换挡区 δ 经
    // tanδ 奇异区"原地转头"的漏洞由编排层的热启动投影封闭，不靠 μ
    double amplitude_mu_initial{1.0};
    // μ⁰ 标定公式的小量 ε_μ：防终点残差范数趋零时分母退化
    double epsilon_mu{1e-4};
    // 充分下降门控：本轮违反度 > κ·上轮 才提升 μ（κ<1）；避免盲目指数
    // 增长导致内层 Riccati 病态
    double mu_gate_kappa{0.9};
    // 罚权重增长倍率 φ（每次提升 μ ← min(φμ, μ_max)）
    double mu_growth_factor{10.0};
    // 幅值组的独立 μ 上限（默认 = μ_max，既有行为不变）：长视窗死亡螺旋
    // 由两分组共用同一 μ_max 的指数攀升驱动——幅值组独立封顶后，λ 仍按
    // μ·g 持续累积（AL 本职机制：罚中心 −λ/μ 逐轮内移压回违反），内层
    // Riccati 不被无界罚权重推入病态；终端组仍可到 μ_max（存在真实需要
    // 高罚权重的终端收敛场景）
    double amplitude_mu_max{1e6};
    // 跟踪权重退火率 γ：每轮 w_ref ← w_ref·γ（豁免点不衰减）
    double anneal_gamma{0.5};
    // 退火保持轮数：前 k 轮不退火（w_ref 恒为 w_ref,0），让 AL 先把终端/
    // 幅值约束建立起来，之后按 γ 快速退火——避免几何退火在早期就削弱
    // 跟踪导致解跑偏撞障（data1 在 γ=0.3 直接退火下阶段一发散的教训）。
    // 0 = 从第 0 轮即退火（既有行为不变）
    int anneal_hold_rounds{0};
    // 换挡代理 σ_β 门宽的逐轮退火调度（仅在 weight_shift>0 时生效）：
    // β(r)=max(shift_beta_final, shift_beta_initial·shift_beta_gamma^r)。
    // 宽门启动（0.3 m/s：梯度覆盖大 |v| 范围，非凸项早期可优化），逐轮
    // 收窄到地板值（0.05 m/s：逼近阶跃的换挡判决，与符号游程滞回阈值
    // 同量级）；地板必须为正——β→0 时 σ_β 梯度在 v=0 处爆炸
    double shift_beta_initial{0.3};
    double shift_beta_final{0.05};
    double shift_beta_gamma{0.8};
    // 候选待融段的临界比阈值：内部 maneuver 的 crit=T⁵·n_pts·dt 低于该
    // 阈值即标记为待融候选（平衡式：平滑项 ~w_j·Δs²/T⁵ vs 跟踪项
    // ~w_ref·Δs²·n_pts·dt，crit 即「融化该段所需的最小 w_j/w_ref」）。
    // 默认值 5000 取自四数据集残余段临界比分布的间隙带（已融段 ≤2.2e3、
    // 从未融段 ≥8.5e3）
    double melt_crit_threshold{5000.0};
    // 候选段的退火率 γ_cand（逐轮 w_cand ← w_cand·γ_cand）：默认 0.5 与
    // 全局 γ 相同——此时候选段与普通点同权，机制等价于关闭（基线行为）；
    // 调小（如 0.25）启用深退火：候选段是「判据上就该融」的段，尽快把
    // 裁决权交还平滑项；非候选段仍按全局 γ 正常退火（同伦类保持不受影响）
    double candidate_anneal_gamma{0.5};
    // 退火终点自适应（逃逸指标冻结，全部默认 0 = 关闭）：任一启用指标
    // 越阈即冻结退火——w_ref 停在当前轮次不再衰减（全路段同时冻结、
    // 无段间对拉，与空间维候选掩码的失败模式不同源），直到收敛判据
    // 满足。冻结期间 μ 增长同步冻结（λ 继续累积——AL 本职机制），避免
    // 「退火冻结后违反度平台期被判未充分下降」的伪 μ 增长坑。本质是
    // 把 trust-region 思想用在退火调度上：退火深度不再是超参，而是由
    // 「解仍在同伦类内」这一可观测量守护的自适应量
    // 解长度环比阈值（>0 启用）：本轮解长度超过 上一轮×该值 即冻结
    double anneal_freeze_length_growth{0.0};
    // 解相对参考位姿的最大横向偏离阈值 (m)（>0 启用）
    double anneal_freeze_lateral_deviation{0.0};
    // 打靶缺陷范数 ‖d‖∞ 阈值（>0 启用）
    double anneal_freeze_defect{0.0};
    // 解长度/参考总长的绝对比阈值（>0 启用）：环比触发在慢速跑飞（每轮
    // +4% 复利、单调不触发）下永远静默，且触发时 w_ref 已深退火、冻结
    // 无力拉回；绝对比在逃逸起步（w_ref 仍高）时即触发。健康数据集的
    // 稳态解长度恒 ≤ 参考长度（融化只会缩短），阈值 1.x 对它们天然静默
    double anneal_freeze_ref_length_ratio{0.0};
};
// 约束残差快照：内层收敛后由外层在收敛轨迹上量取，供乘子更新、门控
// 增长与终止判据共用。残差公式/符号/角度 wrap 与代价求值层严格同一组
// 约定（量测与惩罚失配会直接破坏 AL 收敛性）
struct AlConstraintSnapshot {
    // 幅值不等式残差 g（尺寸 5N，布局与乘子状态一致：阶段 k 的 5 个
    // 约束位于 [5k, 5k+5)，v²/a²/ω² 平方形态 + δ 双侧线性形态）
    Eigen::VectorXd amplitude_g;
    // 终点等式残差 c = [x−xg, y−yg, wrap(θ−θg), v, a]（5 维）
    Eigen::Matrix<double, DDP_TERMINAL_CONSTRAINT_DIM, 1> terminal_c;
    // 状态不等式违反度：max(0, g) 全维最大
    double max_amplitude_violation{0.0};
    // 终点双指标：位置误差 (m) 与朝向误差 (deg)
    double terminal_position_error{0.0};
    double terminal_heading_error_deg{0.0};
    // 打靶缺陷 ‖d‖∞（全节点全分量绝对值最大）
    double defect_norm_inf{0.0};
    // 联合违反度范数 sqrt(‖c‖² + Σ max(0,g)²)：门控增长的比较量
    double violation_norm{0.0};
};
// 联合终止判据的分项结果：未收敛时逐项给出诊断（哪一类判据未达标）
struct AlTerminationCheck {
    // 终点双指标（位置 + 朝向）达标
    bool terminal_ok{false};
    // 状态不等式违反度达标
    bool inequality_ok{false};
    // 打靶缺陷范数达标
    bool defect_ok{false};
    // 三类判据联合（全部达标才宣告收敛）
    bool converged() const { return terminal_ok && inequality_ok && defect_ok; }
};
// AL 外层循环（ALTRO 式乘子/罚权重状态机）：自适应 μ⁰ clip 标定（首轮
// 内层收敛后生效）、乘子更新（等式 λ+=μc、不等式 λ=max(0,λ+μg)）、
// 充分下降门控 μ 增长、跟踪权重退火调度（首/末 maneuver 豁免）与联合
// 终止判据（终点双指标 + 状态不等式违反度 + 缺陷范数）。本类只做调度
// 与判据，不拥有求解器；内外层编排与求解报告属上层求解器职责。
class AlOuterLoop {
   public:
    // 构造校验：迭代上限/容差/clip 区间/门控/退火参数必须为正且自洽，
    // 代价配置提供幅值边界（g 公式与代价求值层同一组，必须与其构造
    // 配置一致），非法输入抛 std::invalid_argument
    AlOuterLoop(AlOuterLoopConfig config, DdpCostConfig cost_config);
    // 当前外层轮次 r（从 0 起，update 后推进）
    int round() const { return round_; }
    // 本轮跟踪权重 w_ref(r) = w_ref,0·γ^r
    double trackingWeight() const;
    // 本轮换挡代理门宽 β(r) = max(β_final, β_initial·γ_β^r)
    double shiftBeta() const;
    // 本轮候选待融段跟踪权重 w_cand(r) = w_ref,0·γ_cand^r（深退火）
    double candidateTrackingWeight() const;
    // 候选待融段掩码（尺寸 N+1）：临界比 crit=T⁵·n_pts·dt 低于阈值的
    // 内部 maneuver 覆盖的点为 true（含与邻段共享的边界点）；首/末
    // maneuver 是融化保护段（承载起点状态与终点语义），任何判据下都不
    // 标记；无 maneuver 元数据的参考（合成用例）不标记任何点
    std::vector<bool> makeMeltCandidateMask(
        const DdpReference& reference) const;
    // 退火豁免掩码（尺寸 N+1）：首/末 maneuver 覆盖的点恒 true（首末段
    // 承载起点状态与终点语义）；单 maneuver 路径整体豁免；无 maneuver
    // 元数据的参考（合成用例）不豁免任何点。掩码由前端元数据生成
    std::vector<bool> makeAnnealExemptMask(const DdpReference& reference) const;
    // 首轮乘子状态：λ=0；终点罚权重取 first_round_mu（弱启动，首轮收敛
    // 后即被标定的 μ⁰ 替换），幅值罚权重取 amplitude_mu_initial（强启动，
    // 从第 0 轮压住 δ/车速等物理边界的越界）
    DdpCostMultiplierState makeInitialMultipliers(std::size_t num_steps) const;
    // 在收敛轨迹上量取约束残差快照（幅值 g 全量 + 终点 c + 聚合量）；
    // states/defects 尺寸必须均为 N+1，不符抛 std::invalid_argument
    AlConstraintSnapshot measure(const DdpReference& reference,
                                 const DdpAlignedVec<DdpState>& states,
                                 const DdpAlignedVec<DdpState>& defects) const;
    // 联合终止判据：终点双指标 + 状态不等式违反度 + ‖d‖∞（边界取等
    // 视为达标）
    AlTerminationCheck checkTermination(
        const AlConstraintSnapshot& snapshot) const;
    // 外层更新（内层收敛且未达标后调用）：首轮先按
    // μ⁰ = clip(J_s′/max(‖c‖², ε_μ), μ_min, μ_max) 标定终点罚权重
    // （标定轮只标定不增长——μ⁰ 刚按违反度量级标定，立即 φ 倍增长与
    // 标定自相矛盾，会把次轮内层直接压入病态），随后 λ 更新（终点等式
    // λ+=μ_term·c、幅值不等式 max(0,λ+μ_amp·g)）、分组门控 μ 增长
    // （自次轮起：终点组/幅值组各自仅当本组违反度未充分下降才
    // μ=min(φμ,μ_max)，避免单点顽固违反把另一组一并推入病态），最后
    // 推进轮次（退火随之生效）。base_cost 为零乘子基础代价 J_s′。
    // 返回本轮是否有任一组 μ 提升；乘子尺寸与快照不符抛 std::invalid_argument
    bool update(const AlConstraintSnapshot& snapshot, double base_cost,
                DdpCostMultiplierState* multipliers);
    // 当前终点罚权重（首轮为 first_round_mu，首轮更新起为标定/门控后的值）
    double mu() const { return mu_terminal_; }
    // 当前幅值不等式罚权重（amplitude_mu_initial 启动，随幅值组门控增长）
    double muAmplitude() const { return mu_amplitude_; }
    // 首轮标定的终点 μ⁰（未标定前为 first_round_mu）
    double calibratedMu() const { return mu_calibrated_; }
    // μ 增长次数（门控断言用）
    std::int64_t mu_increase_count() const { return mu_increase_count_; }
    // 退火终点自适应：逃逸指标上报——任一启用指标越阈即置冻结标志
    // （滞回，冻结后不自动解冻，防权重振荡）；ref_length_ratio 为
    // 解长度/参考总长的绝对比（未启用对应阈值时可传任意值，不被消费）
    void reportEscapeIndicators(double length_growth, double lateral_deviation,
                                double defect_norm, double ref_length_ratio);
    // 当前退火是否已被逃逸指标冻结
    bool annealFrozen() const { return anneal_frozen_; }
    // 清空轮次/违反度历史/计数器与标定状态（供同一实例多次求解复用）
    void reset();

   protected:
    // 数值 clip 到 [lo, hi]
    static constexpr double Clip(double value, double lo, double hi) {
        return std::min(std::max(value, lo), hi);
    }

   protected:
    AlOuterLoopConfig config_;
    // 代价配置（只消费幅值边界与 w_ref,0，与求值层构造配置同源）
    DdpCostConfig cost_config_;
    // 当前外层轮次与两组罚权重状态（终点/幅值分离调度）
    int round_{0};
    // 退火专用轮次（退火冻结时停走，与外层轮次解耦——冻结语义的核心：
    // 冻结只停退火，外层 AL 的 λ/收敛判据照常推进）
    int anneal_round_{0};
    // 逃逸冻结标志（滞回，复位前不自动解冻）
    bool anneal_frozen_{false};
    double mu_terminal_{0.0};
    double mu_amplitude_{0.0};
    double mu_calibrated_{0.0};
    bool mu_calibrated_flag_{false};
    // 上一轮分组违反度（<0 表示尚无历史：标定轮只标定不增长）；
    // 终点组取 ‖c‖₂，幅值组取激活违反范数 sqrt(Σ max(0,g)²)
    double prev_terminal_violation_{-1.0};
    double prev_amplitude_violation_{-1.0};
    // μ 增长计数器（门控断言）
    std::int64_t mu_increase_count_{0};
};
}  // namespace apa_post_processor
