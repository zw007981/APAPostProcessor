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
    // 跟踪权重退火率 γ：每轮 w_ref ← w_ref·γ（豁免点不衰减）
    double anneal_gamma{0.5};
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
