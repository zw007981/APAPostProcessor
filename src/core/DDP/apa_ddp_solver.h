#pragma once

#include <memory>
#include <vector>

#include "al_outer_loop.h"
#include "bicycle_dynamics.h"
#include "ddp_cost.h"
#include "ddp_reference_builder.h"
#include "ms_ilqr.h"

namespace apa_post_processor {
// 阶段一求解编排配置：聚合内层 MS-iLQR、外层 AL 与代价求值三层
struct ApaDdpSolverConfig {
    // 默认构造：固定 merit μ₀=100（自适应规则已证伪），内层判据收紧到
    // 1e-9（防大 μ 量级误判）
    ApaDdpSolverConfig() {
        inner.merit_mu0 = 100.0;
        inner.cost_change_tol = 1e-9;
    }
    // 内层配置
    MsIlqrConfig inner;
    // 外层配置
    AlOuterLoopConfig outer;
    // 目标值
    DdpCostConfig cost;
    // 阶段二外层迭代上限：长视窗实测 10~11 轮收敛，取 16 留余量
    int stage_two_max_outer_iterations{16};
    // 门控罚权重初值取 10：弱启动时 μ 增长被误判"充分下降"而抑制
    double gating_mu_initial{10.0};
    // 门控罚权重上界
    double gating_mu_max{1e6};
    // 门控违反度容差
    double gating_tol{1e-2};
};
enum class ApaDdpStatus {
    CONVERGED,
    MAX_OUTER_ITERATIONS,
    INNER_SOLVER_FAILED,  // ρ_reg 溢出，保留最后可用轨迹
    // 升级机制耗尽（阶段二提前退出）：违反组 μ 全部钉在上限且 λ 漂移
    // 未带来门控级下降，保留最后可用轨迹
    ESCALATION_EXHAUSTED
};
// 升级机制耗尽判据（阶段二提前退出）的输入快照
struct EscalationState {
    bool terminal_ok{false};
    bool terminal_wanted{false};
    bool terminal_pinned{false};
    bool inequality_ok{false};
    bool amplitude_wanted{false};
    bool amplitude_pinned{false};
    bool gating_ok{false};
    bool gating_wanted{false};
    bool gating_pinned{false};
    bool has_gating{false};
    bool inner_stationary{false};
};
// 升级机制耗尽：AL 外层只有两种升级手段——μ 增长（门控式）与 λ 漂移
// （每轮无条件 λ←λ+μg）。当内层已达当前 AL 不动点、所有仍违反组的 μ
// 钉在上限、且本轮门控要求升级而无法升级时，本轮 λ 增量已注入；下一轮
// 仍为该状态则说明 λ 漂移也未带来门控级下降，两种机制均耗尽。
// 判据只消费算法既有状态与常量，不引用任何数据集/轮数经验。
bool EscalationStuck(const EscalationState& s);
struct ApaDdpOuterRecord {
    // 外层轮次
    int outer_index{0};
    // 跟踪权重
    double tracking_weight{0.0};
    // 罚权重 μ
    double mu{0.0};
    // 幅值组罚权重最大值（本轮开始时）
    double mu_amplitude{0.0};
    // 门控组罚权重（本轮开始时，阶段二专属）
    double mu_gating{0.0};
    // μ 是否提升
    bool mu_increased{false};
    // 基础代价 J_s′
    double base_cost{0.0};
    // AL 增广总代价
    double augmented_cost{0.0};
    // 终点位置误差 (m)
    double terminal_position_error{0.0};
    // 终点朝向误差 (deg)
    double terminal_heading_error_deg{0.0};
    // 最大幅值违反度
    double max_amplitude_violation{0.0};
    // 打靶缺陷 ‖d‖∞
    double defect_norm_inf{0.0};
    // 虚拟控制 ‖w‖∞（开关关闭时为 0）
    double virtual_control_norm_inf{0.0};
    // 内层求解状态
    MsIlqrStatus inner_status{MsIlqrStatus::MAX_ITERATIONS};
    // 内层迭代数
    int inner_iterations{0};
};
// 阶段一求解报告
struct ApaDdpReport {
    // 求解状态
    ApaDdpStatus status{ApaDdpStatus::MAX_OUTER_ITERATIONS};
    // 逐轮诊断历史
    std::vector<ApaDdpOuterRecord> history;
    // 外层轮数
    int outer_iterations{0};
    // 累计内层迭代数
    int total_inner_iterations{0};
    // 定义域守卫拒绝数
    std::int64_t domain_guard_rejections{0};
    // 内层溢出冷重启次数
    int inner_restarts{0};
    // 标定 μ⁰
    double mu_initial_calibrated{0.0};
    // 最终罚权重
    double mu_final{0.0};
    // 最终幅值组罚权重最大值
    double mu_amplitude_final{0.0};
    // 最终门控组罚权重
    double mu_gating_final{0.0};
    // 终点位置误差 (m)
    double terminal_position_error{0.0};
    // 终点朝向误差 (deg)
    double terminal_heading_error_deg{0.0};
    // 最大幅值违反度
    double max_amplitude_violation{0.0};
    // 打靶缺陷 ‖d‖∞
    double defect_norm_inf{0.0};
    // 虚拟控制 ‖w‖∞（开关关闭时为 0）
    double virtual_control_norm_inf{0.0};
    // 终态基础代价
    double final_cost{0.0};
    // 终点达标
    bool terminal_ok{false};
    // 不等式达标
    bool inequality_ok{false};
    // 缺陷达标
    bool defect_ok{false};
};
struct ApaDdpStageOneResult {
    DdpAlignedVec<DdpState> states;
    DdpAlignedVec<DdpControl> controls;
    // 求解报告
    ApaDdpReport report;
    // 供阶段二对偶热启动
    DdpCostMultiplierState final_multipliers;
};
struct ApaDdpStageTwoResult {
    DdpAlignedVec<DdpState> states;
    DdpAlignedVec<DdpControl> controls;
    // 求解报告
    ApaDdpReport report;
    // 符号门最大违反度
    double max_sign_violation{0.0};
    // 驻留帽最大违反度
    double max_dwell_violation{0.0};
    // 接缝最大 |v|
    double max_seam_speed{0.0};
    // 门控达标
    bool gating_ok{false};
};
// 内层实例类型在构造时按 config.inner.use_virtual_control 运行时选择
// （编排层虚调用调度，不进入浮点密集函数，默认关闭实例机器码与基线一致）
// APA-DDP 求解编排入口：阶段一全局软化 + 阶段二门控精化，δ 投影防 tanδ
// 奇异区，内层溢出冷重启一次再判死
class ApaDdpSolver {
   public:
    // 构造时注入动力学/代价求值层与配置，构造内层求解器与外层 AL 状态机
    ApaDdpSolver(ApaDdpSolverConfig config, const BicycleDynamics* dynamics,
                 const DdpCostEvaluator* cost_evaluator);
    // 阶段一全局软化：任何出口均返回最后可用轨迹与求解报告
    ApaDdpStageOneResult solveStageOne(const DdpReference& reference);
    // 热启动重载：以调用方给定轨迹启动
    ApaDdpStageOneResult solveStageOne(
        const DdpReference& reference,
        const DdpAlignedVec<DdpState>& warm_states,
        const DdpAlignedVec<DdpControl>& warm_controls);
    // 阶段二门控精化：修剪后重解，禁止直接拼接。dual_seed
    // 为阶段一终态乘子（对偶热启动）
    ApaDdpStageTwoResult solveStageTwo(
        const DdpReference& reference, const DdpGatingPlan& gating_plan,
        const DdpAlignedVec<DdpState>& warm_states,
        const DdpAlignedVec<DdpControl>& warm_controls, double tracking_weight,
        const DdpCostMultiplierState* dual_seed = nullptr);
    // 求解配置（只读）
    const ApaDdpSolverConfig& config() const { return config_; }
    // 内层 MS-iLQR 求解器（只读引用，供诊断消费；实例类型随配置开关）
    const MsIlqrSolverInterface& innerSolver() const { return *inner_solver_; }

   protected:
    // 内层实例工厂：按 config.inner.use_virtual_control 重建对应实例
    // （构造与溢出冷重启共用同一入口，保证实例类型与配置一致）
    void resetInnerSolver();
    // 内层韧性封装：溢出后冷重启重试一次（陈旧 QP 活动集僵局），仍失败才判死
    MsIlqrResult solveInnerResilient(
        const DdpReference& reference, AlOuterLoop* outer,
        DdpCostMultiplierState* multipliers, const DdpCostInput& cost_input,
        const DdpAlignedVec<DdpState>& warm_states,
        const DdpAlignedVec<DdpControl>& warm_controls,
        DdpAlignedVec<DdpState>* virtual_controls, double mu_round,
        ApaDdpReport* report);
    // merit 地板挂钩：取终端/幅值两组 AL 罚权重的较大者
    static double MeritAlHook(double mu_terminal, double mu_amplitude) {
        return std::max(mu_terminal, mu_amplitude);
    }
    // 门控快照：measureGating/updateGating 间的传递结构体。
    // 虽为 .cpp 内部实现细节，但因包含 Eigen 向量成员且需按值返回，
    // 无法在仅有前向声明的头文件中完成——保留在此处以供单元测试白盒访问
    struct GatingSnapshot {
        // 符号门残差 max(0,−s·v)
        Eigen::VectorXd sign_g;
        // 符号门最大违反度
        double max_sign_violation{0.0};
        // 接缝等式残差
        Eigen::VectorXd seam_c;
        // 接缝最大 |v|
        double max_seam_speed{0.0};
        // 驻留帽残差 max(0,|v|−cap)
        Eigen::VectorXd dwell_g;
        // 驻留帽最大违反度
        double max_dwell_violation{0.0};
        // 门控 μ 增长的比较量
        double violation_norm{0.0};
    };
    GatingSnapshot measureGating(const DdpGatingPlan& plan,
                                 const DdpAlignedVec<DdpState>& states) const;
    // Hestenes-Powell + 门控 μ 增长：首轮只记录不增长，返回本轮 μ 是否
    // 提升；out_wanted 非空时输出门控是否提出增长（提前退出判据消费）
    bool updateGating(const GatingSnapshot& snapshot,
                      DdpCostMultiplierState* multipliers, double* gating_mu,
                      double* prev_violation,
                      bool* out_wanted = nullptr) const;
    // 外层 AL 循环公共实现：阶段一与阶段二共享同一套迭代框架。
    // gating_plan 非空时启用阶段二专属的门控量测、收敛判据与乘子更新；
    // 门控输出参数在阶段一时传入 nullptr
    void runOuterLoop(const DdpReference& reference, AlOuterLoop* outer,
                      DdpCostMultiplierState* multipliers,
                      const DdpAlignedVec<DdpState>& warm_start_states,
                      const DdpAlignedVec<DdpControl>& warm_start_controls,
                      DdpAlignedVec<DdpState>* warm_virtual_controls,
                      int max_rounds, double tracking_weight,
                      const std::vector<bool>* anneal_exempt_mask,
                      const DdpGatingPlan* gating_plan, ApaDdpReport* report,
                      double* out_max_sign_violation,
                      double* out_max_dwell_violation,
                      double* out_max_seam_speed,
                      bool* out_gating_ok);

   protected:
    // 配置
    ApaDdpSolverConfig config_;
    // 动力学模型（不持有所有权）
    const BicycleDynamics* dynamics_;
    // 代价求值层（不持有所有权）
    const DdpCostEvaluator* cost_evaluator_;
    // 外层 AL 状态机（跨轮次保持 μ/退火状态）
    AlOuterLoop outer_loop_;
    // RAII 持有以支持溢出冷重启（接口指针，实例类型随配置开关）
    std::unique_ptr<MsIlqrSolverInterface> inner_solver_;
};
}  // namespace apa_post_processor
