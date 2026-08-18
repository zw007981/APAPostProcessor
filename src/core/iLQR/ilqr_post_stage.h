#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "../../spatial/esdf_map.h"
#include "../../util/path.h"
#include "../../util/topology_cleaner.h"
#include "../../util/trajectory.h"
#include "../../vehicle/vehicle_footprint_model.h"
#include "../../vehicle/vehicle_params.h"
#include "apa_ilqr_solver.h"
#include "ilqr_reference_builder.h"

namespace apa_post_processor {
// PIVOT 判据用朝向变化 Δθ（与 ALM 同语义），不用前轮转角差 Δδ
struct iLQRPruneConfig {
    // 剔除弧长阈值 (m)
    double min_arc_length{0.05};
    // PIVOT 朝向变化阈值 (rad)
    double pivot_heading_threshold{0.1};
};
struct iLQRPostStageConfig {
    // 符号游程滞回 (m/s)
    double epsilon_v{0.02};
    // 融化/修剪判据
    iLQRPruneConfig prune{};
    // 驻留窗速度帽 (m/s)
    double v_dwell{0.05};
    // 换挡延迟下限 (s)
    double shift_delay{0.4};
    // 驻留时长安全余量
    double kappa_pad{1.2};
    // 转角速率上限 (rad/s)，必须与求解配置同源
    double omega_max{0.5};
    // 转角加加速度上限 (rad/s²)
    double eta_max{1.0};
    // 接缝 |v| 质量指标（记录不否决）
    double seam_speed_tol{0.02};
    // 驻留窗端点 |ω| 容差：标定为 omega_max+amplitude_check_tol，作为发散探针
    // 真实弯曲参考上优化器以低速滚动出窗完成摆动，本项退化为记录不否决
    double dwell_omega_tol{0.55};
    // v/a 绝对容差（合法性门）
    double amplitude_check_tol{0.05};
    // δ/ω 相对容差 2.1%：与 AL 终止判据 inequality_tol 同包络
    // 旧值 0.2% 比 AL 结构交付严一个数量级，属标定错误
    double amplitude_check_rel_tol{0.021};
    // j/η 盒过冲探针（记录不否决）
    double control_overshoot_tol{0.3};
    // 阶段二跟踪权重地板：深退火时防精化因跟踪过弱脱离热启动邻域
    double stage_two_min_tracking_weight{0.0};
    // true=阶段一末轮跟踪权重已退火到地板时跳过阶段二：深退火
    // 收尾后精化驱动力耗尽，阶段二无收益或负收益；默认 false
    bool skip_stage_two_when_weight_exhausted{false};
    // 校验配置
    TrajectoryValidationConfig validation{};
    iLQRPostStageConfig() { validation.max_terminal_heading_error_deg = 1.5; }
};
// 带滞回符号游程：档位由 sign(v) 恢复（Reeds-Shepp 观点的最终兑现）
struct iLQRSignRun {
    // 运动方向符号
    int sign{0};
    // 起始索引
    std::size_t begin_index{0};
    // 终止索引
    std::size_t end_index{0};
    // 段位移弧长 (m)
    double delta_s{0.0};
    // 朝向变化 (rad)
    double delta_theta{0.0};
};
struct iLQRSeamPlan {
    // 接缝索引
    std::size_t seam_index{0};
    // 窗起始索引
    std::size_t window_begin{0};
    // 窗终止索引
    std::size_t window_end{0};
    // 转向需求 Δδ (rad)
    double delta_delta{0.0};
    // 原地转向最短时间 (s)
    double t_resteer{0.0};
    // 计划驻留时长 (s)
    double t_dwell{0.0};
};
struct iLQRGatingPlanBuild {
    // 门控计划
    iLQRGatingPlan plan;
    std::vector<iLQRSeamPlan> seams;
};
struct iLQRDwellEdit {
    // 窗起始索引
    std::size_t window_begin{0};
    // 窗终止索引
    std::size_t window_end{0};
    // 拉伸后窗口时长 (s)
    double stretched_duration{0.0};
};
struct iLQRSeamReport {
    // 接缝索引
    std::size_t seam_index{0};
    // 转向需求 Δδ (rad)
    double delta_delta{0.0};
    // 原地转向最短时间 (s)
    double t_resteer{0.0};
    // 计划驻留时长 (s)
    double t_dwell{0.0};
    // 实际静止窗时长 (s)
    double dwell_duration{0.0};
    // 接缝点 |v| (m/s)
    double seam_speed{0.0};
    // 窗内 max|v| (m/s)
    double window_max_speed{0.0};
    // 窗端 max|ω| (rad/s)
    double window_end_omega{0.0};
};
// 单项校验量测：判据名 + 实测值 + 阈值 + 是否通过。合法性门与质量指标
// 共用同一记录形态；门逐项全量记录（不短路），供审计与诊断看到全貌
struct iLQRCheckMeasurement {
    // 判据名（如 collision / terminal_position / maneuver_count）
    std::string name;
    // 实测值
    double measured{0.0};
    // 判定阈值
    double threshold{0.0};
    // 是否通过（measured 不超过 threshold，含计数项的不增判据）
    bool passed{false};
};
// 后处理状态码：成功按输出级别区分（阶段二精化 / 阶段一降级），失败
// 语义经状态码上报（任一出口都有安全的回退轨迹）
enum class iLQRPostStageStatus {
    SUCCESS,  // 候选一：阶段二门控精化输出，合法性门全过
    SUCCESS_STAGE_ONE_ONLY,  // 候选二：阶段二不可用或不过门，阶段一解
                             // （经修剪 + 驻留插入）过同一合法性门后输出
    STAGE_ONE_NOT_CONVERGED,  // 阶段一未收敛（不得带病后处理）
    PIVOT_DETECTED,  // 修剪检出 PIVOT（动力学一致解不可能产生的微动）
    PRUNED_PATH_DEGENERATE,   // 修剪后路径退化且降级候选不可用
    STAGE_TWO_NOT_CONVERGED,  // 阶段二未收敛且降级候选不可用
    VALIDATION_FAILED  // 两个候选均未过合法性门（diagnostics 指明哪项）
};
// 结构化诊断：失败项 + 量化程度 + 合法性门/质量指标两层全量量测，
// 供失败回退记录诊断日志与下游方案比较消费
struct iLQRPostStageDiagnostics {
    // 失败项名称（空 = 无失败）
    std::string failed_check;
    // 失败项的量化量测值与判定阈值
    // 失败项量化值
    double measured_value{0.0};
    // 失败项阈值
    double threshold{0.0};
    // 降级原因（仅输出阶段一降级候选或回退时有值）：候选一不可用的
    std::string degraded_reason;
    // 换挡数对比报告：输入 A* maneuver 数 vs 输出物理方向段数
    int input_maneuver_count{0};
    // 输出物理方向段数
    int output_maneuver_count{0};
    // 逐接缝驻留报告（输出候选的量测；回退时为最后评估候选的量测）
    std::vector<iLQRSeamReport> seams;
    // 合法性门全量量测（不短路）：碰撞/终点双指标/运动学四残差/状态幅值
    std::vector<iLQRCheckMeasurement> gate_checks;
    // 质量指标全量量测（记录不否决）：控制盒过冲/接缝与驻留完整性子项/
    // maneuver 数不增/长度比
    std::vector<iLQRCheckMeasurement> metric_checks;
};
// 后处理输出：成功为最终轨迹（含驻留与时间戳，级别由状态码区分），
// 失败为原始 A* 回退轨迹
struct iLQRPostStageResult {
    // 求解状态
    iLQRPostStageStatus status{iLQRPostStageStatus::VALIDATION_FAILED};
    // 成功（含降级成功）：经合法性门校验的最终轨迹；失败：原始 A* 路径
    // 转化的回退轨迹（绝不输出未过合法性门的轨迹）
    Trajectory trajectory;
    // 是否走了原始 A* 回退出口（降级候选输出不算回退）
    bool used_fallback{true};
    // 结构化诊断
    iLQRPostStageDiagnostics diagnostics;
    // 阶段二求解明细（未执行到阶段二时为空）
    std::optional<ApaILQRStageTwoResult> stage_two;
};
// 后处理编排器：游程分析 → 修剪 → 候选一（阶段二重解+驻留）/
// 候选二（阶段一降级），共用合法性门，都不通过才回退
class iLQRPostStage {
   public:
    // 构造时校验滞回/驻留/执行器/修剪参数合法性，注入参考构建器与求解器
    iLQRPostStage(iLQRPostStageConfig config,
                 const iLQRReferenceBuilder* reference_builder,
                 ApaILQRSolver* solver, const VehicleParams& vehicle_params);
    // 主入口：候选一（阶段二）→ 候选二（阶段一降级）→ 原始 A*
    // 回退，输出第一个过合法性门的候选
    iLQRPostStageResult run(const Path& original_path,
                           const iLQRReference& stage_one_reference,
                           const ApaILQRStageOneResult& stage_one_result,
                           const TrajectoryPoint& goal, const ESDFMap& esdf_map,
                           const VehicleFootprintModel& footprint_model);
    // 步骤 1：带滞回符号游程分析——|v|<ε_v 的样本不改变已承诺符号，
    std::vector<iLQRSignRun> analyzeSignRuns(
        const iLQRAlignedVec<iLQRState>& states) const;
    // 由符号游程构建 Maneuver 序列（修剪两遍算法的输入；点携带阶段一
    // 解的 v/a/δ/ω，供转向需求量测消费）
    std::vector<Maneuver> buildManeuvers(
        const iLQRAlignedVec<iLQRState>& states,
        const std::vector<iLQRSignRun>& runs) const;
    // 步骤 2：拓扑修剪红线封装（Δθ 判据的自有分类 + ReconstructPath
    bool pruneManeuvers(std::vector<Maneuver>* maneuvers) const;
    // 步骤 3 准备：由阶段二参考（修剪后重采样网格）与修剪后路径（携带
    iLQRGatingPlanBuild buildGatingPlan(const iLQRReference& stage_two_reference,
                                       const Path& pruned_path) const;
    // 阶段二热启动映射：修剪后路径的逐点阶段一状态量（v/a/δ/ω）按累积
    void buildStageTwoWarmStart(const Path& pruned_path,
                                const iLQRReference& stage_two_reference,
                                iLQRAlignedVec<iLQRState>* warm_states,
                                iLQRAlignedVec<iLQRControl>* warm_controls) const;
    // 步骤 4：驻留插入——逐接缝按阶段二最终轨迹重测 Δδ_j，T_dwell 超过
    Trajectory insertDwells(const iLQRAlignedVec<iLQRState>& states, double dt,
                            const std::vector<iLQRSeamPlan>& seams,
                            std::vector<iLQRSeamReport>* reports) const;
    // 双积分 bang-bang 原地转向最短时间：Δδ ≤ ω²/η 取三角剖面
    static double ComputeResteerTime(double delta_delta, double omega_max,
                                     double eta_max);

   protected:
    // 转向需求量测（构建门控计划）：接缝两侧最后一个 |v|>v_dwell 的
    // 采样点 δ 差；seam_maneuver_index 为接缝左侧 maneuver 下标
    double measureSeamDeltaDelta(const Path& pruned_path,
                                 std::size_t seam_maneuver_index) const;
    // 转向需求量测（驻留插入）：在状态网格上按同一规则重测
    static double MeasureSeamDeltaDeltaFromStates(
        const iLQRAlignedVec<iLQRState>& states, std::size_t seam_index,
        double v_dwell);
    // 候选二（阶段一降级）驻留窗计划：接缝取修剪后保留游程在阶段一
    std::vector<iLQRSeamPlan> buildStageOneSeamPlans(
        const iLQRAlignedVec<iLQRState>& states,
        const std::vector<std::size_t>& seam_indices, double dt) const;
    // 修剪后保留游程的共享边界索引（阶段一状态网格）：相邻两游程均
    std::vector<std::size_t> collectKeptSeamIndices(
        const std::vector<iLQRSignRun>& runs,
        const std::vector<Maneuver>& maneuvers) const;
    // 状态网格点 → 输出轨迹点（κ=tanδ/L 与 θ̇=v·κ 运动学一致）
    TrajectoryPoint stateToPoint(const iLQRState& x, double t) const;
    // 驻留插入的装配阶段：接缝窗外按状态原样转化（时间戳随前方窗口
    // 拉伸量平移），窗内内容按 拉伸时长/窗口原长 线性重定时
    Trajectory assembleRetimedTrajectory(
        const iLQRAlignedVec<iLQRState>& states, double dt,
        const std::vector<iLQRDwellEdit>& edits) const;
    // 回退出口：填充失败状态/诊断 + 原始 A* 路径转化的回退轨迹
    void makeFallback(iLQRPostStageResult* result, iLQRPostStageStatus status,
                      const std::string& failed_check, double measured,
                      double threshold, const Path& original_path) const;
    // 合法性门 + 质量指标两层校验（任一候选共用同一套口径）：
    bool validateOutput(const Trajectory& output,
                        const iLQRAlignedVec<iLQRState>& states,
                        const iLQRAlignedVec<iLQRControl>& controls,
                        double input_length, const TrajectoryPoint& goal,
                        const ESDFMap& esdf_map,
                        const VehicleFootprintModel& footprint_model,
                        iLQRPostStageDiagnostics* diagnostics) const;

   protected:
    // 配置
    iLQRPostStageConfig config_;
    // 参考构建器（不持有所有权，必须非空）
    const iLQRReferenceBuilder* reference_builder_;
    // 求解编排器（不持有所有权，必须非空）
    ApaILQRSolver* solver_;
    // 车辆参数：回退轨迹时间参数化与 κ 反解的依赖
    VehicleParams vehicle_params_;
};
}  // namespace apa_post_processor
