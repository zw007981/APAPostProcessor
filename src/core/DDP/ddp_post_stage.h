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
#include "apa_ddp_solver.h"
#include "ddp_reference_builder.h"

namespace apa_post_processor {
// 后处理与阶段二门控精化配置：符号游程滞回、修剪判据（复用
// util::topology_cleaner 的配置类型与默认阈值）、驻留参数与校验容差。
// 默认值取设计文档 DDP.md 2.5 节参数表，最终标定属端到端调参
struct DdpPostStageConfig {
    // 符号游程滞回阈值 ε_v (m/s)：|v|<ε_v 的样本不计入任何游程
    double epsilon_v{0.02};
    // 拓扑修剪判据（极小弧长 0.05 m / PIVOT 前轮转角差 0.1 rad）
    TopologyCleanupConfig cleanup{};
    // 驻留窗速度帽 v_dwell (m/s)
    double v_dwell{0.05};
    // 换挡执行器延迟下限 T_shift (s)：驻留时长与窗口宽度的下限
    double shift_delay{0.4};
    // 驻留时长安全余量系数 κ_pad：T_dwell=κ_pad·max(T_resteer,T_shift)
    double kappa_pad{1.2};
    // T_resteer 双积分公式参数：转角速度限 ω_max (rad/s) 与转角加加速度
    // 限 η_max (rad/s²)，必须与求解配置同源
    double omega_max{0.5};
    double eta_max{1.0};
    // 校验容差：接缝零速 |v| 上限 (m/s)
    double seam_speed_tol{0.02};
    // 校验容差：驻留窗端点 |ω| 上限 (rad/s)（超阈说明窗口定宽不足）
    double dwell_omega_tol{0.1};
    // 校验容差：状态/控制幅值复检的线性违反度上限
    double amplitude_check_tol{0.05};
    // 校验容差：控制量 j/η 盒过冲的专项上限。前向滚动按设计不截断控制
    // （截断会破坏下降方向，盒约束由后向传递的 box-QP 精确处理），最终
    // 控制序列可经反馈项 K(x'−x̄) 产生有限盒过冲——典型出现在终点静止
    // 等式收紧的边界层（实测最难合成场景约 0.23）。该容差拦截的是求解
    // 器发散级的严重超限，平衡态过冲由下游执行限幅兜底
    double control_overshoot_tol{0.3};
    // 轨迹三门校验配置：构造时把终点朝向容差从通用默认 3.0° 收紧到
    // 1.5°（终点双指标硬指标），碰撞深度 0.02 m 与生产质量门同口径。
    // 构造函数只建立默认值——调用方在构造之后修改本字段（含
    // max_terminal_heading_error_deg）即以其修改值生效，不会被再次覆写
    TrajectoryValidationConfig validation{};
    // 构造时收紧终点朝向容差（双指标 0.05 m/1.5°）
    DdpPostStageConfig() { validation.max_terminal_heading_error_deg = 1.5; }
};
// 带滞回符号游程：同一运动方向下的最大连续样本段（Reeds-Shepp 观点的
// 最终兑现——档位由 sign(v) 恢复，而非优化变量）
struct DdpSignRun {
    // 运动方向符号：+1 前进 / -1 倒退 / 0 全程未决（|v| 恒低于滞回阈值）
    int sign{0};
    // 在状态网格上的起/止索引（含端点；相邻游程共享边界点索引）
    std::size_t begin_index{0};
    std::size_t end_index{0};
    // 段位移弧长（无符号幅值）(m)
    double delta_s{0.0};
    // 朝向变化（已 wrap 到 (-π, π]）(rad)
    double delta_theta{0.0};
};
// 单个接缝的门控计划数据：窗口边界与阶段一输出测得的转向需求
struct DdpSeamPlan {
    // 接缝在阶段二网格上的节点索引
    std::size_t seam_index{0};
    // 驻留窗 [begin, end]（阶段二网格索引，含端点；裁剪到 [0,N] 且不跨
    // 相邻接缝，保证逐接缝处理互不重叠）
    std::size_t window_begin{0};
    std::size_t window_end{0};
    // 转向需求 Δδ_j = |δ_right−δ_left|（阶段一输出接缝前后最后一个
    // |v|>v_dwell 采样点处量取）(rad)
    double delta_delta{0.0};
    // 双积分 bang-bang 原地转向最短时间 (s)
    double t_resteer{0.0};
    // 计划驻留时长 κ_pad·max(T_resteer,T_shift) (s)
    double t_dwell{0.0};
};
// 门控计划构建结果：求值层消费的计划本体 + 驻留插入消费的接缝数据
struct DdpGatingPlanBuild {
    DdpGatingPlan plan;
    std::vector<DdpSeamPlan> seams;
};
// 单接缝的驻留拉伸编辑计划：驻留插入的"量测/计划"阶段产物，
// 供"装配"阶段一次性消费
struct DdpDwellEdit {
    // 驻留窗 [begin, end]（阶段二网格索引，含端点）
    std::size_t window_begin{0};
    std::size_t window_end{0};
    // 拉伸后窗口时长（= max(窗口原长, 量化后的 T_dwell)；0 = 退化窗口
    // 不拉伸、原样透传）(s)
    double stretched_duration{0.0};
};
// 单个接缝的驻留插入报告：校验清单第⑤项与诊断日志的数据来源
struct DdpSeamReport {
    // 接缝在阶段二网格上的节点索引
    std::size_t seam_index{0};
    // 阶段二最终轨迹重测的转向需求 (rad)
    double delta_delta{0.0};
    // 重测 T_resteer / 最终驻留时长目标 (s)
    double t_resteer{0.0};
    double t_dwell{0.0};
    // 实际静止窗时长（时间拉伸后，≥ T_dwell）(s)
    double dwell_duration{0.0};
    // 阶段二解在接缝点的 |v| (m/s)
    double seam_speed{0.0};
    // 阶段二解在驻留窗内的 max|v| (m/s)
    double window_max_speed{0.0};
    // 阶段二解在驻留窗两端点的 max|ω| (rad/s)
    double window_end_omega{0.0};
};
// 后处理状态码：失败语义经状态码上报（任一出口都有安全的回退轨迹）
enum class DdpPostStageStatus {
    SUCCESS,                  // 六项校验全部通过
    STAGE_ONE_NOT_CONVERGED,  // 阶段一未收敛（不得带病后处理）
    PIVOT_DETECTED,           // 修剪检出 PIVOT（按求解失败语义）
    PRUNED_PATH_DEGENERATE,  // 修剪后路径退化（无法重建阶段二参考）
    STAGE_TWO_NOT_CONVERGED,  // 阶段二门控重解未收敛
    VALIDATION_FAILED  // 六项校验某项不过（diagnostics 指明哪项）
};
// 结构化诊断：哪一项失败 + 量化程度，供失败回退记录诊断日志
struct DdpPostStageDiagnostics {
    // 失败项名称（空 = 无失败）
    std::string failed_check;
    // 失败项的量化量测值与判定阈值
    double measured_value{0.0};
    double threshold{0.0};
    // 换挡数对比报告：输入 A* maneuver 数 vs 输出物理方向段数
    int input_maneuver_count{0};
    int output_maneuver_count{0};
    // 逐接缝驻留报告
    std::vector<DdpSeamReport> seams;
};
// 后处理输出：成功为最终轨迹（含驻留与时间戳），失败为原始 A* 回退轨迹
struct DdpPostStageResult {
    DdpPostStageStatus status{DdpPostStageStatus::VALIDATION_FAILED};
    // 成功：阶段二重解 + 驻留插入后的最终轨迹；失败：原始 A* 路径转化的
    // 回退轨迹（绝不输出半成品轨迹）
    Trajectory trajectory;
    // 是否走了回退出口
    bool used_fallback{true};
    DdpPostStageDiagnostics diagnostics;
    // 阶段二求解明细（未执行到阶段二时为空）
    std::optional<ApaDdpStageTwoResult> stage_two;
};
// 后处理与阶段二门控精化编排器（设计文档 DDP.md 2.6 节全部六步）：
// 带滞回符号游程分析 → 拓扑修剪（复用 util::topology_cleaner 两遍算法，
// 红线逐条继承：绝不合并反向相邻段、首/末段受保护、PIVOT 即失败）→
// 阶段二门控重解（必须重解，禁止直接拼接）→ 逐接缝驻留插入（时间拉伸，
// 不改空间剖面）→ 六项校验清单 → 失败回退原始 A* 路径 + 结构化诊断。
class DdpPostStage {
   public:
    // 构造校验：参考构建器/求解器必须非空（不持有所有权），滞回阈值/
    // 驻留参数/执行器限值/容差必须为正且自洽，非法输入抛
    // std::invalid_argument
    DdpPostStage(DdpPostStageConfig config,
                 const DdpReferenceBuilder* reference_builder,
                 ApaDdpSolver* solver, const VehicleParams& vehicle_params);
    // 六步主入口：任一硬校验不过或求解失败均回退原始 A* 路径并记录完整
    // 诊断（失败阶段、量化违反度），保证任何输入下都有安全输出
    DdpPostStageResult run(const Path& original_path,
                           const DdpReference& stage_one_reference,
                           const ApaDdpStageOneResult& stage_one_result,
                           const TrajectoryPoint& goal, const ESDFMap& esdf_map,
                           const VehicleFootprintModel& footprint_model);
    // 步骤 1：带滞回符号游程分析——|v|<ε_v 的样本不改变已承诺符号，
    // 真实换向（反向突破阈值）才切分新游程；游程边界共享（前段终点
    // 索引 = 后段起点索引）
    std::vector<DdpSignRun> analyzeSignRuns(
        const DdpAlignedVec<DdpState>& states) const;
    // 由符号游程构建 Maneuver 序列（修剪两遍算法的输入；点携带阶段一
    // 解的 v/a/δ/ω，供 PIVOT 判据与转向需求量测消费）
    std::vector<Maneuver> buildManeuvers(
        const DdpAlignedVec<DdpState>& states,
        const std::vector<DdpSignRun>& runs) const;
    // 步骤 2：拓扑修剪红线封装（复用 ClassifyAndResetManeuvers 分类，
    // 剔除/合并交由 ReconstructPath 完成，不重复实现）——首/末 maneuver
    // 无论判据量如何均不参与剔除与 PIVOT 重分类；返回 false = 检出
    // PIVOT（默认车辆无钟摆泊车能力，按求解失败语义处理）
    bool pruneManeuvers(std::vector<Maneuver>* maneuvers) const;
    // 步骤 3 准备：由阶段二参考（修剪后重采样网格）与修剪后路径（携带
    // 阶段一 v/δ）构建门控计划：段内符号门、接缝零速等式、逐接缝
    // m_j=⌈max(T_resteer(Δδ_j),T_shift)/(2dt)⌉ 的驻留窗
    DdpGatingPlanBuild buildGatingPlan(const DdpReference& stage_two_reference,
                                       const Path& pruned_path) const;
    // 阶段二热启动映射：修剪后路径的逐点阶段一状态量（v/a/δ/ω）按累积
    // 弧长查表线性插值到重采样网格（位姿取参考位姿本身），控制量由
    // a/ω 差分反解并裁剪进盒——精化的起点是阶段一收敛解本身，而不是
    // 前端初值提取的 bang 速度剖面（后者 v_N≠0 与静止终端矛盾，会把
    // 精化拖成一次完整重解）
    void buildStageTwoWarmStart(const Path& pruned_path,
                                const DdpReference& stage_two_reference,
                                DdpAlignedVec<DdpState>* warm_states,
                                DdpAlignedVec<DdpControl>* warm_controls) const;
    // 步骤 4：驻留插入——逐接缝按阶段二最终轨迹重测 Δδ_j，T_dwell 超过
    // 窗口时长时对窗内内容做线性时间拉伸（v/ω 同比减小、δ 摆动剖面
    // 放慢重定时，可行性严格保持，不改空间路径），时间戳线性重排
    Trajectory insertDwells(const DdpAlignedVec<DdpState>& states, double dt,
                            const std::vector<DdpSeamPlan>& seams,
                            std::vector<DdpSeamReport>* reports) const;
    // 双积分 bang-bang 原地转向最短时间：Δδ ≤ ω²/η 取三角剖面
    // 2√(Δδ/η)（ω 不饱和），否则取梯形剖面 Δδ/ω+ω/η（两分支在切换点
    // 连续）；Δδ<0 或执行器限值非正抛 std::invalid_argument
    static double ComputeResteerTime(double delta_delta, double omega_max,
                                     double eta_max);

   protected:
    // 转向需求量测（构建门控计划）：接缝两侧最后一个 |v|>v_dwell 的
    // 采样点 δ 差；seam_maneuver_index 为接缝左侧 maneuver 下标
    double measureSeamDeltaDelta(const Path& pruned_path,
                                 std::size_t seam_maneuver_index) const;
    // 转向需求量测（驻留插入）：在阶段二状态网格上按同一规则重测
    static double MeasureSeamDeltaDeltaFromStates(
        const DdpAlignedVec<DdpState>& states, std::size_t seam_index,
        double v_dwell);
    // 状态网格点 → 输出轨迹点（κ=tanδ/L 与 θ̇=v·κ 运动学一致）
    TrajectoryPoint stateToPoint(const DdpState& x, double t) const;
    // 驻留插入的装配阶段：接缝窗外按阶段二状态原样转化（时间戳随前方
    // 窗口拉伸量平移），窗内内容按 拉伸时长/窗口原长 线性重定时
    Trajectory assembleRetimedTrajectory(
        const DdpAlignedVec<DdpState>& states, double dt,
        const std::vector<DdpDwellEdit>& edits) const;
    // 回退出口：填充失败状态/诊断 + 原始 A* 路径转化的回退轨迹
    void makeFallback(DdpPostStageResult* result, DdpPostStageStatus status,
                      std::string failed_check, double measured,
                      double threshold, const Path& original_path) const;
    // 步骤 5：六项校验清单（碰撞/终点双指标/运动学三门 + 控制幅值 +
    // 接缝零速与驻留完整性 + maneuver 数不增）；失败时把首项失败填入
    // diagnostics 并返回 false
    bool validateOutput(const Trajectory& output,
                        const ApaDdpStageTwoResult& stage_two,
                        const TrajectoryPoint& goal, const ESDFMap& esdf_map,
                        const VehicleFootprintModel& footprint_model,
                        DdpPostStageDiagnostics* diagnostics) const;

   protected:
    DdpPostStageConfig config_;
    // 参考构建器与求解编排器（不持有所有权，必须非空）
    const DdpReferenceBuilder* reference_builder_;
    ApaDdpSolver* solver_;
    // 车辆参数：回退轨迹时间参数化与 κ 反解的依赖
    VehicleParams vehicle_params_;
};
}  // namespace apa_post_processor
