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
// 融化/修剪判据配置：PIVOT 判据量与 ALM 侧 AlmManeuverMelterConfig 同一
// 物理含义（朝向变化 Δθ），不再借用 TopologyCleanupConfig 的前轮转角差
// Δδ 语义——Δδ 大只说明方向盘在换挡点附近大幅摆动（实测正常 maneuver 的
// 首末 Δδ 达 0.65~0.79 rad），与原地掉头式微动无关；而动力学一致解在
// 微弧长游程内 |Δθ| 受 θ̇=v·tanδ/L 上界约束，不可能逼近阈值
struct DdpPruneConfig {
    // 剔除弧长阈值 (m)：低于该弧长的中间游程判为融化残余予以剔除。
    // 取一个重采样间距（与 countDirectionRuns 的位移过滤口径一致）；
    // 四数据集实测收敛阶段一解的非保护游程弧长均 >= 0.91 m，融化残余
    // 在滞回游程分析层已被吸收，本判据作为安全网存在
    double min_arc_length{0.05};
    // PIVOT 判定的朝向变化阈值 (rad)：极小弧长但 |Δθ| 超过此值判为原地
    // 掉头式微动。动力学一致解在爬行速度下微弧长游程的 |Δθ| 上限约为
    // tanδ_max/L·Δs（远小于本阈值），检出即说明解携带未愈合缺陷或收敛
    // 声明不实
    double pivot_heading_threshold{0.1};
};
// 后处理与阶段二门控精化配置：符号游程滞回、融化/修剪判据（DDP 自有
// Δθ 语义）、驻留参数与校验容差。默认值取设计文档 DDP.md 2.5 节参数表，
// 最终标定属端到端调参
struct DdpPostStageConfig {
    // 符号游程滞回阈值 ε_v (m/s)：|v|<ε_v 的样本不计入任何游程
    double epsilon_v{0.02};
    // 融化/修剪判据（极小弧长 0.05 m / PIVOT 朝向变化 0.1 rad）
    DdpPruneConfig prune{};
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
    // 质量指标参考阈值：接缝零速 |v| (m/s)。阶段二收敛解的接缝速度由
    // 接缝等式门控结构保证（实测 ≤0.005），本项度量换挡质量而非合法性
    double seam_speed_tol{0.02};
    // 质量指标参考阈值：驻留窗端点 |ω| (rad/s)，标定为 omega_max +
    // amplitude_check_tol（与状态幅值门共用同一 AL 平衡包络）。设计文档
    // "摆动完整排入静止窗、窗端 ω≈0" 的前提在真实弯曲参考几何上不成立：
    // 接缝两侧参考连续弯曲时优化器以低速滚动出窗完成残余摆动（残余摆角
    // ~ω_e²/(2η)，在蠕动速度内完成），四数据集实测窗端 |ω| 最高 0.50；
    // 轨迹可执行性由状态幅值门的 |ω|≤ω_max、|η|≤η_max 硬限独立保证，
    // 本项退化为求解器发散级探针（记录不否决）
    double dwell_omega_tol{0.55};
    // 合法性门容差：状态幅值 v/a 复检的绝对容差（AL 平衡包络）。δ/ω 不
    // 走本容差（见 amplitude_check_rel_tol）——0.05 的绝对容差在 δ 量纲上
    // 等价于允许 κ=113%·κ_max，比验收门还松；v/a 直接进入输出轨迹契约，
    // 本项是合法性门、不得放宽
    double amplitude_check_tol{0.05};
    // 合法性门容差：δ/ω 复检的相对容差（相对各自幅值上限的比例）。
    // δ 只在行驶点（|v|≥v_dwell）复检——低速/驻留点的 δ 不产生曲率，
    // 与「停稳后前轮转角无物理要求」同一设计语义；ω 全点复检（执行器
    // 速率在驻留转向中同样工作）。默认 0.2%：数值容差带，吸收 AL 平衡
    // 残余与浮点噪声，不是工程让步
    double amplitude_check_rel_tol{0.002};
    // 质量指标参考阈值：控制量 j/η 盒过冲。前向滚动按设计不截断控制
    // （截断会破坏下降方向，盒约束由后向传递的 box-QP 精确处理），最终
    // 控制序列可经反馈项 K(x'−x̄) 与终端静止等式收紧的边界层产生有限
    // 盒过冲（实测已收敛解的过冲集中出现在最后一个控制步）；j/η 不进入
    // 输出轨迹契约，物理可执行性由状态幅值门独立保证，本项作为求解器
    // 发散级探针记录（不否决）
    double control_overshoot_tol{0.3};
    // 阶段二跟踪权重地板 (0 = 不启用)：阶段二跟踪权重冻结在阶段一末轮
    // 退火值是既有约定（强跟踪重置会与终端罚权重形成失衡平衡）；但深
    // 退火调度下该末轮值可能远低于精化所需的保持量级（如 γ=0.3 时
    // 0.002），地板把阶段二权重钳到 max(末轮值, 地板)——避免精化因
    // 跟踪过弱脱离热启动邻域、门控幅值约束失稳
    double stage_two_min_tracking_weight{0.0};
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
// 单个接缝的驻留插入报告：驻留质量指标与诊断日志的数据来源
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
// 单项校验量测：判据名 + 实测值 + 阈值 + 是否通过。合法性门与质量指标
// 共用同一记录形态；门逐项全量记录（不短路），供审计与诊断看到全貌
struct DdpCheckMeasurement {
    // 判据名（如 collision / terminal_position / maneuver_count）
    std::string name;
    // 实测值与判定阈值（同一量纲；maneuver_count 等计数项按 double 记录）
    double measured{0.0};
    double threshold{0.0};
    // 是否通过（measured 不超过 threshold，含计数项的不增判据）
    bool passed{false};
};
// 后处理状态码：成功按输出级别区分（阶段二精化 / 阶段一降级），失败
// 语义经状态码上报（任一出口都有安全的回退轨迹）
enum class DdpPostStageStatus {
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
struct DdpPostStageDiagnostics {
    // 失败项名称（空 = 无失败）
    std::string failed_check;
    // 失败项的量化量测值与判定阈值
    double measured_value{0.0};
    double threshold{0.0};
    // 降级原因（仅输出阶段一降级候选或回退时有值）：候选一不可用的
    // 原因——pruned_path_degenerate / stage_two_convergence /
    // stage_two_gate:<失败项>；候选一输出时为空
    std::string degraded_reason;
    // 换挡数对比报告：输入 A* maneuver 数 vs 输出物理方向段数
    int input_maneuver_count{0};
    int output_maneuver_count{0};
    // 逐接缝驻留报告（输出候选的量测；回退时为最后评估候选的量测）
    std::vector<DdpSeamReport> seams;
    // 合法性门全量量测（不短路）：碰撞/终点双指标/运动学四残差/状态幅值
    std::vector<DdpCheckMeasurement> gate_checks;
    // 质量指标全量量测（记录不否决）：控制盒过冲/接缝与驻留完整性子项/
    // maneuver 数不增/长度比
    std::vector<DdpCheckMeasurement> metric_checks;
};
// 后处理输出：成功为最终轨迹（含驻留与时间戳，级别由状态码区分），
// 失败为原始 A* 回退轨迹
struct DdpPostStageResult {
    DdpPostStageStatus status{DdpPostStageStatus::VALIDATION_FAILED};
    // 成功（含降级成功）：经合法性门校验的最终轨迹；失败：原始 A* 路径
    // 转化的回退轨迹（绝不输出未过合法性门的轨迹）
    Trajectory trajectory;
    // 是否走了原始 A* 回退出口（降级候选输出不算回退）
    bool used_fallback{true};
    DdpPostStageDiagnostics diagnostics;
    // 阶段二求解明细（未执行到阶段二时为空）
    std::optional<ApaDdpStageTwoResult> stage_two;
};
// 后处理与阶段二门控精化编排器（设计文档 DDP.md 2.6 节）：带滞回符号
// 游程分析 → 拓扑修剪（Δθ 判据，红线逐条继承：绝不合并反向相邻段、
// 首/末段受保护、PIVOT 即失败）→ 分级候选输出——候选一（阶段二门控
// 重解 + 驻留插入，必须重解禁止直接拼接）不过则候选二（阶段一解 +
// 修剪 + 驻留插入），两个候选共用同一套合法性门（碰撞/终点双指标/
// 运动学/状态幅值，不得放宽），质量指标只记录不否决；两个候选都不过
// 门才回退原始 A* 路径 + 结构化诊断。
class DdpPostStage {
   public:
    // 构造校验：参考构建器/求解器必须非空（不持有所有权），滞回阈值/
    // 驻留参数/执行器限值/修剪判据/容差必须为正且自洽，非法输入抛
    // std::invalid_argument
    DdpPostStage(DdpPostStageConfig config,
                 const DdpReferenceBuilder* reference_builder,
                 ApaDdpSolver* solver, const VehicleParams& vehicle_params);
    // 阶段二专用求解器注入（L6.3a，可选）：设置后阶段二门控精化改用该
    // 求解器（其代价求值层可按阶段二独立标定的 ESDF 装配），阶段一相关
    // 路径仍用构造时的求解器；nullptr 或从不调用 = 与阶段一共用
    void setStageTwoSolver(ApaDdpSolver* solver) { stage_two_solver_ = solver; }
    // 主入口：分级候选结构——按优先级依次尝试候选一（阶段二精化）与
    // 候选二（阶段一降级），输出第一个通过合法性门的候选；都不过才
    // 回退原始 A* 路径并记录完整诊断（失败阶段、量化违反度、降级原因）
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
    // 解的 v/a/δ/ω，供转向需求量测消费）
    std::vector<Maneuver> buildManeuvers(
        const DdpAlignedVec<DdpState>& states,
        const std::vector<DdpSignRun>& runs) const;
    // 步骤 2：拓扑修剪红线封装（Δθ 判据的自有分类 + ReconstructPath
    // 重建，不重复实现剔除/合并）——首/末 maneuver 无论判据量如何均不
    // 参与剔除与 PIVOT 重分类；分类只打方向标签（UNKNOWN 剔除 / PIVOT
    // 标记），绝不改写采样点数据（压平位置/清零速度会产生 v≡0 但 θ
    // 变化的自相矛盾状态）；返回 false = 检出 PIVOT（动力学一致解在
    // 微弧长游程内 |Δθ| 不可能超阈，检出即说明解携带未愈合缺陷，按
    // 求解失败语义处理）
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
    // 放慢重定时，可行性严格保持，不改空间剖面），时间戳线性重排
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
    // 转向需求量测（驻留插入）：在状态网格上按同一规则重测
    static double MeasureSeamDeltaDeltaFromStates(
        const DdpAlignedVec<DdpState>& states, std::size_t seam_index,
        double v_dwell);
    // 候选二（阶段一降级）驻留窗计划：接缝取修剪后保留游程在阶段一
    // 状态网格上的共享边界，窗口定宽公式与门控计划同源
    // m_j=⌈max(T_resteer,T_shift)/(2dt)⌉，裁剪到 [0,N] 且不跨相邻接缝
    std::vector<DdpSeamPlan> buildStageOneSeamPlans(
        const DdpAlignedVec<DdpState>& states,
        const std::vector<std::size_t>& seam_indices, double dt) const;
    // 修剪后保留游程的共享边界索引（阶段一状态网格）：相邻两游程均
    // 保留时其共享边界即一个真实换挡接缝；被剔除游程两侧的同向邻居
    // 合并后不产生接缝（融化语义：假换挡不驻留）
    std::vector<std::size_t> collectKeptSeamIndices(
        const std::vector<DdpSignRun>& runs,
        const std::vector<Maneuver>& maneuvers) const;
    // 状态网格点 → 输出轨迹点（κ=tanδ/L 与 θ̇=v·κ 运动学一致）
    TrajectoryPoint stateToPoint(const DdpState& x, double t) const;
    // 驻留插入的装配阶段：接缝窗外按状态原样转化（时间戳随前方窗口
    // 拉伸量平移），窗内内容按 拉伸时长/窗口原长 线性重定时
    Trajectory assembleRetimedTrajectory(
        const DdpAlignedVec<DdpState>& states, double dt,
        const std::vector<DdpDwellEdit>& edits) const;
    // 回退出口：填充失败状态/诊断 + 原始 A* 路径转化的回退轨迹
    void makeFallback(DdpPostStageResult* result, DdpPostStageStatus status,
                      const std::string& failed_check, double measured,
                      double threshold, const Path& original_path) const;
    // 合法性门 + 质量指标两层校验（任一候选共用同一套口径）：
    // 合法性门 = 碰撞/终点双指标/运动学四残差/状态幅值（v/a/δ/ω，
    // 输出轨迹契约直接消费的量），任一项不过即不可输出，不得放宽；
    // 质量指标 = 控制盒过冲/接缝与驻留完整性子项/maneuver 数不增/
    // 长度比，全量记录进 diagnostics 供方案比较，不作为回退触发条件。
    // 两层均逐项全量量测（不短路）；返回合法性门是否通过，不过时把
    // 首个失败门项填入 diagnostics.failed_check
    bool validateOutput(const Trajectory& output,
                        const DdpAlignedVec<DdpState>& states,
                        const DdpAlignedVec<DdpControl>& controls,
                        double input_length, const TrajectoryPoint& goal,
                        const ESDFMap& esdf_map,
                        const VehicleFootprintModel& footprint_model,
                        DdpPostStageDiagnostics* diagnostics) const;

   protected:
    DdpPostStageConfig config_;
    // 参考构建器与求解编排器（不持有所有权，必须非空）
    const DdpReferenceBuilder* reference_builder_;
    ApaDdpSolver* solver_;
    // 阶段二专用求解器（可选，不持有所有权）：为空时阶段二与阶段一共用
    ApaDdpSolver* stage_two_solver_{nullptr};
    // 车辆参数：回退轨迹时间参数化与 κ 反解的依赖
    VehicleParams vehicle_params_;
};
}  // namespace apa_post_processor
