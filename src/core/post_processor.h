#pragma once

#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "../util/config.h"
#include "../util/path.h"
#include "../util/trajectory.h"
#include "ALM/alm_esdf_penalty.h"
#include "ALM/alm_maneuver_melter.h"
#include "ALM/alm_maneuver_segmenter.h"
#include "ALM/alm_preprocessor.h"
#include "ALM/alm_solver.h"
#include "DDP/apa_ddp_solver.h"
#include "DDP/ddp_post_stage.h"
#include "NMPC/nmpc_config.h"
#include "NMPC/nmpc_solver.h"
#include "NMPC/preprocessing_to_ocp_converter.h"

namespace apa_post_processor {

// 输出级别：描述优化器最终输出的质量等级。
enum class OutputLevel {
    kFallback = 0,   // 回退原始路径（求解失败，降级到 A* 输入）
    kDegraded = 1,   // 降级输出（如 DDP 阶段一解、NMPC 末次迭代）
    kFullSuccess = 2  // 完全成功（如 DDP 阶段二精化解）
};

// 后处理完整链路结果：包含最终路径、状态机、是否使用重试与耗时。
//
// 设计约定（2026-08 重构）：
// - optimized_trajectory + algorithm 是「生产输出层」：无论哪种算法，
//   运行结束后只产生一条最终轨迹，调用方无需按算法名 switch 取不同字段。
// - intermediate_traces 是「诊断/审计层」：收纳预处理中间产物、阶段一
//   中间解等调试用轨迹，按名存取，不再每个中间产物占用独立成员字段。
struct PostProcessorResult {
    // ========== 生产输出层（Phase 1 新增，Phase 3 起为唯一输出入口）==========
    // 本次运行的最终优化轨迹（含时间戳与全部运动学量 v/a/δ/δ̇）
    Trajectory optimized_trajectory;
    // 产出该轨迹的算法标识："nmpc" / "alm" / "ddp"
    std::string algorithm;
    // 输出级别：2=完全成功, 1=降级, 0=回退
    OutputLevel output_level{OutputLevel::kFallback};

    // ========== 诊断/审计层（Phase 2 起填充） ==========
    // 中间产物轨迹集合：按名存取（如 "preprocessed" / "alm_preprocessed" /
    // "ddp_stage_one"），供调试与审计消费；生产路径不依赖此字段
    std::vector<std::pair<std::string, Trajectory>> intermediate_traces;

    // ========== 通用字段 ==========
    // 优化后的路径
    Path optimized_path;
    // 是否成功完成
    bool success = false;
    // 是否使用了自适应间隔重试
    bool used_retry = false;
    // 最终状态消息
    std::string message;
    // 端到端总耗时(ms)
    double total_time_ms = 0.0;
    // 最终物理方向段数（换挡次数）：按扁平化轨迹 v 变号统计（多项式段内
    // 过冲如实计入）；轨迹缺失（失败/回退分支）时回退为 Path 机动段
    // 标签数
    int final_maneuvers = 0;
    // 最终路径长度
    double final_length = 0.0;
};

// 自适应间隔重试配置。
struct AdaptiveRetryConfig {
    // 最大额外重试次数
    int max_retries = 2;
    // dense_step_dist 的乘数序列
    std::vector<double> dense_step_dist_multipliers = {1.0, 2.0};
    // nominal_step_s 的乘数序列
    std::vector<double> nominal_step_s_multipliers = {1.0, 1.0};
    // 是否启用静态走廊的标记序列
    std::vector<bool> use_static_corridor_flags = {false, false};
};

// ALM 路径配置：聚合 ALM 链路各阶段的配置结构体。运动学参数（轴距/最大前轮
// 转角/转角速度/加速度上限）由 PostProcessor 按 VehicleParams 自动派生，
// 保证与车辆物理参数同源，此处置放 ALM 各阶段自身的求解/惩罚/修剪配置。
// 继承通用 Config 基类（与 NMPCConfig 模式一致），供场景层统一管理配置。
struct AlmConfig : public Config {
    // 前端解析与降采样配置
    AlmManeuverSegmenterConfig segmenter;
    // 预处理粗优化配置
    AlmPreprocessorConfig preprocessor;
    // PHR-ALM 主求解器配置
    AlmSolverConfig solver;
    // ESDF 双重安全惩罚配置
    AlmEsdfPenaltyConfig esdf_penalty;
    // 机动融化与拓扑修剪配置
    AlmManeuverMelterConfig melter;
    // 纵向速度上限 (m/s)：VehicleParams 未承载的规划量，ALM 路径独立配置
    double max_velocity = 2.0;
};

// DDP 路径配置：聚合 DDP 链路各阶段的配置结构体。状态幅值边界
// （v_max/a_max/δ_max/ω_max）的唯一权威来源是 reference 子配置、且只允许
// 收紧到车辆物理上限以内——clampToVehicleParams() 把 δ_max/ω_max/a_max 钳到
// VehicleParams 真值（δ_max≤max_steer_angle、ω_max≤max_steer_rate、
// a_max≤min(max_accel,|max_decel|)），η_max 的唯一权威来源是
// solver.inner.steer_accel_max——cost/post_stage 中的同源字段由
// synchronizeAmplitudeBounds() 自动同步（构造与 JSON 加载时各同步一次，
// optimizeDdp 在局部副本上先钳制再同步一次），禁止绕过同步独立改写，否则
// 初值裁剪/AL 约束/驻留窗宽三者数值不自洽（Config 基类字段双源缺口历史
// 教训的前置防御）。继承通用 Config 基类（与 AlmConfig 模式一致），供场景
// 层统一管理配置；基类的代价权重字段为 NMPC 路径专用，DDP 路径不读取
// （DDP 权重全部在 solver.cost 中），基类字段的 JSON 覆盖项（outer_row_num）
// 由 LoadBaseConfigOverrides 统一承载。运动学参数（轴距）由 PostProcessor
// 直接取 VehicleParams，不在此重复配置。
struct DdpConfig : public Config {
    // 构造时同步一次幅值边界，保证默认构造即自洽
    DdpConfig() { synchronizeAmplitudeBounds(); }
    // 把权威来源的幅值边界同步进全部消费方：reference.{v_max, a_max,
    // delta_max, omega_max} → solver.cost 同名字段与 post_stage.omega_max，
    // solver.inner.steer_accel_max → post_stage.eta_max
    void synchronizeAmplitudeBounds() {
        solver.cost.v_max = reference.v_max;
        solver.cost.a_max = reference.a_max;
        solver.cost.delta_max = reference.delta_max;
        solver.cost.omega_max = reference.omega_max;
        post_stage.omega_max = reference.omega_max;
        post_stage.eta_max = solver.inner.steer_accel_max;
    }
    // 以车辆物理参数为上界收紧幅值边界（只收紧、不放宽），
    // 收紧后自动同步进全部消费方（等价于先 clamp 再 sync，消除调用顺序
    // 依赖）。DDP 链路曾长期以 JSON 硬编码值运行在被放大 15%/25% 的假
    // 边界上，输出对真实车辆不可执行而三道校验全部漏检。
    void clampToVehicleParams(const VehicleParams& vehicle_params) {
        reference.delta_max =
            std::min(reference.delta_max, vehicle_params.max_steer_angle);
        reference.omega_max =
            std::min(reference.omega_max, vehicle_params.max_steer_rate);
        reference.a_max = std::min(
            reference.a_max, std::min(vehicle_params.max_accel,
                                      std::abs(vehicle_params.max_decel)));
        synchronizeAmplitudeBounds();
    }
    // 参考构建配置（重采样间距/固定步长/打靶间隔/初值裁剪盒边界）
    DdpReferenceBuilderConfig reference;
    // Reeds-Shepp 换挡点短接配置（默认关闭）：参考构建前允许换挡位姿
    // 本身移动、用有界曲率曲线重连，是唯一能突破「换挡点由前端钉死」
    // 这一天花板的几何前处理，见 ShortcutShiftPoints 的设计注释
    DdpRsShortcutConfig rs_shortcut;
    // 融化开/关双候选择优（L7.2，默认关闭）：启用后同一输入分别以求解
    // 配置原样（融化开）与退火率 ≈1（融化关）各跑一遍完整链路，按
    // 「成功 → maneuver 数少 → 长度短」择优输出（详见
    // PreferControlCandidate）。两个调度候选在四数据集上严格互补
    // （data3/data1 必须开融化、data7 关融化净收益 +7.2% 长度且升
    // 阶段二），择优直接消除「对照解防线」缺口；耗时约 +100%
    bool dual_candidate_select{false};
    // 阶段一/二求解编排配置（内层 MS-iLQR + 外层 AL + 代价求值三层）
    ApaDdpSolverConfig solver;
    // ESDF 双 margin 惩罚配置
    DdpEsdfConstraintConfig esdf;
    // 后处理与阶段二门控精化配置
    DdpPostStageConfig post_stage;
};

// 由算法配置详情 JSON 加载 DdpConfig 专有字段覆盖项：仅覆盖显式出现的字段，
// 未出现的保持构造默认值。JSON 节的组织与 C++ 子配置同构
// （reference/solver.{inner,outer,cost}/esdf/post_stage），但 cost 节不接收
// 幅值边界键、post_stage 节不接收 ω/η 上限键——它们在 JSON 层同样以
// reference/inner 节为单一权威来源，加载完成后经 synchronizeAmplitudeBounds()
// 同步进全部消费方。post_stage.prune/post_stage.validation 两个嵌套配置
// 不在 JSON 映射范围内（工程标定量，代码侧配置）。config 为空指针抛
// std::invalid_argument
void LoadDdpConfigOverrides(const nlohmann::json& details, DdpConfig* config);

// 后处理器：NMPC/ALM/DDP 三条并列后处理链路的完整编排器。
class PostProcessor {
   public:
    // 使用车辆参数、车辆 footprint 模型与 ESDF 地图构造后处理器。
    PostProcessor(const VehicleParams& vehicle_params,
                  const VehicleFootprintModel& footprint_model,
                  const ESDFMap& esdf_map);
    // 执行完整后处理链路。
    PostProcessorResult optimize(
        const Path& init_path, const NMPCConfig& nmpc_config,
        const AdaptiveRetryConfig& retry_config = AdaptiveRetryConfig{}) const;
    // 执行 ALM 后处理链路：Path → AlmManeuverSegmenter → AlmPreprocessor →
    // AlmSolver → AlmManeuverMelter，与 NMPC 路径并列且互不影响（不读取也
    // 不修改任何 NMPC 配置状态）。
    PostProcessorResult optimizeAlm(
        const Path& init_path, const AlmConfig& alm_config = AlmConfig{}) const;
    // 执行 DDP 后处理链路：Path → DdpReferenceBuilder（重采样/初值/打靶
    // 布设）→ ApaDdpSolver 阶段一全局软化求解 → DdpPostStage 后处理与阶段二
    // 门控精化 → 分级候选输出（阶段二精化 → 阶段一降级 → 原始 A* 回退），
    // 与 NMPC/ALM 路径并列且互不影响。两个优化候选共用同一合法性门，触发
    // 原始 A* 回退时 success=false、optimized_path/ddp_traj 为空，message
    // 携带结构化诊断（失败阶段 + 失败项 + 量化值/阈值 + 降级原因）；降级
    // 候选输出时 success=true 且 message 如实反映降级级别
    PostProcessorResult optimizeDdp(
        const Path& init_path, const DdpConfig& ddp_config = DdpConfig{}) const;
    // 双候选择优规则（L7.2）：成功优先 → maneuver 数少优先 → 长度短
    // 优先；完全持平保持融化候选（现状语义）。返回 true = 应选对照
    // （关融化）候选。static 以便独立单测
    static bool PreferControlCandidate(const PostProcessorResult& melt,
                                       const PostProcessorResult& control);

   public:
    // 单次尝试的结果。
    struct AttemptResult {
        // 本次尝试产出的最终路径
        Path optimized_path;
        // NMPC 是否收敛
        bool nmpc_converged = false;
        // NMPC 是否至少返回了非空轨迹
        bool nmpc_had_output = false;
        // 状态消息
        std::string message;
        // 本次尝试总耗时(ms)
        double time_ms = 0.0;
        // 预处理轨迹（带时间戳）
        Trajectory preprocessed_traj;
        // NMPC 轨迹（带时间戳）
        Trajectory nmpc_traj;
    };
    // 用给定配置执行一次预处理 + NMPC 尝试。
    AttemptResult runSingleAttempt(const Path& init_path,
                                   const NMPCConfig& nmpc_config) const;
    // 应用第 retry_idx 次重试的配置。
    static NMPCConfig applyRetryConfig(const NMPCConfig& base_config,
                                       const AdaptiveRetryConfig& retry_config,
                                       int retry_idx);

   protected:
    // DDP 链路的单次完整执行（L7.2 拆分出的单遍实现：配置给定什么就跑
    // 什么，不含双候选编排）；语义与 optimizeDdp 文档一致
    PostProcessorResult optimizeDdpSinglePass(
        const Path& init_path, const DdpConfig& ddp_config) const;
    // DDP 链路的双候选编排（融化开/关两遍择优）；optimizeDdp 在其上
    // 叠加短接失败回退，因此需要把本层单独拆出来重复调用
    PostProcessorResult optimizeDdpDualCandidate(
        const Path& init_path, const DdpConfig& ddp_config) const;
    // 由车辆物理参数派生 ALM 运动学配置：轴距/最大前轮转角/转角速度直接同源，
    // 加速度上限取 max_accel 与 |max_decel| 的较小值（双向约束一致化）。
    static BicycleKinematicsConfig DeriveKinematicsConfig(
        const VehicleParams& vehicle_params, double max_velocity);

    VehicleParams vehicle_params_;
    const VehicleFootprintModel& footprint_model_;
    const ESDFMap& esdf_map_;
};
}  // namespace apa_post_processor
