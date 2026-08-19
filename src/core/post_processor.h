#pragma once

#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "../util/config.h"
#include "../util/path.h"
#include "../util/trajectory.h"
#include "MINCO/minco_config.h"
#include "iLQR/apa_ilqr_solver.h"
#include "iLQR/ilqr_config.h"
#include "iLQR/ilqr_post_stage.h"
#include "NMPC/nmpc_config.h"
#include "NMPC/nmpc_solver.h"
#include "NMPC/preprocessing_to_ocp_converter.h"

namespace apa_post_processor {

// 输出级别：描述优化器最终输出的质量等级。
enum class OutputLevel {
    kFallback = 0,   // 回退原始路径（求解失败，降级到 A* 输入）
    kDegraded = 1,   // 降级输出（如 iLQR 阶段一解、NMPC 末次迭代）
    kFullSuccess = 2  // 完全成功（如 iLQR 阶段二精化解）
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
    // 产出该轨迹的算法标识："nmpc" / "minco" / "ilqr"
    std::string algorithm;
    // 输出级别：2=完全成功, 1=降级, 0=回退
    OutputLevel output_level{OutputLevel::kFallback};

    // ========== 诊断/审计层（Phase 2 起填充） ==========
    // 中间产物轨迹集合：按名存取（如 "preprocessed" / "minco_preprocessed" /
    // "ilqr_stage_one"），供调试与审计消费；生产路径不依赖此字段
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

// 由算法配置详情 JSON 加载 iLQRConfig 专有字段覆盖项：仅覆盖显式出现的字段，
// 未出现的保持构造默认值。JSON 节的组织与 C++ 子配置同构
// （reference/solver.{inner,outer,cost}/esdf/post_stage），但 cost 节不接收
// 幅值边界键、post_stage 节不接收 ω/η 上限键——它们在 JSON 层同样以
// reference/inner 节为单一权威来源，加载完成后经 synchronizeAmplitudeBounds()
// 同步进全部消费方。post_stage.prune/post_stage.validation 两个嵌套配置
// 不在 JSON 映射范围内（工程标定量，代码侧配置）。config 为空指针抛
// std::invalid_argument
void LoadiLQRConfigOverrides(const nlohmann::json& details, iLQRConfig* config);

// 由算法配置详情 JSON 加载 MincoConfig 专有字段覆盖项：仅覆盖显式出现的
// 字段，未出现的保持构造默认值。JSON 节与 MincoConfig 扁平字段同名同构
// （当前映射 esdf 节的 margin_safe/margin_comf/weight_safe/weight_comf），
// 后续新增字段按同模式在函数内登记即可。config 为空指针抛
// std::invalid_argument
void LoadMincoConfigOverrides(const nlohmann::json& details,
                              MincoConfig* config);

// 后处理器：NMPC/MINCO/iLQR 三条并列后处理链路的完整编排器。
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
    // 执行 MINCO 后处理链路：Path → MincoManeuverSegmenter → MincoPreprocessor →
    // MincoSolver → MincoManeuverMelter，与 NMPC 路径并列且互不影响（不读取也
    // 不修改任何 NMPC 配置状态）。
    PostProcessorResult optimizeMinco(
        const Path& init_path, const MincoConfig& minco_config = MincoConfig{}) const;
    // 执行 iLQR 后处理链路：Path → iLQRReferenceBuilder（重采样/初值/打靶
    // 布设）→ ApaILQRSolver 阶段一全局软化求解 → iLQRPostStage 后处理与阶段二
    // 门控精化 → 分级候选输出（阶段二精化 → 阶段一降级 → 原始 A* 回退），
    // 与 NMPC/MINCO 路径并列且互不影响。两个优化候选共用同一合法性门，触发
    // 原始 A* 回退时 success=false、optimized_path/ilqr_traj 为空，message
    // 携带结构化诊断（失败阶段 + 失败项 + 量化值/阈值 + 降级原因）；降级
    // 候选输出时 success=true 且 message 如实反映降级级别
    PostProcessorResult optimizeiLQR(
        const Path& init_path, const iLQRConfig& ilqr_config = iLQRConfig{}) const;
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
    // iLQR 链路的单次完整执行（L7.2 拆分出的单遍实现：配置给定什么就跑
    // 什么，不含双候选编排）；语义与 optimizeiLQR 文档一致
    PostProcessorResult optimizeiLQRSinglePass(
        const Path& init_path, const iLQRConfig& ilqr_config) const;
    // iLQR 链路的双候选编排（融化开/关两遍择优）；optimizeiLQR 在其上
    // 叠加短接失败回退，因此需要把本层单独拆出来重复调用
    PostProcessorResult optimizeiLQRDualCandidate(
        const Path& init_path, const iLQRConfig& ilqr_config) const;
    // 由车辆物理参数派生 MINCO 运动学配置：在传入配置副本上覆盖轴距/最大前轮
    // 转角/转角速度（直接同源）、加速度上限取 max_accel 与 |max_decel| 的较小
    // 值（双向约束一致化），返回派生后的完整配置。
    static MincoConfig DeriveKinematicsConfig(
        const VehicleParams& vehicle_params, const MincoConfig& minco_config);

    VehicleParams vehicle_params_;
    const VehicleFootprintModel& footprint_model_;
    const ESDFMap& esdf_map_;
};
}  // namespace apa_post_processor
