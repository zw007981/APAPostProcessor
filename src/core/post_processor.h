#pragma once

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
#include "NMPC/nmpc_config.h"
#include "NMPC/nmpc_solver.h"
#include "NMPC/preprocessing_to_ocp_converter.h"

namespace apa_post_processor {
// 后处理完整链路结果：包含最终路径、状态机、是否使用重试与耗时。
struct PostProcessorResult {
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
    // 预处理产出的轨迹（带时间戳）
    Trajectory preprocessed_traj;
    // NMPC 优化产出的轨迹（带时间戳）
    Trajectory nmpc_traj;
    // ALM 优化产出的轨迹（采样点携带 θ-s 轨迹全局时刻）
    Trajectory alm_traj;
    // ALM 预处理粗优化产出的轨迹（采样点携带 θ-s 轨迹全局时刻）：与
    // alm_traj 经同一套离散化管线产出，作为"优化前/优化后"对比的优化前
    // 基线；仅 ALM 路径填充
    Trajectory alm_preprocessed_traj;
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

// 后处理器：Path → PreprocessingPipeline → NmpcSolver 完整链路。
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
    // 由车辆物理参数派生 ALM 运动学配置：轴距/最大前轮转角/转角速度直接同源，
    // 加速度上限取 max_accel 与 |max_decel| 的较小值（双向约束一致化）。
    static BicycleKinematicsConfig DeriveKinematicsConfig(
        const VehicleParams& vehicle_params, double max_velocity);

    VehicleParams vehicle_params_;
    const VehicleFootprintModel& footprint_model_;
    const ESDFMap& esdf_map_;
};
}  // namespace apa_post_processor
