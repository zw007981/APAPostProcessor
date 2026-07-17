#pragma once

#include <string>
#include <vector>

#include "../util/path.h"
#include "../util/trajectory.h"
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
    // 最终机动段数
    int final_maneuvers = 0;
    // 最终路径长度
    double final_length = 0.0;
    // 预处理产出的轨迹（带时间戳）
    Trajectory preprocessed_traj;
    // NMPC 优化产出的轨迹（带时间戳）
    Trajectory nmpc_traj;
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
    VehicleParams vehicle_params_;
    const VehicleFootprintModel& footprint_model_;
    const ESDFMap& esdf_map_;
};
}  // namespace apa_post_processor
