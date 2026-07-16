#pragma once

#include <Eigen/Core>
#include <chrono>
#include <string>
#include <vector>

#include "../spatial/esdf_map.h"
#include "../util/path.h"
#include "../vehicle/vehicle_footprint_model.h"
#include "../vehicle/vehicle_params.h"
#include "adaptive_resampler.h"
#include "bspline_smoother.h"
#include "differential_flatness_solver.h"
#include "speed_profile_planner.h"
#include "static_corridor_builder.h"

namespace apa_post_processor {
// 预处理管线配置：聚合五个阶段配置与过渡开关。
struct PreprocessingPipelineConfig {
    // B 样条平滑器配置
    BSplineSmootherConfig bspline{};
    // 速度规划器配置
    SpeedProfilePlannerConfig speed{};
    // 微分平坦求解器配置
    DifferentialFlatnessSolverConfig diff_flat{};
    // 自适应重采样器配置
    AdaptiveResamplerConfig resampler{};
    // 静态舒适走廊构建器配置
    StaticCorridorBuilderConfig corridor{};
    // 是否构建静态舒适走廊系数（仅影响舒适度偏好，与安全无关）
    bool use_static_corridor = false;
    // 跨阶段统一的碰撞检测数值容差 (m)，自动传播到 bspline.collision_margin
    double collision_safety_margin = 0.0;
    // 调试输出开关：开启后 run() 在各阶段中间产物中填充调试数据
    bool enable_debug_output = false;
};

// 单个机动段中间产物，供调试数据透传使用。
struct PipelineDebugManeuverOutput {
    // B 样条平滑结果
    BSplineSmoother::Result smooth_result;
    // 速度规划结果
    SpeedProfileResult speed_result;
    // 微分平坦补全结果
    DifferentialFlatnessResult diff_flat_result;
    // 该机动段运动方向
    Direction direction{Direction::UNKNOWN};
};

// 预处理管线结果：固定维数打靶点序列、静态走廊系数、时间步长数组与耗时分解。
struct PreprocessingPipelineResult {
    // 是否全部阶段成功完成
    bool success = false;
    // 状态信息
    std::string status_msg;
    // 固定维数最终打靶点序列 Z_ref/U_ref
    std::vector<TrajectoryPoint> z_ref;
    // 时间步长数组 delta_t
    std::vector<double> delta_t;
    // 静态走廊系数 C_matrix * Z <= d_vector
    Eigen::MatrixXd c_matrix;
    Eigen::VectorXd d_vector;
    // 最终固化维度 N_final
    int final_dimension = 0;
    // 原始 Init Path 对比用点序列（由输入 Path 扁平化得到）
    std::vector<TrajectoryPoint> original_z_ref;
    // 本次 run() 实际使用的 margin 与 footprint 外圆行数
    double hard_margin_used = 0.0;
    double soft_margin_used = 0.18;
    int outer_row_num_used = 1;
    // 各阶段耗时分解 (ms)
    double time_bspline_ms = 0.0;
    double time_speed_ms = 0.0;
    double time_diff_flat_ms = 0.0;
    double time_resample_ms = 0.0;
    double time_corridor_ms = 0.0;
    // 端到端总耗时 (ms)
    double time_total_ms = 0.0;
    // 调试输出：各机动段中间产物
    std::vector<PipelineDebugManeuverOutput> debug_maneuver_outputs;
};

// 预处理管线组装器：按 Maneuver 顺序依次调用各阶段类，输出 NMPC 可摄入的
// 固定维数 Z_ref/U_ref、静态走廊系数与 delta_t 序列。
class PreprocessingPipeline {
   public:
    // 使用管线配置、车辆参数、footprint 模型与 ESDF 地图构造管线。
    PreprocessingPipeline(const PreprocessingPipelineConfig& config,
                          const VehicleParams& vehicle_params,
                          const VehicleFootprintModel& footprint_model,
                          const ESDFMap& esdf_map);
    // 对整条 Path 执行预处理管线。
    PreprocessingPipelineResult run(const Path& path,
                                    double initial_velocity = 0.0,
                                    double initial_steer_angle = 0.0) const;
    // 单个机动段中间产物别名。
    using PerManeuverOutput = PipelineDebugManeuverOutput;

    // 由平滑结果构造速度规划输入。
    SpeedProfileInput buildSpeedProfileInput(
        const BSplineSmoother::Result& smooth_result) const;

    // 由平滑结果与速度规划结果构造微分平坦输入。
    DifferentialFlatnessInput buildDifferentialFlatnessInput(
        const BSplineSmoother::Result& smooth_result,
        const SpeedProfileResult& speed_result) const;

    // 由微分平坦结果与平滑结果构造自适应重采样段输入。
    AdaptiveResamplerSegmentInput buildAdaptiveResamplerSegmentInput(
        const BSplineSmoother::Result& smooth_result,
        const DifferentialFlatnessResult& diff_flat_result,
        Direction direction) const;

    // 对单个密集配点遍历全部子圆查询 ESDF，返回最小距离。
    double computeMinEsdfDistAtPoint(
        const BSplineSmoother::DensePointData& dense_point) const;

    // 验证管线配置与输入合法性。
    void validateInputs(const Path& path) const;

   protected:
    PreprocessingPipelineConfig config_;
    VehicleParams vehicle_params_;
    const VehicleFootprintModel& footprint_model_;
    const ESDFMap& esdf_map_;
    BSplineSmoother bspline_smoother_;
    SpeedProfilePlanner speed_planner_;
    DifferentialFlatnessSolver diff_flat_solver_;
    AdaptiveResampler adaptive_resampler_;
    StaticCorridorBuilder corridor_builder_;
};
}  // namespace apa_post_processor
