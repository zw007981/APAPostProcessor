#pragma once

#include <vector>

#include "../util/maneuver.h"
#include "../util/trajectory_point.h"
#include "../vehicle/vehicle_params.h"
#include "bspline_smoother.h"

namespace apa_post_processor {
// 自适应重采样器配置。
struct AdaptiveResamplerConfig {
    // 预分配内存池上限
    int n_max_pool = 444;
    // 单段最小保留点数
    int n_min_active_per_segment = 15;
    // 名义打靶空间步长 (m)
    double nominal_step_s = 0.15;
    // 密度函数基础刚度权重
    double density_w_base = 1.0;
    // 密度函数曲率敏感度权重
    double density_w_kappa = 3.0;
    // 密度函数障碍物敏感度权重
    double density_w_obs = 10.0;
    // 密集配点间距 (m)
    double dense_step_dist = 0.05;
    // 触发转向补丁的最小角度差 (rad)
    double steer_padding_epsilon = 0.017;
    // 物理角速度极限利用比例
    double steer_safe_rate_ratio = 0.8;
    // 短机动段退化阈值 (m)
    double min_segment_arc_length_for_degradation = 0.1;
    // 时间戳反积分死区保护 (m/s)
    double time_reintegration_epsilon = 1e-3;
    // 补丁点内存池 Margin
    int memory_pool_margin = 40;
    // 障碍物密度项警戒线 (m)
    double obstacle_density_margin = 0.05;
};

// 自适应重采样器单段输入。
struct AdaptiveResamplerSegmentInput {
    // 完整状态/控制序列
    std::vector<TrajectoryPoint> states;
    // 密集配点缓存
    std::vector<BSplineSmoother::DensePointData> dense_points;
    // 该段运动方向
    Direction direction{Direction::UNKNOWN};
};

// 自适应重采样结果
struct AdaptiveResamplerResult {
    bool success = false;
    std::string status_msg;
    // 最终固定维数打靶点序列
    std::vector<TrajectoryPoint> points;
    // 时间步长数组
    std::vector<double> delta_t;
    // 最终固化维度
    int final_dimension = 0;
};

// 轨迹自适应重采样与维数固化器。
class AdaptiveResampler {
   public:
    explicit AdaptiveResampler(const AdaptiveResamplerConfig& config);
    // 对多段密集序列执行自适应重采样与维数固化。
    AdaptiveResamplerResult resample(
        const std::vector<AdaptiveResamplerSegmentInput>& segments,
        const VehicleParams& vehicle_params,
        double initial_steer_angle = 0.0) const;

   protected:
    // 验证输入合法性
    void validateInputs(
        const std::vector<AdaptiveResamplerSegmentInput>& segments,
        const VehicleParams& vehicle_params) const;
    // 计算单段物理弧长
    double computeSegmentArcLength(
        const AdaptiveResamplerSegmentInput& segment) const;
    // 全局维数统筹
    int computeBaseTotalDimension(double total_arc_length,
                                  int segment_count) const;
    // 按弧长比例分发配额
    std::vector<int> distributeActivePoints(
        const std::vector<double>& segment_arc_lengths, int base_total) const;
    // 短段退化：线性插值给 2 个点
    std::vector<TrajectoryPoint> resampleDegenerateSegment(
        const AdaptiveResamplerSegmentInput& segment) const;
    // 常规段信息密度重采样
    std::vector<TrajectoryPoint> resampleSegmentByDensity(
        const AdaptiveResamplerSegmentInput& segment, int n_active,
        const VehicleParams& vehicle_params,
        std::vector<double>& sampled_s_out) const;
    // 构建段内信息密度函数
    std::vector<double> computeDensityFunction(
        const AdaptiveResamplerSegmentInput& segment,
        const VehicleParams& vehicle_params) const;
    // 梯形积分求 CDF
    std::vector<double> integrateCdf(const std::vector<double>& s,
                                     const std::vector<double>& rho) const;
    // 逆向等信息量映射提取打靶点
    std::vector<TrajectoryPoint> extractSamplesByCdf(
        const AdaptiveResamplerSegmentInput& segment,
        const std::vector<double>& s, const std::vector<double>& cdf,
        int n_active, std::vector<double>& sampled_s_out) const;
    // 段内时间戳复原
    void recoverSegmentTimeSteps(
        const std::vector<TrajectoryPoint>& sampled_points,
        const std::vector<double>& sampled_s,
        std::vector<double>& delta_t_out) const;
    // 由前轮偏角反推有向曲率
    double deltaToKappa(double delta, double wheelbase) const;
    // 线性插值角度（处理 2pi 周期性）
    double interpolateAngle(double a0, double a1, double alpha) const;
    // 转向补丁段
    struct SteerPaddingSegment {
        std::vector<TrajectoryPoint> points;
        std::vector<double> delta_t;
        int pad_count = 0;
        double delta_start = 0.0;
        double delta_end = 0.0;
    };
    // 在 anchor 位姿处注入转向补丁
    SteerPaddingSegment buildSteerPaddingSegment(const TrajectoryPoint& anchor,
                                                 double delta_start,
                                                 double delta_end,
                                                 double steer_safe_rate,
                                                 double delta_t_min) const;
    // 评估相邻段间是否需要换挡补丁
    SteerPaddingSegment buildCuspPaddingIfNeeded(
        const TrajectoryPoint& end_prev, const TrajectoryPoint& start_next,
        double steer_safe_rate, double delta_t_min) const;
    // 评估起始转向对齐补丁
    SteerPaddingSegment buildStartPaddingIfNeeded(
        const TrajectoryPoint& first_point, double initial_steer_angle,
        double steer_safe_rate, double delta_t_min) const;
    // 将常规段与补丁段按顺序拼接
    AdaptiveResamplerResult assembleFinalTrajectory(
        std::vector<std::vector<TrajectoryPoint>>& segment_points,
        std::vector<std::vector<double>>& segment_delta_t,
        std::vector<SteerPaddingSegment>& cusp_paddings,
        SteerPaddingSegment& start_padding) const;
    // 维度超限时压缩常规段或补丁点数
    AdaptiveResamplerResult enforceDimensionLimit(
        std::vector<std::vector<TrajectoryPoint>>& segment_points,
        std::vector<std::vector<double>>& segment_delta_t,
        const std::vector<AdaptiveResamplerSegmentInput>& segments,
        const VehicleParams& vehicle_params,
        std::vector<SteerPaddingSegment>& cusp_paddings,
        SteerPaddingSegment& start_padding,
        const std::vector<int>& active_points) const;

   protected:
    AdaptiveResamplerConfig config_;
};
}  // namespace apa_post_processor
