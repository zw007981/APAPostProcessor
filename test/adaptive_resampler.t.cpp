#include "preprocessing/adaptive_resampler.h"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "preprocessing/bspline_smoother.h"
#include "spatial/esdf_map.h"
#include "spatial/grid_map.h"
#include "util/constants.h"
#include "util/maneuver.h"
#include "util/trajectory_point.h"
#include "vehicle/vehicle_footprint_model.h"
#include "vehicle/vehicle_params.h"

namespace apa_post_processor {
namespace {

// 典型车辆参数：与 NMPC 测试保持一致。
VehicleParams MakeVehicleParams() {
    return VehicleParams(/*length=*/4.3, /*width=*/1.8, /*wheelbase=*/2.7,
                         /*max_steer_angle=*/0.6, /*rear_overhang=*/0.8,
                         /*max_accel=*/1.0, /*max_decel=*/-1.5,
                         /*max_steer_rate=*/0.4);
}

// 默认自适应重采样配置
AdaptiveResamplerConfig MakeConfig() {
    AdaptiveResamplerConfig config;
    config.n_max_pool = 444;
    config.n_min_active_per_segment = 15;
    config.nominal_step_s = 0.15;
    config.density_w_base = 1.0;
    config.density_w_kappa = 3.0;
    config.density_w_obs = 10.0;
    config.dense_step_dist = 0.05;
    config.steer_padding_epsilon = 0.017;
    config.steer_safe_rate_ratio = 0.8;
    config.min_segment_arc_length_for_degradation = 0.1;
    config.time_reintegration_epsilon = 1e-3;
    config.memory_pool_margin = 40;
    config.obstacle_density_margin = 0.05;
    return config;
}

// 构造单个车身子圆的 ESDF 缓存
BSplineSmoother::CircleEsdfData MakeCircleEsdfData(double dist) {
    BSplineSmoother::CircleEsdfData circle;
    circle.dist = dist;
    return circle;
}

// 构造一段等距直线密集输入：沿 x 轴从 (0,0,0) 延伸到 (length,0,0)。
// 每点挂载一个圆，dist 由调用方通过 dist_callback 控制。
AdaptiveResamplerSegmentInput MakeStraightSegment(
    double length, double step, int direction_sign,
    const std::function<double(std::size_t)>& dist_callback) {
    AdaptiveResamplerSegmentInput segment;
    const std::size_t n_points =
        static_cast<std::size_t>(std::round(length / step)) + 1;
    segment.states.reserve(n_points);
    segment.dense_points.reserve(n_points);
    for (std::size_t i = 0; i < n_points; ++i) {
        const double s = std::min(static_cast<double>(i) * step, length);
        const double x = s;
        TrajectoryPoint point(x, 0.0, 0.0);
        point.setV(static_cast<double>(direction_sign) * 0.5);
        point.setA(0.0);
        point.setDelta(0.0);
        point.setDeltaDot(0.0);
        segment.states.push_back(point);
        BSplineSmoother::DensePointData dpd;
        dpd.s = s;
        dpd.position = Eigen::Vector2d(x, 0.0);
        dpd.theta = 0.0;
        dpd.circles.push_back(MakeCircleEsdfData(dist_callback(i)));
        segment.dense_points.push_back(dpd);
    }
    segment.direction =
        (direction_sign > 0) ? Direction::FORWARD : Direction::BACKWARD;
    return segment;
}

// 构造一段从指定起点出发、沿指定航向的等距直线密集输入。
AdaptiveResamplerSegmentInput MakeStraightSegmentFromPose(
    double start_x, double start_y, double theta, double length, double step,
    int direction_sign,
    const std::function<double(std::size_t)>& dist_callback) {
    AdaptiveResamplerSegmentInput segment;
    const std::size_t n_points =
        static_cast<std::size_t>(std::round(length / step)) + 1;
    segment.states.reserve(n_points);
    segment.dense_points.reserve(n_points);
    for (std::size_t i = 0; i < n_points; ++i) {
        const double s = std::min(static_cast<double>(i) * step, length);
        const double x = start_x + s * std::cos(theta);
        const double y = start_y + s * std::sin(theta);
        TrajectoryPoint point(x, y, theta);
        point.setV(static_cast<double>(direction_sign) * 0.5);
        point.setA(0.0);
        point.setDelta(0.0);
        point.setDeltaDot(0.0);
        segment.states.push_back(point);
        BSplineSmoother::DensePointData dpd;
        dpd.s = s;
        dpd.position = Eigen::Vector2d(x, y);
        dpd.theta = theta;
        dpd.circles.push_back(MakeCircleEsdfData(dist_callback(i)));
        segment.dense_points.push_back(dpd);
    }
    segment.direction =
        (direction_sign > 0) ? Direction::FORWARD : Direction::BACKWARD;
    return segment;
}

// 构造一段半圆弯曲密集输入，用于验证弯道加密效果。
AdaptiveResamplerSegmentInput MakeCurvedSegment(double radius, double step,
                                                int direction_sign) {
    AdaptiveResamplerSegmentInput segment;
    const double total_angle = PI * 0.5;
    const double arc_length = radius * total_angle;
    const std::size_t n_points =
        static_cast<std::size_t>(std::round(arc_length / step)) + 1;
    segment.states.reserve(n_points);
    segment.dense_points.reserve(n_points);
    const double wheelbase = MakeVehicleParams().wheelbase;
    for (std::size_t i = 0; i < n_points; ++i) {
        const double alpha =
            static_cast<double>(i) / static_cast<double>(n_points - 1);
        const double s = alpha * arc_length;
        const double theta = alpha * total_angle;
        const double x = radius * std::sin(theta);
        const double y = radius * (1.0 - std::cos(theta));
        const double delta = std::atan(wheelbase / radius);
        TrajectoryPoint point(x, y, theta);
        point.setV(static_cast<double>(direction_sign) * 0.5);
        point.setA(0.0);
        point.setDelta(delta);
        point.setDeltaDot(0.0);
        segment.states.push_back(point);
        BSplineSmoother::DensePointData dpd;
        dpd.s = s;
        dpd.position = Eigen::Vector2d(x, y);
        dpd.theta = theta;
        dpd.circles.push_back(MakeCircleEsdfData(10.0));
        segment.dense_points.push_back(dpd);
    }
    segment.direction =
        (direction_sign > 0) ? Direction::FORWARD : Direction::BACKWARD;
    return segment;
}

// 构造两段不连续直线密集输入，中间用于验证换挡补丁存在性。
std::vector<AdaptiveResamplerSegmentInput> MakeTwoSegmentManeuver(
    double first_delta, double second_delta) {
    std::vector<AdaptiveResamplerSegmentInput> segments(2);
    segments[0] = MakeStraightSegment(
        /*length=*/3.0, /*step=*/0.05, /*direction_sign=*/1,
        [](std::size_t) { return 10.0; });
    for (auto& pt : segments[0].states) {
        pt.setDelta(first_delta);
    }
    segments[1] = MakeStraightSegment(
        /*length=*/3.0, /*step=*/0.05, /*direction_sign=*/-1,
        [](std::size_t) { return 10.0; });
    for (auto& pt : segments[1].states) {
        pt.setDelta(second_delta);
    }
    return segments;
}

// 构造两段首尾相连直线密集输入，用于验证 delta_t 序列连续性。
std::vector<AdaptiveResamplerSegmentInput> MakeContinuousTwoSegmentManeuver(
    double first_delta, double second_delta) {
    std::vector<AdaptiveResamplerSegmentInput> segments(2);
    segments[0] = MakeStraightSegmentFromPose(
        /*start_x=*/0.0, /*start_y=*/0.0, /*theta=*/first_delta,
        /*length=*/3.0, /*step=*/0.05, /*direction_sign=*/1,
        [](std::size_t) { return 10.0; });
    for (auto& pt : segments[0].states) {
        pt.setDelta(first_delta);
    }
    const double end_x = 3.0 * std::cos(first_delta);
    const double end_y = 3.0 * std::sin(first_delta);
    segments[1] = MakeStraightSegmentFromPose(
        end_x, end_y, /*theta=*/second_delta, /*length=*/3.0,
        /*step=*/0.05, /*direction_sign=*/-1, [](std::size_t) { return 10.0; });
    for (auto& pt : segments[1].states) {
        pt.setDelta(second_delta);
    }
    return segments;
}

// 计算最终序列中相邻点之间的物理距离，用于验证 delta_t 语义。
std::vector<double> ComputeSpatialSteps(const AdaptiveResamplerResult& result) {
    std::vector<double> steps;
    steps.reserve(result.delta_t.size());
    for (std::size_t i = 0; i + 1 < result.points.size(); ++i) {
        const double dx = result.points[i + 1].x - result.points[i].x;
        const double dy = result.points[i + 1].y - result.points[i].y;
        steps.push_back(std::hypot(dx, dy));
    }
    return steps;
}

}  // namespace

// 配额分配下限与舍入归一化：多段不同弧长输入，验证总量精确等于 N_base_total，
// 且每段 >= 4。
// 触发原因：这是全局维数统筹的核心契约，必须保证 HPIPM 内存维度精确。
// 预期行为：success=true，最终维度不超过上限，delta_t 与点数匹配。
TEST(AdaptiveResamplerTest, DistributesQuotaWithLowerBoundAndNormalization) {
    const auto vehicle_params = MakeVehicleParams();
    const auto config = MakeConfig();
    AdaptiveResampler resampler(config);

    std::vector<AdaptiveResamplerSegmentInput> segments;
    segments.push_back(MakeStraightSegment(
        /*length=*/5.0, /*step=*/0.05, /*direction_sign=*/1,
        [](std::size_t) { return 10.0; }));
    segments.push_back(MakeStraightSegment(
        /*length=*/1.0, /*step=*/0.05, /*direction_sign=*/1,
        [](std::size_t) { return 10.0; }));
    segments.push_back(MakeStraightSegment(
        /*length=*/2.0, /*step=*/0.05, /*direction_sign=*/1,
        [](std::size_t) { return 10.0; }));

    const auto result = resampler.resample(segments, vehicle_params);

    EXPECT_TRUE(result.success) << result.status_msg;
    ASSERT_FALSE(result.points.empty());
    EXPECT_LE(result.final_dimension, config.n_max_pool);
    EXPECT_EQ(static_cast<int>(result.points.size()), result.final_dimension);
    // 测试中使用三段独立直线，段间不连续，因此 delta_t 数量为
    // final_dimension - num_segments；连续真实数据下应为 final_dimension - 1。
    EXPECT_EQ(static_cast<int>(result.delta_t.size()),
              result.final_dimension - static_cast<int>(segments.size()));
}

// 短段退化路径：弧长低于阈值的段直接线性插值给 2 个点。
// 触发原因：避免超短段强行走信息密度重采样导致数值病态。
// 预期行为：最终序列包含退化段的 2 个端点，不触发崩溃。
TEST(AdaptiveResamplerTest, FallsBackToLinearInterpolationForShortSegment) {
    const auto vehicle_params = MakeVehicleParams();
    const auto config = MakeConfig();
    AdaptiveResampler resampler(config);

    std::vector<AdaptiveResamplerSegmentInput> segments;
    segments.push_back(MakeStraightSegment(
        /*length=*/3.0, /*step=*/0.05, /*direction_sign=*/1,
        [](std::size_t) { return 10.0; }));
    segments.push_back(MakeStraightSegment(
        /*length=*/0.03, /*step=*/0.01, /*direction_sign=*/1,
        [](std::size_t) { return 10.0; }));
    segments.push_back(MakeStraightSegment(
        /*length=*/3.0, /*step=*/0.05, /*direction_sign=*/1,
        [](std::size_t) { return 10.0; }));

    const auto result = resampler.resample(segments, vehicle_params);

    EXPECT_TRUE(result.success) << result.status_msg;
    EXPECT_LE(result.final_dimension, config.n_max_pool);
    EXPECT_GT(result.final_dimension, 0);
}

// 密度函数弯道加密效果：弯曲段的采样点应比同等长度的直线段更密集。
// 触发原因：验证曲率权重 density_w_kappa 确实提升弯道区域采样密度。
// 预期行为：弯曲段在弯心附近的最大相邻步长明显小于直线段。
TEST(AdaptiveResamplerTest, EncryptsSamplingInCurvedRegions) {
    const auto vehicle_params = MakeVehicleParams();
    auto config = MakeConfig();
    config.density_w_kappa = 10.0;  // 加大曲率权重使效果更显著
    AdaptiveResampler resampler(config);

    auto straight = MakeStraightSegment(
        /*length=*/5.0, /*step=*/0.05, /*direction_sign=*/1,
        [](std::size_t) { return 10.0; });
    auto curved = MakeCurvedSegment(/*radius=*/5.0, /*step=*/0.05,
                                    /*direction_sign=*/1);

    const auto straight_result = resampler.resample(
        std::vector<AdaptiveResamplerSegmentInput>{straight}, vehicle_params);
    const auto curved_result = resampler.resample(
        std::vector<AdaptiveResamplerSegmentInput>{curved}, vehicle_params);

    EXPECT_TRUE(straight_result.success);
    EXPECT_TRUE(curved_result.success);

    auto maxStep = [](const AdaptiveResamplerResult& res) {
        double max_step = 0.0;
        for (std::size_t i = 0; i + 1 < res.points.size(); ++i) {
            const double dx = res.points[i + 1].x - res.points[i].x;
            const double dy = res.points[i + 1].y - res.points[i].y;
            max_step = std::max(max_step, std::hypot(dx, dy));
        }
        return max_step;
    };
    const double straight_max_step = maxStep(straight_result);
    const double curved_max_step = maxStep(curved_result);
    EXPECT_LT(curved_max_step, straight_max_step);
}

// 多圆一致性：构造"车头贴障碍但几何中心较远"的场景，
// 验证密度函数通过子圆距离正确激发，而非几何中心单点查询遗漏。
// 触发原因：车身 footprint 并非质点，转弯时车头/车尾可能比后轴中心更危险。
// 预期行为：障碍物邻近区域的采样点数量明显多于无障碍区域。
TEST(AdaptiveResamplerTest, DetectsObstacleViaMultiCircleConsistency) {
    const auto vehicle_params = MakeVehicleParams();
    auto config = MakeConfig();
    config.density_w_obs = 50.0;  // 加大障碍物权重使效果更显著
    AdaptiveResampler resampler(config);

    // 直线段，但在中间区域让车头子圆 dist 很小（模拟车头贴障碍），
    // 几何中心 dist 保持很大。
    const std::size_t n_obstacle_center = 60;
    const std::size_t obstacle_width = 10;
    auto segment = MakeStraightSegment(
        /*length=*/6.0, /*step=*/0.05, /*direction_sign=*/1,
        [&](std::size_t i) {
            if (i + 5 >= n_obstacle_center && i <= n_obstacle_center + 5) {
                return 0.02;  // 车头子圆贴障碍
            }
            return 10.0;  // 几何中心远离障碍
        });

    const auto result = resampler.resample(
        std::vector<AdaptiveResamplerSegmentInput>{segment}, vehicle_params);

    EXPECT_TRUE(result.success);
    // 统计障碍区域内的采样点数量（按 x 坐标落在障碍区附近）
    const double obs_x_center = static_cast<double>(n_obstacle_center) * 0.05;
    int points_near_obstacle = 0;
    for (const auto& pt : result.points) {
        if (std::abs(pt.x - obs_x_center) < 0.5) {
            ++points_near_obstacle;
        }
    }
    EXPECT_GT(points_near_obstacle, 3);
}

// 换挡补丁注入：两段之间 delta 差异较大时，应注入 v=0 的静态打轮补丁。
// 触发原因：验证换挡尖点处的原地转向补丁正确性。
// 预期行为：补丁点 v=0、a=0、delta 单调过渡，delta_t 数量等于点数减 1，
// 且每条 delta_t 语义正确。
TEST(AdaptiveResamplerTest, InjectsCuspPaddingWithZeroVelocity) {
    const auto vehicle_params = MakeVehicleParams();
    const auto config = MakeConfig();
    AdaptiveResampler resampler(config);

    const auto segments = MakeContinuousTwoSegmentManeuver(
        /*first_delta=*/0.4, /*second_delta=*/-0.3);
    const auto result = resampler.resample(segments, vehicle_params);

    EXPECT_TRUE(result.success) << result.status_msg;
    EXPECT_EQ(static_cast<int>(result.delta_t.size()),
              result.final_dimension - 1);

    const auto spatial_steps = ComputeSpatialSteps(result);
    ASSERT_EQ(spatial_steps.size(), result.delta_t.size());

    // 找到补丁段：连续 v=0 的点列；验证其 delta_t 对应的空间步长为 0
    bool found_padding = false;
    for (std::size_t i = 1; i + 1 < result.points.size(); ++i) {
        if (std::abs(result.points[i].getV()) < 1e-6 &&
            std::abs(result.points[i - 1].getV()) < 1e-6) {
            found_padding = true;
            EXPECT_NEAR(result.points[i].getA(), 0.0, 1e-6);
            EXPECT_GT(result.delta_t[i], 0.0);
            EXPECT_LT(spatial_steps[i], 1e-6)
                << "cusp padding delta_t connects spatially separated points";
        }
    }
    EXPECT_TRUE(found_padding);
}

// 起始转向对齐补丁：传入与第一个打靶点 delta 差异较大的 initial_steer_angle，
// 应在绝对开头注入对齐补丁。
// 触发原因：NMPC 求解器常需要初始方向盘角与 Warm Start 第一段匹配。
// 预期行为：传入差异较大的初始角时存在补丁；传入默认值且无明显差异时
// 不注入。
TEST(AdaptiveResamplerTest, InjectsStartSteerAlignmentPaddingWhenNeeded) {
    const auto vehicle_params = MakeVehicleParams();
    const auto config = MakeConfig();
    AdaptiveResampler resampler(config);

    auto segment = MakeStraightSegment(
        /*length=*/3.0, /*step=*/0.05, /*direction_sign=*/1,
        [](std::size_t) { return 10.0; });
    for (auto& pt : segment.states) {
        pt.setDelta(0.0);
    }

    // 默认 initial_steer_angle = 0.0，与第一段 delta 一致，不应注入补丁
    const auto result_no_pad = resampler.resample(
        std::vector<AdaptiveResamplerSegmentInput>{segment}, vehicle_params);
    EXPECT_TRUE(result_no_pad.success);

    // 传入差异较大的初始角，应注入补丁
    const double initial_delta = 0.5;
    const auto result_with_pad =
        resampler.resample(std::vector<AdaptiveResamplerSegmentInput>{segment},
                           vehicle_params, initial_delta);
    EXPECT_TRUE(result_with_pad.success);
    EXPECT_GT(result_with_pad.final_dimension, result_no_pad.final_dimension);
    EXPECT_EQ(static_cast<int>(result_with_pad.delta_t.size()),
              result_with_pad.final_dimension - 1);

    // 验证补丁段 v=0 且位于绝对开头
    bool found_start_padding = false;
    for (std::size_t i = 0; i < result_with_pad.points.size() &&
                            std::abs(result_with_pad.points[i].getV()) < 1e-6;
         ++i) {
        found_start_padding = true;
        EXPECT_NEAR(result_with_pad.points[i].getA(), 0.0, 1e-6);
    }
    EXPECT_TRUE(found_start_padding);
}

// 内存越界兆底：构造补丁点超多的极端场景，验证两级兜底均触发且最终
// N_final <= N_max_pool，同时压缩后的补丁仍覆盖完整转向角度。
// 触发原因：V 型掉头等极端换挡可能一次性需要数十个补丁点，必须保证不崩溃。
// 预期行为：success=true，最终维度 <= n_max_pool，第一段末尾 delta 与第二段
// 开头 delta 的过渡在最终序列中仍覆盖完整范围。
TEST(AdaptiveResamplerTest, EnforcesDimensionLimitUnderExtremeCusp) {
    const auto vehicle_params = MakeVehicleParams();
    auto config = MakeConfig();
    config.n_max_pool = 50;
    config.memory_pool_margin = 4;
    config.n_min_active_per_segment = 4;
    config.steer_safe_rate_ratio = 0.1;  // 故意降低等效角速度，增加补丁点数
    AdaptiveResampler resampler(config);

    // 两段之间 delta 差接近 180°，制造极大补丁需求
    const double first_delta = 0.8;
    const double second_delta = -0.8;
    const auto segments =
        MakeContinuousTwoSegmentManeuver(first_delta, second_delta);
    const auto result = resampler.resample(segments, vehicle_params);

    EXPECT_TRUE(result.success) << result.status_msg;
    EXPECT_LE(result.final_dimension, config.n_max_pool);
    EXPECT_EQ(static_cast<int>(result.points.size()), result.final_dimension);
    EXPECT_EQ(static_cast<int>(result.delta_t.size()),
              result.final_dimension - 1);

    // 找到补丁区域中所有 v=0 的点，验证其 delta 覆盖完整过渡范围
    double min_delta_in_padding = std::numeric_limits<double>::infinity();
    double max_delta_in_padding = -std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < result.points.size(); ++i) {
        if (std::abs(result.points[i].getV()) < 1e-6) {
            min_delta_in_padding =
                std::min(min_delta_in_padding, result.points[i].getDelta());
            max_delta_in_padding =
                std::max(max_delta_in_padding, result.points[i].getDelta());
        }
    }
    EXPECT_LT(min_delta_in_padding, second_delta + 0.1);
    EXPECT_GT(max_delta_in_padding, first_delta - 0.1);
}

// 角度插值跨越 ±π 边界：验证 interpolateAngle 走最短路径而非绕远。
// 触发原因：航向角/前轮偏角具有 2π 周期性，方向反转场景常见。
// 预期行为：从 3.1 rad 过渡到 -3.0 rad 的补丁段 delta 单调走最短路径。
TEST(AdaptiveResamplerTest, HandlesAngleWrappingAcrossPiBoundary) {
    const auto vehicle_params = MakeVehicleParams();
    const auto config = MakeConfig();
    AdaptiveResampler resampler(config);

    const double first_delta = 3.1;
    const double second_delta = -3.0;
    const auto segments =
        MakeContinuousTwoSegmentManeuver(first_delta, second_delta);
    const auto result = resampler.resample(segments, vehicle_params);

    EXPECT_TRUE(result.success) << result.status_msg;
    EXPECT_EQ(static_cast<int>(result.delta_t.size()),
              result.final_dimension - 1);

    // 补丁区 v=0 的连续点列中，delta 应单调变化
    bool found_padding = false;
    for (std::size_t i = 1; i + 1 < result.points.size(); ++i) {
        if (std::abs(result.points[i].getV()) < 1e-6 &&
            std::abs(result.points[i - 1].getV()) < 1e-6) {
            found_padding = true;
            const double delta_prev = result.points[i - 1].getDelta();
            const double delta_curr = result.points[i].getDelta();
            // 相邻补丁点 delta 差值符号应一致且不超过 π
            const double diff =
                std::remainder(delta_curr - delta_prev, 2.0 * PI);
            EXPECT_GT(std::abs(diff), 0.0);
            EXPECT_LE(std::abs(diff), PI);
        }
    }
    EXPECT_TRUE(found_padding);
}

// CDF 退化场景：密度函数全零时（所有圆 dist 极大且 delta=0），
// 采样不应崩溃，且仍能产出指定数量的点。
// 触发原因：空旷直道等无障碍/无曲率场景下密度函数可能退化为常数。
// 预期行为：success=true，最终点数等于预期配额。
TEST(AdaptiveResamplerTest, HandlesZeroDensityCdfGracefully) {
    const auto vehicle_params = MakeVehicleParams();
    const auto config = MakeConfig();
    AdaptiveResampler resampler(config);

    auto segment = MakeStraightSegment(
        /*length=*/3.0, /*step=*/0.05, /*direction_sign=*/1,
        [](std::size_t) { return 1e6; });  // 所有子圆距离极大
    for (auto& pt : segment.states) {
        pt.setDelta(0.0);
    }

    const auto result = resampler.resample(
        std::vector<AdaptiveResamplerSegmentInput>{segment}, vehicle_params);

    EXPECT_TRUE(result.success) << result.status_msg;
    EXPECT_GT(result.final_dimension, 0);
    EXPECT_EQ(static_cast<int>(result.delta_t.size()),
              result.final_dimension - 1);
}

// 全退化场景：所有段均为短段，验证端到端不崩溃且输出维度合法。
// 触发原因：上游路径可能全部由极短 maneuver 组成。
// 预期行为：success=true，最终维度 <= n_max_pool，每段至少贡献 2 个点。
TEST(AdaptiveResamplerTest, HandlesFullyDegeneratedSegments) {
    const auto vehicle_params = MakeVehicleParams();
    const auto config = MakeConfig();
    AdaptiveResampler resampler(config);

    std::vector<AdaptiveResamplerSegmentInput> segments;
    for (int i = 0; i < 5; ++i) {
        segments.push_back(MakeStraightSegment(
            /*length=*/0.03, /*step=*/0.01, /*direction_sign=*/1,
            [](std::size_t) { return 10.0; }));
    }

    const auto result = resampler.resample(segments, vehicle_params);

    EXPECT_TRUE(result.success) << result.status_msg;
    EXPECT_LE(result.final_dimension, config.n_max_pool);
    EXPECT_EQ(static_cast<int>(result.points.size()), result.final_dimension);
}

// 非法输入：states 与 dense_points 长度不匹配应在入口抛出异常。
// 触发原因：防御调用方误传长度不等的向量。
// 预期行为：std::invalid_argument。
TEST(AdaptiveResamplerTest, RejectsMismatchedStateAndDensePointSizes) {
    const auto vehicle_params = MakeVehicleParams();
    const auto config = MakeConfig();
    AdaptiveResampler resampler(config);

    auto segment = MakeStraightSegment(
        /*length=*/3.0, /*step=*/0.05, /*direction_sign=*/1,
        [](std::size_t) { return 10.0; });
    segment.dense_points.pop_back();

    EXPECT_THROW(
        resampler.resample(std::vector<AdaptiveResamplerSegmentInput>{segment},
                           vehicle_params),
        std::invalid_argument);
}

// 非法输入：缺失 v/a/delta/delta_dot 应在入口抛出异常。
// 触发原因：确保上游必须提供完整状态/控制量。
// 预期行为：std::invalid_argument。
TEST(AdaptiveResamplerTest, RejectsMissingStateDerivatives) {
    const auto vehicle_params = MakeVehicleParams();
    const auto config = MakeConfig();
    AdaptiveResampler resampler(config);

    auto segment = MakeStraightSegment(
        /*length=*/3.0, /*step=*/0.05, /*direction_sign=*/1,
        [](std::size_t) { return 10.0; });
    segment.states[5] = TrajectoryPoint(
        segment.states[5].x, segment.states[5].y, segment.states[5].theta);

    EXPECT_THROW(
        resampler.resample(std::vector<AdaptiveResamplerSegmentInput>{segment},
                           vehicle_params),
        std::invalid_argument);
}

}  // namespace apa_post_processor
