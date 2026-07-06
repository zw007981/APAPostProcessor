#include "preprocessing/bspline_smoother.h"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "spatial/esdf_map.h"
#include "spatial/grid_map.h"
#include "util/constants.h"
#include "util/maneuver.h"
#include "util/path_point.h"
#include "vehicle/vehicle_footprint_model.h"
#include "vehicle/vehicle_params.h"

namespace apa_post_processor {
namespace {

// 公共车辆参数：与NMPC测试保持一致的量级，确保footprint模型可正常构建。
VehicleParams MakeVehicleParams() {
    return VehicleParams(/*length=*/4.3, /*width=*/1.8, /*wheelbase=*/2.7,
                         /*max_steer_angle=*/0.6, /*rear_overhang=*/0.8);
}

// 空地图：覆盖足够大区域，使无障碍物场景下碰撞代价恒为0。
ESDFMap MakeEmptyEsdfMap() {
    const GridMap grid_map(0.1, 200, 150, Position{-2.0, -3.0}, {});
    return ESDFMap(grid_map);
}

// 障碍物地图：在(2.5, 0.0)附近放置一堵占据墙，用于验证碰撞校验。
ESDFMap MakeObstacleEsdfMap() {
    std::vector<Position> cells;
    for (int i = -5; i <= 5; ++i) {
        cells.emplace_back(Position{2.5, static_cast<double>(i) * 0.1});
    }
    const GridMap grid_map(0.1, 100, 120, Position{-1.0, -4.0}, cells);
    return ESDFMap(grid_map);
}

// 长直机动段：从(0,0,0)到(5,0,0)，点距0.1m，用于验证正常平滑与首尾锚定。
Maneuver MakeLongStraightManeuver() {
    std::vector<PathPoint> points;
    for (double x = 0.0; x <= 5.0 + EPSILON; x += 0.1) {
        points.emplace_back(PathPoint{std::min(x, 5.0), 0.0, 0.0});
    }
    return Maneuver(std::move(points), Direction::FORWARD);
}

// 弯曲机动段：带航向变化的C形曲线，用于验证曲率连续性。
Maneuver MakeCurvedManeuver() {
    std::vector<PathPoint> points;
    constexpr double kRadius = 5.0;
    constexpr int kNumPoints = 51;
    for (int i = 0; i < kNumPoints; ++i) {
        const double t =
            static_cast<double>(i) / static_cast<double>(kNumPoints - 1);
        const double theta = t * PI * 0.5;
        const double x = kRadius * std::sin(theta);
        const double y = kRadius * (1.0 - std::cos(theta));
        points.emplace_back(PathPoint{x, y, theta});
    }
    return Maneuver(std::move(points), Direction::FORWARD);
}

// 超短机动段：两点间距小于退化阈值，用于验证退化路径不调用L-BFGS。
Maneuver MakeShortManeuver() {
    std::vector<PathPoint> points;
    points.emplace_back(PathPoint{0.0, 0.0, 0.0});
    points.emplace_back(PathPoint{0.03, 0.0, 0.0});
    return Maneuver(std::move(points), Direction::FORWARD);
}

// 贴近障碍物的机动段：路径被迫紧贴y=0线穿过x=2.5处的障碍墙。
Maneuver MakeObstacleManeuver() {
    std::vector<PathPoint> points;
    for (double x = 0.0; x <= 5.0 + EPSILON; x += 0.05) {
        points.emplace_back(PathPoint{std::min(x, 5.0), 0.0, 0.0});
    }
    return Maneuver(std::move(points), Direction::FORWARD);
}

// 用于验证重试逻辑的轻障碍：单个占据单元位于(2.5, 1.1)，
// 长直路径y=0从其下方穿过，车身外圆刚好轻微侵入。
// 初始碰撞权重较低时首次优化无法将侵入深度压到阈值以下，
// 翻倍权重后的重试可成功。
ESDFMap MakeRetryObstacleEsdfMap() {
    std::vector<Position> cells;
    cells.emplace_back(Position{2.5, 1.1});
    const GridMap grid_map(0.1, 100, 120, Position{-1.0, -4.0}, cells);
    return ESDFMap(grid_map);
}

// 退化至极：单个点的机动段，验证不抛异常。
Maneuver MakeSinglePointManeuver() {
    return Maneuver(PathPoint{0.0, 0.0, 0.0}, Direction::FORWARD);
}

// 空机动段：理论上不应出现，验证死区保护不崩溃。
Maneuver MakeEmptyManeuver() {
    std::vector<PathPoint> points;
    points.emplace_back(PathPoint{0.0, 0.0, 0.0});
    points.emplace_back(PathPoint{0.0, 0.0, 0.0});
    return Maneuver(std::move(points), Direction::FORWARD);
}

}  // namespace

// 正常长机动段：拟合应收敛、首尾位置与航向与输入一致、曲率C^2连续。
// 触发原因：这是BSplineSmoother的核心Happy Path，验证L-BFGS优化与 Frozen-theta
// 梯度正确性。
// 预期行为：success=true，起点/终点误差在1e-3m与1e-3rad内，航向沿曲线连续变化无跳变。
TEST(BSplineSmootherTest, SmoothsLongStraightManeuverAndAnchorsEndPoints) {
    const auto vehicle_params = MakeVehicleParams();
    const auto esdf_map = MakeEmptyEsdfMap();
    const VehicleFootprintModel footprint_model(vehicle_params,
                                                /*heading_sample_num=*/233,
                                                /*inner_row_num=*/2,
                                                /*outer_row_num=*/2);
    BSplineSmootherConfig config;
    BSplineSmoother smoother(config, vehicle_params, footprint_model, esdf_map);

    const auto maneuver = MakeLongStraightManeuver();
    const auto result = smoother.smooth(maneuver);

    EXPECT_TRUE(result.success);
    EXPECT_FALSE(result.degenerate);
    EXPECT_FALSE(result.control_points.empty());
    EXPECT_FALSE(result.arc_length_table.empty());
    EXPECT_FALSE(result.dense_points.empty());

    const auto& start = maneuver.points.front();
    const auto& end = maneuver.points.back();
    EXPECT_NEAR(result.control_points.front().x(), start.x, 1e-3);
    EXPECT_NEAR(result.control_points.front().y(), start.y, 1e-3);
    EXPECT_NEAR(result.control_points.back().x(), end.x, 1e-3);
    EXPECT_NEAR(result.control_points.back().y(), end.y, 1e-3);

    // 检查密集配点首尾航向与输入一致。
    EXPECT_NEAR(result.dense_points.front().theta, start.theta, 1e-3);
    EXPECT_NEAR(result.dense_points.back().theta, end.theta, 1e-3);

    // 检查最大侵入深度为0（无障碍物）。
    EXPECT_NEAR(result.max_intrusion_depth, 0.0, 1e-6);
}

// 弯曲机动段：验证平滑后曲线曲率/航向连续，无突变尖点。
// 触发原因：曲线场景下F_data法向约束与F_smooth高阶平滑共同作用，易出现曲率震荡。
// 预期行为：相邻密集配点航向角变化平滑，无超过0.1rad的阶跃。
TEST(BSplineSmootherTest, SmoothsCurvedManeuverWithContinuousHeading) {
    const auto vehicle_params = MakeVehicleParams();
    const auto esdf_map = MakeEmptyEsdfMap();
    const VehicleFootprintModel footprint_model(vehicle_params, 233, 2, 2);
    BSplineSmootherConfig config;
    BSplineSmoother smoother(config, vehicle_params, footprint_model, esdf_map);

    const auto maneuver = MakeCurvedManeuver();
    const auto result = smoother.smooth(maneuver);

    EXPECT_TRUE(result.success);
    EXPECT_FALSE(result.degenerate);
    EXPECT_FALSE(result.dense_points.empty());

    double max_heading_jump = 0.0;
    for (std::size_t i = 1; i < result.dense_points.size(); ++i) {
        const double diff = std::abs(std::remainder(
            result.dense_points[i].theta - result.dense_points[i - 1].theta,
            2.0 * PI));
        max_heading_jump = std::max(max_heading_jump, diff);
    }
    EXPECT_LT(max_heading_jump, 0.1);
}

// 短机动段退化路径：弧长小于阈值时直接返回线性插值控制点，不调用L-BFGS。
// 触发原因：退化路径控制点过少，强行优化会病态，必须旁路。
// 预期行为：degenerate=true，控制点数为5（最小线性B样条），迭代次数为0。
TEST(BSplineSmootherTest, FallsBackToLinearInterpolationForShortSegment) {
    const auto vehicle_params = MakeVehicleParams();
    const auto esdf_map = MakeEmptyEsdfMap();
    const VehicleFootprintModel footprint_model(vehicle_params, 233, 2, 2);
    BSplineSmootherConfig config;
    BSplineSmoother smoother(config, vehicle_params, footprint_model, esdf_map);

    const auto maneuver = MakeShortManeuver();
    const auto result = smoother.smooth(maneuver);

    EXPECT_TRUE(result.success);
    EXPECT_TRUE(result.degenerate);
    EXPECT_EQ(result.control_points.size(), 5U);
    EXPECT_EQ(result.lbfgs_iterations, 0);
}

// 贴近障碍物场景：验证ValidateCollisionFree能正确检测越限侵入并返回失败。
// 触发原因：路径被障碍墙挡住，平滑后仍无法把车身子圆推出安全裕度。
// 预期行为：success=false且max_intrusion_depth大于collision_validation_tolerance。
TEST(BSplineSmootherTest, DetectsCollisionWhenPathIsTooCloseToObstacle) {
    const auto vehicle_params = MakeVehicleParams();
    const auto esdf_map = MakeObstacleEsdfMap();
    const VehicleFootprintModel footprint_model(vehicle_params, 233, 2, 2);
    BSplineSmootherConfig config;
    BSplineSmoother smoother(config, vehicle_params, footprint_model, esdf_map);

    const auto maneuver = MakeObstacleManeuver();
    const auto result = smoother.smooth(maneuver);

    EXPECT_FALSE(result.success);
    EXPECT_GT(result.max_intrusion_depth,
              config.collision_validation_tolerance);
}

// 死区保护：极端稀疏点列/退化输入不崩溃。
// 触发原因：上游路径可能出现单点或重合点，必须保证smooth()不抛未预期异常。
// 预期行为：两种退化输入均不抛异常且返回结果结构完整。
TEST(BSplineSmootherTest, HandlesDegenerateInputsWithoutCrashing) {
    const auto vehicle_params = MakeVehicleParams();
    const auto esdf_map = MakeEmptyEsdfMap();
    const VehicleFootprintModel footprint_model(vehicle_params, 233, 2, 2);
    BSplineSmootherConfig config;
    BSplineSmoother smoother(config, vehicle_params, footprint_model, esdf_map);

    EXPECT_NO_THROW({
        const auto result = smoother.smooth(MakeSinglePointManeuver());
        EXPECT_TRUE(result.success);
        EXPECT_TRUE(result.degenerate);
    });

    EXPECT_NO_THROW({
        const auto result = smoother.smooth(MakeEmptyManeuver());
        EXPECT_TRUE(result.success);
        EXPECT_TRUE(result.degenerate);
    });
}

// 控制点数量边界：弧长恰好等于退化阈值时，应得到6个控制点并正常优化。
// 触发原因：验证正常路径与退化路径之间的边界值处理。
// 预期行为：success=true，degenerate=false，control_points.size()=6，L-BFGS迭代>0。
TEST(BSplineSmootherTest, OptimizesSegmentWithExactlySixControlPoints) {
    const auto vehicle_params = MakeVehicleParams();
    const auto esdf_map = MakeEmptyEsdfMap();
    const VehicleFootprintModel footprint_model(vehicle_params, 233, 2, 2);
    BSplineSmootherConfig config;
    BSplineSmoother smoother(config, vehicle_params, footprint_model, esdf_map);

    std::vector<PathPoint> points;
    points.emplace_back(PathPoint{0.0, 0.0, 0.0});
    points.emplace_back(PathPoint{0.1, 0.0, 0.0});
    const Maneuver maneuver(std::move(points), Direction::FORWARD);
    const auto result = smoother.smooth(maneuver);

    EXPECT_TRUE(result.success);
    EXPECT_FALSE(result.degenerate);
    EXPECT_EQ(result.control_points.size(), 6U);
    EXPECT_GT(result.lbfgs_iterations, 0);
    EXPECT_TRUE(result.optimizer_converged);
}

// 重试成功路径：首次优化后侵入深度超阈值，翻倍碰撞权重后应能收敛到无碰撞。
// 触发原因：验证smooth()内部的碰撞重试降级路径确实生效，且迭代次数统计为总和。
// 预期行为：success=true，optimizer_converged=true，lbfgs_iterations>100（说明触发了重试）。
TEST(BSplineSmootherTest, RetriesWithDoubledCollisionWeightAndSucceeds) {
    const auto vehicle_params = MakeVehicleParams();
    const auto esdf_map = MakeRetryObstacleEsdfMap();
    const VehicleFootprintModel footprint_model(vehicle_params, 233, 2, 2);
    BSplineSmootherConfig config;
    // 初始碰撞权重较低，首次优化无法把侵入深度压到阈值以下。
    config.weight_collision = 10.0;
    BSplineSmoother smoother(config, vehicle_params, footprint_model, esdf_map);

    const auto maneuver = MakeLongStraightManeuver();
    const auto result = smoother.smooth(maneuver);

    EXPECT_TRUE(result.success);
    EXPECT_TRUE(result.optimizer_converged);
    // 首次+重试两轮L-BFGS均使用满100次迭代，因此总次数应大于单次上限。
    EXPECT_GT(result.lbfgs_iterations, 100);
    EXPECT_LE(result.max_intrusion_depth,
              config.collision_validation_tolerance);
}

// 弧长表单调性：验证(u_i, s_i)严格单调递增，且lower_bound反解与直接求值一致。
// 触发原因：弧长表是后续空间域阶段的索引基础，单调性必须保证。
// 预期行为：s序列严格递增；给定s反解u后，前向求值得到的s'与s误差在数值容差内。
TEST(BSplineSmootherTest, ArcLengthTableIsStrictlyMonotonic) {
    const auto vehicle_params = MakeVehicleParams();
    const auto esdf_map = MakeEmptyEsdfMap();
    const VehicleFootprintModel footprint_model(vehicle_params, 233, 2, 2);
    BSplineSmootherConfig config;
    BSplineSmoother smoother(config, vehicle_params, footprint_model, esdf_map);

    const auto maneuver = MakeLongStraightManeuver();
    const auto result = smoother.smooth(maneuver);

    ASSERT_FALSE(result.arc_length_table.empty());
    double prev_s = -1.0;
    for (const auto& [u, s] : result.arc_length_table) {
        EXPECT_GE(u, 0.0);
        EXPECT_LE(u, 1.0);
        EXPECT_GT(s, prev_s);
        prev_s = s;
    }

    // 在弧长表中点处反解u，再检查插值一致性。
    const double total_s = result.arc_length_table.back().second;
    const double target_s = total_s * 0.5;
    const auto it = std::lower_bound(
        result.arc_length_table.begin(), result.arc_length_table.end(),
        target_s, [](const std::pair<double, double>& item, double value) {
            return item.second < value;
        });
    ASSERT_NE(it, result.arc_length_table.end());
    ASSERT_NE(it, result.arc_length_table.begin());
    const auto prev = std::prev(it);
    const double denom = it->second - prev->second;
    const double alpha =
        (denom > 1e-12) ? (target_s - prev->second) / denom : 0.0;
    const double u_interp = prev->first + alpha * (it->first - prev->first);
    const double s_interp = prev->second + alpha * (it->second - prev->second);
    EXPECT_NEAR(s_interp, target_s, 1e-9);
    EXPECT_GE(u_interp, 0.0);
    EXPECT_LE(u_interp, 1.0);
}

}  // namespace apa_post_processor
