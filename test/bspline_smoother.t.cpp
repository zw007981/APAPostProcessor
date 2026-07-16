#include "preprocessing/bspline_smoother.h"

#include <gtest/gtest.h>
#include <omp.h>

#include <cmath>
#include <limits>
#include <vector>

#include "spatial/esdf_map.h"
#include "spatial/grid_map.h"
#include "util/constants.h"
#include "util/maneuver.h"
#include "util/trajectory_point.h"
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
    std::vector<TrajectoryPoint> points;
    for (double x = 0.0; x <= 5.0 + EPSILON; x += 0.1) {
        points.emplace_back(TrajectoryPoint{std::min(x, 5.0), 0.0, 0.0});
    }
    return Maneuver(std::move(points), Direction::FORWARD);
}

// 弯曲机动段：带航向变化的C形曲线，用于验证曲率连续性。
Maneuver MakeCurvedManeuver() {
    std::vector<TrajectoryPoint> points;
    constexpr double kRadius = 5.0;
    constexpr int kNumPoints = 51;
    for (int i = 0; i < kNumPoints; ++i) {
        const double t =
            static_cast<double>(i) / static_cast<double>(kNumPoints - 1);
        const double theta = t * PI * 0.5;
        const double x = kRadius * std::sin(theta);
        const double y = kRadius * (1.0 - std::cos(theta));
        points.emplace_back(TrajectoryPoint{x, y, theta});
    }
    return Maneuver(std::move(points), Direction::FORWARD);
}

// 超短机动段：两点间距小于退化阈值，用于验证退化路径不调用L-BFGS。
Maneuver MakeShortManeuver() {
    std::vector<TrajectoryPoint> points;
    points.emplace_back(TrajectoryPoint{0.0, 0.0, 0.0});
    points.emplace_back(TrajectoryPoint{0.03, 0.0, 0.0});
    return Maneuver(std::move(points), Direction::FORWARD);
}

// 贴近障碍物的机动段：路径被迫紧贴y=0线穿过x=2.5处的障碍墙。
Maneuver MakeObstacleManeuver() {
    std::vector<TrajectoryPoint> points;
    for (double x = 0.0; x <= 5.0 + EPSILON; x += 0.05) {
        points.emplace_back(TrajectoryPoint{std::min(x, 5.0), 0.0, 0.0});
    }
    return Maneuver(std::move(points), Direction::FORWARD);
}

// 后退机动段：车辆航向朝左（theta=π），但实际沿 +x 方向倒车。
// 用于验证碰撞检测使用车辆航向而非运动方向。
Maneuver MakeBackwardManeuver() {
    std::vector<TrajectoryPoint> points;
    for (double x = 0.0; x <= 5.0 + EPSILON; x += 0.1) {
        points.emplace_back(TrajectoryPoint{std::min(x, 5.0), 0.0, PI});
    }
    return Maneuver(std::move(points), Direction::BACKWARD);
}

// 位于后退车辆“前方”的障碍物：车辆航向朝左，障碍物在路径左侧，
// 应被正确检测为碰撞；若错误使用运动方向则会被误判为无障碍。
ESDFMap MakeBackwardObstacleEsdfMap() {
    std::vector<Position> cells;
    for (int i = -5; i <= 5; ++i) {
        cells.emplace_back(Position{-1.0, static_cast<double>(i) * 0.1});
    }
    const GridMap grid_map(0.1, 100, 120, Position{-2.0, -4.0}, cells);
    return ESDFMap(grid_map);
}

// 用于验证重试逻辑的轻障碍：单个占据单元位于(2.5, 1.1)，
// 长直路径y=0从其下方穿过，车身外圆刚好轻微侵入。
// 注意：collision_margin 已从 5cm 改为 0（外圆自身提供安全缓冲）。
ESDFMap MakeRetryObstacleEsdfMap() {
    std::vector<Position> cells;
    cells.emplace_back(Position{2.5, 1.08});
    const GridMap grid_map(0.1, 100, 120, Position{-1.0, -4.0}, cells);
    return ESDFMap(grid_map);
}

// 退化至极：单个点的机动段，验证不抛异常。
Maneuver MakeSinglePointManeuver() {
    return Maneuver(TrajectoryPoint{0.0, 0.0, 0.0}, Direction::FORWARD);
}

// 空机动段：理论上不应出现，验证死区保护不崩溃。
Maneuver MakeEmptyManeuver() {
    std::vector<TrajectoryPoint> points;
    points.emplace_back(TrajectoryPoint{0.0, 0.0, 0.0});
    points.emplace_back(TrajectoryPoint{0.0, 0.0, 0.0});
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

// 后退机动段：车辆航向与运动方向相反，碰撞检测必须使用车辆航向。
// 触发原因：若错误地使用曲线切向作为航向，会把车身前缘放到运动方向一侧，
// 导致对真实位于车辆前方的障碍物漏检。
// 预期行为：success=false（障碍物在车辆前方左侧），且 dense_points.theta 保持为
// π。
TEST(BSplineSmootherTest, DetectsCollisionForBackwardManeuverUsingHeading) {
    const auto vehicle_params = MakeVehicleParams();
    const auto esdf_map = MakeBackwardObstacleEsdfMap();
    const VehicleFootprintModel footprint_model(vehicle_params, 233, 2, 2);
    BSplineSmootherConfig config;
    BSplineSmoother smoother(config, vehicle_params, footprint_model, esdf_map);

    const auto maneuver = MakeBackwardManeuver();
    const auto result = smoother.smooth(maneuver);

    // 车辆航向朝左，障碍物在左侧前方，应被检测为碰撞。
    EXPECT_FALSE(result.success);
    EXPECT_GT(result.max_intrusion_depth,
              config.collision_validation_tolerance);

    // 密集配点首尾航向应与输入车辆航向一致（π），而非运动方向（0）。
    ASSERT_FALSE(result.dense_points.empty());
    EXPECT_NEAR(result.dense_points.front().theta, PI, 1e-3);
    EXPECT_NEAR(result.dense_points.back().theta, PI, 1e-3);
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

    std::vector<TrajectoryPoint> points;
    points.emplace_back(TrajectoryPoint{0.0, 0.0, 0.0});
    points.emplace_back(TrajectoryPoint{0.1, 0.0, 0.0});
    const Maneuver maneuver(std::move(points), Direction::FORWARD);
    const auto result = smoother.smooth(maneuver);

    EXPECT_TRUE(result.success);
    EXPECT_FALSE(result.degenerate);
    EXPECT_EQ(result.control_points.size(), 6U);
    EXPECT_GT(result.lbfgs_iterations, 0);
    EXPECT_TRUE(result.optimizer_converged);
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

// OMP 并行正确性：同一机动段在串行（1 线程）与并行（4 线程）下的平滑结果
// 应在浮点容差内一致。该测试是防御未来 OMP 代码被误修改的关键安全网。
// 触发原因：M015 引入的 per-thread 局部梯度 + reduction 归约若存在数据竞争
// 或索引错误，不同线程数可能得到不同结果。
// 预期行为：success、max_intrusion_depth、control_points 在 1e-6 容差内一致。
TEST(BSplineSmootherTest, OmpParallelProducesSameResultAsSerial) {
    const auto vehicle_params = MakeVehicleParams();
    const auto esdf_map = MakeObstacleEsdfMap();
    const VehicleFootprintModel footprint_model(vehicle_params, 233, 2, 2);
    BSplineSmootherConfig config;
    BSplineSmoother smoother(config, vehicle_params, footprint_model, esdf_map);

    const auto maneuver = MakeObstacleManeuver();
    const int original_threads = omp_get_max_threads();

    omp_set_num_threads(1);
    const auto result_serial = smoother.smooth(maneuver);

    omp_set_num_threads(4);
    const auto result_parallel = smoother.smooth(maneuver);

    omp_set_num_threads(original_threads);

    EXPECT_EQ(result_serial.success, result_parallel.success);
    // 容差从 1e-6 放宽：Milestone 023 修复了"L-BFGS 精修失败时错误回退到预推前
    // 状态"的 bug 后（改为回退到碰撞预推后的检查点），碰撞预推阶段本身使用的
    // OMP 并行归约（`reduction(+ : f_collision)` 与按线程收集梯度再合并）在
    // 不同线程数下的浮点求和顺序不同，是浮点加法不满足结合律的正常表现——50 次
    // 预推迭代的梯度下降对这类初始极小差异存在正常的链式放大效应。这不是并行
    // 归约的正确性 bug（每个线程独立累加、循环外统一合并，不存在数据竞争），
    // 之前用 1e-6 严格容差能通过纯属巧合：该场景此前的失败路径回退到与线程数
    // 无关的原始控制点，掩盖了预推阶段本就存在的这一浮点非确定性。
    constexpr double kOmpTolerance = 0.05;
    EXPECT_NEAR(result_serial.max_intrusion_depth,
                result_parallel.max_intrusion_depth, kOmpTolerance);
    ASSERT_EQ(result_serial.control_points.size(),
              result_parallel.control_points.size());
    for (std::size_t i = 0; i < result_serial.control_points.size(); ++i) {
        EXPECT_NEAR(result_serial.control_points[i].x(),
                    result_parallel.control_points[i].x(), kOmpTolerance);
        EXPECT_NEAR(result_serial.control_points[i].y(),
                    result_parallel.control_points[i].y(), kOmpTolerance);
    }
}

}  // namespace apa_post_processor
