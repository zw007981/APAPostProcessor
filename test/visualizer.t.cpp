#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "util/constants.h"
#include "util/maneuver.h"
#include "util/path.h"
#include "util/pose.h"
#include "util/trajectory.h"
#include "util/trajectory_point.h"
#include "util/visualizer.hpp"
#include "vehicle/vehicle_params.h"

namespace apa_post_processor {
namespace {

// 测试访问器：通过派生暴露 Visualizer 的 protected helper，进行纯逻辑白盒测试。
class VisualizerTestAccess : public Visualizer {
   public:
    using Visualizer::appendDetailSeriesFromPath;
    using Visualizer::buildVehicleFootprint;
    using Visualizer::ClampKappaForPlot;
    using Visualizer::ComputeCoordinateInterval;
    using Visualizer::ComputeCoordinatePrecision;
    using Visualizer::tryExtractPath;
    using Visualizer::Visualizer;
    const std::vector<DetailSeriesData>& detailSeries() const {
        return detail_series_;
    }
    const std::vector<TrajectoryPlotEntry>& trajectoryPlots() const {
        return trajectory_plots_;
    }
};

void ExpectPathPointsEqual(const Path& path,
                           const std::vector<Pose>& expected_poses) {
    ASSERT_EQ(path.size(), expected_poses.size());
    std::vector<TrajectoryPoint> points;
    points.reserve(path.size());
    path.forEach([&points](const TrajectoryPoint& point) {
        points.emplace_back(point);
    });
    for (std::size_t i = 0; i < expected_poses.size(); ++i) {
        EXPECT_NEAR(points[i].x, expected_poses[i].x, EPSILON);
        EXPECT_NEAR(points[i].y, expected_poses[i].y, EPSILON);
        EXPECT_NEAR(points[i].theta, expected_poses[i].theta, EPSILON);
    }
}

TEST(VisualizerTryExtractPathTest, ExtractsPath) {
    Path input;
    input.addPoint(Pose{0.0, 0.0, 0.0});
    input.addPoint(Pose{DELTA_DIST, 0.0, 0.0});
    input.finalize();

    VisualizerTestAccess visualizer;
    Path output;
    ASSERT_TRUE(visualizer.tryExtractPath(input, output));

    ExpectPathPointsEqual(output,
                          {Pose{0.0, 0.0, 0.0}, Pose{DELTA_DIST, 0.0, 0.0}});
}

TEST(VisualizerTryExtractPathTest, ExtractsManeuver) {
    const Maneuver input(
        std::vector<TrajectoryPoint>{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}},
        Direction::FORWARD);

    VisualizerTestAccess visualizer;
    Path output;
    ASSERT_TRUE(visualizer.tryExtractPath(input, output));

    ExpectPathPointsEqual(output, {Pose{0.0, 0.0, 0.0}, Pose{1.0, 0.0, 0.0}});
    ASSERT_EQ(output.numManeuvers(), 1U);
    EXPECT_EQ(output.getManeuvers().at(0).direction, Direction::FORWARD);
}

TEST(VisualizerTryExtractPathTest, ExtractsPathPointVector) {
    const std::vector<TrajectoryPoint> input = {{0.0, 0.0, 0.0},
                                                {2.0, 0.0, 0.0}};

    VisualizerTestAccess visualizer;
    Path output;
    ASSERT_TRUE(visualizer.tryExtractPath(input, output));

    ExpectPathPointsEqual(output, {Pose{0.0, 0.0, 0.0}, Pose{2.0, 0.0, 0.0}});
    EXPECT_EQ(output.getManeuvers().at(0).direction, Direction::UNKNOWN);
}

TEST(VisualizerTryExtractPathTest, ExtractsSinglePathPoint) {
    const TrajectoryPoint input{3.0, 4.0, 0.5};

    VisualizerTestAccess visualizer;
    Path output;
    ASSERT_TRUE(visualizer.tryExtractPath(input, output));

    ExpectPathPointsEqual(output, {Pose{3.0, 4.0, 0.5}});
}

TEST(VisualizerTryExtractPathTest, ExtractsPoseVector) {
    const std::vector<Pose> input = {Pose{0.0, 0.0, 0.0}, Pose{1.0, 1.0, 0.5}};

    VisualizerTestAccess visualizer;
    Path output;
    ASSERT_TRUE(visualizer.tryExtractPath(input, output));

    ExpectPathPointsEqual(output, {Pose{0.0, 0.0, 0.0}, Pose{1.0, 1.0, 0.5}});
}

TEST(VisualizerTryExtractPathTest, ExtractsSinglePose) {
    const Pose input{5.0, 6.0, -0.2};

    VisualizerTestAccess visualizer;
    Path output;
    ASSERT_TRUE(visualizer.tryExtractPath(input, output));

    ExpectPathPointsEqual(output, {Pose{5.0, 6.0, -0.2}});
}

TEST(VisualizerTryExtractPathTest, ExtractsViaPointer) {
    Path input;
    input.addPoint(Pose{0.0, 0.0, 0.0});
    input.addPoint(Pose{DELTA_DIST, 0.0, 0.0});
    input.finalize();

    VisualizerTestAccess visualizer;
    Path output;
    ASSERT_TRUE(visualizer.tryExtractPath(&input, output));

    ExpectPathPointsEqual(output,
                          {Pose{0.0, 0.0, 0.0}, Pose{DELTA_DIST, 0.0, 0.0}});
}

TEST(VisualizerTryExtractPathTest, ExtractsViaSharedPtr) {
    auto input = std::make_shared<Path>();
    input->addPoint(Pose{0.0, 0.0, 0.0});
    input->addPoint(Pose{DELTA_DIST, 0.0, 0.0});
    input->finalize();

    VisualizerTestAccess visualizer;
    Path output;
    ASSERT_TRUE(visualizer.tryExtractPath(input, output));

    ExpectPathPointsEqual(output,
                          {Pose{0.0, 0.0, 0.0}, Pose{DELTA_DIST, 0.0, 0.0}});
}

TEST(VisualizerTryExtractPathTest, ExtractsViaReferenceWrapper) {
    Path input;
    input.addPoint(Pose{0.0, 0.0, 0.0});
    input.addPoint(Pose{DELTA_DIST, 0.0, 0.0});
    input.finalize();

    VisualizerTestAccess visualizer;
    Path output;
    ASSERT_TRUE(visualizer.tryExtractPath(std::ref(input), output));

    ExpectPathPointsEqual(output,
                          {Pose{0.0, 0.0, 0.0}, Pose{DELTA_DIST, 0.0, 0.0}});
}

TEST(VisualizerTryExtractPathTest, ReturnsFalseForNullPointer) {
    const Path* null_input = nullptr;

    VisualizerTestAccess visualizer;
    Path output;
    EXPECT_FALSE(visualizer.tryExtractPath(null_input, output));
    EXPECT_TRUE(output.empty());
}

TEST(VisualizerTryExtractPathTest, ReturnsFalseForUnsupportedType) {
    const int unsupported = 42;

    VisualizerTestAccess visualizer;
    Path output;
    EXPECT_FALSE(visualizer.tryExtractPath(unsupported, output));
    EXPECT_TRUE(output.empty());
}

TEST(VisualizerAppendDetailSeriesTest, PopulatesBasicDataForSingleManeuver) {
    Path path;
    path.addPoint(Pose{0.0, 0.0, 0.0});
    path.addPoint(Pose{DELTA_DIST, 0.0, 0.0});
    path.addPoint(Pose{2.0 * DELTA_DIST, 0.0, 0.0});
    path.finalize();

    VisualizerTestAccess visualizer;
    visualizer.appendDetailSeriesFromPath(path, {});

    const auto& series = visualizer.detailSeries();
    ASSERT_EQ(series.size(), 1U);
    const auto& data = series.front();
    ASSERT_EQ(data.index.size(), 3U);
    ASSERT_EQ(data.x.size(), 3U);
    ASSERT_EQ(data.y.size(), 3U);
    ASSERT_EQ(data.heading.size(), 3U);
    ASSERT_EQ(data.ref_s.size(), 3U);
    ASSERT_EQ(data.curvature.size(), 3U);
    EXPECT_NEAR(data.x[0], 0.0, EPSILON);
    EXPECT_NEAR(data.x[1], DELTA_DIST, EPSILON);
    EXPECT_NEAR(data.x[2], 2.0 * DELTA_DIST, EPSILON);
    EXPECT_NEAR(data.ref_s[0], 0.0, EPSILON);
    EXPECT_NEAR(data.ref_s[1], DELTA_DIST, EPSILON);
    EXPECT_NEAR(data.ref_s[2], 2.0 * DELTA_DIST, EPSILON);
    EXPECT_TRUE(data.shift_indices.empty());
}

TEST(VisualizerAppendDetailSeriesTest, MarksShiftIndicesForMultiManeuverPath) {
    Path path;
    path.addPoint(Pose{0.0, 0.0, 0.0});
    path.addPoint(Pose{DELTA_DIST, 0.0, 0.0});
    path.addPoint(Pose{-DELTA_DIST, 0.0, 0.0});
    path.finalize();
    ASSERT_EQ(path.numManeuvers(), 2U);

    VisualizerTestAccess visualizer;
    visualizer.appendDetailSeriesFromPath(path, {});

    const auto& series = visualizer.detailSeries();
    ASSERT_EQ(series.size(), 1U);
    const auto& data = series.front();
    ASSERT_EQ(data.shift_indices.size(), 1U);
    EXPECT_EQ(data.shift_indices.front(), 2U);
}

TEST(VisualizerAppendDetailSeriesTest, UsesKappaWhenAvailable) {
    Path path;
    path.addPoint(Pose{0.0, 0.0, 0.0});
    path.addPoint(Pose{DELTA_DIST, 0.0, 0.0});
    path.addPoint(Pose{2.0 * DELTA_DIST, 0.0, 0.0});
    path.finalize();

    VisualizerTestAccess visualizer;
    visualizer.appendDetailSeriesFromPath(path, {});

    const auto& series = visualizer.detailSeries();
    ASSERT_EQ(series.size(), 1U);
    const auto& data = series.front();
    // 直线路径的曲率应为 0
    for (const auto kappa : data.curvature) {
        EXPECT_NEAR(kappa, 0.0, EPSILON);
    }
}

TEST(VisualizerAppendDetailSeriesTest, SkipsEmptyPath) {
    Path path;

    VisualizerTestAccess visualizer;
    visualizer.appendDetailSeriesFromPath(path, {});

    EXPECT_TRUE(visualizer.detailSeries().empty());
}

// 后轴中心为原点、朝 +x 时：前保险杠 x=length-rear_overhang，后保险杠
// x=-rear_overhang，半宽为 width/2，角点顺序与绘制约定一致。
TEST(VisualizerBuildVehicleFootprintTest, CornersCenteredOnRearAxle) {
    const VehicleParams vehicle_params{4.0, 2.0, 2.5, 0.6, 1.0};

    VisualizerTestAccess visualizer;
    const auto corners =
        visualizer.buildVehicleFootprint(Pose{0.0, 0.0, 0.0}, vehicle_params);

    ASSERT_EQ(corners.size(), 4U);
    EXPECT_NEAR(corners[0].x, 3.0, EPSILON);
    EXPECT_NEAR(corners[0].y, 1.0, EPSILON);
    EXPECT_NEAR(corners[1].x, 3.0, EPSILON);
    EXPECT_NEAR(corners[1].y, -1.0, EPSILON);
    EXPECT_NEAR(corners[2].x, -1.0, EPSILON);
    EXPECT_NEAR(corners[2].y, -1.0, EPSILON);
    EXPECT_NEAR(corners[3].x, -1.0, EPSILON);
    EXPECT_NEAR(corners[3].y, 1.0, EPSILON);
}

// 旋转 90° 后轮廓角点应随航向刚体变换，仍相对后轴中心布局。
TEST(VisualizerBuildVehicleFootprintTest, CornersRotateWithHeading) {
    const VehicleParams vehicle_params{4.0, 2.0, 2.5, 0.6, 1.0};

    VisualizerTestAccess visualizer;
    const auto corners = visualizer.buildVehicleFootprint(
        Pose{1.0, 2.0, PI / 2.0}, vehicle_params);

    ASSERT_EQ(corners.size(), 4U);
    EXPECT_NEAR(corners[0].x, 0.0, EPSILON);
    EXPECT_NEAR(corners[0].y, 5.0, EPSILON);
    EXPECT_NEAR(corners[2].x, 2.0, EPSILON);
    EXPECT_NEAR(corners[2].y, 1.0, EPSILON);
}

TEST(VisualizerPlotTrajectoryTest, DefaultsToNotDrawingStartEnd) {
    const Trajectory traj(
        std::vector<TrajectoryPoint>{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}});
    const VehicleParams vehicle_params{4.0, 2.0, 2.5, 0.6, 1.0};

    VisualizerTestAccess visualizer;
    visualizer.plotTrajectory(traj, vehicle_params);

    ASSERT_EQ(visualizer.trajectoryPlots().size(), 1U);
    EXPECT_FALSE(visualizer.trajectoryPlots().front().draw_start_end);
}

TEST(VisualizerPlotTrajectoryTest, StoresStartEndFlagWhenEnabled) {
    const Trajectory traj(
        std::vector<TrajectoryPoint>{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}});
    const VehicleParams vehicle_params{4.0, 2.0, 2.5, 0.6, 1.0};

    VisualizerTestAccess visualizer;
    visualizer.plotTrajectory(traj, vehicle_params, nullptr, nullptr, nullptr,
                              /*draw_swept_area=*/false,
                              /*draw_start_end=*/true);

    ASSERT_EQ(visualizer.trajectoryPlots().size(), 1U);
    EXPECT_TRUE(visualizer.trajectoryPlots().front().draw_start_end);
}

// 测试场景：kappa 绘图值按 ±kKappaPlotLimit(0.4444) 上下限裁剪——
// 换挡尖点/近重复点处的曲率估计尖峰伪影（除数趋零产生，物理上无意义）
// 贴轨到边界，避免压扁正常量程；正常值、恰在边界的值与非有限值
// （换挡断点）原样保留。
TEST(VisualizerClampKappaForPlotTest, ClampsOutOfRangeValuesAndPreservesRest) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    constexpr double kLimit = 0.4444;
    const std::vector<double> input = {
        0.0, 0.3, -0.3, 3.2, -150.0, nan, INFINITY, -INFINITY, kLimit, -kLimit};
    const auto clamped = VisualizerTestAccess::ClampKappaForPlot(input);
    ASSERT_EQ(clamped.size(), input.size());
    EXPECT_DOUBLE_EQ(clamped[0], 0.0);
    EXPECT_DOUBLE_EQ(clamped[1], 0.3);
    EXPECT_DOUBLE_EQ(clamped[2], -0.3);
    EXPECT_DOUBLE_EQ(clamped[3], kLimit);
    EXPECT_DOUBLE_EQ(clamped[4], -kLimit);
    EXPECT_TRUE(std::isnan(clamped[5]));
    EXPECT_TRUE(std::isinf(clamped[6]));
    EXPECT_TRUE(std::isinf(clamped[7]));
    EXPECT_DOUBLE_EQ(clamped[8], kLimit);
    EXPECT_DOUBLE_EQ(clamped[9], -kLimit);
}

// 测试场景：坐标刻度间隔需在 0.001~10000 全量级自适应——极小量程
// （如 κ 的 ±0.4 量程）不再被 1e-6 硬下限强制返回 1.0，大量程
// （如 margin 的 0~50）依旧产出整数间隔。
TEST(VisualizerComputeCoordinateIntervalTest, AdaptsAcrossMagnitudes) {
    // 非有限值与浮点噪声级跨度回退为 1.0。
    EXPECT_DOUBLE_EQ(VisualizerTestAccess::ComputeCoordinateInterval(
                         std::numeric_limits<double>::infinity(), 6),
                     1.0);
    EXPECT_DOUBLE_EQ(VisualizerTestAccess::ComputeCoordinateInterval(1e-15, 6),
                     1.0);
    // κ 量级：0.8 跨度 / 4 目标刻度 → 0.2 间隔。
    EXPECT_DOUBLE_EQ(VisualizerTestAccess::ComputeCoordinateInterval(0.8, 4),
                     0.2);
    // 极小量程不再触发硬下限：5e-9 跨度 / 5 → 1e-9 间隔（旧实现返回 1.0）。
    EXPECT_DOUBLE_EQ(VisualizerTestAccess::ComputeCoordinateInterval(5e-9, 5),
                     1e-9);
    // 大量级：50 跨度 / 5 → 10 间隔。
    EXPECT_DOUBLE_EQ(VisualizerTestAccess::ComputeCoordinateInterval(50.0, 5),
                     10.0);
}

// 测试场景：优美数乘数档位映射为 1.0/2.0/2.5/5.0/10.0 五档，
// 新增 2.5 档使跨度过渡更平滑。
TEST(VisualizerComputeCoordinateIntervalTest, SelectsNiceNumberMultipliers) {
    // fraction ≤ 1.2 → 1.0 档：跨度 10 / 10 → 间隔 1。
    EXPECT_DOUBLE_EQ(VisualizerTestAccess::ComputeCoordinateInterval(10.0, 10),
                     1.0);
    // fraction ≤ 2.5 → 2.0 档：跨度 0.5 / 2 → raw 0.25 → 间隔 0.2。
    EXPECT_DOUBLE_EQ(VisualizerTestAccess::ComputeCoordinateInterval(0.5, 2),
                     0.2);
    // fraction ≤ 3.5 → 2.5 档：跨度 0.6 / 2 → raw 0.3 → 间隔 0.25。
    EXPECT_DOUBLE_EQ(VisualizerTestAccess::ComputeCoordinateInterval(0.6, 2),
                     0.25);
    // fraction ≤ 7.0 → 5.0 档：跨度 10 / 2 → 间隔 5。
    EXPECT_DOUBLE_EQ(VisualizerTestAccess::ComputeCoordinateInterval(10.0, 2),
                     5.0);
    // fraction > 7.0 → 10.0 档：跨度 9 / 1 → 间隔 10。
    EXPECT_DOUBLE_EQ(VisualizerTestAccess::ComputeCoordinateInterval(9.0, 1),
                     10.0);
}

// 测试场景：刻度精度由间隔幂次决定，带小数的乘数（如 2.5 档的
// 0.25 间隔）额外补偿一位小数，避免标签被截断；逼近 1.0 的
// 间隔不再被强制取整。
TEST(VisualizerComputeCoordinatePrecisionTest, HandlesFractionalMultipliers) {
    // 非有限值与噪声级间隔回退为 0。
    EXPECT_EQ(VisualizerTestAccess::ComputeCoordinatePrecision(INFINITY), 0);
    EXPECT_EQ(VisualizerTestAccess::ComputeCoordinatePrecision(1e-15), 0);
    // 整数乘数：精度由幂次直接决定。
    EXPECT_EQ(VisualizerTestAccess::ComputeCoordinatePrecision(0.1), 1);
    EXPECT_EQ(VisualizerTestAccess::ComputeCoordinatePrecision(0.2), 1);
    EXPECT_EQ(VisualizerTestAccess::ComputeCoordinatePrecision(0.05), 2);
    EXPECT_EQ(VisualizerTestAccess::ComputeCoordinatePrecision(10.0), 0);
    // 带小数的乘数需补偿一位：0.25 → 2 位小数，2.5 → 1 位小数。
    EXPECT_EQ(VisualizerTestAccess::ComputeCoordinatePrecision(0.25), 2);
    EXPECT_EQ(VisualizerTestAccess::ComputeCoordinatePrecision(2.5), 1);
    // 逼近 1.0 的间隔保留小数：0.5 → 1 位小数。
    EXPECT_EQ(VisualizerTestAccess::ComputeCoordinatePrecision(0.5), 1);
}

}  // namespace
}  // namespace apa_post_processor