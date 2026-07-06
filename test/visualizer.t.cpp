#include "util/visualizer.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <vector>

#include "util/constants.h"
#include "util/maneuver.h"
#include "util/path.h"
#include "util/path_point.h"
#include "util/pose.h"

namespace apa_post_processor {
namespace {

// 测试访问器：通过派生暴露 Visualizer 的 protected helper，进行纯逻辑白盒测试。
class VisualizerTestAccess : public Visualizer {
   public:
    using Visualizer::Visualizer;
    using Visualizer::tryExtractPath;
    using Visualizer::collectPathPoints;
    using Visualizer::appendDetailSeriesFromPath;
    const std::vector<DetailSeriesData>& detailSeries() const {
        return detail_series_;
    }
};

void ExpectPathPointsEqual(const Path& path,
                           const std::vector<Pose>& expected_poses) {
    ASSERT_EQ(path.size(), expected_poses.size());
    std::vector<PathPoint> points;
    points.reserve(path.size());
    path.forEach([&points](const PathPoint& point) { points.emplace_back(point); });
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

    ExpectPathPointsEqual(output, {Pose{0.0, 0.0, 0.0}, Pose{DELTA_DIST, 0.0, 0.0}});
}

TEST(VisualizerTryExtractPathTest, ExtractsManeuver) {
    const Maneuver input(
        std::vector<PathPoint>{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}},
        Direction::FORWARD);

    VisualizerTestAccess visualizer;
    Path output;
    ASSERT_TRUE(visualizer.tryExtractPath(input, output));

    ExpectPathPointsEqual(output, {Pose{0.0, 0.0, 0.0}, Pose{1.0, 0.0, 0.0}});
    ASSERT_EQ(output.numManeuvers(), 1U);
    EXPECT_EQ(output.getManeuvers().at(0).direction, Direction::FORWARD);
}

TEST(VisualizerTryExtractPathTest, ExtractsPathPointVector) {
    const std::vector<PathPoint> input = {{0.0, 0.0, 0.0}, {2.0, 0.0, 0.0}};

    VisualizerTestAccess visualizer;
    Path output;
    ASSERT_TRUE(visualizer.tryExtractPath(input, output));

    ExpectPathPointsEqual(output, {Pose{0.0, 0.0, 0.0}, Pose{2.0, 0.0, 0.0}});
    EXPECT_EQ(output.getManeuvers().at(0).direction, Direction::UNKNOWN);
}

TEST(VisualizerTryExtractPathTest, ExtractsSinglePathPoint) {
    const PathPoint input{3.0, 4.0, 0.5};

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

    ExpectPathPointsEqual(output, {Pose{0.0, 0.0, 0.0}, Pose{DELTA_DIST, 0.0, 0.0}});
}

TEST(VisualizerTryExtractPathTest, ExtractsViaSharedPtr) {
    auto input = std::make_shared<Path>();
    input->addPoint(Pose{0.0, 0.0, 0.0});
    input->addPoint(Pose{DELTA_DIST, 0.0, 0.0});
    input->finalize();

    VisualizerTestAccess visualizer;
    Path output;
    ASSERT_TRUE(visualizer.tryExtractPath(input, output));

    ExpectPathPointsEqual(output, {Pose{0.0, 0.0, 0.0}, Pose{DELTA_DIST, 0.0, 0.0}});
}

TEST(VisualizerTryExtractPathTest, ExtractsViaReferenceWrapper) {
    Path input;
    input.addPoint(Pose{0.0, 0.0, 0.0});
    input.addPoint(Pose{DELTA_DIST, 0.0, 0.0});
    input.finalize();

    VisualizerTestAccess visualizer;
    Path output;
    ASSERT_TRUE(visualizer.tryExtractPath(std::ref(input), output));

    ExpectPathPointsEqual(output, {Pose{0.0, 0.0, 0.0}, Pose{DELTA_DIST, 0.0, 0.0}});
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

TEST(VisualizerCollectPathPointsTest, ReturnsFlatSequenceSkippingDuplicateCusp) {
    Path path;
    const PathPoint first{0.0, 0.0, 0.0};
    const PathPoint cusp{1.0, 0.0, 0.0};
    const PathPoint target{0.5, 0.0, 0.0};
    path.getManeuvers().emplace_back(std::vector<PathPoint>{first, cusp},
                                     Direction::FORWARD);
    path.getManeuvers().emplace_back(std::vector<PathPoint>{cusp, target},
                                     Direction::BACKWARD);

    VisualizerTestAccess visualizer;
    const auto points = visualizer.collectPathPoints(path);

    ASSERT_EQ(points.size(), 3U);
    EXPECT_NEAR(points[0].x, 0.0, EPSILON);
    EXPECT_NEAR(points[1].x, 1.0, EPSILON);
    EXPECT_NEAR(points[2].x, 0.5, EPSILON);
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

}  // namespace
}  // namespace apa_post_processor
