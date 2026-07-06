#include "core/NMPC/path_to_ocp_converter.h"

#include "util/path_point.h"

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include <costs/quadratic_tracking.h>
#include <models/bicycle_model_delta.h>

#include "test_fixture_util.h"
#include "util/constants.h"
#include "util/path.h"
#include "vehicle/vehicle_params.h"

namespace apa_post_processor {
namespace {

// 可测试子类：通过继承暴露受保护的辅助方法，便于对速度剖面/弧长插值等纯计算逻辑做白盒测试。
class TestablePathToOcpConverter : public PathToOcpConverter {
   public:
    using PathToOcpConverter::PathToOcpConverter;
    using PathToOcpConverter::buildCumulativeArcLength;
    using PathToOcpConverter::buildSegment;
    using PathToOcpConverter::buildVelocityProfile;
    using PathToOcpConverter::interpolateAtArcLength;
    using PathToOcpConverter::sampleManeuver;
};

// 公共车辆参数：轴距2.7m、最大前轮转角0.6rad，与其他测试文件保持一致的量级。
VehicleParams MakeVehicleParams() {
    return VehicleParams(/*length=*/4.3, /*width=*/1.8, /*wheelbase=*/2.7,
                         /*max_steer_angle=*/0.6, /*rear_overhang=*/0.8);
}

// 构造一条“先前进5m、再后退3m”的直线换挡路径，theta恒为0（曲率恒为0），
// 便于精确校验重采样后的位姿、速度符号与换挡检测。
Path MakeStraightLineSwitchbackPath() {
    Path path;
    for (double x = 0.0; x <= 5.0 + EPSILON; x += 0.1) {
        path.addPoint(Pose(std::min(x, 5.0), 0.0, 0.0));
    }
    for (double x = 5.0; x >= 2.0 - EPSILON; x -= 0.1) {
        path.addPoint(Pose(std::max(x, 2.0), 0.0, 0.0));
    }
    return path;
}

}  // namespace

// 测试构造函数拒绝非法车辆参数（轴距/最大转角非正）与非法配置（步长/速度等非正）。
// 因为这些量都会作为除数或box bound出现，非法值会导致除零或反向约束。
TEST(PathToOcpConverterTest, RejectsInvalidConstructorArguments) {
    const auto valid_params = MakeVehicleParams();
    EXPECT_THROW(PathToOcpConverter(VehicleParams(4.3, 1.8, 0.0, 0.6, 0.8)),
                std::invalid_argument);
    EXPECT_THROW(PathToOcpConverter(VehicleParams(4.3, 1.8, 2.7, 0.0, 0.8)),
                std::invalid_argument);

    PathToOcpConfig bad_dt;
    bad_dt.dt = 0.0;
    EXPECT_THROW(PathToOcpConverter(valid_params, bad_dt), std::invalid_argument);

    PathToOcpConfig bad_slack;
    bad_slack.boundary_velocity_slack = -0.01;
    EXPECT_THROW(PathToOcpConverter(valid_params, bad_slack),
                std::invalid_argument);
}

// 测试三次多项式速度剖面满足边界条件：s(0)=0, v(0)=0, s(tf)约等于弧长, v(tf)约等于0。
// 因为这是初始猜测构造的数学基础，边界条件不满足会导致重采样弧长与实际路径长度不一致。
TEST(PathToOcpConverterTest, VelocityProfileSatisfiesBoundaryConditions) {
    TestablePathToOcpConverter converter(MakeVehicleParams());
    for (double arc_length : {0.5, 2.0, 5.0, 12.3}) {
        const auto profile = converter.buildVelocityProfile(arc_length);
        const double s0 = profile.b * 0.0 + profile.c * 0.0;
        const double v0 = 0.0;
        const double tf = profile.tf;
        const double s_tf =
            profile.b * tf * tf * tf + profile.c * tf * tf;
        const double v_tf = 3.0 * profile.b * tf * tf + 2.0 * profile.c * tf;
        EXPECT_NEAR(s0, 0.0, 1e-9);
        EXPECT_NEAR(v0, 0.0, 1e-9);
        EXPECT_NEAR(s_tf, arc_length, 1e-6);
        EXPECT_NEAR(v_tf, 0.0, 1e-6);
        EXPECT_GT(profile.step_num, 0);
    }
}

// 测试累计弧长数组的正确性：对一条已知间距的直线点序列，累计距离应为间距的整数倍。
TEST(PathToOcpConverterTest, BuildCumulativeArcLengthMatchesKnownSpacing) {
    std::vector<PathPoint> points{PathPoint(0.0, 0.0, 0.0), PathPoint(1.0, 0.0, 0.0),
                             PathPoint(2.5, 0.0, 0.0)};
    const auto cumulative =
        TestablePathToOcpConverter::buildCumulativeArcLength(points);
    ASSERT_EQ(cumulative.size(), 3U);
    EXPECT_DOUBLE_EQ(cumulative[0], 0.0);
    EXPECT_DOUBLE_EQ(cumulative[1], 1.0);
    EXPECT_DOUBLE_EQ(cumulative[2], 2.5);
}

// 测试按弧长插值能在两点间正确线性插值x/y/theta/kappa。
// 因为这是把三次多项式速度剖面的时间-弧长映射转换为具体位姿的关键环节。
TEST(PathToOcpConverterTest, InterpolateAtArcLengthLinearlyBlendsPose) {
    std::vector<PathPoint> points{PathPoint(0.0, 0.0, 0.0), PathPoint(2.0, 0.0, PI / 2.0)};
    points[0].setKappa(0.1);
    points[1].setKappa(0.3);
    const auto cumulative =
        TestablePathToOcpConverter::buildCumulativeArcLength(points);

    const auto mid =
        TestablePathToOcpConverter::interpolateAtArcLength(points, cumulative, 1.0);
    EXPECT_NEAR(mid.x, 1.0, 1e-9);
    EXPECT_NEAR(mid.y, 0.0, 1e-9);
    EXPECT_NEAR(mid.theta, PI / 4.0, 1e-9);
    EXPECT_NEAR(mid.getKappa(), 0.2, 1e-9);

    const auto clamped_low =
        TestablePathToOcpConverter::interpolateAtArcLength(points, cumulative, -1.0);
    EXPECT_NEAR(clamped_low.x, 0.0, 1e-9);
    const auto clamped_high =
        TestablePathToOcpConverter::interpolateAtArcLength(points, cumulative, 100.0);
    EXPECT_NEAR(clamped_high.x, 2.0, 1e-9);
}

// 测试单个直线前进机动段重采样后的首末状态：起点速度/转角为0，终点速度回到0，
// 直线曲率恒为0使前轮转角全程为0。
TEST(PathToOcpConverterTest, SampleManeuverProducesConsistentStraightForwardStates) {
    Path path;
    for (double x = 0.0; x <= 5.0 + EPSILON; x += 0.1) {
        path.addPoint(Pose(std::min(x, 5.0), 0.0, 0.0));
    }
    ASSERT_EQ(path.numManeuvers(), 1U);
    TestablePathToOcpConverter converter(MakeVehicleParams());
    const auto samples = converter.sampleManeuver(path.getManeuvers().front());

    ASSERT_GT(samples.states.size(), 1U);
    EXPECT_EQ(samples.controls.size(), samples.states.size() - 1);
    const auto& first = samples.states.front();
    const auto& last = samples.states.back();
    EXPECT_NEAR(first(0), 0.0, 1e-6);
    EXPECT_NEAR(first(3), 0.0, 1e-6);
    EXPECT_NEAR(last(0), 5.0, 1e-6);
    EXPECT_NEAR(last(3), 0.0, 1e-6);
    for (const auto& state : samples.states) {
        EXPECT_NEAR(state(4), 0.0, 1e-9) << "straight line delta should be zero";
    }
}

// 测试完整换挡场景（前进5m后倒车3m）转换出的MultiStageOCP结构与初始猜测轨迹尺寸。
// 因为这是M2的核心交付：验证段数/换挡检测/全局轨迹长度与首末状态均符合预期。
TEST(PathToOcpConverterTest, ConvertBuildsTwoSegmentSwitchbackScenario) {
    const auto path = MakeStraightLineSwitchbackPath();
    PathToOcpConverter converter(MakeVehicleParams());
    const auto result = converter.convert(path);

    ASSERT_EQ(result.ocp.segments().size(), 2U);
    EXPECT_TRUE(result.ocp.hasGearShift());
    EXPECT_DOUBLE_EQ(result.ocp.segments()[0].v_sign, 1.0);
    EXPECT_DOUBLE_EQ(result.ocp.segments()[1].v_sign, -1.0);

    const int total_steps = result.ocp.totalSteps();
    EXPECT_EQ(static_cast<int>(result.init_guess.x.size()), total_steps + 1);
    EXPECT_EQ(static_cast<int>(result.init_guess.u.size()), total_steps);

    std::string reason;
    EXPECT_TRUE(result.ocp.validate(&reason)) << reason;

    const auto& first_state = result.init_guess.x.front();
    EXPECT_NEAR(first_state(0), 0.0, 1e-6);
    EXPECT_NEAR(first_state(3), 0.0, 1e-6);

    const int forward_n = result.ocp.segments()[0].N;
    const auto& gear_shift_state = result.init_guess.x[forward_n];
    EXPECT_NEAR(gear_shift_state(0), 5.0, 1e-6);
    EXPECT_NEAR(gear_shift_state(3), 0.0, 1e-6)
        << "velocity at gear shift boundary should return to zero";

    const auto& last_state = result.init_guess.x.back();
    EXPECT_NEAR(last_state(0), 2.0, 1e-6);
    EXPECT_NEAR(last_state(3), 0.0, 1e-6);
}

// 测试空Path被拒绝：转换器不应对空输入产生任何有意义的OCP。
TEST(PathToOcpConverterTest, ConvertRejectsEmptyPath) {
    PathToOcpConverter converter(MakeVehicleParams());
    Path empty_path;
    EXPECT_THROW(converter.convert(empty_path), std::invalid_argument);
}

// 测试包含PIVOT机动段的Path被拒绝：当前版本明确不支持原地转向，需显式报错而非静默处理。
TEST(PathToOcpConverterTest, ConvertRejectsPivotManeuver) {
    Path path;
    path.addPoint(Pose(0.0, 0.0, 0.0));
    path.addPoint(Pose(0.5, 0.0, 0.0));
    path.addPoint(Pose(1.0, 0.0, 0.0));
    // 原地大角度旋转：位移几乎为零但航向变化远超PIVOT阈值，触发PIVOT机动段
    path.addPoint(Pose(1.0, 0.0, PI / 2.0));

    ASSERT_GE(path.numManeuvers(), 2U);
    bool has_pivot = false;
    for (const auto& maneuver : path.getManeuvers()) {
        has_pivot = has_pivot || (maneuver.direction == Direction::PIVOT);
    }
    ASSERT_TRUE(has_pivot) << "test setup must actually produce a PIVOT maneuver";

    PathToOcpConverter converter(MakeVehicleParams());
    EXPECT_THROW(converter.convert(path), std::invalid_argument);
}

// 测试代价权重结构：内部机动段只在v/delta施加小权重且x/y/theta不参与跟踪，
// 终端机动段则对x/y/theta/v/delta全部施加跟踪权重且x_ref等于本段末端状态。
// 因为这是删除占位跟踪代价后的核心行为变更，必须有单测直接验证权重矩阵本身，
// 而不能只依赖间接的状态断言。
TEST(PathToOcpConverterTest, BuildSegmentUsesInteriorWeightsForNonTerminalSegment) {
    const auto path = MakeStraightLineSwitchbackPath();
    ASSERT_EQ(path.numManeuvers(), 2U);

    TestablePathToOcpConverter converter(MakeVehicleParams());
    const auto& forward_maneuver = path.getManeuvers().front();
    const auto samples = converter.sampleManeuver(forward_maneuver);
    auto dynamics = std::make_shared<stc_SQP::BicycleModelDelta>(2.7);
    const auto segment =
        converter.buildSegment(forward_maneuver, dynamics, samples, /*is_terminal_segment=*/false);

    const auto* cost =
        dynamic_cast<const stc_SQP::QuadraticTrackingCost*>(segment.cost.get());
    ASSERT_NE(cost, nullptr);
    EXPECT_DOUBLE_EQ(cost->Q()(0, 0), 0.0);
    EXPECT_DOUBLE_EQ(cost->Q()(1, 1), 0.0);
    EXPECT_DOUBLE_EQ(cost->Q()(2, 2), 0.0);
    EXPECT_GT(cost->Q()(3, 3), 0.0);
    EXPECT_GT(cost->Q()(4, 4), 0.0);
    EXPECT_GT(cost->R()(0, 0), 0.0);
    EXPECT_GT(cost->R()(1, 1), 0.0);
}

TEST(PathToOcpConverterTest, BuildSegmentUsesTerminalWeightsForLastSegment) {
    Path path;
    for (double x = 0.0; x <= 5.0 + EPSILON; x += 0.1) {
        path.addPoint(Pose(std::min(x, 5.0), 0.0, 0.0));
    }

    TestablePathToOcpConverter converter(MakeVehicleParams());
    const auto& maneuver = path.getManeuvers().front();
    const auto samples = converter.sampleManeuver(maneuver);
    auto dynamics = std::make_shared<stc_SQP::BicycleModelDelta>(2.7);
    const auto segment =
        converter.buildSegment(maneuver, dynamics, samples, /*is_terminal_segment=*/true);

    const auto* cost =
        dynamic_cast<const stc_SQP::QuadraticTrackingCost*>(segment.cost.get());
    ASSERT_NE(cost, nullptr);
    EXPECT_GT(cost->Q()(0, 0), 0.0);
    EXPECT_GT(cost->Q()(1, 1), 0.0);
    EXPECT_GT(cost->Q()(2, 2), 0.0);
    EXPECT_GT(cost->Q()(3, 3), 0.0);
    EXPECT_GT(cost->Q()(4, 4), 0.0);
    EXPECT_TRUE(cost->xRef().isApprox(samples.states.back()));
}

// 使用data/test.json回归样例做端到端集成测试：验证转换器能处理真实车辆参数与真实
// 多机动段初始路径，产出结构完整（段数与Path一致、可validate通过）的MultiStageOCP。
using PathToOcpConverterIntegrationTest = DataJsonFixture;

TEST_F(PathToOcpConverterIntegrationTest, ConvertsRealRegressionSampleWithoutThrowing) {
    const auto& request = getOptimizeRequest();
    const auto path = Path::FromProto(request.initial_path());
    const auto vehicle_params = VehicleParams::FromProto(request.vehicle());

    PathToOcpConverter converter(vehicle_params);
    PathToOcpConverter::Result result;
    ASSERT_NO_THROW(result = converter.convert(path));

    EXPECT_EQ(result.ocp.segments().size(), path.numManeuvers());
    const int total_steps = result.ocp.totalSteps();
    EXPECT_EQ(static_cast<int>(result.init_guess.x.size()), total_steps + 1);
    EXPECT_EQ(static_cast<int>(result.init_guess.u.size()), total_steps);

    std::string reason;
    EXPECT_TRUE(result.ocp.validate(&reason)) << reason;
    for (const auto& state : result.init_guess.x) {
        EXPECT_TRUE(state.allFinite());
    }
}

}  // namespace apa_post_processor
