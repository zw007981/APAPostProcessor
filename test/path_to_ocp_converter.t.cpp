#include "core/NMPC/path_to_ocp_converter.h"

#include <costs/quadratic_tracking.h>
#include <gtest/gtest.h>
#include <models/bicycle_model_delta.h>

#include <cmath>
#include <stdexcept>

#include "test_fixture_util.h"
#include "util/constants.h"
#include "util/path.h"
#include "util/trajectory_point.h"
#include "vehicle/vehicle_params.h"

namespace apa_post_processor {
namespace {

// 可测试子类：通过继承暴露受保护的辅助方法，便于对速度剖面/弧长插值等纯计算逻辑做白盒测试。
class TestablePathToOcpConverter : public PathToOcpConverter {
   public:
    using PathToOcpConverter::buildCumulativeArcLength;
    using PathToOcpConverter::buildOcp;
    using PathToOcpConverter::buildSegment;
    using PathToOcpConverter::buildVelocityProfile;
    using PathToOcpConverter::computeSegmentProfiles;
    using PathToOcpConverter::generateInitialGuess;
    using PathToOcpConverter::interpolateAtArcLength;
    using PathToOcpConverter::PathToOcpConverter;
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

// 由单个前进机动段构造Path
Path MakeSingleForwardPath(double length) {
    Path path;
    for (double x = 0.0; x <= length + EPSILON; x += 0.1) {
        path.addPoint(Pose(std::min(x, length), 0.0, 0.0));
    }
    return path;
}

// 构造一个均匀的SegmentProfile（默认dt场景），用于白盒测试sampleManeuver/buildSegment
SegmentProfile MakeUniformForwardProfile(double arc_length) {
    PathToOcpConfig config;
    const double tf_ideal = 1.5 * arc_length / config.target_peak_speed;
    const int step_num =
        std::max(1, static_cast<int>(std::llround(tf_ideal / config.dt)));
    SegmentProfile profile;
    profile.N = step_num;
    profile.dt = step_num * config.dt / step_num;  // 即 config.dt
    profile.v_sign = 1.0;
    profile.is_terminal = false;
    return profile;
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
    EXPECT_THROW(PathToOcpConverter(valid_params, bad_dt),
                 std::invalid_argument);

    PathToOcpConfig bad_slack;
    bad_slack.boundary_velocity_slack = -0.01;
    EXPECT_THROW(PathToOcpConverter(valid_params, bad_slack),
                 std::invalid_argument);
}

// 测试三次多项式速度剖面满足边界条件：s(0)=0, v(0)=0, s(tf)约等于弧长,
// v(tf)约等于0。
// 因为这是初始猜测构造的数学基础，边界条件不满足会导致重采样弧长与实际路径长度不一致。
TEST(PathToOcpConverterTest, VelocityProfileSatisfiesBoundaryConditions) {
    TestablePathToOcpConverter converter(MakeVehicleParams());
    for (double arc_length : {0.5, 2.0, 5.0, 12.3}) {
        const auto profile = converter.buildVelocityProfile(arc_length);
        const double s0 = profile.b * 0.0 + profile.c * 0.0;
        const double v0 = 0.0;
        const double tf = profile.tf;
        const double s_tf = profile.b * tf * tf * tf + profile.c * tf * tf;
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
    std::vector<TrajectoryPoint> points{TrajectoryPoint(0.0, 0.0, 0.0),
                                        TrajectoryPoint(1.0, 0.0, 0.0),
                                        TrajectoryPoint(2.5, 0.0, 0.0)};
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
    std::vector<TrajectoryPoint> points{TrajectoryPoint(0.0, 0.0, 0.0),
                                        TrajectoryPoint(2.0, 0.0, PI / 2.0)};
    points[0].setKappa(0.1);
    points[1].setKappa(0.3);
    const auto cumulative =
        TestablePathToOcpConverter::buildCumulativeArcLength(points);

    const auto mid = TestablePathToOcpConverter::interpolateAtArcLength(
        points, cumulative, 1.0);
    EXPECT_NEAR(mid.x, 1.0, 1e-9);
    EXPECT_NEAR(mid.y, 0.0, 1e-9);
    EXPECT_NEAR(mid.theta, PI / 4.0, 1e-9);
    EXPECT_NEAR(mid.getKappa(), 0.2, 1e-9);

    const auto clamped_low = TestablePathToOcpConverter::interpolateAtArcLength(
        points, cumulative, -1.0);
    EXPECT_NEAR(clamped_low.x, 0.0, 1e-9);
    const auto clamped_high =
        TestablePathToOcpConverter::interpolateAtArcLength(points, cumulative,
                                                           100.0);
    EXPECT_NEAR(clamped_high.x, 2.0, 1e-9);
}

// 测试单个直线前进机动段重采样后的首末状态：起点速度/转角为0，终点速度回到0，
// 直线曲率恒为0使前轮转角全程为0。
TEST(PathToOcpConverterTest,
     SampleManeuverProducesConsistentStraightForwardStates) {
    const auto path = MakeSingleForwardPath(5.0);
    ASSERT_EQ(path.numManeuvers(), 1U);
    TestablePathToOcpConverter converter(MakeVehicleParams());
    const auto profile = MakeUniformForwardProfile(5.0);
    const auto velocity_profile = converter.buildVelocityProfile(5.0);
    const auto samples = converter.sampleManeuver(path.getManeuvers().front(),
                                                  profile, velocity_profile);

    ASSERT_GT(samples.states.size(), 1U);
    EXPECT_EQ(samples.controls.size(), samples.states.size() - 1);
    const auto& first = samples.states.front();
    const auto& last = samples.states.back();
    EXPECT_NEAR(first(0), 0.0, 1e-6);
    EXPECT_NEAR(first(3), 0.0, 1e-6);
    EXPECT_NEAR(last(0), 5.0, 1e-6);
    EXPECT_NEAR(last(3), 0.0, 1e-6);
    for (const auto& state : samples.states) {
        EXPECT_NEAR(state(4), 0.0, 1e-9)
            << "straight line delta should be zero";
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
    ASSERT_TRUE(has_pivot)
        << "test setup must actually produce a PIVOT maneuver";

    PathToOcpConverter converter(MakeVehicleParams());
    EXPECT_THROW(converter.convert(path), std::invalid_argument);
}

// 测试职责拆分后的三个公开入口：computeSegmentProfiles、generateInitialGuess、buildOcp
// 组合起来与convert()产出等价。
TEST(PathToOcpConverterTest, SplitEntryPointsReproduceConvertResult) {
    const auto path = MakeStraightLineSwitchbackPath();
    PathToOcpConverter converter(MakeVehicleParams());
    const auto convert_result = converter.convert(path);

    const auto profiles = converter.computeSegmentProfiles(path);
    const auto init_guess = converter.generateInitialGuess(path, profiles);
    const auto ocp = converter.buildOcp(path, profiles, init_guess);

    ASSERT_EQ(ocp.segments().size(), convert_result.ocp.segments().size());
    EXPECT_EQ(ocp.totalSteps(), convert_result.ocp.totalSteps());
    EXPECT_EQ(init_guess.x.size(), convert_result.init_guess.x.size());
    EXPECT_EQ(init_guess.u.size(), convert_result.init_guess.u.size());

    // 验证OCP结构一致：每段N/dt/v_sign/边界/x_ref/stage_params
    for (std::size_t i = 0; i < ocp.segments().size(); ++i) {
        const auto& seg_split = ocp.segments()[i];
        const auto& seg_convert = convert_result.ocp.segments()[i];
        EXPECT_EQ(seg_split.N, seg_convert.N);
        EXPECT_DOUBLE_EQ(seg_split.dt, seg_convert.dt);
        EXPECT_DOUBLE_EQ(seg_split.v_sign, seg_convert.v_sign);
        EXPECT_TRUE(seg_split.x_min.isApprox(seg_convert.x_min));
        EXPECT_TRUE(seg_split.x_max.isApprox(seg_convert.x_max));
        EXPECT_TRUE(seg_split.u_min.isApprox(seg_convert.u_min));
        EXPECT_TRUE(seg_split.u_max.isApprox(seg_convert.u_max));
        ASSERT_EQ(seg_split.stage_params.size(),
                  seg_convert.stage_params.size());
        for (std::size_t k = 0; k < seg_split.stage_params.size(); ++k) {
            EXPECT_DOUBLE_EQ(seg_split.stage_params[k].p(0),
                             seg_convert.stage_params[k].p(0));
            EXPECT_DOUBLE_EQ(seg_split.stage_params[k].p(1),
                             seg_convert.stage_params[k].p(1));
        }
    }

    // 验证初始猜测一致
    for (std::size_t i = 0; i < init_guess.x.size(); ++i) {
        EXPECT_TRUE(init_guess.x[i].isApprox(convert_result.init_guess.x[i]));
    }
    for (std::size_t i = 0; i < init_guess.u.size(); ++i) {
        EXPECT_TRUE(init_guess.u[i].isApprox(convert_result.init_guess.u[i]));
    }
}

// 测试buildOcp对ref_trajectory维度不匹配做fail-early校验。
TEST(PathToOcpConverterTest, BuildOcpRejectsMismatchedRefTrajectorySize) {
    const auto path = MakeSingleForwardPath(5.0);
    TestablePathToOcpConverter converter(MakeVehicleParams());
    const auto profiles = converter.computeSegmentProfiles(path);

    stc_SQP::Trajectory bad_ref;
    bad_ref.x.assign(2, stc_SQP::Vector::Zero(5));  // 维度不足
    EXPECT_THROW(converter.buildOcp(path, profiles, bad_ref),
                 std::invalid_argument);
}

// 测试buildOcp对profiles数量与机动段数量不匹配做fail-early校验。
TEST(PathToOcpConverterTest, BuildOcpRejectsMismatchedProfilesCount) {
    const auto path = MakeSingleForwardPath(5.0);
    TestablePathToOcpConverter converter(MakeVehicleParams());
    const auto profiles = converter.computeSegmentProfiles(path);
    ASSERT_EQ(profiles.size(), 1U);

    // 构造一个合法的参考轨迹（与convert()结果相同即可）。
    const auto ref = converter.generateInitialGuess(path, profiles);

    // profiles数量减半，触发不匹配校验。
    auto too_few = profiles;
    too_few.pop_back();
    EXPECT_THROW(converter.buildOcp(path, too_few, ref), std::invalid_argument);
}

// 测试generateInitialGuess对非均匀dt_array拒绝（当前默认生成器不支持）。
TEST(PathToOcpConverterTest, GenerateInitialGuessRejectsDtArray) {
    const auto path = MakeSingleForwardPath(5.0);
    TestablePathToOcpConverter converter(MakeVehicleParams());
    auto profiles = converter.computeSegmentProfiles(path);
    profiles.front().dt_array.assign(profiles.front().N, 0.05);
    EXPECT_THROW(converter.generateInitialGuess(path, profiles),
                 std::invalid_argument);
}

// 测试代价权重结构：内部机动段只在v/delta施加小权重且x/y/theta不参与跟踪，
// 终端机动段则对x/y/theta/v/delta全部施加跟踪权重且x_ref等于本段末端状态。
// 因为这是删除占位跟踪代价后的核心行为变更，必须有单测直接验证权重矩阵本身，
// 而不能只依赖间接的状态断言。
// Milestone 023 六次重构后默认会对内部段额外施加全程目标牵引代价
// （global_target_position_weight/global_target_heading_weight 默认 1e-3，
// 详见 docs/NMPC.md 6.10 节），因此本测试显式将其关闭为 0，
// 只验证"关闭全程目标牵引时"内部段权重结构本身，全程目标牵引的生效验证见
// 下方 BuildSegmentAppliesGlobalTargetPullToInteriorSegment。
TEST(PathToOcpConverterTest,
     BuildSegmentUsesInteriorWeightsForNonTerminalSegment) {
    const auto path = MakeStraightLineSwitchbackPath();
    ASSERT_EQ(path.numManeuvers(), 2U);

    PathToOcpConfig config;
    config.global_target_position_weight = 0.0;
    config.global_target_heading_weight = 0.0;
    TestablePathToOcpConverter converter(MakeVehicleParams(), config);
    const auto& forward_maneuver = path.getManeuvers().front();
    const auto profile = MakeUniformForwardProfile(forward_maneuver.length());
    auto dynamics = std::make_shared<stc_SQP::BicycleModelDelta>(2.7);
    std::vector<double> theta_refs(profile.N, 0.0);
    std::vector<double> x_refs(profile.N, 0.0);
    std::vector<double> y_refs(profile.N, 0.0);
    const auto segment =
        converter.buildSegment(forward_maneuver, dynamics, profile,
                               /*terminal_x_ref=*/stc_SQP::Vector::Zero(5),
                               /*global_target_x_ref=*/stc_SQP::Vector::Zero(5),
                               theta_refs, x_refs, y_refs);

    ASSERT_NE(segment.cost, nullptr);
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

// 测试全程目标牵引代价（Milestone 023 六次重构新增，参考论文 Eq.(10) J1）：
// 开启 global_target_position_weight/global_target_heading_weight 后，即使是
// 非终端段（内部机动段），x/y/theta 也应对 global_target_x_ref 施加二次跟踪
// 代价（而非像上一测试那样为 0），且 x_ref 应等于传入的常量目标值，与本段/
// 本步的局部参考轨迹无关。
TEST(PathToOcpConverterTest,
     BuildSegmentAppliesGlobalTargetPullToInteriorSegment) {
    const auto path = MakeStraightLineSwitchbackPath();
    ASSERT_EQ(path.numManeuvers(), 2U);

    PathToOcpConfig config;
    config.global_target_position_weight = 1e-3;
    config.global_target_heading_weight = 1e-3;
    TestablePathToOcpConverter converter(MakeVehicleParams(), config);
    const auto& forward_maneuver = path.getManeuvers().front();
    const auto profile = MakeUniformForwardProfile(forward_maneuver.length());
    auto dynamics = std::make_shared<stc_SQP::BicycleModelDelta>(2.7);
    std::vector<double> theta_refs(profile.N, 0.0);
    std::vector<double> x_refs(profile.N, 0.0);
    std::vector<double> y_refs(profile.N, 0.0);
    const stc_SQP::Vector global_target =
        (stc_SQP::Vector(5) << 3.0, 4.0, 0.5, 0.0, 0.0).finished();
    const auto segment =
        converter.buildSegment(forward_maneuver, dynamics, profile,
                               /*terminal_x_ref=*/stc_SQP::Vector::Zero(5),
                               global_target, theta_refs, x_refs, y_refs);

    ASSERT_NE(segment.cost, nullptr);
    const auto* cost =
        dynamic_cast<const stc_SQP::QuadraticTrackingCost*>(segment.cost.get());
    ASSERT_NE(cost, nullptr);
    EXPECT_DOUBLE_EQ(cost->Q()(0, 0), config.global_target_position_weight);
    EXPECT_DOUBLE_EQ(cost->Q()(1, 1), config.global_target_position_weight);
    EXPECT_DOUBLE_EQ(cost->Q()(2, 2), config.global_target_heading_weight);
    EXPECT_DOUBLE_EQ(cost->xRef()(0), global_target(0));
    EXPECT_DOUBLE_EQ(cost->xRef()(1), global_target(1));
    EXPECT_DOUBLE_EQ(cost->xRef()(2), global_target(2));
}

TEST(PathToOcpConverterTest, BuildSegmentUsesTerminalWeightsForLastSegment) {
    const auto path = MakeSingleForwardPath(5.0);

    TestablePathToOcpConverter converter(MakeVehicleParams());
    const auto& maneuver = path.getManeuvers().front();
    auto profile = MakeUniformForwardProfile(maneuver.length());
    profile.is_terminal = true;
    const auto velocity_profile =
        converter.buildVelocityProfile(maneuver.length());
    const auto samples =
        converter.sampleManeuver(maneuver, profile, velocity_profile);
    auto dynamics = std::make_shared<stc_SQP::BicycleModelDelta>(2.7);
    std::vector<double> theta_refs(profile.N, 0.0);
    std::vector<double> x_refs(profile.N, 0.0);
    std::vector<double> y_refs(profile.N, 0.0);
    const auto segment = converter.buildSegment(
        maneuver, dynamics, profile, samples.states.back(),
        /*global_target_x_ref=*/samples.states.back(), theta_refs, x_refs,
        y_refs);

    ASSERT_NE(segment.cost, nullptr);
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

// 测试buildSegment拒绝非法v_sign（非+1.0/-1.0），避免速度方向约束被错误解释。
TEST(PathToOcpConverterTest, BuildSegmentRejectsInvalidVSign) {
    const auto path = MakeSingleForwardPath(5.0);
    TestablePathToOcpConverter converter(MakeVehicleParams());
    const auto& maneuver = path.getManeuvers().front();
    auto profile = MakeUniformForwardProfile(maneuver.length());
    profile.v_sign = 0.0;  // 非法方向
    auto dynamics = std::make_shared<stc_SQP::BicycleModelDelta>(2.7);
    std::vector<double> theta_refs(profile.N, 0.0);
    std::vector<double> x_refs(profile.N, 0.0);
    std::vector<double> y_refs(profile.N, 0.0);
    EXPECT_THROW(converter.buildSegment(
                     maneuver, dynamics, profile, stc_SQP::Vector::Zero(5),
                     stc_SQP::Vector::Zero(5), theta_refs, x_refs, y_refs),
                 std::invalid_argument);
}

// 测试sampleManeuver拒绝与arc_length不一致的VelocityProfile，防止调用方自行构造
// 错误速度剖面导致重采样弧长漂移。
TEST(PathToOcpConverterTest, SampleManeuverRejectsInconsistentVelocityProfile) {
    const auto path = MakeSingleForwardPath(5.0);
    TestablePathToOcpConverter converter(MakeVehicleParams());
    const auto& maneuver = path.getManeuvers().front();
    const auto profile = MakeUniformForwardProfile(maneuver.length());
    auto bad_velocity_profile =
        converter.buildVelocityProfile(maneuver.length());
    bad_velocity_profile.b *= 2.0;  // 破坏s(tf)=arc_length的约束
    EXPECT_THROW(
        converter.sampleManeuver(maneuver, profile, bad_velocity_profile),
        std::invalid_argument);
}

// 使用data/test.json回归样例做端到端集成测试：验证转换器能处理真实车辆参数与真实
// 多机动段初始路径，产出结构完整（段数与Path一致、可validate通过）的MultiStageOCP。
using PathToOcpConverterIntegrationTest = DataJsonFixture;

TEST_F(PathToOcpConverterIntegrationTest,
       ConvertsRealRegressionSampleWithoutThrowing) {
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
