#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include "core/MINCO/minco_trajectory_sampler.h"
#include "core/MINCO/bicycle_kinematics_extractor.h"
#include "util/maneuver.h"

namespace apa_post_processor {
namespace {

// 人造机动段描述：单段多项式构成的 Maneuver（方向/朝向增量/弧长增量/时长）
struct ManeuverSpec {
    Direction direction;
    double delta_theta;
    double delta_s;
    double duration;
};

// 人造场景：MincoTrajectory（K(T) 边界精确满足，段连接处 θ/s 即构造值）与
// 对应的 Maneuver 结构估计。外层 vector 为 Maneuver、内层为其多项式段
// （一个 Maneuver 可跨多段），起点边界取 θ=0/s=0、零速零加速度；终点
// 边界同理静止
struct FabricatedScene {
    MincoTrajectory trajectory;
    std::vector<MincoManeuverEstimate> estimates;
};

FabricatedScene Fabricate(
    const std::vector<std::vector<ManeuverSpec>>& maneuver_specs) {
    FabricatedScene scene;
    MincoBoundaryCondition2d start;
    MincoBoundaryCondition2d end;
    std::vector<Eigen::Vector2d> waypoints;
    std::vector<double> durations;
    double theta = 0.0;
    double arc_length = 0.0;
    int total_segments = 0;
    for (const auto& segment_specs : maneuver_specs) {
        total_segments += static_cast<int>(segment_specs.size());
    }
    int global_index = 0;
    for (const auto& segment_specs : maneuver_specs) {
        MincoManeuverEstimate estimate;
        estimate.direction = segment_specs.front().direction;
        estimate.start_theta = theta;
        estimate.start_arc_length = arc_length;
        for (const auto& spec : segment_specs) {
            theta += spec.delta_theta;
            arc_length += spec.delta_s;
            // 离散化只消费轨迹真实值与段结构/方向，期望位置不参与本模块，置零占位
            estimate.segments.push_back(
                {{0.0, 0.0}, theta, arc_length, spec.duration});
            durations.push_back(spec.duration);
            ++global_index;
            if (global_index < total_segments) {
                waypoints.emplace_back(theta, arc_length);
            }
        }
        scene.estimates.push_back(estimate);
    }
    end.theta = {theta, 0.0, 0.0};
    end.s = {arc_length, 0.0, 0.0};
    scene.trajectory.setTrajectory(start, end, waypoints, durations);
    return scene;
}

// 便捷重载：每个 Maneuver 恰含 1 个多项式段
FabricatedScene Fabricate(const std::vector<ManeuverSpec>& specs) {
    std::vector<std::vector<ManeuverSpec>> grouped;
    grouped.reserve(specs.size());
    for (const auto& spec : specs) {
        grouped.push_back({spec});
    }
    return Fabricate(grouped);
}

// 与车辆物理参数一致的运动学提取器（状态量解析只依赖提取器本身）
BicycleKinematicsExtractor MakeKinematics() {
    MincoConfig config;
    config.wheelbase = 2.7;
    return BicycleKinematicsExtractor(config);
}

// 测试固定的每段采样数：点数断言与积分容差均按该值标定，不随默认值漂移
constexpr int kSamplesPerSegment = 8;

// 测试单 Maneuver（跨 2 个多项式段）的离散化结果与轨迹精确值一致。
// 半开区间采样 + 全局终点补采样：点数 = 段数 × 每段采样数 + 1；首点锚定
// 起点世界坐标；末点 θ/s 取轨迹全局终点的精确值，世界坐标由梯形积分还原，
// 与构造位移的偏差应在积分截断误差量级内（直线构造下两端 s̈=0，梯形主误差
// 项相消，0.02 m 容差足以钉住符号/锚点/量级错误）。
TEST(MincoTrajectorySamplerTest, SingleManeuverSamplingMatchesTrajectory) {
    const FabricatedScene scene = Fabricate({
        {{{Direction::FORWARD, 0.0, 1.0, 2.0},
          {Direction::FORWARD, 0.0, 1.0, 2.0}}},
    });
    const BicycleKinematicsExtractor kinematics = MakeKinematics();
    const std::vector<Maneuver> maneuvers =
        SampleMincoTrajectory(scene.trajectory, scene.estimates, {1.0, 2.0},
                              kinematics, kSamplesPerSegment);
    ASSERT_EQ(maneuvers.size(), 1U);
    EXPECT_EQ(maneuvers[0].direction, Direction::FORWARD);
    ASSERT_EQ(maneuvers[0].points.size(),
              2U * static_cast<std::size_t>(kSamplesPerSegment) + 1U);
    // 首点锚定起点世界坐标（尚未推进积分）
    EXPECT_DOUBLE_EQ(maneuvers[0].points.front().x, 1.0);
    EXPECT_DOUBLE_EQ(maneuvers[0].points.front().y, 2.0);
    // 全部采样点携带完整状态/控制量，且朝向与构造值一致（θ 恒为 0，直线
    // 前进时世界 y 恒为起点 y）
    for (const auto& point : maneuvers[0].points) {
        EXPECT_DOUBLE_EQ(point.theta, 0.0);
        EXPECT_DOUBLE_EQ(point.y, 2.0);
        EXPECT_TRUE(point.hasV());
        EXPECT_TRUE(point.hasA());
        EXPECT_TRUE(point.hasDelta());
        EXPECT_TRUE(point.hasDeltaDot());
    }
    // 末点为轨迹全局终点：θ 取轨迹精确值、端点速度为 0（K(T) 边界精确满足）
    const auto& last = maneuvers[0].points.back();
    EXPECT_DOUBLE_EQ(
        last.theta,
        scene.trajectory.evaluateSegment(1, scene.trajectory.duration(1), 0)
            .x());
    EXPECT_NEAR(last.getV(), 0.0, 1e-9);
    // 梯形积分还原的终点 x ≈ 起点 x + 构造位移 2.0
    EXPECT_NEAR(last.x, 3.0, 0.02);
    // 时间戳：首点 t=0，末点 t=轨迹总时长（2+2=4s），逐点严格递增——
    // θ-s 轨迹本身是时间参数化的多项式，采样点携带真实时刻后运动学
    // 可行性校验（梯形配点残差）才能对 MINCO 产出真正生效
    EXPECT_TRUE(maneuvers[0].points.front().hasT());
    EXPECT_DOUBLE_EQ(maneuvers[0].points.front().getT(), 0.0);
    EXPECT_DOUBLE_EQ(last.getT(), 4.0);
    for (std::size_t j = 1; j < maneuvers[0].points.size(); ++j) {
        EXPECT_GT(maneuvers[0].points[j].getT(),
                  maneuvers[0].points[j - 1].getT());
    }
}

// 测试换挡场景（前进 → 后退）的带符号弧长积分：世界坐标必须按 ṡ 的符号
// 反向推进，而非按 |Δs| 单向累积——若实现错误地用无符号弧长积分，全局终点
// 会落在 1.5 m 而非 0.5 m 附近。
TEST(MincoTrajectorySamplerTest, GearShiftManeuverIntegratesSignedArcLength) {
    const FabricatedScene scene = Fabricate({
        {Direction::FORWARD, 0.0, 1.0, 2.0},
        {Direction::BACKWARD, 0.0, -0.5, 1.5},
    });
    const BicycleKinematicsExtractor kinematics = MakeKinematics();
    const std::vector<Maneuver> maneuvers =
        SampleMincoTrajectory(scene.trajectory, scene.estimates, {0.0, 0.0},
                              kinematics, kSamplesPerSegment);
    ASSERT_EQ(maneuvers.size(), 2U);
    EXPECT_EQ(maneuvers[0].direction, Direction::FORWARD);
    EXPECT_EQ(maneuvers[1].direction, Direction::BACKWARD);
    // 点数：首 Maneuver 段数×采样数（半开区间）；末 Maneuver 额外 +1 全局终点
    ASSERT_EQ(maneuvers[0].points.size(),
              static_cast<std::size_t>(kSamplesPerSegment));
    ASSERT_EQ(maneuvers[1].points.size(),
              static_cast<std::size_t>(kSamplesPerSegment) + 1U);
    // 后退 Maneuver 的净位移必须反向（沿 θ=0 后退 0.5 m，远大于积分截断误差）
    EXPECT_LT(maneuvers[1].points.back().x, maneuvers[1].points.front().x);
    // 全局终点 ≈ 起点 + (1.0 - 0.5)
    EXPECT_NEAR(maneuvers[1].points.back().x, 0.5, 0.02);
}

// 测试多 Maneuver 场景的方向保留与点数分配：方向直接取自估计（离散化不做
// 任何方向推断/改写），只有最后一个 Maneuver 携带全局终点补采样点。
TEST(MincoTrajectorySamplerTest, MultiManeuverCountsAndDirectionsPreserved) {
    const FabricatedScene scene = Fabricate({
        {Direction::FORWARD, 0.0, 1.0, 2.0},
        {Direction::BACKWARD, 0.0, -0.5, 1.5},
        {Direction::FORWARD, 0.0, 1.0, 2.0},
    });
    const BicycleKinematicsExtractor kinematics = MakeKinematics();
    const std::vector<Maneuver> maneuvers =
        SampleMincoTrajectory(scene.trajectory, scene.estimates, {0.0, 0.0},
                              kinematics, kSamplesPerSegment);
    ASSERT_EQ(maneuvers.size(), 3U);
    EXPECT_EQ(maneuvers[0].direction, Direction::FORWARD);
    EXPECT_EQ(maneuvers[1].direction, Direction::BACKWARD);
    EXPECT_EQ(maneuvers[2].direction, Direction::FORWARD);
    EXPECT_EQ(maneuvers[0].points.size(),
              static_cast<std::size_t>(kSamplesPerSegment));
    EXPECT_EQ(maneuvers[1].points.size(),
              static_cast<std::size_t>(kSamplesPerSegment));
    ASSERT_EQ(maneuvers[2].points.size(),
              static_cast<std::size_t>(kSamplesPerSegment) + 1U);
    // 全局终点 ≈ 起点 + (1.0 - 0.5 + 1.0)
    EXPECT_NEAR(maneuvers[2].points.back().x, 1.5, 0.02);
}

// 测试同一工具多次调用互不干扰（无隐藏状态残留）：同一份 estimates 分别
// 离散化两条段结构一致但数值不同的 MincoTrajectory，两次结果各自与对应
// 轨迹一致；对同一条轨迹重复离散化必须逐位一致（纯函数语义）。
TEST(MincoTrajectorySamplerTest, RepeatedCallsArePureAndIndependent) {
    const FabricatedScene scene_a = Fabricate({
        {{{Direction::FORWARD, 0.0, 1.0, 2.0},
          {Direction::FORWARD, 0.0, 1.0, 2.0}}},
    });
    // 与 scene_a 段结构完全一致（1 Maneuver × 2 段、相同时长），仅位移不同
    const FabricatedScene scene_b = Fabricate({
        {{{Direction::FORWARD, 0.0, 2.0, 2.0},
          {Direction::FORWARD, 0.0, 2.0, 2.0}}},
    });
    const BicycleKinematicsExtractor kinematics = MakeKinematics();
    const std::vector<Maneuver> result_a1 =
        SampleMincoTrajectory(scene_a.trajectory, scene_a.estimates, {0.0, 0.0},
                              kinematics, kSamplesPerSegment);
    const std::vector<Maneuver> result_b =
        SampleMincoTrajectory(scene_b.trajectory, scene_a.estimates, {0.0, 0.0},
                              kinematics, kSamplesPerSegment);
    const std::vector<Maneuver> result_a2 =
        SampleMincoTrajectory(scene_a.trajectory, scene_a.estimates, {0.0, 0.0},
                              kinematics, kSamplesPerSegment);
    // 不同轨迹各自正确离散化：终点位移分别为 2.0 与 4.0
    ASSERT_EQ(result_a1.size(), 1U);
    ASSERT_EQ(result_b.size(), 1U);
    EXPECT_NEAR(result_a1[0].points.back().x, 2.0, 0.02);
    EXPECT_NEAR(result_b[0].points.back().x, 4.0, 0.02);
    EXPECT_GT(result_b[0].points.back().x - result_a1[0].points.back().x, 1.5);
    // 重复调用逐位一致：状态残留会导致第二次调用积分锚点/结果漂移
    ASSERT_EQ(result_a1.size(), result_a2.size());
    for (std::size_t m = 0; m < result_a1.size(); ++m) {
        ASSERT_EQ(result_a1[m].points.size(), result_a2[m].points.size());
        EXPECT_EQ(result_a1[m].direction, result_a2[m].direction);
        for (std::size_t j = 0; j < result_a1[m].points.size(); ++j) {
            const TrajectoryPoint& p = result_a1[m].points[j];
            const TrajectoryPoint& q = result_a2[m].points[j];
            EXPECT_DOUBLE_EQ(p.x, q.x);
            EXPECT_DOUBLE_EQ(p.y, q.y);
            EXPECT_DOUBLE_EQ(p.theta, q.theta);
            EXPECT_DOUBLE_EQ(p.getV(), q.getV());
            EXPECT_DOUBLE_EQ(p.getA(), q.getA());
            EXPECT_DOUBLE_EQ(p.getDelta(), q.getDelta());
            EXPECT_DOUBLE_EQ(p.getDeltaDot(), q.getDeltaDot());
        }
    }
}

// 测试非法输入的异常反馈：估计与轨迹的段数结构必须一致（离散化按估计的
// 段结构索引轨迹段），空估计、空 Maneuver、段数不一致、非有限起点、非法
// 采样数都必须以标准异常明确拒绝；合法输入不抛。
TEST(MincoTrajectorySamplerTest, InvalidInputsThrow) {
    const FabricatedScene scene = Fabricate({
        {Direction::FORWARD, 0.0, 1.0, 2.0},
        {Direction::BACKWARD, 0.0, -0.5, 1.5},
    });
    const BicycleKinematicsExtractor kinematics = MakeKinematics();
    EXPECT_NO_THROW(SampleMincoTrajectory(scene.trajectory, scene.estimates,
                                          {0.0, 0.0}, kinematics,
                                          kSamplesPerSegment));
    // 空估计
    EXPECT_THROW(SampleMincoTrajectory(scene.trajectory, {}, {0.0, 0.0},
                                       kinematics, kSamplesPerSegment),
                 std::invalid_argument);
    // 空 segments 的 Maneuver
    EXPECT_THROW(
        SampleMincoTrajectory(scene.trajectory, {MincoManeuverEstimate{}},
                              {0.0, 0.0}, kinematics, kSamplesPerSegment),
        std::invalid_argument);
    // 段数不一致：估计 1 段 vs 轨迹 2 段
    std::vector<MincoManeuverEstimate> mismatched = scene.estimates;
    mismatched.pop_back();
    EXPECT_THROW(SampleMincoTrajectory(scene.trajectory, mismatched, {0.0, 0.0},
                                       kinematics, kSamplesPerSegment),
                 std::invalid_argument);
    // 非有限起点世界坐标
    EXPECT_THROW(
        SampleMincoTrajectory(scene.trajectory, scene.estimates,
                              {std::numeric_limits<double>::quiet_NaN(), 0.0},
                              kinematics, kSamplesPerSegment),
        std::invalid_argument);
    EXPECT_THROW(
        SampleMincoTrajectory(scene.trajectory, scene.estimates,
                              {std::numeric_limits<double>::infinity(), 0.0},
                              kinematics, kSamplesPerSegment),
        std::invalid_argument);
    // 非法采样数
    EXPECT_THROW(SampleMincoTrajectory(scene.trajectory, scene.estimates,
                                       {0.0, 0.0}, kinematics, 1),
                 std::invalid_argument);
    EXPECT_THROW(SampleMincoTrajectory(scene.trajectory, scene.estimates,
                                       {0.0, 0.0}, kinematics, 0),
                 std::invalid_argument);
}

// 测试段结构校验工具本身：合法结构不抛，三类非法结构（空估计/空 Maneuver/
// 段数不一致）均以 std::invalid_argument 明确拒绝。
TEST(MincoTrajectorySamplerTest, StructureCheckRejectsInconsistentEstimates) {
    const FabricatedScene scene = Fabricate({
        {Direction::FORWARD, 0.0, 1.0, 2.0},
        {Direction::BACKWARD, 0.0, -0.5, 1.5},
    });
    EXPECT_NO_THROW(
        CheckMincoSampleStructure(scene.trajectory, scene.estimates));
    EXPECT_THROW(CheckMincoSampleStructure(scene.trajectory, {}),
                 std::invalid_argument);
    EXPECT_THROW(
        CheckMincoSampleStructure(scene.trajectory, {MincoManeuverEstimate{}}),
        std::invalid_argument);
    std::vector<MincoManeuverEstimate> mismatched = scene.estimates;
    mismatched.pop_back();
    EXPECT_THROW(CheckMincoSampleStructure(scene.trajectory, mismatched),
                 std::invalid_argument);
}

}  // namespace
}  // namespace apa_post_processor
