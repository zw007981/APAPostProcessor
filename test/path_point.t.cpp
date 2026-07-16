#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>

#include "util/pose.h"
#include "util/trajectory_point.h"

namespace apa_post_processor {
namespace {

// 测试 TrajectoryPoint 默认构造时所有派生量均未设置。
// 因为默认值统一用 NaN 表示"尚未提供"，所以几何量继承 Pose 的 0 初值，且五个
// hasXxx() 都应返回 false。
TEST(PathPointTest, DefaultConstructLeavesDerivedFieldsUnset) {
    const TrajectoryPoint point;

    EXPECT_DOUBLE_EQ(point.x, 0.0);
    EXPECT_DOUBLE_EQ(point.y, 0.0);
    EXPECT_DOUBLE_EQ(point.theta, 0.0);
    EXPECT_FALSE(point.hasKappa());
    EXPECT_FALSE(point.hasV());
    EXPECT_FALSE(point.hasDelta());
    EXPECT_FALSE(point.hasA());
    EXPECT_FALSE(point.hasDeltaDot());
}

// 测试在未设置派生量时调用 getter 会抛出异常。
// 因为未检查 has 就 get 是调用方逻辑错误，所以应抛出 std::logic_error。
TEST(PathPointTest, GettersThrowWhenFieldsAreNotSet) {
    const TrajectoryPoint point;

    EXPECT_THROW(point.getKappa(), std::logic_error);
    EXPECT_THROW(point.getV(), std::logic_error);
    EXPECT_THROW(point.getDelta(), std::logic_error);
    EXPECT_THROW(point.getA(), std::logic_error);
    EXPECT_THROW(point.getDeltaDot(), std::logic_error);
}

// 测试 setter 写入后对应派生量变为可用且取值正确。
// 因为各派生量独立管理，所以设置一个不应影响其他字段的未设置状态。
TEST(PathPointTest, SettersEnableCorrespondingGetters) {
    TrajectoryPoint point;
    point.setKappa(0.12);
    point.setV(1.5);
    point.setDelta(0.25);
    point.setA(-0.3);
    point.setDeltaDot(0.05);

    EXPECT_TRUE(point.hasKappa());
    EXPECT_DOUBLE_EQ(point.getKappa(), 0.12);
    EXPECT_TRUE(point.hasV());
    EXPECT_DOUBLE_EQ(point.getV(), 1.5);
    EXPECT_TRUE(point.hasDelta());
    EXPECT_DOUBLE_EQ(point.getDelta(), 0.25);
    EXPECT_TRUE(point.hasA());
    EXPECT_DOUBLE_EQ(point.getA(), -0.3);
    EXPECT_TRUE(point.hasDeltaDot());
    EXPECT_DOUBLE_EQ(point.getDeltaDot(), 0.05);
}

// 测试设置后再写入 NaN 可使该派生量回到未设置状态。
// 因为 NaN 是哨兵值，调用方可能用这种方式"清除"已设置的量。
TEST(PathPointTest, SettingNaNResetsFieldToUnset) {
    TrajectoryPoint point;
    point.setKappa(0.12);
    EXPECT_TRUE(point.hasKappa());

    point.setKappa(std::numeric_limits<double>::quiet_NaN());
    EXPECT_FALSE(point.hasKappa());
    EXPECT_THROW(point.getKappa(), std::logic_error);
}

// 测试使用 x、y、theta 构造 TrajectoryPoint 的场景。
// 因为几何量必须直接来自输入且派生量默认未提供，所以应同时初始化 x/y/theta
// 并保持五个 hasXxx() 为 false。
TEST(PathPointTest, ConstructWithValuesInitializesPoseAndLeavesDerivedUnset) {
    const TrajectoryPoint point{1.2, 3.4, 0.6};

    EXPECT_DOUBLE_EQ(point.x, 1.2);
    EXPECT_DOUBLE_EQ(point.y, 3.4);
    EXPECT_DOUBLE_EQ(point.theta, 0.6);
    EXPECT_FALSE(point.hasKappa());
    EXPECT_FALSE(point.hasV());
    EXPECT_FALSE(point.hasDelta());
    EXPECT_FALSE(point.hasA());
    EXPECT_FALSE(point.hasDeltaDot());
}

// 测试基于 Pose 构造 TrajectoryPoint 的场景。
// 因为 Pose 只携带
// x/y/theta，所以转换后几何量应完整保留且所有派生量保持未设置状态。
TEST(PathPointTest, ConstructFromPoseInitializesPoseAndLeavesDerivedUnset) {
    const Pose pose{2.1, 4.3, 0.7};
    const TrajectoryPoint point(pose);

    EXPECT_DOUBLE_EQ(point.x, 2.1);
    EXPECT_DOUBLE_EQ(point.y, 4.3);
    EXPECT_DOUBLE_EQ(point.theta, 0.7);
    EXPECT_FALSE(point.hasKappa());
    EXPECT_FALSE(point.hasV());
    EXPECT_FALSE(point.hasDelta());
    EXPECT_FALSE(point.hasA());
    EXPECT_FALSE(point.hasDeltaDot());
}

// 测试 TrajectoryPoint 继承 Pose 的 theta 归一化行为。
// 因为构造时调用基类 Pose 构造函数，所以超过一圈的角度应折叠回主值域。
TEST(PathPointTest, ConstructNormalizesThetaLikePose) {
    const TrajectoryPoint near_pi_point{0.0, 0.0, 3.14159};
    const TrajectoryPoint positive_wrap_point{0.0, 0.0, 3.0 * PI};
    const TrajectoryPoint negative_wrap_point{0.0, 0.0, -3.0 * PI};

    EXPECT_NEAR(near_pi_point.theta, 3.14159, 1e-12);
    EXPECT_NEAR(positive_wrap_point.theta, -PI, 1e-12);
    EXPECT_NEAR(negative_wrap_point.theta, PI, 1e-12);
}

// 测试 toString 在未设置派生量时的输出场景。
// 因为未设置的派生量不应出现在 JSON 中，所以输出只应包含 x/y/theta 三个字段。
TEST(PathPointTest, ToStringOmitsUnsetDerivedFields) {
    const TrajectoryPoint point{1.234, 5.678, 0.456};

    EXPECT_EQ(point.toString(), std::string("{\"x\": 1.23, \"y\": 5.68, "
                                            "\"theta\": 0.46}"));
}

// 测试 toString 在设置部分派生量后的输出场景。
// 因为只有被 setXxx() 写入的字段才应被序列化，所以输出只包含 kappa/a 而不含
// v/delta/delta_dot。
TEST(PathPointTest, ToStringIncludesOnlySetDerivedFields) {
    TrajectoryPoint point{1.234, 5.678, 0.456};
    point.setKappa(0.1234);
    point.setA(-0.5678);

    EXPECT_EQ(point.toString(), std::string("{\"x\": 1.23, \"y\": 5.68, "
                                            "\"theta\": 0.46, \"kappa\": 0.12, "
                                            "\"a\": -0.57}"));
}

// 测试 TrajectoryPoint 可以隐式向上转型为 const Pose&。
// 因为 TrajectoryPoint 是 public 继承 Pose，所以多态/切片场景下应保持兼容。
TEST(PathPointTest, ImplicitUcastToConstPoseReference) {
    const TrajectoryPoint point{1.0, 2.0, 0.5};
    const Pose& pose_ref = point;

    EXPECT_DOUBLE_EQ(pose_ref.x, 1.0);
    EXPECT_DOUBLE_EQ(pose_ref.y, 2.0);
    EXPECT_DOUBLE_EQ(pose_ref.theta, 0.5);
}

}  // namespace
}  // namespace apa_post_processor
