#include "util/pose.h"

#include <gtest/gtest.h>

#include <string>

namespace apa_post_processor {
namespace {

// 测试 Pose 默认构造的场景。
// 因为默认位姿用于无输入时的安全初值，所以 x、y、theta 都应初始化为 0。
TEST(PoseTest, DefaultConstructInitializesZeroValues) {
    const Pose pose;

    EXPECT_DOUBLE_EQ(pose.x, 0.0);
    EXPECT_DOUBLE_EQ(pose.y, 0.0);
    EXPECT_DOUBLE_EQ(pose.theta, 0.0);
}

// 测试使用 x、y、theta 参数构造 Pose 的场景。
// 因为调用方会直接传入规划位姿，所以构造函数应完整保留三个输入值。
TEST(PoseTest, ConstructWithValuesStoresAllFields) {
    const Pose pose{1.2, 3.4, 0.6};

    EXPECT_DOUBLE_EQ(pose.x, 1.2);
    EXPECT_DOUBLE_EQ(pose.y, 3.4);
    EXPECT_DOUBLE_EQ(pose.theta, 0.6);
}

// 测试 Pose 构造时保留正负坐标和 0 航向角的场景。
// 因为局部路径点可能落在任意象限，所以构造函数只应规范化 theta，不应改写 x/y。
TEST(PoseTest, ConstructWithSignedValuesStoresFieldsExactly) {
    const Pose pose{-1.2, 0.0, 0.0};

    EXPECT_DOUBLE_EQ(pose.x, -1.2);
    EXPECT_DOUBLE_EQ(pose.y, 0.0);
    EXPECT_DOUBLE_EQ(pose.theta, 0.0);
}

// 测试 Pose 构造时航向角归一化的边界场景。
// 因为内部统一使用 remainder 主值域，所以超过一圈的输入应折叠回 [-PI, PI]
// 附近。
TEST(PoseTest, ConstructNormalizesThetaToPrincipalRange) {
    const Pose near_pi_pose{0.0, 0.0, 3.14159};
    const Pose positive_wrap_pose{0.0, 0.0, 3.0 * PI};
    const Pose negative_wrap_pose{0.0, 0.0, -3.0 * PI};

    EXPECT_NEAR(near_pi_pose.theta, 3.14159, 1e-12);
    EXPECT_NEAR(positive_wrap_pose.theta, -PI, 1e-12);
    EXPECT_NEAR(negative_wrap_pose.theta, PI, 1e-12);
}

// 测试从 protobuf Pose 消息构造业务 Pose 的场景。
// 因为外部数据通过 proto 传入，所以 FromProto 应逐字段复制 x、y、theta。
TEST(PoseTest, FromProtoBuildsPoseFields) {
    ::apa::post_processor::Pose proto;
    proto.set_x(2.1);
    proto.set_y(4.3);
    proto.set_theta(0.7);

    const Pose pose = Pose::FromProto(proto);

    EXPECT_DOUBLE_EQ(pose.x, 2.1);
    EXPECT_DOUBLE_EQ(pose.y, 4.3);
    EXPECT_DOUBLE_EQ(pose.theta, 0.7);
}

// 测试 Pose 转换为 JSON 字符串的场景。
// 因为日志输出需要稳定字段名和两位小数精度，所以 toString 应输出 x、y、theta 的
// JSON 文本。
TEST(PoseTest, ToStringBuildsJsonText) {
    const Pose pose{1.234, 5.678, 0.456};

    EXPECT_EQ(pose.toString(), std::string("{\"x\": 1.23, \"y\": 5.68, "
                                           "\"theta\": 0.46}"));
}

// 测试 Pose JSON 输出保留负数和两位小数补零的场景。
// 因为日志解析依赖稳定格式，所以负坐标与 0 航向角也必须按 PRINT_PRECISION
// 输出。
TEST(PoseTest, ToStringKeepsSignedFixedPrecisionFormat) {
    const Pose pose{1.234, -4.5, 0.0};

    EXPECT_EQ(pose.toString(), std::string("{\"x\": 1.23, \"y\": -4.50, "
                                           "\"theta\": 0.00}"));
}

}  // namespace
}  // namespace apa_post_processor