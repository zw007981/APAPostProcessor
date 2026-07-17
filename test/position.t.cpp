#include <gtest/gtest.h>

#include <string>

#include "util/position.h"

namespace apa_post_processor {
namespace {

// 测试 Position 默认构造的场景。
// 因为默认位置用于无输入时的安全初值，所以 x、y 都应初始化为 0。
TEST(PositionTest, DefaultConstructInitializesZeroValues) {
    const Position position;

    EXPECT_DOUBLE_EQ(position.x, 0.0);
    EXPECT_DOUBLE_EQ(position.y, 0.0);
}

// 测试使用 x、y 参数构造 Position 的场景。
// 因为调用方会直接传入二维坐标，所以构造函数应完整保留两个输入值。
TEST(PositionTest, ConstructWithValuesStoresAllFields) {
    const Position position{1.2, 3.4};

    EXPECT_DOUBLE_EQ(position.x, 1.2);
    EXPECT_DOUBLE_EQ(position.y, 3.4);
}

// 测试 Position 构造时保留正数、负数和 0 的场景。
// 因为地图坐标允许落在局部坐标系任意象限，所以构造函数不应裁剪或改写符号。
TEST(PositionTest, ConstructWithSignedValuesStoresFieldsExactly) {
    const Position position{-1.2, 0.0};

    EXPECT_DOUBLE_EQ(position.x, -1.2);
    EXPECT_DOUBLE_EQ(position.y, 0.0);
}

// 测试从 protobuf Position 消息构造业务 Position 的场景。
// 因为外部地图和轨迹数据通过 proto 传入，所以 FromProto 应逐字段复制 x、y。
TEST(PositionTest, FromProtoBuildsPositionFields) {
    ::apa::post_processor::Position proto;
    proto.set_x(2.1);
    proto.set_y(4.3);

    const Position position = Position::FromProto(proto);

    EXPECT_DOUBLE_EQ(position.x, 2.1);
    EXPECT_DOUBLE_EQ(position.y, 4.3);
}

// 测试 Position 转换为 JSON 字符串的场景。
// 因为日志输出需要稳定字段名和两位小数精度，所以 toString 应输出 x、y 的 JSON
// 文本。
TEST(PositionTest, ToStringBuildsJsonText) {
    const Position position{1.234, 5.678};

    EXPECT_EQ(position.toString(), std::string("{\"x\": 1.23, \"y\": 5.68}"));
}

// 测试 Position JSON 输出保留负数和两位小数补零的场景。
// 因为日志解析依赖稳定格式，所以负坐标和整数坐标也必须按 PRINT_PRECISION 输出。
TEST(PositionTest, ToStringKeepsSignedFixedPrecisionFormat) {
    const Position position{1.234, -4.5};

    EXPECT_EQ(position.toString(), std::string("{\"x\": 1.23, \"y\": -4.50}"));
}

}  // namespace
}  // namespace apa_post_processor