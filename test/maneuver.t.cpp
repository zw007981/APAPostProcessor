#include <gtest/gtest.h>

#include <cstddef>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "util/maneuver.h"

namespace apa_post_processor {
namespace {

// 测试单个 PathPoint 构造 Maneuver 的场景。
// 因为 Maneuver 不再根据路径点推断方向，所以默认方向应保持 UNKNOWN。
TEST(ManeuverTest, ConstructWithSinglePathPointKeepsDefaultDirection)
{
    const PathPoint pose{0.0, 0.0, 0.0};
    Maneuver maneuver(pose);

    ASSERT_EQ(maneuver.points.size(), 1U);
    EXPECT_DOUBLE_EQ(maneuver.points.at(0).x, 0.0);
    EXPECT_EQ(maneuver.direction, Direction::UNKNOWN);
}

// 测试显式方向构造 Maneuver 的场景。
// 因为方向已经由调用方决定，所以 Maneuver 只保留传入方向，不再重新计算。
TEST(ManeuverTest, ConstructWithExplicitDirectionStoresDirection)
{
    std::vector<PathPoint> points{
        {0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0},
    };

    Maneuver maneuver(std::move(points), Direction::BACKWARD);

    ASSERT_EQ(maneuver.points.size(), 2U);
    EXPECT_DOUBLE_EQ(maneuver.points.at(1).x, 1.0);
    EXPECT_EQ(maneuver.direction, Direction::BACKWARD);
}

// 测试相邻重叠路径点构造 Maneuver 的场景。
// 因为当前结构只承载外部路径点序列，所以不应再过滤或改写输入点。
TEST(ManeuverTest, ConstructWithVectorPreservesAdjacentOverlap)
{
    std::vector<PathPoint> points{
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0},
    };

    Maneuver maneuver(std::move(points));

    ASSERT_EQ(maneuver.points.size(), 3U);
    EXPECT_DOUBLE_EQ(maneuver.points.at(0).x, 0.0);
    EXPECT_DOUBLE_EQ(maneuver.points.at(1).x, 0.0);
    EXPECT_DOUBLE_EQ(maneuver.points.at(2).x, 1.0);
}

// 测试空路径点序列构造 Maneuver 的非法输入场景。
// 因为空机动段没有业务含义，所以构造函数应抛出 invalid_argument 阻止无效对象生成。
TEST(ManeuverTest, ConstructWithEmptyPointsThrows)
{
    std::vector<PathPoint> points;
    EXPECT_THROW((void)Maneuver(std::move(points)), std::invalid_argument);
}

// 测试方向变化路径点构造 Maneuver 的场景。
// 因为 Maneuver 不再验证方向一致性，所以折返路径应被原样保存。
TEST(ManeuverTest, ConstructWithTurnBackPointsDoesNotInferDirection)
{
    std::vector<PathPoint> points{
        {0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0},
        {0.0, 0.0, 0.0},
    };

    Maneuver maneuver(std::move(points));

    ASSERT_EQ(maneuver.points.size(), 3U);
    EXPECT_DOUBLE_EQ(maneuver.points.at(2).x, 0.0);
    EXPECT_EQ(maneuver.direction, Direction::UNKNOWN);
}

// 测试空 Maneuver 与单点 Maneuver 的长度查询场景。
// 因为少于两个路径点时不存在可累加线段，所以 length 应返回 0。
TEST(ManeuverTest, LengthReturnsZeroWhenPointCountLessThanTwo)
{
    const Maneuver empty_maneuver;
    const Maneuver single_point_maneuver(PathPoint{1.0, 2.0, 0.3});

    EXPECT_DOUBLE_EQ(empty_maneuver.length(), 0.0);
    EXPECT_DOUBLE_EQ(single_point_maneuver.length(), 0.0);
}

// 测试 Maneuver 累加相邻路径点欧氏距离的场景。
// 因为机动段长度代表二维轨迹折线长度，所以每一段相邻点距离都应参与求和。
TEST(ManeuverTest, LengthSumsAdjacentPlanarDistances)
{
    std::vector<PathPoint> points{
        {0.0, 0.0, 0.0},
        {3.0, 4.0, 0.0},
        {3.0, 4.0, 0.0},
        {6.0, 8.0, 0.0},
    };

    const Maneuver maneuver(std::move(points));

    EXPECT_DOUBLE_EQ(maneuver.length(), 10.0);
}

// 测试 Maneuver 两点长度直接等于欧氏距离的场景。
// 因为这是 transform_reduce 的最小有效输入，所以应精确返回单段距离。
TEST(ManeuverTest, LengthReturnsDistanceBetweenTwoPoints)
{
    const Maneuver maneuver(std::vector<PathPoint>{{0.0, 0.0, 0.0},
                                             {3.0, 4.0, 0.0}});

    EXPECT_DOUBLE_EQ(maneuver.length(), 5.0);
}

// 测试 Maneuver 多点直角折线长度累加的场景。
// 因为路径长度按相邻点分段累计，所以直角折线应返回 3 + 4。
TEST(ManeuverTest, LengthSumsRightAnglePolylineSegments)
{
    const Maneuver maneuver(std::vector<PathPoint>{{0.0, 0.0, 0.0},
                                             {3.0, 0.0, 0.0},
                                             {3.0, 4.0, 0.0}});

    EXPECT_DOUBLE_EQ(maneuver.length(), 7.0);
}

// 测试 Maneuver 长度不受航向角变化影响的场景。
// 因为 length 只描述平面路径长度，所以原地旋转或 theta 差异不应增加距离。
TEST(ManeuverTest, LengthIgnoresThetaChange)
{
    std::vector<PathPoint> points{
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 1.0},
        {0.0, 2.0, 2.0},
    };

    const Maneuver maneuver(std::move(points));

    EXPECT_DOUBLE_EQ(maneuver.length(), 2.0);
}

// 测试非 const Maneuver 通过 begin/end 遍历并修改路径点的场景。
// 因为业务侧可能直接迭代调整轨迹状态，所以非 const 迭代器应能访问全部路径点并写回修改结果。
TEST(ManeuverTest, MutableIteratorsTraverseAndModifyPoints)
{
    std::vector<PathPoint> points{
        {0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0},
        {2.0, 0.0, 0.0},
    };

    Maneuver maneuver(std::move(points));
    std::size_t point_count = 0U;
    for (auto point_it = maneuver.begin(); point_it != maneuver.end();
         ++point_it) {
        point_it->y += 1.0;
        ++point_count;
    }

    EXPECT_EQ(point_count, 3U);
    EXPECT_DOUBLE_EQ(maneuver.points.at(0).y, 1.0);
    EXPECT_DOUBLE_EQ(maneuver.points.at(1).y, 1.0);
    EXPECT_DOUBLE_EQ(maneuver.points.at(2).y, 1.0);
}

// 测试 const Maneuver 通过 begin/end 只读遍历路径点的场景。
// 因为只读上下文仍需要按原始顺序访问轨迹，所以 const 迭代器应能完整遍历全部路径点。
TEST(ManeuverTest, ConstIteratorsTraversePointsInOrder)
{
    std::vector<PathPoint> points{
        {0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0},
        {2.0, 0.0, 0.0},
    };

    const Maneuver maneuver(std::move(points));
    std::vector<double> x_values;
    x_values.reserve(3U);
    for (auto point_it = maneuver.begin(); point_it != maneuver.end();
         ++point_it) {
        x_values.emplace_back(point_it->x);
    }

    ASSERT_EQ(x_values.size(), 3U);
    EXPECT_DOUBLE_EQ(x_values.at(0), 0.0);
    EXPECT_DOUBLE_EQ(x_values.at(1), 1.0);
    EXPECT_DOUBLE_EQ(x_values.at(2), 2.0);
}

// 测试 Maneuver 转换为 JSON 字符串的场景。
// 因为机动段现在只展示显式方向和路径点列表，所以应输出 direction 字符串和 points 数组。
TEST(ManeuverTest, ToStringBuildsJsonText)
{
    std::vector<PathPoint> points{
        {0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0},
    };

    const Maneuver maneuver(std::move(points), Direction::FORWARD);
    EXPECT_EQ(maneuver.toString(),
              std::string("{\"direction\": \"FORWARD\", "
                          "\"points\": [{\"x\": 0.00, \"y\": 0.00, "
                          "\"theta\": 0.00}, {\"x\": 1.00, "
                          "\"y\": 0.00, \"theta\": 0.00}]}"));
}

// 测试 Maneuver JSON 输出覆盖所有合法方向枚举的场景。
// 因为日志和调试视图依赖方向名称，所以每个枚举值都应映射到稳定字符串。
TEST(ManeuverTest, ToStringMapsAllKnownDirections)
{
    const PathPoint point{0.0, 0.0, 0.0};

    EXPECT_NE(Maneuver(point, Direction::UNKNOWN).toString().find("\"UNKNOWN\""),
              std::string::npos);
    EXPECT_NE(Maneuver(point, Direction::FORWARD).toString().find("\"FORWARD\""),
              std::string::npos);
    EXPECT_NE(Maneuver(point, Direction::BACKWARD).toString().find("\"BACKWARD\""),
              std::string::npos);
    EXPECT_NE(Maneuver(point, Direction::PIVOT).toString().find("\"PIVOT\""),
              std::string::npos);
}

// 测试 Maneuver JSON 输出遇到非法方向枚举的兜底场景。
// 因为外部数据或调试构造可能产生越界枚举，所以 toString 应降级输出 UNKNOWN。
TEST(ManeuverTest, ToStringFallsBackToUnknownForInvalidDirection)
{
    const Maneuver maneuver(PathPoint{0.0, 0.0, 0.0},
                            static_cast<Direction>(99));

    EXPECT_NE(maneuver.toString().find("\"UNKNOWN\""), std::string::npos);
}

}  // namespace
}  // namespace apa_post_processor
