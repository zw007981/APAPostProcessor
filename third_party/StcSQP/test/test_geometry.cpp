#include <vector>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "util/geometry.h"

using namespace stc_SQP;

TEST(GJK, DistanceBetweenSeparatedSquares)
{
    // 测试目的：验证 gjkConvexDistance2d 计算两个分离正方形的正确距离
    // 流程：构造边长为 2 的中心在 (0,0) 和 (5,0) 的两个正方形，调用 gjkConvexDistance2d
    // 预期效果：距离为 3（两正方形最近边间距）
    std::vector<Eigen::Vector2d> square_a = {
        Eigen::Vector2d(-1.0, -1.0),
        Eigen::Vector2d(1.0, -1.0),
        Eigen::Vector2d(1.0, 1.0),
        Eigen::Vector2d(-1.0, 1.0),
    };
    std::vector<Eigen::Vector2d> square_b = {
        Eigen::Vector2d(4.0, -1.0),
        Eigen::Vector2d(6.0, -1.0),
        Eigen::Vector2d(6.0, 1.0),
        Eigen::Vector2d(4.0, 1.0),
    };
    EXPECT_NEAR(gjkConvexDistance2d(square_a, square_b), 3.0, 1e-12);
}

TEST(GJK, DistanceZeroWhenSquaresOverlap)
{
    // 测试目的：验证两个相交凸多边形的 GJK 距离为 0
    // 流程：构造两个相互重叠的正方形，调用 gjkConvexDistance2d
    // 预期效果：距离为 0
    std::vector<Eigen::Vector2d> square_a = {
        Eigen::Vector2d(-1.0, -1.0),
        Eigen::Vector2d(1.0, -1.0),
        Eigen::Vector2d(1.0, 1.0),
        Eigen::Vector2d(-1.0, 1.0),
    };
    std::vector<Eigen::Vector2d> square_b = {
        Eigen::Vector2d(0.0, 0.0),
        Eigen::Vector2d(2.0, 0.0),
        Eigen::Vector2d(2.0, 2.0),
        Eigen::Vector2d(0.0, 2.0),
    };
    EXPECT_DOUBLE_EQ(gjkConvexDistance2d(square_a, square_b), 0.0);
}

TEST(GJK, DistanceFromRectangleToSegment)
{
    // 测试目的：验证矩形到线段的 GJK 距离（用于 SimpleParkingMap 墙的距离计算）
    // 流程：构造中心在 (0,0)、宽 2 高 2 的矩形，以及位于 x=4 的垂直线段
    // 预期效果：距离为 3（矩形右边界 x=1 到线段 x=4 的距离）
    std::vector<Eigen::Vector2d> rect = {
        Eigen::Vector2d(-1.0, -1.0),
        Eigen::Vector2d(1.0, -1.0),
        Eigen::Vector2d(1.0, 1.0),
        Eigen::Vector2d(-1.0, 1.0),
    };
    std::vector<Eigen::Vector2d> segment = {
        Eigen::Vector2d(4.0, -10.0),
        Eigen::Vector2d(4.0, 10.0),
    };
    EXPECT_NEAR(gjkConvexDistance2d(rect, segment), 3.0, 1e-12);
}

TEST(GJK, SegmentOverloadMatchesVectorPath)
{
    // 测试目的：验证线段端点重载与构造 2 点 vector 的旧路径数值一致
    // 流程：用同一组矩形和线段分别调用两种重载
    // 预期效果：返回距离相等
    std::vector<Eigen::Vector2d> rect = {
        Eigen::Vector2d(-1.0, -1.0),
        Eigen::Vector2d(1.0, -1.0),
        Eigen::Vector2d(1.0, 1.0),
        Eigen::Vector2d(-1.0, 1.0),
    };
    const Eigen::Vector2d seg_start(4.0, -10.0);
    const Eigen::Vector2d seg_end(4.0, 10.0);
    const double via_vector = gjkConvexDistance2d(
        rect, std::vector<Eigen::Vector2d>{ seg_start, seg_end });
    const double via_segment = gjkConvexDistance2d(rect, seg_start, seg_end);
    EXPECT_DOUBLE_EQ(via_vector, via_segment);
}

TEST(GJK, RejectsEmptyPolygon)
{
    // 测试目的：验证 gjkConvexDistance2d 对空多边形输入抛出异常
    // 流程：构造空 polygon_a 或 polygon_b 调用函数
    // 预期效果：抛出 std::invalid_argument
    std::vector<Eigen::Vector2d> empty;
    std::vector<Eigen::Vector2d> square = {
        Eigen::Vector2d(-1.0, -1.0),
        Eigen::Vector2d(1.0, -1.0),
        Eigen::Vector2d(1.0, 1.0),
        Eigen::Vector2d(-1.0, 1.0),
    };
    EXPECT_THROW(gjkConvexDistance2d(empty, square), std::invalid_argument);
    EXPECT_THROW(gjkConvexDistance2d(square, empty), std::invalid_argument);
    EXPECT_THROW(gjkConvexDistance2d(empty, Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(1.0, 0.0)),
        std::invalid_argument);
}
