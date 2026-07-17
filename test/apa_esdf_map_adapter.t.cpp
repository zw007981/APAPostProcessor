#include <gtest/gtest.h>

#include <cmath>

#include "core/NMPC/apa_esdf_map_adapter.h"
#include "spatial/esdf_map.h"
#include "spatial/grid_map.h"
#include "util/position.h"

namespace apa_post_processor {
namespace {

// 测试ApaEsdfMapAdapter把ESDFMap::getDistAndGrad的结果原样转换为EsdfSample。
// 因为StcSQP的圆形ESDF约束只认EsdfMapInterface，适配器的转换必须与底层查询完全一致。
TEST(ApaEsdfMapAdapterTest, QueryDistanceMatchesUnderlyingEsdfMap) {
    const GridMap grid_map(0.5, 4, 3, Position{-1.0, 2.0},
                           {Position{0.0, 2.5}});
    const ESDFMap esdf_map(grid_map);
    const ApaEsdfMapAdapter adapter(esdf_map);

    const Eigen::Vector2d query_point(0.0, 2.5);
    const auto expected =
        esdf_map.getDistAndGrad(query_point.x(), query_point.y());
    const auto sample = adapter.queryDistance(query_point);

    EXPECT_DOUBLE_EQ(sample.distance, expected.first);
    EXPECT_DOUBLE_EQ(sample.gradient.x(), expected.second.x());
    EXPECT_DOUBLE_EQ(sample.gradient.y(), expected.second.y());
}

// 测试适配器在多个不同查询点上均与底层ESDFMap保持一致，覆盖占据区内外两种情形。
// 因为圆形约束会在每个圆心处调用一次queryDistance，必须保证任意坐标都转换正确。
TEST(ApaEsdfMapAdapterTest, QueryDistanceConsistentAcrossMultiplePoints) {
    const std::vector<Position> cells{Position{0.0, 0.0}, Position{1.0, 0.0},
                                      Position{0.0, 1.0}, Position{1.0, 1.0}};
    const GridMap grid_map(1.0, 4, 4, Position{0.0, 0.0}, cells);
    const ESDFMap esdf_map(grid_map);
    const ApaEsdfMapAdapter adapter(esdf_map);

    const std::vector<Eigen::Vector2d> query_points{Eigen::Vector2d(0.0, 0.0),
                                                    Eigen::Vector2d(1.0, 1.0),
                                                    Eigen::Vector2d(2.0, 1.0)};
    for (const auto& point : query_points) {
        const auto expected = esdf_map.getDistAndGrad(point.x(), point.y());
        const auto sample = adapter.queryDistance(point);
        EXPECT_DOUBLE_EQ(sample.distance, expected.first);
        EXPECT_TRUE(sample.gradient.isApprox(expected.second));
    }
}

// 测试适配器对越界查询点会先裁剪到地图内侧再查询，而不是直接返回ESDFMap对越界查询
// 固定给出的(距离=0,
// 梯度=0)。触发原因：无解bug修复后（ESDF碰撞约束由硬约束改为软代价），
// SQP早期迭代可能让某个圆心暂时跑出地图，若适配器像ESDFMap本身一样对越界点返回零梯度，
// 优化器会完全失去把圆心拉回地图内部的方向信息，导致轨迹发散（实测data7.json长度从
// 18.7m发散到33.6m）。预期行为：越界点应被裁剪到边界内侧后再查询，得到与边界附近某个
// 合法内部点一致的非零距离/梯度。
TEST(ApaEsdfMapAdapterTest, QueryDistanceClampsOutOfBoundsPointToMapInterior) {
    const std::vector<Position> cells{Position{0.0, 0.0}, Position{1.0, 0.0},
                                      Position{0.0, 1.0}, Position{1.0, 1.0}};
    const GridMap grid_map(1.0, 4, 4, Position{0.0, 0.0}, cells);
    const ESDFMap esdf_map(grid_map);
    const ApaEsdfMapAdapter adapter(esdf_map);

    // 地图范围为[0,4)x[0,4)，(10.0, 10.0)明显越界
    const Eigen::Vector2d out_of_bounds_point(10.0, 10.0);
    const auto raw = esdf_map.getDistAndGrad(out_of_bounds_point.x(),
                                             out_of_bounds_point.y());
    const auto sample = adapter.queryDistance(out_of_bounds_point);

    // 底层ESDFMap对越界查询固定返回全零结果
    EXPECT_DOUBLE_EQ(raw.first, 0.0);
    EXPECT_TRUE(raw.second.isZero());
    // 适配器裁剪后应得到边界内侧的真实非零距离/梯度，而不是全零
    EXPECT_NE(sample.distance, 0.0);
    EXPECT_FALSE(sample.gradient.isZero());
}

}  // namespace
}  // namespace apa_post_processor
