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

// 测试适配器对越界查询点会先裁剪到地图内侧再查询，而不是直接透传ESDFMap的
// 越界恢复场。触发原因：无解bug修复后（ESDF碰撞约束由硬约束改为软代价），
// SQP早期迭代可能让某个圆心暂时跑出地图，若适配器向优化器返回零梯度，
// 优化器会完全失去把圆心拉回地图内部的方向信息，导致轨迹发散（实测data7.json长度从
// 18.7m发散到33.6m）。预期行为：越界点应被裁剪到边界内侧后再查询，得到与边界附近某个
// 合法内部点一致的非零距离/梯度。
// L8 契约后底层 ESDFMap 的越界查询返回 L8.1 恢复场（d = d_map(p) − s，
// 梯度恒指向图内）而非全零哨兵；适配器的裁剪语义不变，本用例同步锁定
// 底层恢复场的解析值。
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

    // L8.1 恢复场：p = clamp((10,10)) = (4,4)，s = 6*sqrt(2)；base 为边界
    // 角格 (3,3) 的采样值 -sqrt(2)（最近自由格 (2,2) 在对角）；
    // d = base - s = -7*sqrt(2)，梯度 = (p-q)/s = (-1/sqrt(2), -1/sqrt(2))
    EXPECT_NEAR(raw.first, -7.0 * std::sqrt(2.0), 1e-12);
    EXPECT_NEAR(raw.second.x(), -1.0 / std::sqrt(2.0), 1e-12);
    EXPECT_NEAR(raw.second.y(), -1.0 / std::sqrt(2.0), 1e-12);
    // 适配器把 (10,10) 裁剪到 (3.5,3.5)（内缩 res/2）后查询：落在角格
    // (3,3)（索引钳制），距离为 -sqrt(2)，梯度为角格单侧差分 (1-sqrt(2),
    // 1-sqrt(2))——非零且指向图内，优化器不会失去回拉方向
    EXPECT_NEAR(sample.distance, -std::sqrt(2.0), 1e-12);
    EXPECT_NEAR(sample.gradient.x(), 1.0 - std::sqrt(2.0), 1e-12);
    EXPECT_NEAR(sample.gradient.y(), 1.0 - std::sqrt(2.0), 1e-12);
    EXPECT_FALSE(sample.gradient.isZero());
}

}  // namespace
}  // namespace apa_post_processor
