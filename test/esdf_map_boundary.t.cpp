// L8 地图边界语义（M011 Round 4 缺陷修复）的契约测试：
// - L8.2：EDT 前最外圈标记占据，图内场在边界自然衰减到 ~0
// - L8.1：图外按实心障碍处理——d(q) = d_map(p) − ‖q−p‖（p = clamp 到
//   地图矩形），∇d 恒指向图内；恰在边界回落图内场值
// - L8.5：越界查询原子计数（替代逐条告警——单次运行 8 万行日志曾把
//   零梯度平台缺陷掩盖了三个 Milestone）
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "spatial/esdf_map.h"
#include "spatial/grid_map.h"

namespace apa_post_processor {
namespace {

// L8 测试用图：100×100、res=0.1、origin(0,0)（物理域 [0,10)²），唯一
// 障碍在图心 (5,5)——边界场不受其影响（边界采样值恒为 −0.1），便于
// 解析推导期望值
ESDFMap MakeBoundaryTestMap() {
    const GridMap grid_map(0.1, 100, 100, Position{0.0, 0.0},
                           {Position{5.0, 5.0}});
    return ESDFMap(grid_map);
}

// L8.2：边界圈被标记占据——边界采样点 d=−res，向内逐格自然增长
TEST(ESDFMapBoundaryTest, BorderRingDecaysInMapFieldToZeroAtBoundary) {
    const ESDFMap esdf_map = MakeBoundaryTestMap();
    // 西边界采样点（x=0）：被标记占据，d = −res（恰为占据值）
    EXPECT_DOUBLE_EQ(esdf_map.getDist(0.0, 5.0), -0.1);
    // 内移一格：自由且距边界圈一格，d = +res
    EXPECT_DOUBLE_EQ(esdf_map.getDist(0.1, 5.0), 0.1);
    // 内移两格：d = +2·res（场向内自然增长，边界在图内一侧即产生斥力）
    EXPECT_DOUBLE_EQ(esdf_map.getDist(0.2, 5.0), 0.2);
    // 图内侧梯度指向图内（+x 方向）
    const auto [dist, grad] = esdf_map.getDistAndGrad(0.1, 5.0);
    EXPECT_GT(grad.x(), 0.0);
    EXPECT_NEAR(grad.y(), 0.0, 1e-9);
}

// L8.1+L8.2：场在边界上构造性连续（左右极限一致）
TEST(ESDFMapBoundaryTest, FieldIsContinuousAcrossBoundary) {
    const ESDFMap esdf_map = MakeBoundaryTestMap();
    // 西边界 x=0：图内极限 vs 图外极限（穿透 →0⁺）
    EXPECT_NEAR(esdf_map.getDist(1e-9, 5.0), esdf_map.getDist(-1e-9, 5.0),
                1e-6);
    // 东边界远侧闭边（x=10 按 inMap 半开约定为图外，s≈0 回落图内值）
    EXPECT_NEAR(esdf_map.getDist(10.0, 5.0), esdf_map.getDist(9.999999, 5.0),
                1e-6);
}

// L8.1：图外距离 = 边界基值 − 穿透深度（线性），梯度恒指向图内
TEST(ESDFMapBoundaryTest, OutOfMapRecoveryFieldSemantics) {
    const ESDFMap esdf_map = MakeBoundaryTestMap();
    // 西边界外 0.3 m：base=−0.1，d = −0.1−0.3
    const auto [d1, g1] = esdf_map.getDistAndGrad(-0.3, 5.0);
    EXPECT_NEAR(d1, -0.4, 1e-12);
    EXPECT_NEAR(g1.x(), 1.0, 1e-12);  // +x = 指向图内
    EXPECT_NEAR(g1.y(), 0.0, 1e-12);
    // 更深穿透 1.0 m：d = −1.1（线性）
    EXPECT_NEAR(esdf_map.getDist(-1.0, 5.0), -1.1, 1e-12);
    // 角点区域（西北角外）：p=(0,10)，梯度为对角方向（指向图内）。
    // 注意角点栅格的最近自由格在对角方向，base = −√2·res 而非 −res
    const auto [d2, g2] = esdf_map.getDistAndGrad(-0.3, 10.4);
    EXPECT_NEAR(d2, -0.1 * std::sqrt(2.0) - 0.5, 1e-12);  // hypot(0.3,0.4)=0.5
    EXPECT_NEAR(g2.x(), 0.6, 1e-12);
    EXPECT_NEAR(g2.y(), -0.8, 1e-12);
}

// L8.1：穿透深度单调 ⟹ C_safe 单调增、罚单调增（平坦陷阱消灭）
TEST(ESDFMapBoundaryTest, ViolationAndPenaltyMonotoneWithPenetration) {
    const ESDFMap esdf_map = MakeBoundaryTestMap();
    const double r_outer = 0.3;
    const double margin = 0.02;
    const double weight = 100.0;
    double prev_c = -1e9;
    double prev_cost = -1e9;
    for (const double s : {0.05, 0.2, 0.8, 2.0}) {
        const double dist = esdf_map.getDist(-s, 5.0);
        const double c = r_outer + margin - dist;
        const double cost = weight * c * c * c;
        EXPECT_GT(c, prev_c);
        EXPECT_GT(cost, prev_cost);
        prev_c = c;
        prev_cost = cost;
    }
}

// L8.1：图外解析梯度与有限差分对拍（边法向区 + 角点区，避开 45° 棱线）
TEST(ESDFMapBoundaryTest, OutOfMapGradientMatchesFiniteDifference) {
    const ESDFMap esdf_map = MakeBoundaryTestMap();
    const double h = 1e-6;
    const std::vector<Position> probes{Position{-0.45, 5.0},  // 西边法向区
                                       Position{5.0, -0.45},  // 南边法向区
                                       Position{-0.45, 10.35},   // 西北角区
                                       Position{10.35, -0.45}};  // 东南角区
    for (const auto& q : probes) {
        const auto [dist, grad] = esdf_map.getDistAndGrad(q.x, q.y);
        const double fd_x =
            (esdf_map.getDist(q.x + h, q.y) - esdf_map.getDist(q.x - h, q.y)) /
            (2.0 * h);
        const double fd_y =
            (esdf_map.getDist(q.x, q.y + h) - esdf_map.getDist(q.x, q.y - h)) /
            (2.0 * h);
        EXPECT_NEAR(grad.x(), fd_x, 1e-4)
            << "grad_x FD mismatch at (" << q.x << "," << q.y << ")";
        EXPECT_NEAR(grad.y(), fd_y, 1e-4)
            << "grad_y FD mismatch at (" << q.x << "," << q.y << ")";
    }
}

// L8.1：恰在远侧闭边上（s≈0）回落图内场值，梯度连续延拓、非零向量
TEST(ESDFMapBoundaryTest, ExactFarEdgeFallsBackToInMapField) {
    const ESDFMap esdf_map = MakeBoundaryTestMap();
    const auto [dist, grad] = esdf_map.getDistAndGrad(10.0, 5.0);
    EXPECT_DOUBLE_EQ(dist, -0.1);  // 边界采样值（L8.2 标记占据）
    // 连续延拓的图内梯度指向图内（−x），不得为零向量
    EXPECT_LT(grad.x(), 0.0);
    EXPECT_GT(grad.norm(), 0.1);
}

// L8.1：批量查询的越界点与单点恢复场逐位一致
TEST(ESDFMapBoundaryTest, BatchOutOfMapMatchesRecoveryField) {
    const ESDFMap esdf_map = MakeBoundaryTestMap();
    constexpr int kN = 3;
    const double xs[kN] = {-0.3, 5.0, 10.4};
    const double ys[kN] = {5.0, 5.0, -0.2};
    double dists[kN], gx[kN], gy[kN];
    esdf_map.getDistAndGradBatch(xs, ys, kN, dists, gx, gy);
    for (int i = 0; i < kN; ++i) {
        const auto [dist, grad] = esdf_map.getDistAndGrad(xs[i], ys[i]);
        EXPECT_DOUBLE_EQ(dists[i], dist) << "i=" << i;
        EXPECT_DOUBLE_EQ(gx[i], grad.x()) << "i=" << i;
        EXPECT_DOUBLE_EQ(gy[i], grad.y()) << "i=" << i;
    }
}

// L8.5：越界查询按次原子计数（替代逐条告警），可复位；图内查询不计
TEST(ESDFMapBoundaryTest, OutOfMapQueriesCountedAndResettable) {
    const ESDFMap esdf_map = MakeBoundaryTestMap();
    esdf_map.resetOutOfMapQueryCount();
    EXPECT_EQ(esdf_map.outOfMapQueryCount(), 0U);
    esdf_map.getDist(5.0, 4.0);         // 图内
    esdf_map.getDistAndGrad(4.9, 5.0);  // 图内
    EXPECT_EQ(esdf_map.outOfMapQueryCount(), 0U);
    esdf_map.getDist(-1.0, 5.0);         // 图外 +1
    esdf_map.getDistAndGrad(5.0, -1.0);  // 图外 +1
    const double xs[2] = {-1.0, 5.0};
    const double ys[2] = {5.0, 5.0};
    double d[2], gx[2], gy[2];
    esdf_map.getDistAndGradBatch(xs, ys, 2, d, gx, gy);  // 1 图外 +1
    EXPECT_EQ(esdf_map.outOfMapQueryCount(), 3U);
    esdf_map.resetOutOfMapQueryCount();
    EXPECT_EQ(esdf_map.outOfMapQueryCount(), 0U);
}

}  // namespace
}  // namespace apa_post_processor
