#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "util/maneuver.h"
#include "util/topology_cleaner.h"
#include "util/trajectory_point.h"

namespace apa_post_processor {
namespace {

// 辅助函数：构造一个包含 N 个点的 FORWARD 机动段，点沿 x 轴均匀分布。
// delta_val 控制每个点的前轮转角（首尾 Δδ 约等于 delta_val 的差异）。
Maneuver MakeForwardManeuver(int num_points, double segment_length,
                             double delta_start, double delta_end) {
    std::vector<TrajectoryPoint> points;
    points.reserve(static_cast<std::size_t>(num_points));
    for (int i = 0; i < num_points; ++i) {
        const double t =
            static_cast<double>(i) / static_cast<double>(num_points - 1);
        TrajectoryPoint pt(t * segment_length, 0.0, 0.0);
        pt.setV(1.0);
        pt.setDelta(delta_start + t * (delta_end - delta_start));
        pt.setA(0.0);
        pt.setDeltaDot(0.0);
        points.push_back(std::move(pt));
    }
    return Maneuver{std::move(points), Direction::FORWARD};
}

// 辅助函数：构造一个长度极短的 FORWARD 机动段（仅 2 个点，距离极小）。
Maneuver MakeTinyManeuver(double arc_length, double delta_start,
                          double delta_end) {
    std::vector<TrajectoryPoint> points;
    points.reserve(2);
    TrajectoryPoint p0(0.0, 0.0, 0.0);
    p0.setV(0.01);
    p0.setDelta(delta_start);
    p0.setA(0.0);
    p0.setDeltaDot(0.0);
    points.push_back(std::move(p0));
    TrajectoryPoint p1(arc_length, 0.0, 0.0);
    p1.setV(0.01);
    p1.setDelta(delta_end);
    p1.setA(0.0);
    p1.setDeltaDot(0.0);
    points.push_back(std::move(p1));
    return Maneuver{std::move(points), Direction::FORWARD};
}

// ===== ClassifyAndResetManeuvers 测试 =====

// 测试目标：验证正常长机动段不被标记。
// 测试流程：构造弧长约3m、Δδ=0.05rad的FORWARD段，调用分类函数。
// 预期结果：direction保持FORWARD不变。
TEST(TopologyCleanerTest, NormalSegmentUntouched) {
    std::vector<Maneuver> maneuvers;
    maneuvers.push_back(MakeForwardManeuver(10, 3.0, 0.0, 0.05));
    const Direction original_dir = maneuvers[0].direction;
    ClassifyAndResetManeuvers(maneuvers, TopologyCleanupConfig{});
    EXPECT_EQ(maneuvers[0].direction, original_dir);
}

// 测试目标：验证极小段+小Δδ被标记为UNKNOWN（压平废段）。
// 测试流程：构造弧长0.02m、Δδ=0.01rad的极小段，调用分类函数。
// 预期结果：direction被改为UNKNOWN。
TEST(TopologyCleanerTest, FlatTinySegmentMarkedUnknown) {
    std::vector<Maneuver> maneuvers;
    maneuvers.push_back(MakeTinyManeuver(0.02, 0.0, 0.01));
    ClassifyAndResetManeuvers(maneuvers, TopologyCleanupConfig{});
    EXPECT_EQ(maneuvers[0].direction, Direction::UNKNOWN);
}

// 测试目标：验证极小段+大Δδ被转化为PIVOT。
// 测试流程：构造弧长0.02m、Δδ=0.5rad的极小段，调用分类函数。
// 预期结果：direction改为PIVOT，所有点(x,y)锁定为首点坐标，v/a强制置零。
TEST(TopologyCleanerTest, PivotConversionOnLargeDeltaDelta) {
    std::vector<Maneuver> maneuvers;
    maneuvers.push_back(MakeTinyManeuver(0.02, 0.0, 0.5));
    ClassifyAndResetManeuvers(maneuvers, TopologyCleanupConfig{});
    EXPECT_EQ(maneuvers[0].direction, Direction::PIVOT);
    // 验证所有点的(x,y)被锁定为首点坐标
    const double x0 = maneuvers[0].points.front().x;
    const double y0 = maneuvers[0].points.front().y;
    for (const auto& pt : maneuvers[0].points) {
        EXPECT_DOUBLE_EQ(pt.x, x0);
        EXPECT_DOUBLE_EQ(pt.y, y0);
        EXPECT_DOUBLE_EQ(pt.getV(), 0.0);
        EXPECT_DOUBLE_EQ(pt.getA(), 0.0);
    }
}

// 测试目标：验证PIVOT转化后θ保留NMPC原值（不覆写为首点θ）。
// 测试流程：构造两个点的极小段，首尾θ不同(0.0, 0.3)，Δδ=0.5触发PIVOT。
// 预期结果：尾点θ保持0.3不变，不被覆写为0.0。
TEST(TopologyCleanerTest, PivotPreservesTheta) {
    std::vector<TrajectoryPoint> points;
    points.reserve(2);
    TrajectoryPoint p0(0.0, 0.0, 0.0);
    p0.setV(0.01);
    p0.setDelta(0.0);
    p0.setA(0.0);
    points.push_back(std::move(p0));
    TrajectoryPoint p1(0.02, 0.0, 0.3);
    p1.setV(0.01);
    p1.setDelta(0.5);
    p1.setA(0.0);
    points.push_back(std::move(p1));
    std::vector<Maneuver> maneuvers;
    maneuvers.emplace_back(std::move(points), Direction::FORWARD);
    ClassifyAndResetManeuvers(maneuvers, TopologyCleanupConfig{});
    EXPECT_EQ(maneuvers[0].direction, Direction::PIVOT);
    EXPECT_DOUBLE_EQ(maneuvers[0].points[0].theta, 0.0);
    EXPECT_DOUBLE_EQ(maneuvers[0].points[1].theta, 0.3);
}

// 测试目标：验证Δδ恰好等于阈值时被归为压平（≤判定）。
// 测试流程：构造弧长0.02m、Δδ=0.1rad的极小段。
// 预期结果：direction改为UNKNOWN而非PIVOT。
TEST(TopologyCleanerTest, DeltaDeltaAtThresholdIsFlat) {
    std::vector<Maneuver> maneuvers;
    maneuvers.push_back(MakeTinyManeuver(0.02, 0.0, 0.1));
    ClassifyAndResetManeuvers(maneuvers, TopologyCleanupConfig{});
    EXPECT_EQ(maneuvers[0].direction, Direction::UNKNOWN);
}

// 测试目标：验证弧长恰好等于阈值时不触发极小段逻辑。
// 测试流程：构造弧长0.05m、Δδ=0.5rad的段。
// 预期结果：direction保持FORWARD不变（≥阈值不处理）。
TEST(TopologyCleanerTest, ArcLengthAtThresholdNotTiny) {
    std::vector<Maneuver> maneuvers;
    maneuvers.push_back(MakeTinyManeuver(0.05, 0.0, 0.5));
    ClassifyAndResetManeuvers(maneuvers, TopologyCleanupConfig{});
    EXPECT_EQ(maneuvers[0].direction, Direction::FORWARD);
}

// 测试目标：验证空maneuvers向量不崩溃。
// 测试流程：传入空vector调用分类函数。
// 预期结果：正常返回，无异常。
TEST(TopologyCleanerTest, ClassifyEmptyManeuversNoCrash) {
    std::vector<Maneuver> maneuvers;
    EXPECT_NO_THROW(
        ClassifyAndResetManeuvers(maneuvers, TopologyCleanupConfig{}));
}

// ===== ReconstructPath 测试 =====

// 测试目标：验证单段正常路径原样保留。
// 测试流程：构造一个正常FORWARD段，调用ReconstructPath。
// 预期结果：返回的Path包含1个FORWARD段，点数不变。
TEST(TopologyCleanerTest, ReconstructSingleSegmentUnchanged) {
    std::vector<Maneuver> maneuvers;
    maneuvers.push_back(MakeForwardManeuver(5, 2.0, 0.0, 0.02));
    const std::size_t orig_pts = maneuvers[0].points.size();
    Path result = ReconstructPath(maneuvers);
    ASSERT_EQ(result.numManeuvers(), 1u);
    EXPECT_EQ(result.getManeuvers()[0].direction, Direction::FORWARD);
    EXPECT_EQ(result.getManeuvers()[0].points.size(), orig_pts);
}

// 测试目标：验证UNKNOWN段被剔除，相邻同向段自动合并。
// 测试流程：构造[FORWARD正常, UNKNOWN标记段,
// FORWARD正常]，调用ReconstructPath。
// 预期结果：UNKNOWN被剔除，两个FORWARD同向合并为1个段。
TEST(TopologyCleanerTest, ReconstructRemovesUnknownSegments) {
    std::vector<Maneuver> maneuvers;
    maneuvers.push_back(MakeForwardManeuver(5, 2.0, 0.0, 0.02));
    // 手动标记一个UNKNOWN段
    auto unknown_seg = MakeTinyManeuver(0.02, 0.0, 0.01);
    unknown_seg.direction = Direction::UNKNOWN;
    maneuvers.push_back(std::move(unknown_seg));
    maneuvers.push_back(MakeForwardManeuver(5, 1.5, 0.02, 0.04));
    Path result = ReconstructPath(maneuvers);
    // UNKNOWN被剔除后，两个FORWARD相邻同向 → 合并为1段
    EXPECT_EQ(result.numManeuvers(), 1u);
    EXPECT_EQ(result.getManeuvers()[0].direction, Direction::FORWARD);
}

// 测试目标：验证剔除UNKNOWN后相邻同向段合并。
// 测试流程：构造[FORWARD(A), UNKNOWN(flat), FORWARD(B)]，A和B同向。
// 预期结果：返回1个FORWARD段，点数为 A.size+B.size-1（pop_back去重）。
TEST(TopologyCleanerTest, ReconstructMergesSameDirectionAfterPruning) {
    std::vector<Maneuver> maneuvers;
    auto seg_a = MakeForwardManeuver(5, 2.0, 0.0, 0.02);
    const std::size_t n_a = seg_a.points.size();
    maneuvers.push_back(std::move(seg_a));
    auto unknown_seg = MakeTinyManeuver(0.02, 0.02, 0.03);
    unknown_seg.direction = Direction::UNKNOWN;
    maneuvers.push_back(std::move(unknown_seg));
    auto seg_b = MakeForwardManeuver(5, 1.5, 0.02, 0.04);
    const std::size_t n_b = seg_b.points.size();
    maneuvers.push_back(std::move(seg_b));
    Path result = ReconstructPath(maneuvers);
    ASSERT_EQ(result.numManeuvers(), 1u);
    EXPECT_EQ(result.getManeuvers()[0].direction, Direction::FORWARD);
    // n_a + n_b - 1：前段尾点和后段首点重合，pop_back去重
    EXPECT_EQ(result.getManeuvers()[0].points.size(), n_a + n_b - 1);
}

// 测试目标：验证方向不同的相邻段不合并。
// 测试流程：构造[FORWARD, UNKNOWN, BACKWARD]，剔除UNKNOWN后相邻方向不同。
// 预期结果：返回2个段，FORWARD和BACKWARD各一。
TEST(TopologyCleanerTest, ReconstructNoMergeOppositeDirections) {
    std::vector<Maneuver> maneuvers;
    maneuvers.push_back(MakeForwardManeuver(5, 2.0, 0.0, 0.02));
    auto unknown_seg = MakeTinyManeuver(0.02, 0.0, 0.01);
    unknown_seg.direction = Direction::UNKNOWN;
    maneuvers.push_back(std::move(unknown_seg));
    auto bwd_seg = MakeForwardManeuver(5, 1.5, 0.02, 0.04);
    bwd_seg.direction = Direction::BACKWARD;
    maneuvers.push_back(std::move(bwd_seg));
    Path result = ReconstructPath(maneuvers);
    ASSERT_EQ(result.numManeuvers(), 2u);
    EXPECT_EQ(result.getManeuvers()[0].direction, Direction::FORWARD);
    EXPECT_EQ(result.getManeuvers()[1].direction, Direction::BACKWARD);
}

// 测试目标：验证PIVOT段不被剔除且不与相邻段合并。
// 测试流程：构造[FORWARD, PIVOT, FORWARD]。
// 预期结果：返回3个段，PIVOT夹在中间不参与同向合并。
TEST(TopologyCleanerTest, ReconstructPreservesPivotSegments) {
    std::vector<Maneuver> maneuvers;
    maneuvers.push_back(MakeForwardManeuver(5, 2.0, 0.0, 0.02));
    auto pivot_seg = MakeTinyManeuver(0.02, 0.0, 0.5);
    pivot_seg.direction = Direction::PIVOT;
    maneuvers.push_back(std::move(pivot_seg));
    maneuvers.push_back(MakeForwardManeuver(5, 1.5, 0.02, 0.04));
    Path result = ReconstructPath(maneuvers);
    EXPECT_EQ(result.numManeuvers(), 3u);
}

// 测试目标：验证所有段均为UNKNOWN时返回单段兜底。
// 测试流程：传入两个UNKNOWN标记段。
// 预期结果：返回1个段（首个UNKNOWN段被保留作为兜底），不返回空Path。
TEST(TopologyCleanerTest, ReconstructAllUnknownReturnsGuard) {
    std::vector<Maneuver> maneuvers;
    auto seg1 = MakeForwardManeuver(3, 0.5, 0.0, 0.01);
    seg1.direction = Direction::UNKNOWN;
    maneuvers.push_back(std::move(seg1));
    auto seg2 = MakeForwardManeuver(3, 0.3, 0.0, 0.01);
    seg2.direction = Direction::UNKNOWN;
    maneuvers.push_back(std::move(seg2));
    Path result = ReconstructPath(maneuvers);
    EXPECT_EQ(result.numManeuvers(), 1u);
}

// 测试目标：验证完整分类+重构链路：ClassifyAndResetManeuvers →
// ReconstructPath。 测试流程：构造[NORMAL, FLAT(极小+小Δδ), NORMAL,
// PIVOT(极小+大Δδ), NORMAL]
//          的混合序列，先分类再重构。
// 预期结果：FLAT段被剔除，PIVOT保留，同向段合并后共3段。
TEST(TopologyCleanerTest, FullPipelineClassificationAndReconstruction) {
    std::vector<Maneuver> maneuvers;
    // 段0：正常前进 2m
    maneuvers.push_back(MakeForwardManeuver(5, 2.0, 0.0, 0.02));
    // 段1：极小压平(应标记UNKNOWN然后剔除)
    maneuvers.push_back(MakeTinyManeuver(0.02, 0.02, 0.03));
    // 段2：正常前进 1.5m（段0和段2应在剔除段1后合并为一个FORWARD）
    maneuvers.push_back(MakeForwardManeuver(5, 1.5, 0.03, 0.05));
    // 段3：极小原地打轮(应转为PIVOT)
    maneuvers.push_back(MakeTinyManeuver(0.02, 0.05, 0.5));
    // 段4：正常前进 1m
    maneuvers.push_back(MakeForwardManeuver(5, 1.0, 0.05, 0.06));
    const std::size_t orig_count = maneuvers.size();
    ClassifyAndResetManeuvers(maneuvers, TopologyCleanupConfig{});
    // 段1 应为UNKNOWN
    EXPECT_EQ(maneuvers[1].direction, Direction::UNKNOWN);
    // 段3 应为PIVOT
    EXPECT_EQ(maneuvers[3].direction, Direction::PIVOT);
    Path result = ReconstructPath(maneuvers);
    // 预期：段0+段2→合并1段, 段3→PIVOT1段, 段4→1段 = 共3段
    EXPECT_EQ(result.numManeuvers(), 3u)
        << "Expected 3 maneuvers after cleanup, got " << result.numManeuvers();
    EXPECT_FALSE(result.empty());
}

// ===== 真实 NMPC 输出模拟测试 =====

// 辅助函数：构建一个几何连续的 Path，相邻段首尾点严格重合。
// 每段由 (arc_length, direction, delta_start, delta_end) 描述，
// 点沿 x 轴铺设（模拟简化的一维运动），点数与弧长成正比。
Path BuildContinuousPath(
    const std::vector<std::tuple<double, Direction, double, double>>&
        segments) {
    Path path;
    auto& maneuvers = path.getManeuvers();
    double cursor_x = 0.0;
    for (const auto& [arc, dir, d0, d1] : segments) {
        // 每 0.05m 弧长至少 1 个点，最少 3 个点
        const int n = std::max(3, static_cast<int>(arc / 0.05) + 1);
        std::vector<TrajectoryPoint> points;
        points.reserve(static_cast<std::size_t>(n));
        for (int i = 0; i < n; ++i) {
            const double t =
                static_cast<double>(i) / static_cast<double>(n - 1);
            TrajectoryPoint pt(cursor_x + t * arc, 0.0, 0.0);
            pt.setV(1.0);
            pt.setDelta(d0 + t * (d1 - d0));
            pt.setA(0.0);
            pt.setDeltaDot(0.0);
            points.push_back(std::move(pt));
        }
        maneuvers.emplace_back(std::move(points), dir);
        cursor_x += arc;
    }
    path.finalize();
    return path;
}

// 测试目标：模拟 data7 NMPC 实际输出段结构，默认阈值(0.05m)下清洗无效果。
// 测试流程：用类似真实段弧长[7.5,4.3,0.35,1.6,1.3,3.1]m 方向交替 FWD/BWD
//          构造连续 Path，以默认 TopologyCleanupConfig 清洗。
// 预期结果：段数不变(6→6)，因为最短段 0.35m > 默认阈值 0.05m。
TEST(TopologyCleanerTest, RealisticNmpcOutputDefaultThresholdNoEffect) {
    Path path = BuildContinuousPath({
        {7.5, Direction::FORWARD, 0.00, 0.01},
        {4.3, Direction::BACKWARD, 0.01, 0.02},
        {0.35, Direction::FORWARD, 0.02, 0.03},  // 短碎步
        {1.6, Direction::BACKWARD, 0.03, 0.04},
        {1.3, Direction::FORWARD, 0.04, 0.05},
        {3.1, Direction::BACKWARD, 0.05, 0.06},
    });
    const std::size_t before = path.numManeuvers();
    auto& maneuvers = path.getManeuvers();
    ClassifyAndResetManeuvers(maneuvers, TopologyCleanupConfig{});
    // 默认 min_arc_length=0.05，最短段 0.35m 不应被标记
    for (const auto& m : maneuvers) {
        EXPECT_NE(m.direction, Direction::UNKNOWN)
            << "0.35m segment should NOT be marked UNKNOWN with 0.05m "
               "threshold";
    }
    Path cleaned = ReconstructPath(maneuvers);
    EXPECT_EQ(cleaned.numManeuvers(), before)
        << "Default threshold should not reduce maneuvers";
}

// 测试目标：提高阈值到 0.5m 后，0.35m 短碎步被识别为压平废段并剔除合并。
// 测试流程：用相同段结构，以 min_arc_length=0.5m 的配置清洗。
// 预期结果：段[2] (0.35m FWD, Δδ=0.01) 被标记 UNKNOWN 并剔除，
//          段[1] BWD 与段[3] BWD 同向合并 → 最终 5 段。
TEST(TopologyCleanerTest, RaisedThresholdCatchesAndMergesShortSegment) {
    Path path = BuildContinuousPath({
        {7.5, Direction::FORWARD, 0.00, 0.01},
        {4.3, Direction::BACKWARD, 0.01, 0.02},
        {0.35, Direction::FORWARD, 0.02, 0.03},  // 将被标记 UNKNOWN
        {1.6, Direction::BACKWARD, 0.03, 0.04},
        {1.3, Direction::FORWARD, 0.04, 0.05},
        {3.1, Direction::BACKWARD, 0.05, 0.06},
    });
    TopologyCleanupConfig config;
    config.min_arc_length = 0.5;  // 提高到能抓到 0.35m 段
    auto& maneuvers = path.getManeuvers();
    ClassifyAndResetManeuvers(maneuvers, config);
    // 段[2] 0.35m FWD, Δδ=0.01 < 0.1 → 应标记 UNKNOWN
    EXPECT_EQ(maneuvers[2].direction, Direction::UNKNOWN);
    // 其余段不应改变
    EXPECT_EQ(maneuvers[0].direction, Direction::FORWARD);
    EXPECT_EQ(maneuvers[1].direction, Direction::BACKWARD);
    EXPECT_EQ(maneuvers[3].direction, Direction::BACKWARD);
    EXPECT_EQ(maneuvers[4].direction, Direction::FORWARD);
    EXPECT_EQ(maneuvers[5].direction, Direction::BACKWARD);
    Path cleaned = ReconstructPath(maneuvers);
    // 剔除段[2](0.35m FWD)后剩5段，段[1]BWD与段[3]BWD同向合并再减1 → 4段
    EXPECT_EQ(cleaned.numManeuvers(), 4u)
        << "6 segments - 1 removed(0.35m) - 1 merged(BWD+BWD) = 4";
    // 验证合并后的方向序列：FWD, BWD(merged 4.3+1.6m), FWD, BWD
    const auto& result = cleaned.getManeuvers();
    ASSERT_GE(result.size(), 4u);
    EXPECT_EQ(result[0].direction, Direction::FORWARD);
    EXPECT_EQ(result[1].direction, Direction::BACKWARD);
    EXPECT_EQ(result[2].direction, Direction::FORWARD);
    EXPECT_EQ(result[3].direction, Direction::BACKWARD);
}

// 测试目标：验证短段+大Δδ在提高阈值后仍被正确转为PIVOT而非标记UNKNOWN。
// 测试流程：构造[FWD(2m), BWD(0.3m, Δδ=0.5rad), FWD(2m)]，min_arc_length=0.5m。
// 预期结果：0.3m段因 Δδ=0.5>0.1 被转为 PIVOT 而非 UNKNOWN，
//          三段均保留（PIVOT不与相邻FWD合并），最终仍为 3 段。
TEST(TopologyCleanerTest, RaisedThresholdShortPivotNotMerged) {
    Path path = BuildContinuousPath({
        {2.0, Direction::FORWARD, 0.0, 0.02},
        {0.3, Direction::BACKWARD, 0.0, 0.5},  // 短但大 Δδ → PIVOT
        {2.0, Direction::FORWARD, 0.02, 0.04},
    });
    TopologyCleanupConfig config;
    config.min_arc_length = 0.5;
    auto& maneuvers = path.getManeuvers();
    ClassifyAndResetManeuvers(maneuvers, config);
    EXPECT_EQ(maneuvers[1].direction, Direction::PIVOT);
    Path cleaned = ReconstructPath(maneuvers);
    // PIVOT 保留，不与相邻 FORWARD 合并
    EXPECT_EQ(cleaned.numManeuvers(), 3u);
}

// 测试目标：验证连续两个短碎步在提高阈值后被级联合并。
// 测试流程：构造[FWD(3m), BWD(0.15m,Δδ小), FWD(0.2m,Δδ小), BWD(3m)]。
//          两段短段均被标记 UNKNOWN 并剔除，剩余两个长段方向交替不合并。
// 预期结果：4→2 段（FWD和BWD各一，方向不同不合并）。
TEST(TopologyCleanerTest, CascadeRemovalOfConsecutiveShortSegments) {
    Path path = BuildContinuousPath({
        {3.0, Direction::FORWARD, 0.0, 0.01},
        {0.15, Direction::BACKWARD, 0.01, 0.02},  // 短碎步 1
        {0.2, Direction::FORWARD, 0.02, 0.03},    // 短碎步 2
        {3.0, Direction::BACKWARD, 0.03, 0.04},
    });
    TopologyCleanupConfig config;
    config.min_arc_length = 0.5;
    auto& maneuvers = path.getManeuvers();
    ClassifyAndResetManeuvers(maneuvers, config);
    EXPECT_EQ(maneuvers[1].direction, Direction::UNKNOWN);
    EXPECT_EQ(maneuvers[2].direction, Direction::UNKNOWN);
    Path cleaned = ReconstructPath(maneuvers);
    // 剔除两段后只剩 FWD + BWD，方向不同不合并 → 2 段
    EXPECT_EQ(cleaned.numManeuvers(), 2u);
    EXPECT_EQ(cleaned.getManeuvers()[0].direction, Direction::FORWARD);
    EXPECT_EQ(cleaned.getManeuvers()[1].direction, Direction::BACKWARD);
}

}  // namespace
}  // namespace apa_post_processor
