#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <vector>

#include "util/constants.h"
#include "util/trajectory.h"
#include "util/trajectory_point.h"

namespace apa_post_processor {
namespace {
// 测试辅助：构造半径 radius 的逆时针圆弧点序列，航向为切向加
// heading_offset（offset 取 PI 即得"倒车圆弧"），并逐点设定速度
std::vector<TrajectoryPoint> MakeArcPoints(double radius, double step,
                                           std::size_t count,
                                           double heading_offset,
                                           double velocity) {
    std::vector<TrajectoryPoint> points;
    points.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
        const double s = static_cast<double>(i) * step;
        const double angle = s / radius;
        TrajectoryPoint point(radius * std::sin(angle),
                              radius * (1.0 - std::cos(angle)),
                              angle + heading_offset);
        point.setV(velocity);
        points.push_back(std::move(point));
    }
    return points;
}
// 测试辅助：确定性白噪声（固定递推式，不用随机数以保证可复现）
double DeterministicNoise(double amplitude, std::size_t index) {
    constexpr double kModulus = 2147483648.0;
    double state =
        std::fmod(static_cast<double>(index) * 2654435761.0, kModulus);
    state = std::fmod(state * 1103515245.0 + 12345.0, kModulus);
    return amplitude * (2.0 * state / kModulus - 1.0);
}
// 测试辅助：有限值相对给定期望值的最大绝对偏差
double MaxAbsDeviation(const std::vector<double>& values, double expected) {
    double max_deviation = 0.0;
    for (const double value : values) {
        if (std::isfinite(value)) {
            max_deviation = std::max(max_deviation,
                                     std::abs(value - expected));
        }
    }
    return max_deviation;
}
}  // namespace

// 测试场景：半径 10m 的等弧长前进圆弧，航向严格沿切向。
// 预期行为：全部点（含段端——段端走单侧差分）曲率精确等于解析值
// 1/R，说明窗口口径不引入端部偏差。
TEST(TrajectoryKappaTest, ForwardArcMatchesAnalyticCurvature) {
    const std::vector<TrajectoryPoint> points =
        MakeArcPoints(10.0, DELTA_DIST, 60, 0.0, 1.0);
    const auto spans = Trajectory::SplitDirectionSpans(points);
    ASSERT_EQ(spans.size(), 1U);
    const auto series = Trajectory::ComputeWindowKappa(points, spans);
    ASSERT_EQ(series.kappa.size(), points.size());
    EXPECT_TRUE(std::isfinite(series.kappa.front()));
    EXPECT_TRUE(std::isfinite(series.kappa.back()));
    EXPECT_LT(MaxAbsDeviation(series.kappa, 0.1), 1e-6);
    // 弦长累积略短于弧长（每步二阶截断），故按 1e-5 量级比对
    EXPECT_NEAR(series.arc_length.back(), 59.0 * DELTA_DIST, 1e-5);
}

// 测试场景：直线路径（曲率恒为 0），航向完全不旋转。
// 预期行为：全部有限值点曲率接近 0，不因差分放大出虚假曲率。
TEST(TrajectoryKappaTest, StraightLineHasZeroCurvature) {
    std::vector<TrajectoryPoint> points;
    points.reserve(50);
    for (std::size_t i = 0; i < 50; ++i) {
        TrajectoryPoint point(static_cast<double>(i) * DELTA_DIST, 0.0, 0.0);
        point.setV(1.0);
        points.push_back(std::move(point));
    }
    const auto spans = Trajectory::SplitDirectionSpans(points);
    const auto series = Trajectory::ComputeWindowKappa(points, spans);
    EXPECT_LT(MaxAbsDeviation(series.kappa, 0.0), 1e-9);
}

// 测试场景：圆弧航向叠加 ±0.1° 确定性白噪声（模拟上游离散格点量化），
// 分别用 5cm 窄窗与 10cm 加宽窗统计曲率误差。
// 预期行为：两者都低于物理量程的一小部分，但加宽窗把白噪声型误差按
// 窗宽比例明显压低（同量级优于窄窗），这是选它做绘图口径的直接依据。
TEST(TrajectoryKappaTest, WidenedWindowReducesHeadingNoiseAmplification) {
    constexpr double kNoiseAmplitude = 0.1 * DEG2RAD;
    std::vector<TrajectoryPoint> points =
        MakeArcPoints(10.0, DELTA_DIST, 60, 0.0, 1.0);
    for (std::size_t i = 0; i < points.size(); ++i) {
        points[i].theta +=
            DeterministicNoise(kNoiseAmplitude, i);
    }
    const auto spans = Trajectory::SplitDirectionSpans(points);
    WindowKappaConfig narrow_config;
    narrow_config.half_window = DELTA_DIST;
    const auto narrow = Trajectory::ComputeWindowKappa(points, spans, narrow_config);
    const auto widened = Trajectory::ComputeWindowKappa(points, spans);
    const double narrow_error = MaxAbsDeviation(narrow.kappa, 0.1);
    const double widened_error = MaxAbsDeviation(widened.kappa, 0.1);
    EXPECT_LT(widened_error, 0.7 * narrow_error);
    EXPECT_LT(widened_error, 0.25 * 0.1);
}

// 测试场景：两段反向运动（前进 1m 后倒退约 1m），换挡点处航向折返 180°。
// 预期行为：方向段按速度符号切分为两段；折返点两侧分别走各自段的单侧
// 差分，任何有限值都不允许出现"跨段折返被混算"的巨大曲率，也不留空白。
TEST(TrajectoryKappaTest, SpanBoundaryIsNotCrossedAtGearShift) {
    std::vector<TrajectoryPoint> points;
    points.reserve(41);
    for (std::size_t i = 0; i < 21; ++i) {
        TrajectoryPoint point(static_cast<double>(i) * DELTA_DIST, 0.0, 0.0);
        point.setV(1.0);
        points.push_back(std::move(point));
    }
    for (std::size_t i = 1; i < 21; ++i) {
        TrajectoryPoint point(1.0 - static_cast<double>(i) * DELTA_DIST, 0.0,
                              PI);
        point.setV(-1.0);
        points.push_back(std::move(point));
    }
    const auto spans = Trajectory::SplitDirectionSpans(points);
    ASSERT_EQ(spans.size(), 2U);
    EXPECT_EQ(spans.front().begin, 0U);
    EXPECT_EQ(spans.front().end, 21U);
    EXPECT_EQ(spans.back().begin, 21U);
    EXPECT_EQ(spans.back().end, points.size());
    const auto series = Trajectory::ComputeWindowKappa(points, spans);
    EXPECT_LT(MaxAbsDeviation(series.kappa, 0.0), 1e-9);
    EXPECT_TRUE(std::isfinite(series.kappa.back()));
    EXPECT_TRUE(std::isfinite(series.kappa.front()));
}

// 测试场景：速度序列为 [+1, +1, 0.01, -1, -1]，中间夹一个停驻点，
// 点距 0.5m（两侧方向段均长于两倍半窗，不触发微段合并）。
// 预期行为：停驻点（|v| 低于阈值或缺少速度数据）并入当前游程段，
// 不产生虚假换挡边界；第二段从速度真正变号为负的点开始。
TEST(TrajectoryKappaTest, DwellPointsJoinCurrentDirectionRun) {
    std::vector<TrajectoryPoint> points;
    points.reserve(5);
    const double velocities[5] = {1.0, 1.0, 0.01, -1.0, -1.0};
    for (std::size_t i = 0; i < 5; ++i) {
        TrajectoryPoint point(static_cast<double>(i) * 0.5, 0.0, 0.0);
        point.setV(velocities[i]);
        points.push_back(std::move(point));
    }
    const auto spans = Trajectory::SplitDirectionSpans(points);
    ASSERT_EQ(spans.size(), 2U);
    EXPECT_EQ(spans[0].begin, 0U);
    EXPECT_EQ(spans[0].end, 3U);
    EXPECT_EQ(spans[1].begin, 3U);
    EXPECT_EQ(spans[1].end, 5U);
}

// 测试场景：方向游程中夹一个孤立反向点（点距 0.05m，模拟低速蠕行区的
// 速度抖动），其两侧都不是完整的分析窗口。
// 预期行为：该孤立点因两侧可用弧长都不足半窗而置 NaN；两侧长段照常
// 给出接近 0 的曲率，不被折返污染。
TEST(TrajectoryKappaTest, IsolatedDirectionRunStaysIsolated) {
    std::vector<TrajectoryPoint> points;
    points.reserve(21);
    for (std::size_t i = 0; i < 21; ++i) {
        TrajectoryPoint point(static_cast<double>(i) * DELTA_DIST, 0.0, 0.0);
        point.setV(i == 10U ? -1.0 : 1.0);
        points.push_back(std::move(point));
    }
    const auto spans = Trajectory::SplitDirectionSpans(points);
    ASSERT_EQ(spans.size(), 3U);
    EXPECT_EQ(spans[0].begin, 0U);
    EXPECT_EQ(spans[0].end, 10U);
    EXPECT_EQ(spans[1].begin, 10U);
    EXPECT_EQ(spans[1].end, 11U);
    EXPECT_EQ(spans[2].begin, 11U);
    EXPECT_EQ(spans[2].end, 21U);
    const auto series = Trajectory::ComputeWindowKappa(points, spans);
    EXPECT_TRUE(std::isnan(series.kappa[10]));
    EXPECT_LT(MaxAbsDeviation(series.kappa, 0.0), 1e-9);
}

// 测试场景：全部点都未携带速度数据（例如纯几何路径）。
// 预期行为：视为单一方向段，覆盖整个序列，不因缺数据抛异常。
TEST(TrajectoryKappaTest, MissingVelocityGivesSingleSpan) {
    std::vector<TrajectoryPoint> points;
    points.reserve(4);
    for (std::size_t i = 0; i < 4; ++i) {
        points.emplace_back(static_cast<double>(i) * DELTA_DIST, 0.0, 0.0);
    }
    const auto spans = Trajectory::SplitDirectionSpans(points);
    ASSERT_EQ(spans.size(), 1U);
    EXPECT_EQ(spans.front().begin, 0U);
    EXPECT_EQ(spans.front().end, points.size());
}

// 测试场景：倒车圆弧（航向取切向加 PI，几何上与前进圆弧同一轨迹）。
// 预期行为：曲率取几何符号——与前进圆弧同为 +1/R，即倒车段符号与
// tanδ/L 相反（判超限只看幅值，故绘图口径无需按方向再翻符号）。
TEST(TrajectoryKappaTest, BackwardArcKeepsGeometricSign) {
    const std::vector<TrajectoryPoint> points =
        MakeArcPoints(10.0, DELTA_DIST, 60, PI, -1.0);
    const auto spans = Trajectory::SplitDirectionSpans(points);
    ASSERT_EQ(spans.size(), 1U);
    const auto series = Trajectory::ComputeWindowKappa(points, spans);
    EXPECT_LT(MaxAbsDeviation(series.kappa, 0.1), 1e-6);
}

// 测试场景：换挡边界处存在重合点（机动段之间共享的边界点，弧长不前进）。
// 预期行为：累计弧长保持非递减、不产生 NaN 空洞或除零，κ 仍接近 0。
TEST(TrajectoryKappaTest, DuplicateBoundaryPointsDoNotBreakArcLength) {
    std::vector<TrajectoryPoint> points;
    points.reserve(6);
    for (std::size_t i = 0; i < 6; ++i) {
        TrajectoryPoint point(static_cast<double>(i) * DELTA_DIST, 0.0, 0.0);
        point.setV(1.0);
        points.push_back(point);
        if (i == 2U) {
            points.push_back(point);
        }
    }
    const auto spans = Trajectory::SplitDirectionSpans(points);
    const auto series = Trajectory::ComputeWindowKappa(points, spans);
    for (std::size_t i = 1; i < series.arc_length.size(); ++i) {
        EXPECT_GE(series.arc_length[i], series.arc_length[i - 1]);
    }
    EXPECT_LT(MaxAbsDeviation(series.kappa, 0.0), 1e-9);
}

// 测试场景：窗口配置非法（半窗非正、可用半窗下限大于半窗、段区间越界）。
// 预期行为：抛出 std::invalid_argument，避免静默产出无意义的曲率序列。
TEST(TrajectoryKappaTest, RejectsInvalidWindowConfigAndSpan) {
    const std::vector<TrajectoryPoint> points =
        MakeArcPoints(10.0, DELTA_DIST, 20, 0.0, 1.0);
    const auto spans = Trajectory::SplitDirectionSpans(points);
    WindowKappaConfig zero_window;
    zero_window.half_window = 0.0;
    EXPECT_THROW(Trajectory::ComputeWindowKappa(points, spans, zero_window),
                 std::invalid_argument);
    WindowKappaConfig inverted_window;
    inverted_window.half_window = 0.05;
    inverted_window.min_half_window = 0.1;
    EXPECT_THROW(Trajectory::ComputeWindowKappa(points, spans, inverted_window),
                 std::invalid_argument);
    const std::vector<PointSpan> bad_spans{{0U, points.size() + 1U}};
    EXPECT_THROW(Trajectory::ComputeWindowKappa(points, bad_spans), std::invalid_argument);
}
}  // namespace apa_post_processor
