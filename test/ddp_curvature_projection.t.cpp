// 参考保形曲率投影（ProjectReferenceCurvature）单元测试：θ 差分口径的
// 段曲率钳制、航向守恒再分配、整段超限回滚、cusp 独立投影、配置校验。
// 物理语义：前端 A* 参考的隐含曲率 κ = wrap(Δθ)/ds 可能超出车辆物理
// 上限 2~9%（δ 反解同一口径），投影把超限段钳到 κ_cap 并把被钳掉的
// 带符号航向摊到本 maneuver 的未钳段——只改 θ 不改位置，端点/换挡点/
// 同伦类因此自动保持
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "core/DDP/ddp_reference_builder.h"
#include "util/constants.h"

namespace apa_post_processor {
namespace {

// 测试车辆参数：轴距 3.0 m、δ_max 取真值 0.47728（κ_max ≈ 0.17224 /m）
constexpr double kWheelbase = 3.0;
constexpr double kDeltaMax = 0.47728;
constexpr double kCapRatio = 0.95;

double KappaCap() { return kCapRatio * std::tan(kDeltaMax) / kWheelbase; }

DdpCurvatureProjectionConfig MakeConfig() {
    DdpCurvatureProjectionConfig config;
    config.cap_ratio = kCapRatio;
    return config;
}

// 从当前路径末端沿直线追加点（步长 0.05 m，与 A* 点距一致）
void AppendLine(Path* path, double x_to, double y_to, double theta) {
    const double x_from = path->back().x;
    const double y_from = path->back().y;
    const int count = static_cast<int>(std::max(
        1L, std::lround(std::hypot(x_to - x_from, y_to - y_from) / 0.05)));
    for (int i = 1; i <= count; ++i) {
        path->addPoint({x_from + (x_to - x_from) * i / count,
                        y_from + (y_to - y_from) * i / count, theta});
    }
}

// 沿圆弧追加点：圆心 (cx,cy)、半径 radius、圆心角 alpha_from→alpha_to，
// heading 取「alpha 增大方向的切向 + theta_offset」（facing 惯例：
// 倒退 maneuver 的 heading 与位移反向，靠点序方向由 addPoint 推断，
// heading 本身在 cusp 处保持连续）
void AppendArc(Path* path, double cx, double cy, double radius,
               double alpha_from, double alpha_to, double theta_offset) {
    const double sweep = alpha_to - alpha_from;
    const int count = static_cast<int>(
        std::max(1L, std::lround(std::abs(sweep) * radius / 0.05)));
    for (int i = 1; i <= count; ++i) {
        const double alpha = alpha_from + sweep * i / count;
        path->addPoint({cx + radius * std::cos(alpha),
                        cy + radius * std::sin(alpha),
                        alpha + 0.5 * PI + theta_offset});
    }
}

// 全路径段曲率 κ = wrap(Δθ)/ds 的最大绝对值（跳过零位移段）——与
// 参考构建器 δ = atan(L·κ) 反解同一口径
double MaxAbsSegmentKappa(const Path& path) {
    std::vector<Pose> points;
    path.forEach([&points](const TrajectoryPoint& point) {
        points.emplace_back(point.x, point.y, point.theta);
    });
    double max_kappa = 0.0;
    for (std::size_t i = 1; i < points.size(); ++i) {
        const double ds = std::hypot(points[i].x - points[i - 1].x,
                                     points[i].y - points[i - 1].y);
        if (ds <= 1e-9) {
            continue;
        }
        const double kappa =
            std::abs(WrapAngle(points[i].theta - points[i - 1].theta)) / ds;
        max_kappa = std::max(max_kappa, kappa);
    }
    return max_kappa;
}

// 可投影场景：2 m 直线 + 半径 5 m 圆弧（κ=0.2 > κ_cap，扫 0.6 rad）+
// 2 m 切向引出直线，单个前进 maneuver。两侧直线段提供再分配容量
Path BuildProjectableArcPath() {
    Path path;
    path.addPoint({5.0, -2.0, 0.5 * PI});
    AppendLine(&path, 5.0, 0.0, 0.5 * PI);
    AppendArc(&path, 0.0, 0.0, 5.0, 0.0, 0.6, 0.0);
    AppendLine(&path, 5.0 * std::cos(0.6) - 2.0 * std::sin(0.6),
               5.0 * std::sin(0.6) + 2.0 * std::cos(0.6), 0.6 + 0.5 * PI);
    path.finalize();
    return path;
}

// 测试超限弧段被压回 κ_cap 以内：投影后全路径段曲率不超限、位置逐位
// 不变（长度/点数不变）、首末点航向保持（总航向变化严格守恒）
TEST(DdpCurvatureProjectionTest, OverCapArcProjectedBelowCap) {
    const Path input = BuildProjectableArcPath();
    ASSERT_GT(MaxAbsSegmentKappa(input), KappaCap());
    const Path projected =
        ProjectReferenceCurvature(input, kWheelbase, kDeltaMax, MakeConfig());
    EXPECT_LE(MaxAbsSegmentKappa(projected), KappaCap() + 1e-9);
    // 只改 θ 不改位置：点数与总长逐位不变
    EXPECT_EQ(projected.size(), input.size());
    EXPECT_DOUBLE_EQ(projected.length(), input.length());
    // 端点航向保持（守恒 ⟹ maneuver 端点逐位不变）
    EXPECT_NEAR(projected.front().theta, input.front().theta, 1e-9);
    EXPECT_NEAR(projected.back().theta, input.back().theta, 1e-9);
    EXPECT_EQ(projected.numManeuvers(), 1U);
}

// 测试整段超限回滚：无未钳段可摊（全 maneuver 都超限）时无可行投影，
// 必须原样返回（超涯保留）而非破坏端点航向守恒
TEST(DdpCurvatureProjectionTest, WholeManeuverOverCapRollsBack) {
    Path path;
    path.addPoint({5.0, 0.0, 0.5 * PI});
    AppendArc(&path, 0.0, 0.0, 5.0, 0.0, 0.6, 0.0);
    path.finalize();
    ASSERT_GT(MaxAbsSegmentKappa(path), KappaCap());
    const Path projected =
        ProjectReferenceCurvature(path, kWheelbase, kDeltaMax, MakeConfig());
    EXPECT_EQ(projected.size(), path.size());
    EXPECT_DOUBLE_EQ(projected.length(), path.length());
    EXPECT_DOUBLE_EQ(projected.back().theta, path.back().theta);
    EXPECT_GT(MaxAbsSegmentKappa(projected), KappaCap())
        << "整段超限且无摊派容量时必须回滚，超限保留";
}

// 测试 cusp 两侧 maneuver 各自独立投影：换挡点位置保留、方向序列
// （前进→倒退）保留、两侧段曲率均压回 cap、各 maneuver 端点航向不变
TEST(DdpCurvatureProjectionTest, CuspSplitManeuversProjectedIndependently) {
    Path path;
    path.addPoint({0.0, 0.0, 0.0});
    AppendLine(&path, 2.0, 0.0, 0.0);
    // 前进弧：圆心 (2,5)、半径 5，alpha 从 -π/2（即点 (2,0)）扫 0.6 rad
    AppendArc(&path, 2.0, 5.0, 5.0, -0.5 * PI, 0.6 - 0.5 * PI, 0.0);
    // 倒退弧：沿原弧折返 0.45 rad（facing 惯例：heading 保持切向不变，
    // 位移反向由点序表达，addPoint 依 facing 与位移点积判出 BACKWARD）
    AppendArc(&path, 2.0, 5.0, 5.0, 0.6 - 0.5 * PI, 0.15 - 0.5 * PI, 0.0);
    // 倒退引出直线：沿 heading 0.15 的反方向退 1.5 m
    AppendLine(&path, 2.0 + 5.0 * std::sin(0.15) - 1.5 * std::cos(0.15),
               5.0 - 5.0 * std::cos(0.15) - 1.5 * std::sin(0.15), 0.15);
    path.finalize();
    ASSERT_EQ(path.numManeuvers(), 2U);
    ASSERT_GT(MaxAbsSegmentKappa(path), KappaCap());
    const TrajectoryPoint cusp = path.getManeuvers()[0].points.back();
    const Path projected =
        ProjectReferenceCurvature(path, kWheelbase, kDeltaMax, MakeConfig());
    ASSERT_EQ(projected.numManeuvers(), 2U);
    EXPECT_EQ(projected.getManeuvers()[0].direction, Direction::FORWARD);
    EXPECT_EQ(projected.getManeuvers()[1].direction, Direction::BACKWARD);
    // 换挡点位置保留
    const TrajectoryPoint& projected_cusp =
        projected.getManeuvers()[0].points.back();
    EXPECT_NEAR(projected_cusp.x, cusp.x, 1e-9);
    EXPECT_NEAR(projected_cusp.y, cusp.y, 1e-9);
    // 两侧段曲率均压回 cap
    EXPECT_LE(MaxAbsSegmentKappa(projected), KappaCap() + 1e-9);
    // 各 maneuver 端点航向保持
    EXPECT_NEAR(projected.getManeuvers()[0].points.back().theta, cusp.theta,
                1e-9);
    EXPECT_NEAR(projected.getManeuvers()[1].points.back().theta,
                path.getManeuvers()[1].points.back().theta, 1e-9);
}

// 测试 ratio=0 关闭：输出与输入逐位一致
TEST(DdpCurvatureProjectionTest, ZeroRatioPassesThrough) {
    const Path input = BuildProjectableArcPath();
    DdpCurvatureProjectionConfig off;  // cap_ratio 默认 0
    const Path projected =
        ProjectReferenceCurvature(input, kWheelbase, kDeltaMax, off);
    EXPECT_EQ(projected.size(), input.size());
    EXPECT_DOUBLE_EQ(projected.length(), input.length());
    EXPECT_DOUBLE_EQ(projected.back().theta, input.back().theta);
}

// 测试未超限路径逐位透传：所有段 |κ| 均低于 cap 时不做任何调整
TEST(DdpCurvatureProjectionTest, UnderCapPathPassesThrough) {
    Path path;
    path.addPoint({20.0, -2.0, 0.5 * PI});
    AppendLine(&path, 20.0, 0.0, 0.5 * PI);
    AppendArc(&path, 0.0, 0.0, 20.0, 0.0, 0.6, 0.0);  // κ=0.05 < κ_cap
    AppendLine(&path, 20.0 * std::cos(0.6) - 2.0 * std::sin(0.6),
               20.0 * std::sin(0.6) + 2.0 * std::cos(0.6), 0.6 + 0.5 * PI);
    path.finalize();
    ASSERT_LT(MaxAbsSegmentKappa(path), KappaCap());
    const Path projected =
        ProjectReferenceCurvature(path, kWheelbase, kDeltaMax, MakeConfig());
    EXPECT_EQ(projected.size(), path.size());
    EXPECT_DOUBLE_EQ(projected.length(), path.length());
    EXPECT_DOUBLE_EQ(projected.back().theta, path.back().theta);
}

// 测试非法配置显式拒绝：比例越界/NaN、非正轴距、非正 δ_max 都会让
// 投影行为不可预期，必须抛出
TEST(DdpCurvatureProjectionTest, InvalidConfigThrows) {
    const Path path = BuildProjectableArcPath();
    DdpCurvatureProjectionConfig config = MakeConfig();
    config.cap_ratio = -0.1;
    EXPECT_THROW(ProjectReferenceCurvature(path, kWheelbase, kDeltaMax, config),
                 std::invalid_argument);
    config.cap_ratio = 1.5;
    EXPECT_THROW(ProjectReferenceCurvature(path, kWheelbase, kDeltaMax, config),
                 std::invalid_argument);
    config.cap_ratio = std::nan("");
    EXPECT_THROW(ProjectReferenceCurvature(path, kWheelbase, kDeltaMax, config),
                 std::invalid_argument);
    EXPECT_THROW(ProjectReferenceCurvature(path, 0.0, kDeltaMax, MakeConfig()),
                 std::invalid_argument);
    EXPECT_THROW(ProjectReferenceCurvature(path, kWheelbase, 0.0, MakeConfig()),
                 std::invalid_argument);
}

}  // namespace
}  // namespace apa_post_processor
