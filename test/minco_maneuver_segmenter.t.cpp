#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>
#include <vector>

#include "core/MINCO/minco_maneuver_segmenter.h"
#include "util/constants.h"

namespace apa_post_processor {
namespace {

// 测试配置：d_seg=0.6、标称速度 0.5 m/s、标称转向角速度 0.3 rad/s、时长下限
// 0.5 s；结构性用例显式关闭微段融合（fuse_arc_threshold=0），避免默认融合
// 阈值干扰方向切分/弧长累积等结构断言（融合行为由专门用例覆盖）
MincoConfig MakeConfig() {
    MincoConfig config;
    config.fuse_arc_threshold = 0.0;
    return config;
}

// 测试用派生类：暴露受保护的单 Maneuver 解析入口，供防御分支白盒测试
class MincoManeuverSegmenterTestAccessor : public MincoManeuverSegmenter {
   public:
    using MincoManeuverSegmenter::MincoManeuverSegmenter;
    using MincoManeuverSegmenter::segmentManeuver;
};

// 从当前路径末端沿 x 轴追加直线路径点（步长 0.05 m，与 A* 点距一致）
void AppendXLine(Path* path, double x_from, double x_to, double theta) {
    const int count =
        static_cast<int>(std::round(std::abs(x_to - x_from) / 0.05));
    for (int i = 1; i <= count; ++i) {
        const double x = x_from + (x_to - x_from) * i / count;
        path->addPoint({x, 0.0, theta});
    }
}

// 构造 前进1.0m → 后退0.7m → 前进0.5m 的两次换挡路径
Path BuildTwoShiftPath() {
    Path path;
    path.addPoint({0.0, 0.0, 0.0});
    AppendXLine(&path, 0.0, 1.0, 0.0);
    AppendXLine(&path, 1.0, 0.3, 0.0);
    AppendXLine(&path, 0.3, 0.8, 0.0);
    path.finalize();
    return path;
}

// 测试两次换挡路径的宏观切分与带符号累积弧长。
// 因为换挡点即 Maneuver 交界，所以方向序列必须为 前进/后退/前进，且弧长
// 在换挡点处连续、后退段沿负方向累积。
TEST(MincoManeuverSegmenterTest,
     ShiftSplitProducesCorrectDirectionsAndSignedArcLengths) {
    const MincoManeuverSegmenter segmenter(MakeConfig());
    const Path path = BuildTwoShiftPath();
    ASSERT_EQ(path.numManeuvers(), 3);

    const std::vector<MincoManeuverEstimate> estimates = segmenter.segment(path);

    ASSERT_EQ(estimates.size(), 3);
    EXPECT_EQ(estimates[0].direction, Direction::FORWARD);
    EXPECT_EQ(estimates[1].direction, Direction::BACKWARD);
    EXPECT_EQ(estimates[2].direction, Direction::FORWARD);
    // 第一段：L=1.0，M=ceil(1/0.6)=2，K=10，段终点 x=0.5/1.0
    ASSERT_EQ(estimates[0].segments.size(), 2);
    EXPECT_NEAR(estimates[0].segments[0].arc_length, 0.5, 1e-9);
    EXPECT_NEAR(estimates[0].segments[1].arc_length, 1.0, 1e-9);
    EXPECT_NEAR(estimates[0].segments[1].desired_position.x(), 1.0, 1e-9);
    // 第二段（后退）：L=0.7，M=2，K=7，段终点 x=0.65/0.3，弧长负向累积
    EXPECT_NEAR(estimates[1].start_arc_length, 1.0, 1e-9);
    ASSERT_EQ(estimates[1].segments.size(), 2);
    EXPECT_NEAR(estimates[1].segments[0].arc_length, 0.65, 1e-9);
    EXPECT_NEAR(estimates[1].segments[1].arc_length, 0.3, 1e-9);
    EXPECT_NEAR(estimates[1].segments[1].desired_position.x(), 0.3, 1e-9);
    EXPECT_LT(estimates[1].segments[1].arc_length,
              estimates[1].segments[0].arc_length);
    // 第三段（前进）：L=0.5，M=1，段终点 x=0.8
    EXPECT_NEAR(estimates[2].start_arc_length, 0.3, 1e-9);
    ASSERT_EQ(estimates[2].segments.size(), 1);
    EXPECT_NEAR(estimates[2].segments[0].arc_length, 0.8, 1e-9);
    EXPECT_NEAR(estimates[2].segments[0].desired_position.x(), 0.8, 1e-9);
}

// 测试给定弧长/标称段长下分段数 M 与 K_step 锚点抽取的手工对拍。
// 因为 M=ceil(L/d_seg)、K_step=floor((N-1)/M) 是设计文档给出的确定公式，
// 所以锚点位置必须与手工构造的预期逐点一致。
TEST(MincoManeuverSegmenterTest,
     SegmentCountAndAnchorExtractionMatchHandComputation) {
    Path path;
    path.addPoint({0.0, 0.0, 0.0});
    AppendXLine(&path, 0.0, 1.0, 0.0);
    path.finalize();
    ASSERT_EQ(path.numManeuvers(), 1);

    // d_seg=0.3：M=ceil(1/0.3)=4，K=floor(20/4)=5，锚点 {0,5,10,15,20}
    MincoConfig config = MakeConfig();
    config.nominal_segment_length = 0.3;
    const std::vector<MincoManeuverEstimate> estimates =
        MincoManeuverSegmenter(config).segment(path);
    ASSERT_EQ(estimates.size(), 1);
    ASSERT_EQ(estimates[0].segments.size(), 4);
    const double expected_positions[] = {0.25, 0.5, 0.75, 1.0};
    for (int i = 0; i < 4; ++i) {
        EXPECT_NEAR(estimates[0].segments[i].desired_position.x(),
                    expected_positions[i], 1e-9);
        EXPECT_NEAR(estimates[0].segments[i].arc_length, expected_positions[i],
                    1e-9);
        EXPECT_NEAR(estimates[0].segments[i].theta, 0.0, 1e-12);
        // 时长初值：max(0.25/0.5, 0, 0.5) = 0.5
        EXPECT_NEAR(estimates[0].segments[i].duration, 0.5, 1e-9);
    }
    // d_seg=0.4：M=3，K=6，锚点 {0,6,12,20}，末段强制对齐路径终点（更长）
    config.nominal_segment_length = 0.4;
    const std::vector<MincoManeuverEstimate> tail_estimates =
        MincoManeuverSegmenter(config).segment(path);
    ASSERT_EQ(tail_estimates[0].segments.size(), 3);
    EXPECT_NEAR(tail_estimates[0].segments[0].desired_position.x(), 0.3, 1e-9);
    EXPECT_NEAR(tail_estimates[0].segments[1].desired_position.x(), 0.6, 1e-9);
    EXPECT_NEAR(tail_estimates[0].segments[2].desired_position.x(), 1.0, 1e-9);
    EXPECT_NEAR(tail_estimates[0].segments[2].duration, 0.8, 1e-9);
}

// 测试原地转向（PIVOT）机动的处理。
// 因为 PIVOT 不产生位移，所以弧长必须保持不变，时长初值由朝向变化量与
// 标称转向角速度决定。
TEST(MincoManeuverSegmenterTest,
     PivotManeuverKeepsArcLengthAndEstimatesTurnDuration) {
    Path path;
    path.addPoint({0.0, 0.0, 0.0});
    AppendXLine(&path, 0.0, 1.0, 0.0);
    path.addPoint({1.0, 0.0, 0.2});
    path.addPoint({1.0, 0.0, 0.4});
    path.addPoint({1.0, 0.0, 0.6});
    // 沿新朝向 0.6 rad 继续前进
    for (int i = 1; i <= 5; ++i) {
        path.addPoint(
            {1.0 + 0.05 * std::cos(0.6) * i, 0.05 * std::sin(0.6) * i, 0.6});
    }
    path.finalize();
    ASSERT_EQ(path.numManeuvers(), 3);

    const std::vector<MincoManeuverEstimate> estimates =
        MincoManeuverSegmenter(MakeConfig()).segment(path);

    ASSERT_EQ(estimates.size(), 3);
    EXPECT_EQ(estimates[1].direction, Direction::PIVOT);
    // PIVOT 段：L=0 退化为单段，弧长不变，时长 = 0.6/0.3 = 2.0 s
    ASSERT_EQ(estimates[1].segments.size(), 1);
    EXPECT_NEAR(estimates[1].segments[0].arc_length, 1.0, 1e-9);
    EXPECT_NEAR(estimates[1].segments[0].theta, 0.6, 1e-9);
    EXPECT_NEAR(estimates[1].segments[0].duration, 2.0, 1e-9);
    // 后续前进段从不变弧长继续累积
    EXPECT_NEAR(estimates[2].start_arc_length, 1.0, 1e-9);
    EXPECT_NEAR(estimates[2].segments.back().arc_length, 1.25, 1e-9);
}

// 测试弧长小于标称段长的短 Maneuver 退化为单段。
// 因为 M=ceil(L/d_seg) 在 L<d_seg 时等于 1，所以必须产出恰好 1 段而非
// M=0。
TEST(MincoManeuverSegmenterTest, ShortManeuverDegeneratesToSingleSegment) {
    Path path;
    path.addPoint({0.0, 0.0, 0.0});
    AppendXLine(&path, 0.0, 0.2, 0.0);
    path.finalize();

    const std::vector<MincoManeuverEstimate> estimates =
        MincoManeuverSegmenter(MakeConfig()).segment(path);

    ASSERT_EQ(estimates.size(), 1);
    ASSERT_EQ(estimates[0].segments.size(), 1);
    EXPECT_NEAR(estimates[0].segments[0].arc_length, 0.2, 1e-9);
    EXPECT_NEAR(estimates[0].segments[0].duration, 0.5, 1e-9);
}

// 测试单点与两点路径的退化行为。
// 因为退化输入不得产生 M=0 或崩溃，所以单点 Maneuver 也必须有定义良好的
// 单段输出。
TEST(MincoManeuverSegmenterTest, SinglePointAndTwoPointPathsHaveDefinedBehavior) {
    const MincoManeuverSegmenter segmenter(MakeConfig());
    // 单点路径：1 个 UNKNOWN 方向 Maneuver，1 段，弧长 0，时长取下限
    Path single_point_path;
    single_point_path.addPoint({1.0, 2.0, 0.5});
    single_point_path.finalize();
    const std::vector<MincoManeuverEstimate> single_estimates =
        segmenter.segment(single_point_path);
    ASSERT_EQ(single_estimates.size(), 1);
    EXPECT_EQ(single_estimates[0].direction, Direction::UNKNOWN);
    ASSERT_EQ(single_estimates[0].segments.size(), 1);
    EXPECT_NEAR(single_estimates[0].segments[0].arc_length, 0.0, 1e-12);
    EXPECT_NEAR(single_estimates[0].segments[0].theta, 0.5, 1e-12);
    EXPECT_NEAR(single_estimates[0].segments[0].duration, 0.5, 1e-9);
    EXPECT_NEAR(single_estimates[0].segments[0].desired_position.x(), 1.0,
                1e-12);
    // 两点路径：1 段，弧长 0.1
    Path two_point_path;
    two_point_path.addPoint({0.0, 0.0, 0.0});
    two_point_path.addPoint({0.1, 0.0, 0.0});
    two_point_path.finalize();
    const std::vector<MincoManeuverEstimate> two_estimates =
        segmenter.segment(two_point_path);
    ASSERT_EQ(two_estimates.size(), 1);
    ASSERT_EQ(two_estimates[0].segments.size(), 1);
    EXPECT_NEAR(two_estimates[0].segments[0].arc_length, 0.1, 1e-9);
}

// 测试空 Path 的拒绝行为。
// 因为空输入没有可解析的几何内容，所以必须显式抛出标准异常。
TEST(MincoManeuverSegmenterTest, EmptyPathThrows) {
    const MincoManeuverSegmenter segmenter(MakeConfig());
    const Path empty_path;
    EXPECT_THROW(segmenter.segment(empty_path), std::invalid_argument);
}

// 测试空 Maneuver 的防御分支。
// 因为 Path 层不产生空 Maneuver（该分支为防御性代码），所以通过白盒入口
// 直接构造空 Maneuver 验证显式拒绝，防止未来手动构造场景下静默产生
// M=0 的未定义输出。
TEST(MincoManeuverSegmenterTest, EmptyManeuverThrows) {
    const MincoManeuverSegmenterTestAccessor segmenter(MincoConfig{});
    const Maneuver empty_maneuver;
    double cumulative_arc = 0.0;
    double prev_theta = 0.0;
    EXPECT_THROW(
        segmenter.segmentManeuver(empty_maneuver, &cumulative_arc, &prev_theta),
        std::invalid_argument);
}

// 测试朝向角跨越 ±π 边界时的解缠绕。
// 因为 MINCO 多项式拟合的是连续 θ(t)，所以段初值 θ_m 不允许出现 2π 跳变。
TEST(MincoManeuverSegmenterTest, ThetaIsUnwrappedAcrossWrapBoundary) {
    // 单位圆上 φ∈[80°,100°] 的圆弧（步长 1°），朝向 = φ+90°，跨越 ±π
    Path path;
    for (int deg = 80; deg <= 100; ++deg) {
        const double phi = deg * DEG2RAD;
        path.addPoint({std::cos(phi), std::sin(phi),
                       std::remainder(phi + 0.5 * PI, 2.0 * PI)});
    }
    path.finalize();
    ASSERT_EQ(path.numManeuvers(), 1);

    MincoConfig config = MakeConfig();
    config.nominal_segment_length = 0.1;
    const std::vector<MincoManeuverEstimate> estimates =
        MincoManeuverSegmenter(config).segment(path);

    // L = 20° 弧长 ≈ 0.349，M=4，段终点朝向 175°/180°/185°/190°（解缠绕）
    ASSERT_EQ(estimates.size(), 1);
    ASSERT_EQ(estimates[0].segments.size(), 4);
    for (int i = 0; i < 4; ++i) {
        const double expected_theta = (175.0 + 5.0 * i) * DEG2RAD;
        EXPECT_NEAR(estimates[0].segments[i].theta, expected_theta, 1e-6);
        if (i > 0) {
            // 任意相邻段朝向差必须是平滑小量，不允许 2π 跳变
            EXPECT_NEAR(estimates[0].segments[i].theta -
                            estimates[0].segments[i - 1].theta,
                        5.0 * DEG2RAD, 1e-6);
        }
    }
}

// 测试非法配置的拒绝行为。
// 因为错误配置会静默污染全部段初值，所以必须在构造期显式失败。
TEST(MincoManeuverSegmenterTest, InvalidConfigThrows) {
    const auto expect_throw = [](MincoConfig config) {
        EXPECT_THROW(MincoManeuverSegmenter{config}, std::invalid_argument);
    };
    expect_throw([] {
        auto config = MakeConfig();
        config.nominal_segment_length = 0.0;
        return config;
    }());
    expect_throw([] {
        auto config = MakeConfig();
        config.nominal_speed = -0.5;
        return config;
    }());
    expect_throw([] {
        auto config = MakeConfig();
        config.nominal_turn_rate = 0.0;
        return config;
    }());
    expect_throw([] {
        auto config = MakeConfig();
        config.min_segment_duration = -0.1;
        return config;
    }());
    expect_throw([] {
        auto config = MakeConfig();
        config.nominal_segment_length =
            std::numeric_limits<double>::quiet_NaN();
        return config;
    }());
    expect_throw([] {
        auto config = MakeConfig();
        config.fuse_arc_threshold = -0.1;
        return config;
    }());
    expect_throw([] {
        auto config = MakeConfig();
        config.fuse_heading_threshold = 0.0;
        return config;
    }());
    EXPECT_NO_THROW(MincoManeuverSegmenter(MincoConfig{}));
}

// ===== 微段融合（fuse_arc_threshold > 0） =====

// 测试辅助：构造 前进2.0m → 后退0.3m → 前进2.0m 的摆动路径
// （中段为微小后退摆动，|Δs|=0.3m、|Δθ|≈0）
Path BuildWigglePath() {
    Path path;
    path.addPoint({0.0, 0.0, 0.0});
    AppendXLine(&path, 0.0, 2.0, 0.0);
    AppendXLine(&path, 2.0, 1.7, 0.0);
    AppendXLine(&path, 1.7, 3.7, 0.0);
    path.finalize();
    return path;
}

// 测试辅助：校验估计序列的弧长连续性——首段 start_arc_length 为 0、
// 每个后续段的 start_arc_length 等于前一段的末段 arc_length
void ExpectArcContinuity(const std::vector<MincoManeuverEstimate>& estimates) {
    ASSERT_FALSE(estimates.empty());
    EXPECT_NEAR(estimates.front().start_arc_length, 0.0, 1e-9);
    for (std::size_t i = 1; i < estimates.size(); ++i) {
        EXPECT_NEAR(estimates[i].start_arc_length,
                    estimates[i - 1].segments.back().arc_length, 1e-9)
            << "run " << i << " start arc discontinuity";
    }
}

// 测试场景：融合关闭（fuse_arc_threshold=0）时的摆动路径分段。
// 预期行为：3 个估计原样保留，无任何融合。
TEST(MincoManeuverSegmenterTest, FusionOffSwitchKeepsAllRuns) {
    const Path path = BuildWigglePath();
    ASSERT_EQ(path.numManeuvers(), 3);
    const auto estimates = MincoManeuverSegmenter(MakeConfig()).segment(path);
    ASSERT_EQ(estimates.size(), 3);
    EXPECT_EQ(estimates[1].direction, Direction::BACKWARD);
    ExpectArcContinuity(estimates);
}

// 测试场景：开启融合（弧长阈值 0.5m、朝向阈值 0.2 rad）处理 F-B-F 摆动。
// 预期行为：中间 0.3m 后退摆动被移除，两个前进段同向合并为 1 段；
// 合并后总带符号弧长为 4.0m（摆动移除后车辆不再往返），弧长连续。
TEST(MincoManeuverSegmenterTest,
     FusionRemovesInteriorWiggleAndMergesSameDirection) {
    auto config = MakeConfig();
    config.fuse_arc_threshold = 0.5;
    config.fuse_heading_threshold = 0.2;
    const auto estimates =
        MincoManeuverSegmenter(config).segment(BuildWigglePath());
    ASSERT_EQ(estimates.size(), 1);
    EXPECT_EQ(estimates[0].direction, Direction::FORWARD);
    EXPECT_NEAR(estimates[0].start_arc_length, 0.0, 1e-9);
    EXPECT_NEAR(estimates[0].segments.back().arc_length, 4.0, 1e-9);
    // 终点锚点保持原路径终点 (3.7, 0)
    EXPECT_NEAR(estimates[0].segments.back().desired_position.x(), 3.7, 1e-9);
    ExpectArcContinuity(estimates);
}

// 测试场景：首/末段为微小段的摆动路径。
// 预期行为：首末段绝对保护，即使满足融合判据量也不被移除。
TEST(MincoManeuverSegmenterTest, FusionProtectsFirstAndLastRuns) {
    // 末段微小：前进2.0m → 后退0.3m
    Path last_tiny;
    last_tiny.addPoint({0.0, 0.0, 0.0});
    AppendXLine(&last_tiny, 0.0, 2.0, 0.0);
    AppendXLine(&last_tiny, 2.0, 1.7, 0.0);
    last_tiny.finalize();
    // 首段微小：后退0.3m → 前进2.0m
    Path first_tiny;
    first_tiny.addPoint({2.0, 0.0, 0.0});
    AppendXLine(&first_tiny, 2.0, 1.7, 0.0);
    AppendXLine(&first_tiny, 1.7, 3.7, 0.0);
    first_tiny.finalize();
    auto config = MakeConfig();
    config.fuse_arc_threshold = 0.5;
    config.fuse_heading_threshold = 0.2;
    const MincoManeuverSegmenter segmenter(config);
    EXPECT_EQ(segmenter.segment(last_tiny).size(), 2);
    EXPECT_EQ(segmenter.segment(first_tiny).size(), 2);
}

// 测试场景：内部微段带大朝向变化（|Δθ|=1.0 rad 的真实转向调整）。
// 预期行为：朝向阈值保护其不被融合，3 段保留。
TEST(MincoManeuverSegmenterTest, FusionSkipsLargeHeadingChangeRuns) {
    Path path;
    path.addPoint({0.0, 0.0, 0.0});
    AppendXLine(&path, 0.0, 1.0, 0.0);
    // 后退 0.3m 且朝向从 0 变为 1.0 rad（真实转向，非摆动）
    AppendXLine(&path, 1.0, 0.7, 1.0);
    AppendXLine(&path, 0.7, 1.7, 1.0);
    path.finalize();
    ASSERT_EQ(path.numManeuvers(), 3);
    auto config = MakeConfig();
    config.fuse_arc_threshold = 0.5;
    config.fuse_heading_threshold = 0.2;
    const auto estimates = MincoManeuverSegmenter(config).segment(path);
    EXPECT_EQ(estimates.size(), 3);
    ExpectArcContinuity(estimates);
}

// 测试场景：前进段夹两处微小后退摆动的链式路径（F-B-F-B-F）。
// 预期行为：两处摆动全部移除，三个前进段合并为 1 段；总带符号弧长
// 3.0m，弧长连续。
TEST(MincoManeuverSegmenterTest, FusionChainsAcrossMultipleWiggles) {
    Path path;
    path.addPoint({0.0, 0.0, 0.0});
    AppendXLine(&path, 0.0, 1.0, 0.0);
    AppendXLine(&path, 1.0, 0.8, 0.0);
    AppendXLine(&path, 0.8, 1.8, 0.0);
    AppendXLine(&path, 1.8, 1.6, 0.0);
    AppendXLine(&path, 1.6, 2.6, 0.0);
    path.finalize();
    ASSERT_EQ(path.numManeuvers(), 5);
    auto config = MakeConfig();
    config.fuse_arc_threshold = 0.5;
    config.fuse_heading_threshold = 0.2;
    const auto estimates = MincoManeuverSegmenter(config).segment(path);
    ASSERT_EQ(estimates.size(), 1);
    EXPECT_NEAR(estimates[0].segments.back().arc_length, 3.0, 1e-9);
    EXPECT_NEAR(estimates[0].segments.back().desired_position.x(), 2.6, 1e-9);
    ExpectArcContinuity(estimates);
}

// 测试场景：内部 PIVOT 微段（原地旋转，|Δs|≈0 但方向为 PIVOT）。
// 预期行为：PIVOT 不是 FORWARD/BACKWARD，永不融合，3 段保留。
TEST(MincoManeuverSegmenterTest, PivotRunIsNeverFused) {
    Path path;
    path.addPoint({0.0, 0.0, 0.0});
    AppendXLine(&path, 0.0, 1.0, 0.0);
    // 近似原地旋转：位移 <0.05m 且朝向变化 >3°，触发 PIVOT 机动段
    path.addPoint({1.0, 0.0, 0.5});
    AppendXLine(&path, 1.0, 2.0, 0.5);
    path.finalize();
    ASSERT_EQ(path.numManeuvers(), 3);
    ASSERT_EQ(path.getManeuvers()[1].direction, Direction::PIVOT);
    auto config = MakeConfig();
    config.fuse_arc_threshold = 0.5;
    config.fuse_heading_threshold = 0.2;
    const auto estimates = MincoManeuverSegmenter(config).segment(path);
    EXPECT_EQ(estimates.size(), 3);
    EXPECT_EQ(estimates[1].direction, Direction::PIVOT);
    ExpectArcContinuity(estimates);
}

}  // namespace
}  // namespace apa_post_processor
