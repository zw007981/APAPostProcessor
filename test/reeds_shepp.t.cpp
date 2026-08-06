#include "../src/util/reeds_shepp.h"

#include <gtest/gtest.h>

#include <cmath>
#include <random>
#include <vector>

#include "../src/util/constants.h"

namespace apa_post_processor {
namespace {
// 沿 RS 路径积分得到的终点位姿：用采样序列的末点近似（采样步长足够细时
// 与闭式终点一致），供「解真的连接了起终点」这一根本性质的校验
Pose IntegrateEndPose(const RsPath& path, const Pose& start, double radius) {
    const auto samples = SampleReedsShepp(path, start, radius, 0.01);
    return samples.back().pose;
}

// 角度差归一化后取绝对值
double AngleDiff(double a, double b) {
    return std::fabs(std::remainder(a - b, 2.0 * PI));
}

// 测试直线可达构型退化为纯直行：起终点同向共线时最短解就是一段直行，
// 长度等于欧氏距离。触发原因是该构型是闭式解的最基础退化情形，若此处
// 就有偏差说明归一化变换或基元积分公式写错了。预期行为：解有效、无尖点、
// 弧长等于两点距离
TEST(ReedsSheppTest, CollinearSameHeadingDegeneratesToStraightLine) {
    const Pose start{0.0, 0.0, 0.0};
    const Pose goal{5.0, 0.0, 0.0};
    const auto path = ComputeShortestReedsShepp(start, goal, 2.0);
    ASSERT_TRUE(path.valid);
    EXPECT_EQ(path.numCusps(), 0);
    EXPECT_NEAR(path.arcLength(2.0), 5.0, 1e-9);
}

// 测试起终点重合时最短解长度为零：该构型是长度下界的锚点，若枚举器把
// 某个「绕一整圈」的解误判为更短，说明最短候选比较逻辑失效。
// 预期行为：解有效且归一化长度为零
TEST(ReedsSheppTest, IdenticalPosesYieldZeroLengthPath) {
    const Pose pose{1.5, -2.5, 0.7};
    const auto path = ComputeShortestReedsShepp(pose, pose, 3.0);
    ASSERT_TRUE(path.valid);
    EXPECT_NEAR(path.normalized_length, 0.0, 1e-9);
}

// 测试解的终点确实落在目标位姿上：这是 RS 闭式解唯一不可妥协的正确性
// 判据。触发原因是闭式解由 5 个族、每族 4~8 种对称变换拼装，任一变换的
// 符号或转向镜像写反都会产生「长度更短但终点错误」的伪解，而伪解会被
// 最短候选比较器优先选中、静默污染全部下游几何。
// 预期行为：随机构型下积分终点与目标位姿在位置与航向上均一致
TEST(ReedsSheppTest, ShortestPathReachesGoalForRandomPoses) {
    constexpr double RADIUS = 2.5;
    std::mt19937 rng(20260804u);
    std::uniform_real_distribution<double> pos_dist(-12.0, 12.0);
    std::uniform_real_distribution<double> ang_dist(-PI, PI);
    for (int trial = 0; trial < 500; ++trial) {
        const Pose start{pos_dist(rng), pos_dist(rng), ang_dist(rng)};
        const Pose goal{pos_dist(rng), pos_dist(rng), ang_dist(rng)};
        const auto path = ComputeShortestReedsShepp(start, goal, RADIUS);
        ASSERT_TRUE(path.valid) << "trial=" << trial;
        const Pose reached = IntegrateEndPose(path, start, RADIUS);
        EXPECT_NEAR(reached.x, goal.x, 1e-6) << "trial=" << trial;
        EXPECT_NEAR(reached.y, goal.y, 1e-6) << "trial=" << trial;
        EXPECT_NEAR(AngleDiff(reached.theta, goal.theta), 0.0, 1e-6)
            << "trial=" << trial;
    }
}

// 测试最短解不劣于任何单族解：枚举器必须真的取到全局最小，而不是被
// 某一族的解提前锁死。触发原因是候选比较器一旦漏掉 valid 标志的处理，
// 首个候选就会永久占位。预期行为：随机构型下最短长度不超过其中任一
// 有效候选的长度，此处用「解长度不超过朴素上界」作可判定的必要条件
TEST(ReedsSheppTest, ShortestPathIsBoundedByNaiveUpperBound) {
    constexpr double RADIUS = 2.0;
    std::mt19937 rng(19260817u);
    std::uniform_real_distribution<double> pos_dist(-8.0, 8.0);
    std::uniform_real_distribution<double> ang_dist(-PI, PI);
    for (int trial = 0; trial < 200; ++trial) {
        const Pose start{pos_dist(rng), pos_dist(rng), ang_dist(rng)};
        const Pose goal{pos_dist(rng), pos_dist(rng), ang_dist(rng)};
        const auto path = ComputeShortestReedsShepp(start, goal, RADIUS);
        ASSERT_TRUE(path.valid);
        // 「原地转向到目标方位 + 直行 + 原地转向到目标航向」不是可行的
        // RS 解，但其长度是一个宽松的上界：任何有界曲率解都不会比
        // 「先绕最多半圈、直行、再绕最多半圈」更长
        const double straight = std::hypot(goal.x - start.x, goal.y - start.y);
        const double upper = straight + 2.0 * PI * RADIUS;
        EXPECT_LE(path.arcLength(RADIUS), upper) << "trial=" << trial;
    }
}

// 测试采样点的相邻间距不超过步长且首尾对齐：下游要靠采样序列做逐点
// 碰撞校验，间距若超标会漏检薄障碍物。触发原因是分段离散的步数取整
// 若用 floor 而非 ceil 就会产生超标间距。预期行为：首点等于起点、
// 末点等于终点、任意相邻点间距不超过步长
TEST(ReedsSheppTest, SamplingRespectsStepAndEndpoints) {
    constexpr double RADIUS = 1.8;
    constexpr double STEP = 0.05;
    const Pose start{-1.0, 2.0, 1.2};
    const Pose goal{4.0, -3.0, -2.0};
    const auto path = ComputeShortestReedsShepp(start, goal, RADIUS);
    ASSERT_TRUE(path.valid);
    const auto samples = SampleReedsShepp(path, start, RADIUS, STEP);
    ASSERT_GE(samples.size(), 2u);
    EXPECT_NEAR(samples.front().pose.x, start.x, 1e-12);
    EXPECT_NEAR(samples.front().pose.y, start.y, 1e-12);
    EXPECT_NEAR(samples.back().pose.x, goal.x, 1e-6);
    EXPECT_NEAR(samples.back().pose.y, goal.y, 1e-6);
    for (std::size_t i = 1; i < samples.size(); ++i) {
        const double gap = std::hypot(samples[i].pose.x - samples[i - 1].pose.x,
                                      samples[i].pose.y - samples[i - 1].pose.y);
        EXPECT_LE(gap, STEP + 1e-9) << "i=" << i;
    }
}

// 测试尖点计数与采样方向标记一致：换挡段数是本模块存在的全部理由，
// 计数口径必须与「采样点方向标记发生翻转的次数」严格一致，否则上层
// 依据尖点数做的接受判据会与真实几何脱节。
// 预期行为：随机构型下两种口径给出相同的换挡次数
TEST(ReedsSheppTest, CuspCountMatchesSampledDirectionFlips) {
    constexpr double RADIUS = 2.2;
    std::mt19937 rng(31415926u);
    std::uniform_real_distribution<double> pos_dist(-6.0, 6.0);
    std::uniform_real_distribution<double> ang_dist(-PI, PI);
    for (int trial = 0; trial < 200; ++trial) {
        const Pose start{pos_dist(rng), pos_dist(rng), ang_dist(rng)};
        const Pose goal{pos_dist(rng), pos_dist(rng), ang_dist(rng)};
        const auto path = ComputeShortestReedsShepp(start, goal, RADIUS);
        ASSERT_TRUE(path.valid);
        const auto samples = SampleReedsShepp(path, start, RADIUS, 0.05);
        int flips = 0;
        for (std::size_t i = 1; i < samples.size(); ++i) {
            if (samples[i].forward != samples[i - 1].forward) {
                ++flips;
            }
        }
        EXPECT_EQ(flips, path.numCusps()) << "trial=" << trial;
    }
}

// 测试非正转弯半径被拒绝：半径为零会让归一化除法产生无穷大、静默生成
// 全是 NaN 的路径。预期行为：零与负半径均抛出
TEST(ReedsSheppTest, NonPositiveTurningRadiusThrows) {
    const Pose start{0.0, 0.0, 0.0};
    const Pose goal{1.0, 1.0, 0.5};
    EXPECT_THROW(ComputeShortestReedsShepp(start, goal, 0.0),
                 std::invalid_argument);
    EXPECT_THROW(ComputeShortestReedsShepp(start, goal, -1.0),
                 std::invalid_argument);
}

// 测试无效路径与非正采样步长在采样期被拒绝：无解路径的段数据全是默认值，
// 若静默采样会产生一条「停在起点」的假路径并被上层当成可行解。
// 预期行为：无效路径与非正步长均抛出
TEST(ReedsSheppTest, SamplingRejectsInvalidPathAndStep) {
    const Pose start{0.0, 0.0, 0.0};
    const RsPath invalid_path{};
    EXPECT_THROW(SampleReedsShepp(invalid_path, start, 1.0, 0.1),
                 std::invalid_argument);
    const auto path = ComputeShortestReedsShepp(start, Pose{2.0, 1.0, 0.3}, 2.0);
    ASSERT_TRUE(path.valid);
    EXPECT_THROW(SampleReedsShepp(path, start, 2.0, 0.0),
                 std::invalid_argument);
    EXPECT_THROW(SampleReedsShepp(path, start, 2.0, -0.1),
                 std::invalid_argument);
}
}  // namespace
}  // namespace apa_post_processor
