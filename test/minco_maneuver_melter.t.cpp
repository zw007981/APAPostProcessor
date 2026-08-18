#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include "core/MINCO/minco_maneuver_melter.h"
#include "core/MINCO/bicycle_kinematics_extractor.h"
#include "util/maneuver.h"
#include "util/trajectory.h"

namespace apa_post_processor {
namespace {

// 人造机动段描述：单段多项式构成的 Maneuver（方向/朝向增量/弧长增量/时长）
struct ManeuverSpec {
    Direction direction;
    double delta_theta;
    double delta_s;
    double duration;
};

// 人造场景：MincoTrajectory（K(T) 边界精确满足，段连接处 θ/s 即构造值）与
// 对应的 Maneuver 结构估计。外层 vector 为 Maneuver、内层为其多项式段
// （一个 Maneuver 可跨多段），起点边界取 θ=0/s=0、零速零加速度；终点
// 边界同理静止
struct FabricatedScene {
    MincoTrajectory trajectory;
    std::vector<MincoManeuverEstimate> estimates;
};

FabricatedScene Fabricate(
    const std::vector<std::vector<ManeuverSpec>>& maneuver_specs) {
    FabricatedScene scene;
    MincoBoundaryCondition2d start;
    MincoBoundaryCondition2d end;
    std::vector<Eigen::Vector2d> waypoints;
    std::vector<double> durations;
    double theta = 0.0;
    double arc_length = 0.0;
    int total_segments = 0;
    for (const auto& segment_specs : maneuver_specs) {
        total_segments += static_cast<int>(segment_specs.size());
    }
    int global_index = 0;
    for (const auto& segment_specs : maneuver_specs) {
        MincoManeuverEstimate estimate;
        estimate.direction = segment_specs.front().direction;
        estimate.start_theta = theta;
        estimate.start_arc_length = arc_length;
        for (const auto& spec : segment_specs) {
            theta += spec.delta_theta;
            arc_length += spec.delta_s;
            // 融化检测只消费轨迹真实值与段结构，期望位置不参与本模块，置零占位
            estimate.segments.push_back(
                {{0.0, 0.0}, theta, arc_length, spec.duration});
            durations.push_back(spec.duration);
            ++global_index;
            if (global_index < total_segments) {
                waypoints.emplace_back(theta, arc_length);
            }
        }
        scene.estimates.push_back(estimate);
    }
    end.theta = {theta, 0.0, 0.0};
    end.s = {arc_length, 0.0, 0.0};
    scene.trajectory.setTrajectory(start, end, waypoints, durations);
    return scene;
}

// 便捷重载：每个 Maneuver 恰含 1 个多项式段
FabricatedScene Fabricate(const std::vector<ManeuverSpec>& specs) {
    std::vector<std::vector<ManeuverSpec>> grouped;
    grouped.reserve(specs.size());
    for (const auto& spec : specs) {
        grouped.push_back({spec});
    }
    return Fabricate(grouped);
}

// 与车辆物理参数一致的运动学提取器（状态量解析只依赖提取器本身）
BicycleKinematicsExtractor MakeKinematics() {
    MincoConfig config;
    config.wheelbase = 2.7;
    return BicycleKinematicsExtractor(config);
}

// 测试融化检测的判据量与分类：人为构造 前进1.0m → 后退0.01m（融化）→
// 前进1.0m 的三机动场景，中间换挡段 |Δs|/|Δθ| 均低于阈值，必须被识别为
// 融化废段并剔除，前后两个同向段经 topology_cleaner 同向合并为一段。
TEST(MincoManeuverMelterTest, MeltedSegmentIsPrunedAndMerged) {
    const FabricatedScene scene = Fabricate({
        {Direction::FORWARD, 0.0, 1.0, 2.0},
        {Direction::BACKWARD, 0.01, -0.01, 1.0},
        {Direction::FORWARD, 0.0, 1.0, 2.0},
    });
    const MincoManeuverMelter melter(MincoConfig{});
    const std::vector<MincoManeuverMeltInfo> infos =
        melter.detectMelting(scene.trajectory, scene.estimates);
    ASSERT_EQ(infos.size(), 3U);
    EXPECT_EQ(infos[0].classification, MincoMeltClass::NORMAL);
    EXPECT_EQ(infos[1].classification, MincoMeltClass::MELTED);
    EXPECT_EQ(infos[2].classification, MincoMeltClass::NORMAL);
    // 判据量与构造值精确一致（K(T) 边界条件精确满足）
    EXPECT_NEAR(infos[1].arc_displacement, 0.01, 1e-9);
    EXPECT_NEAR(infos[1].heading_change, 0.01, 1e-9);
    const BicycleKinematicsExtractor kinematics = MakeKinematics();
    const MincoMeltResult result = melter.meltAndPrune(
        scene.trajectory, scene.estimates, {0.0, 0.0}, kinematics);
    EXPECT_TRUE(result.pruned);
    EXPECT_EQ(result.removed_count, 1);
    EXPECT_EQ(result.pivot_count, 0);
    ASSERT_EQ(result.path.numManeuvers(), 1U);
    EXPECT_EQ(result.path.getManeuvers().front().direction, Direction::FORWARD);
    // 修剪输出的采样点必须携带完整状态/控制量（与 NMPC 侧输出惯例一致）
    for (const auto& maneuver : result.path.getManeuvers()) {
        for (const auto& point : maneuver.points) {
            EXPECT_TRUE(point.hasV());
            EXPECT_TRUE(point.hasDelta());
            EXPECT_TRUE(point.hasA());
            EXPECT_TRUE(point.hasDeltaDot());
        }
    }
}

// 测试修剪前后终点约束不被扰动。
// 修剪只是对中间废段的局部剔除与拼接，首尾段及其采样点完全不经改动，
// 因此同一轨迹在"启用融化阈值"与"阈值压到不可能触发"两次运行下的
// 首尾位姿必须逐位一致。
TEST(MincoManeuverMelterTest, TerminalPoseNotDisturbedByPruning) {
    const FabricatedScene scene = Fabricate({
        {Direction::FORWARD, 0.0, 1.0, 2.0},
        {Direction::BACKWARD, 0.01, -0.01, 1.0},
        {Direction::FORWARD, 0.0, 1.0, 2.0},
    });
    const BicycleKinematicsExtractor kinematics = MakeKinematics();
    const MincoManeuverMelter melter(MincoConfig{});
    const MincoMeltResult pruned = melter.meltAndPrune(
        scene.trajectory, scene.estimates, {0.0, 0.0}, kinematics);
    ASSERT_TRUE(pruned.pruned);
    // 对照组：阈值压到任何段都无法触发（最小 |Δs|=0.01 > 1e-9），不修剪
    MincoConfig no_melt_config;
    no_melt_config.melt_arc_threshold = 1e-9;
    const MincoManeuverMelter no_melt_melter(no_melt_config);
    const MincoMeltResult unpruned = no_melt_melter.meltAndPrune(
        scene.trajectory, scene.estimates, {0.0, 0.0}, kinematics);
    ASSERT_FALSE(unpruned.pruned);
    EXPECT_DOUBLE_EQ(pruned.path.back().x, unpruned.path.back().x);
    EXPECT_DOUBLE_EQ(pruned.path.back().y, unpruned.path.back().y);
    EXPECT_DOUBLE_EQ(pruned.path.back().theta, unpruned.path.back().theta);
    EXPECT_DOUBLE_EQ(pruned.path.front().x, unpruned.path.front().x);
    EXPECT_DOUBLE_EQ(pruned.path.front().y, unpruned.path.front().y);
    EXPECT_DOUBLE_EQ(pruned.path.front().theta, unpruned.path.front().theta);
}

// 测试"绝不合并方向相反相邻段"红线。
// 两个方向相反但都非融化的相邻段（|Δs| 均高于阈值），修剪后必须原样
// 保留两段且方向不变，不得被误判合并或剔除。
TEST(MincoManeuverMelterTest, OppositeDirectionNeighborsNeverMerged) {
    const FabricatedScene scene = Fabricate({
        {Direction::FORWARD, 0.0, 1.0, 2.0},
        {Direction::BACKWARD, 0.2, -0.5, 1.5},
    });
    const BicycleKinematicsExtractor kinematics = MakeKinematics();
    const MincoManeuverMelter melter(MincoConfig{});
    const MincoMeltResult result = melter.meltAndPrune(
        scene.trajectory, scene.estimates, {0.0, 0.0}, kinematics);
    EXPECT_FALSE(result.pruned);
    EXPECT_EQ(result.removed_count, 0);
    ASSERT_EQ(result.path.numManeuvers(), 2U);
    EXPECT_EQ(result.path.getManeuvers()[0].direction, Direction::FORWARD);
    EXPECT_EQ(result.path.getManeuvers()[1].direction, Direction::BACKWARD);
}

// 测试 PIVOT（原地掉头式微动）段在融化检测中被正确保留而非误删，且保留
// 后不做任何数据改写。
// 中间段 |Δs|=0.01 低于弧长阈值但 |Δθ|=0.5 rad 超过朝向阈值，必须判定为
// PIVOT：direction 改写为 Direction::PIVOT 标签、单独保留不参与合并，但
// 全部采样点的 x,y,v,a,δ,δ̇ 必须与"阈值压到不可能触发 PIVOT"的对照组逐位
// 一致——PIVOT 只是一个信息性分类标签，不改变任何采样数据（旧实现的
// "位置压平到段首 + v/a 清零"会产生 v≡0 但 θ 变化的状态，与自行车模型
// θ̇=v·tanδ/L_base 自相矛盾，已彻底移除）。
TEST(MincoManeuverMelterTest, PivotSegmentIsPreserved) {
    const FabricatedScene scene = Fabricate({
        {Direction::FORWARD, 0.0, 1.0, 2.0},
        {Direction::BACKWARD, 0.5, -0.01, 1.0},
        {Direction::FORWARD, 0.0, 1.0, 2.0},
    });
    const BicycleKinematicsExtractor kinematics = MakeKinematics();
    const MincoManeuverMelter melter(MincoConfig{});
    const std::vector<MincoManeuverMeltInfo> infos =
        melter.detectMelting(scene.trajectory, scene.estimates);
    ASSERT_EQ(infos.size(), 3U);
    EXPECT_EQ(infos[1].classification, MincoMeltClass::PIVOT);
    EXPECT_NEAR(infos[1].heading_change, 0.5, 1e-9);
    const MincoMeltResult result = melter.meltAndPrune(
        scene.trajectory, scene.estimates, {0.0, 0.0}, kinematics);
    EXPECT_EQ(result.removed_count, 0);
    EXPECT_EQ(result.pivot_count, 1);
    ASSERT_EQ(result.path.numManeuvers(), 3U);
    const Maneuver& pivot = result.path.getManeuvers()[1];
    EXPECT_EQ(pivot.direction, Direction::PIVOT);
    ASSERT_GE(pivot.points.size(), 2U);
    // 朝向变化完整保留。半开区间采样下段末端朝向由下一 Maneuver 首点承载，
    // 本段点序只覆盖到 (N-1)/N 时刻，故旋转量只要求保留绝大部分
    // （>0.4 rad，构造值 0.5）；首点朝向与轨迹段起点（构造值 0.0）精确一致
    EXPECT_NEAR(pivot.points.front().theta, 0.0, 1e-9);
    EXPECT_GT(pivot.points.back().theta - pivot.points.front().theta, 0.4);
    // 对照组：弧长阈值压到任何段都无法触发融化/PIVOT 判定，全部保持
    // NORMAL。两次运行的离散化管线完全相同（同一轨迹、同一采样配置），
    // 故 PIVOT 段的全部采样点状态量必须与对照组逐位一致
    MincoConfig no_melt_config;
    no_melt_config.melt_arc_threshold = 1e-9;
    const MincoManeuverMelter no_melt_melter(no_melt_config);
    const MincoMeltResult unpruned = no_melt_melter.meltAndPrune(
        scene.trajectory, scene.estimates, {0.0, 0.0}, kinematics);
    ASSERT_FALSE(unpruned.pruned);
    ASSERT_EQ(unpruned.path.numManeuvers(), 3U);
    const Maneuver& reference = unpruned.path.getManeuvers()[1];
    EXPECT_EQ(reference.direction, Direction::BACKWARD);
    ASSERT_EQ(pivot.points.size(), reference.points.size());
    for (std::size_t j = 0; j < pivot.points.size(); ++j) {
        const TrajectoryPoint& p = pivot.points[j];
        const TrajectoryPoint& r = reference.points[j];
        EXPECT_DOUBLE_EQ(p.x, r.x);
        EXPECT_DOUBLE_EQ(p.y, r.y);
        EXPECT_DOUBLE_EQ(p.theta, r.theta);
        EXPECT_DOUBLE_EQ(p.getV(), r.getV());
        EXPECT_DOUBLE_EQ(p.getA(), r.getA());
        EXPECT_DOUBLE_EQ(p.getDelta(), r.getDelta());
        EXPECT_DOUBLE_EQ(p.getDeltaDot(), r.getDeltaDot());
    }
}

// 测试修复后的 PIVOT 段满足自行车模型运动学关系 θ̇=v·tanδ/L_base。
// 旧实现把 PIVOT 段位置压平、v/a 清零但保留 θ/δ 变化，输出中存在
// "v≡0 但 θ 持续变化"的自相矛盾状态；修复后段内采样保留连续优化产出的
// 真实折返轨迹，用相邻采样点的有限差分近似 θ̇，与解析 v·tanδ/L 比较
// （梯形平均），二者必须一致。
TEST(MincoManeuverMelterTest, PivotSegmentSatisfiesBicycleKinematics) {
    const FabricatedScene scene = Fabricate({
        {Direction::FORWARD, 0.0, 1.0, 2.0},
        {Direction::BACKWARD, 0.5, -0.01, 1.0},
        {Direction::FORWARD, 0.0, 1.0, 2.0},
    });
    const BicycleKinematicsExtractor kinematics = MakeKinematics();
    // 显式钉住每段采样数：下方 FD 容差按 Δt=T/16 标定，不随默认配置漂移
    MincoConfig config;
    config.samples_per_segment = 16;
    const MincoManeuverMelter melter(config);
    const MincoMeltResult result = melter.meltAndPrune(
        scene.trajectory, scene.estimates, {0.0, 0.0}, kinematics);
    ASSERT_EQ(result.pivot_count, 1);
    ASSERT_EQ(result.path.numManeuvers(), 3U);
    const Maneuver& pivot = result.path.getManeuvers()[1];
    // PIVOT 段为单个多项式段（时长 1.0s）的均匀半开区间采样，相邻点 Δt 恒定
    ASSERT_EQ(pivot.points.size(),
              static_cast<std::size_t>(melter.config().samples_per_segment));
    const double dt = 1.0 / melter.config().samples_per_segment;
    const double wheelbase = 2.7;
    // 有限差分梯形近似的截断误差为 O(Δt²·θ‴)：Δt=1/16 时实测最大偏差约
    // 5e-4 rad/s，再叠加 δ 取值公式 ε_g 分母正则化引入的 ~1e-4 相对偏差，
    // 容差取 5e-3 rad/s 留有约一个量级裕量（修复前矛盾状态的偏差达 0.47
    // rad/s，该容差足以钉住回归）。容差与采样数耦合：FD 截断误差随 Δt²
    // 缩放，若上方 samples_per_segment 变更，需按 O(Δt²) 重新标定实测偏差
    // 与容差
    constexpr double kFdTolerance = 5e-3;
    // 逐区间校验运动学关系；同时确认朝向变化由真实纵向运动产生——
    // 不存在"|θ̇| 显著但 v≈0"的矛盾区间（修复前的核心缺陷）
    bool has_significant_rotation = false;
    for (std::size_t j = 0; j + 1 < pivot.points.size(); ++j) {
        const TrajectoryPoint& p0 = pivot.points[j];
        const TrajectoryPoint& p1 = pivot.points[j + 1];
        const double theta_dot_fd = (p1.theta - p0.theta) / dt;
        const double theta_dot_model = 0.5 *
                                       (p0.getV() * std::tan(p0.getDelta()) +
                                        p1.getV() * std::tan(p1.getDelta())) /
                                       wheelbase;
        EXPECT_NEAR(theta_dot_fd, theta_dot_model, kFdTolerance);
        if (std::abs(theta_dot_fd) > 0.05) {
            has_significant_rotation = true;
            EXPECT_GT(0.5 * (std::abs(p0.getV()) + std::abs(p1.getV())), 1e-3);
        }
    }
    EXPECT_TRUE(has_significant_rotation);
}

// 测试首尾段保护：首/尾 Maneuver 即使满足融化判据量也不被剔除。
// 首段决定车辆当前位姿、末段承载 MINCO 精确收敛的终点等式约束（MINCO.md
// 2.6.3 节"微调必须保持已收敛的终点 x,y,θ 等式约束不被扰动"），二者
// 只允许参与同向合并，不允许被当作废段压平。
TEST(MincoManeuverMelterTest, FirstAndLastManeuverNeverRemoved) {
    const FabricatedScene scene = Fabricate({
        {Direction::FORWARD, 0.01, 0.01, 1.0},
        {Direction::BACKWARD, 0.0, -1.0, 2.0},
        {Direction::FORWARD, -0.01, 0.01, 1.0},
    });
    const BicycleKinematicsExtractor kinematics = MakeKinematics();
    const MincoManeuverMelter melter(MincoConfig{});
    const MincoMeltResult result = melter.meltAndPrune(
        scene.trajectory, scene.estimates, {0.0, 0.0}, kinematics);
    EXPECT_EQ(result.removed_count, 0);
    ASSERT_EQ(result.path.numManeuvers(), 3U);
    EXPECT_EQ(result.path.getManeuvers()[0].direction, Direction::FORWARD);
    EXPECT_EQ(result.path.getManeuvers()[1].direction, Direction::BACKWARD);
    EXPECT_EQ(result.path.getManeuvers()[2].direction, Direction::FORWARD);
}

// 测试首尾段保护同时豁免 PIVOT 重分类：首/末 Maneuver 即使满足 PIVOT
// 判据（|Δs| 低于弧长阈值但 |Δθ| 超过朝向阈值）也必须保持 NORMAL——
// 首段承载车辆当前位姿锚点、末段承载 PHR-ALM 精确收敛的终点 x,y,θ 等式
// 约束（MINCO.md 2.6.3 节"微调必须保持已收敛的终点等式约束不被扰动"），
// 其真实运动方向是输出语义的一部分；改写为 PIVOT 标签会丢弃
// FORWARD/BACKWARD 信息并阻断同向合并。典型真实场景是泊车最后一次原地
// 小角度修正对齐车位：该段必须保留"前进/后退对齐"的真实方向标签，
// 且不得因重分类而脱离与相邻同向段的合并。
TEST(MincoManeuverMelterTest, FirstAndLastManeuverNeverPivot) {
    const FabricatedScene scene = Fabricate({
        {Direction::FORWARD, 0.5, 0.01, 1.0},
        {Direction::BACKWARD, 0.0, -1.0, 2.0},
        {Direction::FORWARD, -0.5, 0.01, 1.0},
    });
    const MincoManeuverMelter melter(MincoConfig{});
    const std::vector<MincoManeuverMeltInfo> infos =
        melter.detectMelting(scene.trajectory, scene.estimates);
    ASSERT_EQ(infos.size(), 3U);
    // 首/末 Maneuver 判据量均满足 PIVOT 条件，但分类必须保持 NORMAL
    EXPECT_GT(infos[0].heading_change, 0.1);
    EXPECT_LT(infos[0].arc_displacement, 0.05);
    EXPECT_EQ(infos[0].classification, MincoMeltClass::NORMAL);
    EXPECT_GT(infos[2].heading_change, 0.1);
    EXPECT_LT(infos[2].arc_displacement, 0.05);
    EXPECT_EQ(infos[2].classification, MincoMeltClass::NORMAL);
    const BicycleKinematicsExtractor kinematics = MakeKinematics();
    const MincoMeltResult result = melter.meltAndPrune(
        scene.trajectory, scene.estimates, {0.0, 0.0}, kinematics);
    EXPECT_EQ(result.removed_count, 0);
    EXPECT_EQ(result.pivot_count, 0);
    ASSERT_EQ(result.path.numManeuvers(), 3U);
    // 首/末段方向不得被改写为 PIVOT，几何与朝向变化必须原样保留
    const std::vector<Maneuver>& maneuvers = result.path.getManeuvers();
    EXPECT_EQ(maneuvers[0].direction, Direction::FORWARD);
    EXPECT_EQ(maneuvers[2].direction, Direction::FORWARD);
    EXPECT_GT(
        maneuvers[0].points.back().theta - maneuvers[0].points.front().theta,
        0.4);
    // 末段含全局终点补采样，朝向变化完整覆盖构造值 -0.5
    EXPECT_NEAR(
        maneuvers[2].points.back().theta - maneuvers[2].points.front().theta,
        -0.5, 1e-9);
    // 对照组：阈值压到任何段都无法触发，末端位姿必须与未修剪运行逐位一致
    // （钉住终点不扰动不变量：无论分类结果如何变化，全局终点采样不受分类
    // 与重建过程影响）
    MincoConfig no_melt_config;
    no_melt_config.melt_arc_threshold = 1e-9;
    const MincoManeuverMelter no_melt_melter(no_melt_config);
    const MincoMeltResult unpruned = no_melt_melter.meltAndPrune(
        scene.trajectory, scene.estimates, {0.0, 0.0}, kinematics);
    ASSERT_FALSE(unpruned.pruned);
    EXPECT_DOUBLE_EQ(result.path.back().x, unpruned.path.back().x);
    EXPECT_DOUBLE_EQ(result.path.back().y, unpruned.path.back().y);
    EXPECT_DOUBLE_EQ(result.path.back().theta, unpruned.path.back().theta);
}

// 测试单 Maneuver 跨多个多项式段时判据量取首末段端点差值。
// 首个 Maneuver 由 2 段构成（Δs=0.4+0.6、Δθ=0.05+0.05），若判据量只取
// 第一段端点会错误得到 0.4/0.05；正确实现必须跨段累计为 1.0/0.1。
TEST(MincoManeuverMelterTest, MultiSegmentManeuverMeasuredAcrossSegments) {
    const FabricatedScene scene = Fabricate({
        {{Direction::FORWARD, 0.05, 0.4, 1.0},
         {Direction::FORWARD, 0.05, 0.6, 1.0}},
        {{Direction::BACKWARD, 0.01, -0.01, 1.0}},
        {{Direction::FORWARD, 0.0, 1.0, 2.0}},
    });
    const MincoManeuverMelter melter(MincoConfig{});
    const std::vector<MincoManeuverMeltInfo> infos =
        melter.detectMelting(scene.trajectory, scene.estimates);
    ASSERT_EQ(infos.size(), 3U);
    EXPECT_NEAR(infos[0].arc_displacement, 1.0, 1e-9);
    EXPECT_NEAR(infos[0].heading_change, 0.1, 1e-9);
    EXPECT_EQ(infos[0].classification, MincoMeltClass::NORMAL);
    EXPECT_EQ(infos[1].classification, MincoMeltClass::MELTED);
    const BicycleKinematicsExtractor kinematics = MakeKinematics();
    const MincoMeltResult result = melter.meltAndPrune(
        scene.trajectory, scene.estimates, {0.0, 0.0}, kinematics);
    EXPECT_EQ(result.removed_count, 1);
    ASSERT_EQ(result.path.numManeuvers(), 1U);
    EXPECT_EQ(result.path.getManeuvers().front().direction, Direction::FORWARD);
}

// 测试非法输入的异常反馈。
// 初值估计与轨迹的段数结构必须一致（检测按估计的段结构索引轨迹段），
// 空估计、空 Maneuver、段数不一致、非有限起点都必须在装配期以标准异常
// 明确拒绝。
TEST(MincoManeuverMelterTest, InvalidInputsThrow) {
    const FabricatedScene scene = Fabricate({
        {Direction::FORWARD, 0.0, 1.0, 2.0},
        {Direction::BACKWARD, 0.01, -0.01, 1.0},
        {Direction::FORWARD, 0.0, 1.0, 2.0},
    });
    const BicycleKinematicsExtractor kinematics = MakeKinematics();
    const MincoManeuverMelter melter(MincoConfig{});
    // 空估计
    EXPECT_THROW(
        melter.meltAndPrune(scene.trajectory, {}, {0.0, 0.0}, kinematics),
        std::invalid_argument);
    // 空 segments 的 Maneuver
    EXPECT_THROW(melter.meltAndPrune(scene.trajectory, {MincoManeuverEstimate{}},
                                     {0.0, 0.0}, kinematics),
                 std::invalid_argument);
    // 段数不一致：估计 2 段 vs 轨迹 3 段
    std::vector<MincoManeuverEstimate> mismatched = scene.estimates;
    mismatched.pop_back();
    EXPECT_THROW(melter.meltAndPrune(scene.trajectory, mismatched, {0.0, 0.0},
                                     kinematics),
                 std::invalid_argument);
    // 非有限起点世界坐标
    EXPECT_THROW(
        melter.meltAndPrune(scene.trajectory, scene.estimates,
                            {std::numeric_limits<double>::quiet_NaN(), 0.0},
                            kinematics),
        std::invalid_argument);
    // detectMelting 共享同一套结构校验
    EXPECT_THROW(melter.detectMelting(scene.trajectory, {}),
                 std::invalid_argument);
}

// 测试非法配置的构造期校验。
// 融化阈值决定分类行为，非正阈值与非法采样数必须在构造期显式拒绝。
TEST(MincoManeuverMelterTest, InvalidConfigThrows) {
    EXPECT_NO_THROW(const MincoManeuverMelter valid_melter(MincoConfig{}));
    MincoConfig config;
    config.melt_arc_threshold = 0.0;
    EXPECT_THROW(const MincoManeuverMelter m(config), std::invalid_argument);
    config = {};
    config.melt_heading_threshold = -0.1;
    EXPECT_THROW(const MincoManeuverMelter m(config), std::invalid_argument);
    config = {};
    config.samples_per_segment = 1;
    EXPECT_THROW(const MincoManeuverMelter m(config), std::invalid_argument);
}

// 测试 config() 只读访问器返回构造时的配置值。
TEST(MincoManeuverMelterTest, ConfigAccessorReturnsConstructionValues) {
    MincoConfig config;
    config.melt_arc_threshold = 0.03;
    config.samples_per_segment = 24;
    const MincoManeuverMelter melter(config);
    EXPECT_DOUBLE_EQ(melter.config().melt_arc_threshold, 0.03);
    EXPECT_EQ(melter.config().samples_per_segment, 24);
    EXPECT_DOUBLE_EQ(melter.config().melt_heading_threshold, 0.1);
}

// ===== ResegmentByVelocityDirection 测试 =====
// 辅助：构造带速度的路径点（θ 置零）
TrajectoryPoint MakeVelocityPoint(double x, double y, double v) {
    TrajectoryPoint p(x, y, 0.0);
    p.setV(v);
    return p;
}

// 辅助：以管线停驻阈值（0.05）调用重切分
void ResegmentPath(Path* path) {
    ResegmentByVelocityDirection(path, 0.05);
}

// 单一方向、无段内反转的机动段：重切后结构原样保留（段数/方向/点数不变）。
// 触发场景：正常 FORWARD/BACKWARD 段不应被误拆。
TEST(ResegmentTest, SingleDirectionManeuverUnchanged) {
    Path path;
    path.getManeuvers().emplace_back(
        std::vector<TrajectoryPoint>{MakeVelocityPoint(0.0, 0.0, -0.1),
                                     MakeVelocityPoint(0.0, -1.0, -0.3),
                                     MakeVelocityPoint(0.0, -2.0, -0.3)},
        Direction::BACKWARD);
    ResegmentPath(&path);
    ASSERT_EQ(path.numManeuvers(), 1u);
    EXPECT_EQ(path.getManeuvers()[0].direction, Direction::BACKWARD);
    EXPECT_EQ(path.getManeuvers()[0].points.size(), 3u);
}

// 段内一次方向反转（退-停-进）：切分为方向相反的两段，方向按实际 v 符号
// 标注（不是估计标签 BACKWARD），相邻段共享边界点（停驻点）。
// 触发场景：求解器在单段内部表达退-进-退微调时，输出必须如实分段。
TEST(ResegmentTest, InternalReversalSplitsIntoTwoManeuvers) {
    Path path;
    path.getManeuvers().emplace_back(
        std::vector<TrajectoryPoint>{MakeVelocityPoint(0.0, 0.0, -0.1),
                                     MakeVelocityPoint(0.0, -1.0, -0.3),
                                     MakeVelocityPoint(0.0, -1.2, 0.0),
                                     MakeVelocityPoint(0.0, -0.8, 0.2),
                                     MakeVelocityPoint(0.0, -0.4, 0.2)},
        Direction::BACKWARD);
    ResegmentPath(&path);
    ASSERT_EQ(path.numManeuvers(), 2u);
    EXPECT_EQ(path.getManeuvers()[0].direction, Direction::BACKWARD);
    EXPECT_EQ(path.getManeuvers()[1].direction, Direction::FORWARD);
    const auto& sub0 = path.getManeuvers()[0].points;
    const auto& sub1 = path.getManeuvers()[1].points;
    // 共享边界点：后段首点 = 前段末点（Path::addPoint 语义）
    EXPECT_DOUBLE_EQ(sub0.back().x, sub1.front().x);
    EXPECT_DOUBLE_EQ(sub0.back().y, sub1.front().y);
    // 前段含停驻点（退-停），后段为前进游程
    EXPECT_EQ(sub0.size(), 3u);
    EXPECT_EQ(sub1.size(), 3u);
}

// 段内两次反转（退-停-进-停-退）：切分为三段，方向依次 BACKWARD/FORWARD/
// BACKWARD。触发场景：揉库式"退-进-退"微调必须拆成三段独立 maneuver。
TEST(ResegmentTest, DoubleReversalSplitsIntoThreeManeuvers) {
    Path path;
    path.getManeuvers().emplace_back(
        std::vector<TrajectoryPoint>{MakeVelocityPoint(0.0, 0.0, -0.1),
                                     MakeVelocityPoint(0.0, -1.0, -0.3),
                                     MakeVelocityPoint(0.0, -1.2, 0.0),
                                     MakeVelocityPoint(0.0, -0.8, 0.2),
                                     MakeVelocityPoint(0.0, -0.4, 0.2),
                                     MakeVelocityPoint(0.0, -0.4, 0.0),
                                     MakeVelocityPoint(0.0, -1.0, -0.3),
                                     MakeVelocityPoint(0.0, -1.5, -0.3)},
        Direction::BACKWARD);
    ResegmentPath(&path);
    ASSERT_EQ(path.numManeuvers(), 3u);
    EXPECT_EQ(path.getManeuvers()[0].direction, Direction::BACKWARD);
    EXPECT_EQ(path.getManeuvers()[1].direction, Direction::FORWARD);
    EXPECT_EQ(path.getManeuvers()[2].direction, Direction::BACKWARD);
}

// 无 v 数据/全部停驻的机动段：无法判方向，原样保留（段数/点数不变）。
// 触发场景：无速度信息的路径不应被误拆或丢失。
TEST(ResegmentTest, ManeuverWithoutVelocityUnchanged) {
    Path path;
    path.getManeuvers().emplace_back(
        std::vector<TrajectoryPoint>{TrajectoryPoint(0.0, 0.0, 0.0),
                                     TrajectoryPoint(0.0, -1.0, 0.0)},
        Direction::UNKNOWN);
    ResegmentPath(&path);
    ASSERT_EQ(path.numManeuvers(), 1u);
    EXPECT_EQ(path.getManeuvers()[0].points.size(), 2u);
}

// 前导停驻点归属首个游程：段首停驻点不单独成段，并入第一个方向游程。
// 触发场景：换挡后起步的停驻点必须挂在后继运动段上。
TEST(ResegmentTest, LeadingStopAttachesToFirstRun) {
    Path path;
    path.getManeuvers().emplace_back(
        std::vector<TrajectoryPoint>{MakeVelocityPoint(0.0, 0.0, 0.0),
                                     MakeVelocityPoint(0.0, -0.2, -0.1),
                                     MakeVelocityPoint(0.0, -1.0, -0.3),
                                     MakeVelocityPoint(0.0, -1.2, 0.0),
                                     MakeVelocityPoint(0.0, -0.8, 0.2)},
        Direction::BACKWARD);
    ResegmentPath(&path);
    ASSERT_EQ(path.numManeuvers(), 2u);
    // 首段含前导停驻点 + 后退游程（共 4 点），末段为共享边界点 + 前进游程（2 点）
    EXPECT_EQ(path.getManeuvers()[0].points.size(), 4u);
    EXPECT_EQ(path.getManeuvers()[1].points.size(), 2u);
}
// 回归测试（data1 结构）：求解器可在单个 Maneuver 内部产生 v 方向反转
// （θ-s 优化无 s_dot 符号约束，揉库段内"退-进-退"），此前输出 Path 会把
// 整段标为单一方向，导致 Path 段数与物理方向游程（countDirectionRuns）
// 不一致、段内反转处几何 κ 畸高。本测试构造单 estimate 内部弧长非单调
// （先退后进再退）的场景，走融化+重切分全链后断言两条不变量：① Path 段数
// == 物理方向游程数；② 每段方向标签与其内部点（除共享边界首点）v 符号一致。
TEST(MincoManeuverMelterTest, OutputManeuversMatchPhysicalDirectionRuns) {
    // est0: FORWARD 单段；est1: BACKWARD 三段，弧长增量 -2.0 / +1.2 / -3.0
    // （段内 s 非单调 = 内部折返，模拟 data1 揉库段的"退-进-退"微调）
    const FabricatedScene scene = Fabricate({
        {{Direction::FORWARD, 0.0, 1.0, 2.0}},
        {{Direction::BACKWARD, 0.2, -2.0, 2.0},
         {Direction::BACKWARD, -0.2, 1.2, 2.0},
         {Direction::BACKWARD, 0.0, -3.0, 2.0}},
    });
    const BicycleKinematicsExtractor kinematics = MakeKinematics();
    const MincoManeuverMelter melter(MincoConfig{});
    const MincoMeltResult result = melter.meltAndPrune(
        scene.trajectory, scene.estimates, {0.0, 0.0}, kinematics);
    ASSERT_FALSE(result.path.empty());
    Path path = result.path;
    ResegmentByVelocityDirection(&path, MincoConfig{}.v_epsilon);
    // 不变量①：Path 段数 == 物理方向游程数（countDirectionRuns 口径）
    Trajectory flat;
    for (const auto& m : path.getManeuvers()) {
        for (const auto& p : m.points) {
            flat.push_back(p);
        }
    }
    EXPECT_EQ(path.numManeuvers(),
              static_cast<std::size_t>(flat.countDirectionRuns()));
    // 不变量②：每段方向标签与其内部点 v 符号一致（跳过共享边界首点）
    for (const auto& m : path.getManeuvers()) {
        for (std::size_t i = 1; i < m.points.size(); ++i) {
            const auto& p = m.points[i];
            if (!p.hasV() || std::abs(p.getV()) < MincoConfig{}.v_epsilon) {
                continue;  // 停驻点不判方向
            }
            const Direction expected =
                p.getV() > 0 ? Direction::FORWARD : Direction::BACKWARD;
            EXPECT_EQ(m.direction, expected);
        }
    }
}

}  // namespace
}  // namespace apa_post_processor
