#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <vector>

#include "core/iLQR/apa_ilqr_solver.h"
#include "core/iLQR/ilqr_cost.h"
#include "core/iLQR/ilqr_post_stage.h"
#include "core/iLQR/ilqr_reference_builder.h"
#include "core/iLQR/esdf_constraint.h"
#include "spatial/esdf_map.h"
#include "spatial/grid_map.h"
#include "util/constants.h"
#include "util/path.h"
#include "util/topology_cleaner.h"
#include "vehicle/vehicle_footprint_model.h"
#include "vehicle/vehicle_params.h"

namespace apa_post_processor {
namespace {

constexpr double kWheelbase = 2.7;

// 测试用车辆参数：与既有 iLQR 组件单测一致的五参数构造
VehicleParams MakeVehicleParams() {
    return VehicleParams(4.3, 1.8, 2.7, 0.6, 0.8);
}

// 从当前路径末端沿 x 轴追加直线路径点（步长 0.05 m，与 A* 点距一致）
void AppendXLine(Path* path, double x_from, double x_to, double theta) {
    const int count =
        static_cast<int>(std::round(std::abs(x_to - x_from) / 0.05));
    for (int i = 1; i <= count; ++i) {
        const double x = x_from + (x_to - x_from) * i / count;
        path->addPoint({x, 0.0, theta});
    }
}

// 沿 x 轴构造多点路径：segments 为逐段目标 x（θ 恒 0），方向由 x 增减自然形成
Path BuildXPolyline(const std::vector<double>& segments) {
    Path path;
    path.addPoint({segments.front(), 0.0, 0.0});
    for (std::size_t i = 1; i < segments.size(); ++i) {
        AppendXLine(&path, segments[i - 1], segments[i], 0.0);
    }
    path.finalize();
    return path;
}

// 由 Path 构建前端数据（默认构建配置：0.05 m 重采样、dt=0.1 s、打靶间隔 25 步）
iLQRReference BuildReference(const Path& path) {
    return iLQRReferenceBuilder(iLQRReferenceBuilderConfig{}, MakeVehicleParams())
        .build(path);
}

// 合成小尺度场景的求解配置：μ_min 降到 1.0 防小代价量级下 μ⁰ 标定被下限
// clip 到过强罚权重（与既有编排器单测同一约定，生产默认不受影响）
ApaILQRSolverConfig MakeSyntheticConfig() {
    ApaILQRSolverConfig config;
    config.outer.mu_min = 1.0;
    return config;
}

// 合成状态剖面的一条记录：x 位置、纵向速度、前轮转角、朝向（y=0，
// a/ω 默认 0，需要注入差分反解的用例可显式指定）
struct ProfileEntry {
    double x;
    double v;
    double delta;
    double theta{0.0};
    double a{0.0};
    double omega{0.0};
};

// 向剖面末尾追加 count 个均匀点：x 按 dx 步进，v/δ 恒定（θ/a/ω 恒 0）
void AppendRun(std::vector<ProfileEntry>* profile, double dx, int count,
               double v, double delta) {
    for (int i = 0; i < count; ++i) {
        const double x = profile->empty() ? 0.0 : profile->back().x + dx;
        profile->push_back({x, v, delta, 0.0, 0.0, 0.0});
    }
}

// 由显式剖面构造合成状态轨迹
iLQRAlignedVec<iLQRState> MakeStates(const std::vector<ProfileEntry>& profile) {
    iLQRAlignedVec<iLQRState> states;
    states.reserve(profile.size());
    for (const auto& entry : profile) {
        iLQRState state = iLQRState::Zero();
        state(ILQR_IDX_X) = entry.x;
        state(ILQR_IDX_V) = entry.v;
        state(ILQR_IDX_DELTA) = entry.delta;
        state(ILQR_IDX_THETA) = entry.theta;
        state(ILQR_IDX_A) = entry.a;
        state(ILQR_IDX_OMEGA) = entry.omega;
        states.push_back(state);
    }
    return states;
}

// 后处理器标准装配：参考构建器 + 动力学/求值层 + 求解编排 + 后处理器
struct PostStageFixture {
    PostStageFixture()
        : dynamics(kWheelbase),
          cost_evaluator(solver_config.cost, nullptr),
          solver(solver_config, &dynamics, &cost_evaluator),
          reference_builder(iLQRReferenceBuilderConfig{}, MakeVehicleParams()),
          post_stage(post_config, &reference_builder, &solver,
                     MakeVehicleParams()) {}
    ApaILQRSolverConfig solver_config = MakeSyntheticConfig();
    iLQRPostStageConfig post_config;
    BicycleDynamics dynamics;
    iLQRCostEvaluator cost_evaluator;
    ApaILQRSolver solver;
    iLQRReferenceBuilder reference_builder;
    iLQRPostStage post_stage;
};

// 白盒测试访问器：暴露 protected 的两层校验入口，供"校验容差按
// 预期拦截/放行"的边界测试直接驱动（无需为注入异常解重跑完整求解）
class ValidateOutputAccessor : public iLQRPostStage {
   public:
    using iLQRPostStage::iLQRPostStage;
    using iLQRPostStage::validateOutput;
};

// 白盒测试访问器：暴露 protected 的阶段一驻留窗计划构建，供多接缝
// 窗口裁剪语义的直接驱动（无需先构造完整后处理输入）
class SeamPlanAccessor : public iLQRPostStage {
   public:
    using iLQRPostStage::buildStageOneSeamPlans;
    using iLQRPostStage::iLQRPostStage;
};

// ==================== T_resteer 双积分 bang-bang 公式 ====================

// 三角剖面分支（Δδ 小、ω 不饱和）：取 Δδ=0.09（< ω²/η=0.25），
// 解析解 T=2√(Δδ/η)=2·√0.09=0.6 s，与公式逐位对拍
TEST(iLQRResteerTimeTest, TriangleBranchMatchesAnalytic) {
    const double t = iLQRPostStage::ComputeResteerTime(0.09, 0.5, 1.0);
    EXPECT_NEAR(t, 0.6, 1e-12);
}

// 梯形剖面分支（Δδ 大、ω 饱和）：取 Δδ=1.0（> 0.25），
// 解析解 T=Δδ/ω+ω/η=1.0/0.5+0.5/1.0=2.5 s，与公式逐位对拍
TEST(iLQRResteerTimeTest, TrapezoidBranchMatchesAnalytic) {
    const double t = iLQRPostStage::ComputeResteerTime(1.0, 0.5, 1.0);
    EXPECT_NEAR(t, 2.5, 1e-12);
}

// 分支切换点连续性：Δδ*=ω²/η=0.25 处三角公式 2√0.25=1.0 与梯形公式
// 0.25/0.5+0.5=1.0 相等；切换点两侧无限趋近时函数值必须无缝衔接
TEST(iLQRResteerTimeTest, BranchSwitchIsContinuous) {
    const double switch_point = 0.5 * 0.5 / 1.0;
    const double below =
        iLQRPostStage::ComputeResteerTime(switch_point - 1e-12, 0.5, 1.0);
    const double above =
        iLQRPostStage::ComputeResteerTime(switch_point + 1e-12, 0.5, 1.0);
    EXPECT_NEAR(below, 1.0, 1e-6);
    EXPECT_NEAR(below, above, 1e-6);
    // 单调性：Δδ 越大所需时间越长（两分支各自与跨分支均成立）
    EXPECT_LT(iLQRPostStage::ComputeResteerTime(0.04, 0.5, 1.0),
              iLQRPostStage::ComputeResteerTime(0.16, 0.5, 1.0));
    EXPECT_LT(iLQRPostStage::ComputeResteerTime(0.5, 0.5, 1.0),
              iLQRPostStage::ComputeResteerTime(0.8, 0.5, 1.0));
    // 非法参数拒绝：Δδ<0、执行器限值非正
    EXPECT_THROW(iLQRPostStage::ComputeResteerTime(-0.1, 0.5, 1.0),
                 std::invalid_argument);
    EXPECT_THROW(iLQRPostStage::ComputeResteerTime(0.1, 0.0, 1.0),
                 std::invalid_argument);
    EXPECT_THROW(iLQRPostStage::ComputeResteerTime(0.1, 0.5, -1.0),
                 std::invalid_argument);
}

// ==================== 带滞回符号游程分析 ====================

// 滞回过滤：v 在 ±ε_v（0.02 m/s）内抖动（融化残留的速度涟漪）不产生伪
// cusp——全程 |v|<ε_v 的抖动段不承诺新符号，整条轨迹只恢复出 1 个游程
TEST(iLQRSignRunTest, RippleWithinHysteresisProducesNoCusp) {
    PostStageFixture fixture;
    std::vector<ProfileEntry> profile;
    AppendRun(&profile, 0.05, 21, 0.5, 0.0);
    // 中段插入 ±0.01 抖动（低于滞回阈值，不得切断游程）
    profile.push_back({1.02, 0.01, 0.0});
    profile.push_back({1.03, -0.01, 0.0});
    profile.push_back({1.04, 0.005, 0.0});
    AppendRun(&profile, 0.05, 10, 0.5, 0.0);
    const auto runs = fixture.post_stage.analyzeSignRuns(MakeStates(profile));
    ASSERT_EQ(runs.size(), 1);
    EXPECT_EQ(runs[0].sign, 1);
    EXPECT_EQ(runs[0].begin_index, 0);
    EXPECT_EQ(runs[0].end_index, profile.size() - 1);
}

// 真实换向识别：+0.5 m/s 前进 10 步后 −0.5 m/s 倒退，应恢复出符号相反的
// 两个游程；游程边界共享（前段终点索引 = 后段起点索引），位移量测正确
TEST(iLQRSignRunTest, TrueReversalDetectedWithSharedBoundary) {
    PostStageFixture fixture;
    std::vector<ProfileEntry> profile;
    AppendRun(&profile, 0.05, 11, 0.5, 0.0);    // x: 0 → 0.5
    AppendRun(&profile, -0.05, 10, -0.5, 0.0);  // x: 0.45 → 0.0
    const auto runs = fixture.post_stage.analyzeSignRuns(MakeStates(profile));
    ASSERT_EQ(runs.size(), 2);
    EXPECT_EQ(runs[0].sign, 1);
    EXPECT_EQ(runs[1].sign, -1);
    // 边界共享约定：前段终点即后段起点（首次承诺反号的样本）
    EXPECT_EQ(runs[0].end_index, 11);
    EXPECT_EQ(runs[1].begin_index, 11);
    EXPECT_EQ(runs[1].end_index, profile.size() - 1);
    // 位移量测：前段 0→0.5→0.45（0.55 m），后段 0.45→0（0.45 m）
    EXPECT_NEAR(runs[0].delta_s, 0.55, 1e-9);
    EXPECT_NEAR(runs[1].delta_s, 0.45, 1e-9);
}

// 全程未决样本：所有 |v|<ε_v（车辆近乎静止），无法承诺任何符号——
// 恢复为单个零符号游程，不产生任何伪换挡
TEST(iLQRSignRunTest, UndecidedSamplesNeverCommit) {
    PostStageFixture fixture;
    std::vector<ProfileEntry> profile;
    AppendRun(&profile, 0.01, 10, 0.005, 0.0);
    const auto runs = fixture.post_stage.analyzeSignRuns(MakeStates(profile));
    ASSERT_EQ(runs.size(), 1);
    EXPECT_EQ(runs[0].sign, 0);
}

// ==================== 拓扑修剪红线 ====================

// 红线：绝不合并方向相反的相邻段——前进/倒退/前进三个足量 maneuver 经
// 修剪后全部保留且不发生任何合并（同向合并仅限同方向）
TEST(iLQRTopologyPruneTest, OppositeDirectionNeighborsNotMerged) {
    PostStageFixture fixture;
    std::vector<ProfileEntry> profile;
    AppendRun(&profile, 0.05, 21, 0.5, 0.0);
    AppendRun(&profile, -0.05, 20, -0.5, 0.0);
    AppendRun(&profile, 0.05, 20, 0.5, 0.0);
    auto maneuvers = fixture.post_stage.buildManeuvers(
        MakeStates(profile),
        fixture.post_stage.analyzeSignRuns(MakeStates(profile)));
    ASSERT_EQ(maneuvers.size(), 3);
    EXPECT_TRUE(fixture.post_stage.pruneManeuvers(&maneuvers));
    const Path pruned = ReconstructPath(maneuvers);
    ASSERT_EQ(pruned.numManeuvers(), 3);
    EXPECT_EQ(pruned.getManeuvers()[0].direction, Direction::FORWARD);
    EXPECT_EQ(pruned.getManeuvers()[1].direction, Direction::BACKWARD);
    EXPECT_EQ(pruned.getManeuvers()[2].direction, Direction::FORWARD);
}

// 同向拼接：前进（足量）→ 倒退微游程（|Δs|<0.05 且 |Δδ| 小）→ 前进（足量），
// 中间微游程被压平剔除，前后两个同向 maneuver 直接拼接为一段
TEST(iLQRTopologyPruneTest, SameDirectionNeighborsMergedAfterMelt) {
    PostStageFixture fixture;
    std::vector<ProfileEntry> profile;
    AppendRun(&profile, 0.05, 41, 0.5, 0.0);  // 前进 2 m
    // 倒退微游程：位移 0.025 m（<0.05）、δ 无变化（非 PIVOT）
    profile.push_back({2.00, -0.03, 0.0});
    profile.push_back({1.99, -0.03, 0.0});
    profile.push_back({2.005, 0.5, 0.0});
    AppendRun(&profile, 0.05, 40, 0.5, 0.0);  // 前进继续
    const auto states = MakeStates(profile);
    auto maneuvers = fixture.post_stage.buildManeuvers(
        states, fixture.post_stage.analyzeSignRuns(states));
    ASSERT_EQ(maneuvers.size(), 3);
    EXPECT_TRUE(fixture.post_stage.pruneManeuvers(&maneuvers));
    const Path pruned = ReconstructPath(maneuvers);
    ASSERT_EQ(pruned.numManeuvers(), 1);
    EXPECT_EQ(pruned.getManeuvers()[0].direction, Direction::FORWARD);
}

// 红线：首/末 maneuver 受保护——首段与末段均为微游程（|Δs|<0.05），按
// 判据本应剔除，但首段承载起点状态、末段承载终点语义，均不得剔除与
// 重分类，修剪后三段全部保留
TEST(iLQRTopologyPruneTest, FirstAndLastManeuverProtected) {
    PostStageFixture fixture;
    std::vector<ProfileEntry> profile;
    profile.push_back({0.0, 0.03, 0.0});  // 首段微游程（前进 0.015 m）
    profile.push_back({0.01, 0.03, 0.0});
    profile.push_back({0.005, -0.5, 0.0});
    AppendRun(&profile, -0.05, 30, -0.5, 0.0);  // 中段倒退（足量）
    profile.push_back({-1.44, 0.03, 0.0});  // 末段微游程（前进 0.01 m）
    profile.push_back({-1.43, 0.03, 0.0});
    const auto states = MakeStates(profile);
    auto maneuvers = fixture.post_stage.buildManeuvers(
        states, fixture.post_stage.analyzeSignRuns(states));
    ASSERT_EQ(maneuvers.size(), 3);
    // 判据确认：首末段弧长均低于剔除阈值（验证保护语义而非判据失效）
    EXPECT_LT(maneuvers.front().length(),
              fixture.post_config.prune.min_arc_length);
    EXPECT_LT(maneuvers.back().length(),
              fixture.post_config.prune.min_arc_length);
    EXPECT_TRUE(fixture.post_stage.pruneManeuvers(&maneuvers));
    const Path pruned = ReconstructPath(maneuvers);
    ASSERT_EQ(pruned.numManeuvers(), 3);
    EXPECT_EQ(pruned.getManeuvers()[0].direction, Direction::FORWARD);
    EXPECT_EQ(pruned.getManeuvers()[1].direction, Direction::BACKWARD);
    EXPECT_EQ(pruned.getManeuvers()[2].direction, Direction::FORWARD);
}

// 红线：PIVOT 触发失败语义（Δθ 判据）——中间微游程 |Δs|<0.05 且朝向变化
// 0.3 rad（>0.1 rad 阈值）被判为原地掉头式微动；注意本例 Δδ=0——Δδ 不再
// 参与判据（换挡点两侧的方向盘大幅摆动与原地掉头无关），只有朝向变化
// 才携带原地掉头语义。动力学一致解不可能产生此类微动，修剪必须上报
// 失败（返回 false）而非带病输出
TEST(iLQRTopologyPruneTest, PivotTriggersFailureSemantics) {
    PostStageFixture fixture;
    std::vector<ProfileEntry> profile;
    AppendRun(&profile, 0.05, 41, 0.5, 0.0);
    // 微游程：位移 0.01 m（<0.05），朝向从 0 跳到 0.3 rad（δ 恒 0）
    profile.push_back({2.00, -0.03, 0.0, 0.0});
    profile.push_back({1.99, -0.03, 0.0, 0.3});
    profile.push_back({2.005, 0.5, 0.0, 0.3});
    AppendRun(&profile, 0.05, 40, 0.5, 0.0);
    const auto states = MakeStates(profile);
    auto maneuvers = fixture.post_stage.buildManeuvers(
        states, fixture.post_stage.analyzeSignRuns(states));
    ASSERT_EQ(maneuvers.size(), 3);
    EXPECT_FALSE(fixture.post_stage.pruneManeuvers(&maneuvers));
}

// 判据语义钉住：微游程首末前轮转角差 0.3 rad（超过旧 Δδ 判据 0.1）但朝向
// 不变（Δθ=0）——换挡点两侧的方向盘摆动不携带原地掉头语义，不得误判
// PIVOT，按融化残余剔除并拼接前后同向段（修剪返回 true）
TEST(iLQRTopologyPruneTest, LargeSteerChangeWithoutHeadingChangeIsNotPivot) {
    PostStageFixture fixture;
    std::vector<ProfileEntry> profile;
    AppendRun(&profile, 0.05, 41, 0.5, 0.0);
    // 微游程：位移 0.01 m（<0.05）、δ 从 0.3 跳到 0.0（|Δδ|=0.3）、θ 恒 0
    profile.push_back({2.00, -0.03, 0.3, 0.0});
    profile.push_back({1.99, -0.03, 0.0, 0.0});
    profile.push_back({2.005, 0.5, 0.0, 0.0});
    AppendRun(&profile, 0.05, 40, 0.5, 0.0);
    const auto states = MakeStates(profile);
    auto maneuvers = fixture.post_stage.buildManeuvers(
        states, fixture.post_stage.analyzeSignRuns(states));
    ASSERT_EQ(maneuvers.size(), 3);
    EXPECT_TRUE(fixture.post_stage.pruneManeuvers(&maneuvers));
    // 微游程被剔除、前后同向段拼接为一段
    const Path pruned = ReconstructPath(maneuvers);
    ASSERT_EQ(pruned.numManeuvers(), 1);
    EXPECT_EQ(pruned.getManeuvers()[0].direction, Direction::FORWARD);
}

// 不改写采样数据红线：修剪只打方向标签（UNKNOWN 剔除标记），保留段与
// 剔除段的逐点数据（x/y/θ/v/δ）在修剪前后必须逐位一致——压平位置/清零
// 速度会产生 v≡0 但 θ 变化的自相矛盾状态（ALM 侧已废弃的做法）
TEST(iLQRTopologyPruneTest, PruneDoesNotRewritePoints) {
    PostStageFixture fixture;
    std::vector<ProfileEntry> profile;
    AppendRun(&profile, 0.05, 41, 0.5, 0.0);
    profile.push_back({2.00, -0.03, 0.0, 0.0});
    profile.push_back({1.99, -0.03, 0.0, 0.0});
    profile.push_back({2.005, 0.5, 0.0, 0.0});
    AppendRun(&profile, 0.05, 40, 0.5, 0.0);
    const auto states = MakeStates(profile);
    auto maneuvers = fixture.post_stage.buildManeuvers(
        states, fixture.post_stage.analyzeSignRuns(states));
    ASSERT_EQ(maneuvers.size(), 3);
    const auto before = maneuvers;
    EXPECT_TRUE(fixture.post_stage.pruneManeuvers(&maneuvers));
    ASSERT_EQ(maneuvers.size(), before.size());
    EXPECT_EQ(maneuvers[1].direction, Direction::UNKNOWN);
    for (std::size_t m = 0; m < maneuvers.size(); ++m) {
        ASSERT_EQ(maneuvers[m].points.size(), before[m].points.size());
        for (std::size_t i = 0; i < maneuvers[m].points.size(); ++i) {
            EXPECT_DOUBLE_EQ(maneuvers[m].points[i].x, before[m].points[i].x);
            EXPECT_DOUBLE_EQ(maneuvers[m].points[i].y, before[m].points[i].y);
            EXPECT_DOUBLE_EQ(maneuvers[m].points[i].theta,
                             before[m].points[i].theta);
            EXPECT_DOUBLE_EQ(maneuvers[m].points[i].getV(),
                             before[m].points[i].getV());
            EXPECT_DOUBLE_EQ(maneuvers[m].points[i].getDelta(),
                             before[m].points[i].getDelta());
        }
    }
}

// ==================== 门控计划构建 ====================

// 门控约束方向（易犯符号错误，两个方向都测）：前进 maneuver 的符号门为
// +1（约束 −v≤0，即禁止倒退）、倒退 maneuver 为 −1（约束 +v≤0，即禁止
// 前进），接缝点由零速等式接管、符号门清零
TEST(iLQRGatingPlanTest, SignGateOrientationBothDirections) {
    PostStageFixture fixture;
    std::vector<ProfileEntry> profile;
    AppendRun(&profile, 0.05, 21, 0.5, 0.2);     // 前进 maneuver
    AppendRun(&profile, -0.05, 20, -0.5, -0.3);  // 倒退 maneuver
    const auto states = MakeStates(profile);
    auto maneuvers = fixture.post_stage.buildManeuvers(
        states, fixture.post_stage.analyzeSignRuns(states));
    ASSERT_EQ(maneuvers.size(), 2);
    EXPECT_TRUE(fixture.post_stage.pruneManeuvers(&maneuvers));
    const Path pruned = ReconstructPath(maneuvers);
    const iLQRReference ref2 = fixture.reference_builder.build(pruned);
    ASSERT_EQ(ref2.cusp_indices.size(), 1);
    const auto build = fixture.post_stage.buildGatingPlan(ref2, pruned);
    const std::size_t seam = ref2.cusp_indices[0];
    // 前进段（接缝前）符号门 +1、倒退段（接缝后）−1、接缝点清零
    EXPECT_EQ(build.plan.sign_gate[0], 1);
    EXPECT_EQ(build.plan.sign_gate[seam - 1], 1);
    EXPECT_EQ(build.plan.sign_gate[seam], 0);
    EXPECT_EQ(build.plan.sign_gate[seam + 1], -1);
    EXPECT_EQ(build.plan.sign_gate.back(), -1);
    // 接缝查表一致：接缝位置指向其序号，其余位置为 −1
    EXPECT_EQ(build.plan.seam_lookup[seam], 0);
    EXPECT_EQ(build.plan.seam_lookup[seam + 1], -1);
}

// 驻留窗宽逐接缝依赖转向需求：接缝 1 的 Δδ=|−0.3−0.2|=0.5（梯形分支，
// T_resteer=1.5 s），接缝 2 的 Δδ=|0−(−0.3)|=0.3（T_resteer=1.1 s）——
// 大转角接缝的驻留窗必须更宽，且 T_resteer 量测与解析值一致
TEST(iLQRGatingPlanTest, DwellWindowWidthScalesWithSteerDemand) {
    PostStageFixture fixture;
    std::vector<ProfileEntry> profile;
    AppendRun(&profile, 0.05, 21, 0.5, 0.2);
    AppendRun(&profile, -0.05, 20, -0.5, -0.3);
    AppendRun(&profile, 0.05, 20, 0.5, 0.0);
    const auto states = MakeStates(profile);
    auto maneuvers = fixture.post_stage.buildManeuvers(
        states, fixture.post_stage.analyzeSignRuns(states));
    ASSERT_EQ(maneuvers.size(), 3);
    EXPECT_TRUE(fixture.post_stage.pruneManeuvers(&maneuvers));
    const Path pruned = ReconstructPath(maneuvers);
    const iLQRReference ref2 = fixture.reference_builder.build(pruned);
    ASSERT_EQ(ref2.cusp_indices.size(), 2);
    const auto build = fixture.post_stage.buildGatingPlan(ref2, pruned);
    ASSERT_EQ(build.seams.size(), 2);
    // 转向需求量测（阶段一输出接缝前后最后一个 |v|>v_dwell 点的 δ 差）
    EXPECT_NEAR(build.seams[0].delta_delta, 0.5, 1e-9);
    EXPECT_NEAR(build.seams[1].delta_delta, 0.3, 1e-9);
    // T_resteer 与解析值一致（梯形分支：Δδ/ω+ω/η）
    EXPECT_NEAR(build.seams[0].t_resteer, 0.5 / 0.5 + 0.5 / 1.0, 1e-9);
    EXPECT_NEAR(build.seams[1].t_resteer, 0.3 / 0.5 + 0.5 / 1.0, 1e-9);
    // 大转角接缝窗更宽：m1=⌈1.5/0.2⌉=8 vs m2=⌈1.1/0.2⌉=6
    const auto span0 = build.seams[0].window_end - build.seams[0].window_begin;
    const auto span1 = build.seams[1].window_end - build.seams[1].window_begin;
    EXPECT_GT(span0, span1);
    EXPECT_EQ(span0, 16);
    EXPECT_EQ(span1, 12);
    // 窗内驻留速度帽生效、窗外为零（无窗标记）
    const std::size_t seam0 = ref2.cusp_indices[0];
    EXPECT_GT(build.plan.dwell_v_cap[seam0], 0.0);
    EXPECT_DOUBLE_EQ(build.plan.dwell_v_cap[seam0],
                     fixture.post_config.v_dwell);
    EXPECT_DOUBLE_EQ(build.plan.dwell_v_cap[0], 0.0);
}

// ==================== 代价层门控约束项 ====================

// 最小合成参考（3 位姿、N=2）：仅用于代价层门控项求值
iLQRReference MakeTinyReference() {
    iLQRReference reference;
    reference.dt = 0.1;
    reference.ds = 0.05;
    reference.poses.emplace_back(0.0, 0.0, 0.0);
    reference.poses.emplace_back(0.05, 0.0, 0.0);
    reference.poses.emplace_back(0.10, 0.0, 0.0);
    reference.initial_states.resize(3, iLQRState::Zero());
    reference.initial_controls.resize(2, iLQRControl::Zero());
    return reference;
}

// 最小门控计划：节点 0 符号门 +1 + 驻留帽 0.05，节点 1 符号门 +1 + 接缝
// 等式，节点 2（终端）符号门 +1（应被忽略：终端由终点等式接管）
iLQRGatingPlan MakeTinyPlan() {
    iLQRGatingPlan plan;
    plan.sign_gate = {1, 1, 1};
    plan.seam_indices = {1};
    plan.seam_lookup = {-1, 0, -1};
    plan.dwell_v_cap = {0.05, 0.0, 0.0};
    return plan;
}

// 阶段一/阶段二隔离：阶段一配置下（无门控计划、乘子门控向量为空）门控
// 项恒为零——逐阶段 cost_gating 为 0，总代价与无门控扩展时逐位一致
TEST(iLQRGatingCostTest, StageOneInputProducesZeroGatingCost) {
    const iLQRReference reference = MakeTinyReference();
    const iLQRCostEvaluator evaluator(iLQRCostConfig{}, nullptr);
    const auto multipliers = iLQRCostMultiplierState::MakeZero(2);
    iLQRCostInput input;
    input.tracking_weight = 1.0;
    const iLQRAlignedVec<iLQRState> states(3, iLQRState::Zero());
    const iLQRAlignedVec<iLQRControl> controls(2, iLQRControl::Zero());
    const auto eval =
        evaluator.evaluate(reference, states, controls, multipliers, input);
    for (const auto& stage : eval.stages) {
        EXPECT_DOUBLE_EQ(stage.cost_gating, 0.0);
    }
    // 总代价 = 平滑 + 跟踪（零状态零控制下幅值项不激活）
    double expected = 0.0;
    for (const auto& stage : eval.stages) {
        expected += stage.cost_smooth + stage.cost_tracking +
                    stage.cost_amplitude + stage.cost_esdf +
                    stage.cost_terminal;
    }
    EXPECT_DOUBLE_EQ(eval.total_cost, expected);
}

// 「阶段一禁止启用」断言防御：门控计划与门控乘子必须同在场——只有计划
// 没有乘子、或只有乘子没有计划，都是配置错误，必须显式抛 std::logic_error
TEST(iLQRGatingCostTest, PartialGatingConfigThrows) {
    const iLQRReference reference = MakeTinyReference();
    const iLQRCostEvaluator evaluator(iLQRCostConfig{}, nullptr);
    const iLQRGatingPlan plan = MakeTinyPlan();
    const iLQRAlignedVec<iLQRState> states(3, iLQRState::Zero());
    const iLQRAlignedVec<iLQRControl> controls(2, iLQRControl::Zero());
    iLQRCostInput input;
    input.tracking_weight = 1.0;
    // 只有计划、乘子门控向量为空 → 拒绝
    input.gating_plan = &plan;
    const auto stage_one_multipliers = iLQRCostMultiplierState::MakeZero(2);
    EXPECT_THROW(evaluator.evaluate(reference, states, controls,
                                    stage_one_multipliers, input),
                 std::logic_error);
    // 只有乘子、没有计划 → 拒绝
    input.gating_plan = nullptr;
    const auto gating_multipliers =
        iLQRCostMultiplierState::MakeStageTwoZero(2, 1);
    EXPECT_THROW(evaluator.evaluate(reference, states, controls,
                                    gating_multipliers, input),
                 std::logic_error);
    // 乘子尺寸与计划不符 → 拒绝
    input.gating_plan = &plan;
    auto bad_multipliers = iLQRCostMultiplierState::MakeStageTwoZero(2, 1);
    bad_multipliers.gating_seam_lambda.setZero(2);
    EXPECT_THROW(
        evaluator.evaluate(reference, states, controls, bad_multipliers, input),
        std::invalid_argument);
}

// 门控约束方向（代价层，两个方向都测）：符号门 −s·v≤0 的惩罚与梯度取向——
// s=+1 时负 v 受罚（梯度把 v 向上推），正 v 自由；s=−1 时正 v 受罚
// （梯度把 v 向下推），负 v 自由
TEST(iLQRGatingCostTest, SignGatePenaltyDirectionBothSigns) {
    const iLQRReference reference = MakeTinyReference();
    const iLQRCostEvaluator evaluator(iLQRCostConfig{}, nullptr);
    iLQRGatingPlan plan = MakeTinyPlan();
    plan.seam_indices.clear();
    plan.seam_lookup = {-1, -1, -1};
    plan.dwell_v_cap = {0.0, 0.0, 0.0};
    iLQRCostInput input;
    input.tracking_weight = 0.0;
    input.gating_plan = &plan;
    // 前进门（s=+1）：v=−0.2 受罚
    auto multipliers = iLQRCostMultiplierState::MakeStageTwoZero(2, 0);
    multipliers.gating_sign_mu.setConstant(10.0);
    iLQRAlignedVec<iLQRState> states(3, iLQRState::Zero());
    states[0](ILQR_IDX_V) = -0.2;
    const iLQRAlignedVec<iLQRControl> controls(2, iLQRControl::Zero());
    auto eval =
        evaluator.evaluate(reference, states, controls, multipliers, input);
    // g = −(+1)·(−0.2) = 0.2 > 0：代价 λg+½μg²=0.2，梯度 (λ+μg)·(−1)=−2
    EXPECT_NEAR(eval.stages[0].cost_gating, 0.2, 1e-12);
    EXPECT_NEAR(eval.stages[0].lx(ILQR_IDX_V), -2.0, 1e-9);
    EXPECT_NEAR(eval.stages[0].lxx(ILQR_IDX_V, ILQR_IDX_V), 10.0, 1e-9);
    // v=+0.2 不激活（g<0 且 λ=0）
    states[0](ILQR_IDX_V) = 0.2;
    eval = evaluator.evaluate(reference, states, controls, multipliers, input);
    EXPECT_DOUBLE_EQ(eval.stages[0].cost_gating, 0.0);
    EXPECT_DOUBLE_EQ(eval.stages[0].lx(ILQR_IDX_V), 0.0);
    // 倒退门（s=−1）：v=+0.2 受罚（梯度把 v 向下推）
    plan.sign_gate = {-1, -1, -1};
    eval = evaluator.evaluate(reference, states, controls, multipliers, input);
    EXPECT_NEAR(eval.stages[0].cost_gating, 0.2, 1e-12);
    EXPECT_NEAR(eval.stages[0].lx(ILQR_IDX_V), 2.0, 1e-9);
    // v=−0.2 不激活
    states[0](ILQR_IDX_V) = -0.2;
    eval = evaluator.evaluate(reference, states, controls, multipliers, input);
    EXPECT_DOUBLE_EQ(eval.stages[0].cost_gating, 0.0);
}

// 接缝零速等式与驻留速度帽的取值：等式 c=v 以 λc+½μc² 计入；驻留帽取
// g=|v|−cap 形式（活动区梯度模恒为 1、对符号中性，二阶导在活动区精确）
TEST(iLQRGatingCostTest, SeamEqualityAndDwellGateValues) {
    const iLQRReference reference = MakeTinyReference();
    const iLQRCostEvaluator evaluator(iLQRCostConfig{}, nullptr);
    iLQRGatingPlan plan = MakeTinyPlan();
    plan.sign_gate = {0, 0, 0};  // 关闭符号门，隔离被测项
    iLQRCostInput input;
    input.tracking_weight = 0.0;
    input.gating_plan = &plan;
    auto multipliers = iLQRCostMultiplierState::MakeStageTwoZero(2, 1);
    multipliers.gating_seam_mu.setConstant(10.0);
    multipliers.gating_dwell_mu.setConstant(10.0);
    iLQRAlignedVec<iLQRState> states(3, iLQRState::Zero());
    const iLQRAlignedVec<iLQRControl> controls(2, iLQRControl::Zero());
    // 接缝节点 v=0.1：c=0.1，代价 ½·10·0.01=0.05，梯度 10·0.1=1.0
    states[1](ILQR_IDX_V) = 0.1;
    auto eval =
        evaluator.evaluate(reference, states, controls, multipliers, input);
    EXPECT_NEAR(eval.stages[1].cost_gating, 0.05, 1e-12);
    EXPECT_NEAR(eval.stages[1].lx(ILQR_IDX_V), 1.0, 1e-9);
    EXPECT_NEAR(eval.stages[1].lxx(ILQR_IDX_V, ILQR_IDX_V), 10.0, 1e-9);
    // 驻留帽节点 v=−0.2（cap=0.05）：g=0.2−0.05=0.15，
    // 代价 ½·10·0.0225=0.1125，梯度 (μg)·(−1)=−1.5，GN Hessian μ=10
    states[1](ILQR_IDX_V) = 0.0;
    states[0](ILQR_IDX_V) = -0.2;
    eval = evaluator.evaluate(reference, states, controls, multipliers, input);
    EXPECT_NEAR(eval.stages[0].cost_gating, 0.1125, 1e-12);
    EXPECT_NEAR(eval.stages[0].lx(ILQR_IDX_V), -1.5, 1e-9);
    EXPECT_NEAR(eval.stages[0].lxx(ILQR_IDX_V, ILQR_IDX_V), 10.0, 1e-9);
    // 窗内 |v|≤cap 不激活
    states[0](ILQR_IDX_V) = 0.03;
    eval = evaluator.evaluate(reference, states, controls, multipliers, input);
    EXPECT_DOUBLE_EQ(eval.stages[0].cost_gating, 0.0);
}

// 门控项导数的有限差分对拍：符号门 + 驻留帽同时激活的阶段，ℓ_x 与
// ℓ_xx 的 v 分量与解析值/中心差分一致。符号门残差线性、驻留帽残差
// |v|−cap（活动区光滑），两者的 ½μg² 二阶导在活动区都是精确的 μ——
// GN 形 Hessian 此处无丢弃项，可直接与解析值逐位对拍
TEST(iLQRGatingCostTest, GatingDerivativesMatchFiniteDifference) {
    const iLQRReference reference = MakeTinyReference();
    const iLQRCostEvaluator evaluator(iLQRCostConfig{}, nullptr);
    const iLQRGatingPlan plan = MakeTinyPlan();
    iLQRCostInput input;
    input.tracking_weight = 0.0;
    input.gating_plan = &plan;
    auto multipliers = iLQRCostMultiplierState::MakeStageTwoZero(2, 1);
    multipliers.gating_sign_mu.setConstant(10.0);
    multipliers.gating_seam_mu.setConstant(10.0);
    multipliers.gating_dwell_mu.setConstant(10.0);
    // 节点 0：v=−0.2 同时违反符号门与驻留帽
    iLQRAlignedVec<iLQRState> states(3, iLQRState::Zero());
    states[0](ILQR_IDX_V) = -0.2;
    const iLQRAlignedVec<iLQRControl> controls(2, iLQRControl::Zero());
    const auto eval =
        evaluator.evaluate(reference, states, controls, multipliers, input);
    const auto gating_cost = [&](double v) {
        iLQRAlignedVec<iLQRState> perturbed = states;
        perturbed[0](ILQR_IDX_V) = v;
        return evaluator
            .evaluate(reference, perturbed, controls, multipliers, input)
            .stages[0]
            .cost_gating;
    };
    constexpr double h = 1e-6;
    // 梯度与 Hessian 在活动区都是精确值：与中心差分对拍
    const double fd_grad =
        (gating_cost(-0.2 + h) - gating_cost(-0.2 - h)) / (2.0 * h);
    const double fd_hess = (gating_cost(-0.2 + h) - 2.0 * gating_cost(-0.2) +
                            gating_cost(-0.2 - h)) /
                           (h * h);
    EXPECT_NEAR(eval.stages[0].lx(ILQR_IDX_V), fd_grad, 1e-4);
    // 解析值：符号门梯度 (μ·0.2)·(−1)=−2 + 驻留帽梯度 (μ·0.15)·(−1)=−1.5，
    // 合计 −3.5；Hessian 两组约束各 μ=10，合计 20
    EXPECT_NEAR(eval.stages[0].lx(ILQR_IDX_V), -3.5, 1e-9);
    EXPECT_NEAR(eval.stages[0].lxx(ILQR_IDX_V, ILQR_IDX_V), 20.0, 1e-9);
    EXPECT_NEAR(eval.stages[0].lxx(ILQR_IDX_V, ILQR_IDX_V), fd_hess, 1e-2);
    // 门控只依赖 v：其余状态/控制导数分量恒零
    for (int i = 0; i < ILQR_STATE_DIM; ++i) {
        if (i != ILQR_IDX_V) {
            EXPECT_DOUBLE_EQ(eval.stages[0].lx(i), 0.0);
        }
    }
    EXPECT_DOUBLE_EQ(eval.stages[0].lu.norm(), 0.0);
}

// ==================== 阶段二热启动映射 ====================

// 独立验证热启动映射：修剪后路径的阶段一状态量（v/a/δ/ω）按累积弧长
// 查表线性插值到重采样网格（位姿取参考位姿本身），控制量由 a/ω 差分
// 反解并裁剪进盒——前进 1.0 m（v=0.5, δ=0.2）→ 倒退 1.0 m（v=-0.5,
// δ=-0.3），网格间距与点距同为 0.05 m 使插值结果逐位可预期；注入单点
// a=0.3 与 ω=0.25 阶跃，差分反解 j=Δa/dt=3.0>j_max、η=Δω/dt=2.5>η_max
// 必须被裁剪到盒边界
TEST(iLQRStageTwoWarmStartTest, InterpolatesStatesAndClampsControls) {
    PostStageFixture fixture;
    std::vector<ProfileEntry> profile;
    AppendRun(&profile, 0.05, 21, 0.5, 0.2);     // x: 0 → 1.0（前进）
    AppendRun(&profile, -0.05, 20, -0.5, -0.3);  // x: 0.95 → 0.0（倒退）
    // 单点阶跃注入：a 在索引 10、ω 在索引 15（差分反解产生超盒控制量）
    profile[10].a = 0.3;
    profile[15].omega = 0.25;
    const auto states = MakeStates(profile);
    auto maneuvers = fixture.post_stage.buildManeuvers(
        states, fixture.post_stage.analyzeSignRuns(states));
    ASSERT_EQ(maneuvers.size(), 2);
    EXPECT_TRUE(fixture.post_stage.pruneManeuvers(&maneuvers));
    const Path pruned = ReconstructPath(maneuvers);
    const iLQRReference reference = fixture.reference_builder.build(pruned);
    iLQRAlignedVec<iLQRState> warm_states;
    iLQRAlignedVec<iLQRControl> warm_controls;
    fixture.post_stage.buildStageTwoWarmStart(pruned, reference, &warm_states,
                                              &warm_controls);
    ASSERT_EQ(warm_states.size(), reference.poses.size());
    ASSERT_EQ(warm_controls.size(), reference.poses.size() - 1);
    // 位姿取参考位姿本身
    for (std::size_t k = 0; k < reference.poses.size(); ++k) {
        EXPECT_DOUBLE_EQ(warm_states[k](ILQR_IDX_X), reference.poses[k].x);
        EXPECT_DOUBLE_EQ(warm_states[k](ILQR_IDX_Y), reference.poses[k].y);
        EXPECT_DOUBLE_EQ(warm_states[k](ILQR_IDX_THETA),
                         reference.poses[k].theta);
    }
    // v/δ 按弧长插值：前进段 0.5/0.2、倒退段 −0.5/−0.3，翻转发生在接缝
    // 弧长（1.0 m）之后的第一个网格点（弧长累积与网格目标的浮点路径
    // 不同，插值比值与 1.0 有 1e-15 量级差异，按 1e-9 容差对拍）
    EXPECT_NEAR(warm_states[10](ILQR_IDX_V), 0.5, 1e-9);
    EXPECT_NEAR(warm_states[20](ILQR_IDX_V), 0.5, 1e-9);
    EXPECT_NEAR(warm_states[21](ILQR_IDX_V), -0.5, 1e-9);
    EXPECT_NEAR(warm_states[25](ILQR_IDX_V), -0.5, 1e-9);
    EXPECT_NEAR(warm_states[10](ILQR_IDX_DELTA), 0.2, 1e-9);
    EXPECT_NEAR(warm_states[25](ILQR_IDX_DELTA), -0.3, 1e-9);
    // a/ω 阶跃逐位映射到网格点（点距与网格间距一致，插值在网格点精确）
    EXPECT_NEAR(warm_states[10](ILQR_IDX_A), 0.3, 1e-9);
    EXPECT_NEAR(warm_states[15](ILQR_IDX_OMEGA), 0.25, 1e-9);
    // 控制量差分反解并裁剪进盒：j=±3.0→±j_max、η=±2.5→±η_max，
    // 未受阶跃影响的控制量为 0
    const auto& limits = fixture.solver_config.inner;
    EXPECT_DOUBLE_EQ(warm_controls[9](ILQR_IDX_JERK), limits.jerk_max);
    EXPECT_DOUBLE_EQ(warm_controls[10](ILQR_IDX_JERK), -limits.jerk_max);
    EXPECT_DOUBLE_EQ(warm_controls[14](ILQR_IDX_ETA), limits.steer_accel_max);
    EXPECT_DOUBLE_EQ(warm_controls[15](ILQR_IDX_ETA), -limits.steer_accel_max);
    EXPECT_DOUBLE_EQ(warm_controls[0](ILQR_IDX_JERK), 0.0);
    EXPECT_DOUBLE_EQ(warm_controls[0](ILQR_IDX_ETA), 0.0);
}

// 数据来源链防御（评审加固）：修剪后路径点缺少阶段一状态量（v/a/δ/ω）
// 时，热启动必须以带明确现场信息的 std::logic_error 快速失败——普通
// A* 路径点不携带派生量，正好充当隐式契约违例的注入源
TEST(iLQRStageTwoWarmStartTest, MissingDerivedQuantitiesThrowLogicError) {
    PostStageFixture fixture;
    const Path path = BuildXPolyline({0.0, 1.0, -1.0});
    const iLQRReference reference = fixture.reference_builder.build(path);
    iLQRAlignedVec<iLQRState> warm_states;
    iLQRAlignedVec<iLQRControl> warm_controls;
    EXPECT_THROW(fixture.post_stage.buildStageTwoWarmStart(
                     path, reference, &warm_states, &warm_controls),
                 std::logic_error);
}

// ==================== 阶段二门控重解 ====================

// 手工构建阶段二门控计划（按参考 maneuver 元数据 + 接缝窗宽 m）
iLQRGatingPlan MakeSolverGatingPlan(const iLQRReference& reference,
                                   std::size_t window_half, double v_cap) {
    const std::size_t num_poses = reference.poses.size();
    iLQRGatingPlan plan;
    plan.sign_gate.assign(num_poses, 0);
    for (const auto& maneuver : reference.maneuvers) {
        for (std::size_t k = maneuver.begin_index; k <= maneuver.end_index;
             ++k) {
            plan.sign_gate[k] = maneuver.sign;
        }
    }
    plan.seam_indices = reference.cusp_indices;
    plan.seam_lookup.assign(num_poses, -1);
    plan.dwell_v_cap.assign(num_poses, 0.0);
    for (std::size_t j = 0; j < plan.seam_indices.size(); ++j) {
        const std::size_t seam = plan.seam_indices[j];
        plan.sign_gate[seam] = 0;
        plan.seam_lookup[seam] = static_cast<int>(j);
        const std::size_t begin = seam > window_half ? seam - window_half : 0;
        const std::size_t end = std::min(seam + window_half, num_poses - 1);
        for (std::size_t k = begin; k <= end; ++k) {
            plan.dwell_v_cap[k] = v_cap;
        }
    }
    return plan;
}

// 阶段二输入校验：门控计划尺寸与参考网格不符、热启动尺寸不符等契约
// 违例必须显式抛出，禁止带着畸形问题进入求解循环
TEST(iLQRStageTwoSolveTest, InvalidGatingPlanThrows) {
    PostStageFixture fixture;
    const Path path = BuildXPolyline({0.0, 1.0, -1.0});
    const iLQRReference reference = BuildReference(path);
    const auto plan = MakeSolverGatingPlan(reference, 2, 0.05);
    // 符号门尺寸不符
    auto bad_plan = plan;
    bad_plan.sign_gate.pop_back();
    EXPECT_THROW(fixture.solver.solveStageTwo(reference, bad_plan,
                                              reference.initial_states,
                                              reference.initial_controls, 1.0),
                 std::invalid_argument);
    // 热启动尺寸不符
    auto bad_warm = reference.initial_states;
    bad_warm.pop_back();
    EXPECT_THROW(fixture.solver.solveStageTwo(reference, plan, bad_warm,
                                              reference.initial_controls, 1.0),
                 std::invalid_argument);
}

// 阶段二重解收敛性：前进 1 m → 倒退 2 m 的保留换挡场景，以阶段一解
// 热启动，施加符号门/接缝零速/驻留帽三类门控——解必须收敛且三类门控
// 残差全部压回容差内：前进段无倒退、倒退段无前进、接缝零速、窗内
// |v|≤v_dwell，终点双指标照常达标
TEST(iLQRStageTwoSolveTest, StageTwoEnforcesSignSeamAndDwellGates) {
    PostStageFixture fixture;
    const Path path = BuildXPolyline({0.0, 1.0, -1.0});
    const iLQRReference reference = BuildReference(path);
    // 先跑阶段一获得热启动（保留换挡：倒退 maneuver 承载终点语义）
    const auto stage_one = fixture.solver.solveStageOne(reference);
    ASSERT_EQ(stage_one.report.status, ApaILQRStatus::CONVERGED)
        << "outer=" << stage_one.report.outer_iterations
        << " pos_err=" << stage_one.report.terminal_position_error;
    const std::size_t seam = reference.cusp_indices[0];
    const auto plan = MakeSolverGatingPlan(reference, 2, 0.05);
    // 跟踪权重冻结在阶段一末轮退火值（与生产后处理同一约定）
    const double stage_one_final_weight =
        stage_one.report.history.empty()
            ? fixture.solver_config.cost.weight_ref_base
            : stage_one.report.history.back().tracking_weight;
    const auto stage_two = fixture.solver.solveStageTwo(
        reference, plan, stage_one.states, stage_one.controls,
        stage_one_final_weight, &stage_one.final_multipliers);
    EXPECT_EQ(stage_two.report.status, ApaILQRStatus::CONVERGED)
        << "outer=" << stage_two.report.outer_iterations
        << " sign=" << stage_two.max_sign_violation
        << " dwell=" << stage_two.max_dwell_violation
        << " seam=" << stage_two.max_seam_speed
        << " pos_err=" << stage_two.report.terminal_position_error;
    EXPECT_TRUE(stage_two.gating_ok);
    // 符号门：前进段（窗外）无倒退、倒退段（窗外）无前进
    for (std::size_t k = 0; k + 2 < seam; ++k) {
        EXPECT_GE(stage_two.states[k](ILQR_IDX_V), -0.02) << "k=" << k;
    }
    for (std::size_t k = seam + 3; k < stage_two.states.size(); ++k) {
        EXPECT_LE(stage_two.states[k](ILQR_IDX_V), 0.02) << "k=" << k;
    }
    // 接缝零速与驻留窗速度帽
    EXPECT_LE(stage_two.max_seam_speed, 0.02);
    for (std::size_t k = seam - 2; k <= seam + 2; ++k) {
        EXPECT_LE(std::abs(stage_two.states[k](ILQR_IDX_V)), 0.05 + 0.02)
            << "k=" << k;
    }
    // 终点照常收敛
    EXPECT_LE(stage_two.report.terminal_position_error, 0.05);
    EXPECT_LE(stage_two.report.terminal_heading_error_deg, 1.5);
}

// 白盒测试夹具：派生暴露 protected 门控乘子更新接口（不改动被测实现，
// 仅用于钉住门控 μ 增长排程）
class GatingUpdateAccessor : public ApaILQRSolver {
   public:
    using ApaILQRSolver::ApaILQRSolver;
    using ApaILQRSolver::GatingSnapshot;
    using ApaILQRSolver::updateGating;
};

// 门控 μ 增长的充分下降门控（显式钉住）：首轮只记录不增长；违反度充分
// 下降（≤κ·上轮）时 μ 保持原值；未充分下降（>κ·上轮）时 μ 按 φ 倍提升
// 并同步写回三组门控乘子；λ 更新遵循 Hestenes-Powell（不等式投影非负、
// 等式符号自由）
TEST(iLQRStageTwoGatingMuTest, SufficientDecreaseDoesNotGrowMu) {
    const ApaILQRSolverConfig config = MakeSyntheticConfig();
    const BicycleDynamics dynamics(kWheelbase);
    const iLQRCostEvaluator evaluator(config.cost, nullptr);
    GatingUpdateAccessor solver(config, &dynamics, &evaluator);
    auto multipliers = iLQRCostMultiplierState::MakeStageTwoZero(1, 1);
    // 快照：节点 0 符号门违反 0.2、接缝等式残差 0.1
    GatingUpdateAccessor::GatingSnapshot snapshot;
    snapshot.sign_g = Eigen::VectorXd::Zero(2);
    snapshot.dwell_g = Eigen::VectorXd::Zero(2);
    snapshot.seam_c = Eigen::VectorXd::Zero(1);
    snapshot.sign_g(0) = 0.2;
    snapshot.seam_c(0) = 0.1;
    snapshot.violation_norm = std::sqrt(0.2 * 0.2 + 0.1 * 0.1);
    double gating_mu = 10.0;
    double prev = -1.0;
    // 首轮（prev<0）：只记录不增长；λ 按 μ·g 累积（不等式投影非负）
    EXPECT_FALSE(
        solver.updateGating(snapshot, &multipliers, &gating_mu, &prev));
    EXPECT_DOUBLE_EQ(gating_mu, 10.0);
    EXPECT_DOUBLE_EQ(multipliers.gating_sign_lambda(0), 10.0 * 0.2);
    EXPECT_DOUBLE_EQ(multipliers.gating_seam_lambda(0), 10.0 * 0.1);
    // 次轮充分下降（违反度减半 < κ·prev=0.9·prev）→ μ 不增长
    snapshot.sign_g(0) = 0.1;
    snapshot.seam_c(0) = 0.05;
    snapshot.violation_norm = std::sqrt(0.1 * 0.1 + 0.05 * 0.05);
    EXPECT_FALSE(
        solver.updateGating(snapshot, &multipliers, &gating_mu, &prev));
    EXPECT_DOUBLE_EQ(gating_mu, 10.0);
    // 第三轮未充分下降（0.95·prev > κ·prev）→ μ ×10 并写回三组乘子
    const double prev_norm = snapshot.violation_norm;
    snapshot.violation_norm = 0.95 * prev_norm;
    EXPECT_TRUE(
        solver.updateGating(snapshot, &multipliers, &gating_mu, &prev));
    EXPECT_DOUBLE_EQ(gating_mu, 100.0);
    EXPECT_DOUBLE_EQ(multipliers.gating_sign_mu(0), 100.0);
    EXPECT_DOUBLE_EQ(multipliers.gating_dwell_mu(0), 100.0);
    EXPECT_DOUBLE_EQ(multipliers.gating_seam_mu(0), 100.0);
}

// 阶段二对偶热启动对照：同一原始热启动下，带阶段一终态乘子种子
// （dual_seed）的重解外层轮数不得超过对偶冷启动——已收敛的 AL 平衡由
// λ/μ 承载，续接而非推倒重来（冷启动实测把终端平衡重新打开、轮数翻倍）
TEST(iLQRStageTwoSolveTest, DualSeedWarmStartConvergesNoSlowerThanCold) {
    PostStageFixture fixture;
    const Path path = BuildXPolyline({0.0, 1.0, 0.85, 1.85});
    const iLQRReference reference = BuildReference(path);
    const auto stage_one = fixture.solver.solveStageOne(reference);
    ASSERT_EQ(stage_one.report.status, ApaILQRStatus::CONVERGED);
    // 门控计划按修剪后结构构建（与真实后处理同源）：阶段一解已融化为
    // 单 maneuver（v 全程为正），故全段 +1 符号门、无接缝无驻留窗——
    // 若误用未修剪的三段计划（含倒退门），会迫使阶段二"反向融化"
    iLQRGatingPlan plan;
    plan.sign_gate.assign(reference.poses.size(), 1);
    plan.seam_lookup.assign(reference.poses.size(), -1);
    plan.dwell_v_cap.assign(reference.poses.size(), 0.0);
    const double tracking_weight =
        stage_one.report.history.back().tracking_weight;
    // 两个独立求解器实例：内层 μ_m/ρ_reg 跨 solve 调用保持，对照实验
    // 必须各自独立，避免第一次求解的内层状态污染第二次
    const ApaILQRSolverConfig config = MakeSyntheticConfig();
    const BicycleDynamics dynamics(kWheelbase);
    const iLQRCostEvaluator evaluator(config.cost, nullptr);
    ApaILQRSolver warm_solver(config, &dynamics, &evaluator);
    ApaILQRSolver cold_solver(config, &dynamics, &evaluator);
    const auto warm = warm_solver.solveStageTwo(
        reference, plan, stage_one.states, stage_one.controls, tracking_weight,
        &stage_one.final_multipliers);
    const auto cold =
        cold_solver.solveStageTwo(reference, plan, stage_one.states,
                                  stage_one.controls, tracking_weight, nullptr);
    EXPECT_EQ(warm.report.status, ApaILQRStatus::CONVERGED)
        << "outer=" << warm.report.outer_iterations
        << " pos_err=" << warm.report.terminal_position_error;
    EXPECT_LE(warm.report.outer_iterations, cold.report.outer_iterations)
        << "warm=" << warm.report.outer_iterations
        << " cold=" << cold.report.outer_iterations;
    std::cout << "[iLQR-DUAL] warm_outer=" << warm.report.outer_iterations
              << " cold_outer=" << cold.report.outer_iterations
              << " cold_status=" << static_cast<int>(cold.report.status)
              << std::endl;
}

// ==================== 驻留插入 ====================

// 驻留插入的时间拉伸语义：接缝 Δδ=0.3（T_resteer=1.1 s，T_dwell=1.32 s）
// 超过窗口时长 1.0 s——窗内 δ 摆动剖面按边缘斜坡重定时放慢（平台区
// ρ=1.5、窗口边沿 ρ 平滑过渡回 1，v/ω 同比减小），时间戳严格单调、
// 总时长增加、窗内 |v|≤v_dwell，且重排后轨迹仍通过 Trajectory::validate()
// 运动学门
TEST(iLQRDwellInsertTest, RetimesWindowAndKeepsKinematics) {
    PostStageFixture fixture;
    // 合成阶段二输出：接缝 k=20、驻留窗 [15,25] 的直线 reversal。
    // 剖面按运动学自洽构造：v 分段线性（a 为分段常数，梯形配点残差在
    // 段内精确为零），x 由 v 的梯形积分生成，窗内 |v|≤0.05 线性过零、
    // δ 从 0 线性摆到 0.3（ω=0.3 恒定，窗两端外 ω=0）
    std::vector<double> v_prof(41, 0.0), a_prof(41, 0.0), d_prof(41, 0.3),
        w_prof(41, 0.0);
    for (int k = 0; k <= 5; ++k) {
        v_prof[k] = 0.5;
        d_prof[k] = 0.0;
    }
    for (int k = 6; k <= 14; ++k) {
        v_prof[k] = 0.5 - 0.05 * (k - 5);
        a_prof[k] = -0.5;
        d_prof[k] = 0.0;
    }
    for (int k = 15; k <= 25; ++k) {
        v_prof[k] = 0.05 - 0.01 * (k - 15);
        a_prof[k] = -0.1;
        d_prof[k] = 0.03 * (k - 15);
        w_prof[k] = 0.3;
    }
    for (int k = 26; k <= 34; ++k) {
        v_prof[k] = -0.05 - 0.05 * (k - 25);
        a_prof[k] = -0.5;
    }
    for (int k = 35; k <= 40; ++k) {
        v_prof[k] = -0.5;
    }
    iLQRAlignedVec<iLQRState> states(41, iLQRState::Zero());
    for (std::size_t k = 0; k < states.size(); ++k) {
        states[k](ILQR_IDX_V) = v_prof[k];
        states[k](ILQR_IDX_A) = a_prof[k];
        states[k](ILQR_IDX_DELTA) = d_prof[k];
        states[k](ILQR_IDX_OMEGA) = w_prof[k];
        if (k > 0) {
            states[k](ILQR_IDX_X) =
                states[k - 1](ILQR_IDX_X) + 0.05 * (v_prof[k - 1] + v_prof[k]);
        }
    }
    const std::vector<iLQRSeamPlan> seams = {
        iLQRSeamPlan{20, 15, 25, 0.0, 0.0, 0.0}};
    std::vector<iLQRSeamReport> reports;
    const Trajectory output =
        fixture.post_stage.insertDwells(states, 0.1, seams, &reports);
    ASSERT_EQ(reports.size(), 1);
    // T_dwell=1.2·max(T_resteer(0.3)=1.1, 0.4)=1.32 s → 量化拉伸到 1.4 s
    EXPECT_NEAR(reports[0].delta_delta, 0.3, 1e-9);
    EXPECT_NEAR(reports[0].t_resteer, 1.1, 1e-9);
    EXPECT_NEAR(reports[0].t_dwell, 1.32, 1e-9);
    EXPECT_NEAR(reports[0].dwell_duration, 1.4, 1e-9);
    // 点数增加 (1.4−1.0)/0.1 = 4，时间戳严格单调、总时长 4.0+0.4=4.4 s
    EXPECT_EQ(output.size(), states.size() + 4);
    for (std::size_t i = 1; i < output.size(); ++i) {
        EXPECT_GT(output[i].getT(), output[i - 1].getT());
    }
    EXPECT_NEAR(output.back().getT(), 4.4, 1e-9);
    // 驻留段内 |v|≤v_dwell（拉伸后同比减小）、|δ̇|≤ω_max；
    // 静止段 v/a 近零语义逐点断言：|v|≤0.05+裕量；|a| 的上界为窗内原剖面
    // 幅值 0.1——边缘斜坡重定时在窗口边沿 ρ→1（速率连续过渡到窗外），
    // 边沿样本保持原速、平台区按 1/ρ² 缩放
    const double t_dwell_start = 15 * 0.1;
    for (const auto& pt : output) {
        if (pt.getT() >= t_dwell_start - 1e-9 &&
            pt.getT() <= t_dwell_start + 1.4 + 1e-9) {
            EXPECT_LE(std::abs(pt.getV()), 0.05 + 1e-9);
            EXPECT_LE(std::abs(pt.getA()), 0.1 + 1e-6);
            EXPECT_LE(std::abs(pt.getDeltaDot()), 0.5 + 1e-9);
        }
    }
    // 接缝点零速：接缝时刻（拉伸后 t=1.5+0.5·1.4=2.2 s）处 |v|≈0
    double seam_speed = 1.0;
    for (const auto& pt : output) {
        if (std::abs(pt.getT() - 2.2) < 1e-9) {
            seam_speed = std::abs(pt.getV());
        }
    }
    EXPECT_LE(seam_speed, 1e-9);
    // 重排后轨迹仍通过运动学门（梯形配点残差）：开阔地图下校验三门
    GridMap grid_map(0.1, 300, 200, Position{-15.0, -10.0}, {});
    const ESDFMap esdf_map(grid_map);
    const VehicleFootprintModel footprint_model(MakeVehicleParams(), 233, 2, 2);
    TrajectoryPoint goal;
    goal.x = output.back().x;
    goal.y = output.back().y;
    goal.theta = output.back().theta;
    const auto validation = output.validate(goal, esdf_map, footprint_model);
    EXPECT_TRUE(validation.kinematic_feasible)
        << validation.kinematic_detail
        << " pos=" << validation.max_kinematic_position_residual
        << " head=" << validation.max_kinematic_heading_residual_deg
        << " vel=" << validation.max_kinematic_velocity_residual;
}

// 边缘斜坡重定时钉住「窗口边界速度断点」伪影的修复：窗速未受帽约束的
// 降级候选（窗内 |v| 达 0.3+），若窗口边界以阶跃倍率重定时，边界点对的
// 梯形配点速度残差会顶穿 0.05 运动学门限（实测 0.06~0.12）；边缘斜坡
// 把速率变化摊入窗口边缘，边界点对残差退回原轨迹的一致值
TEST(iLQRDwellInsertTest, FastWindowFlanksKeepBoundaryResidualBelowGate) {
    PostStageFixture fixture;
    // 合成降级候选剖面：接缝 k=20、驻留窗 [15,25]；v 分段线性（a 取段
    // 斜率，段内梯形配点残差精确为零、接头处半格量级），窗口右边缘外
    // |v| 达 0.35——窗速未受帽约束（阶段一解无门控）的典型形态；阶跃
    // 倍率重定时下边界点对残差 ~0.2（必超 0.05 门限），边缘斜坡重定时
    // 下退回原剖面的接头量级
    std::vector<double> v_prof(41, 0.0), a_prof(41, 0.0), d_prof(41, 0.3),
        w_prof(41, 0.0);
    for (int k = 0; k <= 5; ++k) {
        v_prof[k] = 0.5;
        d_prof[k] = 0.0;
    }
    for (int k = 6; k <= 14; ++k) {
        v_prof[k] = 0.5 - 0.05 * (k - 5);
        a_prof[k] = -0.5;
        d_prof[k] = 0.0;
    }
    for (int k = 15; k <= 25; ++k) {
        v_prof[k] = 0.05 - 0.04 * (k - 15);
        a_prof[k] = -0.4;
        d_prof[k] = 0.03 * (k - 15);
        w_prof[k] = 0.3;
    }
    for (int k = 26; k <= 34; ++k) {
        v_prof[k] = -0.35 - 0.02 * (k - 25);
        a_prof[k] = -0.2;
    }
    for (int k = 35; k <= 40; ++k) {
        v_prof[k] = -0.53;
    }
    iLQRAlignedVec<iLQRState> states(41, iLQRState::Zero());
    for (std::size_t k = 0; k < states.size(); ++k) {
        states[k](ILQR_IDX_V) = v_prof[k];
        states[k](ILQR_IDX_A) = a_prof[k];
        states[k](ILQR_IDX_DELTA) = d_prof[k];
        states[k](ILQR_IDX_OMEGA) = w_prof[k];
        if (k > 0) {
            states[k](ILQR_IDX_X) =
                states[k - 1](ILQR_IDX_X) + 0.05 * (v_prof[k - 1] + v_prof[k]);
        }
    }
    // Δδ≈0.3 → T_resteer≈1.1 s、T_dwell≈1.32 s（量化 1.4 s），窗口
    // 原长 1.0 s → 平台倍率 r=1.5（阶跃倍率下边界残差必然超阈）
    const std::vector<iLQRSeamPlan> seams = {
        iLQRSeamPlan{20, 15, 25, 0.0, 0.0, 0.0}};
    std::vector<iLQRSeamReport> reports;
    const Trajectory output =
        fixture.post_stage.insertDwells(states, 0.1, seams, &reports);
    ASSERT_EQ(reports.size(), 1);
    // 逐点对计算梯形配点速度残差（校验③同款公式）：全部点对（含窗口
    // 边界对）都必须低于 0.05 门限
    double worst_residual = 0.0;
    std::size_t worst_pair = 0;
    for (std::size_t i = 0; i + 1 < output.size(); ++i) {
        const auto& p0 = output[i];
        const auto& p1 = output[i + 1];
        const double dt_pair = p1.getT() - p0.getT();
        ASSERT_GT(dt_pair, 0.0);
        const double residual = std::abs(
            (p1.getV() - p0.getV()) - 0.5 * dt_pair * (p0.getA() + p1.getA()));
        if (residual > worst_residual) {
            worst_residual = residual;
            worst_pair = i;
        }
    }
    EXPECT_LE(worst_residual, 0.05)
        << "worst pair=" << worst_pair << " t=" << output[worst_pair].getT()
        << " v0=" << output[worst_pair].getV()
        << " v1=" << output[worst_pair + 1].getV();
}

// 多接缝驻留插入：两个相距仅 2 个网格点的接缝（中间倒退段 0.1 m ≥ 0.05
// 保留不剔除），驻留窗按 m_j 定宽后与相邻窗口部分重叠、须按前一窗口
// 边界裁剪——输出时间戳必须严格单调递增（重叠窗口会让装配阶段重复
// 发射状态点并造成时间戳回退，本用例钉住"逐接缝窗口严格互不重叠"的
// 裁剪语义）
TEST(iLQRDwellInsertTest, ClippedWindowsOfCloseSeamsStayMonotonic) {
    PostStageFixture fixture;
    // 41 状态：前进(0..20, δ=0.2) → 倒退(21..22, δ=-0.3) → 前进(23..40,
    // δ=0.2)；中间倒退段弧长 0.1 m ≥ 0.05，修剪保留
    std::vector<ProfileEntry> profile;
    AppendRun(&profile, 0.05, 21, 0.5, 0.2);
    AppendRun(&profile, -0.05, 2, -0.5, -0.3);
    AppendRun(&profile, 0.05, 18, 0.5, 0.2);
    const auto states = MakeStates(profile);
    SeamPlanAccessor accessor(fixture.post_config, &fixture.reference_builder,
                              &fixture.solver, MakeVehicleParams());
    const auto plans = accessor.buildStageOneSeamPlans(states, {21, 23}, 0.1);
    ASSERT_EQ(plans.size(), 2);
    // 窗口严格互不重叠（裁剪语义直接断言），且恒包含自身接缝
    EXPECT_LT(plans[0].window_end, plans[1].window_begin);
    EXPECT_LE(plans[0].window_begin, plans[0].seam_index);
    EXPECT_GE(plans[0].window_end, plans[0].seam_index);
    EXPECT_LE(plans[1].window_begin, plans[1].seam_index);
    EXPECT_GE(plans[1].window_end, plans[1].seam_index);
    std::vector<iLQRSeamReport> reports;
    const Trajectory output =
        fixture.post_stage.insertDwells(states, 0.1, plans, &reports);
    ASSERT_EQ(reports.size(), 2);
    // 时间戳严格单调递增、点数恰为两个窗口拉伸量之和
    for (std::size_t i = 1; i < output.size(); ++i) {
        EXPECT_GT(output[i].getT(), output[i - 1].getT()) << "i=" << i;
    }
    EXPECT_EQ(output.size(), states.size() + 19);
    // 逐接缝驻留时长达到 T_dwell（两接缝 Δδ=0.5，T_resteer=1.5 s）
    for (const auto& report : reports) {
        EXPECT_NEAR(report.t_resteer, 1.5, 1e-9);
        EXPECT_GE(report.dwell_duration, report.t_dwell - 1e-9);
    }
}

// ==================== 后处理主流程（端到端） ====================

// 融化场景全链路：「前进 1.0 → 倒退 0.15 → 前进 1.0」的阶段一解（v 全程
// 不变号）经后处理恢复为单 maneuver——修剪无残余微段、阶段二重解收敛、
// 六项校验全过、输出非回退轨迹，maneuver 数 3→1 不增
TEST(iLQRPostStageTest, MeltedScenarioOutputsSingleManeuver) {
    PostStageFixture fixture;
    const Path path = BuildXPolyline({0.0, 1.0, 0.85, 1.85});
    const iLQRReference reference = BuildReference(path);
    const auto stage_one = fixture.solver.solveStageOne(reference);
    ASSERT_EQ(stage_one.report.status, ApaILQRStatus::CONVERGED)
        << "outer=" << stage_one.report.outer_iterations;
    GridMap grid_map(0.1, 300, 200, Position{-15.0, -10.0}, {});
    const ESDFMap esdf_map(grid_map);
    const VehicleFootprintModel footprint_model(MakeVehicleParams(), 233, 2, 2);
    TrajectoryPoint goal;
    goal.x = 1.85;
    goal.y = 0.0;
    goal.theta = 0.0;
    const auto result = fixture.post_stage.run(path, reference, stage_one, goal,
                                               esdf_map, footprint_model);
    EXPECT_EQ(result.status, iLQRPostStageStatus::SUCCESS)
        << "failed_check=" << result.diagnostics.failed_check
        << " measured=" << result.diagnostics.measured_value
        << " threshold=" << result.diagnostics.threshold;
    EXPECT_FALSE(result.used_fallback);
    EXPECT_EQ(result.diagnostics.input_maneuver_count, 3);
    EXPECT_EQ(result.diagnostics.output_maneuver_count, 1);
    // 输出轨迹完整：时间戳从零起严格单调、终点静止
    ASSERT_FALSE(result.trajectory.empty());
    for (std::size_t i = 1; i < result.trajectory.size(); ++i) {
        EXPECT_GT(result.trajectory[i].getT(), result.trajectory[i - 1].getT());
    }
    EXPECT_NEAR(result.trajectory.back().getV(), 0.0, 0.05);
    // 无接缝：无驻留插入，总时长 = N·dt
    EXPECT_TRUE(result.diagnostics.seams.empty());
}

// 阶段二跟踪权重地板（显式钉住）：地板高于阶段一末轮退火值时，阶段二
// 的逐轮跟踪权重必须取地板值（深退火调度下防止精化因跟踪过弱脱离热
// 启动邻域）；地板为 0 时保持既有「冻结在阶段一末轮值」行为
TEST(iLQRPostStageTest, StageTwoTrackingWeightFloorApplies) {
    PostStageFixture fixture;
    const Path path = BuildXPolyline({0.0, 1.0, 0.85, 1.85});
    const iLQRReference reference = BuildReference(path);
    const auto stage_one = fixture.solver.solveStageOne(reference);
    ASSERT_EQ(stage_one.report.status, ApaILQRStatus::CONVERGED)
        << "outer=" << stage_one.report.outer_iterations;
    ASSERT_FALSE(stage_one.report.history.empty());
    const double stage_one_final_weight =
        stage_one.report.history.back().tracking_weight;
    // 地板取阶段一末轮值的 4 倍（明确高于末轮值）
    iLQRPostStageConfig floored_config;
    floored_config.stage_two_min_tracking_weight = 4.0 * stage_one_final_weight;
    iLQRPostStage floored_post_stage(floored_config, &fixture.reference_builder,
                                    &fixture.solver, MakeVehicleParams());
    GridMap grid_map(0.1, 300, 200, Position{-15.0, -10.0}, {});
    const ESDFMap esdf_map(grid_map);
    const VehicleFootprintModel footprint_model(MakeVehicleParams(), 233, 2, 2);
    TrajectoryPoint goal;
    goal.x = 1.85;
    goal.y = 0.0;
    goal.theta = 0.0;
    const auto result = floored_post_stage.run(path, reference, stage_one, goal,
                                               esdf_map, footprint_model);
    ASSERT_TRUE(result.stage_two.has_value());
    ASSERT_FALSE(result.stage_two->report.history.empty());
    EXPECT_DOUBLE_EQ(result.stage_two->report.history.front().tracking_weight,
                     4.0 * stage_one_final_weight);
}

// 权重耗尽跳过正例：地板设到极大值使任何末轮跟踪权重都判「耗尽」，
// 阶段一候选合法即跳过阶段二，状态 SUCCESS_STAGE_ONE_ONLY
TEST(iLQRPostStageTest, SkipStageTwoWhenWeightExhausted) {
    PostStageFixture fixture;
    const Path path = BuildXPolyline({0.0, 1.0, 0.85, 1.85});
    const iLQRReference reference = BuildReference(path);
    const auto stage_one = fixture.solver.solveStageOne(reference);
    ASSERT_EQ(stage_one.report.status, ApaILQRStatus::CONVERGED)
        << "outer=" << stage_one.report.outer_iterations;
    GridMap grid_map(0.1, 300, 200, Position{-15.0, -10.0}, {});
    const ESDFMap esdf_map(grid_map);
    const VehicleFootprintModel footprint_model(MakeVehicleParams(), 233, 2, 2);
    TrajectoryPoint goal;
    goal.x = 1.85;
    goal.y = 0.0;
    goal.theta = 0.0;
    iLQRPostStageConfig skip_config;
    skip_config.skip_stage_two_when_weight_exhausted = true;
    skip_config.stage_two_min_tracking_weight = 1e9;
    iLQRPostStage skip_post_stage(skip_config, &fixture.reference_builder,
                                 &fixture.solver, MakeVehicleParams());
    const auto skipped = skip_post_stage.run(path, reference, stage_one, goal,
                                             esdf_map, footprint_model);
    EXPECT_EQ(skipped.status, iLQRPostStageStatus::SUCCESS_STAGE_ONE_ONLY);
    EXPECT_FALSE(skipped.stage_two.has_value());
    EXPECT_FALSE(skipped.used_fallback);
    EXPECT_FALSE(skipped.trajectory.empty());
}

// 权重耗尽跳过反例：地板 0（默认）时末轮权重（退火几何衰减恒正）
// 恒高于地板 → 判据不满足、继续阶段二（SUCCESS + stage_two 在场）
TEST(iLQRPostStageTest, WeightNotExhaustedRunsStageTwo) {
    PostStageFixture fixture;
    const Path path = BuildXPolyline({0.0, 1.0, 0.85, 1.85});
    const iLQRReference reference = BuildReference(path);
    const auto stage_one = fixture.solver.solveStageOne(reference);
    ASSERT_EQ(stage_one.report.status, ApaILQRStatus::CONVERGED)
        << "outer=" << stage_one.report.outer_iterations;
    GridMap grid_map(0.1, 300, 200, Position{-15.0, -10.0}, {});
    const ESDFMap esdf_map(grid_map);
    const VehicleFootprintModel footprint_model(MakeVehicleParams(), 233, 2, 2);
    TrajectoryPoint goal;
    goal.x = 1.85;
    goal.y = 0.0;
    goal.theta = 0.0;
    iLQRPostStageConfig keep_config;
    keep_config.skip_stage_two_when_weight_exhausted = true;
    iLQRPostStage keep_stage(keep_config, &fixture.reference_builder,
                            &fixture.solver, MakeVehicleParams());
    const auto kept = keep_stage.run(path, reference, stage_one, goal,
                                     esdf_map, footprint_model);
    EXPECT_EQ(kept.status, iLQRPostStageStatus::SUCCESS);
    EXPECT_TRUE(kept.stage_two.has_value());
}

// 阶段一候选不合法时不得跳过：goal 放到参考末端之外使阶段一候选
// 终点门失败（trajectory 无值），即使权重耗尽判据成立也必须进入
// 阶段二（stage_two 在场）；阶段二同样过不了终点门，最终两候选
// 均不合法 → VALIDATION_FAILED 回退（绝不输出非法候选）
TEST(iLQRPostStageTest, WeightExhaustedDoesNotSkipIllegalStageOne) {
    PostStageFixture fixture;
    const Path path = BuildXPolyline({0.0, 1.0, 0.85, 1.85});
    const iLQRReference reference = BuildReference(path);
    const auto stage_one = fixture.solver.solveStageOne(reference);
    ASSERT_EQ(stage_one.report.status, ApaILQRStatus::CONVERGED)
        << "outer=" << stage_one.report.outer_iterations;
    GridMap grid_map(0.1, 300, 200, Position{-15.0, -10.0}, {});
    const ESDFMap esdf_map(grid_map);
    const VehicleFootprintModel footprint_model(MakeVehicleParams(), 233, 2, 2);
    TrajectoryPoint goal;
    goal.x = 5.0;
    goal.y = 0.0;
    goal.theta = 0.0;
    iLQRPostStageConfig skip_config;
    skip_config.skip_stage_two_when_weight_exhausted = true;
    skip_config.stage_two_min_tracking_weight = 1e9;
    iLQRPostStage skip_post_stage(skip_config, &fixture.reference_builder,
                                 &fixture.solver, MakeVehicleParams());
    const auto result = skip_post_stage.run(path, reference, stage_one, goal,
                                            esdf_map, footprint_model);
    EXPECT_EQ(result.status, iLQRPostStageStatus::VALIDATION_FAILED);
    EXPECT_TRUE(result.used_fallback);
    EXPECT_TRUE(result.stage_two.has_value());
}

// 保留换挡场景全链路：「前进 1.0 → 倒退 2.0」——后处理恢复 2 个 maneuver，
// 阶段二门控重解后插入驻留（总时长相应增加、驻留段 |v|≤v_dwell、时间戳
// 严格单调），六项校验全过，maneuver 数不增
TEST(iLQRPostStageTest, ReversalScenarioInsertsDwellAndValidates) {
    PostStageFixture fixture;
    const Path path = BuildXPolyline({0.0, 1.0, -1.0});
    const iLQRReference reference = BuildReference(path);
    const auto stage_one = fixture.solver.solveStageOne(reference);
    ASSERT_EQ(stage_one.report.status, ApaILQRStatus::CONVERGED)
        << "outer=" << stage_one.report.outer_iterations;
    GridMap grid_map(0.1, 300, 200, Position{-15.0, -10.0}, {});
    const ESDFMap esdf_map(grid_map);
    const VehicleFootprintModel footprint_model(MakeVehicleParams(), 233, 2, 2);
    TrajectoryPoint goal;
    goal.x = -1.0;
    goal.y = 0.0;
    goal.theta = 0.0;
    const auto result = fixture.post_stage.run(path, reference, stage_one, goal,
                                               esdf_map, footprint_model);
    EXPECT_EQ(result.status, iLQRPostStageStatus::SUCCESS)
        << "failed_check=" << result.diagnostics.failed_check
        << " measured=" << result.diagnostics.measured_value
        << " threshold=" << result.diagnostics.threshold;
    EXPECT_FALSE(result.used_fallback);
    EXPECT_EQ(result.diagnostics.input_maneuver_count, 2);
    EXPECT_EQ(result.diagnostics.output_maneuver_count, 2);
    // 恰好一个接缝：驻留插入生效（时长 ≥ κ_pad·T_shift），接缝零速达标
    ASSERT_EQ(result.diagnostics.seams.size(), 1);
    const auto& seam = result.diagnostics.seams[0];
    EXPECT_GE(seam.dwell_duration, seam.t_dwell - 1e-9);
    EXPECT_GE(
        seam.t_dwell,
        fixture.post_config.kappa_pad * fixture.post_config.shift_delay - 1e-9);
    EXPECT_LE(seam.seam_speed, 0.02);
    // 总时长 = 阶段二时长 + (驻留拉伸 − 窗口时长) > 阶段二时长
    ASSERT_TRUE(result.stage_two.has_value());
    const double stage_two_duration =
        (result.stage_two->states.size() - 1) * reference.dt;
    EXPECT_GT(result.trajectory.duration(), stage_two_duration);
    // 驻留段内 |v|≤v_dwell、时间戳严格单调
    for (std::size_t i = 1; i < result.trajectory.size(); ++i) {
        EXPECT_GT(result.trajectory[i].getT(), result.trajectory[i - 1].getT());
    }
    std::cout << "[iLQR-POST] dwell=" << seam.dwell_duration
              << " t_resteer=" << seam.t_resteer
              << " duration=" << result.trajectory.duration()
              << " stage_two_duration=" << stage_two_duration << std::endl;
}

// 回退路径：两个候选的合法性门都被故意破坏（终点目标放到 100 m 外，
// 阶段二精化候选与阶段一降级候选的终点双指标必然都不过）——必须返回
// 原始 A* 路径转化的回退轨迹（绝不输出未过合法性门的轨迹），且结构化
// 诊断完整（失败项名称 + 量化值 + 阈值 + 降级原因）
TEST(iLQRPostStageTest, FallbackOnValidationFailure) {
    PostStageFixture fixture;
    const Path path = BuildXPolyline({0.0, 1.0, 0.85, 1.85});
    const iLQRReference reference = BuildReference(path);
    const auto stage_one = fixture.solver.solveStageOne(reference);
    ASSERT_EQ(stage_one.report.status, ApaILQRStatus::CONVERGED);
    GridMap grid_map(0.1, 300, 200, Position{-15.0, -10.0}, {});
    const ESDFMap esdf_map(grid_map);
    const VehicleFootprintModel footprint_model(MakeVehicleParams(), 233, 2, 2);
    TrajectoryPoint goal;
    goal.x = 100.0;
    goal.y = 100.0;
    goal.theta = 0.0;
    const auto result = fixture.post_stage.run(path, reference, stage_one, goal,
                                               esdf_map, footprint_model);
    EXPECT_EQ(result.status, iLQRPostStageStatus::VALIDATION_FAILED);
    EXPECT_TRUE(result.used_fallback);
    EXPECT_EQ(result.diagnostics.failed_check, "terminal_position");
    EXPECT_GT(result.diagnostics.measured_value, result.diagnostics.threshold);
    // 回退轨迹 = 原始 A* 路径（起止点一致、非空、时间戳完备）
    ASSERT_FALSE(result.trajectory.empty());
    EXPECT_NEAR(result.trajectory.front().x, 0.0, 1e-9);
    EXPECT_NEAR(result.trajectory.back().x, 1.85, 1e-6);
    EXPECT_TRUE(result.trajectory.back().hasT());
}

// 分级降级：阶段二门控重解预算耗尽未收敛（外层上限置零注入失败），但
// 阶段一解干净收敛——不得回退原始 A* 路径，应输出阶段一降级候选
// （阶段一解 + 修剪 + 驻留插入，过同一合法性门），状态码与诊断如实
// 反映降级（绝不伪装成阶段二完全成功）
TEST(iLQRPostStageTest, StageTwoFailureOutputsStageOneCandidate) {
    // 独立装配：阶段二外层预算置零，门控重解必然 MAX_OUTER_ITERATIONS
    const ApaILQRSolverConfig zero_budget_config = [] {
        ApaILQRSolverConfig config = MakeSyntheticConfig();
        config.stage_two_max_outer_iterations = 0;
        return config;
    }();
    const BicycleDynamics dynamics(kWheelbase);
    const iLQRCostEvaluator cost_evaluator(zero_budget_config.cost, nullptr);
    ApaILQRSolver solver(zero_budget_config, &dynamics, &cost_evaluator);
    const iLQRReferenceBuilder reference_builder(iLQRReferenceBuilderConfig{},
                                                MakeVehicleParams());
    iLQRPostStage post_stage(iLQRPostStageConfig{}, &reference_builder, &solver,
                            MakeVehicleParams());
    const Path path = BuildXPolyline({0.0, 1.0, -1.0});
    const iLQRReference reference = reference_builder.build(path);
    const auto stage_one = solver.solveStageOne(reference);
    ASSERT_EQ(stage_one.report.status, ApaILQRStatus::CONVERGED);
    GridMap grid_map(0.1, 300, 200, Position{-15.0, -10.0}, {});
    const ESDFMap esdf_map(grid_map);
    const VehicleFootprintModel footprint_model(MakeVehicleParams(), 233, 2, 2);
    TrajectoryPoint goal;
    goal.x = -1.0;
    goal.y = 0.0;
    goal.theta = 0.0;
    const auto result = post_stage.run(path, reference, stage_one, goal,
                                       esdf_map, footprint_model);
    EXPECT_EQ(result.status, iLQRPostStageStatus::SUCCESS_STAGE_ONE_ONLY)
        << "failed_check=" << result.diagnostics.failed_check
        << " measured=" << result.diagnostics.measured_value;
    // 降级不伪装：降级原因如实记录，used_fallback 保持 false（输出的是
    // 优化成果而非原始路径），合法性门量测全过
    EXPECT_FALSE(result.used_fallback);
    EXPECT_EQ(result.diagnostics.degraded_reason, "stage_two_convergence");
    ASSERT_TRUE(result.stage_two.has_value());
    for (const auto& check : result.diagnostics.gate_checks) {
        EXPECT_TRUE(check.passed) << check.name;
    }
    // 降级输出语义：阶段一解 + 驻留插入——时间戳严格单调、终点静止、
    // 恰好一个接缝（保留换挡），物理方向段数不增
    ASSERT_FALSE(result.trajectory.empty());
    for (std::size_t i = 1; i < result.trajectory.size(); ++i) {
        EXPECT_GT(result.trajectory[i].getT(), result.trajectory[i - 1].getT());
    }
    EXPECT_NEAR(result.trajectory.back().getV(), 0.0, 0.05);
    ASSERT_EQ(result.diagnostics.seams.size(), 1);
    EXPECT_EQ(result.diagnostics.input_maneuver_count, 2);
    EXPECT_LE(result.diagnostics.output_maneuver_count, 2);
}

// 分级降级的兜底边界：阶段二未收敛（外层上限置零）且阶段一降级候选也
// 不过合法性门（终点目标放到 100 m 外）——两级候选依次失败后回退原始
// A* 路径，诊断同时携带降级原因与降级候选的首个失败门项
TEST(iLQRPostStageTest, StageOneCandidateAlsoIllegalFallsBack) {
    const ApaILQRSolverConfig zero_budget_config = [] {
        ApaILQRSolverConfig config = MakeSyntheticConfig();
        config.stage_two_max_outer_iterations = 0;
        return config;
    }();
    const BicycleDynamics dynamics(kWheelbase);
    const iLQRCostEvaluator cost_evaluator(zero_budget_config.cost, nullptr);
    ApaILQRSolver solver(zero_budget_config, &dynamics, &cost_evaluator);
    const iLQRReferenceBuilder reference_builder(iLQRReferenceBuilderConfig{},
                                                MakeVehicleParams());
    iLQRPostStage post_stage(iLQRPostStageConfig{}, &reference_builder, &solver,
                            MakeVehicleParams());
    const Path path = BuildXPolyline({0.0, 1.0, 0.85, 1.85});
    const iLQRReference reference = reference_builder.build(path);
    const auto stage_one = solver.solveStageOne(reference);
    ASSERT_EQ(stage_one.report.status, ApaILQRStatus::CONVERGED);
    GridMap grid_map(0.1, 300, 200, Position{-15.0, -10.0}, {});
    const ESDFMap esdf_map(grid_map);
    const VehicleFootprintModel footprint_model(MakeVehicleParams(), 233, 2, 2);
    TrajectoryPoint goal;
    goal.x = 100.0;
    goal.y = 100.0;
    goal.theta = 0.0;
    const auto result = post_stage.run(path, reference, stage_one, goal,
                                       esdf_map, footprint_model);
    EXPECT_EQ(result.status, iLQRPostStageStatus::VALIDATION_FAILED);
    EXPECT_TRUE(result.used_fallback);
    EXPECT_EQ(result.diagnostics.degraded_reason, "stage_two_convergence");
    EXPECT_EQ(result.diagnostics.failed_check, "terminal_position");
    ASSERT_FALSE(result.trajectory.empty());
    EXPECT_NEAR(result.trajectory.back().x, 1.85, 1e-6);
}

// 校验分层：质量指标全面超标（窗端 ω 0.56 超包络、接缝速度 0.1 超阈、
// 窗内速度 0.2 超帽、驻留时长不足、maneuver 数倒挂、控制盒过冲 0.5）但
// 合法性门全清的候选必须放行——质量指标全量记录（含逐项 passed 标记）
// 供方案比较，不再作为回退触发条件
TEST(iLQRValidationLayerTest, MetricViolationsDoNotFailGate) {
    PostStageFixture fixture;
    // 先跑通保留换挡场景，取得合法输出轨迹与阶段二结果作为校验输入基底
    const Path path = BuildXPolyline({0.0, 1.0, -1.0});
    const iLQRReference reference = BuildReference(path);
    const auto stage_one = fixture.solver.solveStageOne(reference);
    ASSERT_EQ(stage_one.report.status, ApaILQRStatus::CONVERGED);
    GridMap grid_map(0.1, 300, 200, Position{-15.0, -10.0}, {});
    const ESDFMap esdf_map(grid_map);
    const VehicleFootprintModel footprint_model(MakeVehicleParams(), 233, 2, 2);
    TrajectoryPoint goal;
    goal.x = -1.0;
    const auto good = fixture.post_stage.run(path, reference, stage_one, goal,
                                             esdf_map, footprint_model);
    ASSERT_EQ(good.status, iLQRPostStageStatus::SUCCESS);
    ValidateOutputAccessor accessor(fixture.post_config,
                                    &fixture.reference_builder, &fixture.solver,
                                    MakeVehicleParams());
    iLQRPostStageDiagnostics diag;
    // maneuver 数倒挂注入：输入 1 段、输出 2 段——旧判据下必然回退（且
    // 回退目标的 maneuver 数只会更多，逻辑倒挂），分层后仅作质量记录
    diag.input_maneuver_count = 1;
    diag.seams.push_back(iLQRSeamReport{/*seam_index=*/1,
                                       /*delta_delta=*/0.3,
                                       /*t_resteer=*/0.5,
                                       /*t_dwell=*/0.6,
                                       /*dwell_duration=*/0.5,
                                       /*seam_speed=*/0.1,
                                       /*window_max_speed=*/0.2,
                                       /*window_end_omega=*/0.56});
    // 控制盒过冲注入：|j| 超盒 0.5（> 0.3 参考阈值）——j/η 不进入输出
    // 轨迹契约，物理可执行性由状态幅值门独立保证
    auto doctored_controls = good.stage_two->controls;
    doctored_controls[0](ILQR_IDX_JERK) =
        fixture.solver_config.inner.jerk_max + 0.5;
    EXPECT_TRUE(accessor.validateOutput(good.trajectory, good.stage_two->states,
                                        doctored_controls, path.length(), goal,
                                        esdf_map, footprint_model, &diag));
    EXPECT_TRUE(diag.failed_check.empty());
    // 质量指标全量记录：控制过冲/接缝四项/manuever 数/长度比逐项在场，
    // 注入的超标项 passed=false 但未否决输出
    std::vector<std::string> metric_names;
    metric_names.reserve(diag.metric_checks.size());
    for (const auto& check : diag.metric_checks) {
        metric_names.push_back(check.name);
    }
    EXPECT_THAT(metric_names,
                ::testing::UnorderedElementsAre(
                    "control_amplitude", "seam_zero_speed", "dwell_duration",
                    "dwell_window_speed", "dwell_window_end_omega",
                    "maneuver_count", "length_ratio"));
    const auto find_metric = [&diag](const std::string& name) {
        return std::find_if(diag.metric_checks.begin(),
                            diag.metric_checks.end(),
                            [&name](const auto& c) { return c.name == name; });
    };
    EXPECT_FALSE(find_metric("control_amplitude")->passed);
    EXPECT_FALSE(find_metric("seam_zero_speed")->passed);
    EXPECT_FALSE(find_metric("dwell_duration")->passed);
    EXPECT_FALSE(find_metric("dwell_window_speed")->passed);
    EXPECT_FALSE(find_metric("dwell_window_end_omega")->passed);
    EXPECT_FALSE(find_metric("maneuver_count")->passed);
    EXPECT_TRUE(find_metric("length_ratio")->passed);
    // 死判据已移除：任何一层都不得再出现 dwell_resteer（构造上
    // t_dwell=κ_pad·max(t_resteer,T_shift) 且 κ_pad>=1，该判据恒成立）
    for (const auto& check : diag.gate_checks) {
        EXPECT_NE(check.name, "dwell_resteer");
    }
    for (const auto& check : diag.metric_checks) {
        EXPECT_NE(check.name, "dwell_resteer");
    }
}

// 校验分层：合法性门失败（终点位置 100 m 外）必须拦截，且门量测不
// 短路——碰撞/终点双指标/运动学四残差/状态幅值八项全量在场，失败项
// 为首个未过门项，量化值/阈值正确
TEST(iLQRValidationLayerTest, GateFailureRecordedAlongsideAllMeasurements) {
    PostStageFixture fixture;
    const Path path = BuildXPolyline({0.0, 1.0, -1.0});
    const iLQRReference reference = BuildReference(path);
    const auto stage_one = fixture.solver.solveStageOne(reference);
    ASSERT_EQ(stage_one.report.status, ApaILQRStatus::CONVERGED);
    GridMap grid_map(0.1, 300, 200, Position{-15.0, -10.0}, {});
    const ESDFMap esdf_map(grid_map);
    const VehicleFootprintModel footprint_model(MakeVehicleParams(), 233, 2, 2);
    TrajectoryPoint goal;
    goal.x = -1.0;
    const auto good = fixture.post_stage.run(path, reference, stage_one, goal,
                                             esdf_map, footprint_model);
    ASSERT_EQ(good.status, iLQRPostStageStatus::SUCCESS);
    ValidateOutputAccessor accessor(fixture.post_config,
                                    &fixture.reference_builder, &fixture.solver,
                                    MakeVehicleParams());
    iLQRPostStageDiagnostics diag;
    diag.input_maneuver_count = 2;
    TrajectoryPoint far_goal;
    far_goal.x = 100.0;
    far_goal.y = 100.0;
    EXPECT_FALSE(accessor.validateOutput(
        good.trajectory, good.stage_two->states, good.stage_two->controls,
        path.length(), far_goal, esdf_map, footprint_model, &diag));
    EXPECT_EQ(diag.failed_check, "terminal_position");
    EXPECT_GT(diag.measured_value, diag.threshold);
    std::vector<std::string> gate_names;
    gate_names.reserve(diag.gate_checks.size());
    for (const auto& check : diag.gate_checks) {
        gate_names.push_back(check.name);
    }
    EXPECT_THAT(gate_names,
                ::testing::UnorderedElementsAre(
                    "collision", "terminal_position", "terminal_heading",
                    "kinematic_position", "kinematic_heading",
                    "kinematic_velocity", "kinematic_steer", "amplitude",
                    "amplitude_delta", "amplitude_omega"));
}

// 合法性门：状态幅值（v/a/δ/ω 输出轨迹契约直接消费的量）超标必须拦截——
// 注入 |v| 超过 v_max + amplitude_check_tol 的状态，门判失败且诊断指向
// amplitude；控制量同量级超标（上例）则不否决，两项归类不容互换
TEST(iLQRValidationLayerTest, StateAmplitudeViolationFailsGate) {
    PostStageFixture fixture;
    const Path path = BuildXPolyline({0.0, 1.0, -1.0});
    const iLQRReference reference = BuildReference(path);
    const auto stage_one = fixture.solver.solveStageOne(reference);
    ASSERT_EQ(stage_one.report.status, ApaILQRStatus::CONVERGED);
    GridMap grid_map(0.1, 300, 200, Position{-15.0, -10.0}, {});
    const ESDFMap esdf_map(grid_map);
    const VehicleFootprintModel footprint_model(MakeVehicleParams(), 233, 2, 2);
    TrajectoryPoint goal;
    goal.x = -1.0;
    const auto good = fixture.post_stage.run(path, reference, stage_one, goal,
                                             esdf_map, footprint_model);
    ASSERT_EQ(good.status, iLQRPostStageStatus::SUCCESS);
    ValidateOutputAccessor accessor(fixture.post_config,
                                    &fixture.reference_builder, &fixture.solver,
                                    MakeVehicleParams());
    iLQRPostStageDiagnostics diag;
    diag.input_maneuver_count = 2;
    auto doctored_states = good.stage_two->states;
    doctored_states[0](ILQR_IDX_V) = fixture.solver_config.cost.v_max +
                                    fixture.post_config.amplitude_check_tol +
                                    0.01;
    EXPECT_FALSE(accessor.validateOutput(
        good.trajectory, doctored_states, good.stage_two->controls,
        path.length(), goal, esdf_map, footprint_model, &diag));
    EXPECT_EQ(diag.failed_check, "amplitude");
    EXPECT_DOUBLE_EQ(diag.threshold, fixture.post_config.amplitude_check_tol);
}

// 门项边界值：终点位置误差在阈值内侧（−1e-6 m）必须放行、外侧
//（+1e-6 m）必须拦截——钉住"不得放宽"的边界语义，防止未来改动悄悄
// 放宽门限。（不构造"恰好等于阈值"：浮点下 (x+tol)−x 与 tol 有 1 ulp
// 量级偏差，恰等情形不具备可复现性，内侧/外侧 ±1e-6 是稳健的边界语义）
TEST(iLQRValidationLayerTest, TerminalPositionBoundaryIsInclusive) {
    PostStageFixture fixture;
    const Path path = BuildXPolyline({0.0, 1.0, -1.0});
    const iLQRReference reference = BuildReference(path);
    const auto stage_one = fixture.solver.solveStageOne(reference);
    ASSERT_EQ(stage_one.report.status, ApaILQRStatus::CONVERGED);
    GridMap grid_map(0.1, 300, 200, Position{-15.0, -10.0}, {});
    const ESDFMap esdf_map(grid_map);
    const VehicleFootprintModel footprint_model(MakeVehicleParams(), 233, 2, 2);
    TrajectoryPoint goal;
    goal.x = -1.0;
    const auto good = fixture.post_stage.run(path, reference, stage_one, goal,
                                             esdf_map, footprint_model);
    ASSERT_EQ(good.status, iLQRPostStageStatus::SUCCESS);
    ValidateOutputAccessor accessor(fixture.post_config,
                                    &fixture.reference_builder, &fixture.solver,
                                    MakeVehicleParams());
    // 阈值内侧：goal 置于输出终点正 x 方向 阈值−1e-6 m 处 → 放行
    TrajectoryPoint boundary_goal;
    boundary_goal.x =
        good.trajectory.back().x +
        fixture.post_config.validation.max_terminal_position_error - 1e-6;
    boundary_goal.y = good.trajectory.back().y;
    boundary_goal.theta = good.trajectory.back().theta;
    iLQRPostStageDiagnostics diag_inside;
    diag_inside.input_maneuver_count = 2;
    EXPECT_TRUE(accessor.validateOutput(
        good.trajectory, good.stage_two->states, good.stage_two->controls,
        path.length(), boundary_goal, esdf_map, footprint_model, &diag_inside));
    // 阈值外侧（+1e-6 m）→ 拦截且诊断指向 terminal_position
    boundary_goal.x =
        good.trajectory.back().x +
        fixture.post_config.validation.max_terminal_position_error + 1e-6;
    iLQRPostStageDiagnostics diag_outside;
    diag_outside.input_maneuver_count = 2;
    EXPECT_FALSE(accessor.validateOutput(
        good.trajectory, good.stage_two->states, good.stage_two->controls,
        path.length(), boundary_goal, esdf_map, footprint_model,
        &diag_outside));
    EXPECT_EQ(diag_outside.failed_check, "terminal_position");
}

// 门项边界值：状态幅值违反度在 AL 平衡容差内侧（−1e-6）必须放行且量测
// 不超过容差（拦截侧已由 StateAmplitudeViolationFailsGate 覆盖）——确认
// 放行来自边界语义而非其他裕量
TEST(iLQRValidationLayerTest, AmplitudeBoundaryIsInclusive) {
    PostStageFixture fixture;
    const Path path = BuildXPolyline({0.0, 1.0, -1.0});
    const iLQRReference reference = BuildReference(path);
    const auto stage_one = fixture.solver.solveStageOne(reference);
    ASSERT_EQ(stage_one.report.status, ApaILQRStatus::CONVERGED);
    GridMap grid_map(0.1, 300, 200, Position{-15.0, -10.0}, {});
    const ESDFMap esdf_map(grid_map);
    const VehicleFootprintModel footprint_model(MakeVehicleParams(), 233, 2, 2);
    TrajectoryPoint goal;
    goal.x = -1.0;
    const auto good = fixture.post_stage.run(path, reference, stage_one, goal,
                                             esdf_map, footprint_model);
    ASSERT_EQ(good.status, iLQRPostStageStatus::SUCCESS);
    ValidateOutputAccessor accessor(fixture.post_config,
                                    &fixture.reference_builder, &fixture.solver,
                                    MakeVehicleParams());
    auto doctored_states = good.stage_two->states;
    doctored_states[0](ILQR_IDX_V) = fixture.solver_config.cost.v_max +
                                    fixture.post_config.amplitude_check_tol -
                                    1e-6;
    iLQRPostStageDiagnostics diag;
    diag.input_maneuver_count = 2;
    EXPECT_TRUE(accessor.validateOutput(
        good.trajectory, doctored_states, good.stage_two->controls,
        path.length(), goal, esdf_map, footprint_model, &diag));
    const auto amplitude_check =
        std::find_if(diag.gate_checks.begin(), diag.gate_checks.end(),
                     [](const auto& c) { return c.name == "amplitude"; });
    ASSERT_NE(amplitude_check, diag.gate_checks.end());
    EXPECT_TRUE(amplitude_check->passed);
    EXPECT_LE(amplitude_check->measured,
              fixture.post_config.amplitude_check_tol);
    EXPECT_GT(amplitude_check->measured, 0.0);
}

// δ/ω 按量相对容差：δ 在低速/驻留点（|v|<v_dwell）的超限不产生曲率、
// 豁免复检（与「停稳后前轮转角无物理要求」同一设计语义）；行驶点的
// δ 超限按相对容差 0.2% 判定；ω 全点复检。原统一绝对容差在 δ 量纲上
// 等价于允许 κ=113%·κ_max（校验门比验收门宽），按量分设后口径对齐
TEST(iLQRValidationLayerTest, DeltaCheckedOnlyAtDrivingPointsWithRelTol) {
    PostStageFixture fixture;
    const Path path = BuildXPolyline({0.0, 1.0, -1.0});
    const iLQRReference reference = BuildReference(path);
    const auto stage_one = fixture.solver.solveStageOne(reference);
    ASSERT_EQ(stage_one.report.status, ApaILQRStatus::CONVERGED);
    GridMap grid_map(0.1, 300, 200, Position{-15.0, -10.0}, {});
    const ESDFMap esdf_map(grid_map);
    const VehicleFootprintModel footprint_model(MakeVehicleParams(), 233, 2, 2);
    TrajectoryPoint goal;
    goal.x = -1.0;
    const auto good = fixture.post_stage.run(path, reference, stage_one, goal,
                                             esdf_map, footprint_model);
    ASSERT_EQ(good.status, iLQRPostStageStatus::SUCCESS);
    ValidateOutputAccessor accessor(fixture.post_config,
                                    &fixture.reference_builder, &fixture.solver,
                                    MakeVehicleParams());
    const double delta_max = fixture.solver_config.cost.delta_max;
    // 驻留点（|v|<v_dwell）δ 超限 50%：豁免，门应通过
    {
        auto states = good.stage_two->states;
        states[0](ILQR_IDX_V) = 0.0;
        states[0](ILQR_IDX_DELTA) = 1.5 * delta_max;
        iLQRPostStageDiagnostics diag;
        diag.input_maneuver_count = 2;
        EXPECT_TRUE(accessor.validateOutput(
            good.trajectory, states, good.stage_two->controls, path.length(),
            goal, esdf_map, footprint_model, &diag));
        const auto check = std::find_if(
            diag.gate_checks.begin(), diag.gate_checks.end(),
            [](const auto& c) { return c.name == "amplitude_delta"; });
        ASSERT_NE(check, diag.gate_checks.end());
        EXPECT_TRUE(check->passed);
    }
    // 行驶点（|v|≥v_dwell）δ 超限 3%（>2% 相对容差）：拦截并指向
    // amplitude_delta
    {
        auto states = good.stage_two->states;
        states[0](ILQR_IDX_V) = fixture.post_config.v_dwell;
        states[0](ILQR_IDX_DELTA) = 1.03 * delta_max;
        iLQRPostStageDiagnostics diag;
        diag.input_maneuver_count = 2;
        EXPECT_FALSE(accessor.validateOutput(
            good.trajectory, states, good.stage_two->controls, path.length(),
            goal, esdf_map, footprint_model, &diag));
        EXPECT_EQ(diag.failed_check, "amplitude_delta");
        EXPECT_DOUBLE_EQ(diag.threshold,
                         fixture.post_config.amplitude_check_rel_tol);
    }
    // 行驶点 δ 在容差内侧（+1%）：放行
    {
        auto states = good.stage_two->states;
        states[0](ILQR_IDX_V) = fixture.post_config.v_dwell;
        states[0](ILQR_IDX_DELTA) = 1.01 * delta_max;
        iLQRPostStageDiagnostics diag;
        diag.input_maneuver_count = 2;
        EXPECT_TRUE(accessor.validateOutput(
            good.trajectory, states, good.stage_two->controls, path.length(),
            goal, esdf_map, footprint_model, &diag));
    }
}

// ω 按量相对容差：全点复检（执行器速率在驻留转向中同样工作），超限
// 3% 拦截并指向 amplitude_omega
TEST(iLQRValidationLayerTest, OmegaViolationFailsGateWithRelTol) {
    PostStageFixture fixture;
    const Path path = BuildXPolyline({0.0, 1.0, -1.0});
    const iLQRReference reference = BuildReference(path);
    const auto stage_one = fixture.solver.solveStageOne(reference);
    ASSERT_EQ(stage_one.report.status, ApaILQRStatus::CONVERGED);
    GridMap grid_map(0.1, 300, 200, Position{-15.0, -10.0}, {});
    const ESDFMap esdf_map(grid_map);
    const VehicleFootprintModel footprint_model(MakeVehicleParams(), 233, 2, 2);
    TrajectoryPoint goal;
    goal.x = -1.0;
    const auto good = fixture.post_stage.run(path, reference, stage_one, goal,
                                             esdf_map, footprint_model);
    ASSERT_EQ(good.status, iLQRPostStageStatus::SUCCESS);
    ValidateOutputAccessor accessor(fixture.post_config,
                                    &fixture.reference_builder, &fixture.solver,
                                    MakeVehicleParams());
    auto states = good.stage_two->states;
    states[0](ILQR_IDX_OMEGA) = 1.03 * fixture.solver_config.cost.omega_max;
    iLQRPostStageDiagnostics diag;
    diag.input_maneuver_count = 2;
    EXPECT_FALSE(accessor.validateOutput(
        good.trajectory, states, good.stage_two->controls, path.length(), goal,
        esdf_map, footprint_model, &diag));
    EXPECT_EQ(diag.failed_check, "amplitude_omega");
    EXPECT_DOUBLE_EQ(diag.threshold,
                     fixture.post_config.amplitude_check_rel_tol);
    // ω 在容差内侧（+1%）：放行
    auto ok_states = good.stage_two->states;
    ok_states[0](ILQR_IDX_OMEGA) = 1.01 * fixture.solver_config.cost.omega_max;
    iLQRPostStageDiagnostics ok_diag;
    ok_diag.input_maneuver_count = 2;
    EXPECT_TRUE(accessor.validateOutput(
        good.trajectory, ok_states, good.stage_two->controls, path.length(),
        goal, esdf_map, footprint_model, &ok_diag));
}

// 回退路径：阶段一解中混入 PIVOT 微游程（合成注入：微弧长 + 朝向跳变
// 0.3 rad，模拟携带未愈合缺陷的解）——修剪触发失败语义，直接回退原始
// 路径，不再进入任何候选评估
TEST(iLQRPostStageTest, PivotScenarioFallsBack) {
    PostStageFixture fixture;
    const Path path = BuildXPolyline({0.0, 2.0});
    // 合成阶段一结果：前进 2 m → PIVOT 微游程（θ 跳变 0.3，δ 恒 0）→ 前进继续
    std::vector<ProfileEntry> profile;
    AppendRun(&profile, 0.05, 41, 0.5, 0.0);
    profile.push_back({2.00, -0.03, 0.0, 0.0});
    profile.push_back({1.99, -0.03, 0.0, 0.3});
    profile.push_back({2.005, 0.5, 0.0, 0.3});
    AppendRun(&profile, 0.05, 40, 0.5, 0.0);
    ApaILQRStageOneResult stage_one;
    stage_one.states = MakeStates(profile);
    stage_one.controls.assign(profile.size() - 1, iLQRControl::Zero());
    stage_one.report.status = ApaILQRStatus::CONVERGED;
    iLQRReference reference;
    reference.maneuvers.resize(3);
    GridMap grid_map(0.1, 300, 200, Position{-15.0, -10.0}, {});
    const ESDFMap esdf_map(grid_map);
    const VehicleFootprintModel footprint_model(MakeVehicleParams(), 233, 2, 2);
    TrajectoryPoint goal;
    goal.x = 2.0;
    const auto result = fixture.post_stage.run(path, reference, stage_one, goal,
                                               esdf_map, footprint_model);
    EXPECT_EQ(result.status, iLQRPostStageStatus::PIVOT_DETECTED);
    EXPECT_TRUE(result.used_fallback);
    EXPECT_FALSE(result.stage_two.has_value());
    EXPECT_EQ(result.diagnostics.failed_check, "pivot");
    ASSERT_FALSE(result.trajectory.empty());
}

// 回退路径：阶段一未收敛（外层迭代耗尽）——后处理不得带病进入修剪与
// 阶段二，直接回退原始路径并如实记录失败阶段
TEST(iLQRPostStageTest, StageOneFailureFallsBack) {
    PostStageFixture fixture;
    const Path path = BuildXPolyline({0.0, 2.0});
    ApaILQRStageOneResult stage_one;
    stage_one.states = MakeStates({{0.0, 0.5, 0.0}, {0.05, 0.5, 0.0}});
    stage_one.controls.assign(1, iLQRControl::Zero());
    stage_one.report.status = ApaILQRStatus::MAX_OUTER_ITERATIONS;
    iLQRReference reference;
    reference.maneuvers.resize(1);
    GridMap grid_map(0.1, 300, 200, Position{-15.0, -10.0}, {});
    const ESDFMap esdf_map(grid_map);
    const VehicleFootprintModel footprint_model(MakeVehicleParams(), 233, 2, 2);
    TrajectoryPoint goal;
    const auto result = fixture.post_stage.run(path, reference, stage_one, goal,
                                               esdf_map, footprint_model);
    EXPECT_EQ(result.status, iLQRPostStageStatus::STAGE_ONE_NOT_CONVERGED);
    EXPECT_TRUE(result.used_fallback);
    EXPECT_EQ(result.diagnostics.failed_check, "stage_one_convergence");
    ASSERT_FALSE(result.trajectory.empty());
}

// 回退路径：阶段一解退化为全长 0.02 m 的蠕动轨迹（单游程），修剪后路径
// 总长不足一个重采样间距（0.05 m），无法构建阶段二参考——直接回退并
// 给出精细化诊断（弧长子情形：实测 0.02 m vs 阈值 0.05 m）
TEST(iLQRPostStageTest, DegeneratePrunedPathFallsBack) {
    PostStageFixture fixture;
    const Path path = BuildXPolyline({0.0, 2.0});
    ApaILQRStageOneResult stage_one;
    stage_one.states =
        MakeStates({{0.0, 0.03, 0.0}, {0.01, 0.03, 0.0}, {0.02, 0.03, 0.0}});
    stage_one.controls.assign(2, iLQRControl::Zero());
    stage_one.report.status = ApaILQRStatus::CONVERGED;
    iLQRReference reference;
    reference.maneuvers.resize(1);
    GridMap grid_map(0.1, 300, 200, Position{-15.0, -10.0}, {});
    const ESDFMap esdf_map(grid_map);
    const VehicleFootprintModel footprint_model(MakeVehicleParams(), 233, 2, 2);
    TrajectoryPoint goal;
    const auto result = fixture.post_stage.run(path, reference, stage_one, goal,
                                               esdf_map, footprint_model);
    EXPECT_EQ(result.status, iLQRPostStageStatus::PRUNED_PATH_DEGENERATE);
    EXPECT_TRUE(result.used_fallback);
    EXPECT_EQ(result.diagnostics.failed_check, "pruned_length");
    EXPECT_NEAR(result.diagnostics.measured_value, 0.02, 1e-9);
    EXPECT_NEAR(result.diagnostics.threshold, 0.05, 1e-9);
    ASSERT_FALSE(result.trajectory.empty());
}

}  // namespace
}  // namespace apa_post_processor
