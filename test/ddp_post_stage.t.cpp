#include "core/DDP/ddp_post_stage.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <vector>

#include "core/DDP/apa_ddp_solver.h"
#include "core/DDP/ddp_cost.h"
#include "core/DDP/ddp_reference_builder.h"
#include "core/DDP/esdf_constraint.h"
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

// 测试用车辆参数：与既有 DDP 组件单测一致的五参数构造
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
DdpReference BuildReference(const Path& path) {
    return DdpReferenceBuilder(DdpReferenceBuilderConfig{}, MakeVehicleParams())
        .build(path);
}

// 合成小尺度场景的求解配置：μ_min 降到 1.0 防小代价量级下 μ⁰ 标定被下限
// clip 到过强罚权重（与既有编排器单测同一约定，生产默认不受影响）
ApaDdpSolverConfig MakeSyntheticConfig() {
    ApaDdpSolverConfig config;
    config.outer.mu_min = 1.0;
    return config;
}

// 合成状态剖面的一条记录：x 位置、纵向速度、前轮转角（y=θ=a=ω=0）
struct ProfileEntry {
    double x;
    double v;
    double delta;
};

// 向剖面末尾追加 count 个均匀点：x 按 dx 步进，v/δ 恒定
void AppendRun(std::vector<ProfileEntry>* profile, double dx, int count,
               double v, double delta) {
    for (int i = 0; i < count; ++i) {
        const double x = profile->empty() ? 0.0 : profile->back().x + dx;
        profile->push_back({x, v, delta});
    }
}

// 由显式剖面构造合成状态轨迹
DdpAlignedVec<DdpState> MakeStates(const std::vector<ProfileEntry>& profile) {
    DdpAlignedVec<DdpState> states;
    states.reserve(profile.size());
    for (const auto& entry : profile) {
        DdpState state = DdpState::Zero();
        state(DDP_IDX_X) = entry.x;
        state(DDP_IDX_V) = entry.v;
        state(DDP_IDX_DELTA) = entry.delta;
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
          reference_builder(DdpReferenceBuilderConfig{}, MakeVehicleParams()),
          post_stage(post_config, &reference_builder, &solver,
                     MakeVehicleParams()) {}
    ApaDdpSolverConfig solver_config = MakeSyntheticConfig();
    DdpPostStageConfig post_config;
    BicycleDynamics dynamics;
    DdpCostEvaluator cost_evaluator;
    ApaDdpSolver solver;
    DdpReferenceBuilder reference_builder;
    DdpPostStage post_stage;
};

// ==================== T_resteer 双积分 bang-bang 公式 ====================

// 三角剖面分支（Δδ 小、ω 不饱和）：取 Δδ=0.09（< ω²/η=0.25），
// 解析解 T=2√(Δδ/η)=2·√0.09=0.6 s，与公式逐位对拍
TEST(DdpResteerTimeTest, TriangleBranchMatchesAnalytic) {
    const double t = DdpPostStage::ComputeResteerTime(0.09, 0.5, 1.0);
    EXPECT_NEAR(t, 0.6, 1e-12);
}

// 梯形剖面分支（Δδ 大、ω 饱和）：取 Δδ=1.0（> 0.25），
// 解析解 T=Δδ/ω+ω/η=1.0/0.5+0.5/1.0=2.5 s，与公式逐位对拍
TEST(DdpResteerTimeTest, TrapezoidBranchMatchesAnalytic) {
    const double t = DdpPostStage::ComputeResteerTime(1.0, 0.5, 1.0);
    EXPECT_NEAR(t, 2.5, 1e-12);
}

// 分支切换点连续性：Δδ*=ω²/η=0.25 处三角公式 2√0.25=1.0 与梯形公式
// 0.25/0.5+0.5=1.0 相等；切换点两侧无限趋近时函数值必须无缝衔接
TEST(DdpResteerTimeTest, BranchSwitchIsContinuous) {
    const double switch_point = 0.5 * 0.5 / 1.0;
    const double below =
        DdpPostStage::ComputeResteerTime(switch_point - 1e-12, 0.5, 1.0);
    const double above =
        DdpPostStage::ComputeResteerTime(switch_point + 1e-12, 0.5, 1.0);
    EXPECT_NEAR(below, 1.0, 1e-6);
    EXPECT_NEAR(below, above, 1e-6);
    // 单调性：Δδ 越大所需时间越长（两分支各自与跨分支均成立）
    EXPECT_LT(DdpPostStage::ComputeResteerTime(0.04, 0.5, 1.0),
              DdpPostStage::ComputeResteerTime(0.16, 0.5, 1.0));
    EXPECT_LT(DdpPostStage::ComputeResteerTime(0.5, 0.5, 1.0),
              DdpPostStage::ComputeResteerTime(0.8, 0.5, 1.0));
    // 非法参数拒绝：Δδ<0、执行器限值非正
    EXPECT_THROW(DdpPostStage::ComputeResteerTime(-0.1, 0.5, 1.0),
                 std::invalid_argument);
    EXPECT_THROW(DdpPostStage::ComputeResteerTime(0.1, 0.0, 1.0),
                 std::invalid_argument);
    EXPECT_THROW(DdpPostStage::ComputeResteerTime(0.1, 0.5, -1.0),
                 std::invalid_argument);
}

// ==================== 带滞回符号游程分析 ====================

// 滞回过滤：v 在 ±ε_v（0.02 m/s）内抖动（融化残留的速度涟漪）不产生伪
// cusp——全程 |v|<ε_v 的抖动段不承诺新符号，整条轨迹只恢复出 1 个游程
TEST(DdpSignRunTest, RippleWithinHysteresisProducesNoCusp) {
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
TEST(DdpSignRunTest, TrueReversalDetectedWithSharedBoundary) {
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
TEST(DdpSignRunTest, UndecidedSamplesNeverCommit) {
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
TEST(DdpTopologyPruneTest, OppositeDirectionNeighborsNotMerged) {
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
TEST(DdpTopologyPruneTest, SameDirectionNeighborsMergedAfterMelt) {
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
TEST(DdpTopologyPruneTest, FirstAndLastManeuverProtected) {
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
              fixture.post_config.cleanup.min_arc_length);
    EXPECT_LT(maneuvers.back().length(),
              fixture.post_config.cleanup.min_arc_length);
    EXPECT_TRUE(fixture.post_stage.pruneManeuvers(&maneuvers));
    const Path pruned = ReconstructPath(maneuvers);
    ASSERT_EQ(pruned.numManeuvers(), 3);
    EXPECT_EQ(pruned.getManeuvers()[0].direction, Direction::FORWARD);
    EXPECT_EQ(pruned.getManeuvers()[1].direction, Direction::BACKWARD);
    EXPECT_EQ(pruned.getManeuvers()[2].direction, Direction::FORWARD);
}

// 红线：PIVOT 触发失败语义——中间微游程 |Δs|<0.05 但首尾前轮转角差
// 0.3 rad（>0.1 rad 阈值），被判为原地打轮式微动；默认车辆无钟摆泊车
// 能力，修剪必须上报失败（返回 false）而非带病输出
TEST(DdpTopologyPruneTest, PivotTriggersFailureSemantics) {
    PostStageFixture fixture;
    std::vector<ProfileEntry> profile;
    AppendRun(&profile, 0.05, 41, 0.5, 0.0);
    // 微游程起点 δ=0.3、出边界 δ=0.0：|Δδ|=0.3 > 0.1 → PIVOT
    profile.push_back({2.00, -0.03, 0.3});
    profile.push_back({1.99, -0.03, 0.3});
    profile.push_back({2.005, 0.5, 0.0});
    AppendRun(&profile, 0.05, 40, 0.5, 0.0);
    const auto states = MakeStates(profile);
    auto maneuvers = fixture.post_stage.buildManeuvers(
        states, fixture.post_stage.analyzeSignRuns(states));
    ASSERT_EQ(maneuvers.size(), 3);
    EXPECT_FALSE(fixture.post_stage.pruneManeuvers(&maneuvers));
}

// ==================== 门控计划构建 ====================

// 门控约束方向（易犯符号错误，两个方向都测）：前进 maneuver 的符号门为
// +1（约束 −v≤0，即禁止倒退）、倒退 maneuver 为 −1（约束 +v≤0，即禁止
// 前进），接缝点由零速等式接管、符号门清零
TEST(DdpGatingPlanTest, SignGateOrientationBothDirections) {
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
    const DdpReference ref2 = fixture.reference_builder.build(pruned);
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
TEST(DdpGatingPlanTest, DwellWindowWidthScalesWithSteerDemand) {
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
    const DdpReference ref2 = fixture.reference_builder.build(pruned);
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
DdpReference MakeTinyReference() {
    DdpReference reference;
    reference.dt = 0.1;
    reference.ds = 0.05;
    reference.poses.emplace_back(0.0, 0.0, 0.0);
    reference.poses.emplace_back(0.05, 0.0, 0.0);
    reference.poses.emplace_back(0.10, 0.0, 0.0);
    reference.initial_states.resize(3, DdpState::Zero());
    reference.initial_controls.resize(2, DdpControl::Zero());
    return reference;
}

// 最小门控计划：节点 0 符号门 +1 + 驻留帽 0.05，节点 1 符号门 +1 + 接缝
// 等式，节点 2（终端）符号门 +1（应被忽略：终端由终点等式接管）
DdpGatingPlan MakeTinyPlan() {
    DdpGatingPlan plan;
    plan.sign_gate = {1, 1, 1};
    plan.seam_indices = {1};
    plan.seam_lookup = {-1, 0, -1};
    plan.dwell_v_cap = {0.05, 0.0, 0.0};
    return plan;
}

// 阶段一/阶段二隔离：阶段一配置下（无门控计划、乘子门控向量为空）门控
// 项恒为零——逐阶段 cost_gating 为 0，总代价与无门控扩展时逐位一致
TEST(DdpGatingCostTest, StageOneInputProducesZeroGatingCost) {
    const DdpReference reference = MakeTinyReference();
    const DdpCostEvaluator evaluator(DdpCostConfig{}, nullptr);
    const auto multipliers = DdpCostMultiplierState::MakeZero(2);
    DdpCostInput input;
    input.tracking_weight = 1.0;
    const DdpAlignedVec<DdpState> states(3, DdpState::Zero());
    const DdpAlignedVec<DdpControl> controls(2, DdpControl::Zero());
    const auto eval =
        evaluator.evaluate(reference, states, controls, multipliers, input);
    for (const auto& stage : eval.stages) {
        EXPECT_DOUBLE_EQ(stage.cost_gating, 0.0);
    }
    // 总代价 = 平滑 + 跟踪（零状态零控制下幅值/换挡项均不激活）
    double expected = 0.0;
    for (const auto& stage : eval.stages) {
        expected += stage.cost_smooth + stage.cost_tracking + stage.cost_shift +
                    stage.cost_amplitude + stage.cost_esdf +
                    stage.cost_terminal;
    }
    EXPECT_DOUBLE_EQ(eval.total_cost, expected);
}

// 「阶段一禁止启用」断言防御：门控计划与门控乘子必须同在场——只有计划
// 没有乘子、或只有乘子没有计划，都是配置错误，必须显式抛 std::logic_error
TEST(DdpGatingCostTest, PartialGatingConfigThrows) {
    const DdpReference reference = MakeTinyReference();
    const DdpCostEvaluator evaluator(DdpCostConfig{}, nullptr);
    const DdpGatingPlan plan = MakeTinyPlan();
    const DdpAlignedVec<DdpState> states(3, DdpState::Zero());
    const DdpAlignedVec<DdpControl> controls(2, DdpControl::Zero());
    DdpCostInput input;
    input.tracking_weight = 1.0;
    // 只有计划、乘子门控向量为空 → 拒绝
    input.gating_plan = &plan;
    const auto stage_one_multipliers = DdpCostMultiplierState::MakeZero(2);
    EXPECT_THROW(evaluator.evaluate(reference, states, controls,
                                    stage_one_multipliers, input),
                 std::logic_error);
    // 只有乘子、没有计划 → 拒绝
    input.gating_plan = nullptr;
    const auto gating_multipliers =
        DdpCostMultiplierState::MakeStageTwoZero(2, 1);
    EXPECT_THROW(evaluator.evaluate(reference, states, controls,
                                    gating_multipliers, input),
                 std::logic_error);
    // 乘子尺寸与计划不符 → 拒绝
    input.gating_plan = &plan;
    auto bad_multipliers = DdpCostMultiplierState::MakeStageTwoZero(2, 1);
    bad_multipliers.gating_seam_lambda.setZero(2);
    EXPECT_THROW(
        evaluator.evaluate(reference, states, controls, bad_multipliers, input),
        std::invalid_argument);
}

// 门控约束方向（代价层，两个方向都测）：符号门 −s·v≤0 的惩罚与梯度取向——
// s=+1 时负 v 受罚（梯度把 v 向上推），正 v 自由；s=−1 时正 v 受罚
// （梯度把 v 向下推），负 v 自由
TEST(DdpGatingCostTest, SignGatePenaltyDirectionBothSigns) {
    const DdpReference reference = MakeTinyReference();
    const DdpCostEvaluator evaluator(DdpCostConfig{}, nullptr);
    DdpGatingPlan plan = MakeTinyPlan();
    plan.seam_indices.clear();
    plan.seam_lookup = {-1, -1, -1};
    plan.dwell_v_cap = {0.0, 0.0, 0.0};
    DdpCostInput input;
    input.tracking_weight = 0.0;
    input.gating_plan = &plan;
    // 前进门（s=+1）：v=−0.2 受罚
    auto multipliers = DdpCostMultiplierState::MakeStageTwoZero(2, 0);
    multipliers.gating_sign_mu.setConstant(10.0);
    DdpAlignedVec<DdpState> states(3, DdpState::Zero());
    states[0](DDP_IDX_V) = -0.2;
    const DdpAlignedVec<DdpControl> controls(2, DdpControl::Zero());
    auto eval =
        evaluator.evaluate(reference, states, controls, multipliers, input);
    // g = −(+1)·(−0.2) = 0.2 > 0：代价 λg+½μg²=0.2，梯度 (λ+μg)·(−1)=−2
    EXPECT_NEAR(eval.stages[0].cost_gating, 0.2, 1e-12);
    EXPECT_NEAR(eval.stages[0].lx(DDP_IDX_V), -2.0, 1e-9);
    EXPECT_NEAR(eval.stages[0].lxx(DDP_IDX_V, DDP_IDX_V), 10.0, 1e-9);
    // v=+0.2 不激活（g<0 且 λ=0）
    states[0](DDP_IDX_V) = 0.2;
    eval = evaluator.evaluate(reference, states, controls, multipliers, input);
    EXPECT_DOUBLE_EQ(eval.stages[0].cost_gating, 0.0);
    EXPECT_DOUBLE_EQ(eval.stages[0].lx(DDP_IDX_V), 0.0);
    // 倒退门（s=−1）：v=+0.2 受罚（梯度把 v 向下推）
    plan.sign_gate = {-1, -1, -1};
    eval = evaluator.evaluate(reference, states, controls, multipliers, input);
    EXPECT_NEAR(eval.stages[0].cost_gating, 0.2, 1e-12);
    EXPECT_NEAR(eval.stages[0].lx(DDP_IDX_V), 2.0, 1e-9);
    // v=−0.2 不激活
    states[0](DDP_IDX_V) = -0.2;
    eval = evaluator.evaluate(reference, states, controls, multipliers, input);
    EXPECT_DOUBLE_EQ(eval.stages[0].cost_gating, 0.0);
}

// 接缝零速等式与驻留速度帽的取值：等式 c=v 以 λc+½μc² 计入；驻留帽取
// g=|v|−cap 形式（活动区梯度模恒为 1、对符号中性，二阶导在活动区精确）
TEST(DdpGatingCostTest, SeamEqualityAndDwellGateValues) {
    const DdpReference reference = MakeTinyReference();
    const DdpCostEvaluator evaluator(DdpCostConfig{}, nullptr);
    DdpGatingPlan plan = MakeTinyPlan();
    plan.sign_gate = {0, 0, 0};  // 关闭符号门，隔离被测项
    DdpCostInput input;
    input.tracking_weight = 0.0;
    input.gating_plan = &plan;
    auto multipliers = DdpCostMultiplierState::MakeStageTwoZero(2, 1);
    multipliers.gating_seam_mu.setConstant(10.0);
    multipliers.gating_dwell_mu.setConstant(10.0);
    DdpAlignedVec<DdpState> states(3, DdpState::Zero());
    const DdpAlignedVec<DdpControl> controls(2, DdpControl::Zero());
    // 接缝节点 v=0.1：c=0.1，代价 ½·10·0.01=0.05，梯度 10·0.1=1.0
    states[1](DDP_IDX_V) = 0.1;
    auto eval =
        evaluator.evaluate(reference, states, controls, multipliers, input);
    EXPECT_NEAR(eval.stages[1].cost_gating, 0.05, 1e-12);
    EXPECT_NEAR(eval.stages[1].lx(DDP_IDX_V), 1.0, 1e-9);
    EXPECT_NEAR(eval.stages[1].lxx(DDP_IDX_V, DDP_IDX_V), 10.0, 1e-9);
    // 驻留帽节点 v=−0.2（cap=0.05）：g=0.2−0.05=0.15，
    // 代价 ½·10·0.0225=0.1125，梯度 (μg)·(−1)=−1.5，GN Hessian μ=10
    states[1](DDP_IDX_V) = 0.0;
    states[0](DDP_IDX_V) = -0.2;
    eval = evaluator.evaluate(reference, states, controls, multipliers, input);
    EXPECT_NEAR(eval.stages[0].cost_gating, 0.1125, 1e-12);
    EXPECT_NEAR(eval.stages[0].lx(DDP_IDX_V), -1.5, 1e-9);
    EXPECT_NEAR(eval.stages[0].lxx(DDP_IDX_V, DDP_IDX_V), 10.0, 1e-9);
    // 窗内 |v|≤cap 不激活
    states[0](DDP_IDX_V) = 0.03;
    eval = evaluator.evaluate(reference, states, controls, multipliers, input);
    EXPECT_DOUBLE_EQ(eval.stages[0].cost_gating, 0.0);
}

// 门控项导数的有限差分对拍：符号门 + 驻留帽同时激活的阶段，ℓ_x 与
// ℓ_xx 的 v 分量与解析值/中心差分一致。符号门残差线性、驻留帽残差
// |v|−cap（活动区光滑），两者的 ½μg² 二阶导在活动区都是精确的 μ——
// GN 形 Hessian 此处无丢弃项，可直接与解析值逐位对拍
TEST(DdpGatingCostTest, GatingDerivativesMatchFiniteDifference) {
    const DdpReference reference = MakeTinyReference();
    const DdpCostEvaluator evaluator(DdpCostConfig{}, nullptr);
    const DdpGatingPlan plan = MakeTinyPlan();
    DdpCostInput input;
    input.tracking_weight = 0.0;
    input.gating_plan = &plan;
    auto multipliers = DdpCostMultiplierState::MakeStageTwoZero(2, 1);
    multipliers.gating_sign_mu.setConstant(10.0);
    multipliers.gating_seam_mu.setConstant(10.0);
    multipliers.gating_dwell_mu.setConstant(10.0);
    // 节点 0：v=−0.2 同时违反符号门与驻留帽
    DdpAlignedVec<DdpState> states(3, DdpState::Zero());
    states[0](DDP_IDX_V) = -0.2;
    const DdpAlignedVec<DdpControl> controls(2, DdpControl::Zero());
    const auto eval =
        evaluator.evaluate(reference, states, controls, multipliers, input);
    const auto gating_cost = [&](double v) {
        DdpAlignedVec<DdpState> perturbed = states;
        perturbed[0](DDP_IDX_V) = v;
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
    EXPECT_NEAR(eval.stages[0].lx(DDP_IDX_V), fd_grad, 1e-4);
    // 解析值：符号门梯度 (μ·0.2)·(−1)=−2 + 驻留帽梯度 (μ·0.15)·(−1)=−1.5，
    // 合计 −3.5；Hessian 两组约束各 μ=10，合计 20
    EXPECT_NEAR(eval.stages[0].lx(DDP_IDX_V), -3.5, 1e-9);
    EXPECT_NEAR(eval.stages[0].lxx(DDP_IDX_V, DDP_IDX_V), 20.0, 1e-9);
    EXPECT_NEAR(eval.stages[0].lxx(DDP_IDX_V, DDP_IDX_V), fd_hess, 1e-2);
    // 门控只依赖 v：其余状态/控制导数分量恒零
    for (int i = 0; i < DDP_STATE_DIM; ++i) {
        if (i != DDP_IDX_V) {
            EXPECT_DOUBLE_EQ(eval.stages[0].lx(i), 0.0);
        }
    }
    EXPECT_DOUBLE_EQ(eval.stages[0].lu.norm(), 0.0);
}

// ==================== 阶段二门控重解 ====================

// 手工构建阶段二门控计划（按参考 maneuver 元数据 + 接缝窗宽 m）
DdpGatingPlan MakeSolverGatingPlan(const DdpReference& reference,
                                   std::size_t window_half, double v_cap) {
    const std::size_t num_poses = reference.poses.size();
    DdpGatingPlan plan;
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
TEST(DdpStageTwoSolveTest, InvalidGatingPlanThrows) {
    PostStageFixture fixture;
    const Path path = BuildXPolyline({0.0, 1.0, -1.0});
    const DdpReference reference = BuildReference(path);
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
TEST(DdpStageTwoSolveTest, StageTwoEnforcesSignSeamAndDwellGates) {
    PostStageFixture fixture;
    const Path path = BuildXPolyline({0.0, 1.0, -1.0});
    const DdpReference reference = BuildReference(path);
    // 先跑阶段一获得热启动（保留换挡：倒退 maneuver 承载终点语义）
    const auto stage_one = fixture.solver.solveStageOne(reference);
    ASSERT_EQ(stage_one.report.status, ApaDdpStatus::CONVERGED)
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
    EXPECT_EQ(stage_two.report.status, ApaDdpStatus::CONVERGED)
        << "outer=" << stage_two.report.outer_iterations
        << " sign=" << stage_two.max_sign_violation
        << " dwell=" << stage_two.max_dwell_violation
        << " seam=" << stage_two.max_seam_speed
        << " pos_err=" << stage_two.report.terminal_position_error;
    EXPECT_TRUE(stage_two.gating_ok);
    // 符号门：前进段（窗外）无倒退、倒退段（窗外）无前进
    for (std::size_t k = 0; k + 2 < seam; ++k) {
        EXPECT_GE(stage_two.states[k](DDP_IDX_V), -0.02) << "k=" << k;
    }
    for (std::size_t k = seam + 3; k < stage_two.states.size(); ++k) {
        EXPECT_LE(stage_two.states[k](DDP_IDX_V), 0.02) << "k=" << k;
    }
    // 接缝零速与驻留窗速度帽
    EXPECT_LE(stage_two.max_seam_speed, 0.02);
    for (std::size_t k = seam - 2; k <= seam + 2; ++k) {
        EXPECT_LE(std::abs(stage_two.states[k](DDP_IDX_V)), 0.05 + 0.02)
            << "k=" << k;
    }
    // 终点照常收敛
    EXPECT_LE(stage_two.report.terminal_position_error, 0.05);
    EXPECT_LE(stage_two.report.terminal_heading_error_deg, 1.5);
}

// 白盒测试夹具：派生暴露 protected 门控乘子更新接口（不改动被测实现，
// 仅用于钉住门控 μ 增长排程）
class GatingUpdateAccessor : public ApaDdpSolver {
   public:
    using ApaDdpSolver::ApaDdpSolver;
    using ApaDdpSolver::GatingSnapshot;
    using ApaDdpSolver::updateGating;
};

// 门控 μ 增长的充分下降门控（显式钉住）：首轮只记录不增长；违反度充分
// 下降（≤κ·上轮）时 μ 保持原值；未充分下降（>κ·上轮）时 μ 按 φ 倍提升
// 并同步写回三组门控乘子；λ 更新遵循 Hestenes-Powell（不等式投影非负、
// 等式符号自由）
TEST(DdpStageTwoGatingMuTest, SufficientDecreaseDoesNotGrowMu) {
    const ApaDdpSolverConfig config = MakeSyntheticConfig();
    const BicycleDynamics dynamics(kWheelbase);
    const DdpCostEvaluator evaluator(config.cost, nullptr);
    GatingUpdateAccessor solver(config, &dynamics, &evaluator);
    auto multipliers = DdpCostMultiplierState::MakeStageTwoZero(1, 1);
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
    EXPECT_TRUE(solver.updateGating(snapshot, &multipliers, &gating_mu, &prev));
    EXPECT_DOUBLE_EQ(gating_mu, 100.0);
    EXPECT_DOUBLE_EQ(multipliers.gating_sign_mu(0), 100.0);
    EXPECT_DOUBLE_EQ(multipliers.gating_dwell_mu(0), 100.0);
    EXPECT_DOUBLE_EQ(multipliers.gating_seam_mu(0), 100.0);
}

// 阶段二对偶热启动对照：同一原始热启动下，带阶段一终态乘子种子
// （dual_seed）的重解外层轮数不得超过对偶冷启动——已收敛的 AL 平衡由
// λ/μ 承载，续接而非推倒重来（冷启动实测把终端平衡重新打开、轮数翻倍）
TEST(DdpStageTwoSolveTest, DualSeedWarmStartConvergesNoSlowerThanCold) {
    PostStageFixture fixture;
    const Path path = BuildXPolyline({0.0, 1.0, 0.85, 1.85});
    const DdpReference reference = BuildReference(path);
    const auto stage_one = fixture.solver.solveStageOne(reference);
    ASSERT_EQ(stage_one.report.status, ApaDdpStatus::CONVERGED);
    // 门控计划按修剪后结构构建（与真实后处理同源）：阶段一解已融化为
    // 单 maneuver（v 全程为正），故全段 +1 符号门、无接缝无驻留窗——
    // 若误用未修剪的三段计划（含倒退门），会迫使阶段二"反向融化"
    DdpGatingPlan plan;
    plan.sign_gate.assign(reference.poses.size(), 1);
    plan.seam_lookup.assign(reference.poses.size(), -1);
    plan.dwell_v_cap.assign(reference.poses.size(), 0.0);
    const double tracking_weight =
        stage_one.report.history.back().tracking_weight;
    // 两个独立求解器实例：内层 μ_m/ρ_reg 跨 solve 调用保持，对照实验
    // 必须各自独立，避免第一次求解的内层状态污染第二次
    const ApaDdpSolverConfig config = MakeSyntheticConfig();
    const BicycleDynamics dynamics(kWheelbase);
    const DdpCostEvaluator evaluator(config.cost, nullptr);
    ApaDdpSolver warm_solver(config, &dynamics, &evaluator);
    ApaDdpSolver cold_solver(config, &dynamics, &evaluator);
    const auto warm = warm_solver.solveStageTwo(
        reference, plan, stage_one.states, stage_one.controls, tracking_weight,
        &stage_one.final_multipliers);
    const auto cold =
        cold_solver.solveStageTwo(reference, plan, stage_one.states,
                                  stage_one.controls, tracking_weight, nullptr);
    EXPECT_EQ(warm.report.status, ApaDdpStatus::CONVERGED)
        << "outer=" << warm.report.outer_iterations
        << " pos_err=" << warm.report.terminal_position_error;
    EXPECT_LE(warm.report.outer_iterations, cold.report.outer_iterations)
        << "warm=" << warm.report.outer_iterations
        << " cold=" << cold.report.outer_iterations;
    std::cout << "[DDP-DUAL] warm_outer=" << warm.report.outer_iterations
              << " cold_outer=" << cold.report.outer_iterations
              << " cold_status=" << static_cast<int>(cold.report.status)
              << std::endl;
}

// ==================== 驻留插入 ====================

// 驻留插入的时间拉伸语义：接缝 Δδ=0.3（T_resteer=1.1 s，T_dwell=1.32 s）
// 超过窗口时长 1.0 s——窗内 δ 摆动剖面按 1.4 倍放慢重定时（v/ω 同比
// 减小），时间戳严格单调、总时长增加、窗内 |v|≤v_dwell，且重排后轨迹
// 仍通过 Trajectory::validate() 运动学门
TEST(DdpDwellInsertTest, RetimesWindowAndKeepsKinematics) {
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
    DdpAlignedVec<DdpState> states(41, DdpState::Zero());
    for (std::size_t k = 0; k < states.size(); ++k) {
        states[k](DDP_IDX_V) = v_prof[k];
        states[k](DDP_IDX_A) = a_prof[k];
        states[k](DDP_IDX_DELTA) = d_prof[k];
        states[k](DDP_IDX_OMEGA) = w_prof[k];
        if (k > 0) {
            states[k](DDP_IDX_X) =
                states[k - 1](DDP_IDX_X) + 0.05 * (v_prof[k - 1] + v_prof[k]);
        }
    }
    const std::vector<DdpSeamPlan> seams = {
        DdpSeamPlan{20, 15, 25, 0.0, 0.0, 0.0}};
    std::vector<DdpSeamReport> reports;
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
    // 静止段 v/a 近零语义逐点断言：|v|≤0.05/1.4+裕量、|a|≤0.1/1.96+裕量
    const double t_dwell_start = 15 * 0.1;
    for (const auto& pt : output) {
        if (pt.getT() >= t_dwell_start - 1e-9 &&
            pt.getT() <= t_dwell_start + 1.4 + 1e-9) {
            EXPECT_LE(std::abs(pt.getV()), 0.05 + 1e-9);
            EXPECT_LE(std::abs(pt.getA()), 0.1 / (1.4 * 1.4) + 1e-6);
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

// ==================== 后处理主流程（端到端） ====================

// 融化场景全链路：「前进 1.0 → 倒退 0.15 → 前进 1.0」的阶段一解（v 全程
// 不变号）经后处理恢复为单 maneuver——修剪无残余微段、阶段二重解收敛、
// 六项校验全过、输出非回退轨迹，maneuver 数 3→1 不增
TEST(DdpPostStageTest, MeltedScenarioOutputsSingleManeuver) {
    PostStageFixture fixture;
    const Path path = BuildXPolyline({0.0, 1.0, 0.85, 1.85});
    const DdpReference reference = BuildReference(path);
    const auto stage_one = fixture.solver.solveStageOne(reference);
    ASSERT_EQ(stage_one.report.status, ApaDdpStatus::CONVERGED)
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
    EXPECT_EQ(result.status, DdpPostStageStatus::SUCCESS)
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

// 保留换挡场景全链路：「前进 1.0 → 倒退 2.0」——后处理恢复 2 个 maneuver，
// 阶段二门控重解后插入驻留（总时长相应增加、驻留段 |v|≤v_dwell、时间戳
// 严格单调），六项校验全过，maneuver 数不增
TEST(DdpPostStageTest, ReversalScenarioInsertsDwellAndValidates) {
    PostStageFixture fixture;
    const Path path = BuildXPolyline({0.0, 1.0, -1.0});
    const DdpReference reference = BuildReference(path);
    const auto stage_one = fixture.solver.solveStageOne(reference);
    ASSERT_EQ(stage_one.report.status, ApaDdpStatus::CONVERGED)
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
    EXPECT_EQ(result.status, DdpPostStageStatus::SUCCESS)
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
    std::cout << "[DDP-POST] dwell=" << seam.dwell_duration
              << " t_resteer=" << seam.t_resteer
              << " duration=" << result.trajectory.duration()
              << " stage_two_duration=" << stage_two_duration << std::endl;
}

// 回退路径：校验清单被故意破坏（终点目标放到 100 m 外，终点双指标必然
// 不过）——必须返回原始 A* 路径转化的回退轨迹（绝不输出半成品），且
// 结构化诊断完整（失败项名称 + 量化值 + 阈值）
TEST(DdpPostStageTest, FallbackOnValidationFailure) {
    PostStageFixture fixture;
    const Path path = BuildXPolyline({0.0, 1.0, 0.85, 1.85});
    const DdpReference reference = BuildReference(path);
    const auto stage_one = fixture.solver.solveStageOne(reference);
    ASSERT_EQ(stage_one.report.status, ApaDdpStatus::CONVERGED);
    GridMap grid_map(0.1, 300, 200, Position{-15.0, -10.0}, {});
    const ESDFMap esdf_map(grid_map);
    const VehicleFootprintModel footprint_model(MakeVehicleParams(), 233, 2, 2);
    TrajectoryPoint goal;
    goal.x = 100.0;
    goal.y = 100.0;
    goal.theta = 0.0;
    const auto result = fixture.post_stage.run(path, reference, stage_one, goal,
                                               esdf_map, footprint_model);
    EXPECT_EQ(result.status, DdpPostStageStatus::VALIDATION_FAILED);
    EXPECT_TRUE(result.used_fallback);
    EXPECT_EQ(result.diagnostics.failed_check, "terminal_position");
    EXPECT_GT(result.diagnostics.measured_value, result.diagnostics.threshold);
    // 回退轨迹 = 原始 A* 路径（起止点一致、非空、时间戳完备）
    ASSERT_FALSE(result.trajectory.empty());
    EXPECT_NEAR(result.trajectory.front().x, 0.0, 1e-9);
    EXPECT_NEAR(result.trajectory.back().x, 1.85, 1e-6);
    EXPECT_TRUE(result.trajectory.back().hasT());
}

// 回退路径：阶段一解中混入 PIVOT 微游程（合成注入，模拟求解器吐出
// 原地打轮式微动）——修剪触发失败语义，直接回退原始路径，不再进入
// 阶段二重解
TEST(DdpPostStageTest, PivotScenarioFallsBack) {
    PostStageFixture fixture;
    const Path path = BuildXPolyline({0.0, 2.0});
    // 合成阶段一结果：前进 2 m → PIVOT 微游程（δ 跳变 0.3）→ 前进继续
    std::vector<ProfileEntry> profile;
    AppendRun(&profile, 0.05, 41, 0.5, 0.0);
    profile.push_back({2.00, -0.03, 0.3});
    profile.push_back({1.99, -0.03, 0.3});
    profile.push_back({2.005, 0.5, 0.0});
    AppendRun(&profile, 0.05, 40, 0.5, 0.0);
    ApaDdpStageOneResult stage_one;
    stage_one.states = MakeStates(profile);
    stage_one.controls.assign(profile.size() - 1, DdpControl::Zero());
    stage_one.report.status = ApaDdpStatus::CONVERGED;
    DdpReference reference;
    reference.maneuvers.resize(3);
    GridMap grid_map(0.1, 300, 200, Position{-15.0, -10.0}, {});
    const ESDFMap esdf_map(grid_map);
    const VehicleFootprintModel footprint_model(MakeVehicleParams(), 233, 2, 2);
    TrajectoryPoint goal;
    goal.x = 2.0;
    const auto result = fixture.post_stage.run(path, reference, stage_one, goal,
                                               esdf_map, footprint_model);
    EXPECT_EQ(result.status, DdpPostStageStatus::PIVOT_DETECTED);
    EXPECT_TRUE(result.used_fallback);
    EXPECT_FALSE(result.stage_two.has_value());
    EXPECT_EQ(result.diagnostics.failed_check, "pivot");
    ASSERT_FALSE(result.trajectory.empty());
}

// 回退路径：阶段一未收敛（外层迭代耗尽）——后处理不得带病进入修剪与
// 阶段二，直接回退原始路径并如实记录失败阶段
TEST(DdpPostStageTest, StageOneFailureFallsBack) {
    PostStageFixture fixture;
    const Path path = BuildXPolyline({0.0, 2.0});
    ApaDdpStageOneResult stage_one;
    stage_one.states = MakeStates({{0.0, 0.5, 0.0}, {0.05, 0.5, 0.0}});
    stage_one.controls.assign(1, DdpControl::Zero());
    stage_one.report.status = ApaDdpStatus::MAX_OUTER_ITERATIONS;
    DdpReference reference;
    reference.maneuvers.resize(1);
    GridMap grid_map(0.1, 300, 200, Position{-15.0, -10.0}, {});
    const ESDFMap esdf_map(grid_map);
    const VehicleFootprintModel footprint_model(MakeVehicleParams(), 233, 2, 2);
    TrajectoryPoint goal;
    const auto result = fixture.post_stage.run(path, reference, stage_one, goal,
                                               esdf_map, footprint_model);
    EXPECT_EQ(result.status, DdpPostStageStatus::STAGE_ONE_NOT_CONVERGED);
    EXPECT_TRUE(result.used_fallback);
    EXPECT_EQ(result.diagnostics.failed_check, "stage_one_convergence");
    ASSERT_FALSE(result.trajectory.empty());
}

// 回退路径：阶段一解退化为全长 0.02 m 的蠕动轨迹（单游程），修剪后路径
// 总长不足一个重采样间距（0.05 m），无法构建阶段二参考——直接回退并
// 给出精细化诊断（弧长子情形：实测 0.02 m vs 阈值 0.05 m）
TEST(DdpPostStageTest, DegeneratePrunedPathFallsBack) {
    PostStageFixture fixture;
    const Path path = BuildXPolyline({0.0, 2.0});
    ApaDdpStageOneResult stage_one;
    stage_one.states =
        MakeStates({{0.0, 0.03, 0.0}, {0.01, 0.03, 0.0}, {0.02, 0.03, 0.0}});
    stage_one.controls.assign(2, DdpControl::Zero());
    stage_one.report.status = ApaDdpStatus::CONVERGED;
    DdpReference reference;
    reference.maneuvers.resize(1);
    GridMap grid_map(0.1, 300, 200, Position{-15.0, -10.0}, {});
    const ESDFMap esdf_map(grid_map);
    const VehicleFootprintModel footprint_model(MakeVehicleParams(), 233, 2, 2);
    TrajectoryPoint goal;
    const auto result = fixture.post_stage.run(path, reference, stage_one, goal,
                                               esdf_map, footprint_model);
    EXPECT_EQ(result.status, DdpPostStageStatus::PRUNED_PATH_DEGENERATE);
    EXPECT_TRUE(result.used_fallback);
    EXPECT_EQ(result.diagnostics.failed_check, "pruned_length");
    EXPECT_NEAR(result.diagnostics.measured_value, 0.02, 1e-9);
    EXPECT_NEAR(result.diagnostics.threshold, 0.05, 1e-9);
    ASSERT_FALSE(result.trajectory.empty());
}

}  // namespace
}  // namespace apa_post_processor
