#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <vector>

#include "core/DDP/al_outer_loop.h"
#include "core/DDP/ddp_cost.h"
#include "core/DDP/ddp_reference_builder.h"

namespace apa_post_processor {
namespace {

constexpr double kDt = 0.1;

// 按状态分量布局 [x, y, θ, v, a, δ, ω] 构造七维状态
DdpState MakeState(double x, double y, double theta, double v, double a,
                   double delta, double omega) {
    DdpState state;
    state << x, y, theta, v, a, delta, omega;
    return state;
}

// 构造最小可用参考：外层循环只消费位姿/dt/maneuver 区间元数据
DdpReference MakeReference(const std::vector<Pose>& poses,
                           const std::vector<DdpReferenceManeuver>& maneuvers) {
    DdpReference reference;
    reference.ds = 0.05;
    reference.dt = kDt;
    reference.poses = poses;
    reference.maneuvers = maneuvers;
    return reference;
}

// 沿 x 轴等间距直线位姿序列
std::vector<Pose> MakeLinePoses(std::size_t count, double spacing) {
    std::vector<Pose> poses;
    poses.reserve(count);
    for (std::size_t k = 0; k < count; ++k) {
        poses.emplace_back(spacing * static_cast<double>(k), 0.0, 0.0);
    }
    return poses;
}

// 单 maneuver 元数据（区间 [begin, end]）
DdpReferenceManeuver MakeManeuver(int sign, std::size_t begin,
                                  std::size_t end) {
    DdpReferenceManeuver maneuver;
    maneuver.sign = sign;
    maneuver.begin_index = begin;
    maneuver.end_index = end;
    return maneuver;
}

// 构造全零缺陷序列（尺寸 N+1）
DdpAlignedVec<DdpState> MakeZeroDefects(std::size_t num_poses) {
    DdpAlignedVec<DdpState> defects;
    defects.reserve(num_poses);
    for (std::size_t k = 0; k < num_poses; ++k) {
        defects.push_back(DdpState::Zero());
    }
    return defects;
}

// 构造只含终点残差、幅值约束全满足的快照（violation_norm = ‖c‖）
AlConstraintSnapshot MakeTerminalOnlySnapshot(double c_x) {
    AlConstraintSnapshot snapshot;
    snapshot.amplitude_g = Eigen::VectorXd::Constant(5, -1.0);
    snapshot.terminal_c << c_x, 0.0, 0.0, 0.0, 0.0;
    snapshot.max_amplitude_violation = 0.0;
    snapshot.terminal_position_error = std::abs(c_x);
    snapshot.terminal_heading_error_deg = 0.0;
    snapshot.defect_norm_inf = 0.0;
    snapshot.violation_norm = std::abs(c_x);
    return snapshot;
}

// 测试非法配置逐项被构造校验拒绝：迭代上限/容差/clip 区间/门控/退火
// 参数必须为正且自洽，错误配置会静默污染全部外层调度，必须显式抛出
TEST(AlOuterLoopConfigTest, InvalidConfigThrows) {
    const DdpCostConfig cost_config;
    // 合法基线：不应抛出
    EXPECT_NO_THROW(AlOuterLoop(AlOuterLoopConfig{}, cost_config));
    // 逐项变异：每项独立断言抛出 std::invalid_argument
    AlOuterLoopConfig config;
    config.max_outer_iterations = 0;
    EXPECT_THROW(AlOuterLoop(config, cost_config), std::invalid_argument);
    config = AlOuterLoopConfig{};
    config.terminal_position_tol = 0.0;
    EXPECT_THROW(AlOuterLoop(config, cost_config), std::invalid_argument);
    config = AlOuterLoopConfig{};
    config.terminal_heading_tol_deg = -1.0;
    EXPECT_THROW(AlOuterLoop(config, cost_config), std::invalid_argument);
    config = AlOuterLoopConfig{};
    config.inequality_tol = 0.0;
    EXPECT_THROW(AlOuterLoop(config, cost_config), std::invalid_argument);
    config = AlOuterLoopConfig{};
    config.defect_tol = -1e-3;
    EXPECT_THROW(AlOuterLoop(config, cost_config), std::invalid_argument);
    config = AlOuterLoopConfig{};
    config.mu_min = 0.0;
    EXPECT_THROW(AlOuterLoop(config, cost_config), std::invalid_argument);
    config = AlOuterLoopConfig{};
    config.mu_min = 1e6;
    config.mu_max = 1e2;
    EXPECT_THROW(AlOuterLoop(config, cost_config), std::invalid_argument);
    config = AlOuterLoopConfig{};
    config.first_round_mu = 0.0;
    EXPECT_THROW(AlOuterLoop(config, cost_config), std::invalid_argument);
    config = AlOuterLoopConfig{};
    config.amplitude_mu_initial = -1.0;
    EXPECT_THROW(AlOuterLoop(config, cost_config), std::invalid_argument);
    config = AlOuterLoopConfig{};
    config.epsilon_mu = -1e-4;
    EXPECT_THROW(AlOuterLoop(config, cost_config), std::invalid_argument);
    config = AlOuterLoopConfig{};
    config.mu_gate_kappa = 1.0;
    EXPECT_THROW(AlOuterLoop(config, cost_config), std::invalid_argument);
    config = AlOuterLoopConfig{};
    config.mu_growth_factor = 1.0;
    EXPECT_THROW(AlOuterLoop(config, cost_config), std::invalid_argument);
    config = AlOuterLoopConfig{};
    config.anneal_gamma = 0.0;
    EXPECT_THROW(AlOuterLoop(config, cost_config), std::invalid_argument);
    config = AlOuterLoopConfig{};
    config.anneal_gamma = 1.0;
    EXPECT_THROW(AlOuterLoop(config, cost_config), std::invalid_argument);
    // 换挡代理 β 退火调度：门宽必须满足 0<β_final<=β_initial 且 0<γ_β<1
    config = AlOuterLoopConfig{};
    config.shift_beta_initial = 0.0;
    EXPECT_THROW(AlOuterLoop(config, cost_config), std::invalid_argument);
    config = AlOuterLoopConfig{};
    config.shift_beta_final = -0.05;
    EXPECT_THROW(AlOuterLoop(config, cost_config), std::invalid_argument);
    config = AlOuterLoopConfig{};
    config.shift_beta_initial = 0.05;
    config.shift_beta_final = 0.3;
    EXPECT_THROW(AlOuterLoop(config, cost_config), std::invalid_argument);
    config = AlOuterLoopConfig{};
    config.shift_beta_gamma = 1.0;
    EXPECT_THROW(AlOuterLoop(config, cost_config), std::invalid_argument);
    // 候选段差异化退火：临界比阈值必须为正、候选退火率满足 0<γ_cand<1
    config = AlOuterLoopConfig{};
    config.melt_crit_threshold = 0.0;
    EXPECT_THROW(AlOuterLoop(config, cost_config), std::invalid_argument);
    config = AlOuterLoopConfig{};
    config.candidate_anneal_gamma = 0.0;
    EXPECT_THROW(AlOuterLoop(config, cost_config), std::invalid_argument);
    config = AlOuterLoopConfig{};
    config.candidate_anneal_gamma = 1.0;
    EXPECT_THROW(AlOuterLoop(config, cost_config), std::invalid_argument);
    // 代价配置提供幅值边界，同样必须为正有限
    DdpCostConfig bad_cost;
    bad_cost.v_max = 0.0;
    EXPECT_THROW(AlOuterLoop(AlOuterLoopConfig{}, bad_cost),
                 std::invalid_argument);
}

// 测试首轮乘子状态：λ=0（纯软惩罚启动，不预设先验乘子）、终点罚权重
// 取 first_round_mu 弱启动、幅值罚权重取 amplitude_mu_initial 强启动
// （物理边界越界从第 0 轮即被压回），尺寸布局与代价求值层契约一致
TEST(AlOuterLoopTest, InitialMultipliersUseZeroLambdaAndFirstRoundMu) {
    AlOuterLoopConfig config;
    config.first_round_mu = 2.5;
    config.amplitude_mu_initial = 300.0;
    AlOuterLoop loop(config, DdpCostConfig{});
    const std::size_t num_steps = 7;
    const auto multipliers = loop.makeInitialMultipliers(num_steps);
    ASSERT_EQ(
        multipliers.amplitude_lambda.size(),
        static_cast<Eigen::Index>(DDP_AMPLITUDE_CONSTRAINT_DIM * num_steps));
    ASSERT_EQ(multipliers.amplitude_mu.size(),
              multipliers.amplitude_lambda.size());
    EXPECT_DOUBLE_EQ(multipliers.amplitude_lambda.norm(), 0.0);
    EXPECT_DOUBLE_EQ(multipliers.terminal_lambda.norm(), 0.0);
    for (Eigen::Index i = 0; i < multipliers.amplitude_mu.size(); ++i) {
        EXPECT_DOUBLE_EQ(multipliers.amplitude_mu(i), 300.0);
    }
    for (int i = 0; i < DDP_TERMINAL_CONSTRAINT_DIM; ++i) {
        EXPECT_DOUBLE_EQ(multipliers.terminal_mu(i), 2.5);
    }
}

// 测试约束残差量测：幅值 g 的公式与布局（5k+i）、终点 c 的符号与角度
// wrap（跨 ±π 折回）、聚合量（最大违反度/双指标/联合范数/缺陷无穷范数）
// 必须与代价求值层同一组约定，否则乘子更新与终止判据失配
TEST(AlOuterLoopTest, MeasureComputesExactResidualsAndAggregates) {
    AlOuterLoop loop(AlOuterLoopConfig{}, DdpCostConfig{});
    // 4 位姿（N=3）：终点目标 (0.15, 0, 0.1)
    auto poses = MakeLinePoses(4, 0.05);
    poses[3].theta = 0.1;
    const auto reference = MakeReference(poses, {});
    DdpAlignedVec<DdpState> states;
    states.reserve(4);
    // k=0：v 超限 0.1 → g_v = 1.6²−1.5² = 0.31
    states.push_back(MakeState(0.0, 0.0, 0.0, 1.6, 0.0, 0.0, 0.0));
    // k=1：a 超限 → g_a = 1.2²−1.0² = 0.44；δ=0.7 → g_δ⁺=0.15、g_δ⁻=−1.25
    states.push_back(MakeState(0.05, 0.0, 0.0, 0.0, 1.2, 0.7, 0.0));
    // k=2：δ=−0.7 → g_δ⁺=−1.25、g_δ⁻=0.15；ω=0.6 → g_ω = 0.36−0.25 = 0.11
    states.push_back(MakeState(0.1, 0.0, 0.0, 0.0, 0.0, -0.7, 0.6));
    // 终点：位置残差 (0.05, −0.1)；θ 残差 0.1−0.1−3π/2 → wrap 后 +π/2；
    // v_N=0.03、a_N=−0.02
    states.push_back(
        MakeState(0.2, -0.1, 0.1 - 1.5 * PI, 0.03, -0.02, 0.0, 0.0));
    auto defects = MakeZeroDefects(4);
    defects[2](DDP_IDX_V) = -0.007;
    const auto snapshot = loop.measure(reference, states, defects);
    // 幅值 g：尺寸 5N=15，布局 [5k, 5k+5)
    ASSERT_EQ(snapshot.amplitude_g.size(), 15);
    EXPECT_NEAR(snapshot.amplitude_g(DDP_AMP_V), 0.31, 1e-12);
    EXPECT_NEAR(snapshot.amplitude_g(5 + DDP_AMP_A), 0.44, 1e-12);
    EXPECT_NEAR(snapshot.amplitude_g(5 + DDP_AMP_DELTA_POS), 0.15, 1e-12);
    EXPECT_NEAR(snapshot.amplitude_g(5 + DDP_AMP_DELTA_NEG), -1.25, 1e-12);
    EXPECT_NEAR(snapshot.amplitude_g(10 + DDP_AMP_DELTA_POS), -1.25, 1e-12);
    EXPECT_NEAR(snapshot.amplitude_g(10 + DDP_AMP_DELTA_NEG), 0.15, 1e-12);
    EXPECT_NEAR(snapshot.amplitude_g(10 + DDP_AMP_OMEGA), 0.11, 1e-12);
    // 终点 c = [x−xg, y−yg, wrap(θ−θg), v, a]
    EXPECT_NEAR(snapshot.terminal_c(0), 0.05, 1e-12);
    EXPECT_NEAR(snapshot.terminal_c(1), -0.1, 1e-12);
    EXPECT_NEAR(snapshot.terminal_c(2), 0.5 * PI, 1e-9);
    EXPECT_NEAR(snapshot.terminal_c(3), 0.03, 1e-12);
    EXPECT_NEAR(snapshot.terminal_c(4), -0.02, 1e-12);
    // 聚合量
    EXPECT_NEAR(snapshot.max_amplitude_violation, 0.44, 1e-12);
    EXPECT_NEAR(snapshot.terminal_position_error, std::hypot(0.05, 0.1), 1e-12);
    EXPECT_NEAR(snapshot.terminal_heading_error_deg, 90.0, 1e-6);
    EXPECT_NEAR(snapshot.defect_norm_inf, 0.007, 1e-12);
    const double expected_norm = std::sqrt(
        0.05 * 0.05 + 0.1 * 0.1 + 0.25 * PI * PI + 0.03 * 0.03 + 0.02 * 0.02 +
        0.31 * 0.31 + 0.44 * 0.44 + 0.15 * 0.15 + 0.15 * 0.15 + 0.11 * 0.11);
    EXPECT_NEAR(snapshot.violation_norm, expected_norm, 1e-9);
}

// 测试量测输入校验：状态/缺陷数量与参考位姿不一致时显式抛出，
// 禁止静默产出错位的乘子更新
TEST(AlOuterLoopTest, MeasureRejectsInconsistentSizes) {
    AlOuterLoop loop(AlOuterLoopConfig{}, DdpCostConfig{});
    const auto reference = MakeReference(MakeLinePoses(4, 0.05), {});
    DdpAlignedVec<DdpState> states;
    states.push_back(DdpState::Zero());
    auto defects = MakeZeroDefects(4);
    EXPECT_THROW(loop.measure(reference, states, defects),
                 std::invalid_argument);
    const auto reference_too_short = MakeReference(MakeLinePoses(1, 0.05), {});
    EXPECT_THROW(loop.measure(reference_too_short, states, defects),
                 std::invalid_argument);
}

// 测试退火豁免掩码：首/末 maneuver 覆盖的点恒 true（承载起点状态与终点
// 语义），中间 maneuver 点 false 参与退火；maneuver 共享边界点归属相邻
// 两段，按首/末段区间覆盖判定
TEST(AlOuterLoopTest, AnnealExemptMaskCoversFirstAndLastManeuverOnly) {
    AlOuterLoop loop(AlOuterLoopConfig{}, DdpCostConfig{});
    const auto reference =
        MakeReference(MakeLinePoses(31, 0.05),
                      {MakeManeuver(1, 0, 10), MakeManeuver(-1, 10, 20),
                       MakeManeuver(1, 20, 30)});
    const auto mask = loop.makeAnnealExemptMask(reference);
    ASSERT_EQ(mask.size(), 31);
    for (std::size_t k = 0; k <= 10; ++k) {
        EXPECT_TRUE(mask[k]) << "k = " << k;
    }
    for (std::size_t k = 11; k <= 19; ++k) {
        EXPECT_FALSE(mask[k]) << "k = " << k;
    }
    for (std::size_t k = 20; k <= 30; ++k) {
        EXPECT_TRUE(mask[k]) << "k = " << k;
    }
}

// 测试单 maneuver 路径整体豁免（无可融化的中间段，跟踪权重全程不衰减）；
// 无 maneuver 元数据的参考（合成用例）不豁免任何点
TEST(AlOuterLoopTest, AnnealExemptMaskSingleManeuverAllExempt) {
    AlOuterLoop loop(AlOuterLoopConfig{}, DdpCostConfig{});
    const auto single =
        MakeReference(MakeLinePoses(31, 0.05), {MakeManeuver(1, 0, 30)});
    const auto mask_single = loop.makeAnnealExemptMask(single);
    ASSERT_EQ(mask_single.size(), 31);
    for (const bool exempt : mask_single) {
        EXPECT_TRUE(exempt);
    }
    const auto no_maneuver = MakeReference(MakeLinePoses(31, 0.05), {});
    const auto mask_empty = loop.makeAnnealExemptMask(no_maneuver);
    ASSERT_EQ(mask_empty.size(), 31);
    for (const bool exempt : mask_empty) {
        EXPECT_FALSE(exempt);
    }
}

// 测试跟踪权重退火调度：w_ref(r) = w_ref,0·γ^r，随 update 推进轮次逐轮
// 几何衰减（豁免点不衰减由掩码保证，不在本用例范围）
// 测试候选待融段掩码的生成规则：临界比 crit=T⁵·n_pts·dt 低于阈值的
// 内部 maneuver 覆盖点被标记；首/末段（融化保护：承载起点状态与终点
// 语义）与高临界比内部段不标记；无 maneuver 元数据的参考不标记任何点
TEST(AlOuterLoopTest, MeltCandidateMaskMarksOnlyLowCritInteriorManeuvers) {
    AlOuterLoopConfig config;
    config.melt_crit_threshold = 5000.0;
    const DdpCostConfig cost_config;
    AlOuterLoop loop(config, cost_config);
    // 5 个 maneuver（N=266）：首段 [0,60]（T=6，crit≈4.7e4，本就超阈）
    // / 微段 [60,66]（T=0.6，crit≈0.054）/ 大段 [66,186]（T=12，crit 巨大）
    // / 中段 [186,216]（T=3，crit≈753）/ 末段 [216,266]（保护）
    auto poses = MakeLinePoses(267, 0.05);
    const std::vector<DdpReferenceManeuver> maneuvers = {
        MakeManeuver(1, 0, 60), MakeManeuver(-1, 60, 66),
        MakeManeuver(1, 66, 186), MakeManeuver(-1, 186, 216),
        MakeManeuver(1, 216, 266)};
    const auto reference = MakeReference(poses, maneuvers);
    const auto mask = loop.makeMeltCandidateMask(reference);
    ASSERT_EQ(mask.size(), 267);
    // 首/末段保护：无论临界比如何都不参与候选标记（边界共享点归候选段
    // 的闭区间覆盖，与豁免掩码同一约定：点 60/216 被候选段标记）
    for (std::size_t k = 0; k < 60; ++k) {
        EXPECT_FALSE(mask[k]) << "first maneuver point " << k;
    }
    for (std::size_t k = 217; k <= 266; ++k) {
        EXPECT_FALSE(mask[k]) << "last maneuver point " << k;
    }
    // 低临界比内部段（微段/中段）整段标记（含与邻段共享的边界点）
    for (std::size_t k = 60; k <= 66; ++k) {
        EXPECT_TRUE(mask[k]) << "micro maneuver point " << k;
    }
    for (std::size_t k = 186; k <= 216; ++k) {
        EXPECT_TRUE(mask[k]) << "mid maneuver point " << k;
    }
    // 高临界比内部段不标记
    for (std::size_t k = 67; k < 186; ++k) {
        EXPECT_FALSE(mask[k]) << "large maneuver point " << k;
    }
    // 无元数据参考（合成用例）：不标记任何点
    const auto no_meta = loop.makeMeltCandidateMask(
        MakeReference(poses, std::vector<DdpReferenceManeuver>{}));
    ASSERT_EQ(no_meta.size(), 267);
    for (const bool flag : no_meta) {
        EXPECT_FALSE(flag);
    }
}

// 测试候选段退火权重的独立调度：候选点按 γ_cand 快速衰减（深退火把
// 「是否值得保留」的裁决权交还平滑项），与全局 γ 解耦
TEST(AlOuterLoopTest, CandidateTrackingWeightAnnealsIndependently) {
    AlOuterLoopConfig config;
    config.anneal_gamma = 0.5;
    config.candidate_anneal_gamma = 0.25;
    DdpCostConfig cost_config;
    cost_config.weight_ref_base = 10.0;
    AlOuterLoop loop(config, cost_config);
    EXPECT_DOUBLE_EQ(loop.candidateTrackingWeight(), 10.0);
    const std::size_t num_steps = 1;
    auto multipliers = loop.makeInitialMultipliers(num_steps);
    loop.update(MakeTerminalOnlySnapshot(0.1), 50.0, &multipliers);
    EXPECT_DOUBLE_EQ(loop.candidateTrackingWeight(), 2.5);
    EXPECT_DOUBLE_EQ(loop.trackingWeight(), 5.0);
    loop.update(MakeTerminalOnlySnapshot(0.01), 50.0, &multipliers);
    EXPECT_DOUBLE_EQ(loop.candidateTrackingWeight(), 0.625);
    EXPECT_DOUBLE_EQ(loop.trackingWeight(), 2.5);
}

// 测试退火终点自适应（逃逸指标冻结）：任一启用指标越阈即冻结退火——
// w_ref 停在当前深度不再衰减（全路段同时、无段间对拉）；冻结期间 μ
// 增长同步冻结（λ 继续累积——AL 本职机制），避免「违反度平台期被判
// 未充分下降」的伪 μ 增长；冻结滞回（reset 前不自动解冻）；默认全关
// 时上报零副作用
TEST(AlOuterLoopTest, AnnealFreezeStopsScheduleOnly) {
    AlOuterLoopConfig config;
    config.anneal_gamma = 0.5;
    config.anneal_freeze_length_growth = 1.05;
    config.anneal_freeze_lateral_deviation = 1.0;
    config.anneal_freeze_defect = 0.3;
    config.amplitude_mu_initial = 1000.0;
    DdpCostConfig cost_config;
    cost_config.weight_ref_base = 10.0;
    AlOuterLoop loop(config, cost_config);
    auto multipliers = loop.makeInitialMultipliers(1);
    EXPECT_FALSE(loop.annealFrozen());
    // 第 0 轮正常退火：update 后 w_ref 减半、μ 按门控正常推进
    loop.update(MakeTerminalOnlySnapshot(0.1), 50.0, &multipliers);
    EXPECT_DOUBLE_EQ(loop.trackingWeight(), 5.0);
    // 触发冻结（长度环比 1.2 > 1.05 阈值）→ 冻结标志置位
    loop.reportEscapeIndicators(1.2, 0.0, 0.0, 0.0);
    EXPECT_TRUE(loop.annealFrozen());
    // 冻结后的 update：w_ref 停在 5.0 不再衰减；冻结只停退火——违反度
    // 未充分下降时 μ 门控增长照常（AL 本职工作，冻结发生在求解中段）
    const double mu_before = loop.mu();
    const bool increased =
        loop.update(MakeTerminalOnlySnapshot(0.1), 50.0, &multipliers);
    EXPECT_TRUE(increased);
    EXPECT_DOUBLE_EQ(loop.trackingWeight(), 5.0);
    EXPECT_GT(loop.mu(), mu_before);
    // λ 照常累积（AL 本职机制不受冻结影响）
    EXPECT_GT(multipliers.terminal_lambda(0), 0.0);
    // 冻结滞回：继续 update 仍冻结（不自动解冻）
    loop.update(MakeTerminalOnlySnapshot(0.08), 50.0, &multipliers);
    EXPECT_TRUE(loop.annealFrozen());
    EXPECT_DOUBLE_EQ(loop.trackingWeight(), 5.0);
    // reset 解除冻结，调度从头开始
    loop.reset();
    EXPECT_FALSE(loop.annealFrozen());
    EXPECT_DOUBLE_EQ(loop.trackingWeight(), 10.0);
}

// 测试逃逸指标的越阈语义：未启用（=0）的指标不参与判定；四个指标任一
// 独立越阈均触发冻结；非法阈值（负数）构造拒绝
TEST(AlOuterLoopTest, AnnealFreezeIndicatorSemantics) {
    // 全部关闭（默认）：任何上报都不冻结
    {
        AlOuterLoop loop(AlOuterLoopConfig{}, DdpCostConfig{});
        loop.reportEscapeIndicators(1e9, 1e9, 1e9, 0.0);
        EXPECT_FALSE(loop.annealFrozen());
    }
    // 仅长度指标启用：仅长度越阈触发，其余指标越阈不触发
    {
        AlOuterLoopConfig config;
        config.anneal_freeze_length_growth = 1.1;
        AlOuterLoop loop(config, DdpCostConfig{});
        loop.reportEscapeIndicators(1.0, 1e9, 1e9, 0.0);
        EXPECT_FALSE(loop.annealFrozen());
        loop.reportEscapeIndicators(1.2, 0.0, 0.0, 0.0);
        EXPECT_TRUE(loop.annealFrozen());
    }
    // 仅偏离指标启用
    {
        AlOuterLoopConfig config;
        config.anneal_freeze_lateral_deviation = 0.5;
        AlOuterLoop loop(config, DdpCostConfig{});
        loop.reportEscapeIndicators(0.0, 0.4, 0.0, 0.0);
        EXPECT_FALSE(loop.annealFrozen());
        loop.reportEscapeIndicators(0.0, 0.6, 0.0, 0.0);
        EXPECT_TRUE(loop.annealFrozen());
    }
    // 仅缺陷指标启用
    {
        AlOuterLoopConfig config;
        config.anneal_freeze_defect = 0.3;
        AlOuterLoop loop(config, DdpCostConfig{});
        loop.reportEscapeIndicators(0.0, 0.0, 0.2, 0.0);
        EXPECT_FALSE(loop.annealFrozen());
        loop.reportEscapeIndicators(0.0, 0.0, 0.4, 0.0);
        EXPECT_TRUE(loop.annealFrozen());
    }
    // 仅绝对长度比指标启用
    {
        AlOuterLoopConfig config;
        config.anneal_freeze_ref_length_ratio = 1.2;
        AlOuterLoop loop(config, DdpCostConfig{});
        loop.reportEscapeIndicators(1e9, 1e9, 1e9, 1.1);
        EXPECT_FALSE(loop.annealFrozen());
        loop.reportEscapeIndicators(0.0, 0.0, 0.0, 1.3);
        EXPECT_TRUE(loop.annealFrozen());
    }
    // 非法阈值显式拒绝
    AlOuterLoopConfig bad;
    bad.anneal_freeze_length_growth = -1.0;
    EXPECT_THROW(AlOuterLoop(bad, DdpCostConfig{}), std::invalid_argument);
    bad = AlOuterLoopConfig{};
    bad.anneal_freeze_lateral_deviation = -0.1;
    EXPECT_THROW(AlOuterLoop(bad, DdpCostConfig{}), std::invalid_argument);
    bad = AlOuterLoopConfig{};
    bad.anneal_freeze_defect = -0.1;
    EXPECT_THROW(AlOuterLoop(bad, DdpCostConfig{}), std::invalid_argument);
    bad = AlOuterLoopConfig{};
    bad.anneal_freeze_ref_length_ratio = -0.1;
    EXPECT_THROW(AlOuterLoop(bad, DdpCostConfig{}), std::invalid_argument);
}

TEST(AlOuterLoopTest, TrackingWeightAnnealsGeometricallyWithRound) {
    AlOuterLoopConfig config;
    config.anneal_gamma = 0.5;
    DdpCostConfig cost_config;
    cost_config.weight_ref_base = 10.0;
    AlOuterLoop loop(config, cost_config);
    EXPECT_DOUBLE_EQ(loop.trackingWeight(), 10.0);
    const std::size_t num_steps = 1;
    auto multipliers = loop.makeInitialMultipliers(num_steps);
    // 两轮良性更新（违反度持续充分下降，μ 只在首轮无条件增长一次）
    loop.update(MakeTerminalOnlySnapshot(0.1), 50.0, &multipliers);
    EXPECT_DOUBLE_EQ(loop.trackingWeight(), 5.0);
    loop.update(MakeTerminalOnlySnapshot(0.01), 50.0, &multipliers);
    EXPECT_DOUBLE_EQ(loop.trackingWeight(), 2.5);
    EXPECT_EQ(loop.round(), 2);
}

// 测试换挡代理门宽 β 的逐轮退火调度：β(r)=max(β_final, β_initial·γ_β^r)
// ——宽门启动（梯度覆盖大 |v| 范围、非凸项可优化），逐轮收窄到地板值
// （逼近阶跃的换挡判决）；到达地板后保持，不再继续收窄（β→0 的梯度在
// v=0 处爆炸，必须留地板）
TEST(AlOuterLoopTest, ShiftBetaAnnealsAndFloorsPerRound) {
    AlOuterLoopConfig config;
    config.shift_beta_initial = 0.3;
    config.shift_beta_final = 0.05;
    config.shift_beta_gamma = 0.5;
    const DdpCostConfig cost_config;
    AlOuterLoop loop(config, cost_config);
    EXPECT_DOUBLE_EQ(loop.shiftBeta(), 0.3);
    const std::size_t num_steps = 1;
    auto multipliers = loop.makeInitialMultipliers(num_steps);
    loop.update(MakeTerminalOnlySnapshot(0.1), 50.0, &multipliers);
    EXPECT_DOUBLE_EQ(loop.shiftBeta(), 0.15);
    loop.update(MakeTerminalOnlySnapshot(0.01), 50.0, &multipliers);
    EXPECT_DOUBLE_EQ(loop.shiftBeta(), 0.075);
    loop.update(MakeTerminalOnlySnapshot(0.001), 50.0, &multipliers);
    // 0.3·0.5³=0.0375 < β_final → 地板 0.05
    EXPECT_DOUBLE_EQ(loop.shiftBeta(), 0.05);
    loop.update(MakeTerminalOnlySnapshot(0.0001), 50.0, &multipliers);
    EXPECT_DOUBLE_EQ(loop.shiftBeta(), 0.05);
}

// 测试分段常数退火：前 anneal_hold_rounds 轮保持 w_ref,0（先让 AL 把
// 约束建立起来），之后按 γ 几何退火；hold=0 时退化为纯几何退火
TEST(AlOuterLoopTest, TrackingWeightHoldsThenAnneals) {
    AlOuterLoopConfig config;
    config.anneal_gamma = 0.3;
    config.anneal_hold_rounds = 3;
    DdpCostConfig cost_config;
    cost_config.weight_ref_base = 10.0;
    AlOuterLoop loop(config, cost_config);
    const std::size_t num_steps = 1;
    auto multipliers = loop.makeInitialMultipliers(num_steps);
    // 保持期（r=0..3）：恒为 w_ref,0（保持 k 轮后首个退火值出现在 r=k+1）
    EXPECT_DOUBLE_EQ(loop.trackingWeight(), 10.0);
    loop.update(MakeTerminalOnlySnapshot(0.1), 50.0, &multipliers);
    EXPECT_DOUBLE_EQ(loop.trackingWeight(), 10.0);
    loop.update(MakeTerminalOnlySnapshot(0.09), 50.0, &multipliers);
    EXPECT_DOUBLE_EQ(loop.trackingWeight(), 10.0);
    loop.update(MakeTerminalOnlySnapshot(0.08), 50.0, &multipliers);
    EXPECT_DOUBLE_EQ(loop.trackingWeight(), 10.0);
    // 退火期（r=4 起）：w_ref,0·γ^(r−hold)
    loop.update(MakeTerminalOnlySnapshot(0.07), 50.0, &multipliers);
    EXPECT_DOUBLE_EQ(loop.trackingWeight(), 3.0);
    loop.update(MakeTerminalOnlySnapshot(0.06), 50.0, &multipliers);
    EXPECT_DOUBLE_EQ(loop.trackingWeight(), 0.9);
    // 非法保持轮数显式拒绝
    AlOuterLoopConfig bad;
    bad.anneal_hold_rounds = -1;
    EXPECT_THROW(AlOuterLoop(bad, cost_config), std::invalid_argument);
}

// 测试自适应 μ⁰ 标定：首轮内层收敛后按 μ⁰=clip(J_s′/max(‖c‖²,ε),μ_min,
// μ_max) 替换临时权重，乘子更新即以标定的 μ⁰ 执行；覆盖正常值/下界
// clip/上界 clip/残差趋零防退化四种情形
TEST(AlOuterLoopTest, FirstUpdateCalibratesAdaptiveMu) {
    const DdpCostConfig cost_config;
    // 正常值：J_s′=50、‖c‖²=0.01 → μ⁰ = 5000 ∈ [1e2, 1e6]
    {
        AlOuterLoop loop(AlOuterLoopConfig{}, cost_config);
        auto multipliers = loop.makeInitialMultipliers(1);
        loop.update(MakeTerminalOnlySnapshot(0.1), 50.0, &multipliers);
        EXPECT_DOUBLE_EQ(loop.calibratedMu(), 5000.0);
        // λ 更新使用标定的 μ⁰：等式 λ = μ⁰·c
        EXPECT_DOUBLE_EQ(multipliers.terminal_lambda(0), 500.0);
        // 标定轮只标定不增长：μ 保持 μ⁰，计数器为 0
        EXPECT_DOUBLE_EQ(loop.mu(), 5000.0);
        EXPECT_DOUBLE_EQ(multipliers.terminal_mu(0), 5000.0);
        EXPECT_EQ(loop.mu_increase_count(), 0);
    }
    // 下界 clip：J_s′ 过小 → μ⁰ = μ_min = 100
    {
        AlOuterLoop loop(AlOuterLoopConfig{}, cost_config);
        auto multipliers = loop.makeInitialMultipliers(1);
        loop.update(MakeTerminalOnlySnapshot(0.1), 0.5, &multipliers);
        EXPECT_DOUBLE_EQ(loop.calibratedMu(), 100.0);
        EXPECT_DOUBLE_EQ(loop.mu(), 100.0);
    }
    // 上界 clip：J_s′ 过大 → μ⁰ = μ_max = 1e6，增长后仍钉在 μ_max
    {
        AlOuterLoop loop(AlOuterLoopConfig{}, cost_config);
        auto multipliers = loop.makeInitialMultipliers(1);
        loop.update(MakeTerminalOnlySnapshot(0.1), 1e9, &multipliers);
        EXPECT_DOUBLE_EQ(loop.calibratedMu(), 1e6);
        EXPECT_DOUBLE_EQ(loop.mu(), 1e6);
    }
    // 残差趋零：max(‖c‖², ε_μ) 防分母退化
    {
        AlOuterLoop loop(AlOuterLoopConfig{}, cost_config);
        auto multipliers = loop.makeInitialMultipliers(1);
        loop.update(MakeTerminalOnlySnapshot(0.0), 50.0, &multipliers);
        EXPECT_DOUBLE_EQ(loop.calibratedMu(), 50.0 / 1e-4);
    }
}

// 测试乘子更新的 ALTRO 规则：等式 λ ← λ+μc（符号自由）、不等式
// λ ← max(0, λ+μg)（投影恒非负；g<0 且 λ=0 时保持不激活，λ>0 时随
// 持续可行逐步回落到零）
TEST(AlOuterLoopTest, MultiplierUpdatesFollowAltroRules) {
    AlOuterLoopConfig config;
    config.amplitude_mu_initial = 1000.0;
    AlOuterLoop loop(config, DdpCostConfig{});
    auto multipliers = loop.makeInitialMultipliers(1);
    AlConstraintSnapshot snapshot;
    snapshot.amplitude_g = Eigen::VectorXd::Zero(DDP_AMPLITUDE_CONSTRAINT_DIM);
    snapshot.amplitude_g(DDP_AMP_V) = 0.2;
    snapshot.amplitude_g(DDP_AMP_A) = -0.1;
    snapshot.terminal_c << 0.1, -0.2, 0.05, 0.01, -0.03;
    snapshot.violation_norm = 1.0;
    // 首轮：μ⁰ = clip(50/(0.01+0.04+0.0025+0.0001+0.0009), 100, 1e6) = clip(
    // 50/0.0535, ...) ≈ 934.58
    loop.update(snapshot, 50.0, &multipliers);
    const double mu0 = loop.calibratedMu();
    EXPECT_NEAR(mu0, 50.0 / 0.0535, 1e-6);
    // 等式：λ = μ⁰·c（逐元素、符号自由）
    for (int i = 0; i < DDP_TERMINAL_CONSTRAINT_DIM; ++i) {
        EXPECT_NEAR(multipliers.terminal_lambda(i),
                    mu0 * snapshot.terminal_c(i), 1e-6);
    }
    // 不等式：g>0 时 λ = μ_amp·g > 0（μ_amp 取强启动值 1e3，不参与终点
    // 标定）；g<0 且 λ=0 时保持 0（不激活）
    EXPECT_NEAR(multipliers.amplitude_lambda(DDP_AMP_V), 1000.0 * 0.2, 1e-6);
    EXPECT_DOUBLE_EQ(multipliers.amplitude_lambda(DDP_AMP_A), 0.0);
    // 次轮：构造使已激活约束持续可行的快照——g_v 转负且幅值足以把 λ 压回
    // 零（投影钳制）；另一约束 λ 部分回落后仍保持非负
    AlOuterLoop loop2(config, DdpCostConfig{});
    auto multipliers2 = loop2.makeInitialMultipliers(2);
    multipliers2.amplitude_lambda(DDP_AMP_V) = 100.0;
    multipliers2.amplitude_lambda(DDP_AMP_A) = 1.0;
    AlConstraintSnapshot snapshot2;
    snapshot2.amplitude_g =
        Eigen::VectorXd::Constant(2 * DDP_AMPLITUDE_CONSTRAINT_DIM, -1.0);
    snapshot2.amplitude_g(DDP_AMP_A) = -1e-6;
    snapshot2.terminal_c.setZero();
    snapshot2.violation_norm = 0.5;
    loop2.update(snapshot2, 50.0, &multipliers2);
    // λ_v = max(0, 100 + μ_amp·(−1.0)) = 0（μ_amp = 1e3 保证压回）
    EXPECT_DOUBLE_EQ(multipliers2.amplitude_lambda(DDP_AMP_V), 0.0);
    // λ_a = max(0, 1.0 + μ_amp·(−1e-6)) = 1.0 − 1e-3 > 0（部分回落）
    EXPECT_NEAR(multipliers2.amplitude_lambda(DDP_AMP_A), 1.0 - 1e-3, 1e-9);
    // 全部分量投影后恒非负
    for (Eigen::Index i = 0; i < multipliers2.amplitude_lambda.size(); ++i) {
        EXPECT_GE(multipliers2.amplitude_lambda(i), 0.0);
    }
}

// 测试门控 μ 增长（充分下降门控）：标定轮（首轮）只标定不增长；自次轮
// 起仅当本轮违反度 > κ·上轮时才 μ ← min(φμ, μ_max)；充分下降轮次 μ
// 不增长（计数器断言），避免盲目指数增长导致内层 Riccati 病态
TEST(AlOuterLoopTest, MuGrowthIsGatedBySufficientDecrease) {
    AlOuterLoopConfig config;
    config.mu_gate_kappa = 0.9;
    config.amplitude_mu_initial = 1000.0;
    AlOuterLoop loop(config, DdpCostConfig{});
    auto multipliers = loop.makeInitialMultipliers(1);
    // 首轮：标定 μ⁰ = 200/1.0 = 200，标定轮不增长，计数 0
    loop.update(MakeTerminalOnlySnapshot(1.0), 200.0, &multipliers);
    EXPECT_DOUBLE_EQ(loop.calibratedMu(), 200.0);
    EXPECT_DOUBLE_EQ(loop.mu(), 200.0);
    EXPECT_EQ(loop.mu_increase_count(), 0);
    // 次轮：违反度 0.85 ≤ 0.9·1.0 = 0.9，充分下降 → 不增长，计数保持 0
    const bool increased =
        loop.update(MakeTerminalOnlySnapshot(0.85), 200.0, &multipliers);
    EXPECT_FALSE(increased);
    EXPECT_DOUBLE_EQ(loop.mu(), 200.0);
    EXPECT_EQ(loop.mu_increase_count(), 0);
    // 第三轮：违反度 0.8 > 0.9·0.85 = 0.765，未充分下降 → 增长，计数 1
    const bool increased3 =
        loop.update(MakeTerminalOnlySnapshot(0.8), 200.0, &multipliers);
    EXPECT_TRUE(increased3);
    EXPECT_DOUBLE_EQ(loop.mu(), 2000.0);
    EXPECT_EQ(loop.mu_increase_count(), 1);
    //  μ 增长后写回乘子状态（下一轮内层即用新罚权重；幅值组违反度
    //  恒零、其门控不触发，μ_amp 保持强启动值 1e3）
    EXPECT_DOUBLE_EQ(multipliers.terminal_mu(0), 2000.0);
    EXPECT_DOUBLE_EQ(multipliers.amplitude_mu(0), 1000.0);
}

// 测试分组独立门控：幅值违反度停滞时仅 μ_amp 增长（终端残差充分下降
// 时 μ_term 不动）——单点顽固的物理边界违反不得把终端罚权重一并
// 推入病态，反之亦然
TEST(AlOuterLoopTest, MuGrowthGatesAreIndependentPerGroup) {
    AlOuterLoopConfig config;
    config.amplitude_mu_initial = 1000.0;
    AlOuterLoop loop(config, DdpCostConfig{});
    auto multipliers = loop.makeInitialMultipliers(1);
    // 首轮：标定（μ_term = 100/0.25 = 400），记录基线违反度
    AlConstraintSnapshot snapshot;
    snapshot.amplitude_g =
        Eigen::VectorXd::Constant(DDP_AMPLITUDE_CONSTRAINT_DIM, -1.0);
    snapshot.amplitude_g(DDP_AMP_DELTA_POS) = 0.3;
    snapshot.terminal_c << 0.5, 0.0, 0.0, 0.0, 0.0;
    loop.update(snapshot, 100.0, &multipliers);
    EXPECT_DOUBLE_EQ(loop.mu(), 400.0);
    EXPECT_DOUBLE_EQ(loop.muAmplitude(), 1000.0);
    EXPECT_EQ(loop.mu_increase_count(), 0);
    // 次轮：终端残差 0.5→0.1（充分下降）、幅值违反 0.3→0.29（停滞，
    // 0.29 > 0.9·0.3=0.27）→ 仅 μ_amp 增长（×φ），μ_term 不动
    snapshot.terminal_c << 0.1, 0.0, 0.0, 0.0, 0.0;
    snapshot.amplitude_g(DDP_AMP_DELTA_POS) = 0.29;
    const bool increased = loop.update(snapshot, 100.0, &multipliers);
    EXPECT_TRUE(increased);
    EXPECT_DOUBLE_EQ(loop.mu(), 400.0);
    EXPECT_DOUBLE_EQ(loop.muAmplitude(), 10000.0);
    EXPECT_EQ(loop.mu_increase_count(), 1);
    // 第三轮：幅值违反 0.29→0.1、终端残差 0.1→0.05（均充分下降）→
    // 两组均不增长
    snapshot.amplitude_g(DDP_AMP_DELTA_POS) = 0.1;
    snapshot.terminal_c << 0.05, 0.0, 0.0, 0.0, 0.0;
    const bool increased3 = loop.update(snapshot, 100.0, &multipliers);
    EXPECT_FALSE(increased3);
    EXPECT_DOUBLE_EQ(loop.mu(), 400.0);
    EXPECT_DOUBLE_EQ(loop.muAmplitude(), 10000.0);
    EXPECT_EQ(loop.mu_increase_count(), 1);
}

// 测试 μ 增长的 μ_max 封顶：接近上界时 min(φμ, μ_max) 钉在边界，
// 罚权重不越过病态阈值
TEST(AlOuterLoopTest, MuGrowthClippedAtMuMax) {
    AlOuterLoop loop(AlOuterLoopConfig{}, DdpCostConfig{});
    auto multipliers = loop.makeInitialMultipliers(1);
    // 标定 μ⁰ = clip(5e6/0.01, 1e2, 1e6) = 1e6（上界 clip），标定轮不增长
    loop.update(MakeTerminalOnlySnapshot(0.1), 5e6, &multipliers);
    EXPECT_DOUBLE_EQ(loop.calibratedMu(), 1e6);
    EXPECT_DOUBLE_EQ(loop.mu(), 1e6);
    // 后续违反度不下降（0.1 > 0.9·0.1）触发增长，但仍封顶 1e6
    loop.update(MakeTerminalOnlySnapshot(0.1), 5e6, &multipliers);
    EXPECT_DOUBLE_EQ(loop.mu(), 1e6);
    EXPECT_EQ(loop.mu_increase_count(), 1);
    EXPECT_DOUBLE_EQ(multipliers.terminal_mu(0), 1e6);
}

// 测试幅值组的独立 μ 上限：幅值组增长封顶 amplitude_mu_max（终端组仍可
// 到全局 μ_max）——长视窗死亡螺旋由两组共用同一 μ_max 的指数攀升驱动；
// 幅值组独立封顶后 λ 按 μ·g 持续累积（AL 的本职机制，罚中心逐轮内移），
// 内层 Riccati 不再被无界罚权重推入病态。amplitude_mu_max 默认等于
// mu_max（既有行为不变），且不得小于幅值初始罚权重
TEST(AlOuterLoopTest, AmplitudeGroupHasIndependentMuCap) {
    AlOuterLoopConfig config;
    config.amplitude_mu_initial = 1000.0;
    config.amplitude_mu_max = 1e4;
    AlOuterLoop loop(config, DdpCostConfig{});
    auto multipliers = loop.makeInitialMultipliers(1);
    // 首轮：标定（μ_term=400），记录基线违反度；幅值违反 0.3
    AlConstraintSnapshot snapshot;
    snapshot.amplitude_g =
        Eigen::VectorXd::Constant(DDP_AMPLITUDE_CONSTRAINT_DIM, -1.0);
    snapshot.amplitude_g(DDP_AMP_DELTA_POS) = 0.3;
    snapshot.terminal_c << 0.5, 0.0, 0.0, 0.0, 0.0;
    loop.update(snapshot, 100.0, &multipliers);
    EXPECT_DOUBLE_EQ(loop.mu(), 400.0);
    EXPECT_DOUBLE_EQ(loop.muAmplitude(), 1000.0);
    // 次轮：幅值违反停滞（0.29 > 0.9·0.3）→ μ_amp 1000→1e4（未触顶）；
    // 终端充分下降不增长
    snapshot.terminal_c << 0.1, 0.0, 0.0, 0.0, 0.0;
    snapshot.amplitude_g(DDP_AMP_DELTA_POS) = 0.29;
    loop.update(snapshot, 100.0, &multipliers);
    EXPECT_DOUBLE_EQ(loop.muAmplitude(), 10000.0);
    // 第三轮：幅值违反仍停滞 → min(φ·1e4, 1e4) 钉在独立上限 1e4
    // （全局 μ_max=1e6 不约束幅值组）；终端组违反停滞则继续增长
    snapshot.amplitude_g(DDP_AMP_DELTA_POS) = 0.28;
    snapshot.terminal_c << 0.095, 0.0, 0.0, 0.0, 0.0;
    loop.update(snapshot, 100.0, &multipliers);
    EXPECT_DOUBLE_EQ(loop.muAmplitude(), 10000.0);
    EXPECT_DOUBLE_EQ(loop.mu(), 4000.0);
    EXPECT_DOUBLE_EQ(multipliers.amplitude_mu(0), 10000.0);
    // 非法配置：独立上限小于幅值初始罚权重 → 构造拒绝
    AlOuterLoopConfig bad;
    bad.amplitude_mu_initial = 1000.0;
    bad.amplitude_mu_max = 100.0;
    EXPECT_THROW(AlOuterLoop(bad, DdpCostConfig{}), std::invalid_argument);
}

// 测试联合终止判据：终点双指标 + 状态不等式违反度 + 缺陷范数三类
// 分项任一不达标即不收敛；边界取等视为达标（≤ 语义）
TEST(AlOuterLoopTest, TerminationCheckCombinesAllCriteria) {
    AlOuterLoopConfig config;
    config.terminal_position_tol = 0.05;
    config.terminal_heading_tol_deg = 1.5;
    config.inequality_tol = 1e-2;
    config.defect_tol = 1e-3;
    AlOuterLoop loop(config, DdpCostConfig{});
    AlConstraintSnapshot ok;
    const auto check_ok = loop.checkTermination(ok);
    EXPECT_TRUE(check_ok.terminal_ok);
    EXPECT_TRUE(check_ok.inequality_ok);
    EXPECT_TRUE(check_ok.defect_ok);
    EXPECT_TRUE(check_ok.converged());
    // 终点位置超标
    auto snapshot = ok;
    snapshot.terminal_position_error = 0.06;
    EXPECT_FALSE(loop.checkTermination(snapshot).converged());
    EXPECT_FALSE(loop.checkTermination(snapshot).terminal_ok);
    // 终点朝向超标
    snapshot = ok;
    snapshot.terminal_heading_error_deg = 1.6;
    EXPECT_FALSE(loop.checkTermination(snapshot).converged());
    // 双指标边界取等达标
    snapshot = ok;
    snapshot.terminal_position_error = 0.05;
    snapshot.terminal_heading_error_deg = 1.5;
    EXPECT_TRUE(loop.checkTermination(snapshot).converged());
    // 状态不等式违反度超标
    snapshot = ok;
    snapshot.max_amplitude_violation = 0.02;
    const auto check_ineq = loop.checkTermination(snapshot);
    EXPECT_FALSE(check_ineq.inequality_ok);
    EXPECT_FALSE(check_ineq.converged());
    // 缺陷范数超标
    snapshot = ok;
    snapshot.defect_norm_inf = 2e-3;
    const auto check_defect = loop.checkTermination(snapshot);
    EXPECT_FALSE(check_defect.defect_ok);
    EXPECT_FALSE(check_defect.converged());
}

// 测试调度状态复位：同一实例多次求解复用时，reset 清空轮次/违反度
// 历史/计数器/标定标志，下一次 update 重新执行首轮标定与无条件增长
TEST(AlOuterLoopTest, ResetRestoresInitialSchedulingState) {
    AlOuterLoopConfig config;
    config.first_round_mu = 1.0;
    AlOuterLoop loop(config, DdpCostConfig{});
    auto multipliers = loop.makeInitialMultipliers(1);
    loop.update(MakeTerminalOnlySnapshot(0.1), 50.0, &multipliers);
    ASSERT_EQ(loop.round(), 1);
    ASSERT_EQ(loop.mu_increase_count(), 0);
    ASSERT_NE(loop.mu(), config.first_round_mu);
    loop.reset();
    EXPECT_EQ(loop.round(), 0);
    EXPECT_EQ(loop.mu_increase_count(), 0);
    EXPECT_DOUBLE_EQ(loop.mu(), config.first_round_mu);
    // 复位后首轮重新标定（calibratedMu 随之刷新；标定轮不增长）
    loop.update(MakeTerminalOnlySnapshot(0.1), 200.0, &multipliers);
    EXPECT_DOUBLE_EQ(loop.calibratedMu(), 20000.0);
    EXPECT_DOUBLE_EQ(loop.mu(), 20000.0);
    EXPECT_EQ(loop.mu_increase_count(), 0);
}

// 测试乘子尺寸契约：update 对尺寸不符的乘子状态显式抛出，
// 禁止错位更新污染下一轮内层
TEST(AlOuterLoopTest, UpdateRejectsMismatchedMultiplierSizes) {
    AlOuterLoop loop(AlOuterLoopConfig{}, DdpCostConfig{});
    auto multipliers = loop.makeInitialMultipliers(1);
    multipliers.amplitude_lambda.resize(3);
    EXPECT_THROW(loop.update(MakeTerminalOnlySnapshot(0.1), 50.0, &multipliers),
                 std::invalid_argument);
}

}  // namespace
}  // namespace apa_post_processor
