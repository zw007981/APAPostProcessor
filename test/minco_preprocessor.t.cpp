#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include "core/MINCO/minco_maneuver_segmenter.h"
#include "core/MINCO/minco_preprocessor.h"

namespace apa_post_processor {
namespace {

// 测试用派生类：暴露受保护的问题装配与代价求值入口，供白盒梯度对拍
class MincoPreprocessorTestAccessor : public MincoPreprocessor {
   public:
    using MincoPreprocessor::MincoPreprocessor;
    using MincoPreprocessor::buildProblem;
    using MincoPreprocessor::evaluateCostAndGradient;
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

// 构造从原点出发沿 x 轴的直线前进路径
Path BuildStraightPath(double length) {
    Path path;
    path.addPoint({0.0, 0.0, 0.0});
    AppendXLine(&path, 0.0, length, 0.0);
    path.finalize();
    return path;
}

// 构造 前进1.0m → 后退0.5m 的单次换挡路径
Path BuildGearShiftPath() {
    Path path;
    path.addPoint({0.0, 0.0, 0.0});
    AppendXLine(&path, 0.0, 1.0, 0.0);
    AppendXLine(&path, 1.0, 0.5, 0.0);
    path.finalize();
    return path;
}

// 单 Maneuver 单段的最小合法初值估计（直行 1m）
std::vector<MincoManeuverEstimate> MakeSimpleEstimates() {
    std::vector<MincoManeuverEstimate> estimates(1);
    estimates[0].direction = Direction::FORWARD;
    estimates[0].start_theta = 0.0;
    estimates[0].start_arc_length = 0.0;
    estimates[0].segments = {{{1.0, 0.0}, 0.0, 1.0, 1.0}};
    return estimates;
}

// 测试直线单机动场景下的收敛性。
// 因为初值来自 MincoManeuverSegmenter 的真实解析结果且场景无障碍无转弯，
// 所以松收敛阈值下 L-BFGS 必须收敛，各段末端世界坐标必须贴近前端锚点，
// 终点硬边界（零速）必须精确满足。
TEST(MincoPreprocessorTest, StraightSingleManeuverConverges) {
    const MincoPreprocessor preprocessor(MincoConfig{});
    const Path path = BuildStraightPath(2.0);
    const std::vector<MincoManeuverEstimate> estimates =
        MincoManeuverSegmenter(MincoConfig{}).segment(path);
    const MincoPreprocessorResult result =
        preprocessor.preprocess(estimates, {0.0, 0.0});
    EXPECT_TRUE(result.optimizer_converged);
    EXPECT_TRUE(result.success);
    ASSERT_EQ(result.segment_end_positions.size(), 4U);
    EXPECT_LT(result.max_endpoint_error, 0.1);
    EXPECT_NEAR(result.segment_end_positions.back().x(), 2.0, 0.1);
    EXPECT_NEAR(result.segment_end_positions.back().y(), 0.0, 0.1);
    // 终点边界：速度/角速度均为 0（硬边界精确满足）
    const Eigen::Vector2d end_rate =
        result.trajectory.evaluate(result.trajectory.totalDuration(), 1);
    EXPECT_NEAR(end_rate.x(), 0.0, 1e-9);
    EXPECT_NEAR(end_rate.y(), 0.0, 1e-9);
    // 无换挡点，段时长均为正且与初值同量级
    EXPECT_EQ(result.max_cusp_speed, 0.0);
    ASSERT_EQ(result.durations.size(), 4U);
    for (const double duration : result.durations) {
        EXPECT_GT(duration, 0.0);
        EXPECT_LT(duration, 10.0);
    }
}

// 测试单次换挡两机动场景下的收敛性与换挡点残余速度。
// 因为换挡点 ṡ=0 在 MINCO 框架内只能以软惩罚近似，所以收敛后换挡点残余
// 速度必须被压到接近 0 的工程容差内，且两段末端位置都要贴近各自锚点。
TEST(MincoPreprocessorTest, GearShiftTwoManeuverConverges) {
    const MincoPreprocessor preprocessor(MincoConfig{});
    const Path path = BuildGearShiftPath();
    ASSERT_EQ(path.numManeuvers(), 2);
    const std::vector<MincoManeuverEstimate> estimates =
        MincoManeuverSegmenter(MincoConfig{}).segment(path);
    ASSERT_EQ(estimates.size(), 2U);
    const MincoPreprocessorResult result =
        preprocessor.preprocess(estimates, {0.0, 0.0});
    EXPECT_TRUE(result.optimizer_converged);
    EXPECT_TRUE(result.success);
    // M = 2（前进段）+ 1（后退段）= 3 段，2 个内部航点，1 个换挡点
    ASSERT_EQ(result.durations.size(), 3U);
    ASSERT_EQ(result.waypoints.size(), 2U);
    ASSERT_EQ(result.segment_end_positions.size(), 3U);
    EXPECT_LT(result.max_cusp_speed, 0.05);
    EXPECT_LT(result.max_endpoint_error, 0.1);
    EXPECT_NEAR(result.segment_end_positions.back().x(), 0.5, 0.1);
}

// 测试解析梯度与中心差分数值梯度对拍。
// 因为 J_pre 的梯度链（采样点代价 → 系数 c → 伴随 K(T)^{-T} → 航点/s_f，
// 以及 K(T) 行对 T 的解析依赖 → τ）横跨四个环节、任何一环出错都会被
// 放大，所以用独立于手推公式的有限差分做第三方验证；手工构造含换挡与
// 曲率的场景并压低物理上限，使四项物理惩罚处于激活区（可行区分支梯度
// 恒零，没有验证价值）。
TEST(MincoPreprocessorTest, AnalyticGradientMatchesFiniteDifference) {
    std::vector<MincoManeuverEstimate> estimates(2);
    estimates[0].direction = Direction::FORWARD;
    estimates[0].start_theta = 0.0;
    estimates[0].start_arc_length = 0.0;
    estimates[0].segments = {
        {{0.4, 0.3}, 0.6, 0.5, 0.8},
        {{0.9, 0.35}, 0.3, 1.0, 0.7},
    };
    estimates[1].direction = Direction::BACKWARD;
    estimates[1].start_theta = 0.3;
    estimates[1].start_arc_length = 1.0;
    estimates[1].segments = {
        {{0.55, 0.1}, 0.15, 0.6, 0.9},
    };
    // 压低物理上限使惩罚激活；权重取适中量级，控制代价在 O(10)~O(100)，
    // 抑制有限差分的舍入误差
    MincoConfig config;
    config.max_velocity = 0.5;
    config.max_acceleration = 0.5;
    config.max_steer_rate = 0.3;
    config.weight_endpoint_track = 5.0;
    config.pre_weight_velocity = 10.0;
    config.pre_weight_acceleration = 10.0;
    config.pre_weight_steer_angle = 10.0;
    config.pre_weight_steer_rate = 10.0;
    config.pre_weight_duration_balance = 2.0;
    config.pre_weight_gear_cusp = 10.0;
    config.pre_epsilon_time = 0.05;
    const MincoPreprocessorTestAccessor preprocessor(config);
    const MincoPreprocessorProblem problem =
        preprocessor.buildProblem(estimates, {0.0, 0.0});
    ASSERT_EQ(problem.numSegments(), 3);
    ASSERT_EQ(problem.variableCount(), 8);
    ASSERT_EQ(problem.cusp_segment_indices.size(), 1U);
    EXPECT_EQ(problem.cusp_segment_indices[0], 1);
    const Eigen::VectorXd x0 = problem.initialGuess();
    Eigen::VectorXd analytic(x0.size());
    preprocessor.evaluateCostAndGradient(problem, x0, &analytic);
    for (int k = 0; k < x0.size(); ++k) {
        const double step = 1e-6 * std::max(1.0, std::abs(x0[k]));
        Eigen::VectorXd x_plus = x0;
        Eigen::VectorXd x_minus = x0;
        x_plus[k] += step;
        x_minus[k] -= step;
        Eigen::VectorXd grad_scratch(x0.size());
        const double f_plus = preprocessor.evaluateCostAndGradient(
            problem, x_plus, &grad_scratch);
        const double f_minus = preprocessor.evaluateCostAndGradient(
            problem, x_minus, &grad_scratch);
        const double numeric = (f_plus - f_minus) / (2.0 * step);
        EXPECT_NEAR(
            analytic[k], numeric,
            1e-6 * std::max({1.0, std::abs(analytic[k]), std::abs(numeric)}))
            << "决策变量下标 " << k;
    }
}

// 测试段时长平衡约束生效。
// 因为初始时长被人为构造为病态悬殊分布（0.5/4.0/0.5，比值 8，超出
// [ε_low, ε_upp]=[0.5, 2.0] 倍均值的允许带），所以优化后段间时长比必须
// 被压回平衡约束的渐近边界（比值 4）附近。
TEST(MincoPreprocessorTest, DurationBalanceConstrainsSpread) {
    std::vector<MincoManeuverEstimate> estimates(1);
    estimates[0].direction = Direction::FORWARD;
    estimates[0].start_theta = 0.0;
    estimates[0].start_arc_length = 0.0;
    estimates[0].segments = {
        {{0.5, 0.0}, 0.0, 0.5, 0.5},
        {{1.0, 0.0}, 0.0, 1.0, 4.0},
        {{1.5, 0.0}, 0.0, 1.5, 0.5},
    };
    const MincoPreprocessor preprocessor(MincoConfig{});
    const MincoPreprocessorResult result =
        preprocessor.preprocess(estimates, {0.0, 0.0});
    ASSERT_TRUE(result.success);
    ASSERT_EQ(result.durations.size(), 3U);
    const auto [min_it, max_it] =
        std::minmax_element(result.durations.begin(), result.durations.end());
    const double ratio = *max_it / *min_it;
    const double mean =
        (result.durations[0] + result.durations[1] + result.durations[2]) / 3.0;
    EXPECT_LT(ratio, 4.5);
    EXPECT_GT(*min_it, 0.4 * mean);
    EXPECT_LT(*max_it, 2.2 * mean);
}

// 测试物理上无法满足跟踪目标时的显式失败反馈。
// 因为速度上限被压到 1e-3 m/s 且速度惩罚权重极大、时间正则抑制时长
// 膨胀，"移动"在任何时长下都比"不动"代价更高，优化器只能放弃跟踪
// 目标，所以必须显式报告失败（success=false 且跟踪误差大），而不是
// 静默返回一条看似正常的轨迹。误差下界锚定收敛容差语义（0.1 m）的 5
// 倍余量，不锚具体数值——终点跟踪权重变化只改变残余误差的量级（实测
// track=10 时 >1.0 m、track=20 时 ~0.88 m），不改变"远超容差而失败"
// 的判据
TEST(MincoPreprocessorTest, ConflictingLimitsReturnFailure) {
    MincoConfig config;
    config.max_velocity = 1e-3;
    config.pre_weight_velocity = 1e9;
    config.pre_epsilon_time = 1.0;
    const MincoPreprocessor preprocessor(config);
    const Path path = BuildStraightPath(2.0);
    const std::vector<MincoManeuverEstimate> estimates =
        MincoManeuverSegmenter(MincoConfig{}).segment(path);
    const MincoPreprocessorResult result =
        preprocessor.preprocess(estimates, {0.0, 0.0});
    EXPECT_FALSE(result.success);
    EXPECT_GT(result.max_endpoint_error, 0.5);
}

// 测试非法输入的异常反馈。
// 因为 J_pre 的装配依赖初值估计的全部字段（位置/朝向/弧长/时长），
// 所以任何空输入、非有限字段或非正时长都必须在装配期以标准异常明确
// 拒绝，而不是带着垃圾数据进入 L-BFGS。
TEST(MincoPreprocessorTest, InvalidInputsThrow) {
    const MincoPreprocessor preprocessor(MincoConfig{});
    // 空估计
    EXPECT_THROW(preprocessor.preprocess({}, {0.0, 0.0}),
                 std::invalid_argument);
    // 空 segments 的 Maneuver
    EXPECT_THROW(preprocessor.preprocess({MincoManeuverEstimate{}}, {0.0, 0.0}),
                 std::invalid_argument);
    // 非有限起点世界坐标
    EXPECT_THROW(preprocessor.preprocess(
                     MakeSimpleEstimates(),
                     {std::numeric_limits<double>::quiet_NaN(), 0.0}),
                 std::invalid_argument);
    // 非有限字段：朝向/弧长/目标点
    auto nan_theta = MakeSimpleEstimates();
    nan_theta[0].segments[0].theta = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(preprocessor.preprocess(nan_theta, {0.0, 0.0}),
                 std::invalid_argument);
    auto inf_position = MakeSimpleEstimates();
    inf_position[0].segments[0].desired_position.x() =
        std::numeric_limits<double>::infinity();
    EXPECT_THROW(preprocessor.preprocess(inf_position, {0.0, 0.0}),
                 std::invalid_argument);
    // 非正时长
    auto zero_duration = MakeSimpleEstimates();
    zero_duration[0].segments[0].duration = 0.0;
    EXPECT_THROW(preprocessor.preprocess(zero_duration, {0.0, 0.0}),
                 std::invalid_argument);
    auto negative_duration = MakeSimpleEstimates();
    negative_duration[0].segments[0].duration = -1.0;
    EXPECT_THROW(preprocessor.preprocess(negative_duration, {0.0, 0.0}),
                 std::invalid_argument);
}

// 测试非法配置的构造期校验。
// 因为配置字段直接决定 J_pre 的代价地形与求解行为，所以负权重、非法
// 采样数、奇数辛普森区间、倒挂的时长平衡系数等都必须在构造期拒绝。
TEST(MincoPreprocessorTest, InvalidConfigThrows) {
    EXPECT_NO_THROW(const MincoPreprocessor valid_preprocessor(MincoConfig{}));
    MincoConfig config;
    config.pre_weight_velocity = -1.0;
    EXPECT_THROW(const MincoPreprocessor p(config), std::invalid_argument);
    config = {};
    config.pre_duration_balance_lower = 2.0;
    EXPECT_THROW(const MincoPreprocessor p(config), std::invalid_argument);
    config = {};
    config.pre_simpson_subintervals = 3;
    EXPECT_THROW(const MincoPreprocessor p(config), std::invalid_argument);
    config = {};
    config.pre_physics_samples_per_segment = 1;
    EXPECT_THROW(const MincoPreprocessor p(config), std::invalid_argument);
    config = {};
    config.pre_lbfgs_max_iterations = 0;
    EXPECT_THROW(const MincoPreprocessor p(config), std::invalid_argument);
}

// 测试 config() 只读访问器返回构造时的配置值。
// 因为同模块其它类（分段器/运动学提取器/ESDF 惩罚）均提供该访问器供
// 下游（主优化）读取配置，所以返回值必须与构造入参逐字段一致。
TEST(MincoPreprocessorTest, ConfigAccessorReturnsConstructionValues) {
    MincoConfig config;
    config.weight_endpoint_track = 7.5;
    config.pre_simpson_subintervals = 12;
    config.convergence_position_tolerance = 0.2;
    const MincoPreprocessor preprocessor(config);
    EXPECT_DOUBLE_EQ(preprocessor.config().weight_endpoint_track, 7.5);
    EXPECT_EQ(preprocessor.config().pre_simpson_subintervals, 12);
    EXPECT_DOUBLE_EQ(preprocessor.config().convergence_position_tolerance, 0.2);
    EXPECT_DOUBLE_EQ(preprocessor.config().pre_weight_gear_cusp, 1000.0);
}

}  // namespace
}  // namespace apa_post_processor
