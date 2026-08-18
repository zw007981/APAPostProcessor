#include <gtest/gtest.h>

#include <Eigen/Core>
#include <cmath>
#include <stdexcept>
#include <vector>

#include "core/MINCO/minco_trajectory.h"

namespace apa_post_processor {
namespace {

using CoeffMatrix = MincoTrajectory::CoeffMatrix;

// 单段 5 阶多项式 Hermite 插值的手推解析系数（归一化 tau=t/T 基）。
// 由端点位置/速度/加速度六个约束直接解出，供系数级精度对拍。
Eigen::Matrix<double, 6, 1> BuildAnalyticHermiteCoeffs(
    const MincoBoundaryCondition& start, const MincoBoundaryCondition& end,
    double duration) {
    const double c0 = start.pos;
    const double c1 = start.vel * duration;
    const double c2 = 0.5 * start.acc * duration * duration;
    const double d_pos = end.pos - (c0 + c1 + c2);
    const double d_vel = end.vel * duration - (c1 + 2.0 * c2);
    const double d_acc = end.acc * duration * duration - 2.0 * c2;
    Eigen::Matrix<double, 6, 1> coeffs;
    coeffs << c0, c1, c2,                          //
        10.0 * d_pos - 4.0 * d_vel + 0.5 * d_acc,  //
        -15.0 * d_pos + 7.0 * d_vel - d_acc,       //
        6.0 * d_pos - 3.0 * d_vel + 0.5 * d_acc;   //
    return coeffs;
}

// 构造一组固定的多段测试输入（M=4），供边界/航点/连续性校验复用
void BuildMultiSegmentInput(MincoBoundaryCondition2d* start,
                            MincoBoundaryCondition2d* end,
                            std::vector<Eigen::Vector2d>* waypoints,
                            std::vector<double>* durations) {
    start->theta = {0.1, 0.2, 0.0};
    start->s = {0.0, 0.5, 0.2};
    end->theta = {2.0, -0.1, 0.05};
    end->s = {3.0, 0.4, -0.1};
    *waypoints = {{0.5, 0.7}, {1.5, 1.8}, {1.0, 2.2}};
    *durations = {0.8, 1.5, 2.0, 0.6};
}

// 按阶数取边界条件分量：0→pos、1→vel、2→acc
double ExpectedBoundaryComponent(const MincoBoundaryCondition& bc, int order) {
    if (order == 0) {
        return bc.pos;
    }
    if (order == 1) {
        return bc.vel;
    }
    return bc.acc;
}

// 测试用派生类：暴露受保护的静态基函数行，供白盒对拍
class MincoTrajectoryTestAccessor : public MincoTrajectory {
   public:
    using MincoTrajectory::DerivativeBasisRow;
};

// 测试单段轨迹的系数与手推 Hermite 解析解一致。
// 因为单段退化为标准两点 Hermite 插值、存在闭式解，所以系数必须逐位吻合。
TEST(MincoTrajectoryTest, SingleSegmentMatchesAnalyticHermiteSolution) {
    const MincoBoundaryCondition2d start{{0.3, 0.1, -0.05}, {0.0, 0.8, 0.1}};
    const MincoBoundaryCondition2d end{{1.2, -0.2, 0.3}, {2.4, 0.0, 0.0}};
    const std::vector<double> durations{1.7};
    MincoTrajectory trajectory;
    trajectory.setTrajectory(start, end, {}, durations);

    const Eigen::Matrix<double, 6, 1> expected_theta =
        BuildAnalyticHermiteCoeffs(start.theta, end.theta, 1.7);
    const Eigen::Matrix<double, 6, 1> expected_s =
        BuildAnalyticHermiteCoeffs(start.s, end.s, 1.7);

    ASSERT_EQ(trajectory.numSegments(), 1);
    EXPECT_LE((trajectory.coeffsTheta().col(0) - expected_theta)
                  .cwiseAbs()
                  .maxCoeff(),
              1e-9);
    EXPECT_LE((trajectory.coeffsS().col(0) - expected_s).cwiseAbs().maxCoeff(),
              1e-9);
}

// 测试单段轨迹按时刻求值还原端点边界条件。
// 因为求值是后续代价/梯度计算的唯一读取入口，所以 0~2 阶导数必须在两端
// 精确还原输入的 PVA。
TEST(MincoTrajectoryTest, SingleSegmentEvaluateMatchesBoundaryConditions) {
    const MincoBoundaryCondition2d start{{0.3, 0.1, -0.05}, {0.0, 0.8, 0.1}};
    const MincoBoundaryCondition2d end{{1.2, -0.2, 0.3}, {2.4, 0.0, 0.0}};
    const std::vector<double> durations{1.7};
    MincoTrajectory trajectory;
    trajectory.setTrajectory(start, end, {}, durations);

    const double total = trajectory.totalDuration();
    EXPECT_DOUBLE_EQ(total, 1.7);
    for (int order = 0; order <= 2; ++order) {
        const Eigen::Vector2d at_start = trajectory.evaluate(0.0, order);
        const Eigen::Vector2d at_end = trajectory.evaluate(total, order);
        EXPECT_NEAR(at_start.x(), ExpectedBoundaryComponent(start.theta, order),
                    1e-9);
        EXPECT_NEAR(at_start.y(), ExpectedBoundaryComponent(start.s, order),
                    1e-9);
        EXPECT_NEAR(at_end.x(), ExpectedBoundaryComponent(end.theta, order),
                    1e-9);
        EXPECT_NEAR(at_end.y(), ExpectedBoundaryComponent(end.s, order), 1e-9);
    }
}

// 测试多段轨迹同时满足端点边界、内部航点与 1~4 阶导数连续性。
// 因为 K(T) 装配的全部语义就是这组约束，所以逐条满足即证明装配正确。
TEST(MincoTrajectoryTest, MultiSegmentSatisfiesAllConstraints) {
    MincoBoundaryCondition2d start, end;
    std::vector<Eigen::Vector2d> waypoints;
    std::vector<double> durations;
    BuildMultiSegmentInput(&start, &end, &waypoints, &durations);
    MincoTrajectory trajectory;
    trajectory.setTrajectory(start, end, waypoints, durations);
    const int num_segments = trajectory.numSegments();
    ASSERT_EQ(num_segments, 4);

    // 起点/终点 PVA 精确还原（θ 与 s 两维、起点与终点全部覆盖）
    for (int order = 0; order <= 2; ++order) {
        const Eigen::Vector2d at_start =
            trajectory.evaluateSegment(0, 0.0, order);
        const Eigen::Vector2d at_end = trajectory.evaluateSegment(
            num_segments - 1, durations[num_segments - 1], order);
        EXPECT_NEAR(at_start.x(), ExpectedBoundaryComponent(start.theta, order),
                    1e-9);
        EXPECT_NEAR(at_start.y(), ExpectedBoundaryComponent(start.s, order),
                    1e-9);
        EXPECT_NEAR(at_end.x(), ExpectedBoundaryComponent(end.theta, order),
                    1e-9);
        EXPECT_NEAR(at_end.y(), ExpectedBoundaryComponent(end.s, order), 1e-9);
    }
    // 每个内部航点同时是前一段末端与后一段起点的位置
    for (int i = 0; i + 1 < num_segments; ++i) {
        const Eigen::Vector2d from_left =
            trajectory.evaluateSegment(i, durations[i], 0);
        const Eigen::Vector2d from_right =
            trajectory.evaluateSegment(i + 1, 0.0, 0);
        EXPECT_NEAR(from_left.x(), waypoints[i].x(), 1e-9);
        EXPECT_NEAR(from_left.y(), waypoints[i].y(), 1e-9);
        EXPECT_NEAR(from_right.x(), waypoints[i].x(), 1e-9);
        EXPECT_NEAR(from_right.y(), waypoints[i].y(), 1e-9);
        // 1~4 阶导数在航点两侧连续
        for (int order = 1; order <= 4; ++order) {
            const Eigen::Vector2d left =
                trajectory.evaluateSegment(i, durations[i], order);
            const Eigen::Vector2d right =
                trajectory.evaluateSegment(i + 1, 0.0, order);
            EXPECT_NEAR(left.x(), right.x(), 1e-8);
            EXPECT_NEAR(left.y(), right.y(), 1e-8);
        }
    }
}

// 测试跃度（5 阶导数）在航点处允许不连续。
// 因为 MINCO 只强制到 2h-2=4 阶连续，所以跃度跳变是设计特性而非缺陷。
TEST(MincoTrajectoryTest, JerkMayBeDiscontinuousAtJunctions) {
    MincoBoundaryCondition2d start, end;
    std::vector<Eigen::Vector2d> waypoints;
    std::vector<double> durations;
    BuildMultiSegmentInput(&start, &end, &waypoints, &durations);
    MincoTrajectory trajectory;
    trajectory.setTrajectory(start, end, waypoints, durations);

    const Eigen::Vector2d left_jerk =
        trajectory.evaluateSegment(0, durations[0], 5);
    const Eigen::Vector2d right_jerk = trajectory.evaluateSegment(1, 0.0, 5);
    EXPECT_GT(std::abs(left_jerk.x() - right_jerk.x()), 1e-3);
}

// 测试 order=0 时基函数行退化为单项式基 [1, τ, τ², τ³, τ⁴, τ⁵]。
// 因为求值与 K(T) 装配的全部高阶导数行都由该基行求导得到，所以 0 阶退化
// 形式必须显式精确成立，且与段时长无关。
TEST(MincoTrajectoryTest, DerivativeBasisRowOrderZeroMatchesMonomialBasis) {
    for (const double tau : {0.0, 0.3, 0.7, 1.0}) {
        const Eigen::Matrix<double, 1, 6> row =
            MincoTrajectoryTestAccessor::DerivativeBasisRow(tau, 0, 2.5);
        for (int k = 0; k < 6; ++k) {
            EXPECT_NEAR(row[k], std::pow(tau, k), 1e-12);
        }
    }
}

// 测试终点弧长 s_f 的伴随梯度与中心差分一致。
// 因为 s_f 只通过 b 间接影响系数，所以 K(T)^{-T} 伴随给出的解析梯度必须
// 与"改动 s_f 重建轨迹"的数值梯度吻合。
TEST(MincoTrajectoryTest,
     FinalArcLengthAdjointGradientMatchesCentralDifference) {
    const MincoBoundaryCondition2d start{{0.1, 0.05, 0.02}, {0.0, 0.5, 0.1}};
    const MincoBoundaryCondition2d end{{1.3, -0.1, 0.0}, {2.2, 0.3, 0.05}};
    const std::vector<Eigen::Vector2d> waypoints{{0.4, 0.6}, {0.9, 1.1}};
    const std::vector<double> durations{1.2, 0.8, 1.5};
    MincoTrajectory trajectory;
    trajectory.setTrajectory(start, end, waypoints, durations);

    // 取线性标量目标 J = w·c_s，其 ∂J/∂c_s 恰为 w 本身
    CoeffMatrix weights(CoeffMatrix::Zero(6, 3));
    for (int seg = 0; seg < 3; ++seg) {
        for (int k = 0; k < 6; ++k) {
            weights(k, seg) =
                0.4 * (k + 1) - 0.25 * (seg + 1) + 0.1 * (k * seg);
        }
    }
    const auto objective = [&weights](const MincoTrajectory& traj) {
        return (weights.array() * traj.coeffsS().array()).sum();
    };
    const double analytic = trajectory.finalArcLengthAdjointGradient(weights);

    const double eps = 1e-4;
    MincoBoundaryCondition2d end_plus = end;
    end_plus.s.pos += eps;
    MincoTrajectory traj_plus;
    traj_plus.setTrajectory(start, end_plus, waypoints, durations);
    MincoBoundaryCondition2d end_minus = end;
    end_minus.s.pos -= eps;
    MincoTrajectory traj_minus;
    traj_minus.setTrajectory(start, end_minus, waypoints, durations);
    const double central =
        (objective(traj_plus) - objective(traj_minus)) / (2.0 * eps);

    EXPECT_LE(std::abs(analytic - central) / std::max(std::abs(central), 1e-12),
              1e-6);
}

// 测试 τ↔T 双射在两条分段上均互逆。
// 因为重参数化是 L-BFGS 无约束优化的前提，所以正反映射必须严格互逆。
TEST(MincoTrajectoryTest, TimeBijectionRoundTrip) {
    const std::vector<double> durations{0.05, 0.3, 0.7, 1.0, 1.5, 3.0, 20.0};
    for (const double duration : durations) {
        const double tau = MincoTrajectory::DurationToTau(duration);
        EXPECT_NEAR(MincoTrajectory::TauToDuration(tau), duration, 1e-12);
    }
    const std::vector<double> taus{-5.0, -1.0, -0.3, 0.0, 0.2, 1.0, 4.0};
    for (const double tau : taus) {
        const double duration = MincoTrajectory::TauToDuration(tau);
        EXPECT_GT(duration, 0.0);
        EXPECT_NEAR(MincoTrajectory::DurationToTau(duration), tau, 1e-12);
    }
}

// 测试时间双射的解析导数与中心差分一致，并验证 T=1 处一阶连续。
// 因为梯度链式反传依赖 dT/dτ，所以解析导数必须与数值差分吻合。
TEST(MincoTrajectoryTest, TimeBijectionDerivativesMatchFiniteDifference) {
    const double h = 1e-5;
    const std::vector<double> taus{-2.0, -0.001, 0.0, 0.5, 3.0};
    for (const double tau : taus) {
        const double central = (MincoTrajectory::TauToDuration(tau + h) -
                                MincoTrajectory::TauToDuration(tau - h)) /
                               (2.0 * h);
        EXPECT_NEAR(MincoTrajectory::TauToDurationDerivative(tau), central,
                    1e-6);
    }
    const std::vector<double> durations{0.3, 1.0, 3.0};
    for (const double duration : durations) {
        const double central = (MincoTrajectory::DurationToTau(duration + h) -
                                MincoTrajectory::DurationToTau(duration - h)) /
                               (2.0 * h);
        EXPECT_NEAR(MincoTrajectory::DurationToTauDerivative(duration), central,
                    1e-6);
    }
    // T=1（τ=0）处左右导数均应为 1，保证分段点一阶连续可导
    EXPECT_NEAR(MincoTrajectory::TauToDurationDerivative(0.0), 1.0, 1e-12);
    EXPECT_NEAR(MincoTrajectory::DurationToTauDerivative(1.0), 1.0, 1e-12);
    EXPECT_NEAR(MincoTrajectory::TauToDurationDerivative(-1e-9) -
                    MincoTrajectory::TauToDurationDerivative(1e-9),
                0.0, 1e-6);
}

// 测试极短/极长段时长混合的退化场景。
// 因为时间尺度悬殊会恶化 K(T) 条件数，所以该场景下只要求不产生
// NaN/Inf 且端点还原大致成立。
TEST(MincoTrajectoryTest, ExtremeDurationsStayFinite) {
    const MincoBoundaryCondition2d start{{0.0, 0.1, 0.0}, {0.0, 0.2, 0.0}};
    const MincoBoundaryCondition2d end{{1.0, 0.0, 0.0}, {5.0, 0.1, 0.0}};
    const std::vector<Eigen::Vector2d> waypoints{{0.3, 1.0}, {0.8, 4.0}};
    const std::vector<double> durations{0.05, 30.0, 0.08};
    MincoTrajectory trajectory;
    trajectory.setTrajectory(start, end, waypoints, durations);

    EXPECT_TRUE(trajectory.coeffsTheta().allFinite());
    EXPECT_TRUE(trajectory.coeffsS().allFinite());
    const Eigen::Vector2d at_start = trajectory.evaluate(0.0, 0);
    const Eigen::Vector2d at_end =
        trajectory.evaluate(trajectory.totalDuration(), 0);
    EXPECT_NEAR(at_start.x(), start.theta.pos, 1e-2);
    EXPECT_NEAR(at_start.y(), start.s.pos, 1e-2);
    EXPECT_NEAR(at_end.x(), end.theta.pos, 1e-2);
    EXPECT_NEAR(at_end.y(), end.s.pos, 1e-2);
}

// 测试非法输入与未初始化状态的拒绝行为。
// 因为边界外的调用属于调用方逻辑错误，所以必须抛出对应标准异常。
TEST(MincoTrajectoryTest, InvalidInputsThrow) {
    MincoTrajectory trajectory;
    EXPECT_THROW(trajectory.evaluate(0.0, 0), std::logic_error);

    const MincoBoundaryCondition2d start{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
    const MincoBoundaryCondition2d end{{1.0, 0.0, 0.0}, {1.0, 0.0, 0.0}};
    EXPECT_THROW(trajectory.setTrajectory(start, end, {}, {}),
                 std::invalid_argument);
    EXPECT_THROW(trajectory.setTrajectory(start, end, {}, {0.0}),
                 std::invalid_argument);
    EXPECT_THROW(trajectory.setTrajectory(start, end, {}, {-1.0}),
                 std::invalid_argument);
    EXPECT_THROW(
        trajectory.setTrajectory(start, end, {},
                                 {std::numeric_limits<double>::quiet_NaN()}),
        std::invalid_argument);
    EXPECT_THROW(
        trajectory.setTrajectory(start, end, {{0.5, 0.5}}, {1.0, 1.0, 1.0}),
        std::invalid_argument);
    EXPECT_THROW(MincoTrajectory::DurationToTau(0.0), std::invalid_argument);
    EXPECT_THROW(MincoTrajectory::DurationToTau(-1.0), std::invalid_argument);
    EXPECT_THROW(MincoTrajectory::DurationToTauDerivative(0.0),
                 std::invalid_argument);

    trajectory.setTrajectory(start, end, {}, {2.0});
    EXPECT_THROW(trajectory.evaluate(0.0, 6), std::invalid_argument);
    EXPECT_THROW(trajectory.evaluate(-1.0, 0), std::out_of_range);
    EXPECT_THROW(trajectory.evaluate(3.0, 0), std::out_of_range);
    EXPECT_THROW(trajectory.evaluateSegment(1, 0.0, 0), std::out_of_range);
    EXPECT_THROW(
        trajectory.finalArcLengthAdjointGradient(CoeffMatrix::Zero(6, 2)),
        std::invalid_argument);
    // 端点处数值级微小越界应被容忍（内部截断），不得抛异常
    EXPECT_NO_THROW(trajectory.evaluate(2.0 + 1e-12, 0));
}

}  // namespace
}  // namespace apa_post_processor
