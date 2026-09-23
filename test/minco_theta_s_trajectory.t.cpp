#include <gtest/gtest.h>

#include <Eigen/Core>
#include <cmath>
#include <stdexcept>
#include <vector>

#include "core/MINCO_THETA_S/minco_theta_s_trajectory.h"

namespace apa_post_processor {
namespace {

using CoeffMatrix = MincoThetaSTrajectory::CoeffMatrix;

// 单段 5 阶多项式 Hermite 插值的手推解析系数（归一化 tau=t/T 基）。
// 由端点位置/速度/加速度六个约束直接解出，供系数级精度对拍。
Eigen::Matrix<double, 6, 1> BuildAnalyticHermiteCoeffs(
    const MincoThetaSBoundaryCondition& start, const MincoThetaSBoundaryCondition& end,
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
void BuildMultiSegmentInput(MincoThetaSBoundaryCondition2d* start,
                            MincoThetaSBoundaryCondition2d* end,
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
double ExpectedBoundaryComponent(const MincoThetaSBoundaryCondition& bc, int order) {
    if (order == 0) {
        return bc.pos;
    }
    if (order == 1) {
        return bc.vel;
    }
    return bc.acc;
}

// 测试用派生类：暴露受保护的静态基函数行，供白盒对拍
class MincoThetaSTrajectoryTestAccessor : public MincoThetaSTrajectory {
   public:
    using MincoThetaSTrajectory::DerivativeBasisRow;
};

// 测试单段轨迹的系数与手推 Hermite 解析解一致。
// 因为单段退化为标准两点 Hermite 插值、存在闭式解，所以系数必须逐位吻合。
TEST(MincoThetaSTrajectoryTest, SingleSegmentMatchesAnalyticHermiteSolution) {
    const MincoThetaSBoundaryCondition2d start{{0.3, 0.1, -0.05}, {0.0, 0.8, 0.1}};
    const MincoThetaSBoundaryCondition2d end{{1.2, -0.2, 0.3}, {2.4, 0.0, 0.0}};
    const std::vector<double> durations{1.7};
    MincoThetaSTrajectory trajectory;
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
TEST(MincoThetaSTrajectoryTest, SingleSegmentEvaluateMatchesBoundaryConditions) {
    const MincoThetaSBoundaryCondition2d start{{0.3, 0.1, -0.05}, {0.0, 0.8, 0.1}};
    const MincoThetaSBoundaryCondition2d end{{1.2, -0.2, 0.3}, {2.4, 0.0, 0.0}};
    const std::vector<double> durations{1.7};
    MincoThetaSTrajectory trajectory;
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
TEST(MincoThetaSTrajectoryTest, MultiSegmentSatisfiesAllConstraints) {
    MincoThetaSBoundaryCondition2d start, end;
    std::vector<Eigen::Vector2d> waypoints;
    std::vector<double> durations;
    BuildMultiSegmentInput(&start, &end, &waypoints, &durations);
    MincoThetaSTrajectory trajectory;
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
// 因为 MINCO_THETA_S 只强制到 2h-2=4 阶连续，所以跃度跳变是设计特性而非缺陷。
TEST(MincoThetaSTrajectoryTest, JerkMayBeDiscontinuousAtJunctions) {
    MincoThetaSBoundaryCondition2d start, end;
    std::vector<Eigen::Vector2d> waypoints;
    std::vector<double> durations;
    BuildMultiSegmentInput(&start, &end, &waypoints, &durations);
    MincoThetaSTrajectory trajectory;
    trajectory.setTrajectory(start, end, waypoints, durations);

    const Eigen::Vector2d left_jerk =
        trajectory.evaluateSegment(0, durations[0], 5);
    const Eigen::Vector2d right_jerk = trajectory.evaluateSegment(1, 0.0, 5);
    EXPECT_GT(std::abs(left_jerk.x() - right_jerk.x()), 1e-3);
}

// 测试 order=0 时基函数行退化为单项式基 [1, τ, τ², τ³, τ⁴, τ⁵]。
// 因为求值与 K(T) 装配的全部高阶导数行都由该基行求导得到，所以 0 阶退化
// 形式必须显式精确成立，且与段时长无关。
TEST(MincoThetaSTrajectoryTest, DerivativeBasisRowOrderZeroMatchesMonomialBasis) {
    for (const double tau : {0.0, 0.3, 0.7, 1.0}) {
        const Eigen::Matrix<double, 1, 6> row =
            MincoThetaSTrajectoryTestAccessor::DerivativeBasisRow(tau, 0, 2.5);
        for (int k = 0; k < 6; ++k) {
            EXPECT_NEAR(row[k], std::pow(tau, k), 1e-12);
        }
    }
}

// 测试批量 0~2 阶求值与逐阶 evaluateSegment 调用逐位一致。
// 因为批量接口是求解器热路径上三次单阶调用的合并替代，返回值必须逐位
// 等价才能保证代价/梯度装配不变；采样点覆盖段端点、内部点与辛普森/
// 物理合并采样的全部节点位置。
TEST(MincoThetaSTrajectoryTest, EvaluateSegmentOrders02MatchesPerOrderCalls) {
    MincoThetaSBoundaryCondition2d start, end;
    std::vector<Eigen::Vector2d> waypoints;
    std::vector<double> durations;
    BuildMultiSegmentInput(&start, &end, &waypoints, &durations);
    MincoThetaSTrajectory trajectory;
    trajectory.setTrajectory(start, end, waypoints, durations);
    for (int i = 0; i < trajectory.numSegments(); ++i) {
        for (const double tau : {0.0, 0.125, 0.25, 0.5, 0.625, 0.875, 1.0}) {
            const double local_time = tau * durations[i];
            const MincoThetaSSegmentSample sample =
                trajectory.evaluateSegmentOrders02(i, local_time);
            const Eigen::Vector2d d0 =
                trajectory.evaluateSegment(i, local_time, 0);
            const Eigen::Vector2d d1 =
                trajectory.evaluateSegment(i, local_time, 1);
            const Eigen::Vector2d d2 =
                trajectory.evaluateSegment(i, local_time, 2);
            EXPECT_DOUBLE_EQ(sample.d0.x(), d0.x());
            EXPECT_DOUBLE_EQ(sample.d0.y(), d0.y());
            EXPECT_DOUBLE_EQ(sample.d1.x(), d1.x());
            EXPECT_DOUBLE_EQ(sample.d1.y(), d1.y());
            EXPECT_DOUBLE_EQ(sample.d2.x(), d2.x());
            EXPECT_DOUBLE_EQ(sample.d2.y(), d2.y());
        }
    }
    // 端点数值级微小越界：与单阶查询同样的截断容忍语义
    EXPECT_NO_THROW(trajectory.evaluateSegmentOrders02(0, -1e-12));
    EXPECT_NO_THROW(trajectory.evaluateSegmentOrders02(
        0, durations[0] + 1e-12));
}

// 测试 0/1/2 阶基函数行批量构造与逐阶 DerivativeBasisRow 逐位一致。
// 因为梯度装配将改为批量行构造以共享幂次链，三行必须与单行版本逐位
// 等价，否则代价梯度发生不可控漂移。
TEST(MincoThetaSTrajectoryTest, DerivativeBasisRows012MatchPerOrderRows) {
    for (const double tau : {0.0, 0.13, 0.25, 0.5, 0.9, 1.0}) {
        for (const double duration : {0.6, 1.2, 3.0}) {
            Eigen::Matrix<double, 1, 6> row0;
            Eigen::Matrix<double, 1, 6> row1;
            Eigen::Matrix<double, 1, 6> row2;
            MincoThetaSTrajectory::DerivativeBasisRows012(tau, duration, &row0, &row1,
                                                    &row2);
            const auto expect0 =
                MincoThetaSTrajectory::DerivativeBasisRow(tau, 0, duration);
            const auto expect1 =
                MincoThetaSTrajectory::DerivativeBasisRow(tau, 1, duration);
            const auto expect2 =
                MincoThetaSTrajectory::DerivativeBasisRow(tau, 2, duration);
            for (int k = 0; k < 6; ++k) {
                EXPECT_DOUBLE_EQ(row0[k], expect0[k]);
                EXPECT_DOUBLE_EQ(row1[k], expect1[k]);
                EXPECT_DOUBLE_EQ(row2[k], expect2[k]);
            }
        }
    }
}

// 测试批量求值接口的非法输入拒绝行为。
// 与单阶查询同一约定：未初始化抛 logic_error，段索引越界抛 out_of_range。
TEST(MincoThetaSTrajectoryTest, EvaluateSegmentOrders02RejectsInvalidInput) {
    MincoThetaSTrajectory trajectory;
    EXPECT_THROW(trajectory.evaluateSegmentOrders02(0, 0.0),
                 std::logic_error);
    const MincoThetaSBoundaryCondition2d start{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
    const MincoThetaSBoundaryCondition2d end{{1.0, 0.0, 0.0}, {1.0, 0.0, 0.0}};
    trajectory.setTrajectory(start, end, {}, {2.0});
    EXPECT_THROW(trajectory.evaluateSegmentOrders02(1, 0.0),
                 std::out_of_range);
    EXPECT_THROW(trajectory.evaluateSegmentOrders02(-1, 0.0),
                 std::out_of_range);
}

// 测试段两端 3/4 阶导数批量接口与逐阶调用逐位一致。
// 因为 ∂J/∂T 装配原本把同一段两端的 3/4 阶导数逐阶求值 4 次，批量接口
// 合并后必须给出完全相同的位模式，否则时间变量梯度发生不可控漂移；用例
// 覆盖全部段（各段时长不同，覆盖 T<1 与 T>1 两侧的幂次路径）。
TEST(MincoThetaSTrajectoryTest, SegmentEndOrders34MatchesPerOrderCalls) {
    MincoThetaSBoundaryCondition2d start, end;
    std::vector<Eigen::Vector2d> waypoints;
    std::vector<double> durations;
    BuildMultiSegmentInput(&start, &end, &waypoints, &durations);
    MincoThetaSTrajectory trajectory;
    trajectory.setTrajectory(start, end, waypoints, durations);
    for (int i = 0; i < trajectory.numSegments(); ++i) {
        const double duration = durations[i];
        const MincoThetaSSegmentEndOrders34 ders =
            trajectory.evaluateSegmentEndOrders34(i);
        const Eigen::Vector2d start3 = trajectory.evaluateSegment(i, 0.0, 3);
        const Eigen::Vector2d start4 = trajectory.evaluateSegment(i, 0.0, 4);
        const Eigen::Vector2d end3 =
            trajectory.evaluateSegment(i, duration, 3);
        const Eigen::Vector2d end4 =
            trajectory.evaluateSegment(i, duration, 4);
        EXPECT_EQ(ders.start_order3.x(), start3.x());
        EXPECT_EQ(ders.start_order3.y(), start3.y());
        EXPECT_EQ(ders.start_order4.x(), start4.x());
        EXPECT_EQ(ders.start_order4.y(), start4.y());
        EXPECT_EQ(ders.end_order3.x(), end3.x());
        EXPECT_EQ(ders.end_order3.y(), end3.y());
        EXPECT_EQ(ders.end_order4.x(), end4.x());
        EXPECT_EQ(ders.end_order4.y(), end4.y());
    }
}

// 测试段两端批量接口的非法输入拒绝行为。
// 与其余求值入口同一约定：未初始化抛 logic_error，段索引越界抛
// out_of_range。
TEST(MincoThetaSTrajectoryTest, SegmentEndOrders34RejectsInvalidInput) {
    MincoThetaSTrajectory trajectory;
    EXPECT_THROW(trajectory.evaluateSegmentEndOrders34(0), std::logic_error);
    const MincoThetaSBoundaryCondition2d start{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
    const MincoThetaSBoundaryCondition2d end{{1.0, 0.0, 0.0}, {1.0, 0.0, 0.0}};
    trajectory.setTrajectory(start, end, {}, {2.0});
    EXPECT_THROW(trajectory.evaluateSegmentEndOrders34(1),
                 std::out_of_range);
    EXPECT_THROW(trajectory.evaluateSegmentEndOrders34(-1),
                 std::out_of_range);
}

// 测试辛普森首末节点样本在段两端的 1/2 阶导数与逐阶调用逐位一致。
// 因为 ∂J/∂T 装配改为直接复用节点样本的端点 1/2 阶导数，该复用成立的
// 前提正是两条求值路径在 tau=0 与 tau=1 处给出完全相同的位模式。
TEST(MincoThetaSTrajectoryTest, EndpointNodeSamplesMatchPerOrderCalls) {
    MincoThetaSBoundaryCondition2d start, end;
    std::vector<Eigen::Vector2d> waypoints;
    std::vector<double> durations;
    BuildMultiSegmentInput(&start, &end, &waypoints, &durations);
    MincoThetaSTrajectory trajectory;
    trajectory.setTrajectory(start, end, waypoints, durations);
    for (int i = 0; i < trajectory.numSegments(); ++i) {
        const double duration = durations[i];
        const MincoThetaSSegmentSample at_start =
            trajectory.evaluateSegmentOrders02(i, 0.0);
        const MincoThetaSSegmentSample at_end =
            trajectory.evaluateSegmentOrders02(i, duration);
        const Eigen::Vector2d start1 = trajectory.evaluateSegment(i, 0.0, 1);
        const Eigen::Vector2d start2 = trajectory.evaluateSegment(i, 0.0, 2);
        const Eigen::Vector2d end1 =
            trajectory.evaluateSegment(i, duration, 1);
        const Eigen::Vector2d end2 =
            trajectory.evaluateSegment(i, duration, 2);
        EXPECT_EQ(at_start.d1.x(), start1.x());
        EXPECT_EQ(at_start.d1.y(), start1.y());
        EXPECT_EQ(at_start.d2.x(), start2.x());
        EXPECT_EQ(at_start.d2.y(), start2.y());
        EXPECT_EQ(at_end.d1.x(), end1.x());
        EXPECT_EQ(at_end.d1.y(), end1.y());
        EXPECT_EQ(at_end.d2.x(), end2.x());
        EXPECT_EQ(at_end.d2.y(), end2.y());
    }
}

// 测试终点弧长 s_f 的伴随梯度与中心差分一致。
// 因为 s_f 只通过 b 间接影响系数，所以 K(T)^{-T} 伴随给出的解析梯度必须
// 与"改动 s_f 重建轨迹"的数值梯度吻合。
TEST(MincoThetaSTrajectoryTest,
     FinalArcLengthAdjointGradientMatchesCentralDifference) {
    const MincoThetaSBoundaryCondition2d start{{0.1, 0.05, 0.02}, {0.0, 0.5, 0.1}};
    const MincoThetaSBoundaryCondition2d end{{1.3, -0.1, 0.0}, {2.2, 0.3, 0.05}};
    const std::vector<Eigen::Vector2d> waypoints{{0.4, 0.6}, {0.9, 1.1}};
    const std::vector<double> durations{1.2, 0.8, 1.5};
    MincoThetaSTrajectory trajectory;
    trajectory.setTrajectory(start, end, waypoints, durations);

    // 取线性标量目标 J = w·c_s，其 ∂J/∂c_s 恰为 w 本身
    CoeffMatrix weights(CoeffMatrix::Zero(6, 3));
    for (int seg = 0; seg < 3; ++seg) {
        for (int k = 0; k < 6; ++k) {
            weights(k, seg) =
                0.4 * (k + 1) - 0.25 * (seg + 1) + 0.1 * (k * seg);
        }
    }
    const auto objective = [&weights](const MincoThetaSTrajectory& traj) {
        return (weights.array() * traj.coeffsS().array()).sum();
    };
    const double analytic = trajectory.finalArcLengthAdjointGradient(weights);

    const double eps = 1e-4;
    MincoThetaSBoundaryCondition2d end_plus = end;
    end_plus.s.pos += eps;
    MincoThetaSTrajectory traj_plus;
    traj_plus.setTrajectory(start, end_plus, waypoints, durations);
    MincoThetaSBoundaryCondition2d end_minus = end;
    end_minus.s.pos -= eps;
    MincoThetaSTrajectory traj_minus;
    traj_minus.setTrajectory(start, end_minus, waypoints, durations);
    const double central =
        (objective(traj_plus) - objective(traj_minus)) / (2.0 * eps);

    EXPECT_LE(std::abs(analytic - central) / std::max(std::abs(central), 1e-12),
              1e-6);
}

// 测试 τ↔T 双射在两条分段上均互逆。
// 因为重参数化是 L-BFGS 无约束优化的前提，所以正反映射必须严格互逆。
TEST(MincoThetaSTrajectoryTest, TimeBijectionRoundTrip) {
    const std::vector<double> durations{0.05, 0.3, 0.7, 1.0, 1.5, 3.0, 20.0};
    for (const double duration : durations) {
        const double tau = MincoThetaSTrajectory::DurationToTau(duration);
        EXPECT_NEAR(MincoThetaSTrajectory::TauToDuration(tau), duration, 1e-12);
    }
    const std::vector<double> taus{-5.0, -1.0, -0.3, 0.0, 0.2, 1.0, 4.0};
    for (const double tau : taus) {
        const double duration = MincoThetaSTrajectory::TauToDuration(tau);
        EXPECT_GT(duration, 0.0);
        EXPECT_NEAR(MincoThetaSTrajectory::DurationToTau(duration), tau, 1e-12);
    }
}

// 测试时间双射的解析导数与中心差分一致，并验证 T=1 处一阶连续。
// 因为梯度链式反传依赖 dT/dτ，所以解析导数必须与数值差分吻合。
TEST(MincoThetaSTrajectoryTest, TimeBijectionDerivativesMatchFiniteDifference) {
    const double h = 1e-5;
    const std::vector<double> taus{-2.0, -0.001, 0.0, 0.5, 3.0};
    for (const double tau : taus) {
        const double central = (MincoThetaSTrajectory::TauToDuration(tau + h) -
                                MincoThetaSTrajectory::TauToDuration(tau - h)) /
                               (2.0 * h);
        EXPECT_NEAR(MincoThetaSTrajectory::TauToDurationDerivative(tau), central,
                    1e-6);
    }
    const std::vector<double> durations{0.3, 1.0, 3.0};
    for (const double duration : durations) {
        const double central = (MincoThetaSTrajectory::DurationToTau(duration + h) -
                                MincoThetaSTrajectory::DurationToTau(duration - h)) /
                               (2.0 * h);
        EXPECT_NEAR(MincoThetaSTrajectory::DurationToTauDerivative(duration), central,
                    1e-6);
    }
    // T=1（τ=0）处左右导数均应为 1，保证分段点一阶连续可导
    EXPECT_NEAR(MincoThetaSTrajectory::TauToDurationDerivative(0.0), 1.0, 1e-12);
    EXPECT_NEAR(MincoThetaSTrajectory::DurationToTauDerivative(1.0), 1.0, 1e-12);
    EXPECT_NEAR(MincoThetaSTrajectory::TauToDurationDerivative(-1e-9) -
                    MincoThetaSTrajectory::TauToDurationDerivative(1e-9),
                0.0, 1e-6);
}

// 测试极短/极长段时长混合的退化场景。
// 因为时间尺度悬殊会恶化 K(T) 条件数，所以该场景下只要求不产生
// NaN/Inf 且端点还原大致成立。
TEST(MincoThetaSTrajectoryTest, ExtremeDurationsStayFinite) {
    const MincoThetaSBoundaryCondition2d start{{0.0, 0.1, 0.0}, {0.0, 0.2, 0.0}};
    const MincoThetaSBoundaryCondition2d end{{1.0, 0.0, 0.0}, {5.0, 0.1, 0.0}};
    const std::vector<Eigen::Vector2d> waypoints{{0.3, 1.0}, {0.8, 4.0}};
    const std::vector<double> durations{0.05, 30.0, 0.08};
    MincoThetaSTrajectory trajectory;
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
TEST(MincoThetaSTrajectoryTest, InvalidInputsThrow) {
    MincoThetaSTrajectory trajectory;
    EXPECT_THROW(trajectory.evaluate(0.0, 0), std::logic_error);

    const MincoThetaSBoundaryCondition2d start{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
    const MincoThetaSBoundaryCondition2d end{{1.0, 0.0, 0.0}, {1.0, 0.0, 0.0}};
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
    EXPECT_THROW(MincoThetaSTrajectory::DurationToTau(0.0), std::invalid_argument);
    EXPECT_THROW(MincoThetaSTrajectory::DurationToTau(-1.0), std::invalid_argument);
    EXPECT_THROW(MincoThetaSTrajectory::DurationToTauDerivative(0.0),
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
