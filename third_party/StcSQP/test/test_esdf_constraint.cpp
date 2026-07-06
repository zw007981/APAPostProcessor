#include <limits>
#include <random>

#include <gtest/gtest.h>

#include "../src/constraints/esdf_distance_constraint.h"
#include "../src/core/vehicle_geometry.h"
#include "../src/math/math_util.hpp"
#include "../src/util/constants.h"

using namespace stc_SQP;

namespace {
// 对状态 x 施加第 i 维的微小扰动，theta 维度做角度回绕以避免跨 π 边界。
Vector perturbState(const Vector& x, int dim, double eps)
{
    Vector xp = x;
    xp(dim) += eps;
    if (dim == 2) {
        xp(2) = math_util::NormalizeAngle(xp(2));
    }
    return xp;
}

// 通过中心差分计算 Cx 的数值参考，与解析 Jacobian 对比。
void verifyJacobianAgainstFiniteDifference(const EsdfDistanceConstraint& constraint,
    const Vector& x, const Vector& u, const Vector& p, double eps)
{
    Matrix Cx, Cu;
    constraint.jacobian(x, u, p, Cx, Cu);

    Vector g_minus, g_plus;
    Matrix Cx_fd(constraint.ng(), x.size());
    for (int i = 0; i < x.size(); ++i) {
        constraint.evaluate(perturbState(x, i, -eps), u, p, g_minus);
        constraint.evaluate(perturbState(x, i, eps), u, p, g_plus);
        Cx_fd.col(i) = (g_plus - g_minus) / (2.0 * eps);
    }

    EXPECT_TRUE(Cx.isApprox(Cx_fd, 1e-6))
        << "Analytical Cx differs from central difference, max error " << (Cx - Cx_fd).cwiseAbs().maxCoeff();
    EXPECT_NEAR(Cu.norm(), 0.0, 1e-12);
}

// 构造一个"车辆四角点到某个固定采样点距离场"的参数向量：对每个角点采样
// distance/gradient/query_pos，query_pos 取角点在状态 x 处的世界坐标，
// 使得 evaluate 在 x 处的结果恰好等于 margin - distance（一阶展开误差为零）。
Vector buildParametersAtLinearizationPoint(const Vector& x, double distance,
    const Eigen::Vector2d& gradient)
{
    Vector p = Vector::Zero(STAGE_PARAM_DIM);
    const double theta = x(2), c = std::cos(theta), s = std::sin(theta);
    const Eigen::Vector2d center(x(0), x(1));
    const auto corners_local = EsdfDistanceConstraint::cornerLocalPositions();
    for (int k = 0; k < EsdfDistanceConstraint::kNumCorners; ++k) {
        const Eigen::Vector2d& local = corners_local[k];
        const Eigen::Vector2d corner_world(
            center(0) + c * local(0) - s * local(1), center(1) + s * local(0) + c * local(1));
        EsdfDistanceConstraint::packCornerSample(k, distance, gradient, corner_world, p);
    }
    return p;
}
} // namespace

TEST(EsdfDistanceConstraint, RejectsNegativeSafetyMargin)
{
    // 测试目的：验证构造函数拒绝非法（负数/非有限）安全裕度
    // 流程：分别用负数、NaN 构造 EsdfDistanceConstraint
    // 预期效果：均抛出 std::invalid_argument
    EXPECT_THROW(EsdfDistanceConstraint(-0.1), std::invalid_argument);
    EXPECT_THROW(EsdfDistanceConstraint(std::numeric_limits<double>::quiet_NaN()),
        std::invalid_argument);
}

TEST(EsdfDistanceConstraint, DimensionsAreCorrect)
{
    // 测试目的：验证约束维度与文档一致
    // 流程：构造约束后检查 ng()、evaluate/jacobian 输出形状
    // 预期效果：ng = 4，Cx 为 4 x nx，Cu 为 4 x nu 且恒为零
    EsdfDistanceConstraint constraint(0.3);
    EXPECT_EQ(constraint.ng(), EsdfDistanceConstraint::kNumCorners);

    Vector x(5);
    x << 1.0, 2.0, 0.3, 1.5, 0.05;
    Vector u = Vector::Zero(2);
    Vector p = buildParametersAtLinearizationPoint(x, 5.0, Eigen::Vector2d(1.0, 0.0));

    Vector g;
    constraint.evaluate(x, u, p, g);
    EXPECT_EQ(g.size(), 4);

    Matrix Cx, Cu;
    constraint.jacobian(x, u, p, Cx, Cu);
    EXPECT_EQ(Cx.rows(), 4);
    EXPECT_EQ(Cx.cols(), 5);
    EXPECT_EQ(Cu.rows(), 4);
    EXPECT_EQ(Cu.cols(), 2);
    EXPECT_NEAR(Cu.norm(), 0.0, 1e-12);
}

TEST(EsdfDistanceConstraint, EvaluateMatchesExpectedAtLinearizationPoint)
{
    // 测试目的：验证在采样点本身（query_pos = 当前角点位置）处，一阶展开误差应为零，
    //          即 g = safety_margin - distance 精确成立
    // 流程：构造 4 角点采样均取相同 distance/gradient，query_pos 恰为角点当前世界坐标
    // 预期效果：evaluate 结果精确等于 margin - distance（数值误差 < 1e-12）
    const double margin = 0.4, distance = 3.5;
    EsdfDistanceConstraint constraint(margin);

    Vector x(5);
    x << -2.0, 1.0, 0.7, 0.5, -0.02;
    Vector u = Vector::Zero(2);
    Vector p = buildParametersAtLinearizationPoint(x, distance, Eigen::Vector2d(0.6, 0.8));

    Vector g;
    constraint.evaluate(x, u, p, g);
    for (int k = 0; k < g.size(); ++k) {
        EXPECT_NEAR(g(k), margin - distance, 1e-12);
    }
}

TEST(EsdfDistanceConstraint, JacobianMatchesFiniteDifferenceOnDeterministicCase)
{
    // 测试目的：验证解析 Jacobian（含角点旋转对 theta 的导数）与中心差分一致
    // 流程：固定状态与参数，调用 jacobian 后用 evaluate 做中心差分对比
    // 预期效果：误差 < 1e-6，theta 列非零（角点随航向旋转），v/delta 列为零
    Vector x(5);
    x << 1.0, 2.0, 0.3, 1.5, 0.05;
    Vector u = Vector::Zero(2);
    Vector p = buildParametersAtLinearizationPoint(x, 5.0, Eigen::Vector2d(0.8, -0.6));

    EsdfDistanceConstraint constraint(0.3);
    verifyJacobianAgainstFiniteDifference(constraint, x, u, p, 1e-7);

    Matrix Cx, Cu;
    constraint.jacobian(x, u, p, Cx, Cu);
    EXPECT_NEAR(Cx.col(3).norm(), 0.0, 1e-12);
    EXPECT_NEAR(Cx.col(4).norm(), 0.0, 1e-12);
    EXPECT_GT(Cx.col(2).norm(), 1e-6);
}

TEST(EsdfDistanceConstraint, JacobianMatchesFiniteDifferenceOnRandomCases)
{
    // 测试目的：在随机位姿与随机梯度采样下，验证解析 Jacobian 稳定正确
    // 流程：随机生成 20 组 (x, gradient)，对每一组做中心差分并比较
    // 预期效果：所有样本的 Jacobian 与数值差分误差 < 1e-6
    std::mt19937 rng(7);
    std::uniform_real_distribution<double> x_dist(-5.0, 5.0);
    std::uniform_real_distribution<double> theta_dist(-PI + 0.1, PI - 0.1);
    std::uniform_real_distribution<double> v_dist(0.0, 5.0);
    std::uniform_real_distribution<double> delta_dist(-0.4, 0.4);
    std::uniform_real_distribution<double> grad_dist(-1.0, 1.0);
    std::uniform_real_distribution<double> dist_dist(0.1, 10.0);

    EsdfDistanceConstraint constraint(0.25);
    for (int trial = 0; trial < 20; ++trial) {
        Vector x(5);
        x << x_dist(rng), x_dist(rng), theta_dist(rng), v_dist(rng), delta_dist(rng);
        Vector u = Vector::Zero(2);
        Eigen::Vector2d gradient(grad_dist(rng), grad_dist(rng));
        Vector p = buildParametersAtLinearizationPoint(x, dist_dist(rng), gradient);
        verifyJacobianAgainstFiniteDifference(constraint, x, u, p, 1e-7);
    }
}

TEST(EsdfDistanceConstraint, PackCornerSampleRejectsInvalidInputs)
{
    // 测试目的：验证 packCornerSample 对非法角点索引/p 维度/非有限输入的防御性检查
    // 流程：分别传入越界 corner_idx、错误维度 p、NaN 距离
    // 预期效果：均抛出 std::invalid_argument
    Vector p = Vector::Zero(STAGE_PARAM_DIM);
    EXPECT_THROW(EsdfDistanceConstraint::packCornerSample(
                     -1, 1.0, Eigen::Vector2d(1, 0), Eigen::Vector2d(0, 0), p),
        std::invalid_argument);
    EXPECT_THROW(EsdfDistanceConstraint::packCornerSample(
                     EsdfDistanceConstraint::kNumCorners, 1.0, Eigen::Vector2d(1, 0),
                     Eigen::Vector2d(0, 0), p),
        std::invalid_argument);

    Vector bad_p = Vector::Zero(STAGE_PARAM_DIM - 1);
    EXPECT_THROW(EsdfDistanceConstraint::packCornerSample(
                     0, 1.0, Eigen::Vector2d(1, 0), Eigen::Vector2d(0, 0), bad_p),
        std::invalid_argument);

    EXPECT_THROW(EsdfDistanceConstraint::packCornerSample(0,
                     std::numeric_limits<double>::quiet_NaN(), Eigen::Vector2d(1, 0),
                     Eigen::Vector2d(0, 0), p),
        std::invalid_argument);
}

TEST(EsdfDistanceConstraint, CloneProducesIndependentEquivalentCopy)
{
    // 测试目的：验证 clone() 返回功能等价的独立副本（多线程 clone 池要求）
    // 流程：clone 后分别对同一 (x,u,p) 求值，比较结果与安全裕度
    // 预期效果：两者 evaluate 结果一致，safetyMargin 一致
    EsdfDistanceConstraint original(0.5);
    auto cloned = original.clone();

    Vector x(5);
    x << 0.0, 0.0, 0.1, 1.0, 0.0;
    Vector u = Vector::Zero(2);
    Vector p = buildParametersAtLinearizationPoint(x, 2.0, Eigen::Vector2d(1.0, 0.0));

    Vector g_original, g_cloned;
    original.evaluate(x, u, p, g_original);
    cloned->evaluate(x, u, p, g_cloned);
    EXPECT_TRUE(g_original.isApprox(g_cloned, 1e-12));
    EXPECT_DOUBLE_EQ(
        original.safetyMargin(), static_cast<EsdfDistanceConstraint&>(*cloned).safetyMargin());
}

TEST(EsdfDistanceConstraint, EvaluateAndJacobianMatchesSeparateCalls)
{
    // 测试目的：验证 evaluateAndJacobian 与分别调用 evaluate + jacobian 结果一致
    // 流程：构造同一点 (x,u,p)，分别用合并接口与单独接口求值，比较 g/Cx/Cu
    // 预期效果：合并接口的输出与单独调用完全一致
    Vector x(5);
    x << 1.0, -2.0, 0.4, 0.8, -0.05;
    Vector u = Vector::Zero(2);
    Vector p = buildParametersAtLinearizationPoint(x, 4.0, Eigen::Vector2d(-0.6, 0.8));

    EsdfDistanceConstraint constraint(0.3);

    Vector g_combined;
    Matrix Cx_combined, Cu_combined;
    constraint.evaluateAndJacobian(x, u, p, g_combined, Cx_combined, Cu_combined);

    Vector g_separate;
    Matrix Cx_separate, Cu_separate;
    constraint.evaluate(x, u, p, g_separate);
    constraint.jacobian(x, u, p, Cx_separate, Cu_separate);

    EXPECT_TRUE(g_combined.isApprox(g_separate, 1e-12));
    EXPECT_TRUE(Cx_combined.isApprox(Cx_separate, 1e-12));
    EXPECT_TRUE(Cu_combined.isApprox(Cu_separate, 1e-12));
}

TEST(EsdfDistanceConstraint, SafetyMarginShiftsConstraintByConstant)
{
    // 测试目的：验证 safety_margin 只使约束值 g 整体平移，不改变其几何形状
    // 流程：固定 (x,u,p)，用两个不同 margin 构造约束并 evaluate，比较差值
    // 预期效果：g_margin2 - g_margin1 精确等于 margin2 - margin1
    Vector x(5);
    x << 0.5, -1.0, 0.2, 1.0, 0.0;
    Vector u = Vector::Zero(2);
    Vector p = buildParametersAtLinearizationPoint(x, 2.5, Eigen::Vector2d(0.0, 1.0));

    const double margin1 = 0.2, margin2 = 0.7;
    EsdfDistanceConstraint constraint1(margin1);
    EsdfDistanceConstraint constraint2(margin2);

    Vector g1, g2;
    constraint1.evaluate(x, u, p, g1);
    constraint2.evaluate(x, u, p, g2);

    EXPECT_TRUE((g2 - g1).isApproxToConstant(margin2 - margin1, 1e-12));
}

TEST(EsdfDistanceConstraint, RejectsStateVectorShorterThanThree)
{
    // 测试目的：验证 x 维度不足 3 时 evaluate/jacobian 抛出 std::invalid_argument
    // 流程：构造 x.size()<3 的输入，分别调用 evaluate 与 jacobian
    // 预期效果：均抛出 std::invalid_argument
    EsdfDistanceConstraint constraint(0.3);
    Vector x = Vector::Zero(2);
    Vector u = Vector::Zero(2);
    Vector p = Vector::Zero(STAGE_PARAM_DIM);
    Vector g;
    Matrix Cx, Cu;

    EXPECT_THROW(constraint.evaluate(x, u, p, g), std::invalid_argument);
    EXPECT_THROW(constraint.jacobian(x, u, p, Cx, Cu), std::invalid_argument);
}

TEST(EsdfDistanceConstraint, WorksWithExtendedStateVectorBeyondFiveDims)
{
    // 测试目的：验证 EsdfDistanceConstraint 对 nx>5 的扩展状态向量仍只使用前 3 维，
    //          后续维度对应的 Cx 列为 0——体现约束与具体动力学模型解耦的设计
    // 流程：构造 nx=7 的状态与参数，调用 jacobian，检查 Cx 形状与后四列
    // 预期效果：Cx 为 4x7，前 3 列非零（theta 列），后 4 列全为 0；Cu 仍为 4x2 且为 0
    const int nx = 7, nu = 2;
    Vector x(nx);
    x << 1.0, 2.0, 0.3, 1.5, 0.05, 10.0, -5.0;
    Vector u = Vector::Zero(nu);
    Vector p = buildParametersAtLinearizationPoint(x.head(5), 5.0, Eigen::Vector2d(0.8, -0.6));

    EsdfDistanceConstraint constraint(0.3);
    Matrix Cx, Cu;
    constraint.jacobian(x, u, p, Cx, Cu);

    EXPECT_EQ(Cx.rows(), EsdfDistanceConstraint::kNumCorners);
    EXPECT_EQ(Cx.cols(), nx);
    EXPECT_EQ(Cu.rows(), EsdfDistanceConstraint::kNumCorners);
    EXPECT_EQ(Cu.cols(), nu);
    EXPECT_NEAR(Cu.norm(), 0.0, 1e-12);
    EXPECT_NEAR(Cx.col(3).norm(), 0.0, 1e-12);
    EXPECT_NEAR(Cx.col(4).norm(), 0.0, 1e-12);
    EXPECT_NEAR(Cx.col(5).norm(), 0.0, 1e-12);
    EXPECT_NEAR(Cx.col(6).norm(), 0.0, 1e-12);
    EXPECT_GT(Cx.col(2).norm(), 1e-6);
}
