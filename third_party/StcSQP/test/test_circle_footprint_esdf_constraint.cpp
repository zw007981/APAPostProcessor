#include <limits>
#include <random>

#include <gtest/gtest.h>

#include "../src/constraints/circle_footprint_esdf_constraint.h"
#include "../src/math/math_util.hpp"
#include "../src/util/constants.h"

using namespace stc_SQP;

namespace {
// 一组测试用的车身局部圆心坐标：3个圆，非对称分布，便于区分x/y/theta方向的敏感度
std::vector<Eigen::Vector2d> testLocalCircles()
{
    return { Eigen::Vector2d(1.5, 0.8), Eigen::Vector2d(1.5, -0.8), Eigen::Vector2d(-1.0, 0.0) };
}

// 对状态x施加第i维的微小扰动，theta维度做角度回绕以避免跨π边界
Vector perturbState(const Vector& x, int dim, double eps)
{
    Vector xp = x;
    xp(dim) += eps;
    if (dim == 2) {
        xp(2) = math_util::NormalizeAngle(xp(2));
    }
    return xp;
}

// 通过中心差分计算Cx的数值参考，与解析Jacobian对比
void verifyJacobianAgainstFiniteDifference(const CircleFootprintEsdfConstraint& constraint,
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
        << "Analytical Cx differs from central difference, max error "
        << (Cx - Cx_fd).cwiseAbs().maxCoeff();
    EXPECT_NEAR(Cu.norm(), 0.0, 1e-12);
}

// 构造一个"各圆到某个固定采样点距离场"的参数向量：每个圆采样distance/gradient/query_pos，
// query_pos取该圆在状态x处的世界坐标，使得evaluate在x处的结果恰好等于
// (circle_radius + safety_margin) - distance（一阶展开误差为零）
Vector buildParametersAtLinearizationPoint(const std::vector<Eigen::Vector2d>& circles_local,
    const Vector& x, double distance, const Eigen::Vector2d& gradient)
{
    Vector p = Vector::Zero(STAGE_PARAM_DIM);
    const double theta = x(2), c = std::cos(theta), s = std::sin(theta);
    const Eigen::Vector2d center(x(0), x(1));
    for (int k = 0; k < static_cast<int>(circles_local.size()); ++k) {
        const Eigen::Vector2d& local = circles_local[k];
        const Eigen::Vector2d circle_world(
            center(0) + c * local(0) - s * local(1), center(1) + s * local(0) + c * local(1));
        CircleFootprintEsdfConstraint::packCircleSample(k, distance, gradient, circle_world, p);
    }
    return p;
}
} // namespace

TEST(CircleFootprintEsdfConstraint, RejectsInvalidConstructorArguments)
{
    // 测试目的：验证构造函数拒绝空圆列表、超出kMaxCircles的圆列表、非法半径与安全裕度
    // 流程：分别用空vector、超量vector、非正半径、负裕度构造
    // 预期效果：均抛出std::invalid_argument
    EXPECT_THROW(CircleFootprintEsdfConstraint({}, 0.5, 0.1), std::invalid_argument);

    std::vector<Eigen::Vector2d> too_many(CircleFootprintEsdfConstraint::kMaxCircles + 1,
        Eigen::Vector2d(0.0, 0.0));
    EXPECT_THROW(CircleFootprintEsdfConstraint(too_many, 0.5, 0.1), std::invalid_argument);

    EXPECT_THROW(CircleFootprintEsdfConstraint(testLocalCircles(), 0.0, 0.1),
        std::invalid_argument);
    EXPECT_THROW(CircleFootprintEsdfConstraint(testLocalCircles(), -0.5, 0.1),
        std::invalid_argument);
    EXPECT_THROW(CircleFootprintEsdfConstraint(testLocalCircles(), 0.5, -0.1),
        std::invalid_argument);
}

TEST(CircleFootprintEsdfConstraint, DimensionsMatchConstructedCircleCount)
{
    // 测试目的：验证约束维度随构造时传入的圆数量变化，而非固定为矩形版本的4
    // 流程：用3个圆构造约束后检查ng()、evaluate/jacobian输出形状
    // 预期效果：ng = 3，Cx为3 x nx，Cu为3 x nu且恒为零
    const auto circles = testLocalCircles();
    CircleFootprintEsdfConstraint constraint(circles, 0.8, 0.3);
    EXPECT_EQ(constraint.ng(), static_cast<int>(circles.size()));

    Vector x(5);
    x << 1.0, 2.0, 0.3, 1.5, 0.05;
    Vector u = Vector::Zero(2);
    Vector p = buildParametersAtLinearizationPoint(circles, x, 5.0, Eigen::Vector2d(1.0, 0.0));

    Vector g;
    constraint.evaluate(x, u, p, g);
    EXPECT_EQ(g.size(), 3);

    Matrix Cx, Cu;
    constraint.jacobian(x, u, p, Cx, Cu);
    EXPECT_EQ(Cx.rows(), 3);
    EXPECT_EQ(Cx.cols(), 5);
    EXPECT_EQ(Cu.rows(), 3);
    EXPECT_EQ(Cu.cols(), 2);
    EXPECT_NEAR(Cu.norm(), 0.0, 1e-12);
}

TEST(CircleFootprintEsdfConstraint, EvaluateMatchesExpectedAtLinearizationPoint)
{
    // 测试目的：验证在采样点本身（query_pos = 当前圆心位置）处，一阶展开误差应为零，
    //          即g = (circle_radius + safety_margin) - distance精确成立
    // 流程：构造各圆采样均取相同distance/gradient，query_pos恰为圆心当前世界坐标
    // 预期效果：evaluate结果精确等于(radius + margin) - distance（数值误差 < 1e-12）
    const double radius = 0.6, margin = 0.4, distance = 3.5;
    const auto circles = testLocalCircles();
    CircleFootprintEsdfConstraint constraint(circles, radius, margin);

    Vector x(5);
    x << -2.0, 1.0, 0.7, 0.5, -0.02;
    Vector u = Vector::Zero(2);
    Vector p = buildParametersAtLinearizationPoint(circles, x, distance, Eigen::Vector2d(0.6, 0.8));

    Vector g;
    constraint.evaluate(x, u, p, g);
    for (int k = 0; k < g.size(); ++k) {
        EXPECT_NEAR(g(k), (radius + margin) - distance, 1e-12);
    }
}

TEST(CircleFootprintEsdfConstraint, JacobianMatchesFiniteDifferenceOnRandomCases)
{
    // 测试目的：在随机位姿与随机梯度采样下，验证解析Jacobian（含圆心旋转对theta的导数）
    //          与中心差分一致，覆盖矩形版本之外的"任意圆数量"场景
    // 流程：随机生成20组(x, gradient)，对每一组做中心差分并比较
    // 预期效果：所有样本的Jacobian与数值差分误差 < 1e-6
    std::mt19937 rng(11);
    std::uniform_real_distribution<double> x_dist(-5.0, 5.0);
    std::uniform_real_distribution<double> theta_dist(-PI + 0.1, PI - 0.1);
    std::uniform_real_distribution<double> v_dist(0.0, 5.0);
    std::uniform_real_distribution<double> delta_dist(-0.4, 0.4);
    std::uniform_real_distribution<double> grad_dist(-1.0, 1.0);
    std::uniform_real_distribution<double> dist_dist(0.1, 10.0);

    const auto circles = testLocalCircles();
    CircleFootprintEsdfConstraint constraint(circles, 0.5, 0.25);
    for (int trial = 0; trial < 20; ++trial) {
        Vector x(5);
        x << x_dist(rng), x_dist(rng), theta_dist(rng), v_dist(rng), delta_dist(rng);
        Vector u = Vector::Zero(2);
        Eigen::Vector2d gradient(grad_dist(rng), grad_dist(rng));
        Vector p = buildParametersAtLinearizationPoint(circles, x, dist_dist(rng), gradient);
        verifyJacobianAgainstFiniteDifference(constraint, x, u, p, 1e-7);
    }
}

TEST(CircleFootprintEsdfConstraint, PackCircleSampleRejectsInvalidInputs)
{
    // 测试目的：验证packCircleSample对非法圆索引/p维度/非有限输入的防御性检查
    // 流程：分别传入越界circle_idx、错误维度p、NaN距离
    // 预期效果：均抛出std::invalid_argument
    Vector p = Vector::Zero(STAGE_PARAM_DIM);
    EXPECT_THROW(CircleFootprintEsdfConstraint::packCircleSample(
                     -1, 1.0, Eigen::Vector2d(1, 0), Eigen::Vector2d(0, 0), p),
        std::invalid_argument);
    EXPECT_THROW(CircleFootprintEsdfConstraint::packCircleSample(
                     CircleFootprintEsdfConstraint::kMaxCircles, 1.0, Eigen::Vector2d(1, 0),
                     Eigen::Vector2d(0, 0), p),
        std::invalid_argument);

    Vector bad_p = Vector::Zero(STAGE_PARAM_DIM - 1);
    EXPECT_THROW(CircleFootprintEsdfConstraint::packCircleSample(
                     0, 1.0, Eigen::Vector2d(1, 0), Eigen::Vector2d(0, 0), bad_p),
        std::invalid_argument);

    EXPECT_THROW(CircleFootprintEsdfConstraint::packCircleSample(0,
                     std::numeric_limits<double>::quiet_NaN(), Eigen::Vector2d(1, 0),
                     Eigen::Vector2d(0, 0), p),
        std::invalid_argument);
}

TEST(CircleFootprintEsdfConstraint, CloneProducesIndependentEquivalentCopy)
{
    // 测试目的：验证clone()返回功能等价的独立副本（多线程clone池要求）
    // 流程：clone后分别对同一(x,u,p)求值，比较结果与半径/安全裕度
    // 预期效果：两者evaluate结果一致，circleRadius/safetyMargin一致
    const auto circles = testLocalCircles();
    CircleFootprintEsdfConstraint original(circles, 0.6, 0.5);
    auto cloned = original.clone();

    Vector x(5);
    x << 0.0, 0.0, 0.1, 1.0, 0.0;
    Vector u = Vector::Zero(2);
    Vector p = buildParametersAtLinearizationPoint(circles, x, 2.0, Eigen::Vector2d(0.5, -0.5));

    Vector g_original, g_cloned;
    original.evaluate(x, u, p, g_original);
    cloned->evaluate(x, u, p, g_cloned);
    EXPECT_TRUE(g_original.isApprox(g_cloned));

    auto* cloned_typed = dynamic_cast<CircleFootprintEsdfConstraint*>(cloned.get());
    ASSERT_NE(cloned_typed, nullptr);
    EXPECT_DOUBLE_EQ(cloned_typed->circleRadius(), original.circleRadius());
    EXPECT_DOUBLE_EQ(cloned_typed->safetyMargin(), original.safetyMargin());
}
