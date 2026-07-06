#include <atomic>
#include <random>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "../src/constraints/convex_corridor_constraint.h"
#include "../src/util/constants.h"
#include "../src/core/vehicle_geometry.h"
#include "../src/generated/corridor.h"
#include "../src/math/math_util.hpp"

using namespace stc_SQP;

TEST(ConvexCorridorConstraint, ParameterDimMatchesGeneratedConstant)
{
    // 静态断言：中性层 STAGE_PARAM_DIM 必须与 CasADi 生成常量一致，
    // 防止生成脚本维度变化时 OCP / ProblemUpdater 校验滞后。
    static_assert(STAGE_PARAM_DIM == CORRIDOR_P_DIM,
        "STAGE_PARAM_DIM and CORRIDOR_P_DIM mismatch");
    EXPECT_EQ(STAGE_PARAM_DIM, CORRIDOR_P_DIM);
}

namespace {
// 控制维度（仅用于 jacobian 的 Cu 占位）
constexpr int NU = 2;
// 凸走廊参数在 p 中的起始偏移与长度
constexpr int HS_START = 15;
constexpr int N_HS = 10;

// 为给定状态随机生成一组“车辆位于可行域内部”的半空间参数。
// p[15:35] 存储 10 个单位法向量 A_i（展平）和截距 b_i，其余补零。
Vector build_random_parameters(const Vector& x, std::mt19937& rng)
{
    std::uniform_real_distribution<double> angle_dist(0.0, 2.0 * PI);
    std::uniform_real_distribution<double> margin_dist(0.5, 2.0);

    Vector p = Vector::Zero(CORRIDOR_P_DIM);
    const double x0 = x(0);
    const double y0 = x(1);

    for (int i = 0; i < N_HS; ++i) {
        const double phi = angle_dist(rng);
        const double ax = std::cos(phi);
        const double ay = std::sin(phi);
        // 保证 (x0, y0) 严格在可行域内部：b_i = A_i^T c + margin
        const double b = ax * x0 + ay * y0 + margin_dist(rng);
        p(HS_START + 2 * i + 0) = ax;
        p(HS_START + 2 * i + 1) = ay;
        p(HS_START + 2 * N_HS + i) = b;
    }
    return p;
}

// 对状态 x 施加第 i 维的微小扰动，theta 维度做角度回绕以避免跨 π 边界。
Vector perturb_state(const Vector& x, int dim, double eps)
{
    Vector xp = x;
    xp(dim) += eps;
    if (dim == 2) {
        xp(2) = math_util::NormalizeAngle(xp(2));
    }
    return xp;
}

// 通过中心差分计算 Cx 的数值参考，与 CasADi 符号 Jacobian 对比。
void verify_jacobian_against_finite_difference(const ConvexCorridorConstraint& constraint,
    const Vector& x, const Vector& u, const Vector& p, double eps)
{
    Matrix Cx, Cu;
    constraint.jacobian(x, u, p, Cx, Cu);

    Vector g_minus, g_plus;
    Matrix Cx_fd(constraint.ng(), CORRIDOR_NX);
    for (int i = 0; i < CORRIDOR_NX; ++i) {
        constraint.evaluate(perturb_state(x, i, -eps), u, p, g_minus);
        constraint.evaluate(perturb_state(x, i, eps), u, p, g_plus);
        Cx_fd.col(i) = (g_plus - g_minus) / (2.0 * eps);
    }

    EXPECT_TRUE(Cx.isApprox(Cx_fd, 1e-6))
        << "CasADi Cx differs from central difference, max error " << (Cx - Cx_fd).cwiseAbs().maxCoeff();
}

} // namespace

TEST(ConvexCorridorConstraint, JacobianMatchesFiniteDifferenceOnDeterministicCase)
{
    // 测试目的：在固定（非边界）位姿与参数下，验证生成的 Cx 与中心差分一致
    // 流程：构造 p 使车辆位于半空间交集内部，调用 jacobian 后再用 evaluate 做中心差分
    // 预期效果：Cx 与数值差分误差 < 1e-6，且 v/kappa 对应列为零，theta 列非零
    Vector x(CORRIDOR_NX);
    x << 1.0, 2.0, 0.3, 1.5, 0.05;
    Vector u = Vector::Zero(NU);

    Vector p = Vector::Zero(CORRIDOR_P_DIM);
    // 构造一个简单的正方形可行域：x 在 [0,5], y 在 [0,5]
    const double bounds[10][3] = {
        { 1.0, 0.0, 5.0 }, // x <= 5
        { -1.0, 0.0, 0.0 }, // -x <= 0
        { 0.0, 1.0, 5.0 }, // y <= 5
        { 0.0, -1.0, 0.0 }, // -y <= 0
        { 1.0, 1.0, 8.0 },
        { -1.0, 1.0, 3.0 },
        { 1.0, -1.0, 3.0 },
        { -1.0, -1.0, 2.0 },
        { 0.8, 0.6, 5.0 },
        { -0.6, 0.8, 2.0 },
    };
    for (int i = 0; i < N_HS; ++i) {
        p(HS_START + 2 * i + 0) = bounds[i][0];
        p(HS_START + 2 * i + 1) = bounds[i][1];
        p(HS_START + 2 * N_HS + i) = bounds[i][2];
    }

    ConvexCorridorConstraint constraint(p, NU);
    verify_jacobian_against_finite_difference(constraint, x, u, p, 1e-7);

    Matrix Cx, Cu;
    constraint.jacobian(x, u, p, Cx, Cu);
    // 凸走廊约束不依赖 v 与 kappa，对应列应为零
    EXPECT_NEAR(Cx.col(3).norm(), 0.0, 1e-12);
    EXPECT_NEAR(Cx.col(4).norm(), 0.0, 1e-12);
    // theta 列必须非零（车辆角点随航向旋转）
    EXPECT_GT(Cx.col(2).norm(), 1e-6);
    // Cu 恒为零
    EXPECT_NEAR(Cu.norm(), 0.0, 1e-12);
}

TEST(ConvexCorridorConstraint, JacobianMatchesFiniteDifferenceOnRandomCases)
{
    // 测试目的：在随机位姿与随机半空间参数下，验证 Cx 的符号求导稳定正确
    // 流程：随机生成 20 组 (x, p)，对每一组做中心差分并比较
    // 预期效果：所有样本的 Jacobian 与数值差分误差 < 1e-6
    std::mt19937 rng(42);
    std::uniform_real_distribution<double> x_dist(-5.0, 5.0);
    std::uniform_real_distribution<double> theta_dist(-PI + 0.1, PI - 0.1);
    std::uniform_real_distribution<double> v_dist(0.0, 5.0);
    std::uniform_real_distribution<double> kappa_dist(-0.3, 0.3);

    for (int trial = 0; trial < 20; ++trial) {
        Vector x(CORRIDOR_NX);
        x << x_dist(rng), x_dist(rng), theta_dist(rng), v_dist(rng), kappa_dist(rng);
        Vector u = Vector::Zero(NU);
        Vector p = build_random_parameters(x, rng);

        ConvexCorridorConstraint constraint(p, NU);
        verify_jacobian_against_finite_difference(constraint, x, u, p, 1e-7);
    }
}

TEST(ConvexCorridorConstraint, DimensionsAreCorrect)
{
    // 测试目的：验证约束的维度常量与文档一致
    // 流程：构造约束后检查 ng()、evaluate 输出长度、jacobian 输出矩阵形状
    // 预期效果：ng = CORRIDOR_G_DIM，Cx = 40x5，Cu = 40xnu
    Vector p = Vector::Zero(CORRIDOR_P_DIM);
    ConvexCorridorConstraint constraint(p, NU);
    EXPECT_EQ(constraint.ng(), CORRIDOR_G_DIM);

    Vector x(CORRIDOR_NX);
    x << 0.0, 0.0, 0.0, 0.0, 0.0;
    Vector u(NU);
    u << 0.1, 0.2;
    Vector g;
    constraint.evaluate(x, u, p, g);
    EXPECT_EQ(g.size(), CORRIDOR_G_DIM);

    Matrix Cx, Cu;
    constraint.jacobian(x, u, p, Cx, Cu);
    EXPECT_EQ(Cx.rows(), CORRIDOR_G_DIM);
    EXPECT_EQ(Cx.cols(), CORRIDOR_NX);
    EXPECT_EQ(Cu.rows(), CORRIDOR_G_DIM);
    EXPECT_EQ(Cu.cols(), NU);
}

TEST(ConvexCorridorConstraint, DifferentParametersUpdateBehavior)
{
    // 测试目的：验证显式传入不同半空间参数会影响约束值
    // 流程：同一约束实例先后以 p1、p2 调用 evaluate 并比较
    // 预期效果：两次约束值不同（参数确实生效）
    Vector x(CORRIDOR_NX);
    x << 1.0, 1.0, 0.0, 0.0, 0.0;
    Vector u = Vector::Zero(NU);

    std::mt19937 rng(123);
    Vector p1 = build_random_parameters(x, rng);
    Vector p2 = build_random_parameters(x, rng);

    ConvexCorridorConstraint constraint(NU);
    Vector g1, g2;
    constraint.evaluate(x, u, p1, g1);
    constraint.evaluate(x, u, p2, g2);
    EXPECT_FALSE(g1.isApprox(g2));
}

TEST(ConvexCorridorConstraint, AnalyticOracleForOneCornerAndHalfspace)
{
    // 测试目的：独立手算一个角点、一个半空间的 g 与 theta 导数，避免“实现自洽”掩盖漏掉角点或 theta 非线性
    // 流程：取单个半空间 A=(1,0), b=0，车辆前左角点 local=(L_F, W/2)，
    //      给定 x=[x0,y0,theta,0,0]，验证 g = x0 + L_F*cosθ - W/2*sinθ，
    //      d g / d theta = -L_F*sinθ - W/2*cosθ
    // 预期效果：evaluate 与 jacobian 与解析公式误差 < 1e-9
    Vector p = Vector::Zero(CORRIDOR_P_DIM);
    // 仅使用第一个半空间：A=(1,0), b=0
    p(HS_START + 0) = 1.0;
    p(HS_START + 1) = 0.0;
    p(HS_START + 2 * N_HS + 0) = 0.0;

    const double theta = 0.5;
    Vector x(CORRIDOR_NX);
    x << 1.0, 2.0, theta, 0.0, 0.0;
    Vector u = Vector::Zero(NU);

    ConvexCorridorConstraint constraint(p, NU);
    Vector g;
    constraint.evaluate(x, u, p, g);
    const double expected_g = x(0) + vehicle_geometry::kLf * std::cos(theta)
        - (vehicle_geometry::kWidth / 2.0) * std::sin(theta);
    EXPECT_NEAR(g(0), expected_g, 1e-9);

    Matrix Cx, Cu;
    constraint.jacobian(x, u, p, Cx, Cu);
    const double expected_dg_dtheta = -vehicle_geometry::kLf * std::sin(theta)
        - (vehicle_geometry::kWidth / 2.0) * std::cos(theta);
    EXPECT_NEAR(Cx(0, 0), 1.0, 1e-9); // dg/dx
    EXPECT_NEAR(Cx(0, 1), 0.0, 1e-9); // dg/dy
    EXPECT_NEAR(Cx(0, 2), expected_dg_dtheta, 1e-9); // dg/dtheta
    EXPECT_NEAR(Cx(0, 3), 0.0, 1e-9); // dg/dv
    EXPECT_NEAR(Cx(0, 4), 0.0, 1e-9); // dg/dkappa
}

TEST(ConvexCorridorConstraint, EvaluateAndJacobianMatchesSeparateCalls)
{
    // 测试目的：验证一次调用同时得到 g/Cx 的结果与分别调用 evaluate + jacobian 一致
    // 流程：随机生成 (x,p)，分别调用合并接口与单独接口，比较输出
    // 预期效果：g 与 Cx 完全一致
    Vector x(CORRIDOR_NX);
    x << 1.5, -0.5, 0.4, 1.0, 0.05;
    Vector u = Vector::Zero(NU);

    std::mt19937 rng(7);
    Vector p = build_random_parameters(x, rng);
    ConvexCorridorConstraint constraint(p, NU);

    Vector g_sep, g_comb;
    Matrix Cx_sep, Cx_comb, Cu_sep, Cu_comb;
    constraint.evaluate(x, u, p, g_sep);
    constraint.jacobian(x, u, p, Cx_sep, Cu_sep);
    constraint.evaluateAndJacobian(x, u, p, g_comb, Cx_comb, Cu_comb);

    EXPECT_TRUE(g_sep.isApprox(g_comb, 1e-12));
    EXPECT_TRUE(Cx_sep.isApprox(Cx_comb, 1e-12));
    EXPECT_TRUE(Cu_sep.isApprox(Cu_comb, 1e-12));
}

TEST(ConvexCorridorConstraint, CloneCreatesIndependentWorkspace)
{
    // 测试目的：验证 clone() 得到独立实例，两个实例互不污染工作区
    // 流程：clone 后分别对原实例与副本串行调用 evaluate，比较结果一致；再修改副本参数
    // 预期效果：副本与原实例结果相同，且修改副本参数不影响原实例
    Vector x(CORRIDOR_NX);
    x << 1.0, 2.0, 0.3, 0.0, 0.0;
    Vector u = Vector::Zero(NU);
    Vector p = Vector::Zero(CORRIDOR_P_DIM);
    p(HS_START + 0) = 1.0;
    p(HS_START + 2 * N_HS + 0) = 5.0;

    ConvexCorridorConstraint original(p, NU);
    auto copy_ptr = original.clone();
    auto& copy = dynamic_cast<ConvexCorridorConstraint&>(*copy_ptr);

    Vector g_orig, g_copy;
    original.evaluate(x, u, p, g_orig);
    copy.evaluate(x, u, p, g_copy);
    EXPECT_TRUE(g_orig.isApprox(g_copy, 1e-12));

    // 对副本使用不同参数后，原实例应保持不变
    Vector p2 = p;
    p2(HS_START + 2 * N_HS + 0) = 10.0;
    copy.evaluate(x, u, p2, g_copy);
    EXPECT_FALSE(g_orig.isApprox(g_copy));
    original.evaluate(x, u, p, g_orig);
    EXPECT_NEAR(g_orig(0),
        x(0) + vehicle_geometry::kLf * std::cos(x(2))
            - (vehicle_geometry::kWidth / 2.0) * std::sin(x(2)) - 5.0,
        1e-9);
}

TEST(ConvexCorridorConstraint, CloneSupportsConcurrentEvaluation)
{
    // 测试目的：验证 clone() 后的副本与原实例可在多线程中并发调用 evaluateAndJacobian 而不产生数据竞争
    // 流程：主实例与克隆实例各交给一个线程，对同一点反复执行 1000 次 evaluateAndJacobian，
    //      与主线程预先算好的参考值比较
    // 预期效果：所有调用均成功，结果与参考值一致
    Vector x(CORRIDOR_NX);
    x << 1.0, 2.0, 0.3, 0.0, 0.0;
    Vector u = Vector::Zero(NU);
    Vector p = Vector::Zero(CORRIDOR_P_DIM);
    p(HS_START + 0) = 1.0;
    p(HS_START + 2 * N_HS + 0) = 5.0;

    ConvexCorridorConstraint original(p, NU);
    auto cloned_ptr = original.clone();
    auto& cloned = dynamic_cast<ConvexCorridorConstraint&>(*cloned_ptr);

    Vector g_ref;
    Matrix Cx_ref, Cu_ref;
    original.evaluateAndJacobian(x, u, p, g_ref, Cx_ref, Cu_ref);

    std::atomic<int> success_count(0);
    const int iterations = 1000;
    auto worker = [&](ConvexCorridorConstraint& c) {
        for (int i = 0; i < iterations; ++i) {
            Vector g;
            Matrix Cx, Cu;
            c.evaluateAndJacobian(x, u, p, g, Cx, Cu);
            if (g.isApprox(g_ref, 1e-12) && Cx.isApprox(Cx_ref, 1e-12)
                && Cu.isApprox(Cu_ref, 1e-12)) {
                success_count.fetch_add(1, std::memory_order_relaxed);
            }
        }
    };

    std::thread t1([&]() { worker(original); });
    std::thread t2([&]() { worker(cloned); });
    t1.join();
    t2.join();
    EXPECT_EQ(success_count.load(), 2 * iterations)
        << "Concurrent calls produced abnormal results or data race";
}

TEST(ConvexCorridorConstraint, RejectsInvalidDimensions)
{
    // 测试目的：验证构造期与运行期对非法维度抛出 std::invalid_argument
    // 流程：分别传入错误 p 维度、错误 nu、错误 x 维度、错误 u 维度，断言抛异常
    // 预期效果：所有非法输入都被拒绝
    Vector good_p = Vector::Zero(CORRIDOR_P_DIM);
    Vector bad_p = Vector::Zero(CORRIDOR_P_DIM - 1);
    Vector good_x(CORRIDOR_NX);
    good_x.setZero();
    Vector bad_x(CORRIDOR_NX - 1);
    bad_x.setZero();
    Vector good_u = Vector::Zero(NU);
    Vector bad_u = Vector::Zero(NU - 1);

    EXPECT_THROW(ConvexCorridorConstraint constraint(bad_p, NU), std::invalid_argument);
    EXPECT_THROW(ConvexCorridorConstraint constraint(good_p, -1), std::invalid_argument);

    ConvexCorridorConstraint constraint(good_p, NU);
    Vector g;
    Matrix Cx, Cu;
    EXPECT_THROW(constraint.evaluate(bad_x, good_u, good_p, g), std::invalid_argument);
    EXPECT_THROW(constraint.evaluate(good_x, bad_u, good_p, g), std::invalid_argument);
    EXPECT_THROW(constraint.jacobian(bad_x, good_u, good_p, Cx, Cu), std::invalid_argument);
}
