#include <cmath>

#include <gtest/gtest.h>

#include "../src/generated/dynamics_delta.h"
#include "../src/generated/dynamics_delta_jac.h"
#include "../src/generated/dynamics_kappa.h"
#include "../src/generated/dynamics_kappa_jac.h"
#include "../src/math/math_util.hpp"
#include "../src/models/bicycle_model_delta.h"
#include "../src/models/bicycle_model_kappa.h"
#include "../src/models/casadi_wrapper.h"

using namespace stc_SQP;

namespace {
// 独立的 RK4 积分器，仅使用 evaluate，作为数值差分的无偏基准
Vector rk4_integrate(const DynamicalSystem& model, const Vector& x, const Vector& u,
    double dt)
{
    Vector k1, k2, k3, k4;
    Vector x2, x3, x4;
    model.evaluate(x, u, k1);
    x2 = x + 0.5 * dt * k1;
    model.evaluate(x2, u, k2);
    x3 = x + 0.5 * dt * k2;
    model.evaluate(x3, u, k3);
    x4 = x + dt * k3;
    model.evaluate(x4, u, k4);
    return x + dt / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}

// 通用 Jacobian 数值差分校验：验证 model 的 discretizeAndLinearize 输出 A/B 与 RK4 差分一致
template <typename ModelType>
void verify_jacobian_against_finite_difference(const ModelType& model, const Vector& x,
    const Vector& u, double dt, double v_sign = 1.0)
{
    Vector x_next;
    Matrix A, B;
    model.discretizeAndLinearize(x, u, dt, v_sign, x_next, A, B);
    const Vector x_next_fd = rk4_integrate(model, x, u, dt);
    EXPECT_TRUE(x_next.isApprox(x_next_fd, 1e-6))
        << "RK4 state propagation differs from independent implementation";
    const double eps = 1e-7;
    Matrix A_fd(model.nx(), model.nx());
    for (int i = 0; i < model.nx(); ++i) {
        Vector x_perturbed = x;
        x_perturbed(i) += eps;
        const Vector x_next_perturbed = rk4_integrate(model, x_perturbed, u, dt);
        A_fd.col(i) = (x_next_perturbed - x_next_fd) / eps;
    }
    EXPECT_TRUE(A.isApprox(A_fd, 1e-6)) << "State Jacobian A differs from numerical difference";
    Matrix B_fd(model.nx(), model.nu());
    for (int i = 0; i < model.nu(); ++i) {
        Vector u_perturbed = u;
        u_perturbed(i) += eps;
        const Vector x_next_perturbed = rk4_integrate(model, x, u_perturbed, dt);
        B_fd.col(i) = (x_next_perturbed - x_next_fd) / eps;
    }
    EXPECT_TRUE(B.isApprox(B_fd, 1e-6)) << "Input Jacobian B differs from numerical difference";
}

// 测试派生类：暴露 protected 的 computeContinuousJacobians 供白盒测试
class TestableBicycleModelKappa : public BicycleModelKappa {
public:
    void expose_continuous_jacobians(const Vector& x, Matrix& A_c, Matrix& B_c) const
    {
        computeContinuousJacobians(x, A_c, B_c);
    }
};

class TestableBicycleModelDelta : public BicycleModelDelta {
public:
    explicit TestableBicycleModelDelta(double wheelbase)
        : BicycleModelDelta(wheelbase)
    {
    }
    void expose_continuous_jacobians(const Vector& x, Matrix& A_c, Matrix& B_c) const
    {
        computeContinuousJacobians(x, A_c, B_c);
    }
};

// 将 CasADi 稀疏矩阵（CCS 格式）展开为 Eigen 稠密矩阵
void casadi_sparse_to_dense(const long long* sp, const double* values, Matrix& M)
{
    const int rows = static_cast<int>(sp[0]);
    const int cols = static_cast<int>(sp[1]);
    const long long* col_ptr = &sp[2];
    const long long* row_idx = &sp[2 + cols + 1];
    M.setZero(rows, cols);
    int value_idx = 0;
    for (int col = 0; col < cols; ++col) {
        for (long long k = col_ptr[col]; k < col_ptr[col + 1]; ++k) {
            M(static_cast<int>(row_idx[k]), col) = values[value_idx++];
        }
    }
}

// 调用 CasADi 生成的 dynamics_*_jac，解包返回连续时间 f、A、B。
// 注意 CasADi 生成的 A/B 是稀疏矩阵，必须按 sparsity_out 展开，不能直接当稠密数据读。
void evaluate_casadi_jac(casadi::CasADiFunction& jac_func,
    const long long* (*sparsity_out)(long long), const Vector& x,
    const Vector& u, int nx, int nu, Vector& f, Matrix& A, Matrix& B)
{
    f.resize(nx);
    A.resize(nx, nx);
    B.resize(nx, nu);
    // 先查询非零元个数以分配足够缓冲区
    const long long* sp_a = sparsity_out(1);
    const long long* sp_b = sparsity_out(2);
    const int nnz_a = static_cast<int>(sp_a[2 + nx]); // colptr[n]
    const int nnz_b = static_cast<int>(sp_b[2 + nu]);
    std::vector<double> values_a(nnz_a);
    std::vector<double> values_b(nnz_b);
    std::vector<const double*> arg = { x.data(), u.data() };
    std::vector<double*> res = { f.data(), values_a.data(), values_b.data() };
    jac_func(arg, res);
    casadi_sparse_to_dense(sp_a, values_a.data(), A);
    casadi_sparse_to_dense(sp_b, values_b.data(), B);
}

} // namespace

// BicycleModelKappa 测试：曲率控制版阿克曼模型
TEST(BicycleModelKappa, EvaluateProducesExpectedDynamics)
{
    // 测试目的：验证连续动力学 evaluate 输出与解析公式一致
    // 流程：给定固定状态与控制，调用 evaluate 后逐项比较
    // 预期效果：xdot = [v*cosθ, v*sinθ, v*κ, a, κ_dot]
    BicycleModelKappa model;
    Vector x(5), u(2), xdot(5);
    x << 1.0, 2.0, 0.5, 3.0, 0.1;
    u << 1.5, 0.05;
    model.evaluate(x, u, xdot);
    EXPECT_NEAR(xdot(0), 3.0 * std::cos(0.5), 1e-12);
    EXPECT_NEAR(xdot(1), 3.0 * std::sin(0.5), 1e-12);
    EXPECT_NEAR(xdot(2), 3.0 * 0.1, 1e-12);
    EXPECT_NEAR(xdot(3), 1.5, 1e-12);
    EXPECT_NEAR(xdot(4), 0.05, 1e-12);
}

TEST(BicycleModelKappa, DiscretizeAndLinearizeMatchesFiniteDifference)
{
    // 测试目的：验证手写推导的离散 Jacobian A/B 与 RK4 数值差分一致
    // 流程：取典型状态与控制，调用 discretizeAndLinearize，再用 RK4 做中心差分
    // 预期效果：A、B 的每一列与差分结果误差 < 1e-6
    BicycleModelKappa model;
    Vector x(5), u(2);
    x << 1.0, 0.5, 0.3, 2.0, 0.05;
    u << 0.5, 0.02;
    verify_jacobian_against_finite_difference(model, x, u, 0.05);
}

TEST(BicycleModelKappa, RetractWrapsThetaOnSO2)
{
    // 测试目的：验证 retract 对 theta 维度使用 SO2 流形更新，其余维度线性相加
    // 流程：构造状态与增量，调用 retract 后检查各维度
    // 预期效果：theta_new = retract(theta, delta_theta)，其他维度直接相加
    BicycleModelKappa model;
    Vector x(5), delta(5), x_new(5);
    x << 0.0, 0.0, 3.0, 1.0, 0.0;
    delta << 1.0, 2.0, 0.5, 0.0, 0.0;
    model.retract(x, 1.0, delta, x_new);
    EXPECT_NEAR(x_new(0), 1.0, 1e-12);
    EXPECT_NEAR(x_new(1), 2.0, 1e-12);
    EXPECT_NEAR(x_new(2), so2::Retract(3.0, 0.5), 1e-12);
    EXPECT_NEAR(x_new(3), 1.0, 1e-12);
    EXPECT_NEAR(x_new(4), 0.0, 1e-12);
}

TEST(BicycleModelKappa, RetractHandlesAngleWrapAcrossPiBoundary)
{
    // 测试目的：验证 retract 在跨越 π/-π 边界时正确回绕
    // 流程：取接近 π 的角度并施加正向增量
    // 预期效果：结果落在 [-π, π] 区间内
    BicycleModelKappa model;
    Vector x(5), delta(5), x_new(5);
    x << 0.0, 0.0, PI - 0.1, 1.0, 0.0;
    delta << 0.0, 0.0, 0.3, 0.0, 0.0;
    model.retract(x, 1.0, delta, x_new);
    EXPECT_NEAR(x_new(2), so2::Retract(PI - 0.1, 0.3), 1e-12);
    EXPECT_GT(x_new(2), -PI);
    EXPECT_LE(x_new(2), PI);
}

TEST(BicycleModelKappa, CrossValidationWithCasADiGeneratedDynamics)
{
    // 测试目的：验证 C++ 手写解析动力学与 CasADi 生成代码在数学上严格等价
    // 流程：实例化 CasADiWrapper 加载 dynamics_kappa.c，对同一 (x,u) 比较 evaluate 输出
    // 预期效果：两者差值 L2 范数 < 1e-8
    BicycleModelKappa hand_model;
    casadi::CasADiFunction casadi_dynamics(
        reinterpret_cast<casadi::FunctionPointer>(dynamics_kappa),
        reinterpret_cast<casadi::WorkSizeFunction>(dynamics_kappa_work), 2, 1);
    Vector x(5), u(2);
    x << 1.0, 0.5, 0.3, 2.0, 0.05;
    u << 0.5, 0.02;
    Vector hand_xdot(5);
    hand_model.evaluate(x, u, hand_xdot);
    Vector casadi_xdot(5);
    std::vector<const double*> arg = { x.data(), u.data() };
    std::vector<double*> res = { casadi_xdot.data() };
    casadi_dynamics(arg, res);
    EXPECT_TRUE(hand_xdot.isApprox(casadi_xdot, 1e-8))
        << "Handwritten model differs from CasADi generated dynamics output";
}

// BicycleModelDelta 测试：前轮转角控制版阿克曼模型
TEST(BicycleModelDelta, EvaluateProducesExpectedDynamics)
{
    // 测试目的：验证 Delta 版连续动力学 evaluate 输出与解析公式一致
    // 流程：给定固定状态与控制，调用 evaluate 后逐项比较
    // 预期效果：xdot = [v*cosθ, v*sinθ, v*tanδ/L, a, δ_dot]
    BicycleModelDelta model(2.8);
    Vector x(5), u(2), xdot(5);
    x << 1.0, 2.0, 0.5, 3.0, 0.1;
    u << 1.5, 0.05;
    model.evaluate(x, u, xdot);
    const double L = 2.8;
    EXPECT_NEAR(xdot(0), 3.0 * std::cos(0.5), 1e-12);
    EXPECT_NEAR(xdot(1), 3.0 * std::sin(0.5), 1e-12);
    EXPECT_NEAR(xdot(2), 3.0 * std::tan(0.1) / L, 1e-12);
    EXPECT_NEAR(xdot(3), 1.5, 1e-12);
    EXPECT_NEAR(xdot(4), 0.05, 1e-12);
}

TEST(BicycleModelDelta, DiscretizeAndLinearizeMatchesFiniteDifference)
{
    // 测试目的：验证 Delta 版手写推导的离散 Jacobian A/B 与 RK4 数值差分一致
    // 流程：取典型状态与控制，调用 discretizeAndLinearize，再用 RK4 做差分
    // 预期效果：A、B 的每一列与差分结果误差 < 1e-6
    BicycleModelDelta model(2.8);
    Vector x(5), u(2);
    x << 1.0, 0.5, 0.3, 2.0, 0.1;
    u << 0.5, 0.02;
    verify_jacobian_against_finite_difference(model, x, u, 0.05);
}

TEST(BicycleModelDelta, RetractWrapsThetaOnSO2)
{
    // 测试目的：验证 Delta 版 retract 同样对 theta 使用 SO2 流形更新
    // 流程：构造状态与增量，调用 retract 后检查 theta 维度
    // 预期效果：theta_new = retract(theta, delta_theta)
    BicycleModelDelta model(2.8);
    Vector x(5), delta(5), x_new(5);
    x << 0.0, 0.0, PI - 0.1, 1.0, 0.0;
    delta << 0.0, 0.0, 0.3, 0.0, 0.0;
    model.retract(x, 1.0, delta, x_new);
    EXPECT_NEAR(x_new(2), so2::Retract(PI - 0.1, 0.3), 1e-12);
}

TEST(BicycleModelDelta, CrossValidationWithCasADiGeneratedDynamics)
{
    // 测试目的：验证 Delta 版 C++ 手写解析动力学与 CasADi 生成代码等价
    // 流程：加载 dynamics_delta.c，对同一 (x,u) 比较 evaluate 输出
    // 预期效果：两者差值 L2 范数 < 1e-8
    BicycleModelDelta hand_model(2.8);
    casadi::CasADiFunction casadi_dynamics(
        reinterpret_cast<casadi::FunctionPointer>(dynamics_delta),
        reinterpret_cast<casadi::WorkSizeFunction>(dynamics_delta_work), 2, 1);
    Vector x(5), u(2);
    x << 1.0, 0.5, 0.3, 2.0, 0.1;
    u << 0.5, 0.02;
    Vector hand_xdot(5);
    hand_model.evaluate(x, u, hand_xdot);
    Vector casadi_xdot(5);
    std::vector<const double*> arg = { x.data(), u.data() };
    std::vector<double*> res = { casadi_xdot.data() };
    casadi_dynamics(arg, res);
    EXPECT_TRUE(hand_xdot.isApprox(casadi_xdot, 1e-8))
        << "Delta handwritten model differs from CasADi generated dynamics output";
}

TEST(BicycleModelKappa, ContinuousJacobianCrossValidationWithCasADi)
{
    // 测试目的：验证手写连续 Jacobian A_c/B_c 与 CasADi 符号推导结果一致
    // 流程：通过测试派生类 expose_continuous_jacobians 获取手写 A_c/B_c，
    //      再调用 dynamics_kappa_jac 解包 CasADi 的 A/B，逐元素比较
    // 预期效果：两者差值 < 1e-8
    TestableBicycleModelKappa hand_model;
    casadi::CasADiFunction casadi_jac(
        reinterpret_cast<casadi::FunctionPointer>(dynamics_kappa_jac),
        reinterpret_cast<casadi::WorkSizeFunction>(dynamics_kappa_jac_work), 2, 3);
    Vector x(5), u(2);
    x << 1.0, 0.5, 0.3, 2.0, 0.05;
    u << 0.5, 0.02;
    Matrix hand_A(5, 5), hand_B(5, 2);
    hand_model.expose_continuous_jacobians(x, hand_A, hand_B);
    Vector casadi_f(5);
    Matrix casadi_A(5, 5), casadi_B(5, 2);
    evaluate_casadi_jac(casadi_jac, dynamics_kappa_jac_sparsity_out, x, u, 5, 2, casadi_f,
        casadi_A, casadi_B);
    EXPECT_TRUE(hand_A.isApprox(casadi_A, 1e-8))
        << "Handwritten A_c differs from CasADi symbolic Jacobian A";
    EXPECT_TRUE(hand_B.isApprox(casadi_B, 1e-8))
        << "Handwritten B_c differs from CasADi symbolic Jacobian B";
}

TEST(BicycleModelKappa, DiscretizedThetaIsWrappedAfterPropagation)
{
    // 测试目的：验证 discretizeAndLinearize 输出的 theta 已被规范化到 (-π, π]
    // 流程：取 theta 接近 π 且 v*κ 为正的状态，积分后 theta 应跨过 π 边界并回绕
    // 预期效果：x_next(2) 落在 (-π, π] 区间内
    BicycleModelKappa model;
    Vector x(5), u(2), x_next(5);
    Matrix A(5, 5), B(5, 2);
    x << 0.0, 0.0, PI - 0.05, 2.0, 0.5;
    u << 0.0, 0.0;
    model.discretizeAndLinearize(x, u, 0.1, 1.0, x_next, A, B);
    EXPECT_GT(x_next(2), -PI);
    EXPECT_LE(x_next(2), PI);
    EXPECT_NEAR(x_next(2), math_util::NormalizeAngle(x(2) + x(3) * x(4) * 0.1), 1e-3);
}

TEST(BicycleModelDelta, ContinuousJacobianCrossValidationWithCasADi)
{
    // 测试目的：验证 Delta 版手写连续 Jacobian A_c/B_c 与 CasADi 符号推导结果一致
    // 流程：同 Kappa 版，调用 dynamics_delta_jac 解包 A/B 并比较
    // 预期效果：两者差值 < 1e-8
    TestableBicycleModelDelta hand_model(2.8);
    casadi::CasADiFunction casadi_jac(
        reinterpret_cast<casadi::FunctionPointer>(dynamics_delta_jac),
        reinterpret_cast<casadi::WorkSizeFunction>(dynamics_delta_jac_work), 2, 3);
    Vector x(5), u(2);
    x << 1.0, 0.5, 0.3, 2.0, 0.1;
    u << 0.5, 0.02;
    Matrix hand_A(5, 5), hand_B(5, 2);
    hand_model.expose_continuous_jacobians(x, hand_A, hand_B);
    Vector casadi_f(5);
    Matrix casadi_A(5, 5), casadi_B(5, 2);
    evaluate_casadi_jac(casadi_jac, dynamics_delta_jac_sparsity_out, x, u, 5, 2, casadi_f,
        casadi_A, casadi_B);
    EXPECT_TRUE(hand_A.isApprox(casadi_A, 1e-8))
        << "Delta handwritten A_c differs from CasADi symbolic Jacobian A";
    EXPECT_TRUE(hand_B.isApprox(casadi_B, 1e-8))
        << "Delta handwritten B_c differs from CasADi symbolic Jacobian B";
}

TEST(BicycleModelDelta, DiscretizedThetaIsWrappedAfterPropagation)
{
    // 测试目的：验证 Delta 版 discretizeAndLinearize 输出的 theta 已被规范化
    // 流程：取 theta 接近 π 且 v*tanδ/L 为正的状态，积分后 theta 应跨过 π 边界
    // 预期效果：x_next(2) 落在 (-π, π] 区间内
    BicycleModelDelta model(2.8);
    Vector x(5), u(2), x_next(5);
    Matrix A(5, 5), B(5, 2);
    x << 0.0, 0.0, PI - 0.05, 2.0, 0.1;
    u << 0.0, 0.0;
    model.discretizeAndLinearize(x, u, 0.1, 1.0, x_next, A, B);
    EXPECT_GT(x_next(2), -PI);
    EXPECT_LE(x_next(2), PI);
}

TEST(BicycleModelDelta, RejectsNonPositiveWheelbase)
{
    // 测试目的：验证 BicycleModelDelta 对非法轴距（<=0）抛出异常
    // 流程：尝试用 0 或负数构造模型
    // 预期效果：抛出 std::invalid_argument
    EXPECT_THROW(BicycleModelDelta model(0.0), std::invalid_argument);
    EXPECT_THROW(BicycleModelDelta model(-1.0), std::invalid_argument);
}

TEST(BicycleModelKappa, DiscretizeMatchesDiscretizeAndLinearizeXNext)
{
    // 测试目的：验证轻量 discretize() 接口返回的 x_next 与 discretizeAndLinearize() 一致
    // 流程：对同一 (x, u, dt, v_sign) 分别调用两个接口，比较 x_next
    // 预期效果：两者差值 < 1e-12，且 theta 均被规范化
    BicycleModelKappa model;
    Vector x(5), u(2);
    x << 0.5, -0.3, PI - 0.1, 1.5, 0.2;
    u << 0.2, 0.05;
    Vector x_next_full(5), x_next_light(5);
    Matrix A(5, 5), B(5, 2);
    model.discretizeAndLinearize(x, u, 0.1, -1.0, x_next_full, A, B);
    model.discretize(x, u, 0.1, -1.0, x_next_light);
    EXPECT_TRUE(x_next_full.isApprox(x_next_light, 1e-12))
        << "discretize() and discretizeAndLinearize() x_next differ";
    EXPECT_GT(x_next_light(2), -PI);
    EXPECT_LE(x_next_light(2), PI);
}

TEST(BicycleModelDelta, DiscretizeMatchesDiscretizeAndLinearizeXNext)
{
    // 测试目的：验证 Delta 版轻量 discretize() 接口返回的 x_next 与 discretizeAndLinearize() 一致
    // 流程：对同一 (x, u, dt, v_sign) 分别调用两个接口，比较 x_next
    // 预期效果：两者差值 < 1e-12，且 theta 均被规范化
    BicycleModelDelta model(2.8);
    Vector x(5), u(2);
    x << 0.5, -0.3, PI - 0.1, 1.5, 0.2;
    u << 0.2, 0.05;
    Vector x_next_full(5), x_next_light(5);
    Matrix A(5, 5), B(5, 2);
    model.discretizeAndLinearize(x, u, 0.1, -1.0, x_next_full, A, B);
    model.discretize(x, u, 0.1, -1.0, x_next_light);
    EXPECT_TRUE(x_next_full.isApprox(x_next_light, 1e-12))
        << "discretize() and discretizeAndLinearize() x_next differ";
    EXPECT_GT(x_next_light(2), -PI);
    EXPECT_LE(x_next_light(2), PI);
}
