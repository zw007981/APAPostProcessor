#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include <gtest/gtest.h>

#include "constraints/box_constraint.h"
#include "constraints/constraint.hpp"
#include "util/constants.h"
#include "costs/cost_term.hpp"
#include "costs/quadratic_tracking.h"
#include "models/bicycle_model_kappa.h"
#include "models/dynamical_system.h"
#include "ocp/multi_stage_ocp.h"
#include "qp/dense_qp_solver.h"
#include "qp/hpipm_solver.h"
#include "qp/qp_data.h"
#include "qp/qp_solution.h"
#include "qp/qp_solver.h"
#include "sqp/sqp_algorithm.h"
#include "util/constants.h"

using namespace stc_SQP;

// ===================== 测试辅助：双积分器线性动力学 =====================
class DoubleIntegrator : public DynamicalSystem {
public:
    DoubleIntegrator(const Matrix& A, const Matrix& B)
        : A_(A)
        , B_(B)
    {
    }
    int nx() const override { return A_.rows(); }
    int nu() const override { return B_.cols(); }
    void evaluate(const Vector& x, const Vector& u, Vector& x_dot) const override
    {
        (void)x;
        (void)u;
        (void)x_dot;
    }
    void discretizeAndLinearize(const Vector& x, const Vector& u, double dt,
        double v_sign, Vector& x_next, Matrix& A, Matrix& B) const override
    {
        (void)dt;
        (void)v_sign; // 测试用双积分器不解释方向符号
        x_next = A_ * x + B_ * u;
        A = A_;
        B = B_;
    }

private:
    Matrix A_;
    Matrix B_;
};

// ===================== 测试辅助：一般控制约束 g(u) = u(idx) - limit <= 0 =====================
class ControlUpperBoundConstraint : public Constraint {
public:
    ControlUpperBoundConstraint(int control_index, double limit)
        : control_index_(control_index)
        , limit_(limit)
    {
    }
    int ng() const override { return 1; }
    void evaluate(const Vector& x, const Vector& u, const Vector& p, Vector& g) const override
    {
        (void)x;
        (void)p;
        g.resize(1);
        g(0) = u(control_index_) - limit_;
    }
    void jacobian(const Vector& x, const Vector& u, const Vector& p, Matrix& Cx,
        Matrix& Cu) const override
    {
        (void)x;
        (void)u;
        (void)p;
        Cx.setZero(1, x.size());
        Cu.setZero(1, u.size());
        Cu(0, control_index_) = 1.0;
    }
    std::shared_ptr<Constraint> clone() const override
    {
        return std::make_shared<ControlUpperBoundConstraint>(control_index_, limit_);
    }

private:
    int control_index_ = 0;
    double limit_ = 0.0;
};

// ===================== 测试辅助：记录收到 v_sign 的 mock 动力学 =====================
class SignRecordingDynamics : public DynamicalSystem {
public:
    SignRecordingDynamics(int nx, int nu)
        : nx_(nx)
        , nu_(nu)
    {
    }
    int nx() const override { return nx_; }
    int nu() const override { return nu_; }
    void evaluate(const Vector& x, const Vector& u, Vector& x_dot) const override
    {
        (void)x;
        (void)u;
        x_dot.setZero(nx_);
    }
    void discretizeAndLinearize(const Vector& x, const Vector& u, double dt,
        double v_sign, Vector& x_next, Matrix& A, Matrix& B) const override
    {
        (void)x;
        (void)u;
        (void)dt;
        received_v_sign_ = v_sign;
        x_next = Vector::Zero(nx_);
        A = Matrix::Identity(nx_, nx_);
        B = Matrix::Zero(nx_, nu_);
    }
    double receivedVSign() const { return received_v_sign_; }

private:
    int nx_ = 0;
    int nu_ = 0;
    mutable double received_v_sign_ = 0.0;
};

// ===================== 测试辅助：白盒访问 SQPSolver 内部方法 =====================
class SQPSolverTestAccess : public SQPSolver {
public:
    explicit SQPSolverTestAccess(std::unique_ptr<QPSolver> qp_solver)
        : SQPSolver(std::move(qp_solver))
    {
    }
    using SQPSolver::applyRetraction;
    const Trajectory& currentTraj() const { return current_traj_; }
};

// ===================== 测试辅助：总是返回失败的 QP 求解器 =====================
class FailingQPSolver : public QPSolver {
public:
    QPSolverStatus solve(const QPData&, QPSolution&) override
    {
        return QPSolverStatus::INFEASIBLE;
    }
    void setTolerance(double) override {}
    void setWarmStart(const QPSolution&) override {}
};

// ===================== 测试辅助：返回固定 delta 的 Mock QP 求解器 =====================
class MockDeltaQPSolver : public QPSolver {
public:
    explicit MockDeltaQPSolver(const Trajectory& delta)
        : delta_(delta)
    {
    }
    QPSolverStatus solve(const QPData& qp_data, QPSolution& qp_sol) override
    {
        for (int k = 0; k <= qp_data.N; ++k) {
            qp_sol.x[k] = delta_.x[k];
        }
        for (int k = 0; k < qp_data.N; ++k) {
            qp_sol.u[k] = delta_.u[k];
        }
        return QPSolverStatus::SUCCESS;
    }
    void setTolerance(double) override {}
    void setWarmStart(const QPSolution&) override {}

private:
    Trajectory delta_;
};

// ===================== 测试辅助：OCP 构造工厂 =====================
StageSegment makeDoubleIntegratorSegment(int N, double dt, const Matrix& A,
    const Matrix& B, const Vector& x_ref, const Matrix& Q, const Matrix& R,
    const Vector& x_min, const Vector& x_max, const Vector& u_min,
    const Vector& u_max)
{
    StageSegment segment;
    segment.dynamics = std::make_shared<DoubleIntegrator>(A, B);
    segment.cost = std::make_shared<QuadraticTrackingCost>(x_ref, Q, R);
    segment.N = N;
    segment.dt = dt;
    segment.v_sign = 1.0;
    segment.x_min = x_min;
    segment.x_max = x_max;
    segment.u_min = u_min;
    segment.u_max = u_max;
    return segment;
}

MultiStageOCP makeDoubleIntegratorOCP(int N, double dt, const Matrix& A,
    const Matrix& B, const Vector& x_ref, const Matrix& Q, const Matrix& R,
    const Vector& x_min, const Vector& x_max, const Vector& u_min,
    const Vector& u_max)
{
    MultiStageOCP ocp;
    ocp.addSegment(makeDoubleIntegratorSegment(N, dt, A, B, x_ref, Q, R,
        x_min, x_max, u_min, u_max));
    return ocp;
}

Trajectory makeZeroTrajectory(int N, int nx, int nu)
{
    Trajectory traj;
    traj.resize(N, nx, nu);
    for (auto& xk : traj.x) {
        xk.setZero();
    }
    for (auto& uk : traj.u) {
        uk.setZero();
    }
    return traj;
}

// ===================== 测试用例 =====================

TEST(SQPAlgorithm, DetectsGearShiftAndDowngradesRTI)
{
    // 测试目的：当 OCP 存在相邻段速度方向变号（换挡点）时，
    //          SQP 引擎必须自动在本次 solve 中将 use_rti 降级并设置降级标记
    // 流程：构造前进(N=5)+后退(N=5)的两段 OCP，初始 use_rti=true，调用 solve
    // 预期效果：rtiDowngraded() 返回 true，solve 返回成功；
    //          options().use_rti 不应被永久改写
    const int n1 = 5, n2 = 5;
    const int nx = 5, nu = 2;
    const Vector x_ref = Vector::Zero(nx);
    const Matrix Q = Matrix::Identity(nx, nx) * 1e-3;
    const Matrix R = Matrix::Identity(nu, nu) * 1e-3;
    const Vector x_min = Vector::Constant(nx, -1e3);
    const Vector x_max = Vector::Constant(nx, 1e3);
    const Vector u_min = Vector::Constant(nu, -1e3);
    const Vector u_max = Vector::Constant(nu, 1e3);

    MultiStageOCP ocp;
    auto dynamics = std::make_shared<BicycleModelKappa>();
    StageSegment seg1;
    seg1.dynamics = dynamics;
    seg1.cost = std::make_shared<QuadraticTrackingCost>(x_ref, Q, R);
    seg1.N = n1;
    seg1.dt = 0.1;
    seg1.v_sign = +1.0;
    seg1.x_min = x_min;
    seg1.x_max = x_max;
    seg1.u_min = u_min;
    seg1.u_max = u_max;
    ocp.addSegment(seg1);

    StageSegment seg2;
    seg2.dynamics = dynamics;
    seg2.cost = std::make_shared<QuadraticTrackingCost>(x_ref, Q, R);
    seg2.N = n2;
    seg2.dt = 0.1;
    seg2.v_sign = -1.0;
    seg2.x_min = x_min;
    seg2.x_max = x_max;
    seg2.u_min = u_min;
    seg2.u_max = u_max;
    ocp.addSegment(seg2);

    ASSERT_TRUE(ocp.hasGearShift());

    Trajectory init_guess = makeZeroTrajectory(ocp.totalSteps(), ocp.nx(), ocp.nu());
    SQPSolver solver(std::make_unique<DenseQPSolver>());
    solver.options().use_rti = true;

    Trajectory solution;
    const bool ok = solver.solve(ocp, init_guess, solution);
    EXPECT_TRUE(ok);
    EXPECT_TRUE(solver.rtiDowngraded());
    // use_rti 不应被永久改写，后续复用 solver 时仍可使用 RTI
    EXPECT_TRUE(solver.options().use_rti);
}

TEST(SQPAlgorithm, RetractionAvoidsTwoPiJump)
{
    // 测试目的：验证 applyRetraction 通过 dynamics()->retract() 更新 theta，
    //          使得跨越 π/-π 边界的角度不会产生 2π 跳变
    // 流程：构造 BicycleModelKappa 单段 OCP，当前 theta = π - 0.1，
    //      delta theta = +0.3（越界），调用 applyRetraction(alpha=1)
    // 预期效果：结果 theta = NormalizeAngle(π - 0.1 + 0.3) = -π + 0.2，而非 π + 0.2
    MultiStageOCP ocp;
    auto dynamics = std::make_shared<BicycleModelKappa>();
    StageSegment segment;
    segment.dynamics = dynamics;
    segment.N = 1;
    segment.dt = 0.1;
    segment.v_sign = 1.0;
    segment.x_min = Vector::Constant(5, -1e3);
    segment.x_max = Vector::Constant(5, 1e3);
    segment.u_min = Vector::Constant(2, -1e3);
    segment.u_max = Vector::Constant(2, 1e3);
    ocp.addSegment(segment);

    Trajectory current, delta, result;
    current.resize(1, 5, 2);
    delta.resize(1, 5, 2);
    result.resize(1, 5, 2);

    current.x[0] << 0.0, 0.0, PI - 0.1, 0.5, 0.0;
    current.x[1] << 0.0, 0.0, PI - 0.1, 0.5, 0.0;
    current.u[0] << 0.0, 0.0;

    delta.x[0].setZero();
    delta.x[0](2) = 0.3; // 越界增量
    delta.x[1].setZero();
    delta.x[1](2) = 0.3;
    delta.u[0] << 0.1, 0.2;

    SQPSolverTestAccess solver(nullptr);
    EXPECT_TRUE(solver.applyRetraction(ocp, current, 1.0, delta, result));

    const double expected_theta = math_util::NormalizeAngle(PI - 0.1 + 0.3);
    EXPECT_NEAR(result.x[0](2), expected_theta, 1e-12);
    EXPECT_NEAR(result.x[1](2), expected_theta, 1e-12);
    EXPECT_FALSE(std::isnan(result.x[0](2)));
    EXPECT_FALSE(std::isnan(result.x[1](2)));

    // 控制应为线性相加
    EXPECT_NEAR(result.u[0](0), 0.1, 1e-12);
    EXPECT_NEAR(result.u[0](1), 0.2, 1e-12);
}

TEST(SQPAlgorithm, QPFailurePreventsApplyingDelta)
{
    // 测试目的：验证 solveQP 返回非 SUCCESS 时，SQP 引擎不会将 delta_traj_ 应用到 current_traj_
    // 流程：构造 OCP 与初始猜测，注入总是返回 INFEASIBLE 的 FailingQPSolver，调用 solve
    // 预期效果：solve 返回 false，current_traj_ 保持为初始猜测（未被污染）
    const int nx = 2, nu = 1, N = 3;
    const double dt = 0.1;
    Matrix A(nx, nx);
    A << 1.0, dt,
        0.0, 1.0;
    Matrix B(nx, nu);
    B << 0.0,
        dt;
    const Vector x_ref = Vector::Zero(nx);
    const Matrix Q = Matrix::Identity(nx, nx);
    const Matrix R = Matrix::Identity(nu, nu) * 0.1;
    const Vector x_min = Vector::Constant(nx, -1e3);
    const Vector x_max = Vector::Constant(nx, 1e3);
    const Vector u_min = Vector::Constant(nu, -1e3);
    const Vector u_max = Vector::Constant(nu, 1e3);

    MultiStageOCP ocp = makeDoubleIntegratorOCP(N, dt, A, B, x_ref, Q, R,
        x_min, x_max, u_min, u_max);

    Trajectory init_guess = makeZeroTrajectory(N, nx, nu);
    init_guess.x[0] << 1.0, 2.0;
    init_guess.u[0] << 0.1;

    SQPSolverTestAccess solver(std::make_unique<FailingQPSolver>());
    Trajectory solution;
    const bool ok = solver.solve(ocp, init_guess, solution);
    EXPECT_FALSE(ok);

    const Trajectory& current = solver.currentTraj();
    ASSERT_EQ(current.x.size(), init_guess.x.size());
    ASSERT_EQ(current.u.size(), init_guess.u.size());
    for (size_t k = 0; k < current.x.size(); ++k) {
        EXPECT_TRUE(current.x[k].isApprox(init_guess.x[k], 1e-12))
            << "current_traj_ at k=" << k << " contaminated by QP failure path";
    }
    for (size_t k = 0; k < current.u.size(); ++k) {
        EXPECT_TRUE(current.u[k].isApprox(init_guess.u[k], 1e-12))
            << "current_traj_ control at k=" << k << " contaminated by QP failure path";
    }
}

TEST(SQPAlgorithm, DeltaQPSemanticsCorrect)
{
    // 测试目的：验证生产 SQPSolver 把 QP 解解释为 delta，并正确叠加到 current_traj_
    // 流程：构造 BicycleModelKappa 单段 OCP，注入 MockDeltaQPSolver 返回已知 delta，调用 solve
    // 预期效果：current_traj_ = initial ⊕ delta（状态走 retract，控制线性相加）
    const int nx = 5, nu = 2, N = 2;
    MultiStageOCP ocp;
    auto dynamics = std::make_shared<BicycleModelKappa>();
    StageSegment segment;
    segment.dynamics = dynamics;
    segment.N = N;
    segment.dt = 0.1;
    segment.v_sign = 1.0;
    segment.x_min = Vector::Constant(nx, -1e3);
    segment.x_max = Vector::Constant(nx, 1e3);
    segment.u_min = Vector::Constant(nu, -1e3);
    segment.u_max = Vector::Constant(nu, 1e3);
    ocp.addSegment(segment);

    Trajectory init_guess = makeZeroTrajectory(N, nx, nu);
    init_guess.x[0] << 0.0, 0.0, PI - 0.05, 0.5, 0.0;
    init_guess.x[1] << 0.1, 0.2, PI - 0.05, 0.5, 0.0;
    init_guess.x[2] << 0.2, 0.4, PI - 0.05, 0.5, 0.0;
    init_guess.u[0] << 0.0, 0.0;
    init_guess.u[1] << 0.0, 0.0;

    Trajectory delta;
    delta.resize(N, nx, nu);
    delta.x[0].setZero();
    delta.x[0](2) = 0.1;
    delta.x[1].setZero();
    delta.x[1](2) = 0.1;
    delta.x[2].setZero();
    delta.x[2](2) = 0.1;
    delta.u[0] << 0.1, 0.2;
    delta.u[1] << 0.3, 0.4;

    SQPSolverTestAccess solver(std::make_unique<MockDeltaQPSolver>(delta));
    // 使用 RTI 模式：只执行一次迭代，避免第二次迭代把 delta 当零处理；
    // 本测试仅验证 delta/retract 语义，不验证 Full SQP 收敛。
    // 关闭线搜索：MockDeltaQPSolver 返回固定 delta，不保证是实际 merit 下降方向。
    solver.options().use_rti = true;
    solver.options().use_line_search = false;
    Trajectory solution;
    const bool ok = solver.solve(ocp, init_guess, solution);
    EXPECT_TRUE(ok);

    const double expected_theta = math_util::NormalizeAngle(PI - 0.05 + 0.1);
    EXPECT_NEAR(solution.x[0](2), expected_theta, 1e-12);
    EXPECT_NEAR(solution.x[1](2), expected_theta, 1e-12);
    EXPECT_NEAR(solution.x[2](2), expected_theta, 1e-12);
    EXPECT_NEAR(solution.u[0](0), 0.1, 1e-12);
    EXPECT_NEAR(solution.u[0](1), 0.2, 1e-12);
    EXPECT_NEAR(solution.u[1](0), 0.3, 1e-12);
    EXPECT_NEAR(solution.u[1](1), 0.4, 1e-12);
}

TEST(SQPAlgorithm, DirectDenseSolverFindsNonZeroDelta)
{
    // 测试目的：验证用 SQP 装配出的 QPData 能被 DenseQPSolver 正确求解并返回非零 delta
    // 流程：手工构造与 SQP 完全一致的 QPData，调用 DenseQPSolver
    // 预期效果：delta 非零且满足动力学
    const int nx = 2, nu = 1, N = 2;
    const double dt = 0.1;
    Matrix A(nx, nx);
    A << 1.0, dt,
        0.0, 1.0;
    Matrix B(nx, nu);
    B << 0.0,
        dt;
    auto qp = std::make_unique<QPData>(N, nx, nu, 0);
    for (int k = 0; k <= N; ++k) {
        qp->Q[k] = Matrix::Identity(nx, nx);
        qp->lbx[k] = Vector::Constant(nx, -1e3);
        qp->ubx[k] = Vector::Constant(nx, 1e3);
    }
    qp->lbx[0] = Vector::Zero(nx);
    qp->ubx[0] = Vector::Zero(nx);
    qp->q[0] = Vector::Zero(nx);
    qp->q[1] << 1.0, 0.0;
    qp->q[2] << 1.0, 0.0;
    for (int k = 0; k < N; ++k) {
        qp->A[k] = A;
        qp->B[k] = B;
        qp->b[k] = Vector::Zero(nx);
        qp->R[k] = Matrix::Identity(nu, nu) * 0.1;
        qp->S[k] = Matrix::Zero(nu, nx);
        qp->r[k] = Vector::Zero(nu);
        qp->lbu[k] = Vector::Constant(nu, -1e3);
        qp->ubu[k] = Vector::Constant(nu, 1e3);
    }
    DenseQPSolver solver;
    QPSolution sol;
    EXPECT_EQ(solver.solve(*qp, sol), QPSolverStatus::SUCCESS);
    // delta_x0 必须为零
    EXPECT_TRUE(sol.x[0].isZero(1e-12));
    // delta 不应全为零（否则无法解释非零 q）
    bool all_zero = true;
    for (int k = 0; k <= N; ++k) {
        if (sol.x[k].norm() > 1e-9) {
            all_zero = false;
        }
    }
    for (int k = 0; k < N; ++k) {
        if (sol.u[k].norm() > 1e-9) {
            all_zero = false;
        }
    }
    EXPECT_FALSE(all_zero);
    // 检查动力学
    for (int k = 0; k < N; ++k) {
        const Vector dx_next = A * sol.x[k] + B * sol.u[k] + qp->b[k];
        EXPECT_TRUE(sol.x[k + 1].isApprox(dx_next, 1e-9));
    }
}

TEST(SQPAlgorithm, ProductionLQRConvergesAndSatisfiesDynamics)
{
    // 测试目的：验证生产 SQPSolver 在双积分器 LQR 问题上能收敛，且解满足动力学
    // 流程：构造 DoubleIntegrator + QuadraticTrackingCost + 宽松 BoxConstraint，
    //      用 DenseQPSolver 求解
    // 预期效果：SQP 收敛，初始状态保持，轨迹满足 x_{k+1} = A x_k + B u_k，
    //          末端状态比初始状态更接近参考点
    const int nx = 2, nu = 1, N = 20;
    const double dt = 0.1;
    Matrix A(nx, nx);
    A << 1.0, dt,
        0.0, 1.0;
    Matrix B(nx, nu);
    B << 0.0,
        dt;
    const Matrix Q = Matrix::Identity(nx, nx);
    const Matrix R = Matrix::Identity(nu, nu) * 0.1;

    const Vector x_ref = Vector::Zero(nx);
    const Vector x_min = Vector::Constant(nx, -1e3);
    const Vector x_max = Vector::Constant(nx, 1e3);
    const Vector u_min = Vector::Constant(nu, -1e3);
    const Vector u_max = Vector::Constant(nu, 1e3);

    MultiStageOCP ocp = makeDoubleIntegratorOCP(N, dt, A, B, x_ref, Q, R,
        x_min, x_max, u_min, u_max);

    Trajectory init_guess = makeZeroTrajectory(N, nx, nu);
    init_guess.x[0] << 1.0, 0.0;
    // 构造有效动力学 rollout，使 b[k] = 0，便于验证
    for (int k = 0; k < N; ++k) {
        init_guess.x[k + 1] = A * init_guess.x[k] + B * init_guess.u[k];
    }

    SQPSolver solver(std::make_unique<DenseQPSolver>());
    solver.options().stationarity_tol = 1e-10;
    Trajectory solution;
    const bool ok = solver.solve(ocp, init_guess, solution);
    ASSERT_TRUE(ok);

    // 初始状态必须保持（delta_x0 = 0）
    EXPECT_TRUE(solution.x[0].isApprox(init_guess.x[0], 1e-12));

    // 轨迹必须满足离散动力学
    const double dyn_tol = 1e-10;
    for (int k = 0; k < N; ++k) {
        const Vector x_next_expected = A * solution.x[k] + B * solution.u[k];
        const double residual = (solution.x[k + 1] - x_next_expected).norm();
        EXPECT_LT(residual, dyn_tol)
            << "k=" << k << " dynamics residual too large: " << residual
            << " x[k]=" << solution.x[k].transpose()
            << " u[k]=" << solution.u[k].transpose()
            << " x[k+1]=" << solution.x[k + 1].transpose()
            << " expected=" << x_next_expected.transpose();
    }

    // 末端状态应比初始状态更接近参考点
    const double initial_state_norm = init_guess.x[0].norm();
    const double terminal_state_norm = solution.x[N].norm();
    if (terminal_state_norm >= initial_state_norm) {
        for (int k = 0; k <= N; ++k) {
            std::cerr << "x[" << k << "]=" << solution.x[k].transpose() << "\n";
            if (k < N) {
                std::cerr << "u[" << k << "]=" << solution.u[k].transpose() << "\n";
            }
        }
    }
    EXPECT_LT(terminal_state_norm, initial_state_norm);
}

TEST(MultiStageOCP, ValidatesDimensionMismatch)
{
    // 测试目的：验证 MultiStageOCP::validate 能检测各段动力学维度不一致
    // 流程：先添加 nx=2 段，再添加 nx=3 段
    // 预期效果：addSegment 抛出 std::invalid_argument
    MultiStageOCP ocp;
    Matrix A = Matrix::Identity(2, 2);
    Matrix B = Matrix::Zero(2, 1);
    StageSegment seg1;
    seg1.dynamics = std::make_shared<DoubleIntegrator>(A, B);
    seg1.N = 5;
    seg1.dt = 0.1;
    seg1.v_sign = 1.0;
    ocp.addSegment(seg1);

    Matrix A2 = Matrix::Identity(3, 3);
    Matrix B2 = Matrix::Zero(3, 1);
    StageSegment seg2;
    seg2.dynamics = std::make_shared<DoubleIntegrator>(A2, B2);
    seg2.N = 5;
    seg2.dt = 0.1;
    seg2.v_sign = 1.0;
    EXPECT_THROW(ocp.addSegment(seg2), std::invalid_argument);
}

TEST(MultiStageOCP, ValidatesNegativeNAndDt)
{
    // 测试目的：验证 addSegment 对非法 N、dt、v_sign 抛出异常
    MultiStageOCP ocp;
    Matrix A = Matrix::Identity(2, 2);
    Matrix B = Matrix::Zero(2, 1);
    StageSegment segment;
    segment.dynamics = std::make_shared<DoubleIntegrator>(A, B);
    segment.N = -1;
    segment.dt = 0.1;
    segment.v_sign = 1.0;
    EXPECT_THROW(ocp.addSegment(segment), std::invalid_argument);

    segment.N = 5;
    segment.dt = 0.0;
    EXPECT_THROW(ocp.addSegment(segment), std::invalid_argument);

    segment.dt = 0.1;
    segment.v_sign = 0.0;
    EXPECT_THROW(ocp.addSegment(segment), std::invalid_argument);
}

TEST(MultiStageOCP, DetectsGearShift)
{
    // 测试目的：验证 hasGearShift 正确识别相邻段速度方向变号
    MultiStageOCP ocp;
    Matrix A = Matrix::Identity(2, 2);
    Matrix B = Matrix::Zero(2, 1);
    StageSegment seg1;
    seg1.dynamics = std::make_shared<DoubleIntegrator>(A, B);
    seg1.N = 5;
    seg1.dt = 0.1;
    seg1.v_sign = 1.0;
    ocp.addSegment(seg1);
    EXPECT_FALSE(ocp.hasGearShift());

    StageSegment seg2;
    seg2.dynamics = std::make_shared<DoubleIntegrator>(A, B);
    seg2.N = 5;
    seg2.dt = 0.1;
    seg2.v_sign = -1.0;
    ocp.addSegment(seg2);
    EXPECT_TRUE(ocp.hasGearShift());
}

TEST(MultiStageOCP, ValidatesStageParamsConsistency)
{
    // 流程：构造合法段，分别填入长度错误、维度为 0、维度不一致、完全合法的 stage_params，调用 validate
    // 预期效果：非法配置返回 false 且提示包含 stage_params，合法配置返回 true
    Matrix A = Matrix::Identity(2, 2);
    Matrix B = Matrix::Zero(2, 1);

    auto make_ocp_with_params = [&](const std::vector<StageParameters>& params) {
        MultiStageOCP ocp;
        StageSegment segment;
        segment.dynamics = std::make_shared<DoubleIntegrator>(A, B);
        segment.N = 5;
        segment.dt = 0.1;
        segment.v_sign = 1.0;
        segment.x_min = Vector::Constant(2, -1.0);
        segment.x_max = Vector::Constant(2, 1.0);
        segment.u_min = Vector::Constant(1, -1.0);
        segment.u_max = Vector::Constant(1, 1.0);
        segment.stage_params = params;
        ocp.addSegment(segment);
        return ocp;
    };

    // 长度错误
    std::vector<StageParameters> wrong_size(3);
    wrong_size[0].p = Vector::Zero(STAGE_PARAM_DIM);
    MultiStageOCP ocp_wrong = make_ocp_with_params(wrong_size);
    std::string reason;
    EXPECT_FALSE(ocp_wrong.validate(&reason));
    EXPECT_NE(reason.find("stage_params"), std::string::npos);

    // 维度为 0
    std::vector<StageParameters> zero_dim(5);
    MultiStageOCP ocp_zero = make_ocp_with_params(zero_dim);
    EXPECT_FALSE(ocp_zero.validate(&reason));

    // 维度不一致
    std::vector<StageParameters> inconsistent(5);
    for (int i = 0; i < 5; ++i) {
        inconsistent[i].p = Vector::Zero(STAGE_PARAM_DIM);
    }
    inconsistent[2].p = Vector::Zero(10);
    MultiStageOCP ocp_inconsistent = make_ocp_with_params(inconsistent);
    EXPECT_FALSE(ocp_inconsistent.validate(&reason));

    // 合法配置
    std::vector<StageParameters> valid(5);
    for (int i = 0; i < 5; ++i) {
        valid[i].p = Vector::Zero(STAGE_PARAM_DIM);
    }
    MultiStageOCP ocp_valid = make_ocp_with_params(valid);
    EXPECT_TRUE(ocp_valid.validate(&reason));
}

TEST(MultiStageOCP, RejectsInvalidDtArrayElements)
{
    // 测试目的：验证 dt_array 中的非法元素（<=0 / nan / inf）在 addSegment 阶段即被拒绝
    // 流程：分别构造 dt_array 包含负数、零、nan、inf 的段，调用 addSegment
    // 预期效果：所有非法情况均抛出 std::invalid_argument
    Matrix A = Matrix::Identity(2, 2);
    Matrix B = Matrix::Zero(2, 1);

    auto make_segment = [&](const std::vector<double>& dt_array) {
        StageSegment seg;
        seg.dynamics = std::make_shared<DoubleIntegrator>(A, B);
        seg.N = static_cast<int>(dt_array.size());
        seg.dt_array = dt_array;
        seg.v_sign = 1.0;
        seg.x_min = Vector::Constant(2, -1.0);
        seg.x_max = Vector::Constant(2, 1.0);
        seg.u_min = Vector::Constant(1, -1.0);
        seg.u_max = Vector::Constant(1, 1.0);
        return seg;
    };

    MultiStageOCP ocp_neg;
    EXPECT_THROW(ocp_neg.addSegment(make_segment({ 0.1, -0.1 })), std::invalid_argument);

    MultiStageOCP ocp_zero;
    EXPECT_THROW(ocp_zero.addSegment(make_segment({ 0.1, 0.0 })), std::invalid_argument);

    MultiStageOCP ocp_nan;
    EXPECT_THROW(ocp_nan.addSegment(make_segment({ 0.1, std::numeric_limits<double>::quiet_NaN() })),
        std::invalid_argument);

    MultiStageOCP ocp_inf;
    EXPECT_THROW(ocp_inf.addSegment(make_segment({ 0.1, std::numeric_limits<double>::infinity() })),
        std::invalid_argument);
}

TEST(SQPAlgorithm, ControlConstraintCuIsEnforced)
{
    // 测试目的：验证一般约束中的控制 Jacobian Cu 被正确装配到 QPData::D 并参与求解
    // 流程：在双积分器 LQR 中加入一般控制约束 u(0) <= -0.5，使用 DenseQPSolver
    // 预期效果：SQP 收敛，且所有时刻控制满足 u(0) <= -0.5
    const int nx = 2, nu = 1, N = 20;
    const double dt = 0.1;
    Matrix A(nx, nx);
    A << 1.0, dt,
        0.0, 1.0;
    Matrix B(nx, nu);
    B << 0.0,
        dt;
    const Matrix Q = Matrix::Identity(nx, nx);
    const Matrix R = Matrix::Identity(nu, nu) * 0.1;

    const Vector x_ref = Vector::Zero(nx);
    const Vector x_min = Vector::Constant(nx, -1e3);
    const Vector x_max = Vector::Constant(nx, 1e3);
    const Vector u_min = Vector::Constant(nu, -1e3);
    const Vector u_max = Vector::Constant(nu, 1e3);

    StageSegment segment = makeDoubleIntegratorSegment(N, dt, A, B, x_ref, Q, R,
        x_min, x_max, u_min, u_max);
    // 加入一般控制约束 u(0) <= -0.5，依赖 Cu 而非 box bound
    segment.constraints.push_back(
        std::make_shared<ControlUpperBoundConstraint>(0, -0.5));
    MultiStageOCP ocp;
    ocp.addSegment(segment);

    Trajectory init_guess = makeZeroTrajectory(N, nx, nu);
    init_guess.x[0] << 1.0, 0.0;
    for (int k = 0; k < N; ++k) {
        init_guess.x[k + 1] = A * init_guess.x[k] + B * init_guess.u[k];
    }

    SQPSolver solver(std::make_unique<DenseQPSolver>());
    solver.options().stationarity_tol = 1e-8;
    solver.options().kkt_tol = 1e-8;
    solver.options().constr_viol_tol = 1e-8;
    Trajectory solution;
    const bool ok = solver.solve(ocp, init_guess, solution);
    ASSERT_TRUE(ok);
    EXPECT_TRUE(solver.converged());

    for (int k = 0; k < N; ++k) {
        EXPECT_LE(solution.u[k](0), -0.5 + 1e-6)
            << "k=" << k << " control constraint u <= -0.5 violated";
    }

    // 初始状态保持
    EXPECT_TRUE(solution.x[0].isApprox(init_guess.x[0], 1e-12));
}

TEST(SQPAlgorithm, VSignIsPassedToDynamicsLinearization)
{
    // 测试目的：验证 SQP linearize() 把 segment.v_sign 正确传给动力学模型
    // 流程：构造一个 SignRecordingDynamics 段，设 v_sign=-1.0，调用 solve
    // 预期效果：动力学至少收到一次 v_sign=-1.0
    const int nx = 2, nu = 1, N = 3;
    auto dynamics = std::make_shared<SignRecordingDynamics>(nx, nu);

    MultiStageOCP ocp;
    StageSegment segment;
    segment.dynamics = dynamics;
    segment.cost = std::make_shared<QuadraticTrackingCost>(
        Vector::Zero(nx), Matrix::Identity(nx, nx), Matrix::Identity(nu, nu));
    segment.N = N;
    segment.dt = 0.1;
    segment.v_sign = -1.0;
    segment.x_min = Vector::Constant(nx, -1e3);
    segment.x_max = Vector::Constant(nx, 1e3);
    segment.u_min = Vector::Constant(nu, -1e3);
    segment.u_max = Vector::Constant(nu, 1e3);
    ocp.addSegment(segment);

    Trajectory init_guess = makeZeroTrajectory(N, nx, nu);
    SQPSolver solver(std::make_unique<DenseQPSolver>());
    Trajectory solution;
    EXPECT_TRUE(solver.solve(ocp, init_guess, solution));
    EXPECT_DOUBLE_EQ(dynamics->receivedVSign(), -1.0);
}

TEST(MultiStageOCP, RejectsInvalidUniformDt)
{
    // 测试目的：验证均匀 dt（dt_array 为空）时，NaN/Inf/非正数在 addSegment 阶段即被拒绝
    // 流程：构造 dt_array 为空但 dt 分别为 nan、+inf、-inf、0 的段，调用 addSegment
    // 预期效果：所有非法情况均抛出 std::invalid_argument
    StageSegment seg;
    seg.dynamics = std::make_shared<DoubleIntegrator>(
        Matrix::Identity(2, 2), Matrix::Zero(2, 1));
    seg.N = 2;
    seg.dt_array.clear();
    seg.v_sign = 1.0;
    seg.x_min = Vector::Constant(2, -1.0);
    seg.x_max = Vector::Constant(2, 1.0);
    seg.u_min = Vector::Constant(1, -1.0);
    seg.u_max = Vector::Constant(1, 1.0);

    MultiStageOCP ocp_nan;
    seg.dt = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(ocp_nan.addSegment(seg), std::invalid_argument);

    MultiStageOCP ocp_inf;
    seg.dt = std::numeric_limits<double>::infinity();
    EXPECT_THROW(ocp_inf.addSegment(seg), std::invalid_argument);

    MultiStageOCP ocp_neg_inf;
    seg.dt = -std::numeric_limits<double>::infinity();
    EXPECT_THROW(ocp_neg_inf.addSegment(seg), std::invalid_argument);

    MultiStageOCP ocp_zero;
    seg.dt = 0.0;
    EXPECT_THROW(ocp_zero.addSegment(seg), std::invalid_argument);
}

// ===================== 测试辅助：返回非法输出的 mock 动力学 =====================
class BadOutputDynamics : public DynamicalSystem {
public:
    enum class Mode {
        kWrongSizeXNext,
        kWrongSizeA,
        kWrongSizeB,
        kNaN,
    };
    BadOutputDynamics(int nx, int nu, Mode mode)
        : nx_(nx)
        , nu_(nu)
        , mode_(mode)
    {
    }
    int nx() const override { return nx_; }
    int nu() const override { return nu_; }
    void evaluate(const Vector& x, const Vector& u, Vector& x_dot) const override
    {
        (void)x;
        (void)u;
        x_dot.setZero(nx_);
    }
    void discretizeAndLinearize(const Vector& x, const Vector& u, double dt,
        double v_sign, Vector& x_next, Matrix& A, Matrix& B) const override
    {
        (void)x;
        (void)u;
        (void)dt;
        (void)v_sign;
        switch (mode_) {
        case Mode::kWrongSizeXNext:
            x_next = Vector::Zero(nx_ + 1);
            A = Matrix::Identity(nx_, nx_);
            B = Matrix::Zero(nx_, nu_);
            break;
        case Mode::kWrongSizeA:
            x_next = Vector::Zero(nx_);
            A = Matrix::Zero(nx_ + 1, nx_);
            B = Matrix::Zero(nx_, nu_);
            break;
        case Mode::kWrongSizeB:
            x_next = Vector::Zero(nx_);
            A = Matrix::Identity(nx_, nx_);
            B = Matrix::Zero(nx_, nu_ + 1);
            break;
        case Mode::kNaN:
            x_next = Vector::Constant(nx_, std::numeric_limits<double>::quiet_NaN());
            A = Matrix::Identity(nx_, nx_);
            B = Matrix::Zero(nx_, nu_);
            break;
        }
    }

private:
    int nx_ = 0;
    int nu_ = 0;
    Mode mode_ = Mode::kNaN;
};

TEST(SQPAlgorithm, RejectsInvalidDynamicsLinearizationOutput)
{
    // 测试目的：验证 SQP linearize() 对动力学返回的错维度或 NaN 输出会拒绝而不是崩溃
    // 流程：用 BadOutputDynamics 分别构造 x_next 维度错误、A 维度错误、B 维度错误、NaN 四种 OCP，
    //      调用 solve
    // 预期效果：所有情况 solve 均返回 false
    const int nx = 2, nu = 1, N = 3;
    auto make_ocp = [&](BadOutputDynamics::Mode mode) {
        MultiStageOCP ocp;
        StageSegment segment;
        segment.dynamics = std::make_shared<BadOutputDynamics>(nx, nu, mode);
        segment.cost = std::make_shared<QuadraticTrackingCost>(
            Vector::Zero(nx), Matrix::Identity(nx, nx), Matrix::Identity(nu, nu));
        segment.N = N;
        segment.dt = 0.1;
        segment.v_sign = 1.0;
        segment.x_min = Vector::Constant(nx, -1e3);
        segment.x_max = Vector::Constant(nx, 1e3);
        segment.u_min = Vector::Constant(nu, -1e3);
        segment.u_max = Vector::Constant(nu, 1e3);
        ocp.addSegment(segment);
        return ocp;
    };

    Trajectory init_guess = makeZeroTrajectory(N, nx, nu);
    for (const auto mode : { BadOutputDynamics::Mode::kWrongSizeXNext,
             BadOutputDynamics::Mode::kWrongSizeA,
             BadOutputDynamics::Mode::kWrongSizeB,
             BadOutputDynamics::Mode::kNaN }) {
        const auto ocp = make_ocp(mode);
        SQPSolver solver(std::make_unique<DenseQPSolver>());
        Trajectory solution;
        EXPECT_FALSE(solver.solve(ocp, init_guess, solution))
            << "mode " << static_cast<int>(mode) << " should be rejected by linearize()";
    }
}

// ===================== 测试：初始猜测含非有限值 =====================
TEST(SQPAlgorithm, RejectsInvalidInitialGuessFiniteValues)
{
    // 测试目的：验证 SQP 在初始猜测含 NaN 或 inf 时不会继续求解而是返回 false
    // 流程：构造合法 OCP，但初始猜测分别写入 NaN 和 inf，调用 solve
    // 预期效果：两种情况 solve 均返回 false
    const int nx = 2, nu = 1, N = 3;
    MultiStageOCP ocp;
    StageSegment segment;
    segment.dynamics = std::make_shared<DoubleIntegrator>(
        Matrix::Identity(nx, nx), Matrix::Zero(nx, nu));
    segment.cost = std::make_shared<QuadraticTrackingCost>(
        Vector::Zero(nx), Matrix::Identity(nx, nx), Matrix::Identity(nu, nu));
    segment.N = N;
    segment.dt = 0.1;
    segment.v_sign = 1.0;
    segment.x_min = Vector::Constant(nx, -1e3);
    segment.x_max = Vector::Constant(nx, 1e3);
    segment.u_min = Vector::Constant(nu, -1e3);
    segment.u_max = Vector::Constant(nu, 1e3);
    ocp.addSegment(segment);

    auto make_bad_init = [&](double bad_value) {
        Trajectory init = makeZeroTrajectory(N, nx, nu);
        init.x[1](0) = bad_value;
        return init;
    };

    for (const double bad : { std::numeric_limits<double>::quiet_NaN(),
             std::numeric_limits<double>::infinity() }) {
        SQPSolver solver(std::make_unique<DenseQPSolver>());
        Trajectory solution;
        EXPECT_FALSE(solver.solve(ocp, make_bad_init(bad), solution))
            << "Should return false when initial guess contains non-finite values";
    }
}

// ===================== 测试辅助：收敛检查中返回非法输出的动力学 =====================
class BadAtConvergenceDynamics : public DynamicalSystem {
public:
    BadAtConvergenceDynamics(int nx, int nu)
        : nx_(nx)
        , nu_(nu)
    {
    }
    int nx() const override { return nx_; }
    int nu() const override { return nu_; }
    void evaluate(const Vector& x, const Vector& u, Vector& x_dot) const override
    {
        (void)x;
        (void)u;
        x_dot.setZero(nx_);
    }
    void discretizeAndLinearize(const Vector& x, const Vector& u, double dt,
        double v_sign, Vector& x_next, Matrix& A, Matrix& B) const override
    {
        (void)x;
        (void)u;
        (void)dt;
        (void)v_sign;
        // 第一次调用（linearize）返回合法；后续调用（checkConvergence）返回 NaN
        if (call_count_++ == 0) {
            x_next = Vector::Zero(nx_);
            A = Matrix::Identity(nx_, nx_);
            B = Matrix::Zero(nx_, nu_);
        } else {
            x_next = Vector::Constant(nx_, std::numeric_limits<double>::quiet_NaN());
            A = Matrix::Identity(nx_, nx_);
            B = Matrix::Zero(nx_, nu_);
        }
    }

private:
    int nx_ = 0;
    int nu_ = 0;
    mutable int call_count_ = 0;
};

TEST(SQPAlgorithm, RejectsInvalidDynamicsInConvergenceCheck)
{
    // 测试目的：验证 checkConvergence() 重新评估动力学时若收到非法输出，会安全返回 false
    // 流程：构造使第一次 linearize 合法、第二次 checkConvergence 非法的动力学，
    //      触发一次 Full SQP 迭代后的收敛检查
    // 预期效果：solve 返回 false，且不崩溃
    const int nx = 2, nu = 1, N = 3;
    MultiStageOCP ocp;
    StageSegment segment;
    segment.dynamics = std::make_shared<BadAtConvergenceDynamics>(nx, nu);
    segment.cost = std::make_shared<QuadraticTrackingCost>(
        Vector::Zero(nx), Matrix::Identity(nx, nx), Matrix::Identity(nu, nu));
    segment.N = N;
    segment.dt = 0.1;
    segment.v_sign = 1.0;
    segment.x_min = Vector::Constant(nx, -1e3);
    segment.x_max = Vector::Constant(nx, 1e3);
    segment.u_min = Vector::Constant(nu, -1e3);
    segment.u_max = Vector::Constant(nu, 1e3);
    ocp.addSegment(segment);

    SQPSolver solver(std::make_unique<DenseQPSolver>());
    solver.options().max_iter = 1;      // 保证至少触发一次收敛检查
    solver.options().use_line_search = false;
    Trajectory init_guess = makeZeroTrajectory(N, nx, nu);
    Trajectory solution;
    EXPECT_FALSE(solver.solve(ocp, init_guess, solution))
        << "Should return false when dynamics returns NaN during convergence check";
}

// ===================== 测试：基础 Armijo 线搜索在 delta 非下降时回退步长 =====================
TEST(SQPAlgorithm, LineSearchRejectsWorseningDeltaInRTI)
{
    // 测试目的：验证 RTI 模式下，当 QP 返回的 delta 不是 merit 下降方向且
    //      回退到最小步长仍使 merit 恶化时，lineSearch 返回 false 且不应用 delta。
    // 流程：构造简单 LQR，注入 Mock QP 求解器返回会显著增大代价的 delta；
    //      在 RTI + 线搜索开启模式下调用 solve。
    // 预期效果：solve 返回 false，current_traj_ 保持为初始猜测。
    const int nx = 2, nu = 1, N = 2;
    MultiStageOCP ocp;
    StageSegment segment;
    // B=0：控制不影响状态，避免真实 QP 解与 mock delta 冲突
    segment.dynamics = std::make_shared<DoubleIntegrator>(
        Matrix::Identity(nx, nx), Matrix::Zero(nx, nu));
    segment.cost = std::make_shared<QuadraticTrackingCost>(
        Vector::Zero(nx), Matrix::Identity(nx, nx), Matrix::Identity(nu, nu));
    segment.N = N;
    segment.dt = 0.1;
    segment.v_sign = 1.0;
    segment.x_min = Vector::Constant(nx, -1e3);
    segment.x_max = Vector::Constant(nx, 1e3);
    segment.u_min = Vector::Constant(nu, -1e3);
    segment.u_max = Vector::Constant(nu, 1e3);
    ocp.addSegment(segment);

    Trajectory init_guess = makeZeroTrajectory(N, nx, nu);
    init_guess.x[0] << 1.0, 0.0;
    init_guess.x[1] << 1.0, 0.0;
    init_guess.x[2] << 1.0, 0.0;

    // 构造一个会显著增大代价的 delta：x 方向 +10
    Trajectory bad_delta;
    bad_delta.resize(N, nx, nu);
    for (int k = 0; k <= N; ++k) {
        bad_delta.x[k] << 10.0, 0.0;
    }
    for (int k = 0; k < N; ++k) {
        bad_delta.u[k] << 0.0;
    }

    SQPSolverTestAccess solver(std::make_unique<MockDeltaQPSolver>(bad_delta));
    solver.options().use_rti = true;
    solver.options().use_line_search = true;
    solver.options().line_search_alpha_min = 1e-4;

    Trajectory solution;
    EXPECT_FALSE(solver.solve(ocp, init_guess, solution))
        << "RTI mode non-descent direction should be safely rejected by line search";

    // delta 不应被应用到 current_traj_
    const Trajectory& current = solver.currentTraj();
    for (int k = 0; k <= N; ++k) {
        EXPECT_TRUE(current.x[k].isApprox(init_guess.x[k], 1e-12))
            << "RTI line search failure: current_traj_ at k=" << k << " was incorrectly updated";
    }
}

// ===================== 测试：多段相邻段状态边界交集为空时 validate 拒绝 =====================
TEST(MultiStageOCP, RejectsEmptyBoundaryStateBoundIntersection)
{
    // 测试目的：验证 MultiStageOCP::validate 在相邻段边界状态 box bound 无交集时返回 false
    // 流程：构造两段 OCP，第一段 x_max(0)=0.0，第二段 x_min(0)=0.1，调用 validate
    // 预期效果：validate 返回 false，错误信息包含“无交集”
    MultiStageOCP ocp;
    Matrix A = Matrix::Identity(2, 2);
    Matrix B = Matrix::Zero(2, 1);

    StageSegment seg1;
    seg1.dynamics = std::make_shared<DoubleIntegrator>(A, B);
    seg1.N = 5;
    seg1.dt = 0.1;
    seg1.v_sign = 1.0;
    seg1.x_min = Vector::Constant(2, -1.0);
    seg1.x_max = Vector::Constant(2, 1.0);
    seg1.x_max(0) = 0.0;
    seg1.u_min = Vector::Constant(1, -1.0);
    seg1.u_max = Vector::Constant(1, 1.0);
    ocp.addSegment(seg1);

    StageSegment seg2;
    seg2.dynamics = std::make_shared<DoubleIntegrator>(A, B);
    seg2.N = 5;
    seg2.dt = 0.1;
    seg2.v_sign = 1.0;
    seg2.x_min = Vector::Constant(2, -1.0);
    seg2.x_min(0) = 0.1;
    seg2.x_max = Vector::Constant(2, 1.0);
    seg2.u_min = Vector::Constant(1, -1.0);
    seg2.u_max = Vector::Constant(1, 1.0);
    ocp.addSegment(seg2);

    std::string reason;
    EXPECT_FALSE(ocp.validate(&reason));
    EXPECT_NE(reason.find("intersection"), std::string::npos);
}

// ===================== 测试：QP 装配对段间边界状态 bound 取交集 =====================
TEST(SQPAlgorithm, RespectsBoundaryStateBoundIntersection)
{
    // 测试目的：验证 assembleQP 对段间边界状态 bound 取交集，而非只使用后一段 bound
    // 流程：构造一维双段 OCP（x_next = x + dt * u），第一段 x_max=0.5，第二段 x_max=1.0；
    //      初始猜测在段边界处 x=0.6，调用 solve
    // 预期效果：边界状态被压回 <= 0.5，即交集上界生效
    const int nx = 1, nu = 1;
    const int N1 = 2, N2 = 2;
    const double dt = 0.1;
    Matrix A(1, 1);
    A << 1.0;
    Matrix B(1, 1);
    B << dt;

    MultiStageOCP ocp;
    StageSegment seg1;
    seg1.dynamics = std::make_shared<DoubleIntegrator>(A, B);
    seg1.cost = std::make_shared<QuadraticTrackingCost>(
        Vector::Zero(nx), Matrix::Identity(nx, nx), Matrix::Identity(nu, nu));
    seg1.N = N1;
    seg1.dt = dt;
    seg1.v_sign = 1.0;
    seg1.x_min = Vector::Constant(nx, -1e3);
    seg1.x_max = Vector::Constant(nx, 0.5);
    seg1.u_min = Vector::Constant(nu, -1e3);
    seg1.u_max = Vector::Constant(nu, 1e3);
    ocp.addSegment(seg1);

    StageSegment seg2;
    seg2.dynamics = std::make_shared<DoubleIntegrator>(A, B);
    seg2.cost = std::make_shared<QuadraticTrackingCost>(
        Vector::Zero(nx), Matrix::Identity(nx, nx), Matrix::Identity(nu, nu));
    seg2.N = N2;
    seg2.dt = dt;
    seg2.v_sign = 1.0;
    seg2.x_min = Vector::Constant(nx, -1e3);
    seg2.x_max = Vector::Constant(nx, 1.0);
    seg2.u_min = Vector::Constant(nu, -1e3);
    seg2.u_max = Vector::Constant(nu, 1e3);
    ocp.addSegment(seg2);

    Trajectory init_guess = makeZeroTrajectory(N1 + N2, nx, nu);
    init_guess.x[0](0) = 0.4;
    init_guess.x[N1](0) = 0.6; // 段边界：超出第一段上界 0.5
    init_guess.x[N1 + N2](0) = 0.6;

    SQPSolver solver(std::make_unique<DenseQPSolver>());
    solver.options().constr_viol_tol = 1e-8;
    Trajectory solution;
    ASSERT_TRUE(solver.solve(ocp, init_guess, solution));

    // 段边界状态必须被压到交集上界 0.5 以内
    EXPECT_LE(solution.x[N1](0), 0.5 + 1e-6)
        << "Inter-segment boundary state bound intersection not applied, still exceeds first segment upper bound";
}

// ===================== 测试辅助：回调抛异常的动力学 =====================
class ThrowingDynamics : public DynamicalSystem {
public:
    ThrowingDynamics(int nx, int nu)
        : nx_(nx)
        , nu_(nu)
    {
    }
    int nx() const override { return nx_; }
    int nu() const override { return nu_; }
    void evaluate(const Vector& x, const Vector& u, Vector& x_dot) const override
    {
        (void)x;
        (void)u;
        x_dot.setZero(nx_);
    }
    void discretizeAndLinearize(const Vector& x, const Vector& u, double dt,
        double v_sign, Vector& x_next, Matrix& A, Matrix& B) const override
    {
        (void)x;
        (void)u;
        (void)dt;
        (void)v_sign;
        (void)x_next;
        (void)A;
        (void)B;
        throw std::runtime_error("ThrowingDynamics: deliberate exception");
    }

private:
    int nx_ = 0;
    int nu_ = 0;
};

TEST(SQPAlgorithm, RejectsThrowingDynamicsInLinearization)
{
    // 测试目的：验证 linearize() 捕获动力学回调异常并安全返回 false
    // 流程：用 ThrowingDynamics 构造 OCP，调用 solve
    // 预期效果：solve 返回 false，不崩溃
    const int nx = 2, nu = 1, N = 3;
    MultiStageOCP ocp;
    StageSegment segment;
    segment.dynamics = std::make_shared<ThrowingDynamics>(nx, nu);
    segment.cost = std::make_shared<QuadraticTrackingCost>(
        Vector::Zero(nx), Matrix::Identity(nx, nx), Matrix::Identity(nu, nu));
    segment.N = N;
    segment.dt = 0.1;
    segment.v_sign = 1.0;
    segment.x_min = Vector::Constant(nx, -1e3);
    segment.x_max = Vector::Constant(nx, 1e3);
    segment.u_min = Vector::Constant(nu, -1e3);
    segment.u_max = Vector::Constant(nu, 1e3);
    ocp.addSegment(segment);

    SQPSolver solver(std::make_unique<DenseQPSolver>());
    Trajectory init_guess = makeZeroTrajectory(N, nx, nu);
    Trajectory solution;
    EXPECT_FALSE(solver.solve(ocp, init_guess, solution))
        << "Should return false when dynamics callback throws";
}

// ===================== 测试辅助：回调抛异常的代价 =====================
class ThrowingCost : public QuadraticTrackingCost {
public:
    ThrowingCost(const Vector& x_ref, const Matrix& Q, const Matrix& R)
        : QuadraticTrackingCost(x_ref, Q, R)
    {
    }
    void gradient(const Vector& x, const Vector& u, Vector& q, Vector& r) const override
    {
        (void)x;
        (void)u;
        (void)q;
        (void)r;
        throw std::runtime_error("ThrowingCost: deliberate exception");
    }
};

TEST(SQPAlgorithm, RejectsThrowingCostInAssembleCost)
{
    // 测试目的：验证 assembleCost() 捕获代价回调异常并安全返回 false
    // 流程：用 ThrowingCost 构造 OCP，调用 solve
    // 预期效果：solve 返回 false
    const int nx = 2, nu = 1, N = 3;
    MultiStageOCP ocp;
    StageSegment segment;
    segment.dynamics = std::make_shared<DoubleIntegrator>(
        Matrix::Identity(nx, nx), Matrix::Zero(nx, nu));
    segment.cost = std::make_shared<ThrowingCost>(
        Vector::Zero(nx), Matrix::Identity(nx, nx), Matrix::Identity(nu, nu));
    segment.N = N;
    segment.dt = 0.1;
    segment.v_sign = 1.0;
    segment.x_min = Vector::Constant(nx, -1e3);
    segment.x_max = Vector::Constant(nx, 1e3);
    segment.u_min = Vector::Constant(nu, -1e3);
    segment.u_max = Vector::Constant(nu, 1e3);
    ocp.addSegment(segment);

    SQPSolver solver(std::make_unique<DenseQPSolver>());
    Trajectory init_guess = makeZeroTrajectory(N, nx, nu);
    Trajectory solution;
    EXPECT_FALSE(solver.solve(ocp, init_guess, solution))
        << "Should return false when cost callback throws";
}

// ===================== 测试辅助：回调抛异常的约束 =====================
class ThrowingConstraint : public Constraint {
public:
    int ng() const override { return 1; }
    void evaluate(const Vector& x, const Vector& u, const Vector& p, Vector& g) const override
    {
        (void)x;
        (void)u;
        (void)p;
        g.resize(1);
        g(0) = -1.0;
    }
    void jacobian(const Vector& x, const Vector& u, const Vector& p, Matrix& Cx,
        Matrix& Cu) const override
    {
        (void)x;
        (void)u;
        (void)p;
        (void)Cx;
        (void)Cu;
        throw std::runtime_error("ThrowingConstraint: deliberate exception");
    }
    std::shared_ptr<Constraint> clone() const override
    {
        return std::make_shared<ThrowingConstraint>();
    }
};

TEST(SQPAlgorithm, RejectsThrowingConstraintInAssembleConstraints)
{
    // 测试目的：验证 assembleGeneralConstraints() 捕获约束回调异常并安全返回 false
    // 流程：在 OCP 中加入 ThrowingConstraint，调用 solve
    // 预期效果：solve 返回 false
    const int nx = 2, nu = 1, N = 3;
    MultiStageOCP ocp;
    StageSegment segment;
    segment.dynamics = std::make_shared<DoubleIntegrator>(
        Matrix::Identity(nx, nx), Matrix::Zero(nx, nu));
    segment.cost = std::make_shared<QuadraticTrackingCost>(
        Vector::Zero(nx), Matrix::Identity(nx, nx), Matrix::Identity(nu, nu));
    segment.constraints.push_back(std::make_shared<ThrowingConstraint>());
    segment.N = N;
    segment.dt = 0.1;
    segment.v_sign = 1.0;
    segment.x_min = Vector::Constant(nx, -1e3);
    segment.x_max = Vector::Constant(nx, 1e3);
    segment.u_min = Vector::Constant(nu, -1e3);
    segment.u_max = Vector::Constant(nu, 1e3);
    ocp.addSegment(segment);

    SQPSolver solver(std::make_unique<DenseQPSolver>());
    Trajectory init_guess = makeZeroTrajectory(N, nx, nu);
    Trajectory solution;
    EXPECT_FALSE(solver.solve(ocp, init_guess, solution))
        << "Should return false when constraint callback throws";
}

// ===================== 测试辅助：返回 NaN 标量代价的代价 =====================
class NaNCost : public QuadraticTrackingCost {
public:
    NaNCost(const Vector& x_ref, const Matrix& Q, const Matrix& R)
        : QuadraticTrackingCost(x_ref, Q, R)
    {
    }
    void evaluate(const Vector& x, const Vector& u, double& cost) const override
    {
        (void)x;
        (void)u;
        cost = std::numeric_limits<double>::quiet_NaN();
    }
};

TEST(SQPAlgorithm, RejectsNaNCostInMerit)
{
    // 测试目的：验证 computeMerit() 遇到非有限代价时返回 +inf，导致线搜索失败、solve 返回 false
    // 流程：用 NaNCost 构造 OCP，保证 QP 装配合法但 merit 非有限，调用 solve
    // 预期效果：solve 返回 false
    const int nx = 2, nu = 1, N = 3;
    MultiStageOCP ocp;
    StageSegment segment;
    segment.dynamics = std::make_shared<DoubleIntegrator>(
        Matrix::Identity(nx, nx), Matrix::Zero(nx, nu));
    segment.cost = std::make_shared<NaNCost>(
        Vector::Zero(nx), Matrix::Identity(nx, nx), Matrix::Identity(nu, nu));
    segment.N = N;
    segment.dt = 0.1;
    segment.v_sign = 1.0;
    segment.x_min = Vector::Constant(nx, -1e3);
    segment.x_max = Vector::Constant(nx, 1e3);
    segment.u_min = Vector::Constant(nu, -1e3);
    segment.u_max = Vector::Constant(nu, 1e3);
    ocp.addSegment(segment);

    SQPSolver solver(std::make_unique<DenseQPSolver>());
    Trajectory init_guess = makeZeroTrajectory(N, nx, nu);
    Trajectory solution;
    EXPECT_FALSE(solver.solve(ocp, init_guess, solution))
        << "Should return false when merit function contains NaN";
}

// ===================== 测试：Full SQP 对非下降方向严格拒绝 =====================
TEST(SQPAlgorithm, LineSearchRejectsNonDescentDeltaInFullSQP)
{
    // 测试目的：验证 Full SQP 模式下，当 QP 返回的 delta 不是 merit 下降方向且
    //          回退到最小步长仍不满足 Armijo 时，iterate() 返回 false 且不应用 delta
    // 流程：构造简单 LQR，注入 MockDeltaQPSolver 返回会显著增大代价的 delta，
    //      关闭 RTI，调用 solve
    // 预期效果：solve 返回 false，current_traj_ 保持为初始猜测
    const int nx = 2, nu = 1, N = 2;
    MultiStageOCP ocp;
    StageSegment segment;
    segment.dynamics = std::make_shared<DoubleIntegrator>(
        Matrix::Identity(nx, nx), Matrix::Zero(nx, nu));
    segment.cost = std::make_shared<QuadraticTrackingCost>(
        Vector::Zero(nx), Matrix::Identity(nx, nx), Matrix::Identity(nu, nu));
    segment.N = N;
    segment.dt = 0.1;
    segment.v_sign = 1.0;
    segment.x_min = Vector::Constant(nx, -1e3);
    segment.x_max = Vector::Constant(nx, 1e3);
    segment.u_min = Vector::Constant(nu, -1e3);
    segment.u_max = Vector::Constant(nu, 1e3);
    ocp.addSegment(segment);

    Trajectory init_guess = makeZeroTrajectory(N, nx, nu);
    init_guess.x[0] << 1.0, 0.0;
    init_guess.x[1] << 1.0, 0.0;
    init_guess.x[2] << 1.0, 0.0;

    Trajectory bad_delta;
    bad_delta.resize(N, nx, nu);
    for (int k = 0; k <= N; ++k) {
        bad_delta.x[k] << 10.0, 0.0;
    }
    for (int k = 0; k < N; ++k) {
        bad_delta.u[k] << 0.0;
    }

    SQPSolverTestAccess solver(std::make_unique<MockDeltaQPSolver>(bad_delta));
    solver.options().use_rti = false; // Full SQP
    solver.options().use_line_search = true;
    solver.options().line_search_alpha_min = 1e-4;

    Trajectory solution;
    EXPECT_FALSE(solver.solve(ocp, init_guess, solution))
        << "Full SQP non-descent direction should be rejected by line search";

    // delta 不应被应用到 current_traj_
    const Trajectory& current = solver.currentTraj();
    for (int k = 0; k <= N; ++k) {
        EXPECT_TRUE(current.x[k].isApprox(init_guess.x[k], 1e-12))
            << "Full SQP line search failure: current_traj_ at k=" << k << " was incorrectly updated";
    }
}

// ===================== 测试：关闭线搜索时 Full SQP 接受非下降方向 =====================
TEST(SQPAlgorithm, FullSQPAcceptsNonDescentDeltaWhenLineSearchDisabled)
{
    // 测试目的：验证关闭线搜索后，即使 QP 返回的 delta 不是 merit 下降方向，
    //          Full SQP 也会接受并应用该步，solve() 返回 true。
    // 流程：复用 LineSearchRejectsNonDescentDeltaInFullSQP 的 OCP 与 bad_delta，
    //      仅将 use_line_search 设为 false，调用 solve。
    // 预期效果：solve 返回 true；current_traj_ 已被 bad_delta 更新（x[0..N] 增加了 10.0）。
    const int nx = 2, nu = 1, N = 2;
    MultiStageOCP ocp;
    StageSegment segment;
    segment.dynamics = std::make_shared<DoubleIntegrator>(
        Matrix::Identity(nx, nx), Matrix::Zero(nx, nu));
    segment.cost = std::make_shared<QuadraticTrackingCost>(
        Vector::Zero(nx), Matrix::Identity(nx, nx), Matrix::Identity(nu, nu));
    segment.N = N;
    segment.dt = 0.1;
    segment.v_sign = 1.0;
    segment.x_min = Vector::Constant(nx, -1e3);
    segment.x_max = Vector::Constant(nx, 1e3);
    segment.u_min = Vector::Constant(nu, -1e3);
    segment.u_max = Vector::Constant(nu, 1e3);
    ocp.addSegment(segment);

    Trajectory init_guess = makeZeroTrajectory(N, nx, nu);
    init_guess.x[0] << 1.0, 0.0;
    init_guess.x[1] << 1.0, 0.0;
    init_guess.x[2] << 1.0, 0.0;

    Trajectory bad_delta;
    bad_delta.resize(N, nx, nu);
    for (int k = 0; k <= N; ++k) {
        bad_delta.x[k] << 10.0, 0.0;
    }
    for (int k = 0; k < N; ++k) {
        bad_delta.u[k] << 0.0;
    }

    SQPSolverTestAccess solver(std::make_unique<MockDeltaQPSolver>(bad_delta));
    solver.options().use_rti = false;
    solver.options().use_line_search = false;
    solver.options().max_iter = 1;
    solver.options().kkt_tol = 1e6;
    solver.options().constr_viol_tol = 1e6;
    solver.options().stationarity_tol = 1e6;

    Trajectory solution;
    EXPECT_TRUE(solver.solve(ocp, init_guess, solution))
        << "Full SQP with line search disabled should accept non-descent delta";

    const Trajectory& current = solver.currentTraj();
    for (int k = 0; k <= N; ++k) {
        EXPECT_NEAR(current.x[k](0), init_guess.x[k](0) + bad_delta.x[k](0), 1e-12)
            << "Full SQP without line search did not apply bad delta at k=" << k;
    }
}

// ===================== 测试：非法 solver option 被校验拒绝 =====================
TEST(SQPAlgorithm, RejectsInvalidSolverOptions)
{
    // 测试目的：验证 validateProblem() 对核心收敛阈值与线搜索参数的合法性校验
    // 流程：构造合法 OCP，分别设置非法的 kkt_tol、constr_viol_tol、stationarity_tol、
    //      reg_min、line_search_rho、line_search_c、line_search_alpha_min、merit_penalty，
    //      调用 solve。
    // 预期效果：所有非法配置均返回 false
    const int nx = 2, nu = 1, N = 2;
    MultiStageOCP ocp;
    StageSegment segment;
    segment.dynamics = std::make_shared<DoubleIntegrator>(
        Matrix::Identity(nx, nx), Matrix::Zero(nx, nu));
    segment.cost = std::make_shared<QuadraticTrackingCost>(
        Vector::Zero(nx), Matrix::Identity(nx, nx), Matrix::Identity(nu, nu));
    segment.N = N;
    segment.dt = 0.1;
    segment.v_sign = 1.0;
    segment.x_min = Vector::Constant(nx, -1e3);
    segment.x_max = Vector::Constant(nx, 1e3);
    segment.u_min = Vector::Constant(nu, -1e3);
    segment.u_max = Vector::Constant(nu, 1e3);
    ocp.addSegment(segment);
    Trajectory init_guess = makeZeroTrajectory(N, nx, nu);

    auto make_solver = [&]() {
        SQPSolver solver(std::make_unique<DenseQPSolver>());
        return solver;
    };

    const auto bad_options = {
        std::pair<const char*, std::function<void(SQPSolverOptions&)>> {
            "kkt_tol = NaN", [](SQPSolverOptions& o) { o.kkt_tol = std::numeric_limits<double>::quiet_NaN(); } },
        { "kkt_tol = 0", [](SQPSolverOptions& o) { o.kkt_tol = 0.0; } },
        { "constr_viol_tol = NaN", [](SQPSolverOptions& o) { o.constr_viol_tol = std::numeric_limits<double>::quiet_NaN(); } },
        { "constr_viol_tol = -1", [](SQPSolverOptions& o) { o.constr_viol_tol = -1.0; } },
        { "stationarity_tol = inf", [](SQPSolverOptions& o) { o.stationarity_tol = std::numeric_limits<double>::infinity(); } },
        { "stationarity_tol = 0", [](SQPSolverOptions& o) { o.stationarity_tol = 0.0; } },
        { "reg_min = NaN", [](SQPSolverOptions& o) { o.reg_min = std::numeric_limits<double>::quiet_NaN(); } },
        { "line_search_rho = 1", [](SQPSolverOptions& o) { o.line_search_rho = 1.0; } },
        { "line_search_c = 0", [](SQPSolverOptions& o) { o.line_search_c = 0.0; } },
        { "line_search_alpha_min = -1", [](SQPSolverOptions& o) { o.line_search_alpha_min = -1.0; } },
        { "merit_penalty = inf", [](SQPSolverOptions& o) { o.merit_penalty = std::numeric_limits<double>::infinity(); } },
    };

    for (const auto& [name, setter] : bad_options) {
        SQPSolver solver = make_solver();
        setter(solver.options());
        Trajectory solution;
        EXPECT_FALSE(solver.solve(ocp, init_guess, solution))
            << "Invalid solver option (" << name << ") should be rejected by validation";
    }
}

// ===================== 测试：RTI 模式下非有限 merit 被安全拒绝 =====================
TEST(SQPAlgorithm, RTIRejectsNonFiniteMerit)
{
    // 测试目的：验证 RTI 模式下，当 trial merit 非有限时不会继续应用 delta
    // 流程：使用 NaNCost（QP 装配合法，但 merit 评估返回 NaN），开启 RTI 与线搜索，
    //      调用 solve。
    // 预期效果：solve 返回 false
    const int nx = 2, nu = 1, N = 2;
    MultiStageOCP ocp;
    StageSegment segment;
    segment.dynamics = std::make_shared<DoubleIntegrator>(
        Matrix::Identity(nx, nx), Matrix::Zero(nx, nu));
    segment.cost = std::make_shared<NaNCost>(
        Vector::Zero(nx), Matrix::Identity(nx, nx), Matrix::Identity(nu, nu));
    segment.N = N;
    segment.dt = 0.1;
    segment.v_sign = 1.0;
    segment.x_min = Vector::Constant(nx, -1e3);
    segment.x_max = Vector::Constant(nx, 1e3);
    segment.u_min = Vector::Constant(nu, -1e3);
    segment.u_max = Vector::Constant(nu, 1e3);
    ocp.addSegment(segment);

    Trajectory init_guess = makeZeroTrajectory(N, nx, nu);
    SQPSolver solver(std::make_unique<DenseQPSolver>());
    solver.options().use_rti = true;
    solver.options().use_line_search = true;
    Trajectory solution;
    EXPECT_FALSE(solver.solve(ocp, init_guess, solution))
        << "Should return false when RTI mode merit is non-finite";
}

// ===================== 测试：line_search_alpha_min 上界校验 =====================
TEST(SQPAlgorithm, RejectsLineSearchAlphaMinGreaterThanOne)
{
    // 测试目的：验证 validateProblem() 拒绝 line_search_alpha_min > 1.0，
    //          防止“最小步长”反而放大步长。
    // 流程：构造合法 OCP，但设置 line_search_alpha_min=2.0，调用 solve。
    // 预期效果：solve 返回 false
    const int nx = 2, nu = 1, N = 2;
    MultiStageOCP ocp;
    StageSegment segment;
    segment.dynamics = std::make_shared<DoubleIntegrator>(
        Matrix::Identity(nx, nx), Matrix::Zero(nx, nu));
    segment.cost = std::make_shared<QuadraticTrackingCost>(
        Vector::Zero(nx), Matrix::Identity(nx, nx), Matrix::Identity(nu, nu));
    segment.N = N;
    segment.dt = 0.1;
    segment.v_sign = 1.0;
    segment.x_min = Vector::Constant(nx, -1e3);
    segment.x_max = Vector::Constant(nx, 1e3);
    segment.u_min = Vector::Constant(nu, -1e3);
    segment.u_max = Vector::Constant(nu, 1e3);
    ocp.addSegment(segment);

    SQPSolver solver(std::make_unique<DenseQPSolver>());
    solver.options().line_search_alpha_min = 2.0;
    Trajectory init_guess = makeZeroTrajectory(N, nx, nu);
    Trajectory solution;
    EXPECT_FALSE(solver.solve(ocp, init_guess, solution))
        << "line_search_alpha_min > 1.0 should be rejected by validation";
}

// ===================== 测试：OCP 拒绝非法无穷边界 =====================
TEST(MultiStageOCP, RejectsInvalidInfiniteBounds)
{
    // 测试目的：验证 MultiStageOCP::validate() 拒绝不合法的无穷边界：
    //          下界为 +inf、上界为 -inf、同号无穷成对出现。
    // 流程：分别构造状态/控制边界为 (+inf,+inf)、(-inf,-inf)、(+inf,有限) 的段，
    //      调用 validate；同时验证单边无界 (-inf, +inf) 合法。
    // 预期效果：非法情况返回 false，合法情况返回 true。
    const int nx = 2, nu = 1;
    Matrix A = Matrix::Identity(nx, nx);
    Matrix B = Matrix::Zero(nx, nu);

    auto make_segment = [&](const Vector& x_min, const Vector& x_max,
                            const Vector& u_min, const Vector& u_max) {
        StageSegment seg;
        seg.dynamics = std::make_shared<DoubleIntegrator>(A, B);
        seg.N = 2;
        seg.dt = 0.1;
        seg.v_sign = 1.0;
        seg.x_min = x_min;
        seg.x_max = x_max;
        seg.u_min = u_min;
        seg.u_max = u_max;
        return seg;
    };

    const Vector x_valid_min = Vector::Constant(nx, -std::numeric_limits<double>::infinity());
    const Vector x_valid_max = Vector::Constant(nx, std::numeric_limits<double>::infinity());
    const Vector x_plus_inf = Vector::Constant(nx, std::numeric_limits<double>::infinity());
    const Vector x_minus_inf = Vector::Constant(nx, -std::numeric_limits<double>::infinity());
    const Vector u_valid_min = Vector::Constant(nu, -std::numeric_limits<double>::infinity());
    const Vector u_valid_max = Vector::Constant(nu, std::numeric_limits<double>::infinity());
    const Vector u_plus_inf = Vector::Constant(nu, std::numeric_limits<double>::infinity());
    const Vector u_minus_inf = Vector::Constant(nu, -std::numeric_limits<double>::infinity());

    // 合法：单边无界
    {
        MultiStageOCP ocp;
        ocp.addSegment(make_segment(x_valid_min, x_valid_max, u_valid_min, u_valid_max));
        EXPECT_TRUE(ocp.validate(nullptr));
    }

    // 非法：状态上下界均为 +inf
    {
        MultiStageOCP ocp;
        ocp.addSegment(make_segment(x_plus_inf, x_plus_inf, u_valid_min, u_valid_max));
        std::string reason;
        EXPECT_FALSE(ocp.validate(&reason));
        EXPECT_NE(reason.find("infinite"), std::string::npos);
    }

    // 非法：状态上下界均为 -inf
    {
        MultiStageOCP ocp;
        ocp.addSegment(make_segment(x_minus_inf, x_minus_inf, u_valid_min, u_valid_max));
        std::string reason;
        EXPECT_FALSE(ocp.validate(&reason));
        EXPECT_NE(reason.find("infinite"), std::string::npos);
    }

    // 非法：状态下界为 +inf（上界有限）
    {
        MultiStageOCP ocp;
        ocp.addSegment(make_segment(x_plus_inf, Vector::Constant(nx, 1.0), u_valid_min, u_valid_max));
        std::string reason;
        EXPECT_FALSE(ocp.validate(&reason));
        EXPECT_NE(reason.find("infinite"), std::string::npos);
    }

    // 非法：控制上界为 -inf
    {
        MultiStageOCP ocp;
        ocp.addSegment(make_segment(x_valid_min, x_valid_max, u_valid_min, u_minus_inf));
        std::string reason;
        EXPECT_FALSE(ocp.validate(&reason));
        EXPECT_NE(reason.find("infinite"), std::string::npos);
    }
}

// ===================== 测试：约束维度 ng <= 0 被拒绝 =====================
class NegativeNgConstraint : public Constraint {
public:
    int ng() const override { return -1; }
    void evaluate(const Vector& x, const Vector& u, const Vector& p, Vector& g) const override
    {
        (void)x;
        (void)u;
        (void)p;
        g.resize(0);
    }
    void jacobian(const Vector& x, const Vector& u, const Vector& p, Matrix& Cx,
        Matrix& Cu) const override
    {
        (void)x;
        (void)u;
        (void)p;
        Cx.resize(0, 0);
        Cu.resize(0, 0);
    }
    std::shared_ptr<Constraint> clone() const override
    {
        return std::make_shared<NegativeNgConstraint>();
    }
};

class ZeroNgConstraint : public Constraint {
public:
    int ng() const override { return 0; }
    void evaluate(const Vector& x, const Vector& u, const Vector& p, Vector& g) const override
    {
        (void)x;
        (void)u;
        (void)p;
        g.resize(0);
    }
    void jacobian(const Vector& x, const Vector& u, const Vector& p, Matrix& Cx,
        Matrix& Cu) const override
    {
        (void)x;
        (void)u;
        (void)p;
        Cx.resize(0, 0);
        Cu.resize(0, 0);
    }
    std::shared_ptr<Constraint> clone() const override
    {
        return std::make_shared<ZeroNgConstraint>();
    }
};

TEST(MultiStageOCP, RejectsNonPositiveConstraintDimension)
{
    // 测试目的：验证 MultiStageOCP::validate() 拒绝 Constraint::ng() <= 0 的非法约束，
    //          避免后续 checkConvergence 对空向量调用 maxCoeff() 触发 Eigen 断言。
    // 流程：分别构造含 NegativeNgConstraint 与 ZeroNgConstraint 的段，调用 validate。
    // 预期效果：均返回 false，错误信息包含“ng”
    auto make_ocp_with_constraint = [](const std::shared_ptr<Constraint>& constraint) {
        MultiStageOCP ocp;
        StageSegment segment;
        segment.dynamics = std::make_shared<DoubleIntegrator>(
            Matrix::Identity(2, 2), Matrix::Zero(2, 1));
        segment.cost = std::make_shared<QuadraticTrackingCost>(
            Vector::Zero(2), Matrix::Identity(2, 2), Matrix::Identity(1, 1));
        segment.N = 2;
        segment.dt = 0.1;
        segment.v_sign = 1.0;
        segment.x_min = Vector::Constant(2, -1e3);
        segment.x_max = Vector::Constant(2, 1e3);
        segment.u_min = Vector::Constant(1, -1e3);
        segment.u_max = Vector::Constant(1, 1e3);
        segment.constraints.push_back(constraint);
        ocp.addSegment(segment);
        return ocp;
    };

    const std::vector<std::shared_ptr<Constraint>> bad_constraints = {
        std::make_shared<NegativeNgConstraint>(),
        std::make_shared<ZeroNgConstraint>()
    };
    for (const auto& constraint : bad_constraints) {
        const auto ocp = make_ocp_with_constraint(constraint);
        std::string reason;
        EXPECT_FALSE(ocp.validate(&reason))
            << "ng=" << constraint->ng() << " constraint should be rejected";
        EXPECT_NE(reason.find("ng"), std::string::npos);
    }
}


// 测试目的：验证 OpenMP 并行 linearize 与串行 linearize 结果一致。
// 流程：构造 N 超过 omp_parallel_threshold 的含一般约束 OCP，分别用 use_omp=true/false
//      求解同一问题，比较两条轨迹。
// 预期效果：两者均成功，状态/控制 L2 误差 < 1e-10。
TEST(SQPAlgorithm, ParallelLinearizeMatchesSerial) {
    const int N = 60, nx = 2, nu = 1;
    const double dt = 0.1;
    const Vector x_ref = Vector::Zero(nx);
    const Matrix Q = Matrix::Identity(nx, nx);
    const Matrix R = Matrix::Identity(nu, nu);
    const Vector x_min = Vector::Constant(nx, -1e3);
    const Vector x_max = Vector::Constant(nx, 1e3);
    const Vector u_min = Vector::Constant(nu, -1e3);
    const Vector u_max = Vector::Constant(nu, 1e3);

    MultiStageOCP ocp = makeDoubleIntegratorOCP(N, dt, Matrix::Identity(nx, nx),
        dt * Matrix::Identity(nx, nu), x_ref, Q, R, x_min, x_max, u_min, u_max);
    // 加入一个需要 clone 的一般约束，确保并行路径会创建独立工作区
    ocp.segments()[0].constraints.push_back(
        std::make_shared<ControlUpperBoundConstraint>(0, 0.5));

    Trajectory init = makeZeroTrajectory(N, nx, nu);
    Trajectory sol_serial, sol_parallel;

    {
        auto qp_solver = std::make_unique<HPIPMQPSolver>(N, nx, nu, nx, nu, 1, 0, -1);
        SQPSolver solver(std::move(qp_solver));
        solver.options().use_omp = false;
        solver.options().omp_parallel_threshold = 10;
        ASSERT_TRUE(solver.solve(ocp, init, sol_serial));
    }

    {
        auto qp_solver = std::make_unique<HPIPMQPSolver>(N, nx, nu, nx, nu, 1, 0, -1);
        SQPSolver solver(std::move(qp_solver));
        solver.options().use_omp = true;
        solver.options().omp_parallel_threshold = 10;
        ASSERT_TRUE(solver.solve(ocp, init, sol_parallel));
    }

    for (int k = 0; k <= N; ++k) {
        EXPECT_LT((sol_parallel.x[k] - sol_serial.x[k]).norm(), 1e-10)
            << "x mismatch at stage " << k;
    }
    for (int k = 0; k < N; ++k) {
        EXPECT_LT((sol_parallel.u[k] - sol_serial.u[k]).norm(), 1e-10)
            << "u mismatch at stage " << k;
    }
}
