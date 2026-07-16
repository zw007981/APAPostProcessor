#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>

#include <gtest/gtest.h>

#include "constraints/constraint.hpp"
#include "costs/quadratic_tracking.h"
#include "models/dynamical_system.h"
#include "ocp/multi_stage_ocp.h"
#include "strategies/auto_adaptive_strategy.h"
#include "strategies/manual_hierarchical_strategy.h"
#include "util/trajectory.h"

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
        (void)v_sign;
        x_next = A_ * x + B_ * u;
        A = A_;
        B = B_;
    }

private:
    Matrix A_;
    Matrix B_;
};

// 一般控制约束：u(idx) - limit <= 0
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
    std::shared_ptr<Constraint> clone() const
    {
        return std::make_shared<ControlUpperBoundConstraint>(control_index_, limit_);
    }

private:
    int control_index_ = 0;
    double limit_ = 0.0;
};

// 构造一个简单的 LQR 双积分器 OCP
static MultiStageOCP makeLqrOCP(int N, double dt, int nx, int nu,
    bool add_control_constraint = false)
{
    Matrix A = Matrix::Identity(nx, nx);
    Matrix B = Matrix::Zero(nx, nu);
    for (int i = 0; i < nu; ++i) {
        B(i, i) = dt;
    }
    Vector x_ref = Vector::Zero(nx);
    Matrix Q = Matrix::Identity(nx, nx);
    Matrix R = Matrix::Identity(nu, nu);
    Vector x_min = Vector::Constant(nx, -1e3);
    Vector x_max = Vector::Constant(nx, 1e3);
    Vector u_min = Vector::Constant(nu, -1e3);
    Vector u_max = Vector::Constant(nu, 1e3);

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
    if (add_control_constraint) {
        segment.constraints.push_back(std::make_shared<ControlUpperBoundConstraint>(0, 0.5));
    }

    MultiStageOCP ocp;
    ocp.addSegment(segment);
    return ocp;
}

static Trajectory makeZeroInitialGuess(int N, int nx, int nu)
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

// 测试目的：AutoAdaptiveStrategy 在短 N（< short_n_threshold）下能收敛
// 流程：构造 N=10 的双积分器 LQR，初始猜测为 0，调用策略求解
// 预期效果：solve 返回 true，最终状态接近参考点 0
TEST(AutoAdaptiveStrategy, SolvesShortNLqr) {
    const int N = 10, nx = 2, nu = 1;
    const double dt = 0.1;
    AutoAdaptiveStrategy strategy(0, 0, 0, 0, /*block_size=*/10, /*short_n_threshold=*/50);
    MultiStageOCP ocp = makeLqrOCP(N, dt, nx, nu);
    Trajectory init = makeZeroInitialGuess(N, nx, nu);
    Trajectory sol;
    EXPECT_TRUE(strategy.solve(ocp, init, sol));
    for (const auto& xk : sol.x) {
        EXPECT_LT(xk.norm(), 1e-3);
    }
}

// 测试目的：AutoAdaptiveStrategy 在长 N（>= short_n_threshold）下能收敛
// 流程：构造 N=60 的双积分器 LQR，调用策略求解，验证结果
// 预期效果：solve 返回 true，最终状态接近参考点 0
TEST(AutoAdaptiveStrategy, SolvesLongNLqr) {
    const int N = 60, nx = 2, nu = 1;
    const double dt = 0.1;
    AutoAdaptiveStrategy strategy(0, 0, 0, 0, /*block_size=*/10, /*short_n_threshold=*/50);
    MultiStageOCP ocp = makeLqrOCP(N, dt, nx, nu);
    Trajectory init = makeZeroInitialGuess(N, nx, nu);
    Trajectory sol;
    EXPECT_TRUE(strategy.solve(ocp, init, sol));
    for (const auto& xk : sol.x) {
        EXPECT_LT(xk.norm(), 1e-3);
    }
}

// 测试目的：AutoAdaptiveStrategy 构造函数对非法参数抛出异常
// 流程：分别传入 block_size<=0、short_n_threshold<0、nbx/nbu/ng/ns<0
// 预期效果：均触发 std::invalid_argument
TEST(AutoAdaptiveStrategy, RejectsInvalidConstructorArguments) {
    EXPECT_THROW(AutoAdaptiveStrategy(0, 0, 0, 0, 0, 50), std::invalid_argument); // block_size=0
    EXPECT_THROW(AutoAdaptiveStrategy(0, 0, 0, 0, 10, -1), std::invalid_argument); // short_n_threshold<0
    EXPECT_THROW(AutoAdaptiveStrategy(-1, 0, 0, 0), std::invalid_argument); // nbx<0
    EXPECT_THROW(AutoAdaptiveStrategy(0, -1, 0, 0), std::invalid_argument); // nbu<0
    EXPECT_THROW(AutoAdaptiveStrategy(0, 0, -1, 0), std::invalid_argument); // ng<0
    EXPECT_THROW(AutoAdaptiveStrategy(0, 0, 0, -1), std::invalid_argument); // ns<0
}

// 测试目的：AutoAdaptiveStrategy 检测到传入 ng 与 OCP 实际一般约束维度不一致时返回 false
// 流程：构造带一个控制约束的 OCP（实际 ng=1），但用 ng=0 构造策略并调用 solve
// 预期效果：solve 返回 false，不崩溃
TEST(AutoAdaptiveStrategy, RejectsMismatchedNg) {
    const int N = 20, nx = 2, nu = 1;
    const double dt = 0.1;
    AutoAdaptiveStrategy strategy(0, 0, /*ng=*/0, 0);
    MultiStageOCP ocp = makeLqrOCP(N, dt, nx, nu, /*add_control_constraint=*/true);
    Trajectory init = makeZeroInitialGuess(N, nx, nu);
    Trajectory sol;
    EXPECT_FALSE(strategy.solve(ocp, init, sol));
}

// ===================== ManualHierarchicalStrategy 测试 =====================

// 测试辅助：暴露 protected 插值/下采样方法，便于白盒测试
class TestableManualHierarchicalStrategy : public ManualHierarchicalStrategy {
public:
    TestableManualHierarchicalStrategy(int nbx, int nbu, int ng, int ns,
        const HierarchicalOptions& opts = HierarchicalOptions(),
        int block_size = 10, int short_n_threshold = 50)
        : ManualHierarchicalStrategy(nbx, nbu, ng, ns, opts, block_size, short_n_threshold)
    {
    }

    using ManualHierarchicalStrategy::interpolateDynamicsConsistent;
    using ManualHierarchicalStrategy::downsampleTrajectory;
};

// 测试目的：ManualHierarchicalStrategy 对 LQR 问题能收敛
// 流程：构造 N=40 的双积分器 LQR，分别测试 enable=true/false，调用策略求解
// 预期效果：两种模式均返回 true，最终状态接近参考点 0
TEST(ManualHierarchicalStrategy, SolvesLqrWithAndWithoutHierarchy) {
    const int N = 40, nx = 2, nu = 1;
    const double dt = 0.1;

    HierarchicalOptions opts;
    opts.enable = true;
    opts.coarse_n = 4; // 粗化为 4 步
    opts.coarse_max_iter = 5;
    opts.fine_max_iter = 5;
    ManualHierarchicalStrategy strategy(0, 0, 0, 0, opts);

    MultiStageOCP ocp = makeLqrOCP(N, dt, nx, nu);
    Trajectory init = makeZeroInitialGuess(N, nx, nu);
    Trajectory sol;
    EXPECT_TRUE(strategy.solve(ocp, init, sol));
    for (const auto& xk : sol.x) {
        EXPECT_LT(xk.norm(), 1e-2);
    }

    HierarchicalOptions opts_disabled;
    opts_disabled.enable = false;
    opts_disabled.fine_max_iter = 10;
    ManualHierarchicalStrategy strategy_disabled(0, 0, 0, 0, opts_disabled);
    Trajectory sol2;
    EXPECT_TRUE(strategy_disabled.solve(ocp, init, sol2));
    for (const auto& xk : sol2.x) {
        EXPECT_LT(xk.norm(), 1e-2);
    }
}

// 测试目的：ManualHierarchicalStrategy 构造函数对非法参数抛出异常
// 流程：分别传入 nbx/nbu/ng/ns<0、block_size<=0、short_n_threshold<0、max_iter<=0
// 预期效果：均触发 std::invalid_argument
TEST(ManualHierarchicalStrategy, RejectsInvalidConstructorArguments) {
    HierarchicalOptions opts;
    EXPECT_THROW(ManualHierarchicalStrategy(-1, 0, 0, 0), std::invalid_argument);
    EXPECT_THROW(ManualHierarchicalStrategy(0, -1, 0, 0), std::invalid_argument);
    EXPECT_THROW(ManualHierarchicalStrategy(0, 0, -1, 0), std::invalid_argument);
    EXPECT_THROW(ManualHierarchicalStrategy(0, 0, 0, -1), std::invalid_argument);
    EXPECT_THROW(ManualHierarchicalStrategy(0, 0, 0, 0, opts, 0), std::invalid_argument); // block_size=0
    EXPECT_THROW(ManualHierarchicalStrategy(0, 0, 0, 0, opts, 10, -1), std::invalid_argument);
    opts.coarse_max_iter = 0;
    EXPECT_THROW(ManualHierarchicalStrategy(0, 0, 0, 0, opts), std::invalid_argument);
}

// 测试目的：ManualHierarchicalStrategy 检测到传入 ng 与 OCP 实际一般约束维度不一致时返回 false
// 流程：构造带一个控制约束的 OCP（实际 ng=1），但用 ng=0 构造策略并调用 solve
// 预期效果：solve 返回 false，不崩溃
TEST(ManualHierarchicalStrategy, RejectsMismatchedNg) {
    const int N = 20, nx = 2, nu = 1;
    const double dt = 0.1;
    ManualHierarchicalStrategy strategy(0, 0, /*ng=*/0, 0);
    MultiStageOCP ocp = makeLqrOCP(N, dt, nx, nu, /*add_control_constraint=*/true);
    Trajectory init = makeZeroInitialGuess(N, nx, nu);
    Trajectory sol;
    EXPECT_FALSE(strategy.solve(ocp, init, sol));
}

// 测试目的：验证 interpolateDynamicsConsistent 控制插值不越界外推
// 流程：构造粗解（coarse_N=3，控制序列为 0, 1, 2），细 OCP 步数远大于粗 OCP，
//      调用插值方法；由于细段时间超出粗控制序列覆盖范围，alpha 应被 clamp 到 1。
// 预期效果：细段末端控制等于粗段最后一个控制（2），状态传播满足动力学。
TEST(ManualHierarchicalStrategy, InterpolationClampsControlAlpha) {
    const int nx = 2, nu = 1;
    const double dt = 0.1;

    // 粗 OCP：3 步，dt=1.0，总时长 3.0
    MultiStageOCP coarse_ocp = makeLqrOCP(/*N=*/3, /*dt=*/1.0, nx, nu);
    // 细 OCP：30 步，dt=0.1，总时长 3.0
    MultiStageOCP fine_ocp = makeLqrOCP(/*N=*/30, /*dt=*/0.1, nx, nu);

    Trajectory coarse_sol;
    coarse_sol.resize(3, nx, nu);
    for (auto& xk : coarse_sol.x) {
        xk.setZero();
    }
    coarse_sol.u[0] << 0.0;
    coarse_sol.u[1] << 1.0;
    coarse_sol.u[2] << 2.0;

    TestableManualHierarchicalStrategy strategy(0, 0, 0, 0);
    Trajectory fine_guess = strategy.interpolateDynamicsConsistent(coarse_sol, coarse_ocp, fine_ocp);

    // 末端控制必须被 clamp 到粗段最后一个控制，不能外推出 >2 的值
    EXPECT_NEAR(fine_guess.u.back()(0), 2.0, 1e-12);
    // 状态序列满足动力学（双积分器 u 为常量加速度）
    for (size_t k = 0; k + 1 < fine_guess.x.size(); ++k) {
        Vector unit_x = Vector::Zero(nx);
        unit_x(0) = 1.0;
        // fine_guess.u[k] 是 1 维向量，必须显式取标量再与 unit_x 相乘，
        // 避免 Release 模式下出现未定义行为的动态尺寸矩阵乘积。
        const Vector expected_x_next = fine_guess.x[k]
            + (dt * fine_guess.u[k](0)) * unit_x;
        EXPECT_TRUE((fine_guess.x[k + 1] - expected_x_next).norm() < 1e-12)
            << "k=" << k << " x_next mismatch";
    }
}

// 测试目的：验证 downsampleTrajectory 按最近邻正确下采样
// 流程：构造一个正弦状的细轨迹，按粗 OCP 步长下采样，检查每点取自最近细点。
TEST(ManualHierarchicalStrategy, DownsampleTrajectoryNearestNeighbor) {
    const int nx = 2, nu = 1;
    const double fine_dt = 0.1;
    const double coarse_dt = 0.25;
    const int fine_N = 30;
    const int coarse_N = static_cast<int>(std::round(fine_N * fine_dt / coarse_dt)); // 12

    MultiStageOCP fine_ocp = makeLqrOCP(fine_N, fine_dt, nx, nu);
    MultiStageOCP coarse_ocp = makeLqrOCP(coarse_N, coarse_dt, nx, nu);

    Trajectory fine;
    fine.resize(fine_N, nx, nu);
    for (int k = 0; k <= fine_N; ++k) {
        fine.x[k] << std::sin(k * fine_dt), std::cos(k * fine_dt);
        if (k < fine_N) {
            fine.u[k] << static_cast<double>(k);
        }
    }

    TestableManualHierarchicalStrategy strategy(0, 0, 0, 0);
    Trajectory coarse = strategy.downsampleTrajectory(fine, fine_ocp, coarse_ocp);

    EXPECT_EQ(static_cast<int>(coarse.x.size()), coarse_N + 1);
    EXPECT_EQ(static_cast<int>(coarse.u.size()), coarse_N);
    for (int j = 0; j < coarse_N; ++j) {
        const double t = j * coarse_dt;
        int expected_idx = static_cast<int>(std::round(t / fine_dt));
        expected_idx = std::max(0, std::min(fine_N - 1, expected_idx));
        EXPECT_TRUE((coarse.x[j] - fine.x[expected_idx]).norm() < 1e-12)
            << "j=" << j << " nearest index mismatch";
        EXPECT_TRUE((coarse.u[j] - fine.u[expected_idx]).norm() < 1e-12)
            << "j=" << j << " control nearest index mismatch";
    }
    // 末端状态
    EXPECT_TRUE((coarse.x[coarse_N] - fine.x[fine_N]).norm() < 1e-12);
}

// 测试目的：验证 ManualHierarchicalStrategy 对带一般约束的问题仍与 AutoAdaptiveStrategy 一致
// 流程：构造带控制上界的 LQR，分别用两种策略求解，比较最终状态轨迹
// 预期效果：两者均成功，状态轨迹 L2 误差 < 1e-6
TEST(ManualHierarchicalStrategy, MatchesAutoAdaptiveOnConstrainedLqr) {
    const int N = 40, nx = 2, nu = 1;
    const double dt = 0.1;

    HierarchicalOptions opts;
    opts.coarse_n = 5;
    opts.coarse_max_iter = 5;
    opts.fine_max_iter = 5;
    ManualHierarchicalStrategy manual(0, 0, /*ng=*/1, 0, opts);
    AutoAdaptiveStrategy auto_strat(0, 0, /*ng=*/1, 0, /*block_size=*/10, /*short_n_threshold=*/50);

    MultiStageOCP ocp = makeLqrOCP(N, dt, nx, nu, /*add_control_constraint=*/true);
    Trajectory init = makeZeroInitialGuess(N, nx, nu);
    Trajectory sol_manual, sol_auto;
    ASSERT_TRUE(manual.solve(ocp, init, sol_manual));
    ASSERT_TRUE(auto_strat.solve(ocp, init, sol_auto));

    for (int k = 0; k <= N; ++k) {
        EXPECT_LT((sol_manual.x[k] - sol_auto.x[k]).norm(), 1e-6)
            << "x mismatch at stage " << k;
    }
    for (int k = 0; k < N; ++k) {
        EXPECT_LT((sol_manual.u[k] - sol_auto.u[k]).norm(), 1e-6)
            << "u mismatch at stage " << k;
    }
}
