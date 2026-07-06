#include <cmath>
#include <limits>
#include <memory>

#include <gtest/gtest.h>

#include "../src/qp/dense_qp_solver.h"
#include "../src/qp/qp_data.h"

using namespace stc_SQP;

// 构造一个无普通约束的 LQR QP，用于 Dense 求解器基础测试
static std::unique_ptr<QPData> buildDenseLqrQP(int N, int nx, int nu) {
    auto qp = std::make_unique<QPData>(N, nx, nu, 0);
    const double inf = 1e3;
    for (int k = 0; k <= N; ++k) {
        qp->Q[k] = Matrix::Identity(nx, nx);
        qp->q[k] = Vector::Zero(nx);
        qp->lbx[k] = Vector::Constant(nx, -inf);
        qp->ubx[k] = Vector::Constant(nx, inf);
    }
    qp->Q[N] = 10.0 * Matrix::Identity(nx, nx);
    qp->lbx[0] << 1.0, 0.0;
    qp->ubx[0] << 1.0, 0.0;
    for (int k = 0; k < N; ++k) {
        qp->A[k] = Matrix::Identity(nx, nx);
        qp->B[k] = Matrix::Zero(nx, nu);
        qp->B[k](0, 0) = 1.0;
        qp->B[k](1, 0) = 1.0;
        qp->b[k] = Vector::Zero(nx);
        qp->R[k] = Matrix::Identity(nu, nu);
        qp->S[k] = Matrix::Zero(nu, nx);
        qp->r[k] = Vector::Zero(nu);
        qp->lbu[k] = Vector::Constant(nu, -inf);
        qp->ubu[k] = Vector::Constant(nu, inf);
    }
    return qp;
}

// 构造一个带软约束的 QP：普通约束 x <= 0.5 通过上界松弛允许违反
static std::unique_ptr<QPData> buildDenseSoftQP(int N, int nx, int nu) {
    const int ng = 1;
    auto qp = std::make_unique<QPData>(N, nx, nu, ng);
    const double inf = 1e3;
    for (int k = 0; k <= N; ++k) {
        qp->Q[k] = Matrix::Identity(nx, nx);
        qp->q[k] = Vector::Zero(nx);
        qp->lbx[k] = Vector::Constant(nx, -inf);
        qp->ubx[k] = Vector::Constant(nx, inf);
    }
    qp->Q[N] = 10.0 * Matrix::Identity(nx, nx);
    qp->lbx[0] << 1.0, 0.0;
    qp->ubx[0] << 1.0, 0.0;
    for (int k = 0; k < N; ++k) {
        qp->A[k] = Matrix::Identity(nx, nx);
        qp->B[k] = Matrix::Zero(nx, nu);
        qp->B[k](0, 0) = 1.0;
        qp->B[k](1, 0) = 1.0;
        qp->b[k] = Vector::Zero(nx);
        qp->R[k] = Matrix::Identity(nu, nu);
        qp->S[k] = Matrix::Zero(nu, nx);
        qp->r[k] = Vector::Zero(nu);
        qp->lbu[k] = Vector::Constant(nu, -inf);
        qp->ubu[k] = Vector::Constant(nu, inf);
        qp->C[k] = Matrix::Zero(ng, nx);
        qp->C[k](0, 0) = 1.0;
        qp->d[k] = Vector::Constant(ng, 0.5);
    }
    qp->soft_config = std::make_unique<SoftConstraintConfig>();
    qp->soft_config->ns = 1;
    qp->soft_config->Zl = Vector::Constant(1, 0.0);
    qp->soft_config->Zu = Vector::Constant(1, 1e2);
    qp->soft_config->zl = Vector::Constant(1, 0.0);
    qp->soft_config->zu = Vector::Constant(1, 0.0);
    qp->soft_config->idxs = { 0 };
    return qp;
}

// 构造一个初始状态自由的单阶段标量 QP：x1 = u0，代价含 x0 的线性项
// 用于验证初始状态未被固定时，求解器仍能通过边界和动力学正常求解
static std::unique_ptr<QPData> buildFreeInitialScalarQP(double lbx0, double ubx0) {
    const int N = 1, nx = 1, nu = 1, ng = 0;
    auto qp = std::make_unique<QPData>(N, nx, nu, ng);
    qp->Q[0] = Matrix::Identity(nx, nx);
    qp->q[0] = Vector::Ones(nx);        // 线性项使自由 x0 最优为 -1
    qp->lbx[0] = Vector::Constant(nx, lbx0);
    qp->ubx[0] = Vector::Constant(nx, ubx0);
    qp->Q[1] = Matrix::Identity(nx, nx);
    qp->q[1] = Vector::Zero(nx);
    qp->lbx[1] = Vector::Constant(nx, -1e3);
    qp->ubx[1] = Vector::Constant(nx, 1e3);
    qp->A[0] = Matrix::Zero(nx, nx);
    qp->B[0] = Matrix::Identity(nu, nu);
    qp->b[0] = Vector::Zero(nx);
    qp->R[0] = Matrix::Identity(nu, nu);
    qp->S[0] = Matrix::Zero(nu, nx);
    qp->r[0] = Vector::Zero(nu);
    qp->lbu[0] = Vector::Constant(nu, -1e3);
    qp->ubu[0] = Vector::Constant(nu, 1e3);
    return qp;
}

// 测试目的：验证 Dense LDLT 求解器在简单 LQR 问题上收敛
// 流程：构造无普通约束 QP，调用 DenseQPSolver
// 预期效果：返回 SUCCESS
TEST(DenseQPSolver, SolvesSimpleLqr) {
    const int N = 5, nx = 2, nu = 1;
    auto qp_data = buildDenseLqrQP(N, nx, nu);
    DenseQPSolver solver;
    solver.setTolerance(1e-8);
    QPSolution sol;
    EXPECT_EQ(solver.solve(*qp_data, sol), QPSolverStatus::SUCCESS);
}

// 测试目的：验证 Dense LDLT 求解器拒绝越界软约束索引
// 流程：构造有效软约束 QP，将 idxs 改为 ng，调用 DenseQPSolver
// 预期效果：返回 INVALID_ARGUMENT
TEST(DenseQPSolver, RejectsOutOfBoundsSoftIdx) {
    const int N = 5, nx = 2, nu = 1;
    auto qp_data = buildDenseSoftQP(N, nx, nu);
    qp_data->soft_config->idxs = { 1 }; // 越界
    DenseQPSolver solver;
    QPSolution sol;
    EXPECT_EQ(solver.solve(*qp_data, sol), QPSolverStatus::INVALID_ARGUMENT);
}

// 测试目的：验证 Dense LDLT 求解器拒绝权重向量长度与 ns 不匹配的配置
// 流程：构造有效软约束 QP，将 Zu 长度改为 2，调用 DenseQPSolver
// 预期效果：返回 INVALID_ARGUMENT
TEST(DenseQPSolver, RejectsMismatchedSoftWeights) {
    const int N = 5, nx = 2, nu = 1;
    auto qp_data = buildDenseSoftQP(N, nx, nu);
    qp_data->soft_config->Zu = Vector::Constant(2, 1e2); // 长度不匹配
    DenseQPSolver solver;
    QPSolution sol;
    EXPECT_EQ(solver.solve(*qp_data, sol), QPSolverStatus::INVALID_ARGUMENT);
}

// 测试目的：验证 Dense LDLT 求解器拒绝重复的软约束索引
// 流程：构造有效软约束 QP，将 idxs 改为重复索引 {0, 0}，调用 DenseQPSolver
// 预期效果：返回 INVALID_ARGUMENT，避免 slack 映射被静默覆盖
TEST(DenseQPSolver, RejectsDuplicateSoftIdx) {
    const int N = 5, nx = 2, nu = 1;
    auto qp_data = buildDenseSoftQP(N, nx, nu);
    qp_data->soft_config->ns = 2;
    qp_data->soft_config->Zl = Vector::Constant(2, 0.0);
    qp_data->soft_config->Zu = Vector::Constant(2, 1e2);
    qp_data->soft_config->zl = Vector::Constant(2, 0.0);
    qp_data->soft_config->zu = Vector::Constant(2, 0.0);
    qp_data->soft_config->idxs = { 0, 0 }; // 重复索引
    DenseQPSolver solver;
    QPSolution sol;
    EXPECT_EQ(solver.solve(*qp_data, sol), QPSolverStatus::INVALID_ARGUMENT);
}

// 测试目的：验证 Dense LDLT 求解器拒绝 ns == 0 但配置容器非空的非法配置
// 流程：将 soft_config->ns 改为 0，保持 idxs/Zu 非空，调用 DenseQPSolver
// 预期效果：返回 INVALID_ARGUMENT
TEST(DenseQPSolver, RejectsNsZeroWithNonEmptyConfig) {
    const int N = 5, nx = 2, nu = 1;
    auto qp_data = buildDenseSoftQP(N, nx, nu);
    qp_data->soft_config->ns = 0;
    DenseQPSolver solver;
    QPSolution sol;
    EXPECT_EQ(solver.solve(*qp_data, sol), QPSolverStatus::INVALID_ARGUMENT);
}

// 测试目的：验证 Dense LDLT 求解器拒绝默认构造或未初始化的 QPData
// 流程：构造 N/nx/nu 为 0 的 QPData，或关键 vector 尺寸不完整，调用 DenseQPSolver
// 预期效果：返回 INVALID_ARGUMENT，不触发越界访问
TEST(DenseQPSolver, RejectsInvalidQPData) {
    DenseQPSolver solver;
    QPSolution sol;
    // 默认构造对象
    QPData default_data;
    EXPECT_EQ(solver.solve(default_data, sol), QPSolverStatus::INVALID_ARGUMENT);
    // 维度合法但 A 的 vector 尺寸不完整（模拟部分初始化）
    QPData incomplete(3, 2, 1, 0);
    incomplete.A.clear();
    EXPECT_EQ(solver.solve(incomplete, sol), QPSolverStatus::INVALID_ARGUMENT);
}

// 测试目的：验证 setTolerance 只接受有限正数容差
// 流程：分别传入 0、负数、NaN、inf 和合法值，检查是否抛异常
// 预期效果：非法值抛 std::invalid_argument，合法值可正常设置
TEST(DenseQPSolver, RejectsInvalidTolerance) {
    DenseQPSolver solver;
    EXPECT_NO_THROW(solver.setTolerance(1e-8));
    EXPECT_THROW(solver.setTolerance(0.0), std::invalid_argument);
    EXPECT_THROW(solver.setTolerance(-1e-6), std::invalid_argument);
    EXPECT_THROW(solver.setTolerance(std::numeric_limits<double>::quiet_NaN()),
                  std::invalid_argument);
    EXPECT_THROW(solver.setTolerance(std::numeric_limits<double>::infinity()),
                  std::invalid_argument);
    EXPECT_THROW(solver.setTolerance(-std::numeric_limits<double>::infinity()),
                  std::invalid_argument);
}

// 测试目的：验证非有限初始状态边界不会被误判为固定初始状态
// 流程：构造 lbx[0] = -inf、ubx[0] = inf 的标量 QP，其中自由 x0 最优为 -1
// 预期效果：求解成功且 x0 取到自由最优值 -1，而不是被固定为某个有限值
TEST(DenseQPSolver, DoesNotTreatNonFiniteInitialBoundsAsFixed) {
    auto qp_data = buildFreeInitialScalarQP(
        -std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity());
    DenseQPSolver solver;
    solver.setTolerance(1e-8);
    QPSolution sol;
    EXPECT_EQ(solver.solve(*qp_data, sol), QPSolverStatus::SUCCESS);
    ASSERT_EQ(sol.x.size(), 2u);
    EXPECT_NEAR(sol.x[0](0), -1.0, 1e-6);
}

// 测试目的：验证有限但不等的初始状态边界不会被当作等式固定
// 流程：构造 lbx[0] = -10、ubx[0] = 10 的标量 QP，自由 x0 最优为 -1
// 预期效果：求解成功，x0 在边界内且不等于任一边界
TEST(DenseQPSolver, SolvesWithFreeButBoundedInitialState) {
    auto qp_data = buildFreeInitialScalarQP(-10.0, 10.0);
    DenseQPSolver solver;
    solver.setTolerance(1e-8);
    QPSolution sol;
    EXPECT_EQ(solver.solve(*qp_data, sol), QPSolverStatus::SUCCESS);
    ASSERT_EQ(sol.x.size(), 2u);
    EXPECT_NEAR(sol.x[0](0), -1.0, 1e-6);
    EXPECT_GT(sol.x[0](0), -10.0);
    EXPECT_LT(sol.x[0](0), 10.0);
}
