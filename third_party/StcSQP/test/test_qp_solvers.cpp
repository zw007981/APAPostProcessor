#include <cmath>
#include <limits>
#include <memory>
#include <utility>

#include <gtest/gtest.h>

#include "constraints/convex_corridor_constraint.h"
#include "costs/quadratic_tracking.h"
#include "generated/corridor.h"
#include "models/bicycle_model_delta.h"
#include "ocp/multi_stage_ocp.h"
#include "problem_updater.h"
#include "simple_parking_map.h"
#include "sqp/sqp_algorithm.h"
#include "../src/qp/dense_qp_solver.h"
#include "../src/qp/hpipm_solver.h"
#include "../src/qp/qp_data.h"
#include "../src/qp/soft_constraint_validation.h"

using namespace stc_SQP;

// 组合判据：在数值接近 0 时使用绝对容差，否则使用相对容差
static bool isClose(const Vector& a, const Vector& b, double rel_tol, double abs_tol) {
    return (a - b).norm() <= rel_tol * std::max(a.norm(), b.norm()) + abs_tol;
}

// 填充 LQR QP 的公共动力/代价矩阵
static void fillLqrDynamicsAndCost(QPData* qp, int N, int nx, int nu) {
    const double inf = 1e3;
    for (int k = 0; k <= N; ++k) {
        qp->Q[k] = Matrix::Identity(nx, nx);
        qp->q[k] = Vector::Zero(nx);
        qp->lbx[k] = Vector::Constant(nx, -inf);
        qp->ubx[k] = Vector::Constant(nx, inf);
    }
    qp->Q[N] = 10.0 * Matrix::Identity(nx, nx);
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
}

// 构造无普通约束的 LQR QP
static std::unique_ptr<QPData> buildLqrQP(int N, int nx, int nu) {
    auto qp = std::make_unique<QPData>(N, nx, nu, 0);
    fillLqrDynamicsAndCost(qp.get(), N, nx, nu);
    // 初始状态固定为 [1, 0]
    qp->lbx[0] << 1.0, 0.0;
    qp->ubx[0] << 1.0, 0.0;
    return qp;
}

// 构造含硬普通约束 x <= 0.5 的 QP
static std::unique_ptr<QPData> buildHardConstraintQP(int N, int nx, int nu) {
    const int ng = 1;
    auto qp = std::make_unique<QPData>(N, nx, nu, ng);
    fillLqrDynamicsAndCost(qp.get(), N, nx, nu);
    // 初始状态固定为 [1, 0]；x0 违反约束，硬约束不可行，因此测试会改为可行点
    qp->lbx[0] << 1.0, 0.0;
    qp->ubx[0] << 1.0, 0.0;
    for (int k = 0; k < N; ++k) {
        qp->C[k] = Matrix::Zero(ng, nx);
        qp->C[k](0, 0) = 1.0;
        qp->d[k] = Vector::Constant(ng, 0.5);
    }
    return qp;
}

// 构造带软约束的简单 OCP QP：普通约束 x <= 0.5 通过上界松弛允许违反
static std::unique_ptr<QPData> buildSoftConstraintQP(int N, int nx, int nu) {
    const int ng = 1;
    auto qp = std::make_unique<QPData>(N, nx, nu, ng);
    fillLqrDynamicsAndCost(qp.get(), N, nx, nu);
    qp->soft_config = std::make_unique<SoftConstraintConfig>();
    qp->soft_config->ns = 1;
    qp->soft_config->Zl = Vector::Constant(1, 0.0);
    qp->soft_config->Zu = Vector::Constant(1, 1e2);
    qp->soft_config->zl = Vector::Constant(1, 0.0);
    qp->soft_config->zu = Vector::Constant(1, 0.0);
    qp->soft_config->idxs = { 0 };
    // 初始状态固定为 [1, 0]，必然触发约束违反与松弛
    qp->lbx[0] << 1.0, 0.0;
    qp->ubx[0] << 1.0, 0.0;
    for (int k = 0; k < N; ++k) {
        qp->C[k] = Matrix::Zero(ng, nx);
        qp->C[k](0, 0) = 1.0;
        qp->d[k] = Vector::Constant(ng, 0.5);
    }
    return qp;
}

// 测试目的：验证软约束校验 helper 对非法 ng_max 返回 false
// 流程：构造合法 SoftConstraintConfig，但传入负的 ng_max
// 预期效果：soft_constraint_validation::validate 返回 false
TEST(QPSolvers, SoftConstraintValidatorRejectsNegativeNgMax) {
    SoftConstraintConfig cfg;
    cfg.ns = 1;
    cfg.Zl = Vector::Constant(1, 0.0);
    cfg.Zu = Vector::Constant(1, 1e2);
    cfg.zl = Vector::Constant(1, 0.0);
    cfg.zu = Vector::Constant(1, 0.0);
    cfg.idxs = { 0 };
    EXPECT_FALSE(soft_constraint_validation::validate(cfg, -1));
}

// 测试目的：验证 HPIPM 在无任何普通约束的 LQR 问题上能正常收敛
// 流程：构造仅含初始状态固定与动力学的 QP，调用 HPIPM
// 预期效果：返回 SUCCESS
TEST(QPSolvers, HpipmSolvesSimpleLqr) {
    const int N = 5, nx = 2, nu = 1, ng = 0, ns = 0;
    auto qp_data = buildLqrQP(N, nx, nu);
    HPIPMQPSolver solver(N, nx, nu, nx, nu, ng, ns, -1);
    solver.setTolerance(1e-8);
    QPSolution sol;
    const auto status = solver.solve(*qp_data, sol);
    EXPECT_EQ(status, QPSolverStatus::SUCCESS);
}

// 测试目的：验证 HPIPM 能处理普通硬约束（无软约束）
// 流程：构造含 x <= 0.5 硬约束的 QP，将初始状态设为可行点，调用 HPIPM
// 预期效果：返回 SUCCESS
TEST(QPSolvers, HpipmHandlesHardGeneralConstraints) {
    const int N = 5, nx = 2, nu = 1, ng = 1, ns = 0;
    auto qp_data = buildHardConstraintQP(N, nx, nu);
    // 将初始状态设为可行点，避免硬约束不可行导致 IPM 步长失败
    qp_data->lbx[0] = Vector::Zero(nx);
    qp_data->ubx[0] = Vector::Zero(nx);
    HPIPMQPSolver solver(N, nx, nu, nx, nu, ng, ns, -1);
    solver.setTolerance(1e-8);
    QPSolution sol;
    const auto status = solver.solve(*qp_data, sol);
    EXPECT_EQ(status, QPSolverStatus::SUCCESS);
}

// 测试目的：对比 HPIPM 与 Dense LDLT active-set 求解器在带软约束 QP 上的解
// 流程：构造同一 QPData，分别调用两种求解器，比较状态/控制轨迹
// 预期效果：两种求解器得到的状态与控制 L2 误差均小于 1e-8
TEST(QPSolvers, HpipmMatchesDenseLDLTWithSoftConstraints) {
    const int N = 10, nx = 2, nu = 1, ng = 1, ns = 1;
    auto qp_data = buildSoftConstraintQP(N, nx, nu);
    HPIPMQPSolver hpipm_solver(N, nx, nu, nx, nu, ng, ns, -1);
    hpipm_solver.setTolerance(1e-12);
    DenseQPSolver dense_solver;
    dense_solver.setTolerance(1e-12);
    QPSolution sol_hp, sol_dense;
    const auto status_hp = hpipm_solver.solve(*qp_data, sol_hp);
    const auto status_dense = dense_solver.solve(*qp_data, sol_dense);
    EXPECT_EQ(status_hp, QPSolverStatus::SUCCESS);
    EXPECT_EQ(status_dense, QPSolverStatus::SUCCESS);
    // 状态/控制轨迹对比：非零分量相对误差 < 1e-8，接近 0 分量绝对误差 < 1e-6
    for (int k = 0; k <= N; ++k) {
        EXPECT_TRUE(isClose(sol_hp.x[k], sol_dense.x[k], 1e-8, 1e-6))
            << "x mismatch at stage " << k << "\nhpipm: " << sol_hp.x[k].transpose()
            << "\ndense: " << sol_dense.x[k].transpose();
    }
    for (int k = 0; k < N; ++k) {
        EXPECT_TRUE(isClose(sol_hp.u[k], sol_dense.u[k], 1e-8, 1e-6))
            << "u mismatch at stage " << k << "\nhpipm: " << sol_hp.u[k].transpose()
            << "\ndense: " << sol_dense.u[k].transpose();
    }
    // 松弛变量对比：IPM 对非激活约束会留下约 1e-7 量级的互补残差，使用绝对误差 1e-6
    for (int k = 0; k < N; ++k) {
        EXPECT_LT((sol_hp.s[k] - sol_dense.s[k]).norm(), 1e-6)
            << "s mismatch at stage " << k << "\nhpipm: " << sol_hp.s[k].transpose()
            << "\ndense: " << sol_dense.s[k].transpose();
    }
}

// 构造多维软约束 QP：两个普通约束 x0 <= 0.5、x1 <= 0.5 均通过上界松弛允许违反
static std::unique_ptr<QPData> buildMultiSoftConstraintQP(int N, int nx, int nu) {
    const int ng = 2;
    auto qp = std::make_unique<QPData>(N, nx, nu, ng);
    fillLqrDynamicsAndCost(qp.get(), N, nx, nu);
    qp->soft_config = std::make_unique<SoftConstraintConfig>();
    qp->soft_config->ns = 2;
    qp->soft_config->Zl = Vector::Constant(2, 0.0);
    qp->soft_config->Zu = Vector::Constant(2, 1e2);
    qp->soft_config->zl = Vector::Constant(2, 0.0);
    qp->soft_config->zu = Vector::Constant(2, 0.0);
    qp->soft_config->idxs = { 0, 1 };
    // 初始状态固定为 [1, 0]，第一个约束必然触发松弛，第二个约束处于边界
    qp->lbx[0] << 1.0, 0.0;
    qp->ubx[0] << 1.0, 0.0;
    for (int k = 0; k < N; ++k) {
        qp->C[k] = Matrix::Zero(ng, nx);
        qp->C[k](0, 0) = 1.0;
        qp->C[k](1, 1) = 1.0;
        qp->d[k] = Vector::Constant(ng, 0.5);
    }
    return qp;
}

// 测试目的：验证多维软约束（ng=2, ns=2）下 HPIPM 与 Dense 解一致
// 流程：构造两个普通约束均软化的 QPData，分别调用两种求解器，比较轨迹与多个 slack
// 预期效果：状态/控制相对误差 < 1e-8，slack 绝对误差 < 1e-6
TEST(QPSolvers, HpipmMatchesDenseLDLTWithMultiDimensionalSoftConstraints) {
    const int N = 10, nx = 2, nu = 1, ng = 2, ns = 2;
    auto qp_data = buildMultiSoftConstraintQP(N, nx, nu);
    HPIPMQPSolver hpipm_solver(N, nx, nu, nx, nu, ng, ns, -1);
    hpipm_solver.setTolerance(1e-12);
    DenseQPSolver dense_solver;
    dense_solver.setTolerance(1e-12);
    QPSolution sol_hp, sol_dense;
    const auto status_hp = hpipm_solver.solve(*qp_data, sol_hp);
    const auto status_dense = dense_solver.solve(*qp_data, sol_dense);
    EXPECT_EQ(status_hp, QPSolverStatus::SUCCESS);
    EXPECT_EQ(status_dense, QPSolverStatus::SUCCESS);
    for (int k = 0; k <= N; ++k) {
        EXPECT_TRUE(isClose(sol_hp.x[k], sol_dense.x[k], 1e-8, 1e-6))
            << "x mismatch at stage " << k << "\nhpipm: " << sol_hp.x[k].transpose()
            << "\ndense: " << sol_dense.x[k].transpose();
    }
    for (int k = 0; k < N; ++k) {
        EXPECT_TRUE(isClose(sol_hp.u[k], sol_dense.u[k], 1e-8, 1e-6))
            << "u mismatch at stage " << k << "\nhpipm: " << sol_hp.u[k].transpose()
            << "\ndense: " << sol_dense.u[k].transpose();
    }
    for (int k = 0; k < N; ++k) {
        EXPECT_LT((sol_hp.s[k] - sol_dense.s[k]).norm(), 1e-6)
            << "s mismatch at stage " << k << "\nhpipm: " << sol_hp.s[k].transpose()
            << "\ndense: " << sol_dense.s[k].transpose();
    }
}

// 测试目的：验证当软约束索引越界时，HPIPM 与 Dense 求解器均返回 INVALID_ARGUMENT
// 流程：构造有效软约束 QP，将 idxs 改为 ng，分别调用两种求解器
// 预期效果：两者均拒绝非法配置
TEST(QPSolvers, BothSolversRejectOutOfBoundsSoftIdx) {
    const int N = 5, nx = 2, nu = 1, ng = 1, ns = 1;
    auto qp_data = buildSoftConstraintQP(N, nx, nu);
    qp_data->soft_config->idxs = { ng }; // 越界
    HPIPMQPSolver hpipm_solver(N, nx, nu, nx, nu, ng, ns, -1);
    DenseQPSolver dense_solver;
    QPSolution sol_hp, sol_dense;
    EXPECT_EQ(hpipm_solver.solve(*qp_data, sol_hp), QPSolverStatus::INVALID_ARGUMENT);
    EXPECT_EQ(dense_solver.solve(*qp_data, sol_dense), QPSolverStatus::INVALID_ARGUMENT);
}

// 测试目的：验证当软约束权重向量长度与 ns 不匹配时，两种求解器均返回 INVALID_ARGUMENT
// 流程：构造有效软约束 QP，将 Zu 长度改为 2，分别调用两种求解器
// 预期效果：两者均拒绝非法配置
TEST(QPSolvers, BothSolversRejectMismatchedSoftWeights) {
    const int N = 5, nx = 2, nu = 1, ng = 1, ns = 1;
    auto qp_data = buildSoftConstraintQP(N, nx, nu);
    qp_data->soft_config->Zu = Vector::Constant(2, 1e2); // 长度不匹配
    HPIPMQPSolver hpipm_solver(N, nx, nu, nx, nu, ng, ns, -1);
    DenseQPSolver dense_solver;
    QPSolution sol_hp, sol_dense;
    EXPECT_EQ(hpipm_solver.solve(*qp_data, sol_hp), QPSolverStatus::INVALID_ARGUMENT);
    EXPECT_EQ(dense_solver.solve(*qp_data, sol_dense), QPSolverStatus::INVALID_ARGUMENT);
}

// 测试目的：验证当软约束索引重复时，两种求解器均返回 INVALID_ARGUMENT
// 流程：构造 ng=2/ns=2 的合法软约束 QP，将 idxs 改为重复索引 {0, 0}，分别调用两种求解器
// 预期效果：两者均拒绝非法配置，避免 slack 映射被静默覆盖
TEST(QPSolvers, BothSolversRejectDuplicateSoftIdx) {
    const int N = 5, nx = 2, nu = 1, ng = 2, ns = 2;
    auto qp_data = buildMultiSoftConstraintQP(N, nx, nu);
    qp_data->soft_config->idxs = { 0, 0 }; // 重复索引
    HPIPMQPSolver hpipm_solver(N, nx, nu, nx, nu, ng, ns, -1);
    DenseQPSolver dense_solver;
    QPSolution sol_hp, sol_dense;
    EXPECT_EQ(hpipm_solver.solve(*qp_data, sol_hp), QPSolverStatus::INVALID_ARGUMENT);
    EXPECT_EQ(dense_solver.solve(*qp_data, sol_dense), QPSolverStatus::INVALID_ARGUMENT);
}

// 测试目的：验证当 ns == 0 但配置容器非空时，两种求解器均拒绝该配置
// 流程：将 soft_config->ns 改为 0，保持 idxs/Zu 非空，分别调用两种求解器
// 预期效果：两者均返回 INVALID_ARGUMENT，避免配置被静默忽略
TEST(QPSolvers, BothSolversRejectNsZeroWithNonEmptyConfig) {
    const int N = 5, nx = 2, nu = 1, ng = 1, ns = 1;
    auto qp_data = buildSoftConstraintQP(N, nx, nu);
    qp_data->soft_config->ns = 0; // ns=0 但 idxs/Zu 仍非空
    HPIPMQPSolver hpipm_solver(N, nx, nu, nx, nu, ng, ns, -1);
    DenseQPSolver dense_solver;
    QPSolution sol_hp, sol_dense;
    EXPECT_EQ(hpipm_solver.solve(*qp_data, sol_hp), QPSolverStatus::INVALID_ARGUMENT);
    EXPECT_EQ(dense_solver.solve(*qp_data, sol_dense), QPSolverStatus::INVALID_ARGUMENT);
}

// 测试目的：验证 HPIPM 拒绝软约束配置 ns 与求解器构造 ns 不一致的情况
// 流程：构造 ns=1 的 QPData，但用 ns=2 构造 HPIPMQPSolver
// 预期效果：返回 INVALID_ARGUMENT
TEST(QPSolvers, HpipmRejectsSoftConfigNsMismatch) {
    const int N = 5, nx = 2, nu = 1, ng = 1, ns_solver = 2, ns_data = 1;
    auto qp_data = buildSoftConstraintQP(N, nx, nu); // ns_data == 1
    HPIPMQPSolver solver(N, nx, nu, nx, nu, ng, ns_solver, -1);
    QPSolution sol;
    EXPECT_EQ(solver.solve(*qp_data, sol), QPSolverStatus::INVALID_ARGUMENT);
}

// 测试目的：验证 HPIPM 在构造 ns > 0 但 QPData 未提供软约束配置时拒绝求解
// 流程：用 buildHardConstraintQP 得到无 soft_config 的 QPData，调用 ns=1 的 HPIPM
// 预期效果：返回 INVALID_ARGUMENT
TEST(QPSolvers, HpipmRejectsMissingSoftConfig) {
    const int N = 5, nx = 2, nu = 1, ng = 1, ns = 1;
    auto qp_data = buildHardConstraintQP(N, nx, nu); // 无 soft_config
    HPIPMQPSolver solver(N, nx, nu, nx, nu, ng, ns, -1);
    QPSolution sol;
    EXPECT_EQ(solver.solve(*qp_data, sol), QPSolverStatus::INVALID_ARGUMENT);
}

// 测试目的：验证 HPIPM 在构造 ns == 0 但 QPData 携带 ns > 0 的软约束配置时拒绝求解
// 流程：用 buildSoftConstraintQP 得到 ns=1 的 QPData，但用 ns=0 构造 HPIPM
// 预期效果：返回 INVALID_ARGUMENT
TEST(QPSolvers, HpipmRejectsUnexpectedSoftConfig) {
    const int N = 5, nx = 2, nu = 1, ng = 1, ns = 0;
    auto qp_data = buildSoftConstraintQP(N, nx, nu); // ns_data == 1
    HPIPMQPSolver solver(N, nx, nu, nx, nu, ng, ns, -1);
    QPSolution sol;
    EXPECT_EQ(solver.solve(*qp_data, sol), QPSolverStatus::INVALID_ARGUMENT);
}

// 测试目的：验证 HPIPM 在构造 ns == 0 但 soft_config->ns == 0 且配置容器非空时拒绝求解
// 流程：用 buildSoftConstraintQP 得到容器非空的配置，将 ns 改为 0，用 ns=0 构造 HPIPM
// 预期效果：返回 INVALID_ARGUMENT，避免非法空维度配置被静默忽略
TEST(QPSolvers, HpipmRejectsNsZeroWithNonEmptyConfig) {
    const int N = 5, nx = 2, nu = 1, ng = 1, ns = 0;
    auto qp_data = buildSoftConstraintQP(N, nx, nu);
    qp_data->soft_config->ns = 0; // ns=0 但 idxs/Zu 仍非空
    HPIPMQPSolver solver(N, nx, nu, nx, nu, ng, ns, -1);
    QPSolution sol;
    EXPECT_EQ(solver.solve(*qp_data, sol), QPSolverStatus::INVALID_ARGUMENT);
}

// 测试目的：验证 HPIPM 在 QPData 维度与求解器构造维度不一致时返回 INVALID_ARGUMENT
// 流程：构造维度为 N=10 的 QPData，但用 N=5 构造 HPIPM
// 预期效果：返回 INVALID_ARGUMENT
TEST(QPSolvers, HpipmRejectsQPDataDimensionMismatch) {
    const int N = 10, nx = 2, nu = 1, ng = 1, ns = 1;
    auto qp_data = buildSoftConstraintQP(N, nx, nu);
    HPIPMQPSolver solver(5, nx, nu, nx, nu, ng, ns, -1);
    QPSolution sol;
    EXPECT_EQ(solver.solve(*qp_data, sol), QPSolverStatus::INVALID_ARGUMENT);
}

// 测试目的：验证 HPIPMQPSolver 构造函数对非法参数抛出 invalid_argument
// 流程：分别传入越界 nbx、负 ng、负 ns、非法 cond_N，期望构造函数抛出异常
// 预期效果：所有非法参数组合均触发 std::invalid_argument
TEST(QPSolvers, HpipmRejectsInvalidConstructorArguments) {
    EXPECT_THROW(HPIPMQPSolver(5, 2, 1, 3, 1, 0, 0, -1), std::invalid_argument); // nbx > nx
    EXPECT_THROW(HPIPMQPSolver(5, 2, 1, 2, 2, 0, 0, -1), std::invalid_argument); // nbu > nu
    EXPECT_THROW(HPIPMQPSolver(5, 2, 1, 2, 1, -1, 0, -1), std::invalid_argument); // ng < 0
    EXPECT_THROW(HPIPMQPSolver(5, 2, 1, 2, 1, 0, -1, -1), std::invalid_argument); // ns < 0
    EXPECT_THROW(HPIPMQPSolver(5, 2, 1, 2, 1, 0, 0, 0), std::invalid_argument);  // cond_N = 0 非法
    EXPECT_THROW(HPIPMQPSolver(5, 2, 1, 2, 1, 0, 0, 6), std::invalid_argument);  // cond_N > N 非法
    EXPECT_THROW(HPIPMQPSolver(5, 2, 1, 2, 1, 0, 0, -2), std::invalid_argument); // cond_N < -1 非法
    EXPECT_NO_THROW(HPIPMQPSolver(5, 2, 1, 2, 1, 0, 0, 3));
}

// 测试目的：验证 HPIPM setTolerance 只接受有限正数容差
// 流程：分别传入 0、负数、NaN、inf 和合法值，检查是否抛异常
// 预期效果：非法值抛 std::invalid_argument，合法值可正常设置
TEST(QPSolvers, HpipmRejectsInvalidTolerance) {
    HPIPMQPSolver solver(5, 2, 1, 2, 1, 0, 0, -1);
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

// 测试目的：验证 HPIPM 在 QPData 内部容器尺寸被破坏时返回 INVALID_ARGUMENT
// 流程：构造合法 QPData 后清空 A vector，再调用 HPIPM
// 预期效果：返回 INVALID_ARGUMENT，不触发越界访问
TEST(QPSolvers, HpipmRejectsInvalidQPDataContainers) {
    const int N = 5, nx = 2, nu = 1, ng = 1, ns = 1;
    auto qp_data = buildSoftConstraintQP(N, nx, nu);
    qp_data->A.clear();
    HPIPMQPSolver solver(N, nx, nu, nx, nu, ng, ns, -1);
    QPSolution sol;
    EXPECT_EQ(solver.solve(*qp_data, sol), QPSolverStatus::INVALID_ARGUMENT);
}

// 构造含非零 D（控制约束 u <= limit）的 QP
static std::unique_ptr<QPData> buildControlConstraintQP(int N, int nx, int nu,
    double limit) {
    const int ng = 1;
    auto qp = std::make_unique<QPData>(N, nx, nu, ng);
    fillLqrDynamicsAndCost(qp.get(), N, nx, nu);
    // 初始状态固定为 [1, 0]
    qp->lbx[0] << 1.0, 0.0;
    qp->ubx[0] << 1.0, 0.0;
    for (int k = 0; k < N; ++k) {
        // 普通约束：u(0) <= limit  =>  Cu * u <= limit, Cu = [1]
        qp->C[k] = Matrix::Zero(ng, nx);
        qp->D[k] = Matrix::Zero(ng, nu);
        qp->D[k](0, 0) = 1.0;
        qp->d[k] = Vector::Constant(ng, limit);
    }
    return qp;
}

// 测试目的：验证 HPIPM 与 Dense 在含非零 D 的控制约束 QP 上解一致
// 流程：构造 u <= -0.5 的控制约束 QP，分别调用 HPIPM 和 Dense，比较状态/控制轨迹
// 预期效果：两者均返回 SUCCESS，状态/控制相对误差 < 1e-8
TEST(QPSolvers, HpipmMatchesDenseLDLTWithNonZeroDControlConstraint) {
    const int N = 10, nx = 2, nu = 1, ng = 1, ns = 0;
    const double limit = -0.5;
    auto qp_data = buildControlConstraintQP(N, nx, nu, limit);
    HPIPMQPSolver hpipm_solver(N, nx, nu, nx, nu, ng, ns, -1);
    hpipm_solver.setTolerance(1e-12);
    DenseQPSolver dense_solver;
    dense_solver.setTolerance(1e-12);
    QPSolution sol_hp, sol_dense;
    const auto status_hp = hpipm_solver.solve(*qp_data, sol_hp);
    const auto status_dense = dense_solver.solve(*qp_data, sol_dense);
    EXPECT_EQ(status_hp, QPSolverStatus::SUCCESS);
    EXPECT_EQ(status_dense, QPSolverStatus::SUCCESS);
    for (int k = 0; k <= N; ++k) {
        EXPECT_TRUE(isClose(sol_hp.x[k], sol_dense.x[k], 1e-8, 1e-6))
            << "x mismatch at stage " << k;
    }
    for (int k = 0; k < N; ++k) {
        EXPECT_TRUE(isClose(sol_hp.u[k], sol_dense.u[k], 1e-8, 1e-6))
            << "u mismatch at stage " << k;
        // 额外验证控制约束确实生效
        EXPECT_LE(sol_hp.u[k](0), limit + 1e-6)
            << "HPIPM solution violates control constraint u <= " << limit;
        EXPECT_LE(sol_dense.u[k](0), limit + 1e-6)
            << "Dense solution violates control constraint u <= " << limit;
    }
}

// 测试目的：验证软约束（ns>0）与 Partial Condensing（0<cond_N<N）组合路径在 HPIPM 包装器中被正确处理。
// 流程：使用一维软约束 QP（ns=1, ng=1），HPIPM 请求 cond_N=5，Dense 使用无凝聚，
//      对比两种求解器的状态/控制/松弛变量。
// 注意：当前 HPIPM 版本对 ns>0 且 0<cond_N<N 的真正 Partial Condensing 会返回 MAX_ITER，
//       因此 HPIPMQPSolver 在构造期自动回退到无凝聚路径（与 cond_N=N 等价），
//       本用例验证的是该回退后的结果仍与 Dense 一致。
// 预期效果：状态/控制相对误差 < 1e-8，slack 绝对误差 < 1e-6。
TEST(QPSolvers, HpipmPartialCondensingMatchesDenseLDLTWithSoftConstraints) {
    const int N = 10, nx = 2, nu = 1, ng = 1, ns = 1;
    auto qp_data = buildSoftConstraintQP(N, nx, nu);
    HPIPMQPSolver hpipm_solver(N, nx, nu, nx, nu, ng, ns, /*cond_N=*/5);
    hpipm_solver.setTolerance(1e-12);
    DenseQPSolver dense_solver;
    dense_solver.setTolerance(1e-12);
    QPSolution sol_hp, sol_dense;
    const auto status_hp = hpipm_solver.solve(*qp_data, sol_hp);
    const auto status_dense = dense_solver.solve(*qp_data, sol_dense);
    ASSERT_EQ(status_hp, QPSolverStatus::SUCCESS);
    ASSERT_EQ(status_dense, QPSolverStatus::SUCCESS);
    for (int k = 0; k <= N; ++k) {
        EXPECT_TRUE(isClose(sol_hp.x[k], sol_dense.x[k], 1e-8, 1e-6))
            << "x mismatch at stage " << k << "\nhpipm: " << sol_hp.x[k].transpose()
            << "\ndense: " << sol_dense.x[k].transpose();
    }
    for (int k = 0; k < N; ++k) {
        EXPECT_TRUE(isClose(sol_hp.u[k], sol_dense.u[k], 1e-8, 1e-6))
            << "u mismatch at stage " << k << "\nhpipm: " << sol_hp.u[k].transpose()
            << "\ndense: " << sol_dense.u[k].transpose();
        EXPECT_LT((sol_hp.s[k] - sol_dense.s[k]).norm(), 1e-6)
            << "s mismatch at stage " << k << "\nhpipm: " << sol_hp.s[k].transpose()
            << "\ndense: " << sol_dense.s[k].transpose();
    }
}

// 构造一个单段 BicycleModelDelta 场景，用于锁定 HPIPM 默认容差在大量级线性化点下的数值问题。
// origin_offset 用于切换“全局坐标（量级约 -150）”与“局部坐标（以起点为原点）”。
static std::pair<MultiStageOCP, Trajectory> buildLargeMagnitudeSingleSegmentScenario(
    const Eigen::Vector3d& origin_offset)
{
    const int N = 25, nx = 5, nu = 2;
    const double dt = 0.1, L = 2.8, v0 = 1.0, delta0 = 0.05;
    auto dynamics = std::make_shared<BicycleModelDelta>(L);

    Trajectory init_guess;
    init_guess.resize(N, nx, nu);
    init_guess.x[0] << origin_offset(0), origin_offset(1), origin_offset(2), v0, delta0;
    for (int k = 0; k < N; ++k) {
        init_guess.u[k].setZero();
        dynamics->discretize(init_guess.x[k], init_guess.u[k], dt, 1.0, init_guess.x[k + 1]);
    }

    StageSegment seg;
    seg.dynamics = dynamics;
    seg.N = N;
    seg.dt = dt;
    seg.v_sign = 1.0;
    seg.x_min = Vector::Constant(nx, -1e4);
    seg.x_max = Vector::Constant(nx, 1e4);
    seg.x_min(3) = 0.0;
    seg.x_max(3) = 2.5;
    seg.x_min(4) = -0.5;
    seg.x_max(4) = 0.5;
    seg.u_min = Vector::Constant(nu, -3.0);
    seg.u_max = Vector::Constant(nu, 3.0);
    seg.cost = std::make_shared<QuadraticTrackingCost>(init_guess.x[N],
        Matrix::Identity(nx, nx) * 1e-2, Matrix::Identity(nu, nu) * 1e-2, /*theta_idx=*/2);
    seg.constraints.push_back(std::make_shared<ConvexCorridorConstraint>(nu));

    MultiStageOCP ocp;
    ocp.addSegment(seg);
    return { std::move(ocp), std::move(init_guess) };
}

//          HPIPM 默认容差（约 1e-8）会导致首次 QP 求解失败；
//          改为局部坐标并将 HPIPM 容差放宽到 1e-4（或使用 Dense 求解器）后可成功收敛。
// 流程：构造 BicycleModelDelta + ConvexCorridorConstraint 单段场景，分别用全局坐标默认 HPIPM、
//      局部坐标 1e-4 HPIPM、局部坐标 Dense 求解。
// 预期效果：全局坐标默认 HPIPM solve 返回 false；局部坐标 1e-4 HPIPM 与 Dense solve 返回 true。
TEST(QPSolvers, HpipmDefaultToleranceFailsForLargeMagnitudeLinearizationPoint) {
    SimpleParkingMap map; // 空地图：走廊约束全部退化为惰性 0<=0，只保留真实 ng 维度
    UpdaterConfig config;
    config.selection_radius = 200.0;
    config.max_step_displacement = 0.5;
    config.safety_margin = 0.1;
    config.top_k = 10;
    ProblemUpdater updater(config);

    // 全局坐标 + HPIPM 默认容差：应失败
    {
        auto [ocp, init_guess] = buildLargeMagnitudeSingleSegmentScenario(
            Eigen::Vector3d(-150.0, -10.0, 0.1));
        updater.updateOcp(init_guess, map, ocp);
        auto qp_solver = std::make_unique<HPIPMQPSolver>(
            ocp.totalSteps(), ocp.nx(), ocp.nu(), ocp.nx(), ocp.nu(), CORRIDOR_G_DIM, 0, -1);
        // 默认容差，不调用 setTolerance
        SQPSolver solver(std::move(qp_solver));
        solver.options().max_iter = 10;
        solver.options().use_line_search = false;
        Trajectory solution;
        EXPECT_FALSE(solver.solve(ocp, init_guess, solution))
            << "Expected HPIPM default tolerance to fail on large-magnitude linearization point";
    }

    // 局部坐标 + HPIPM 容差 1e-4：应成功
    {
        auto [ocp, init_guess] = buildLargeMagnitudeSingleSegmentScenario(
            Eigen::Vector3d(0.0, 0.0, 0.1));
        updater.updateOcp(init_guess, map, ocp);
        auto qp_solver = std::make_unique<HPIPMQPSolver>(
            ocp.totalSteps(), ocp.nx(), ocp.nu(), ocp.nx(), ocp.nu(), CORRIDOR_G_DIM, 0, -1);
        qp_solver->setTolerance(1e-4);
        SQPSolver solver(std::move(qp_solver));
        solver.options().max_iter = 10;
        solver.options().use_line_search = false;
        Trajectory solution;
        EXPECT_TRUE(solver.solve(ocp, init_guess, solution));
    }

    // 局部坐标 + Dense 求解器：应成功
    {
        auto [ocp, init_guess] = buildLargeMagnitudeSingleSegmentScenario(
            Eigen::Vector3d(0.0, 0.0, 0.1));
        updater.updateOcp(init_guess, map, ocp);
        SQPSolver solver(std::make_unique<DenseQPSolver>());
        solver.options().max_iter = 10;
        solver.options().use_line_search = false;
        Trajectory solution;
        EXPECT_TRUE(solver.solve(ocp, init_guess, solution));
    }
}
