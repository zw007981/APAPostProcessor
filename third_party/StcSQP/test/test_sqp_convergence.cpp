#include <cmath>
#include <memory>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "constraints/box_constraint.h"
#include "costs/cost_term.hpp"
#include "costs/quadratic_tracking.h"
#include "math/math_util.hpp"

using namespace stc_SQP;

// ===================== 测试辅助：仅含状态的末端二次代价 =====================
class TerminalQuadraticCost : public CostTerm {
public:
    TerminalQuadraticCost(const Vector& x_ref, const Matrix& Q)
        : x_ref_(x_ref)
        , Q_(Q)
    {
    }

    void evaluate(const Vector& x, const Vector& u, double& cost) const override
    {
        (void)u;
        const Vector dx = x - x_ref_;
        cost = 0.5 * dx.dot(Q_ * dx);
    }

    void gradient(const Vector& x, const Vector& u, Vector& q, Vector& r) const override
    {
        (void)u;
        q = Q_ * (x - x_ref_);
        r = Vector::Zero(0);
    }

    void hessian(const Vector& x, const Vector& u, Matrix& Q, Matrix& R, Matrix& S) const override
    {
        (void)x;
        (void)u;
        Q = Q_;
        R = Matrix::Zero(0, 0);
        S = Matrix::Zero(0, x_ref_.size());
    }

    std::shared_ptr<CostTerm> clone() const
    {
        return std::make_shared<TerminalQuadraticCost>(x_ref_, Q_);
    }

protected:
    Vector x_ref_;
    Matrix Q_;
};

// ===================== 测试辅助：DARE 迭代求解器 =====================
Matrix solveDare(const Matrix& A, const Matrix& B, const Matrix& Q, const Matrix& R)
{
    // 测试目的：通过离散代数 Riccati 方程迭代得到稳态反馈的 P
    // 流程：从 P0 = Q 开始迭代 Riccati 方程直到收敛
    // 预期效果：P 满足 P = Q + A^T P A - A^T P B (R + B^T P B)^{-1} B^T P A
    Matrix P = Q;
    const int max_iter = 1000;
    const double tol = 1e-12;
    for (int i = 0; i < max_iter; ++i) {
        const Matrix BT_P_B = B.transpose() * P * B;
        const Matrix P_next = Q + A.transpose() * P * A
            - A.transpose() * P * B * (R + BT_P_B).ldlt().solve(B.transpose() * P * A);
        if ((P_next - P).norm() < tol) {
            return P_next;
        }
        P = P_next;
    }
    return P;
}

// 它从 CostTerm 获取梯度与 Hessian，从 BoxConstraint 获取约束值与 Jacobian，
// 然后求解标准 LQR 问题：
//   min sum_{k=0}^{N-1} L_stage(x_k, u_k) + L_terminal(x_N)
//   s.t. x_0 = x0, x_{k+1} = A x_k + B u_k
//        x_min <= x_k <= x_max, u_min <= u_k <= u_max
// 对于二次代价与线性动力学，一次 Newton/SQP 迭代即可收敛；当 BoxConstraint
// 宽松不激活时，结果退化为无约束 LQR 解析解。
struct SimpleLQRSQP {
    int N = 0;
    int nx = 0;
    int nu = 0;

    std::vector<Vector> x; // N+1
    std::vector<Vector> u; // N

    struct ActiveConstraint {
        int stage; // 0..N，N 表示末端 x_N
        int row; // 在 BoxConstraint::jacobian 输出中的行索引
        int var_idx; // 在 z 向量中的列索引
        double coeff; // Jacobian 系数（来自 BoxConstraint::jacobian）
        bool is_state; // true 表示状态，false 表示控制
        double g_value; // 当前约束值 g_all(row)
    };

    bool solve(const Matrix& A, const Matrix& B, const Vector& x0, int steps,
        const CostTerm& stage_cost, const CostTerm& terminal_cost,
        const BoxConstraint* box = nullptr)
    {
        N = steps;
        nx = x0.size();
        nu = B.cols();

        // 初始猜测：全零
        x.assign(N + 1, Vector::Zero(nx));
        u.assign(N, Vector::Zero(nu));
        x[0] = x0;

        // 变量 z = [x_0; u_0; x_1; u_1; ...; x_{N-1}; u_{N-1}; x_N]
        const int nvar = (N + 1) * nx + N * nu;
        const int neq = (N + 1) * nx; // x0 + N*dynamics

        Matrix H = Matrix::Zero(nvar, nvar);
        Vector g = Vector::Zero(nvar);

        // 通过 stage_cost 接口装配 stage Hessian 与 gradient
        Vector q(nx), r(nu);
        Matrix Q(nx, nx), R(nu, nu), S(nu, nx);
        for (int k = 0; k < N; ++k) {
            stage_cost.hessian(x[k], u[k], Q, R, S);
            stage_cost.gradient(x[k], u[k], q, r);
            const int x_off = k * (nx + nu);
            const int u_off = x_off + nx;
            H.block(x_off, x_off, nx, nx) = Q;
            H.block(x_off, u_off, nx, nu) = S.transpose();
            H.block(u_off, x_off, nu, nx) = S;
            H.block(u_off, u_off, nu, nu) = R;
            g.segment(x_off, nx) = q;
            g.segment(u_off, nu) = r;
        }

        // 通过 terminal_cost 接口装配末端 Hessian 与 gradient
        Vector qN(nx), r_term(0);
        Matrix QN(nx, nx), RN(0, 0), SN(0, nx);
        Vector empty_u(0);
        terminal_cost.hessian(x[N], empty_u, QN, RN, SN);
        terminal_cost.gradient(x[N], empty_u, qN, r_term);
        const int xN_offset = N * (nx + nu);
        H.block(xN_offset, xN_offset, nx, nx) = QN;
        g.segment(xN_offset, nx) = qN;

        // 等式约束：初始条件 + 动力学
        Matrix Aeq = Matrix::Zero(neq, nvar);
        Vector ceq = Vector::Zero(neq);
        Aeq.block(0, 0, nx, nx) = Matrix::Identity(nx, nx);
        ceq.head(nx) = x[0] - x0;
        for (int k = 0; k < N; ++k) {
            const int row = nx + k * nx;
            const int x_off = k * (nx + nu);
            const int u_off = x_off + nx;
            const int x_next_off = (k + 1) * (nx + nu);

            Aeq.block(row, x_next_off, nx, nx) = Matrix::Identity(nx, nx);
            Aeq.block(row, x_off, nx, nx) = -A;
            Aeq.block(row, u_off, nx, nu) = -B;

            ceq.segment(row, nx) = x[k + 1] - A * x[k] - B * u[k];
        }

        // active set 循环处理 BoxConstraint
        std::vector<ActiveConstraint> active_set;
        const Vector zero_u = Vector::Zero(nu);
        const int max_active_set_iter = 20;

        for (int as_iter = 0; as_iter < max_active_set_iter; ++as_iter) {
            // 每次 active set 迭代前重新计算等式约束残差（更新后的 x/u 可能改变残差）
            ceq.head(nx) = x[0] - x0;
            for (int k = 0; k < N; ++k) {
                const int row = nx + k * nx;
                ceq.segment(row, nx) = x[k + 1] - A * x[k] - B * u[k];
            }
            const int n_active = static_cast<int>(active_set.size());
            const int n_total_eq = neq + n_active;

            Matrix KKT = Matrix::Zero(nvar + n_total_eq, nvar + n_total_eq);
            Vector rhs = Vector::Zero(nvar + n_total_eq);

            KKT.topLeftCorner(nvar, nvar) = H;
            KKT.block(0, nvar, nvar, neq) = Aeq.transpose();
            KKT.block(nvar, 0, neq, nvar) = Aeq;
            rhs.head(nvar) = -g;
            rhs.segment(nvar, neq) = -ceq;

            // 加入当前 active box constraints 作为等式
            Matrix C_active = Matrix::Zero(n_active, nvar);
            Vector g_active = Vector::Zero(n_active);
            if (n_active > 0) {
                // active_set 非空意味着 box 必须非空；此处做防御性断言以消除编译器 null 警告
                if (box == nullptr) {
                    return false;
                }
                for (int a = 0; a < n_active; ++a) {
                    const ActiveConstraint& ac = active_set[a];
                    C_active(a, ac.var_idx) = ac.coeff;
                    // 每轮根据当前 x/u 重新计算 active 约束值，避免多轮迭代中残差漂移
                    const Vector& uk_active = (ac.stage < N) ? u[ac.stage] : zero_u;
                    Vector g_all_active;
                    box->evaluate(x[ac.stage], uk_active, Vector(), g_all_active);
                    g_active(a) = g_all_active(ac.row);
                }
                KKT.block(0, nvar + neq, nvar, n_active) = C_active.transpose();
                KKT.block(nvar + neq, 0, n_active, nvar) = C_active;
                rhs.segment(nvar + neq, n_active) = -g_active;
            }

            const Vector sol = KKT.ldlt().solve(rhs);
            const Vector dz = sol.head(nvar);

            // 应用 Newton 步长（alpha = 1，因为问题完全是二次的）
            for (int k = 0; k <= N; ++k) {
                const int x_off = k * (nx + nu);
                x[k] += dz.segment(x_off, nx);
            }
            for (int k = 0; k < N; ++k) {
                const int u_off = k * (nx + nu) + nx;
                u[k] += dz.segment(u_off, nu);
            }

            if (box == nullptr) {
                break;
            }

            // 检查所有 box constraints 是否有违反，通过 BoxConstraint 接口取值与 Jacobian
            bool added = false;
            Vector g_all;
            Matrix Cx, Cu;
            for (int k = 0; k <= N && !added; ++k) {
                const Vector& uk = (k < N) ? u[k] : zero_u;
                box->evaluate(x[k], uk, Vector(), g_all);
                box->jacobian(x[k], uk, Vector(), Cx, Cu);

                // 末端 x_N 没有对应的 u 变量，因此只检查状态相关的约束行
                const int max_row = (k < N) ? g_all.size() : 2 * nx;

                for (int i = 0; i < max_row; ++i) {
                    if (g_all(i) > 1e-9) {
                        ActiveConstraint ac;
                        ac.stage = k;
                        ac.row = i;
                        ac.g_value = g_all(i);
                        ac.is_state = (i < 2 * nx);
                        // x0 由初始条件固定，不施加 box constraint
                        if (ac.is_state && ac.stage == 0) {
                            continue;
                        }
                        // 从 BoxConstraint::jacobian 的 Cx/Cu 中提取非零系数与局部列索引
                        if (ac.is_state) {
                            for (int j = 0; j < nx; ++j) {
                                if (std::abs(Cx(i, j)) > 1e-12) {
                                    ac.var_idx = k * (nx + nu) + j;
                                    ac.coeff = Cx(i, j);
                                    break;
                                }
                            }
                        } else {
                            for (int j = 0; j < nu; ++j) {
                                if (std::abs(Cu(i, j)) > 1e-12) {
                                    ac.var_idx = k * (nx + nu) + nx + j;
                                    ac.coeff = Cu(i, j);
                                    break;
                                }
                            }
                        }
                        bool exists = false;
                        for (const auto& existing : active_set) {
                            if (existing.stage == ac.stage && existing.row == ac.row) {
                                exists = true;
                                break;
                            }
                        }
                        if (!exists) {
                            active_set.push_back(ac);
                            added = true;
                        }
                    }
                }
            }

            if (!added) {
                // 检查 active constraints 的乘子是否非负
                bool removed = false;
                if (n_active > 0) {
                    Vector mu = sol.segment(nvar + neq, n_active);
                    for (int a = n_active - 1; a >= 0; --a) {
                        if (mu(a) < -1e-9) {
                            active_set.erase(active_set.begin() + a);
                            removed = true;
                        }
                    }
                }
                if (!removed) {
                    break; // active set 收敛
                }
            }
        }

        return true;
    }
};

// ===================== 测试用例 =====================

TEST(QuadraticTrackingCost, ComputesCostGradientAndHessianCorrectly)
{
    // 测试目的：验证 QuadraticTrackingCost 的标量代价、梯度、Hessian 计算正确
    // 流程：构造一个带航向角的参考点，分别调用 evaluate/gradient/hessian 与手算结果比较
    // 预期效果：代价、梯度、Hessian 与解析表达式一致，角度误差使用 SO2 差值
    const int nx = 3;
    const int nu = 2;
    Vector x_ref(nx);
    x_ref << 1.0, 2.0, PI - 0.1;
    Matrix Q = Matrix::Identity(nx, nx);
    Matrix R = Matrix::Identity(nu, nu);

    QuadraticTrackingCost cost(x_ref, Q, R, /*theta_idx=*/2);

    Vector x(nx), u(nu);
    x << 1.2, 1.8, -PI + 0.2; // 角度跨过 π/-π 边界
    u << 0.5, -0.3;

    double value = 0.0;
    cost.evaluate(x, u, value);

    // dx = [0.2, -0.2, NormalizeAngle((-pi+0.2) - (pi-0.1))]
    //    = [0.2, -0.2, NormalizeAngle(-2*pi+0.3)] = [0.2, -0.2, 0.3]
    const double expected_dtheta = math_util::NormalizeAngle(x(2) - x_ref(2));
    const double expected_cost = 0.5 * (0.04 + 0.04 + expected_dtheta * expected_dtheta + 0.25 + 0.09);
    EXPECT_NEAR(value, expected_cost, 1e-12);

    Vector q(nx), r(nu);
    cost.gradient(x, u, q, r);
    EXPECT_NEAR(q(0), 0.2, 1e-12);
    EXPECT_NEAR(q(1), -0.2, 1e-12);
    EXPECT_NEAR(q(2), expected_dtheta, 1e-12);
    EXPECT_NEAR(r(0), 0.5, 1e-12);
    EXPECT_NEAR(r(1), -0.3, 1e-12);

    Matrix Q_hess(nx, nx), R_hess(nu, nu), S(nu, nx);
    cost.hessian(x, u, Q_hess, R_hess, S);
    EXPECT_TRUE(Q_hess.isIdentity(1e-12));
    EXPECT_TRUE(R_hess.isIdentity(1e-12));
    EXPECT_TRUE(S.isZero(1e-12));
}

TEST(QuadraticTrackingCost, DirectAngleDifferenceWouldFail)
{
    // 测试目的：验证使用 SO2 差值与直接欧氏相减在跨过 π/-π 边界时结果不同
    // 流程：构造角度差跨过边界的两个状态，比较 SO2 误差与直接相减
    // 预期效果：SO2 误差给出最短角度差，直接相减会得到绝对值更大的错误结果
    const int nx = 1;
    const int nu = 1;
    Vector x_ref(nx);
    x_ref << PI - 0.1;
    Matrix Q = Matrix::Identity(nx, nx);
    Matrix R = Matrix::Identity(nu, nu);
    QuadraticTrackingCost cost(x_ref, Q, R, /*theta_idx=*/0);

    Vector x(nx), u(nu);
    x << -PI + 0.2;
    u << 0.0;

    double value = 0.0;
    cost.evaluate(x, u, value);

    const double so2_error = math_util::NormalizeAngle(x(0) - x_ref(0));
    const double direct_error = x(0) - x_ref(0);
    const double expected_cost = 0.5 * so2_error * so2_error;

    EXPECT_NEAR(value, expected_cost, 1e-12);
    EXPECT_LT(std::abs(so2_error), std::abs(direct_error));
    EXPECT_NEAR(so2_error, 0.3, 1e-12);
}

TEST(QuadraticTrackingCost, RejectsInvalidDimensionsAndThetaIndex)
{
    // 测试目的：验证 QuadraticTrackingCost 构造函数对非法维度与非法 theta_idx 抛出异常
    // 流程：分别构造 Q 维度错误、R 非方阵、theta_idx 越界三种非法输入，检查是否抛出 std::invalid_argument
    // 预期效果：所有非法构造均抛出 std::invalid_argument
    const int nx = 2;
    const int nu = 1;
    Vector x_ref(nx);
    x_ref << 0.0, 0.0;
    Matrix Q_ok = Matrix::Identity(nx, nx);
    Matrix R_ok = Matrix::Identity(nu, nu);

    Matrix Q_bad(nx + 1, nx);
    Q_bad.setIdentity();
    EXPECT_THROW(QuadraticTrackingCost(x_ref, Q_bad, R_ok), std::invalid_argument);

    Matrix R_bad(nu, nu + 1);
    R_bad.setIdentity();
    EXPECT_THROW(QuadraticTrackingCost(x_ref, Q_ok, R_bad), std::invalid_argument);

    EXPECT_THROW(QuadraticTrackingCost(x_ref, Q_ok, R_ok, /*theta_idx=*/nx), std::invalid_argument);
    EXPECT_THROW(QuadraticTrackingCost(x_ref, Q_ok, R_ok, /*theta_idx=*/-2), std::invalid_argument);
}

TEST(QuadraticTrackingCost, RejectsNonSymmetricWeightMatrices)
{
    // 测试目的：验证 QuadraticTrackingCost 构造函数对非对称 Q/R 抛出异常
    // 流程：构造非对称的 Q 和 R，检查是否抛出 std::invalid_argument
    // 预期效果：所有非法构造均抛出 std::invalid_argument
    const int nx = 2;
    const int nu = 2;
    Vector x_ref(nx);
    x_ref << 0.0, 0.0;
    Matrix Q_ok = Matrix::Identity(nx, nx);
    Matrix R_ok = Matrix::Identity(nu, nu);

    Matrix Q_bad(nx, nx);
    Q_bad << 1.0, 2.0,
        3.0, 4.0;
    EXPECT_THROW(QuadraticTrackingCost(x_ref, Q_bad, R_ok), std::invalid_argument);

    Matrix R_bad(nu, nu);
    R_bad << 1.0, 2.0,
        3.0, 4.0;
    EXPECT_THROW(QuadraticTrackingCost(x_ref, Q_ok, R_bad), std::invalid_argument);
}

TEST(BoxConstraint, EvaluatesAndLinearizesCorrectly)
{
    // 测试目的：验证 BoxConstraint 的约束值与 Jacobian 符合 g(x,u) <= 0 的形式
    // 流程：构造上下界，在边界内部与外部点分别调用 evaluate/jacobian，检查符号与维度
    // 预期效果：g 的前 nx 项为 x_min - x，接下来为 x - x_max；Jacobian 为 [-I; I; 0; 0] 与 [0; 0; -I; I]
    const int nx = 2;
    const int nu = 1;
    Vector x_min(nx), x_max(nx), u_min(nu), u_max(nu);
    x_min << -1.0, -2.0;
    x_max << 1.0, 2.0;
    u_min << -0.5;
    u_max << 0.5;

    BoxConstraint box(x_min, x_max, u_min, u_max);
    EXPECT_EQ(box.ng(), 2 * nx + 2 * nu);

    Vector x(nx), u(nu);
    x << 0.0, 0.0;
    u << 0.0;

    Vector g;
    box.evaluate(x, u, Vector(), g);
    EXPECT_EQ(g.size(), box.ng());
    EXPECT_NEAR(g(0), -1.0, 1e-12); // x_min(0) - x(0)
    EXPECT_NEAR(g(1), -2.0, 1e-12); // x_min(1) - x(1)
    EXPECT_NEAR(g(2), -1.0, 1e-12); // x(0) - x_max(0)
    EXPECT_NEAR(g(3), -2.0, 1e-12); // x(1) - x_max(1)
    EXPECT_NEAR(g(4), -0.5, 1e-12); // u_min - u
    EXPECT_NEAR(g(5), -0.5, 1e-12); // u - u_max

    Matrix Cx, Cu;
    box.jacobian(x, u, Vector(), Cx, Cu);
    EXPECT_EQ(Cx.rows(), box.ng());
    EXPECT_EQ(Cx.cols(), nx);
    EXPECT_EQ(Cu.rows(), box.ng());
    EXPECT_EQ(Cu.cols(), nu);

    EXPECT_NEAR(Cx(0, 0), -1.0, 1e-12);
    EXPECT_NEAR(Cx(1, 1), -1.0, 1e-12);
    EXPECT_NEAR(Cx(2, 0), 1.0, 1e-12);
    EXPECT_NEAR(Cx(3, 1), 1.0, 1e-12);
    EXPECT_TRUE(Cx.bottomRows(2).isZero(1e-12));

    EXPECT_NEAR(Cu(4, 0), -1.0, 1e-12);
    EXPECT_NEAR(Cu(5, 0), 1.0, 1e-12);
    EXPECT_TRUE(Cu.topRows(4).isZero(1e-12));
}

TEST(BoxConstraint, RejectsMismatchedDimensions)
{
    // 测试目的：验证 BoxConstraint 构造函数对维度不一致的上下界抛出异常
    // 流程：分别构造 x_min/x_max 维度不一致、u_min/u_max 维度不一致的输入，检查是否抛出 std::invalid_argument
    // 预期效果：所有非法构造均抛出 std::invalid_argument
    Vector x_min(2), x_max(2), u_min(1), u_max(1);
    x_min << -1.0, -1.0;
    x_max << 1.0, 1.0;
    u_min << -0.5;
    u_max << 0.5;

    Vector x_min_bad(2), x_max_bad(3);
    x_min_bad << -1.0, -1.0;
    x_max_bad << 1.0, 1.0, 1.0;
    EXPECT_THROW(BoxConstraint(x_min_bad, x_max_bad, u_min, u_max), std::invalid_argument);

    Vector u_min_bad(1), u_max_bad(2);
    u_min_bad << -0.5;
    u_max_bad << 0.5, 0.5;
    EXPECT_THROW(BoxConstraint(x_min, x_max, u_min_bad, u_max_bad), std::invalid_argument);
}

TEST(SQPConvergence, UnconstrainedLQRConvergesInOneIteration)
{
    // 测试目的：验证通过 CostTerm 接口装配的 SQP 引擎在纯 LQR 问题上能一次迭代收敛到解析解
    // 流程：
    //   1. 构造双积分器离散系统 (A, B) 与权重 (Q, R)
    //   2. 通过 DARE 得到稳态反馈增益 K 与 Riccati 矩阵 P
    //   3. 使用 QuadraticTrackingCost 作为 stage cost，TerminalQuadraticCost 作为 terminal cost
    //   4. 用 SimpleLQRSQP 求解并比较解析反馈轨迹 u_k = -K x_k
    // 预期效果：一次迭代后状态与控制的 L2 误差小于 1e-10
    const int nx = 2;
    const int nu = 1;
    const int N = 20;

    const double dt = 0.1;
    Matrix A(nx, nx);
    A << 1.0, dt,
        0.0, 1.0;
    Matrix B(nx, nu);
    B << 0.0,
        dt;

    Matrix Q = Matrix::Identity(nx, nx);
    Matrix R = Matrix::Identity(nu, nu) * 0.1;

    const Matrix P = solveDare(A, B, Q, R);
    const Matrix K = (R + B.transpose() * P * B).ldlt().solve(B.transpose() * P * A);

    Vector x0(nx);
    x0 << 1.0, 0.0;

    Vector x_ref = Vector::Zero(nx);
    QuadraticTrackingCost stage_cost(x_ref, Q, R);
    TerminalQuadraticCost terminal_cost(x_ref, P);

    SimpleLQRSQP solver;
    const bool ok = solver.solve(A, B, x0, N, stage_cost, terminal_cost);
    ASSERT_TRUE(ok);

    // 解析解：稳态 LQR 反馈 u = -K x，状态按 x_{k+1} = (A - BK) x_k 演化
    Vector x_analytic = x0;
    const double tol = 1e-10;
    for (int k = 0; k < N; ++k) {
        const Vector u_analytic = -K * x_analytic;
        EXPECT_TRUE(solver.x[k].isApprox(x_analytic, tol));
        EXPECT_TRUE(solver.u[k].isApprox(u_analytic, tol));
        x_analytic = A * x_analytic + B * u_analytic;
    }
    EXPECT_TRUE(solver.x[N].isApprox(x_analytic, tol));
}

TEST(SQPConvergence, LQRWithInactiveBoxConstraintsConvergesInOneIteration)
{
    // 测试目的：验证通过 CostTerm + BoxConstraint 接口装配的 SQP 引擎在 BoxConstraint 宽松不激活时仍能一次收敛
    // 流程：构造与上一条测试相同的 LQR 问题并加入宽松 BoxConstraint，调用带 box 的 SQP 求解器
    // 预期效果：所有 box 约束 inactive，最优解与无约束解析解一致
    const int nx = 2;
    const int nu = 1;
    const int N = 20;

    const double dt = 0.1;
    Matrix A(nx, nx);
    A << 1.0, dt,
        0.0, 1.0;
    Matrix B(nx, nu);
    B << 0.0,
        dt;

    Matrix Q = Matrix::Identity(nx, nx);
    Matrix R = Matrix::Identity(nu, nu) * 0.1;
    const Matrix P = solveDare(A, B, Q, R);
    const Matrix K = (R + B.transpose() * P * B).ldlt().solve(B.transpose() * P * A);

    Vector x0(nx);
    x0 << 1.0, 0.0;

    Vector x_ref = Vector::Zero(nx);
    QuadraticTrackingCost stage_cost(x_ref, Q, R);
    TerminalQuadraticCost terminal_cost(x_ref, P);

    // 宽松 box：足够大以确保 inactive
    Vector x_min(nx), x_max(nx), u_min(nu), u_max(nu);
    x_min << -10.0, -10.0;
    x_max << 10.0, 10.0;
    u_min << -10.0;
    u_max << 10.0;
    BoxConstraint box(x_min, x_max, u_min, u_max);

    SimpleLQRSQP solver;
    const bool ok = solver.solve(A, B, x0, N, stage_cost, terminal_cost, &box);
    ASSERT_TRUE(ok);

    // 检查所有 box constraints inactive（使用 BoxConstraint 接口）
    Vector g;
    const Vector zero_u = Vector::Zero(nu);
    for (int k = 0; k <= N; ++k) {
        const Vector& uk = (k < N) ? solver.u[k] : zero_u;
        box.evaluate(solver.x[k], uk, Vector(), g);
        for (int i = 0; i < g.size(); ++i) {
            EXPECT_LT(g(i), -1e-6) << "Box constraint should be inactive at k=" << k << ", i=" << i;
        }
    }

    // 与解析解一致
    Vector x_analytic = x0;
    const double tol = 1e-10;
    for (int k = 0; k < N; ++k) {
        const Vector u_analytic = -K * x_analytic;
        EXPECT_TRUE(solver.x[k].isApprox(x_analytic, tol));
        EXPECT_TRUE(solver.u[k].isApprox(u_analytic, tol));
        x_analytic = A * x_analytic + B * u_analytic;
    }
    EXPECT_TRUE(solver.x[N].isApprox(x_analytic, tol));
}

TEST(QuadraticTrackingCost, RejectsRuntimeMismatchedInputDimensions)
{
    // 测试目的：验证 QuadraticTrackingCost 在运行期对维度不匹配的 x 或 u 抛出异常
    // 流程：构造合法的 QuadraticTrackingCost，然后分别传入错误维度的 x 和 u 调用 evaluate/gradient/hessian
    // 预期效果：所有调用均抛出 std::invalid_argument
    const int nx = 2;
    const int nu = 1;
    Vector x_ref(nx);
    x_ref << 0.0, 0.0;
    Matrix Q = Matrix::Identity(nx, nx);
    Matrix R = Matrix::Identity(nu, nu);
    QuadraticTrackingCost cost(x_ref, Q, R);

    Vector x_bad(nx + 1);
    x_bad << 0.0, 0.0, 0.0;
    Vector u_bad(nu + 1);
    u_bad << 0.0, 0.0;
    Vector x_ok(nx);
    x_ok << 0.0, 0.0;
    Vector u_ok(nu);
    u_ok << 0.0;

    double value = 0.0;
    Vector q(nx), r(nu);
    Matrix Q_out(nx, nx), R_out(nu, nu), S(nu, nx);

    EXPECT_THROW(cost.evaluate(x_bad, u_ok, value), std::invalid_argument);
    EXPECT_THROW(cost.evaluate(x_ok, u_bad, value), std::invalid_argument);
    EXPECT_THROW(cost.gradient(x_bad, u_ok, q, r), std::invalid_argument);
    EXPECT_THROW(cost.gradient(x_ok, u_bad, q, r), std::invalid_argument);
    EXPECT_THROW(cost.hessian(x_bad, u_ok, Q_out, R_out, S), std::invalid_argument);
    EXPECT_THROW(cost.hessian(x_ok, u_bad, Q_out, R_out, S), std::invalid_argument);
}

TEST(BoxConstraint, RejectsRuntimeMismatchedInputDimensions)
{
    // 测试目的：验证 BoxConstraint 在运行期对维度不匹配的 x 或 u 抛出异常
    // 流程：构造合法的 BoxConstraint，然后分别传入错误维度的 x 和 u 调用 evaluate/jacobian
    // 预期效果：所有调用均抛出 std::invalid_argument
    const int nx = 2;
    const int nu = 1;
    Vector x_min(nx), x_max(nx), u_min(nu), u_max(nu);
    x_min << -1.0, -1.0;
    x_max << 1.0, 1.0;
    u_min << -0.5;
    u_max << 0.5;
    BoxConstraint box(x_min, x_max, u_min, u_max);

    Vector x_bad(nx + 1);
    x_bad << 0.0, 0.0, 0.0;
    Vector u_bad(nu + 1);
    u_bad << 0.0, 0.0;
    Vector x_ok(nx);
    x_ok << 0.0, 0.0;
    Vector u_ok(nu);
    u_ok << 0.0;

    Vector g;
    Matrix Cx, Cu;
    EXPECT_THROW(box.evaluate(x_bad, u_ok, Vector(), g), std::invalid_argument);
    EXPECT_THROW(box.evaluate(x_ok, u_bad, Vector(), g), std::invalid_argument);
    EXPECT_THROW(box.jacobian(x_bad, u_ok, Vector(), Cx, Cu), std::invalid_argument);
    EXPECT_THROW(box.jacobian(x_ok, u_bad, Vector(), Cx, Cu), std::invalid_argument);
}

TEST(BoxConstraint, RejectsInvertedBounds)
{
    // 测试目的：验证 BoxConstraint 对下界大于上界的非法语义抛出异常
    // 流程：分别构造 x_min > x_max 和 u_min > u_max 的输入，检查是否抛出 std::invalid_argument
    // 预期效果：所有非法构造均抛出 std::invalid_argument
    Vector x_min(1), x_max(1), u_min(1), u_max(1);
    x_min << 1.0;
    x_max << -1.0;
    u_min << -0.5;
    u_max << 0.5;
    EXPECT_THROW(BoxConstraint(x_min, x_max, u_min, u_max), std::invalid_argument);

    x_min << -1.0;
    x_max << 1.0;
    u_min << 1.0;
    u_max << -1.0;
    EXPECT_THROW(BoxConstraint(x_min, x_max, u_min, u_max), std::invalid_argument);
}

TEST(SQPConvergence, ScalarLQRControlLowerBoundActivates)
{
    // 测试目的：验证 BoxConstraint 控制下界激活时，active-set 装配路径（含 jacobian）正确工作
    // 流程：
    //   1. 构造标量系统 A=B=1, Q=1, R=0.1，通过 DARE 得到 P 与稳态反馈增益 K
    //   2. 计算无约束最优控制 u_unc = -K * x0
    //   3. 设置控制下界 u_min = 0.5 * u_unc（高于 u_unc，使其激活）
    //   4. 用 SimpleLQRSQP 求解，验证控制被 clip 到 u_min
    // 预期效果：solver.u[0] 等于 u_min，对应 box 约束 g(2) = 0，其余 box 约束 inactive
    const int nx = 1;
    const int nu = 1;
    const int N = 1;

    Matrix A(nx, nx);
    A << 1.0;
    Matrix B(nx, nu);
    B << 1.0;
    Matrix Q(nx, nx);
    Q << 1.0;
    Matrix R(nu, nu);
    R << 0.1;

    const Matrix P = solveDare(A, B, Q, R);
    const Matrix K = (R + B.transpose() * P * B).ldlt().solve(B.transpose() * P * A);

    Vector x0(nx);
    x0 << 1.0;
    const double u_unc = (-K * x0)(0);
    ASSERT_LT(u_unc, 0.0) << "Unconstrained optimal control should be negative";
    const double u_min_val = 0.5 * u_unc; // 高于 u_unc，使下界激活

    Vector x_ref = Vector::Zero(nx);
    QuadraticTrackingCost stage_cost(x_ref, Q, R);
    TerminalQuadraticCost terminal_cost(x_ref, P);

    Vector x_min(nx), x_max(nx), u_min(nu), u_max(nu);
    x_min << -10.0;
    x_max << 10.0;
    u_min << u_min_val;
    u_max << 10.0;
    BoxConstraint box(x_min, x_max, u_min, u_max);

    SimpleLQRSQP solver;
    const bool ok = solver.solve(A, B, x0, N, stage_cost, terminal_cost, &box);
    ASSERT_TRUE(ok);

    EXPECT_NEAR(solver.u[0](0), u_min_val, 1e-8);
    EXPECT_NEAR(solver.x[1](0), x0(0) + u_min_val, 1e-8);

    Vector g;
    box.evaluate(solver.x[0], solver.u[0], Vector(), g);
    EXPECT_NEAR(g(2), 0.0, 1e-8); // u_min - u = 0，下界激活
    EXPECT_LT(g(3), -1e-6); // u - u_max < 0，上界 inactive
}

TEST(SQPConvergence, ScalarLQRControlUpperBoundActivates)
{
    // 测试目的：验证 BoxConstraint 控制上界激活时，active-set 装配路径（含 jacobian）正确工作
    // 流程：与下界测试对称，设置控制上界 u_max 低于无约束最优控制，使其激活
    // 预期效果：solver.u[0] 等于 u_max，对应 box 约束 g(3) = 0，其余 box 约束 inactive
    const int nx = 1;
    const int nu = 1;
    const int N = 1;

    Matrix A(nx, nx);
    A << 1.0;
    Matrix B(nx, nu);
    B << 1.0;
    Matrix Q(nx, nx);
    Q << 1.0;
    Matrix R(nu, nu);
    R << 0.1;

    const Matrix P = solveDare(A, B, Q, R);
    const Matrix K = (R + B.transpose() * P * B).ldlt().solve(B.transpose() * P * A);

    Vector x0(nx);
    x0 << 1.0;
    const double u_unc = (-K * x0)(0);
    ASSERT_LT(u_unc, 0.0) << "Unconstrained optimal control should be negative";
    const double u_max_val = 1.5 * u_unc; // 低于 u_unc（更负），使上界激活

    Vector x_ref = Vector::Zero(nx);
    QuadraticTrackingCost stage_cost(x_ref, Q, R);
    TerminalQuadraticCost terminal_cost(x_ref, P);

    Vector x_min(nx), x_max(nx), u_min(nu), u_max(nu);
    x_min << -10.0;
    x_max << 10.0;
    u_min << -10.0;
    u_max << u_max_val;
    BoxConstraint box(x_min, x_max, u_min, u_max);

    SimpleLQRSQP solver;
    const bool ok = solver.solve(A, B, x0, N, stage_cost, terminal_cost, &box);
    ASSERT_TRUE(ok);

    EXPECT_NEAR(solver.u[0](0), u_max_val, 1e-8);
    EXPECT_NEAR(solver.x[1](0), x0(0) + u_max_val, 1e-8);

    Vector g;
    box.evaluate(solver.x[0], solver.u[0], Vector(), g);
    EXPECT_LT(g(2), -1e-6); // u_min - u < 0，下界 inactive
    EXPECT_NEAR(g(3), 0.0, 1e-8); // u - u_max = 0，上界激活
}

TEST(QuadraticTrackingCost, RejectsNegativeWeightMatrices)
{
    // 测试目的：验证 QuadraticTrackingCost 构造函数对非半正定 Q/R 抛出异常
    // 流程：构造含负特征值的对称 Q 和 R，检查是否抛出 std::invalid_argument
    // 预期效果：所有非法构造均抛出 std::invalid_argument
    const int nx = 2;
    const int nu = 2;
    Vector x_ref(nx);
    x_ref << 0.0, 0.0;
    Matrix Q_ok = Matrix::Identity(nx, nx);
    Matrix R_ok = Matrix::Identity(nu, nu);

    Matrix Q_bad(nx, nx);
    Q_bad << 1.0, 0.0,
        0.0, -1.0;
    EXPECT_THROW(QuadraticTrackingCost(x_ref, Q_bad, R_ok), std::invalid_argument);

    Matrix R_bad(nu, nu);
    R_bad << -0.1, 0.0,
        0.0, 1.0;
    EXPECT_THROW(QuadraticTrackingCost(x_ref, Q_ok, R_bad), std::invalid_argument);
}

TEST(QuadraticTrackingCost, ComputesWeightedCostGradientAndHessianWithDenseSymmetricWeights)
{
    // 测试目的：验证 QuadraticTrackingCost 在带非对角耦合的对称权重下仍正确
    // 流程：构造稠密对称 Q/R，手算代价、梯度、Hessian 并与接口输出比较
    // 预期效果：evaluate/gradient/hessian 与解析公式一致
    const int nx = 2;
    const int nu = 2;
    Vector x_ref(nx);
    x_ref << 1.0, 2.0;
    Matrix Q(nx, nx);
    Q << 2.0, 0.5,
        0.5, 3.0;
    Matrix R(nu, nu);
    R << 1.0, -0.3,
        -0.3, 2.0;

    QuadraticTrackingCost cost(x_ref, Q, R);

    Vector x(nx), u(nu);
    x << 1.5, 1.0;
    u << 0.5, -0.2;

    const Vector dx = x - x_ref;
    const double expected_cost = 0.5 * dx.dot(Q * dx) + 0.5 * u.dot(R * u);

    double value = 0.0;
    cost.evaluate(x, u, value);
    EXPECT_NEAR(value, expected_cost, 1e-12);

    Vector q(nx), r(nu);
    cost.gradient(x, u, q, r);
    EXPECT_TRUE(q.isApprox(Q * dx, 1e-12));
    EXPECT_TRUE(r.isApprox(R * u, 1e-12));

    Matrix Q_hess(nx, nx), R_hess(nu, nu), S(nu, nx);
    cost.hessian(x, u, Q_hess, R_hess, S);
    EXPECT_TRUE(Q_hess.isApprox(Q, 1e-12));
    EXPECT_TRUE(R_hess.isApprox(R, 1e-12));
    EXPECT_TRUE(S.isZero(1e-12));
}

TEST(BoxConstraint, EvaluatesViolationSignsForEachBound)
{
    // 测试目的：验证 BoxConstraint 在状态/控制分别越界时，只有对应行变为正值
    // 流程：构造上下界，分别触发 x < x_min、x > x_max、u < u_min、u > u_max 四种情况
    // 预期效果：g 向量中仅越界行大于 0，其余行仍小于等于 0
    // 行布局：g = [x_min - x(2); x - x_max(2); u_min - u(2); u - u_max(2)]，共 8 行
    const int nx = 2;
    const int nu = 2;
    Vector x_min(nx), x_max(nx), u_min(nu), u_max(nu);
    x_min << -1.0, -2.0;
    x_max << 1.0, 2.0;
    u_min << -0.5, -1.0;
    u_max << 0.5, 1.0;
    BoxConstraint box(x_min, x_max, u_min, u_max);

    Vector g;

    // 状态第 0 维低于下界
    Vector x(nx), u(nu);
    x << -2.0, 0.0;
    u << 0.0, 0.0;
    box.evaluate(x, u, Vector(), g);
    EXPECT_GT(g(0), 0.0); // x_min(0) - x(0) > 0
    EXPECT_LE(g(1), 0.0);
    EXPECT_LE(g(2), 0.0);
    EXPECT_LE(g(3), 0.0);
    EXPECT_LE(g(4), 0.0);
    EXPECT_LE(g(5), 0.0);
    EXPECT_LE(g(6), 0.0);
    EXPECT_LE(g(7), 0.0);

    // 状态第 1 维高于上界
    x << 0.0, 3.0;
    box.evaluate(x, u, Vector(), g);
    EXPECT_LE(g(0), 0.0);
    EXPECT_LE(g(1), 0.0);
    EXPECT_LE(g(2), 0.0); // x(0) - x_max(0) = -1 <= 0
    EXPECT_GT(g(3), 0.0); // x(1) - x_max(1) > 0
    EXPECT_LE(g(4), 0.0);
    EXPECT_LE(g(5), 0.0);
    EXPECT_LE(g(6), 0.0);
    EXPECT_LE(g(7), 0.0);

    // 控制第 0 维低于下界
    x << 0.0, 0.0;
    u << -1.0, 0.0;
    box.evaluate(x, u, Vector(), g);
    EXPECT_LE(g(0), 0.0);
    EXPECT_LE(g(1), 0.0);
    EXPECT_LE(g(2), 0.0);
    EXPECT_LE(g(3), 0.0);
    EXPECT_GT(g(4), 0.0); // u_min(0) - u(0) > 0
    EXPECT_LE(g(5), 0.0);
    EXPECT_LE(g(6), 0.0);
    EXPECT_LE(g(7), 0.0);

    // 控制第 1 维高于上界
    u << 0.0, 2.0;
    box.evaluate(x, u, Vector(), g);
    EXPECT_LE(g(0), 0.0);
    EXPECT_LE(g(1), 0.0);
    EXPECT_LE(g(2), 0.0);
    EXPECT_LE(g(3), 0.0);
    EXPECT_LE(g(4), 0.0);
    EXPECT_LE(g(5), 0.0);
    EXPECT_LE(g(6), 0.0);
    EXPECT_GT(g(7), 0.0); // u(1) - u_max(1) > 0
}

TEST(SQPConvergence, ScalarLQRStateAndControlBoundsActivateTogether)
{
    // 测试目的：验证 active-set helper 在两条约束同时激活时仍能正确求解
    // 流程：构造标量 N=1 LQR，让控制下界与状态上界同时 active
    // 预期效果：solver.u[0] 等于 u_min，solver.x[1] 等于 x_max，两条对应约束均为 0
    const int nx = 1;
    const int nu = 1;
    const int N = 1;

    Matrix A(nx, nx);
    A << 1.0;
    Matrix B(nx, nu);
    B << 1.0;
    Matrix Q(nx, nx);
    Q << 1.0;
    Matrix R(nu, nu);
    R << 1.0;

    const Matrix P = solveDare(A, B, Q, R);
    const Matrix K = (R + B.transpose() * P * B).ldlt().solve(B.transpose() * P * A);

    Vector x0(nx);
    x0 << 1.0;
    const double u_unc = (-K * x0)(0);
    ASSERT_LT(u_unc, -0.4) << "Unconstrained optimal control should be sufficiently negative so both constructed bounds become active";

    Vector x_ref = Vector::Zero(nx);
    QuadraticTrackingCost stage_cost(x_ref, Q, R);
    TerminalQuadraticCost terminal_cost(x_ref, P);

    const double u_min_val = -0.3;
    const double x_max_val = 0.7;
    Vector x_min(nx), x_max(nx), u_min(nu), u_max(nu);
    x_min << -10.0;
    x_max << x_max_val;
    u_min << u_min_val;
    u_max << 10.0;
    BoxConstraint box(x_min, x_max, u_min, u_max);

    SimpleLQRSQP solver;
    const bool ok = solver.solve(A, B, x0, N, stage_cost, terminal_cost, &box);
    ASSERT_TRUE(ok);

    EXPECT_NEAR(solver.u[0](0), u_min_val, 1e-8);
    EXPECT_NEAR(solver.x[1](0), x_max_val, 1e-8);

    Vector g;
    // x0 由初始条件固定，本身可能违反 box，但优化器不应将其纳入 active set
    box.evaluate(solver.x[0], solver.u[0], Vector(), g);
    EXPECT_GT(g(1), 1e-6); // x0 - x_max > 0，说明初始状态本身在 box 外，但这不是优化变量
    EXPECT_NEAR(g(2), 0.0, 1e-8); // u_min - u = 0，控制下界激活

    // x1 被优化到状态上界
    Vector g1;
    box.evaluate(solver.x[1], Vector::Zero(nu), Vector(), g1);
    EXPECT_NEAR(g1(1), 0.0, 1e-8); // x1 - x_max = 0，状态上界激活
    EXPECT_LT(g1(0), -1e-6);

    // u0 被优化到控制下界
    EXPECT_LT(g(0), -1e-6);
    EXPECT_LT(g(3), -1e-6);
}
