#include "dense_qp_solver.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "soft_constraint_validation.h"

namespace stc_SQP {
QPSolverStatus DenseQPSolver::solve(const QPData& qp_data, QPSolution& qp_sol) {
    // 1. 输入校验
    if (const auto status = validateInput(qp_data); status != QPSolverStatus::SUCCESS) {
        return status;
    }
    // 2. 构造稠密变量布局
    const Layout layout = buildLayout(qp_data);
    // 3. 装配代价与约束
    Assembly assembly;
    assembleCost(qp_data, layout, assembly.H, assembly.g);
    assembleEqualityConstraints(qp_data, layout, assembly.A_eq, assembly.b_eq);
    assembleInequalityConstraints(qp_data, layout, assembly.A_ineq, assembly.b_ineq);
    // 4. active-set 求解
    Vector z;
    const auto status = solveActiveSet(assembly, z);
    if (status != QPSolverStatus::SUCCESS) {
        return status;
    }
    if (!z.allFinite()) {
        return QPSolverStatus::NAN_IN_SOLUTION;
    }
    // 5. 写回 QPSolution
    writeSolution(z, layout, qp_sol);
    return QPSolverStatus::SUCCESS;
}

QPSolverStatus DenseQPSolver::validateInput(const QPData& qp_data) const {
    // 软约束配置合法性校验
    if (!soft_constraint_validation::validate(qp_data)) {
        return QPSolverStatus::INVALID_ARGUMENT;
    }
    // QPData 维度与容器尺寸校验
    if (!hasValidDimensionsAndContainers(qp_data)) {
        return QPSolverStatus::INVALID_ARGUMENT;
    }
    return QPSolverStatus::SUCCESS;
}

bool DenseQPSolver::hasValidDimensionsAndContainers(const QPData& qp_data) const {
    const int N = qp_data.N;
    const int nx = qp_data.nx;
    const int nu = qp_data.nu;
    const int ng = qp_data.ng_max;
    // Dense 求解器要求维度为正（nu 为 0 时无控制变量，当前不支持）
    if (N <= 0 || nx <= 0 || nu <= 0 || ng < 0) {
        return false;
    }
    return qp_data.hasValidContainerSizes();
}

DenseQPSolver::Layout DenseQPSolver::buildLayout(const QPData& qp_data) const {
    Layout layout;
    layout.N = qp_data.N;
    layout.nx = qp_data.nx;
    layout.nu = qp_data.nu;
    layout.ng = qp_data.ng_max;
    layout.has_soft = qp_data.soft_config != nullptr && qp_data.soft_config->ns > 0;
    layout.ns = layout.has_soft ? qp_data.soft_config->ns : 0;
    layout.n_var = (layout.N + 1) * layout.nx + layout.N * layout.nu +
                   layout.N * layout.ns;
    return layout;
}

bool DenseQPSolver::isInitialStateFixed(const QPData& qp_data) const {
    const int nx = qp_data.nx;
    for (int i = 0; i < nx; ++i) {
        const double lb = qp_data.lbx[0](i);
        const double ub = qp_data.ubx[0](i);
        if (!std::isfinite(lb) || !std::isfinite(ub) ||
            std::abs(lb - ub) > tol_) {
            return false;
        }
    }
    return true;
}

void DenseQPSolver::assembleCost(const QPData& qp_data, const Layout& layout,
    Matrix& H, Vector& g) const {
    const int N = layout.N;
    const int nx = layout.nx;
    const int nu = layout.nu;
    const int ns = layout.ns;
    H = Matrix::Zero(layout.n_var, layout.n_var);
    g = Vector::Zero(layout.n_var);
    // 阶段状态代价 Q、q
    for (int k = 0; k <= N; ++k) {
        const int x_off = layout.xOffset(k);
        H.block(x_off, x_off, nx, nx) += qp_data.Q[k];
        g.segment(x_off, nx) += qp_data.q[k];
    }
    // 阶段控制代价 R、r 与交叉项 S
    for (int k = 0; k < N; ++k) {
        const int x_off = layout.xOffset(k);
        const int u_off = layout.uOffset(k);
        H.block(u_off, u_off, nu, nu) += qp_data.R[k];
        g.segment(u_off, nu) += qp_data.r[k];
        H.block(u_off, x_off, nu, nx) += qp_data.S[k];
        H.block(x_off, u_off, nx, nu) += qp_data.S[k].transpose();
    }
    if (layout.has_soft && ns > 0) {
        for (int k = 0; k < N; ++k) {
            const int s_off = layout.sOffset(k);
            H.block(s_off, s_off, ns, ns).diagonal() += qp_data.soft_config->Zu;
            g.segment(s_off, ns) += qp_data.soft_config->zu;
        }
    }
}

void DenseQPSolver::assembleEqualityConstraints(const QPData& qp_data,
    const Layout& layout, Matrix& A_eq, Vector& b_eq) const {
    const int N = layout.N;
    const int nx = layout.nx;
    const int nu = layout.nu;
    const bool x0_fixed = isInitialStateFixed(qp_data);
    const int n_eq = (x0_fixed ? nx : 0) + N * nx;
    A_eq = Matrix::Zero(n_eq, layout.n_var);
    b_eq = Vector::Zero(n_eq);
    int row = 0;
    // 初始状态固定条件
    if (x0_fixed) {
        A_eq.block(0, 0, nx, nx) = Matrix::Identity(nx, nx);
        b_eq.head(nx) = qp_data.lbx[0];
        row += nx;
    }
    // 动力学等式：x_{k+1} - A_k x_k - B_k u_k = b_k
    for (int k = 0; k < N; ++k) {
        const int r = row + k * nx;
        const int x_off = layout.xOffset(k);
        const int x_next_off = layout.xOffset(k + 1);
        const int u_off = layout.uOffset(k);
        A_eq.block(r, x_next_off, nx, nx) = Matrix::Identity(nx, nx);
        A_eq.block(r, x_off, nx, nx) -= qp_data.A[k];
        A_eq.block(r, u_off, nx, nu) -= qp_data.B[k];
        b_eq.segment(r, nx) = qp_data.b[k];
    }
}

void DenseQPSolver::assembleInequalityConstraints(const QPData& qp_data,
    const Layout& layout, Matrix& A_ineq, Vector& b_ineq) const {
    const int N = layout.N;
    const int nx = layout.nx;
    const int nu = layout.nu;
    const int ng = layout.ng;
    const int ns = layout.ns;
    const bool x0_fixed = isInitialStateFixed(qp_data);
    // 最大可能不等式行数：状态上下界 + 控制上下界 + 普通约束 + slack 非负
    const int max_ineq = 2 * (N + 1) * nx + 2 * N * nu + N * ng + N * ns;
    A_ineq = Matrix::Zero(max_ineq, layout.n_var);
    b_ineq = Vector::Zero(max_ineq);
    int row = 0;
    row = appendStateBounds(qp_data, layout, x0_fixed, A_ineq, b_ineq, row);
    row = appendControlBounds(qp_data, layout, A_ineq, b_ineq, row);
    // 软约束行号映射：普通约束行 -> 局部 slack 变量索引
    std::vector<int> soft_local(ng, -1);
    if (layout.has_soft && ns > 0) {
        for (int s = 0; s < ns; ++s) {
            const int idx = qp_data.soft_config->idxs[s];
            if (idx >= 0 && idx < ng) {
                soft_local[idx] = s;
            }
        }
    }
    row = appendGeneralConstraints(qp_data, layout, soft_local, A_ineq, b_ineq, row);
    row = appendSlackNonNegativity(layout, A_ineq, b_ineq, row);
    // 截断到实际使用行数
    A_ineq.conservativeResize(row, layout.n_var);
    b_ineq.conservativeResize(row);
}

int DenseQPSolver::appendStateBounds(const QPData& qp_data, const Layout& layout,
    bool x0_fixed, Matrix& A_ineq, Vector& b_ineq, int row) const {
    const int N = layout.N;
    const int nx = layout.nx;
    for (int k = 0; k <= N; ++k) {
        const int x_off = layout.xOffset(k);
        for (int i = 0; i < nx; ++i) {
            const double lb = qp_data.lbx[k](i);
            const double ub = qp_data.ubx[k](i);
            if (std::isfinite(ub)) {
                // 初始状态已作为等式固定时，不再添加重复上界
                if (!x0_fixed || k != 0 || std::abs(lb - ub) > tol_) {
                    A_ineq.row(row).setZero();
                    A_ineq(row, x_off + i) = 1.0;
                    b_ineq(row) = ub;
                    ++row;
                }
            }
            if (std::isfinite(lb) && std::abs(lb - ub) > tol_) {
                A_ineq.row(row).setZero();
                A_ineq(row, x_off + i) = -1.0;
                b_ineq(row) = -lb;
                ++row;
            }
        }
    }
    return row;
}

int DenseQPSolver::appendControlBounds(const QPData& qp_data, const Layout& layout,
    Matrix& A_ineq, Vector& b_ineq, int row) const {
    const int N = layout.N;
    const int nu = layout.nu;
    for (int k = 0; k < N; ++k) {
        const int u_off = layout.uOffset(k);
        for (int i = 0; i < nu; ++i) {
            const double lb = qp_data.lbu[k](i);
            const double ub = qp_data.ubu[k](i);
            if (std::isfinite(ub)) {
                A_ineq.row(row).setZero();
                A_ineq(row, u_off + i) = 1.0;
                b_ineq(row) = ub;
                ++row;
            }
            if (std::isfinite(lb) && std::abs(lb - ub) > tol_) {
                A_ineq.row(row).setZero();
                A_ineq(row, u_off + i) = -1.0;
                b_ineq(row) = -lb;
                ++row;
            }
        }
    }
    return row;
}

int DenseQPSolver::appendGeneralConstraints(const QPData& qp_data, const Layout& layout,
    const std::vector<int>& soft_local,
    Matrix& A_ineq, Vector& b_ineq, int row) const {
    const int N = layout.N;
    const int nx = layout.nx;
    const int nu = layout.nu;
    const int ng = layout.ng;
    const int ns = layout.ns;
    for (int k = 0; k < N; ++k) {
        const int x_off = layout.xOffset(k);
        const int u_off = layout.uOffset(k);
        const int s_off = layout.sOffset(k);
        for (int j = 0; j < ng; ++j) {
            const double rhs = qp_data.d[k](j);
            if (!std::isfinite(rhs)) {
                continue;
            }
            A_ineq.row(row).setZero();
            A_ineq.block(row, x_off, 1, nx) = qp_data.C[k].row(j);
            A_ineq.block(row, u_off, 1, nu) = qp_data.D[k].row(j);
            const int local_s = soft_local[j];
            if (local_s >= 0) {
                A_ineq(row, s_off + local_s) = -1.0;
            }
            b_ineq(row) = rhs;
            ++row;
        }
    }
    return row;
}

int DenseQPSolver::appendSlackNonNegativity(const Layout& layout,
    Matrix& A_ineq, Vector& b_ineq, int row) const {
    const int N = layout.N;
    const int ns = layout.ns;
    if (ns <= 0) {
        return row;
    }
    const Matrix neg_identity = -Matrix::Identity(ns, ns);
    for (int k = 0; k < N; ++k) {
        const int s_off = layout.sOffset(k);
        A_ineq.block(row, s_off, ns, ns) = neg_identity;
        b_ineq.segment(row, ns).setZero();
        row += ns;
    }
    return row;
}

QPSolverStatus DenseQPSolver::solveActiveSet(const Assembly& assembly, Vector& z) const {
    const int n_var = static_cast<int>(assembly.H.rows());
    const int n_eq = static_cast<int>(assembly.A_eq.rows());
    const int n_ineq = static_cast<int>(assembly.A_ineq.rows());
    std::vector<int> active_set;
    active_set.reserve(n_ineq);
    std::vector<char> is_active(n_ineq, 0);
    const int max_iter = std::max(1000, 10 * n_ineq);
    z = Vector::Zero(n_var);
    bool converged = false;
    // 预分配 active-set 循环中复用的 workspace，避免每轮重复分配大矩阵
    const int max_n_act = n_eq + n_ineq;
    Matrix A_act = Matrix::Zero(max_n_act, n_var);
    Vector b_act = Vector::Zero(max_n_act);
    Matrix KKT = Matrix::Zero(n_var + max_n_act, n_var + max_n_act);
    Vector rhs = Vector::Zero(n_var + max_n_act);
    Vector sol = Vector::Zero(n_var + max_n_act);
    for (int iter = 0; iter < max_iter; ++iter) {
        const int n_act = n_eq + static_cast<int>(active_set.size());
        A_act.setZero();
        b_act.setZero();
        if (n_eq > 0) {
            A_act.topRows(n_eq) = assembly.A_eq;
            b_act.head(n_eq) = assembly.b_eq;
        }
        for (int a = 0; a < static_cast<int>(active_set.size()); ++a) {
            const int idx = active_set[a];
            A_act.row(n_eq + a) = assembly.A_ineq.row(idx);
            b_act(n_eq + a) = assembly.b_ineq(idx);
        }
        // KKT: [H A^T; A 0] [z; lambda] = [-g; b]
        // 注意：KKT 是预分配的大矩阵，实际有效块必须按左上角连续存放，
        // 因此用 block(0, n_var, ...) 而不是 topRightCorner(...)。
        KKT.setZero();
        KKT.topLeftCorner(n_var, n_var) = assembly.H;
        if (n_act > 0) {
            KKT.block(0, n_var, n_var, n_act) = A_act.topRows(n_act).transpose();
            KKT.block(n_var, 0, n_act, n_var) = A_act.topRows(n_act);
        }
        rhs.setZero();
        rhs.head(n_var) = -assembly.g;
        rhs.segment(n_var, n_act) = b_act.head(n_act);
        Eigen::LDLT<Matrix> ldlt(KKT.topLeftCorner(n_var + n_act, n_var + n_act));
        if (ldlt.info() != Eigen::Success) {
            return QPSolverStatus::UNKNOWN_ERROR;
        }
        sol.head(n_var + n_act) = ldlt.solve(rhs.head(n_var + n_act));
        if (!sol.head(n_var + n_act).allFinite()) {
            return QPSolverStatus::NAN_IN_SOLUTION;
        }
        z = sol.head(n_var);
        // 检查所有不等式违反情况
        Vector residual = assembly.A_ineq * z - assembly.b_ineq;
        int worst_idx = -1;
        double worst_val = tol_;
        for (int i = 0; i < n_ineq; ++i) {
            if (!is_active[i] && residual(i) > worst_val) {
                worst_val = residual(i);
                worst_idx = i;
            }
        }
        if (worst_idx < 0) {
            // 检查 active inequality 乘子符号
            bool removed = false;
            if (!active_set.empty()) {
                const Vector lambda_act = sol.segment(n_var + n_eq, active_set.size());
                int remove_pos = -1;
                double most_negative = -tol_;
                for (int a = 0; a < static_cast<int>(active_set.size()); ++a) {
                    const double lam = lambda_act(a);
                    if (lam < most_negative) {
                        most_negative = lam;
                        remove_pos = a;
                    }
                }
                if (remove_pos >= 0) {
                    is_active[active_set[remove_pos]] = 0;
                    active_set.erase(active_set.begin() + remove_pos);
                    removed = true;
                }
            }
            if (!removed) {
                converged = true;
                break;
            }
        } else {
            active_set.push_back(worst_idx);
            is_active[worst_idx] = 1;
        }
    }
    if (!converged) {
        return QPSolverStatus::MAX_ITER_REACHED;
    }
    // 最终校验：等式与不等式残差必须在容差内
    if (n_eq > 0) {
        const double eq_res = (assembly.A_eq * z - assembly.b_eq).norm();
        if (eq_res > 1e-9) {
            return QPSolverStatus::UNKNOWN_ERROR;
        }
    }
    const double ineq_res = (assembly.A_ineq * z - assembly.b_ineq).cwiseMax(0.0).norm();
    if (ineq_res > tol_) {
        return QPSolverStatus::INFEASIBLE;
    }
    return QPSolverStatus::SUCCESS;
}

void DenseQPSolver::writeSolution(const Vector& z, const Layout& layout,
    QPSolution& qp_sol) const {
    const int N = layout.N;
    const int nx = layout.nx;
    const int nu = layout.nu;
    const int ns = layout.ns;
    qp_sol.resize(N, nx, nu, ns);
    for (int k = 0; k <= N; ++k) {
        qp_sol.x[k] = z.segment(layout.xOffset(k), nx);
    }
    for (int k = 0; k < N; ++k) {
        qp_sol.u[k] = z.segment(layout.uOffset(k), nu);
        if (ns > 0) {
            qp_sol.s[k] = z.segment(layout.sOffset(k), ns);
        }
    }
}
} // namespace stc_SQP
