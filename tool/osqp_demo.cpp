/// @file    tool/osqp_demo.cpp
/// @brief   OSQP 二次规划求解器集成验证 Demo
///
/// 通过 OsqpSolver 包装类求解一个简单的 2 变量 QP 问题，
/// 验证：(1) OSQP 编译链接正常  (2) Pimpl 隔离方案有效
///
/// 注意：本文件 #include "osqp_solver.h" 而非 "osqp.h"，
/// 这是项目规定的唯一正确用法，绝不可直接包含 osqp.h。

#include <cmath>
#include <cstdio>
#include <vector>

#include "../src/core/osqp_solver.h"

using apa_post_processor::OsqpSolver;
using apa_post_processor::OsqpSolverConfig;
using apa_post_processor::OsqpStatus;

int main() {
    // ============================================================
    // 问题定义：求解以下凸二次规划
    //   minimize   0.5 * x' P x + q' x
    //   subject to l <= A x <= u
    //
    // P = [[4, 1],    q = [1,    A = [[1, 1],    l = [1,    u = [1,
    //      [1, 2]]         1]         [1, 0],         0]         0.7]
    //                                 [0, 1]]         0]         0.7]
    // ============================================================

    const int n = 2;  // 决策变量数量
    const int m = 3;  // 约束数量

    // ---- 目标函数 Hessian 矩阵 P (CSC 稀疏格式) ----
    // P = [4, 1; 1, 2]，仅存储下三角非零元
    const std::vector<double> P_x = {4.0, 1.0, 2.0};
    const std::vector<int>    P_i = {0, 0, 1};    // 行索引 (0-based)
    const std::vector<int>    P_p = {0, 1, 3};    // 列指针

    // ---- 目标函数线性项 q ----
    const std::vector<double> q = {1.0, 1.0};

    // ---- 约束矩阵 A (CSC 稀疏格式) ----
    // A = [1, 1; 1, 0; 0, 1]
    const std::vector<double> A_x = {1.0, 1.0, 1.0, 1.0};
    const std::vector<int>    A_i = {0, 1, 0, 2};
    const std::vector<int>    A_p = {0, 2, 4};

    // ---- 约束上下界 ----
    const std::vector<double> l = {1.0, 0.0, 0.0};
    const std::vector<double> u = {1.0, 0.7, 0.7};

    // ============================================================
    // 通过 OsqpSolver 包装类求解（宏污染完全隔离在 osqp_solver.cpp 内）
    // ============================================================
    OsqpSolver solver;
    OsqpSolverConfig config;
    config.verbose = false;

    bool setup_ok = solver.setup(n, m, P_x, P_i, P_p, q, A_x, A_i, A_p, l, u, config);
    if (!setup_ok) {
        std::printf("[FAIL] OsqpSolver::setup() 失败\n");
        return 1;
    }

    auto result = solver.solve();

    // ============================================================
    // 输出结果
    // ============================================================
    std::printf("===== OSQP Demo 求解结果 =====\n");
    std::printf("状态: %s\n", result.status_msg.c_str());
    std::printf("迭代次数: %d\n", result.iter);
    std::printf("目标函数值: %.6f\n", result.obj_val);
    std::printf("最优解 x:\n");
    for (int i = 0; i < n; ++i) {
        std::printf("  x[%d] = %.6f\n", i, result.x[static_cast<size_t>(i)]);
    }

    // 验证：理论最优解约为 x ≈ [0.3, 0.7]
    const double x0_expected = 0.3;
    const double x1_expected = 0.7;
    const double tol = 2e-3;  // OSQP 默认精度 eps_abs=1e-3，容许 2e-3 误差

    bool ok = true;
    if (std::fabs(result.x[0] - x0_expected) > tol) {
        std::printf("[FAIL] x[0] = %.6f 偏离预期 %.6f\n", result.x[0], x0_expected);
        ok = false;
    }
    if (std::fabs(result.x[1] - x1_expected) > tol) {
        std::printf("[FAIL] x[1] = %.6f 偏离预期 %.6f\n", result.x[1], x1_expected);
        ok = false;
    }

    if (ok) {
        std::printf("\n[PASS] OSQP 集成验证通过！（通过 OsqpSolver Pimpl 包装）\n");
    }

    return ok ? 0 : 1;
}
