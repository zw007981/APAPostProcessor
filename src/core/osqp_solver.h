// OSQP 二次规划求解器的干净 C++ 包装（Pimpl 隔离）
//
// 设计背景：为什么需要这个包装？
// OSQP 的公开头文件（尤其是 constants.h）将 RHO、SIGMA、ALPHA、MAX_ITER、
// EPS_ABS、EPS_REL、DELTA、POLISH、VERBOSE 等常用缩写定义为 C 宏，直接暴露在
// 全局命名空间。复杂项目中任何包含 osqp.h 的翻译单元都会被这些宏污染，极易与
// 项目自身或其他第三方库的标识符发生冲突。
//
// 硬规定（不可违反）：
// - #include "osqp.h" 只允许出现在 osqp_solver.cpp 这一个文件里；
// - 项目其他任何 .h/.hpp/.cpp 文件如需使用 OSQP 功能，一律 #include
//   "osqp_solver.h"，通过本包装类的干净接口交互；
// - 禁止在任何公共头文件中直接或间接包含 osqp.h。
//
// 设计思路：采用 Pimpl (Pointer-to-Implementation) 惯用法将 OSQP 所有 C
// 类型（OSQPWorkspace、OSQPSettings、OSQPData、csc 等）完全隐藏在 .cpp 中，
// 对外只暴露纯 C++17 类型（std::vector<double>），实现宏污染零泄漏、编译期
// 依赖隔离（OSQP 升级只需重编译 osqp_solver.cpp）以及 RAII 自动资源管理
// （杜绝手动 c_malloc/c_free）。

#pragma once

#include <memory>
#include <string>
#include <vector>

namespace apa_post_processor {
// OSQP 的求解状态编码（语义化重导出，不依赖 OSQP 宏）
enum class OsqpStatus {
    Solved,                    // OSQP_SOLVED
    SolvedInaccurate,          // OSQP_SOLVED_INACCURATE
    MaxIterReached,            // OSQP_MAX_ITER_REACHED
    PrimalInfeasible,          // OSQP_PRIMAL_INFEASIBLE
    DualInfeasible,            // OSQP_DUAL_INFEASIBLE
    PrimalInfeasibleInaccurate,
    DualInfeasibleInaccurate,
    NonConvex,                 // OSQP_NON_CVX
    SetupError,                // osqp_setup 失败
    Unsolved,                  // 尚未调用 solve()
};

// OSQP 求解器配置（语义化，不依赖 OSQP 宏）
struct OsqpSolverConfig {
    double rho           = 0.1;     // 增广拉格朗日步长参数 (ADMM step-size)
    double sigma         = 1e-6;    // 正则化参数
    double alpha         = 1.6;     // 过松弛参数 (over-relaxation)
    double eps_abs       = 1e-3;    // 绝对收敛容差
    double eps_rel       = 1e-3;    // 相对收敛容差
    int    max_iter      = 4000;    // 最大迭代次数
    bool   verbose       = false;   // 是否输出求解日志
    bool   warm_start    = true;    // 是否启用热启动
    bool   polish        = false;   // 是否启用解精炼 (polishing)
    bool   adaptive_rho  = true;    // 是否自适应调整 rho
};

// 单次 QP 求解结果
struct OsqpResult {
    OsqpStatus           status = OsqpStatus::Unsolved;
    std::string          status_msg;         // 原始状态字符串（如 "solved"）
    int                  iter        = 0;    // 实际迭代次数
    double               obj_val     = 0.0;  // 目标函数值
    std::vector<double>  x;                  // 原始变量最优解
    std::vector<double>  y;                  // 对偶变量（拉格朗日乘子）
};

// OSQP 求解器包装类（Pimpl 隔离 OSQP C 头文件中的宏污染）
//
// 用法示例：
//   OsqpSolver solver;
//   solver.setup(n, m, P_x, P_i, P_p, q, A_x, A_i, A_p, l, u);
//   OsqpResult result = solver.solve();
//   if (result.status == OsqpStatus::Solved) {
//       // 使用 result.x
//   }
class OsqpSolver {
public:
    OsqpSolver();
    // 显式声明析构/移动在 .cpp 中实现（Pimpl 要求完整类型可见）
    ~OsqpSolver();
    OsqpSolver(OsqpSolver&& other) noexcept;
    OsqpSolver& operator=(OsqpSolver&& other) noexcept;
    // 禁止拷贝（内部持有 OSQP workspace 裸指针，语义上不应拷贝）
    OsqpSolver(const OsqpSolver&) = delete;
    OsqpSolver& operator=(const OsqpSolver&) = delete;
    // 用 CSC 稀疏格式的问题数据初始化求解器：
    // n/m 为决策变量与约束数量；P_x/P_i/P_p 为 Hessian 矩阵的 CSC 三元组
    // （数值/行索引/列指针，列指针长度 n+1）；q 为线性项系数向量（长度 n）；
    // A_x/A_i/A_p 为约束矩阵的 CSC 三元组；l/u 为约束下界/上界向量（长度 m）；
    // config 为可选的求解器参数。返回值表示是否初始化成功。
    bool setup(int n, int m,
               const std::vector<double>& P_x,
               const std::vector<int>&    P_i,
               const std::vector<int>&    P_p,
               const std::vector<double>& q,
               const std::vector<double>& A_x,
               const std::vector<int>&    A_i,
               const std::vector<int>&    A_p,
               const std::vector<double>& l,
               const std::vector<double>& u,
               const OsqpSolverConfig& config = OsqpSolverConfig{});
    // 求解已配置的 QP 问题，返回结果（状态、解向量、目标值等）
    OsqpResult solve();
    // 更新线性项 q 和约束上下界 l, u，无需重新 factorize，
    // 适用于 NMPC 中反复求解仅边界变化的 QP 子问题的场景，返回是否更新成功
    bool updateBounds(const std::vector<double>& q_new,
                      const std::vector<double>& l_new,
                      const std::vector<double>& u_new);
    // 求解器是否已完成 setup
    bool isReady() const;

protected:
    // Pimpl：将 OSQP 所有 C 类型藏在 .cpp 中，彻底隔离宏污染
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
}  // namespace apa_post_processor
