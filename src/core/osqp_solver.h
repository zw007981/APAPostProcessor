// OSQP 二次规划求解器的 C++ 包装（Pimpl 隔离 OSQP C 头文件中的宏污染）。

#pragma once

#include <memory>
#include <string>
#include <vector>

namespace apa_post_processor {
// OSQP 求解状态编码
enum class OsqpStatus {
    Solved,
    SolvedInaccurate,
    MaxIterReached,
    PrimalInfeasible,
    DualInfeasible,
    PrimalInfeasibleInaccurate,
    DualInfeasibleInaccurate,
    NonConvex,
    SetupError,
    Unsolved,
};

// OSQP 求解器配置
struct OsqpSolverConfig {
    double rho = 0.1;
    double sigma = 1e-6;
    double alpha = 1.6;
    double eps_abs = 1e-3;
    double eps_rel = 1e-3;
    int max_iter = 4000;
    bool verbose = false;
    bool warm_start = true;
    bool polish = false;
    bool adaptive_rho = true;
};

// 单次 QP 求解结果
struct OsqpResult {
    OsqpStatus status = OsqpStatus::Unsolved;
    std::string status_msg;
    int iter = 0;
    double obj_val = 0.0;
    std::vector<double> x;
    std::vector<double> y;
};

// OSQP 求解器包装类（Pimpl 隔离宏污染）。
class OsqpSolver {
   public:
    OsqpSolver();
    ~OsqpSolver();
    OsqpSolver(OsqpSolver&& other) noexcept;
    OsqpSolver& operator=(OsqpSolver&& other) noexcept;
    OsqpSolver(const OsqpSolver&) = delete;
    OsqpSolver& operator=(const OsqpSolver&) = delete;
    // 用 CSC 稀疏格式初始化 QP 问题。
    bool setup(int n, int m, const std::vector<double>& P_x,
               const std::vector<int>& P_i, const std::vector<int>& P_p,
               const std::vector<double>& q, const std::vector<double>& A_x,
               const std::vector<int>& A_i, const std::vector<int>& A_p,
               const std::vector<double>& l, const std::vector<double>& u,
               const OsqpSolverConfig& config = OsqpSolverConfig{});
    // 求解 QP 问题。
    OsqpResult solve();
    // 更新边界（无需重新 factorize）。
    bool updateBounds(const std::vector<double>& q_new,
                      const std::vector<double>& l_new,
                      const std::vector<double>& u_new);
    // 是否已完成 setup。
    bool isReady() const;

   protected:
    // Pimpl：将 OSQP 所有 C 类型藏在 .cpp 中，彻底隔离宏污染
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
}  // namespace apa_post_processor
