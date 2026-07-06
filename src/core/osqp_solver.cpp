/// @file    src/core/osqp_solver.cpp
/// @brief   OsqpSolver 的 Pimpl 实现
///
/// **这是本项目中唯一允许直接包含 osqp.h 的文件。**
/// 原因参见 osqp_solver.h 头注释中的「硬规定」。
///
/// 将 OSQP 所有 C 类型（OSQPWorkspace / OSQPSettings / OSQPData / csc）及
/// 其伴随的宏（RHO, SIGMA, ALPHA 等）完全隔离在本翻译单元内，不向项目其他
/// 编译单元泄漏。

#include "osqp_solver.h"

#include <cstring>

#include "osqp.h"  // 仅此一处，隔离宏污染

namespace apa_post_processor {

// ============================================================
// Pimpl 内部实现：持有所有 OSQP C 类型
// ============================================================
struct OsqpSolver::Impl {
    // 问题维度
    int n_ = 0;
    int m_ = 0;

    // OSQP workspace（由 osqp_setup 分配，osqp_cleanup 释放）
    OSQPWorkspace* work_ = nullptr;

    // 求解器参数
    OsqpSolverConfig config_;

    // ---- 问题数据存储（CSC 格式）----
    // csc_matrix() 不拷贝数据，仅存储指针，因此原始数据必须在此处保活
    std::vector<c_float> P_x_;
    std::vector<c_int> P_i_;
    std::vector<c_int> P_p_;
    std::vector<c_float> q_;
    std::vector<c_float> A_x_;
    std::vector<c_int> A_i_;
    std::vector<c_int> A_p_;
    std::vector<c_float> l_;
    std::vector<c_float> u_;

    ~Impl() {
        if (work_ != nullptr) {
            osqp_cleanup(work_);
            work_ = nullptr;
        }
    }

    /// @brief 将 OsqpStatus 枚举映射为可读字符串
    static const char* statusToString(OsqpStatus s) {
        switch (s) {
            case OsqpStatus::Solved:
                return "solved";
            case OsqpStatus::SolvedInaccurate:
                return "solved inaccurate";
            case OsqpStatus::MaxIterReached:
                return "max iterations reached";
            case OsqpStatus::PrimalInfeasible:
                return "primal infeasible";
            case OsqpStatus::DualInfeasible:
                return "dual infeasible";
            case OsqpStatus::PrimalInfeasibleInaccurate:
                return "primal infeasible inaccurate";
            case OsqpStatus::DualInfeasibleInaccurate:
                return "dual infeasible inaccurate";
            case OsqpStatus::NonConvex:
                return "non-convex problem";
            case OsqpStatus::SetupError:
                return "setup error";
            case OsqpStatus::Unsolved:
                return "unsolved";
        }
        return "unknown";
    }

    /// @brief 将 OSQP 的状态码映射到我们的 OsqpStatus
    static OsqpStatus mapStatus(c_int status_val) {
        switch (status_val) {
            case OSQP_SOLVED:
                return OsqpStatus::Solved;
            case OSQP_SOLVED_INACCURATE:
                return OsqpStatus::SolvedInaccurate;
            case OSQP_MAX_ITER_REACHED:
                return OsqpStatus::MaxIterReached;
            case OSQP_PRIMAL_INFEASIBLE:
                return OsqpStatus::PrimalInfeasible;
            case OSQP_DUAL_INFEASIBLE:
                return OsqpStatus::DualInfeasible;
            case OSQP_PRIMAL_INFEASIBLE_INACCURATE:
                return OsqpStatus::PrimalInfeasibleInaccurate;
            case OSQP_DUAL_INFEASIBLE_INACCURATE:
                return OsqpStatus::DualInfeasibleInaccurate;
            case OSQP_NON_CVX:
                return OsqpStatus::NonConvex;
            default:
                return OsqpStatus::SetupError;
        }
    }
};

// ============================================================
// 构造 / 析构 / 移动
// ============================================================
OsqpSolver::OsqpSolver() : impl_(std::make_unique<Impl>()) {}

OsqpSolver::~OsqpSolver() = default;

OsqpSolver::OsqpSolver(OsqpSolver&& other) noexcept = default;

OsqpSolver& OsqpSolver::operator=(OsqpSolver&& other) noexcept = default;

// ============================================================
// setup —— 将 C++ 数据转换为 OSQP C 结构并初始化求解器
// ============================================================
bool OsqpSolver::setup(int n, int m, const std::vector<double>& P_x,
                       const std::vector<int>& P_i, const std::vector<int>& P_p,
                       const std::vector<double>& q,
                       const std::vector<double>& A_x,
                       const std::vector<int>& A_i, const std::vector<int>& A_p,
                       const std::vector<double>& l,
                       const std::vector<double>& u,
                       const OsqpSolverConfig& config) {
    // 先清理上一次的 workspace（如果存在）
    if (impl_->work_ != nullptr) {
        osqp_cleanup(impl_->work_);
        impl_->work_ = nullptr;
    }

    impl_->n_ = n;
    impl_->m_ = m;
    impl_->config_ = config;

    // ---- 将 double → c_float, int → c_int 拷贝到内部存储 ----
    // 类型转换是必须的：OSQP 的 c_float / c_int 可能在特定编译配置下
    // 不同于 double / int（如 c_float=float 或 c_int=long long）
    impl_->P_x_.assign(P_x.begin(), P_x.end());
    impl_->P_i_.assign(P_i.begin(), P_i.end());
    impl_->P_p_.assign(P_p.begin(), P_p.end());
    impl_->q_.assign(q.begin(), q.end());
    impl_->A_x_.assign(A_x.begin(), A_x.end());
    impl_->A_i_.assign(A_i.begin(), A_i.end());
    impl_->A_p_.assign(A_p.begin(), A_p.end());
    impl_->l_.assign(l.begin(), l.end());
    impl_->u_.assign(u.begin(), u.end());

    // ---- 构建 OSQP 的 CSC 稀疏矩阵（不拷贝数据，仅持有指针）----
    csc* P_mat = csc_matrix(static_cast<c_int>(n), static_cast<c_int>(n),
                            static_cast<c_int>(P_x.size()), impl_->P_x_.data(),
                            impl_->P_i_.data(), impl_->P_p_.data());
    csc* A_mat = csc_matrix(static_cast<c_int>(m), static_cast<c_int>(n),
                            static_cast<c_int>(A_x.size()), impl_->A_x_.data(),
                            impl_->A_i_.data(), impl_->A_p_.data());

    // ---- 填充 OSQPData ----
    OSQPData data{};
    data.n = static_cast<c_int>(n);
    data.m = static_cast<c_int>(m);
    data.P = P_mat;
    data.q = impl_->q_.data();
    data.A = A_mat;
    data.l = impl_->l_.data();
    data.u = impl_->u_.data();

    // ---- 配置求解器参数 ----
    OSQPSettings* settings = (OSQPSettings*)c_malloc(sizeof(OSQPSettings));
    if (settings == nullptr) {
        c_free(P_mat);
        c_free(A_mat);
        return false;
    }
    osqp_set_default_settings(settings);

    settings->rho = static_cast<c_float>(config.rho);
    settings->sigma = static_cast<c_float>(config.sigma);
    settings->alpha = static_cast<c_float>(config.alpha);
    settings->eps_abs = static_cast<c_float>(config.eps_abs);
    settings->eps_rel = static_cast<c_float>(config.eps_rel);
    settings->max_iter = static_cast<c_int>(config.max_iter);
    settings->verbose = config.verbose ? 1 : 0;
    settings->warm_start = config.warm_start ? 1 : 0;
    settings->polish = config.polish ? 1 : 0;
    settings->adaptive_rho = config.adaptive_rho ? 1 : 0;

    // ---- 初始化求解器 ----
    c_int exitflag = osqp_setup(&impl_->work_, &data, settings);

    // osqp_setup 已深拷贝 data 中的矩阵数据，可以安全释放临时 CSC 结构体
    c_free(P_mat);
    c_free(A_mat);
    c_free(settings);

    return exitflag == 0;
}

// ============================================================
// solve —— 执行求解并包装结果
// ============================================================
OsqpResult OsqpSolver::solve() {
    OsqpResult result;

    if (impl_->work_ == nullptr) {
        result.status = OsqpStatus::SetupError;
        result.status_msg = "solver not initialized; call setup() first";
        return result;
    }

    c_int exitflag = osqp_solve(impl_->work_);
    if (exitflag != 0) {
        result.status = OsqpStatus::SetupError;
        result.status_msg = "osqp_solve returned error";
        return result;
    }

    // 映射状态
    result.status = Impl::mapStatus(impl_->work_->info->status_val);
    result.status_msg = impl_->work_->info->status;
    result.iter = static_cast<int>(impl_->work_->info->iter);
    result.obj_val = static_cast<double>(impl_->work_->info->obj_val);

    // 拷贝原始变量解
    result.x.resize(static_cast<size_t>(impl_->n_));
    for (int i = 0; i < impl_->n_; ++i) {
        result.x[static_cast<size_t>(i)] =
            static_cast<double>(impl_->work_->solution->x[i]);
    }

    // 拷贝对偶变量
    result.y.resize(static_cast<size_t>(impl_->m_));
    for (int i = 0; i < impl_->m_; ++i) {
        result.y[static_cast<size_t>(i)] =
            static_cast<double>(impl_->work_->solution->y[i]);
    }

    return result;
}

// ============================================================
// updateBounds —— 热启动场景下仅更新线性项与约束边界
// ============================================================
bool OsqpSolver::updateBounds(const std::vector<double>& q_new,
                              const std::vector<double>& l_new,
                              const std::vector<double>& u_new) {
    if (impl_->work_ == nullptr) {
        return false;
    }
    // 将新数据拷贝到内部存储，保活
    impl_->q_.assign(q_new.begin(), q_new.end());
    impl_->l_.assign(l_new.begin(), l_new.end());
    impl_->u_.assign(u_new.begin(), u_new.end());

    c_int ret_q = osqp_update_lin_cost(impl_->work_, impl_->q_.data());
    c_int ret_b =
        osqp_update_bounds(impl_->work_, impl_->l_.data(), impl_->u_.data());

    return (ret_q == 0) && (ret_b == 0);
}

bool OsqpSolver::isReady() const { return impl_->work_ != nullptr; }

}  // namespace apa_post_processor
