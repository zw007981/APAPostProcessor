#include "hpipm_solver.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <vector>

#include "soft_constraint_validation.h"

#include <blasfeo_common.h>
#include <blasfeo_target.h>
#include <hpipm_common.h>
#include <hpipm_d_ocp_qp.h>
#include <hpipm_d_ocp_qp_dim.h>
#include <hpipm_d_ocp_qp_ipm.h>
#include <hpipm_d_ocp_qp_sol.h>
#include <hpipm_d_ocp_qp_solver.h>
#include <hpipm_d_part_cond.h>

namespace stc_SQP {
namespace {
    // 自定义释放器：配合 std::aligned_alloc 使用 std::free
    struct AlignedDeleter {
        void operator()(void* p) const
        {
            std::free(p);
        }
    };
    // 64 字节对齐的内存缓冲区，满足 HPIPM/BLASFEO 内部对齐假设
    struct AlignedBuffer {
        AlignedBuffer() = default;
        explicit AlignedBuffer(size_t size)
        {
            // std::aligned_alloc 要求 size 为对齐值的整数倍
            const size_t aligned_size = (size + 63) & ~size_t(63);
            void* raw = std::aligned_alloc(64, aligned_size);
            if (!raw) {
                throw std::bad_alloc();
            }
            ptr.reset(static_cast<char*>(raw));
        }
        char* data() const { return ptr.get(); }
        std::unique_ptr<char, AlignedDeleter> ptr;
    };
} // namespace

struct HPIPMQPSolver::Impl {
    // ====================================================
    // OCP QP 维度参数（与 QPData 维度对应）
    // ====================================================
    // 预测时域步数
    int N = 0;
    // 状态维度
    int nx = 0;
    // 控制维度
    int nu = 0;
    // 带边界的状态维度（<= nx）
    int nbx = 0;
    // 带边界的控制维度（<= nu）
    int nbu = 0;
    // 普通约束维度
    int ng = 0;
    // 每步软约束维度
    int ns = 0;
    // 凝聚后的宏观步数，-1 表示不启用
    int cond_N = -1;
    // 是否启用 Partial Condensing（cond_N > 0 且 cond_N < N）
    bool use_partial_condensing = false;
    // 求解精度
    double tol = 1e-12;
    // ====================================================
    // HPIPM C 结构体：描述 QP 维度、QP 本体、解、求解器参数与工作空间
    // ====================================================
    struct d_ocp_qp_dim dim;
    struct d_ocp_qp qp;
    struct d_ocp_qp_sol qp_sol;
    struct d_ocp_qp_solver_arg arg;
    struct d_ocp_qp_solver_ws ws;
    // 64 字节对齐内存缓冲，分别对应上述 HPIPM C 结构体
    AlignedBuffer dim_mem;
    AlignedBuffer qp_mem;
    AlignedBuffer qp_sol_mem;
    AlignedBuffer arg_mem;
    AlignedBuffer ws_mem;
    // ====================================================
    // Partial Condensing 专用资源（仅当 use_partial_condensing 时创建）
    // ====================================================
    // 每块原始步数（长度 cond_N）
    std::vector<int> block_size;
    // 凝聚后维度 / QP / 解
    struct d_ocp_qp_dim cond_dim;
    struct d_ocp_qp cond_qp;
    struct d_ocp_qp_sol cond_qp_sol;
    // Partial Condensing 参数与工作空间
    struct d_part_cond_qp_arg cond_arg;
    struct d_part_cond_qp_ws cond_ws;
    // 凝聚后 QP 的 IPM 参数与工作空间（因 solver API 不直接支持 condensing）
    struct d_ocp_qp_ipm_arg ipm_arg;
    struct d_ocp_qp_ipm_ws ipm_ws;
    AlignedBuffer cond_dim_mem;
    AlignedBuffer cond_qp_mem;
    AlignedBuffer cond_qp_sol_mem;
    AlignedBuffer cond_arg_mem;
    AlignedBuffer cond_ws_mem;
    AlignedBuffer ipm_arg_mem;
    AlignedBuffer ipm_ws_mem;
    // 盒式边界变量索引（每个阶段重复使用的 0..nbx-1 / 0..nbu-1）
    std::vector<std::vector<int>> idxbx;
    std::vector<std::vector<int>> idxbu;
    // ====================================================
    // 可复用临时缓冲，在构造期按维度预分配，solve() 中只填值复用
    // ====================================================
    // 普通约束 D 矩阵备份：按阶段持久化，避免 C API 保存指针时各阶段指向同一块内存
    std::vector<std::vector<double>> D_backup;
    // 普通约束上界备份（将 +inf 裁剪为大有限数，避免 HPIPM 内部出现非有限值）
    std::vector<std::vector<double>> ug_backup;
    // box 边界备份（将 ±inf 裁剪为大有限数，保证 HPIPM 数值稳定）
    std::vector<std::vector<double>> lbx_backup;
    std::vector<std::vector<double>> ubx_backup;
    std::vector<std::vector<double>> lbu_backup;
    std::vector<std::vector<double>> ubu_backup;
    // 普通约束下界掩码，恒为 0 以禁用下界
    std::vector<double> lg_mask;
    // 软约束在 HPIPM 全局约束索引中的位置
    std::vector<int> soft_idxs;
    // 软约束下界 L2 权重
    std::vector<double> Zl;
    // 软约束上界 L2 权重
    std::vector<double> Zu;
    // 软约束下界 L1 权重
    std::vector<double> zl;
    // 软约束上界 L1 权重
    std::vector<double> zu;
    // HPIPM 软约束 reverse map（未软化位置为 -1）
    std::vector<int> idxs_rev;
};

HPIPMQPSolver::HPIPMQPSolver(int N, int nx, int nu, int nbx, int nbu, int ng,
    int ns, int cond_N)
    : pimpl_(std::make_unique<Impl>())
{
    if (N <= 0 || nx <= 0 || nu <= 0 || ng < 0 || ns < 0) {
        throw std::invalid_argument("HPIPMQPSolver: dimensions must be positive (ng/ns may be 0)");
    }
    if (nbx < 0 || nbx > nx || nbu < 0 || nbu > nu) {
        throw std::invalid_argument("HPIPMQPSolver: nbx/nbu out of valid range");
    }
    if (cond_N < -1 || cond_N == 0 || cond_N > N) {
        throw std::invalid_argument(
            "HPIPMQPSolver: cond_N must be -1 or a macro-step in [1, N]");
    }
    pimpl_->N = N;
    pimpl_->nx = nx;
    pimpl_->nu = nu;
    pimpl_->nbx = nbx;
    pimpl_->nbu = nbu;
    pimpl_->ng = ng;
    pimpl_->ns = ns;
    pimpl_->cond_N = cond_N;
    // 因此该组合在构造期即回退到无凝聚路径（与 cond_N=N 等价）。
    // 生产环境中 corridor 约束未启用软约束，此回退不影响性能验收。
    pimpl_->use_partial_condensing = (cond_N > 0 && cond_N < N && ns == 0);

    // 构造原始 OCP QP 维度
    pimpl_->dim_mem = AlignedBuffer(d_ocp_qp_dim_memsize(N));
    d_ocp_qp_dim_create(N, &pimpl_->dim, pimpl_->dim_mem.data());
    std::vector<int> nx_arr(N + 1, nx);
    std::vector<int> nu_arr(N + 1, 0);
    for (int k = 0; k < N; ++k) {
        nu_arr[k] = nu;
    }
    std::vector<int> nbx_arr(N + 1, nbx);
    std::vector<int> nbu_arr(N + 1, 0);
    for (int k = 0; k < N; ++k) {
        nbu_arr[k] = nbu;
    }
    std::vector<int> ng_arr(N + 1, 0);
    for (int k = 0; k < N; ++k) {
        ng_arr[k] = ng;
    }
    std::vector<int> ns_arr(N + 1, 0);
    for (int k = 0; k < N; ++k) {
        ns_arr[k] = ns;
    }
    d_ocp_qp_dim_set_all(nx_arr.data(), nu_arr.data(), nbx_arr.data(),
        nbu_arr.data(), ng_arr.data(), ns_arr.data(), &pimpl_->dim);

    // 构造原始 QP 与解结构
    pimpl_->qp_mem = AlignedBuffer(d_ocp_qp_memsize(&pimpl_->dim));
    d_ocp_qp_create(&pimpl_->dim, &pimpl_->qp, pimpl_->qp_mem.data());
    pimpl_->qp_sol_mem = AlignedBuffer(d_ocp_qp_sol_memsize(&pimpl_->dim));
    d_ocp_qp_sol_create(&pimpl_->dim, &pimpl_->qp_sol, pimpl_->qp_sol_mem.data());

    if (!pimpl_->use_partial_condensing) {
        // 无凝聚：使用 HPIPM solver API
        pimpl_->arg_mem = AlignedBuffer(d_ocp_qp_solver_arg_memsize(&pimpl_->dim));
        d_ocp_qp_solver_arg_create(&pimpl_->dim, &pimpl_->arg, pimpl_->arg_mem.data());
        d_ocp_qp_solver_arg_set_default(ROBUST, &pimpl_->dim, &pimpl_->arg);
        pimpl_->ws_mem = AlignedBuffer(d_ocp_qp_solver_ws_memsize(&pimpl_->dim, &pimpl_->arg));
        d_ocp_qp_solver_ws_create(&pimpl_->dim, &pimpl_->arg, &pimpl_->ws, pimpl_->ws_mem.data());
    } else {
        // Partial Condensing：cond_N 为宏观步数，block_size 由 HPIPM 自动计算
        pimpl_->block_size.resize(cond_N + 1);
        d_part_cond_qp_compute_block_size(N, cond_N, pimpl_->block_size.data());

        pimpl_->cond_dim_mem = AlignedBuffer(d_ocp_qp_dim_memsize(cond_N));
        d_ocp_qp_dim_create(cond_N, &pimpl_->cond_dim, pimpl_->cond_dim_mem.data());
        d_part_cond_qp_compute_dim(
            &pimpl_->dim, pimpl_->block_size.data(), &pimpl_->cond_dim);

        pimpl_->cond_qp_mem = AlignedBuffer(d_ocp_qp_memsize(&pimpl_->cond_dim));
        d_ocp_qp_create(&pimpl_->cond_dim, &pimpl_->cond_qp, pimpl_->cond_qp_mem.data());
        pimpl_->cond_qp_sol_mem = AlignedBuffer(d_ocp_qp_sol_memsize(&pimpl_->cond_dim));
        d_ocp_qp_sol_create(
            &pimpl_->cond_dim, &pimpl_->cond_qp_sol, pimpl_->cond_qp_sol_mem.data());

        pimpl_->cond_arg_mem = AlignedBuffer(d_part_cond_qp_arg_memsize(cond_N));
        d_part_cond_qp_arg_create(cond_N, &pimpl_->cond_arg, pimpl_->cond_arg_mem.data());
        d_part_cond_qp_arg_set_default(&pimpl_->cond_arg);

        pimpl_->cond_ws_mem = AlignedBuffer(
            d_part_cond_qp_ws_memsize(&pimpl_->dim, pimpl_->block_size.data(),
                &pimpl_->cond_dim, &pimpl_->cond_arg));
        d_part_cond_qp_ws_create(&pimpl_->dim, pimpl_->block_size.data(), &pimpl_->cond_dim,
            &pimpl_->cond_arg, &pimpl_->cond_ws, pimpl_->cond_ws_mem.data());

        // 凝聚后 QP 使用 IPM 直接求解
        pimpl_->ipm_arg_mem = AlignedBuffer(d_ocp_qp_ipm_arg_memsize(&pimpl_->cond_dim));
        d_ocp_qp_ipm_arg_create(
            &pimpl_->cond_dim, &pimpl_->ipm_arg, pimpl_->ipm_arg_mem.data());
        d_ocp_qp_ipm_arg_set_default(ROBUST, &pimpl_->ipm_arg);

        pimpl_->ipm_ws_mem = AlignedBuffer(
            d_ocp_qp_ipm_ws_memsize(&pimpl_->cond_dim, &pimpl_->ipm_arg));
        d_ocp_qp_ipm_ws_create(
            &pimpl_->cond_dim, &pimpl_->ipm_arg, &pimpl_->ipm_ws, pimpl_->ipm_ws_mem.data());
    }
    // 预生成边界变量索引
    pimpl_->idxbx.resize(N + 1);
    pimpl_->idxbu.resize(N + 1);
    for (int k = 0; k <= N; ++k) {
        pimpl_->idxbx[k].resize(nbx);
        for (int i = 0; i < nbx; ++i) {
            pimpl_->idxbx[k][i] = i;
        }
    }
    for (int k = 0; k <= N; ++k) {
        pimpl_->idxbu[k].resize(nbu);
        for (int i = 0; i < nbu; ++i) {
            pimpl_->idxbu[k][i] = i;
        }
    }
    // 预分配可复用临时缓冲：按阶段独立存储，防止 HPIPM setter 保存指针导致别名。
    // 构造期直接 resize 到固定长度，solve() 中只通过索引/拷贝写入，避免实时路径动态分配。
    if (ng > 0 && nu > 0) {
        pimpl_->D_backup.resize(N);
        for (auto& buf : pimpl_->D_backup) {
            buf.resize(ng * nu);
        }
    }
    if (ng > 0) {
        pimpl_->lg_mask.assign(ng, 0.0);
        pimpl_->ug_backup.resize(N);
        for (auto& buf : pimpl_->ug_backup) {
            buf.resize(ng);
        }
    }
    if (nbx > 0) {
        pimpl_->lbx_backup.resize(N + 1);
        pimpl_->ubx_backup.resize(N + 1);
        for (auto& buf : pimpl_->lbx_backup) {
            buf.resize(nbx);
        }
        for (auto& buf : pimpl_->ubx_backup) {
            buf.resize(nbx);
        }
    }
    if (nbu > 0) {
        pimpl_->lbu_backup.resize(N);
        pimpl_->ubu_backup.resize(N);
        for (auto& buf : pimpl_->lbu_backup) {
            buf.resize(nbu);
        }
        for (auto& buf : pimpl_->ubu_backup) {
            buf.resize(nbu);
        }
    }
    if (ns > 0) {
        pimpl_->soft_idxs.resize(ns);
        pimpl_->Zl.resize(ns);
        pimpl_->Zu.resize(ns);
        pimpl_->zl.resize(ns);
        pimpl_->zu.resize(ns);
    }
    if (nbu + nbx + ng > 0) {
        pimpl_->idxs_rev.assign(nbu + nbx + ng, -1);
    }
}

HPIPMQPSolver::~HPIPMQPSolver() = default;

void HPIPMQPSolver::setTolerance(double tol)
{
    if (!std::isfinite(tol) || tol <= 0.0) {
        throw std::invalid_argument("HPIPMQPSolver: tolerance must be a finite positive number");
    }
    pimpl_->tol = tol;
}

void HPIPMQPSolver::setWarmStart(const QPSolution&)
{
}

QPSolverStatus HPIPMQPSolver::mapHpipmStatus(int status) const
{
    switch (status) {
    case SUCCESS:
        return QPSolverStatus::SUCCESS;
    case MAX_ITER:
        return QPSolverStatus::MAX_ITER_REACHED;
    case MIN_STEP:
        return QPSolverStatus::UNKNOWN_ERROR;
    case NAN_SOL:
        return QPSolverStatus::NAN_IN_SOLUTION;
    case INCONS_EQ:
        return QPSolverStatus::INFEASIBLE;
    default:
        return QPSolverStatus::UNKNOWN_ERROR;
    }
}

QPSolverStatus HPIPMQPSolver::solve(const QPData& qp_data, QPSolution& qp_sol)
{
    // 校验 QPData 维度与求解器构造维度一致
    if (qp_data.N != pimpl_->N || qp_data.nx != pimpl_->nx || qp_data.nu != pimpl_->nu || qp_data.ng_max != pimpl_->ng) {
        return QPSolverStatus::INVALID_ARGUMENT;
    }
    // 校验 QPData 内部容器尺寸，防止外部修改 N 或清空 vector 后导致越界访问
    if (!qp_data.hasValidContainerSizes()) {
        return QPSolverStatus::INVALID_ARGUMENT;
    }
    const int N = pimpl_->N;
    const int nx = pimpl_->nx;
    const int nu = pimpl_->nu;
    const int nbx = pimpl_->nbx;
    const int nbu = pimpl_->nbu;
    const int ng = pimpl_->ng;
    const int ns = pimpl_->ns;
    const bool has_soft = ns > 0;
    // 软约束维度与存在性校验：防止越界索引或配置被静默忽略
    if (has_soft) {
        if (qp_data.soft_config == nullptr || qp_data.soft_config->ns != ns || !soft_constraint_validation::validate(*qp_data.soft_config, ng, ns)) {
            return QPSolverStatus::INVALID_ARGUMENT;
        }
    } else if (qp_data.soft_config != nullptr && !soft_constraint_validation::validate(*qp_data.soft_config, ng, 0)) {
        return QPSolverStatus::INVALID_ARGUMENT;
    }
    // 设置 QP 数据
    for (int k = 0; k < N; ++k) {
        d_ocp_qp_set_A(k, const_cast<double*>(qp_data.rawA(k)), &pimpl_->qp);
        d_ocp_qp_set_B(k, const_cast<double*>(qp_data.rawB(k)), &pimpl_->qp);
        d_ocp_qp_set_b(k, const_cast<double*>(qp_data.rawb(k)), &pimpl_->qp);
        d_ocp_qp_set_Q(k, const_cast<double*>(qp_data.rawQ(k)), &pimpl_->qp);
        d_ocp_qp_set_R(k, const_cast<double*>(qp_data.rawR(k)), &pimpl_->qp);
        d_ocp_qp_set_S(k, const_cast<double*>(qp_data.rawS(k)), &pimpl_->qp);
        d_ocp_qp_set_q(k, const_cast<double*>(qp_data.rawq(k)), &pimpl_->qp);
        d_ocp_qp_set_r(k, const_cast<double*>(qp_data.rawr(k)), &pimpl_->qp);
    }
    d_ocp_qp_set_Q(N, const_cast<double*>(qp_data.rawQ(N)), &pimpl_->qp);
    d_ocp_qp_set_q(N, const_cast<double*>(qp_data.rawq(N)), &pimpl_->qp);
    // 盒式边界：HPIPM C API 需要非 const 指针，且不能接收 ±inf，因此裁剪为大有限数
    constexpr double k_inf_bound = 1e12;
    auto clip_bound = [k_inf_bound](double v) {
        if (std::isinf(v)) {
            return v > 0.0 ? k_inf_bound : -k_inf_bound;
        }
        return v;
    };
    for (int k = 0; k <= N; ++k) {
        if (nbx > 0) {
            const double* raw_lbx = qp_data.rawLbx(k);
            const double* raw_ubx = qp_data.rawUbx(k);
            for (int i = 0; i < nbx; ++i) {
                pimpl_->lbx_backup[k][i] = clip_bound(raw_lbx[i]);
                pimpl_->ubx_backup[k][i] = clip_bound(raw_ubx[i]);
            }
            d_ocp_qp_set_idxbx(k, pimpl_->idxbx[k].data(), &pimpl_->qp);
            d_ocp_qp_set_lbx(k, pimpl_->lbx_backup[k].data(), &pimpl_->qp);
            d_ocp_qp_set_ubx(k, pimpl_->ubx_backup[k].data(), &pimpl_->qp);
        }
        if (nbu > 0 && k < N) {
            const double* raw_lbu = qp_data.rawLbu(k);
            const double* raw_ubu = qp_data.rawUbu(k);
            for (int i = 0; i < nbu; ++i) {
                pimpl_->lbu_backup[k][i] = clip_bound(raw_lbu[i]);
                pimpl_->ubu_backup[k][i] = clip_bound(raw_ubu[i]);
            }
            d_ocp_qp_set_idxbu(k, pimpl_->idxbu[k].data(), &pimpl_->qp);
            d_ocp_qp_set_lbu(k, pimpl_->lbu_backup[k].data(), &pimpl_->qp);
            d_ocp_qp_set_ubu(k, pimpl_->ubu_backup[k].data(), &pimpl_->qp);
        }
    }
    // 普通约束：C x + D u <= d；下界通过 mask=0 禁用，避免引入人工下界
    for (int k = 0; k < N; ++k) {
        if (ng > 0) {
            d_ocp_qp_set_C(k, const_cast<double*>(qp_data.rawC(k)), &pimpl_->qp);
            if (nu > 0) {
                // HPIPM C API 需要非 const double*，而 QPData 只暴露 const 原始指针，
                // 因此复制到按阶段独立的备份缓冲区（构造期已 resize，此处只写入）。
                const double* raw_d = qp_data.rawD(k);
                std::copy(raw_d, raw_d + ng * nu, pimpl_->D_backup[k].begin());
                d_ocp_qp_set_D(k, pimpl_->D_backup[k].data(), &pimpl_->qp);
            }
            d_ocp_qp_set_lg_mask(k, pimpl_->lg_mask.data(), &pimpl_->qp);
            // 将 +inf 裁剪为大有限数，保证 HPIPM 数值稳定；Dense 求解器可直接跳过 +inf。
            const double* raw_ug = qp_data.rawd(k);
            constexpr double k_inf_replace = 1e12;
            for (int j = 0; j < ng; ++j) {
                pimpl_->ug_backup[k][j] =
                    std::isinf(raw_ug[j]) ? k_inf_replace : raw_ug[j];
            }
            d_ocp_qp_set_ug(k, pimpl_->ug_backup[k].data(), &pimpl_->qp);
        }
    }
    // 软约束：idxs 需要转换为 HPIPM 的“全部双边约束”中的全局位置
    if (has_soft) {
        for (int j = 0; j < ns; ++j) {
            // 普通约束在 [lbu, ubu, lbx, ubx, lg, ug] 中的偏移为 nbu + nbx
            pimpl_->soft_idxs[j] = nbu + nbx + qp_data.soft_config->idxs[j];
        }
        for (int j = 0; j < ns; ++j) {
            pimpl_->Zl[j] = qp_data.soft_config->Zl(j);
            pimpl_->Zu[j] = qp_data.soft_config->Zu(j);
            pimpl_->zl[j] = qp_data.soft_config->zl(j);
            pimpl_->zu[j] = qp_data.soft_config->zu(j);
        }
        // idxs_rev 为 HPIPM 内部 reverse map：被软化位置填其在 idxs 中的序号，
        // 未软化位置保持 -1。
        std::fill(pimpl_->idxs_rev.begin(), pimpl_->idxs_rev.end(), -1);
        for (int j = 0; j < ns; ++j) {
            pimpl_->idxs_rev[pimpl_->soft_idxs[j]] = j;
        }
        for (int k = 0; k < N; ++k) {
            d_ocp_qp_set_Zl(k, pimpl_->Zl.data(), &pimpl_->qp);
            d_ocp_qp_set_Zu(k, pimpl_->Zu.data(), &pimpl_->qp);
            d_ocp_qp_set_zl(k, pimpl_->zl.data(), &pimpl_->qp);
            d_ocp_qp_set_zu(k, pimpl_->zu.data(), &pimpl_->qp);
            d_ocp_qp_set_idxs_rev(k, pimpl_->idxs_rev.data(), &pimpl_->qp);
            d_ocp_qp_set_idxs(k, pimpl_->soft_idxs.data(), &pimpl_->qp);
        }
    }
    // 设置求解精度与迭代上限，并执行求解
    int iter_max = 1000;
    // HPIPM 内部对极接近 0 的变量仍有约 1e-7 的互补残差，因此 tol 不小于 1e-12
    double tol = std::max(pimpl_->tol, 1e-12);
    int status = 0;
    if (!pimpl_->use_partial_condensing) {
        d_ocp_qp_solver_set_iter_max(&iter_max, &pimpl_->ws);
        d_ocp_qp_solver_set_tol_stat(&tol, &pimpl_->ws);
        d_ocp_qp_solver_set_tol_eq(&tol, &pimpl_->ws);
        d_ocp_qp_solver_set_tol_ineq(&tol, &pimpl_->ws);
        d_ocp_qp_solver_set_tol_comp(&tol, &pimpl_->ws);
        d_ocp_qp_solver_solve(&pimpl_->qp, &pimpl_->qp_sol, &pimpl_->ws);
        d_ocp_qp_solver_get_status(&pimpl_->ws, &status);
    } else {
        // Partial Condensing：凝聚 -> 求解凝聚后 QP -> 展开解
        d_ocp_qp_ipm_arg_set_iter_max(&iter_max, &pimpl_->ipm_arg);
        d_ocp_qp_ipm_arg_set_tol_stat(&tol, &pimpl_->ipm_arg);
        d_ocp_qp_ipm_arg_set_tol_eq(&tol, &pimpl_->ipm_arg);
        d_ocp_qp_ipm_arg_set_tol_ineq(&tol, &pimpl_->ipm_arg);
        d_ocp_qp_ipm_arg_set_tol_comp(&tol, &pimpl_->ipm_arg);
        d_part_cond_qp_cond(
            &pimpl_->qp, &pimpl_->cond_qp, &pimpl_->cond_arg, &pimpl_->cond_ws);
        d_ocp_qp_ipm_solve(
            &pimpl_->cond_qp, &pimpl_->cond_qp_sol, &pimpl_->ipm_arg, &pimpl_->ipm_ws);
        d_ocp_qp_ipm_get_status(&pimpl_->ipm_ws, &status);
        if (status == SUCCESS) {
            d_part_cond_qp_expand_sol(&pimpl_->qp, &pimpl_->cond_qp_sol, &pimpl_->qp_sol,
                &pimpl_->cond_arg, &pimpl_->cond_ws);
        }
    }
    if (status != SUCCESS) {
        return mapHpipmStatus(status);
    }
    // 提取解
    qp_sol.resize(N, nx, nu, ns);
    for (int k = 0; k <= N; ++k) {
        d_ocp_qp_sol_get_x(k, &pimpl_->qp_sol, qp_sol.x[k].data());
    }
    for (int k = 0; k < N; ++k) {
        d_ocp_qp_sol_get_u(k, &pimpl_->qp_sol, qp_sol.u[k].data());
    }
    if (has_soft) {
        for (int k = 0; k < N; ++k) {
            d_ocp_qp_sol_get_su(k, &pimpl_->qp_sol, qp_sol.s[k].data());
        }
    }
    return QPSolverStatus::SUCCESS;
}
} // namespace stc_SQP
