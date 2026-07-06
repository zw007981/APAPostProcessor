#include "manual_hierarchical_strategy.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

#include "../qp/hpipm_solver.h"
#include "../sqp/sqp_algorithm.h"
#include "strategy_common.hpp"

namespace stc_SQP {

ManualHierarchicalStrategy::ManualHierarchicalStrategy(int nbx, int nbu, int ng, int ns,
    const HierarchicalOptions& opts, int block_size, int short_n_threshold)
    : opts_(opts)
    , nbx_(nbx)
    , nbu_(nbu)
    , ng_(ng)
    , ns_(ns)
    , block_size_(block_size)
    , short_n_threshold_(short_n_threshold)
{
    if (nbx_ < 0 || nbu_ < 0 || ng_ < 0 || ns_ < 0) {
        throw std::invalid_argument("ManualHierarchicalStrategy: nbx/nbu/ng/ns must be non-negative");
    }
    if (block_size_ <= 0) {
        throw std::invalid_argument("ManualHierarchicalStrategy: block_size must be greater than 0");
    }
    if (short_n_threshold_ < 0) {
        throw std::invalid_argument("ManualHierarchicalStrategy: short_n_threshold cannot be negative");
    }
    if (opts_.coarse_max_iter <= 0 || opts_.fine_max_iter <= 0) {
        throw std::invalid_argument("ManualHierarchicalStrategy: max_iter must be greater than 0");
    }
}

bool ManualHierarchicalStrategy::solve(const MultiStageOCP& ocp,
    const Trajectory& initial_guess, Trajectory& solution)
{
    const int N = ocp.totalSteps();
    const int nx = ocp.nx();
    const int nu = ocp.nu();

    std::string reason;
    if (!strategy_internal::validateSolverDimensions(
            nbx_, nbu_, ng_, ns_, nx, nu, &reason)) {
        LOG_ERROR("ManualHierarchicalStrategy: dimension validation failed - ", reason);
        return false;
    }
    if (N <= 0) {
        LOG_ERROR("ManualHierarchicalStrategy: OCP total steps must be greater than 0");
        return false;
    }

    const int actual_ng = strategy_internal::computeOcpNgMax(ocp);
    if (ng_ != actual_ng) {
        LOG_ERROR("ManualHierarchicalStrategy: ng passed at construction=", ng_,
            " inconsistent with OCP actual general constraint dimension ", actual_ng);
        return false;
    }

    if (!opts_.enable) {
        return solveOnce(ocp, initial_guess, solution, opts_.fine_max_iter);
    }

    // 1. 构造粗 OCP（coarsen 保证段数不变）
    MultiStageOCP coarse_ocp = ocp.coarsen(opts_.coarse_n, opts_.coarse_dt);
    assert(coarse_ocp.segments().size() == ocp.segments().size());

    // 2. 粗优化（细初始猜测需下采样到粗长度）
    Trajectory coarse_init = downsampleTrajectory(initial_guess, ocp, coarse_ocp);
    Trajectory coarse_solution;
    if (!solveOnce(coarse_ocp, coarse_init, coarse_solution, opts_.coarse_max_iter)) {
        LOG_ERROR("ManualHierarchicalStrategy: coarse optimization failed");
        return false;
    }

    // 3. 动力学一致插值得到细 OCP 初始猜测
    Trajectory fine_guess = interpolateDynamicsConsistent(coarse_solution, coarse_ocp, ocp);

    // 4. 细优化
    return solveOnce(ocp, fine_guess, solution, opts_.fine_max_iter);
}

bool ManualHierarchicalStrategy::solveOnce(const MultiStageOCP& ocp,
    const Trajectory& initial_guess, Trajectory& solution, int max_iter) const
{
    const int N = ocp.totalSteps();
    const int nx = ocp.nx();
    const int nu = ocp.nu();

    std::string reason;
    if (!strategy_internal::validateSolverDimensions(
            nbx_, nbu_, ng_, ns_, nx, nu, &reason)) {
        LOG_ERROR("ManualHierarchicalStrategy::solveOnce: dimension validation failed - ", reason);
        return false;
    }
    if (N <= 0) {
        LOG_ERROR("ManualHierarchicalStrategy::solveOnce: OCP total steps must be greater than 0");
        return false;
    }

    const int actual_ng = strategy_internal::computeOcpNgMax(ocp);
    if (ng_ != actual_ng) {
        LOG_ERROR("ManualHierarchicalStrategy::solveOnce: ng passed at construction=", ng_,
            " inconsistent with OCP actual general constraint dimension ", actual_ng);
        return false;
    }

    // 粗/细 OCP 的 N 都可能较小；经验上 HPIPM 直接求解（cond_N=-1）在纯 LQR 等简单
    // 问题上偶发 MIN_STEP 失败，而 Partial Condensing 路径更稳健。因此对任意 N 都启用
    // condensing（N 极小时 cond_N=N，等价于无凝聚），保证策略端到端可用。
    const int cond_N = strategy_internal::computeCondN(
        N, block_size_, short_n_threshold_, /*force_condensing=*/true);
    const bool use_omp = strategy_internal::computeUseOmp(N, short_n_threshold_);

    auto qp_solver = std::make_unique<HPIPMQPSolver>(N, nx, nu, nbx_, nbu_, ng_, ns_, cond_N);
    SQPSolver solver(std::move(qp_solver));
    solver.options().use_omp = use_omp;
    solver.options().max_iter = max_iter;
    return solver.solve(ocp, initial_guess, solution);
}

Trajectory ManualHierarchicalStrategy::interpolateDynamicsConsistent(
    const Trajectory& coarse_solution, const MultiStageOCP& coarse_ocp,
    const MultiStageOCP& fine_ocp) const
{
    const int fine_N = fine_ocp.totalSteps();
    const int nx = fine_ocp.nx();
    const int nu = fine_ocp.nu();
    Trajectory fine_guess;
    fine_guess.resize(fine_N, nx, nu);

    int fine_global = 0;
    int coarse_global = 0;
    const size_t nseg = fine_ocp.segments().size();
    assert(nseg == coarse_ocp.segments().size());

    Vector x_next(nx);
    for (size_t s = 0; s < nseg; ++s) {
        const auto& fseg = fine_ocp.segments()[s];
        const auto& cseg = coarse_ocp.segments()[s];
        const int coarse_Ns = cseg.N;
        const double coarse_dt = cseg.dt;
        const double fine_dt = fseg.dt;

        // 段起始状态取自粗解，保证跨段连续性
        fine_guess.x[fine_global] = coarse_solution.x[coarse_global];

        for (int j = 0; j < fseg.N; ++j) {
            const double t = j * fine_dt;

            // 控制：在粗控制序列上做线性插值，alpha 限制在 [0, 1] 避免外推
            if (coarse_Ns == 1) {
                fine_guess.u[fine_global + j] = coarse_solution.u[coarse_global];
            } else {
                int l = std::min(coarse_Ns - 2, static_cast<int>(t / coarse_dt));
                const double alpha = std::clamp(
                    (t - l * coarse_dt) / coarse_dt, 0.0, 1.0);
                fine_guess.u[fine_global + j] =
                    (1.0 - alpha) * coarse_solution.u[coarse_global + l]
                    + alpha * coarse_solution.u[coarse_global + l + 1];
            }

            // 状态：用离散动力学传播，保证 x[k+1] = f(x[k], u[k])
            fseg.dynamics->discretize(
                fine_guess.x[fine_global + j], fine_guess.u[fine_global + j],
                fine_dt, fseg.v_sign, x_next);
            fine_guess.x[fine_global + j + 1] = x_next;
        }

        fine_global += fseg.N;
        coarse_global += coarse_Ns;
    }

    return fine_guess;
}

Trajectory ManualHierarchicalStrategy::downsampleTrajectory(const Trajectory& fine,
    const MultiStageOCP& fine_ocp, const MultiStageOCP& coarse_ocp) const
{
    const int coarse_N = coarse_ocp.totalSteps();
    const int nx = coarse_ocp.nx();
    const int nu = coarse_ocp.nu();
    Trajectory coarse;
    coarse.resize(coarse_N, nx, nu);

    int fine_global = 0;
    int coarse_global = 0;
    const size_t nseg = fine_ocp.segments().size();
    assert(nseg == coarse_ocp.segments().size());

    for (size_t s = 0; s < nseg; ++s) {
        const auto& fseg = fine_ocp.segments()[s];
        const auto& cseg = coarse_ocp.segments()[s];
        const double fine_dt = fseg.dt;
        const double coarse_dt = cseg.dt;

        for (int j = 0; j < cseg.N; ++j) {
            const double t = j * coarse_dt;
            int fine_j = static_cast<int>(std::round(t / fine_dt));
            fine_j = std::max(0, std::min(fseg.N - 1, fine_j));
            coarse.x[coarse_global + j] = fine.x[fine_global + fine_j];
            coarse.u[coarse_global + j] = fine.u[fine_global + fine_j];
        }
        coarse.x[coarse_global + cseg.N] = fine.x[fine_global + fseg.N];

        fine_global += fseg.N;
        coarse_global += cseg.N;
    }

    return coarse;
}

} // namespace stc_SQP
