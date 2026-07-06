#include "auto_adaptive_strategy.h"

#include <stdexcept>
#include <string>

#include "../qp/hpipm_solver.h"
#include "../sqp/sqp_algorithm.h"
#include "strategy_common.hpp"

namespace stc_SQP {
AutoAdaptiveStrategy::AutoAdaptiveStrategy(int nbx, int nbu, int ng, int ns, int block_size,
    int short_n_threshold)
    : block_size_(block_size)
    , short_n_threshold_(short_n_threshold)
    , nbx_(nbx)
    , nbu_(nbu)
    , ng_(ng)
    , ns_(ns)
{
    if (block_size_ <= 0) {
        throw std::invalid_argument("AutoAdaptiveStrategy: block_size must be greater than 0");
    }
    if (short_n_threshold_ < 0) {
        throw std::invalid_argument("AutoAdaptiveStrategy: short_n_threshold cannot be negative");
    }
    if (nbx_ < 0 || nbu_ < 0 || ng_ < 0 || ns_ < 0) {
        throw std::invalid_argument("AutoAdaptiveStrategy: nbx/nbu/ng/ns must be non-negative");
    }
}

bool AutoAdaptiveStrategy::solve(const MultiStageOCP& ocp, const Trajectory& initial_guess,
    Trajectory& solution)
{
    const int N = ocp.totalSteps();
    const int nx = ocp.nx();
    const int nu = ocp.nu();

    std::string reason;
    if (!strategy_internal::validateSolverDimensions(
            nbx_, nbu_, ng_, ns_, nx, nu, &reason)) {
        LOG_ERROR("AutoAdaptiveStrategy: dimension validation failed - ", reason);
        return false;
    }
    if (N <= 0) {
        LOG_ERROR("AutoAdaptiveStrategy: OCP total steps must be greater than 0");
        return false;
    }

    const int actual_ng = strategy_internal::computeOcpNgMax(ocp);
    if (ng_ != actual_ng) {
        LOG_ERROR("AutoAdaptiveStrategy: ng passed at construction=", ng_,
            " inconsistent with OCP actual general constraint dimension ", actual_ng);
        return false;
    }

    const int cond_N = strategy_internal::computeCondN(N, block_size_, short_n_threshold_,
        /*force_condensing=*/false);
    const bool use_omp = strategy_internal::computeUseOmp(N, short_n_threshold_);

    auto qp_solver = std::make_unique<HPIPMQPSolver>(N, nx, nu, nbx_, nbu_, ng_, ns_, cond_N);
    SQPSolver solver(std::move(qp_solver));
    solver.options().use_omp = use_omp;
    return solver.solve(ocp, initial_guess, solution);
}
} // namespace stc_SQP
