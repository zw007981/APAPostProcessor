#include "sqp_algorithm.h"

#include <algorithm>
#include <cassert>
#include <limits>
#include <vector>

#ifdef _OPENMP
#include <omp.h>
#endif

#include "../util/constants.h"

namespace {
// 获取第 segment 第 step_in_segment 步的显式参数 p；若 stage_params 为空则返回空向量。
const stc_SQP::Vector& getStageParameter(const stc_SQP::StageSegment& segment,
    int step_in_segment)
{
    if (!segment.stage_params.empty()) {
        assert(step_in_segment >= 0
            && step_in_segment < static_cast<int>(segment.stage_params.size()));
        return segment.stage_params[step_in_segment].p;
    }
    static const stc_SQP::Vector kEmptyParameter;
    return kEmptyParameter;
}
} // namespace

namespace stc_SQP {
SQPSolver::SQPSolver(std::unique_ptr<QPSolver> qp_solver)
    : qp_solver_(std::move(qp_solver))
{
}

void SQPSolver::setExternalQPData(std::unique_ptr<QPData> qp_data) {
    external_qp_data_ = std::move(qp_data);
}

std::unique_ptr<QPData> SQPSolver::takeQPData() {
    return std::move(qp_data_);
}

bool SQPSolver::solve(const MultiStageOCP& ocp,
    const Trajectory& initial_guess, Trajectory& solution)
{
    ocp_ = &ocp;
    iter_count_ = 0;
    rti_downgraded_ = false;
    converged_ = false;

    // 清理 OMP 约束克隆池：每次 solve() 重新克隆，避免前一次求解遗留的
    // 约束状态（如迭代走廊的内部迭代器）在第二次 solve 中产生不一致行为。
    thread_constraint_clones_.clear();

    std::string reason;
    if (!validateProblem(ocp, initial_guess, &reason)) {
        LOG_ERROR("Problem validation failed: ", reason);
        return false;
    }

    // RTI 降级逻辑：泊车含换挡点时必须使用 Full SQP
    // 使用局部变量，避免永久改写用户传入的 options_.use_rti
    bool use_rti_this_solve = options_.use_rti;
    if (ocp.hasGearShift()) {
        if (use_rti_this_solve) {
            use_rti_this_solve = false;
            rti_downgraded_ = true;
            LOG_WARN("Gear shift detected in OCP, forcing use_rti to false and switching to Full SQP");
        }
    }

    const int N = ocp.totalSteps();
    const int nx = ocp.nx();
    const int nu = ocp.nu();
    rti_mode_active_ = use_rti_this_solve;
    current_traj_ = initial_guess;
    delta_traj_.resize(N, nx, nu);
    cost_q_.resize(nx);
    cost_r_.resize(nu);
    cost_Q_.resize(nx, nx);
    cost_R_.resize(nu, nu);
    cost_S_.resize(nu, nx);
    lin_x_next_.resize(nx);
    lin_A_.resize(nx, nx);
    lin_B_.resize(nx, nu);

    // 计算普通约束维度上限
    int ng_max = 0;
    for (const auto& segment : ocp.segments()) {
        int ng_seg = 0;
        for (const auto& constraint : segment.constraints) {
            ng_seg += constraint ? constraint->ng() : 0;
        }
        ng_max = std::max(ng_max, ng_seg);
    }

    // 若外部已注入 QPData 且维度匹配，则复用；否则内部新建。
    if (external_qp_data_) {
        if (external_qp_data_->N == N && external_qp_data_->nx == nx &&
            external_qp_data_->nu == nu &&
            external_qp_data_->ng_max == ng_max) {
            qp_data_ = std::move(external_qp_data_);
            qp_data_->reset();
        } else {
            LOG_WARN("External QPData dimension mismatch, falling back to new allocation");
            external_qp_data_.reset();
            qp_data_ = std::make_unique<QPData>(N, nx, nu, ng_max);
        }
    } else {
        qp_data_ = std::make_unique<QPData>(N, nx, nu, ng_max);
    }
    const int qp_slack_dim = qp_solver_ ? qp_solver_->slackDim() : 0;
    qp_solution_.resize(N, nx, nu, qp_slack_dim);
    mutable_g_.resize(ng_max);
    zero_u_.resize(nu);
    zero_u_.setZero();

    // 若底层 QP 求解器支持软约束且用户已配置，则把软约束配置透传给 QPData。
    // HPIPM 求解器在 solve() 内会校验 ns、idxs 与 ng_max 的合法性。
    if (qp_slack_dim > 0 &&
        options_.soft_constraint_config.ns == qp_slack_dim) {
        qp_data_->soft_config = std::make_unique<SoftConstraintConfig>(
            options_.soft_constraint_config);
    }

    // 注意：每线程 clone 副本在 solve 入口基于当前约束/代价生成；若上层在两次 solve 之间
    //      更新了约束参数，下一次 solve 会重新 clone，保证参数不陈旧。
    const bool use_omp_this_solve = [this, N]() {
#ifdef _OPENMP
        return options_.use_omp && N >= options_.omp_parallel_threshold;
#else
        (void)N;
        return false;
#endif
    }();
    if (use_omp_this_solve) {
        int max_threads = 1;
#ifdef _OPENMP
        max_threads = omp_get_max_threads();
#endif
        thread_constraint_clones_.resize(max_threads);
        thread_cost_clones_.resize(max_threads);
        const int num_segments = static_cast<int>(ocp.segments().size());
        for (int t = 0; t < max_threads; ++t) {
            thread_constraint_clones_[t].resize(num_segments);
            thread_cost_clones_[t].resize(num_segments);
            for (int seg_idx = 0; seg_idx < num_segments; ++seg_idx) {
                const auto& segment = ocp.segments()[seg_idx];
                for (const auto& constraint : segment.constraints) {
                    if (constraint) {
                        thread_constraint_clones_[t][seg_idx].push_back(constraint->clone());
                    }
                }
                if (segment.cost) {
                    thread_cost_clones_[t][seg_idx] = segment.cost->clone();
                }
            }
        }
    } else {
        thread_constraint_clones_.clear();
        thread_cost_clones_.clear();
    }

    if (use_rti_this_solve) {
        // RTI 模式：仅执行一次线性化 + QP + 更新；收敛判断不适用
        iter_count_ = 0;
        if (!iterate()) {
            LOG_ERROR("RTI single iteration failed");
            return false;
        }
        converged_ = true; // RTI 语义：单步 QP 成功即视为完成
        solution = current_traj_;
        return true;
    }

    // Full SQP 模式：迭代至收敛或 max_iter。
    // 未收敛时仍把 current_traj_ 作为 last iterate 写入 solution 并返回 false。

    for (iter_count_ = 0; iter_count_ < options_.max_iter; ++iter_count_) {
        if (!iterate()) {
            LOG_ERROR("SQP iteration failed at iteration ", iter_count_);
            return false;
        }
        if (checkConvergence()) {
            converged_ = true;
            break;
        }
    }

    solution = current_traj_;
    return converged_;
}

bool SQPSolver::validateProblem(const MultiStageOCP& ocp,
    const Trajectory& initial_guess, std::string* reason) const
{
    auto set_reason = [reason](const char* msg) {
        if (reason != nullptr) {
            *reason = msg;
        }
    };
    std::string ocp_reason;
    if (!ocp.validate(&ocp_reason)) {
        set_reason(ocp_reason.c_str());
        return false;
    }
    if (options_.max_iter <= 0) {
        set_reason("max_iter must be greater than 0");
        return false;
    }
    if (!std::isfinite(options_.line_search_rho) || options_.line_search_rho <= 0.0
        || options_.line_search_rho >= 1.0) {
        set_reason("line_search_rho must be in (0, 1)");
        return false;
    }
    if (!std::isfinite(options_.line_search_c) || options_.line_search_c <= 0.0
        || options_.line_search_c >= 1.0) {
        set_reason("line_search_c must be in (0, 1)");
        return false;
    }
    if (!std::isfinite(options_.line_search_alpha_min)
        || options_.line_search_alpha_min <= 0.0
        || options_.line_search_alpha_min > 1.0) {
        set_reason("line_search_alpha_min must be in (0, 1]");
        return false;
    }
    if (!std::isfinite(options_.merit_penalty) || options_.merit_penalty < 0.0) {
        set_reason("merit_penalty must be a non-negative finite number");
        return false;
    }
    if (!std::isfinite(options_.kkt_tol) || options_.kkt_tol <= 0.0) {
        set_reason("kkt_tol must be a finite positive number");
        return false;
    }
    if (!std::isfinite(options_.constr_viol_tol) || options_.constr_viol_tol <= 0.0) {
        set_reason("constr_viol_tol must be a finite positive number");
        return false;
    }
    if (!std::isfinite(options_.stationarity_tol) || options_.stationarity_tol <= 0.0) {
        set_reason("stationarity_tol must be a finite positive number");
        return false;
    }
    if (!std::isfinite(options_.reg_min) || options_.reg_min <= 0.0) {
        set_reason("reg_min must be a finite positive number");
        return false;
    }
    const int N = ocp.totalSteps();
    const int nx = ocp.nx();
    const int nu = ocp.nu();
    if (static_cast<int>(initial_guess.x.size()) != N + 1) {
        set_reason("initial guess state sequence length must be N+1");
        return false;
    }
    if (static_cast<int>(initial_guess.u.size()) != N) {
        set_reason("initial guess control sequence length must be N");
        return false;
    }
    for (int k = 0; k <= N; ++k) {
        if (initial_guess.x[k].size() != nx) {
            set_reason("initial guess state vector dimension mismatch with nx");
            return false;
        }
        if (!initial_guess.x[k].allFinite()) {
            set_reason("initial guess state contains non-finite values (NaN/Inf)");
            return false;
        }
    }
    for (int k = 0; k < N; ++k) {
        if (initial_guess.u[k].size() != nu) {
            set_reason("initial guess control vector dimension mismatch with nu");
            return false;
        }
        if (!initial_guess.u[k].allFinite()) {
            set_reason("initial guess control contains non-finite values (NaN/Inf)");
            return false;
        }
    }
    if (qp_solver_ == nullptr) {
        set_reason("QP solver not initialized");
        return false;
    }
    set_reason("");
    return true;
}

bool SQPSolver::iterate()
{
    if (qp_data_ == nullptr) {
        return false;
    }
    qp_data_->reset();
    if (!linearize()) {
        return false;
    }
    if (!assembleQP()) {
        return false;
    }
    // 求解 QP：只有返回 SUCCESS 时 delta_traj_ 才有效
    if (!solveQP()) {
        // 失败兜底：delta_traj_ 不可用，禁止应用到 current_traj_
        return false;
    }
    double alpha = 1.0;
    if (options_.use_line_search) {
        if (!lineSearch(alpha)) {
            return false;
        }
    }
    // 使用流形更新保证 theta 不越界；retract 失败则该次迭代失败
    if (!applyRetraction(*ocp_, current_traj_, alpha, delta_traj_, current_traj_)) {
        LOG_ERROR("iterate retract application failed, refusing to update current_traj_");
        return false;
    }
    return true;
}

bool SQPSolver::linearize()
{
    if (ocp_ == nullptr || qp_data_ == nullptr) {
        return false;
    }
    const int N = qp_data_->N;
    const int nx = qp_data_->nx;
    const int nu = qp_data_->nu;

    // 构建扁平化的步描述符，便于后续按 global_k 直接并行分发
    struct LinearizeStep {
        int segment_idx = 0;
        int step_in_segment = 0;
        int global_k = 0;
        double dt = 0.0;
    };
    std::vector<LinearizeStep> steps;
    steps.reserve(N);
    int global_k = 0;
    for (int seg_idx = 0; seg_idx < static_cast<int>(ocp_->segments().size()); ++seg_idx) {
        const auto& segment = ocp_->segments()[seg_idx];
        if (!segment.dynamics) {
            LOG_ERROR("linearize found null dynamics at segment", seg_idx);
            return false;
        }
        for (int i = 0; i < segment.N; ++i) {
            const double dt = segment.stepSize(i);
            if (!std::isfinite(dt) || dt <= 0.0) {
                LOG_ERROR("linearize encountered invalid timestep dt=", dt, " at global_k=", global_k);
                return false;
            }
            steps.push_back({ seg_idx, i, global_k, dt });
            ++global_k;
        }
    }

    const bool use_omp = [this, N]() {
#ifdef _OPENMP
        return options_.use_omp && N >= options_.omp_parallel_threshold;
#else
        (void)N;
        return false;
#endif
    }();

    if (!use_omp) {
        // 串行路径：使用成员 scratch buffer 与原始 Constraint 指针
        for (const auto& step : steps) {
            const auto& constraints = ocp_->segments()[step.segment_idx].constraints;
            if (!linearizeStep(step.segment_idx, step.step_in_segment, step.global_k, step.dt,
                    constraints, lin_x_next_, lin_A_, lin_B_)) {
                return false;
            }
        }
        return true;
    }

    // 并行路径：每个线程持有独立的 Constraint clone 副本，避免 CasADiFunction 工作区竞争。
    // 约束克隆池已在 solve() 入口预分配，本处直接按线程索引取用。
    bool has_error = false;
    int fail_global_k = std::numeric_limits<int>::max();
#pragma omp parallel reduction(|| : has_error) reduction(min : fail_global_k)
    {
        const int thread_id = [this]() {
#ifdef _OPENMP
            return omp_get_thread_num();
#else
            return 0;
#endif
        }();
        // 防御性检查：若线程 ID 超出预分配池（理论上不应发生），回退到空指针序列
        const bool pool_ok = (thread_id < static_cast<int>(thread_constraint_clones_.size()));
        std::vector<std::vector<std::shared_ptr<Constraint>>> fallback_constraints;
        std::vector<std::vector<std::shared_ptr<Constraint>>>& local_constraints =
            pool_ok ? thread_constraint_clones_[thread_id] : fallback_constraints;

        Vector local_x_next(nx);
        Matrix local_A(nx, nx);
        Matrix local_B(nx, nu);

#pragma omp for schedule(static)
        for (int i = 0; i < static_cast<int>(steps.size()); ++i) {
            const auto& step = steps[i];
            if (!pool_ok) {
                has_error = true;
                fail_global_k = std::min(fail_global_k, step.global_k);
                continue;
            }
            try {
                if (!linearizeStep(step.segment_idx, step.step_in_segment, step.global_k, step.dt,
                        local_constraints[step.segment_idx], local_x_next, local_A, local_B)) {
                    has_error = true;
                    fail_global_k = std::min(fail_global_k, step.global_k);
                }
            } catch (const std::exception& e) {
                // 并行区内避免调用线程不安全的 logger；记录失败状态由外层统一处理
                (void)e;
                has_error = true;
                fail_global_k = std::min(fail_global_k, step.global_k);
            }
        }
    }

    if (has_error) {
        if (fail_global_k != std::numeric_limits<int>::max()) {
            LOG_ERROR("linearize failed in OpenMP parallel path at global_k=", fail_global_k, ", rerunning serially for detailed diagnostics");
            const auto [seg_idx, step_in_segment] = ocp_->globalStepToSegment(fail_global_k);
            const auto& segment = ocp_->segments()[seg_idx];
            linearizeStep(seg_idx, step_in_segment, fail_global_k,
                segment.stepSize(step_in_segment), segment.constraints,
                lin_x_next_, lin_A_, lin_B_);
        }
        return false;
    }
    return true;
}

bool SQPSolver::linearizeStep(int segment_idx, int step_in_segment, int global_k, double dt,
    const std::vector<std::shared_ptr<Constraint>>& constraints, Vector& local_x_next,
    Matrix& local_A, Matrix& local_B)
{
    // 并行区内禁止调用线程不安全的 logger；串行路径可正常记录。
    bool in_parallel = false;
#ifdef _OPENMP
    in_parallel = (omp_in_parallel() != 0);
#endif

    try {
        const int nx = qp_data_->nx;
        const int nu = qp_data_->nu;
        const int ng_max = qp_data_->ng_max;
        const auto& segment = ocp_->segments()[segment_idx];
        const Vector& x = current_traj_.x[global_k];
        const Vector& u = current_traj_.u[global_k];

        // 1) 动力学线性化
        segment.dynamics->discretizeAndLinearize(
            x, u, dt, segment.v_sign, local_x_next, local_A, local_B);
        if (local_x_next.size() != nx || local_A.rows() != nx || local_A.cols() != nx ||
            local_B.rows() != nx || local_B.cols() != nu) {
            if (!in_parallel) {
                LOG_ERROR("linearizeStep received invalid dimension dynamics output at global_k=", global_k);
            }
            return false;
        }
        if (!local_x_next.allFinite() || !local_A.allFinite() || !local_B.allFinite()) {
            if (!in_parallel) {
                LOG_ERROR("linearizeStep received non-finite dynamics values at global_k=", global_k);
            }
            return false;
        }
        qp_data_->A[global_k] = local_A;
        qp_data_->B[global_k] = local_B;
        qp_data_->b[global_k] = local_x_next - current_traj_.x[global_k + 1];

        // 2) 一般约束线性化（delta 语义：g + Cx*dx + Cu*du <= 0，即 Cx*dx + Cu*du <= -g）
        const Vector& p = getStageParameter(segment, step_in_segment);
        int row = 0;
        for (const auto& constraint : constraints) {
            if (!constraint) {
                continue;
            }
            const int ng = constraint->ng();
            if (row + ng > ng_max) {
                if (!in_parallel) {
                    LOG_ERROR("linearizeStep at global_k=", global_k,
                        " total constraint dimension exceeds ng_max: row=", row, " ng=", ng, " ng_max=", ng_max);
                }
                return false;
            }
            Vector g;
            Matrix Cx, Cu;
            if (!evaluateConstraintLinearization(*constraint, x, u, p, global_k, in_parallel, g,
                    Cx, Cu)) {
                return false;
            }
            qp_data_->C[global_k].middleRows(row, ng) = Cx;
            qp_data_->D[global_k].middleRows(row, ng) = Cu;
            qp_data_->d[global_k].segment(row, ng) = -g;
            row += ng;
        }
        // 未使用的行填充为无意义约束 C=0, D=0, d=+inf；Dense 求解器通过 isfinite 跳过
        if (row < ng_max) {
            qp_data_->C[global_k].bottomRows(ng_max - row).setZero();
            qp_data_->D[global_k].bottomRows(ng_max - row).setZero();
            qp_data_->d[global_k].tail(ng_max - row).setConstant(
                std::numeric_limits<double>::infinity());
        }
        return true;
    } catch (const std::exception& e) {
        if (!in_parallel) {
            LOG_ERROR("linearizeStep threw exception at global_k=", global_k, ": ", e.what());
        }
        return false;
    }
}

bool SQPSolver::assembleQP()
{
    if (qp_data_ == nullptr || !qp_data_->hasValidContainerSizes()) {
        return false;
    }
    const int N = qp_data_->N;

    // 构建扁平化的步描述符，便于后续按 global_k 直接并行分发
    struct AssembleStep {
        int segment_idx = 0;
        int step_in_segment = 0;
        int global_k = 0;
    };
    std::vector<AssembleStep> steps;
    steps.reserve(N);
    int global_k = 0;
    for (int seg_idx = 0; seg_idx < static_cast<int>(ocp_->segments().size()); ++seg_idx) {
        const auto& segment = ocp_->segments()[seg_idx];
        for (int i = 0; i < segment.N; ++i) {
            steps.push_back({ seg_idx, i, global_k });
            ++global_k;
        }
    }

    const bool use_omp = [this, N]() {
#ifdef _OPENMP
        return options_.use_omp && N >= options_.omp_parallel_threshold;
#else
        (void)N;
        return false;
#endif
    }();

    if (!use_omp) {
        // 串行路径：使用成员 scratch buffer 与原始 CostTerm 指针
        for (const auto& step : steps) {
            const auto& segment = ocp_->segments()[step.segment_idx];
            const Vector& x = current_traj_.x[step.global_k];
            const Vector& u = current_traj_.u[step.global_k];
            if (!assembleCost(step.global_k, segment, x, u)) {
                return false;
            }
            assembleBounds(step.global_k, segment, x, u);
        }
    } else {
        // 并行路径：每个线程持有独立的 CostTerm clone 副本。
        // cost 克隆池已在 solve() 入口预分配，本处直接按线程索引取用。
        // assembleBounds 只做简单 box bound 计算，不在本次并行化范围内，仍串行处理。
        bool has_error = false;
        int fail_global_k = std::numeric_limits<int>::max();
#pragma omp parallel reduction(|| : has_error) reduction(min : fail_global_k)
        {
            const int thread_id = [this]() {
#ifdef _OPENMP
                return omp_get_thread_num();
#else
                return 0;
#endif
            }();
            const bool pool_ok = (thread_id < static_cast<int>(thread_cost_clones_.size()));
            Vector local_q(qp_data_->nx);
            Vector local_r(qp_data_->nu);
            Matrix local_Q(qp_data_->nx, qp_data_->nx);
            Matrix local_R(qp_data_->nu, qp_data_->nu);
            Matrix local_S(qp_data_->nu, qp_data_->nx);

#pragma omp for schedule(static)
            for (int i = 0; i < static_cast<int>(steps.size()); ++i) {
                const auto& step = steps[i];
                if (!pool_ok) {
                    has_error = true;
                    fail_global_k = std::min(fail_global_k, step.global_k);
                    continue;
                }
                const CostTerm* local_cost = nullptr;
                if (step.segment_idx < static_cast<int>(thread_cost_clones_[thread_id].size())) {
                    local_cost = thread_cost_clones_[thread_id][step.segment_idx].get();
                }
                const Vector& x = current_traj_.x[step.global_k];
                const Vector& u = current_traj_.u[step.global_k];
                try {
                    if (local_cost == nullptr ||
                        !assembleCostImpl(step.global_k, *local_cost, x, u, local_q, local_r,
                            local_Q, local_R, local_S)) {
                        has_error = true;
                        fail_global_k = std::min(fail_global_k, step.global_k);
                    }
                } catch (const std::exception& e) {
                    (void)e;
                    has_error = true;
                    fail_global_k = std::min(fail_global_k, step.global_k);
                }
            }
        }
        if (has_error) {
            if (fail_global_k != std::numeric_limits<int>::max()) {
                LOG_ERROR("assembleQP failed in OpenMP parallel path at global_k=", fail_global_k);
            }
            return false;
        }
        for (const auto& step : steps) {
            const auto& segment = ocp_->segments()[step.segment_idx];
            assembleBounds(step.global_k, segment, current_traj_.x[step.global_k],
                current_traj_.u[step.global_k]);
        }
    }

    // 段间边界状态的 box bound 取相邻段交集，避免 QP 只吃到后一段 bound
    // 而 checkConvergence() 同时检查两段 bound 导致无法收敛。
    int boundary_k = 0;
    const size_t num_segments = ocp_->segments().size();
    for (size_t seg_idx = 1; seg_idx < num_segments; ++seg_idx) {
        boundary_k += ocp_->segments()[seg_idx - 1].N;
        const auto& prev = ocp_->segments()[seg_idx - 1];
        const auto& next = ocp_->segments()[seg_idx];
        const Vector& x = current_traj_.x[boundary_k];
        qp_data_->lbx[boundary_k] = prev.x_min.cwiseMax(next.x_min) - x;
        qp_data_->ubx[boundary_k] = prev.x_max.cwiseMin(next.x_max) - x;
        for (int i = 0; i < qp_data_->nx; ++i) {
            if (qp_data_->lbx[boundary_k](i) > qp_data_->ubx[boundary_k](i)) {
                LOG_ERROR("inter-segment boundary state at dim", boundary_k, " has no intersection of adjacent segment box bounds: lower=", qp_data_->lbx[boundary_k](i),
                    " upper=", qp_data_->ubx[boundary_k](i));
                return false;
            }
        }
    }

    // 终端 stage（k=N）的代价与边界
    // 后续里程碑应引入 terminal cost 契约，避免终端状态项依赖人为零控制。
    const auto& last_segment = ocp_->segments().back();
    const Vector& xN = current_traj_.x[N];
    if (!assembleCost(N, last_segment, xN, zero_u_)) {
        return false;
    }
    assembleBounds(N, last_segment, xN, zero_u_);
    // 终端无控制边界，delta_u_N 不存在
    qp_data_->lbx[N] = last_segment.x_min - xN;
    qp_data_->ubx[N] = last_segment.x_max - xN;

    // 初始 delta_x0 必须为零
    qp_data_->lbx[0].setZero();
    qp_data_->ubx[0].setZero();

    return true;
}

bool SQPSolver::assembleCost(int global_k, const StageSegment& segment,
    const Vector& x, const Vector& u)
{
    if (segment.cost) {
        return assembleCostImpl(global_k, *segment.cost, x, u, cost_q_, cost_r_,
            cost_Q_, cost_R_, cost_S_);
    }
    // 无代价时施加小正则化，保持 QP 良态；与 segment.cost == nullptr 语义一致。
    const int nx = qp_data_->nx;
    const int nu = qp_data_->nu;
    const bool is_terminal = (global_k == qp_data_->N);
    qp_data_->q[global_k].setZero();
    qp_data_->Q[global_k].setIdentity();
    qp_data_->Q[global_k] *= options_.reg_min;
    if (!is_terminal) {
        qp_data_->r[global_k].setZero();
        qp_data_->R[global_k].setIdentity();
        qp_data_->R[global_k] *= options_.reg_min;
        qp_data_->S[global_k].setZero();
    }
    return true;
}

bool SQPSolver::assembleCostImpl(int global_k, const CostTerm& cost_term,
    const Vector& x, const Vector& u, Vector& cost_q, Vector& cost_r, Matrix& cost_Q,
    Matrix& cost_R, Matrix& cost_S)
{
    const int nx = qp_data_->nx;
    const int nu = qp_data_->nu;
    const bool is_terminal = (global_k == qp_data_->N);

    // 对根代价项调用组合求值接口，消除分别调用 gradient/hessian 造成的重复 ESDF 查询。
    cost_q.setZero(nx);
    cost_Q.setZero(nx, nx);
    if (!is_terminal) {
        cost_r.setZero(nu);
        cost_R.setZero(nu, nu);
        cost_S.setZero(nu, nx);
    }

    double term_cost = 0.0;
    Vector term_q(nx);
    Vector term_r(nu);
    Matrix term_Q(nx, nx);
    Matrix term_R(nu, nu);
    Matrix term_S(nu, nx);
    try {
        cost_term.evaluateGradientAndHessian(x, u, term_cost, term_q, term_r,
            term_Q, term_R, term_S);
    } catch (const std::exception& e) {
        LOG_ERROR("assembleCost cost callback threw exception at step ", global_k, ": ", e.what());
        return false;
    }
    // 防御性校验：维度与有限性
    if (term_q.size() != nx || term_Q.rows() != nx || term_Q.cols() != nx ||
        (!is_terminal && (term_r.size() != nu || term_R.rows() != nu ||
                             term_R.cols() != nu || term_S.rows() != nu ||
                             term_S.cols() != nx))) {
        LOG_ERROR("assembleCost received invalid dimension output at step ", global_k);
        return false;
    }
    if (!term_q.allFinite() || !term_Q.allFinite() ||
        (!is_terminal &&
            (!term_r.allFinite() || !term_R.allFinite() ||
                !term_S.allFinite()))) {
        LOG_ERROR("assembleCost received non-finite values at step ", global_k);
        return false;
    }
    cost_q = term_q;
    cost_Q = term_Q;
    if (!is_terminal) {
        cost_r = term_r;
        cost_R = term_R;
        cost_S = term_S;
    }

    // 全局 Hessian 正则化（默认 0.0，不改变既有行为）：无条件叠加到本 stage 的
    // Q/R 对角上，抑制早期迭代因线性化误差导致的过大步长，详见头文件字段注释。
    if (options_.hessian_regularization > 0.0) {
        cost_Q.diagonal().array() += options_.hessian_regularization;
        if (!is_terminal) {
            cost_R.diagonal().array() += options_.hessian_regularization;
        }
    }

    qp_data_->q[global_k] = cost_q;
    qp_data_->Q[global_k] = cost_Q;
    if (!is_terminal) {
        qp_data_->r[global_k] = cost_r;
        qp_data_->R[global_k] = cost_R;
        qp_data_->S[global_k] = cost_S;
    }
    return true;
}

void SQPSolver::assembleBounds(int global_k, const StageSegment& segment,
    const Vector& x, const Vector& u)
{
    qp_data_->lbx[global_k] = segment.x_min - x;
    qp_data_->ubx[global_k] = segment.x_max - x;
    if (global_k < qp_data_->N) {
        qp_data_->lbu[global_k] = segment.u_min - u;
        qp_data_->ubu[global_k] = segment.u_max - u;
    }
}

bool SQPSolver::evaluateConstraintValue(const Constraint& constraint, const Vector& x,
    const Vector& u, const Vector& p, int global_k, bool in_parallel, Vector& g) const
{
    const int ng = constraint.ng();
    g.resize(ng);
    try {
        constraint.evaluate(x, u, p, g);
    } catch (const std::exception& e) {
        if (!in_parallel) {
            LOG_ERROR("constraint evaluation threw exception at step ", global_k, ": ", e.what());
        }
        return false;
    }
    if (g.size() != ng || !g.allFinite()) {
        if (!in_parallel) {
            LOG_ERROR("constraint evaluation received invalid output at step ", global_k);
        }
        return false;
    }
    return true;
}

bool SQPSolver::evaluateConstraintLinearization(const Constraint& constraint, const Vector& x,
    const Vector& u, const Vector& p, int global_k, bool in_parallel, Vector& g, Matrix& Cx,
    Matrix& Cu) const
{
    try {
        constraint.evaluateAndJacobian(x, u, p, g, Cx, Cu);
    } catch (const std::exception& e) {
        if (!in_parallel) {
            LOG_ERROR("constraint linearization threw exception at step ", global_k, ": ", e.what());
        }
        return false;
    }
    const int nx = qp_data_->nx;
    const int nu = qp_data_->nu;
    const int ng = constraint.ng();
    if (g.size() != ng || Cx.rows() != ng || Cx.cols() != nx || Cu.rows() != ng
        || Cu.cols() != nu) {
        if (!in_parallel) {
            LOG_ERROR("constraint linearization received invalid dimension output at step ", global_k);
        }
        return false;
    }
    if (!g.allFinite() || !Cx.allFinite() || !Cu.allFinite()) {
        if (!in_parallel) {
            LOG_ERROR("constraint linearization received non-finite values at step ", global_k);
        }
        return false;
    }
    return true;
}

bool SQPSolver::solveQP()
{
    if (qp_solver_ == nullptr || qp_data_ == nullptr) {
        return false;
    }
    // Full SQP 从第二次迭代开始，使用上一次成功 QP 解作为 HPIPM IPM 热启动；
    // RTI 模式或首次迭代不设置，避免无意义初值或单步模式被污染。
    if (!rti_mode_active_ && options_.use_qp_warm_start && iter_count_ > 0) {
        qp_solver_->setWarmStart(qp_solution_);
    }
    const QPSolverStatus status = qp_solver_->solve(*qp_data_, qp_solution_);
    if (status != QPSolverStatus::SUCCESS) {
        LOG_WARN("QP solve failed with status code:", static_cast<int>(status),
            "; delta_traj_ is not available, do not apply");
        return false;
    }

    // 防御性校验：QP 解的容器长度、维度与有限性
    const int N = qp_data_->N;
    const int nx = qp_data_->nx;
    const int nu = qp_data_->nu;
    if (static_cast<int>(qp_solution_.x.size()) != N + 1 ||
        static_cast<int>(qp_solution_.u.size()) != N) {
        LOG_ERROR("QP solution container length invalid");
        return false;
    }
    for (int k = 0; k <= N; ++k) {
        if (qp_solution_.x[k].size() != nx || !qp_solution_.x[k].allFinite()) {
            LOG_ERROR("QP solution state at step ", k, " has invalid dimension or value");
            return false;
        }
    }
    for (int k = 0; k < N; ++k) {
        if (qp_solution_.u[k].size() != nu || !qp_solution_.u[k].allFinite()) {
            LOG_ERROR("QP solution control at step ", k, " has invalid dimension or value");
            return false;
        }
    }

    // 仅当求解成功且解合法时才将方向复制到 delta_traj_
    for (int k = 0; k <= N; ++k) {
        delta_traj_.x[k] = qp_solution_.x[k];
    }
    for (int k = 0; k < N; ++k) {
        delta_traj_.u[k] = qp_solution_.u[k];
    }
    return true;
}

double SQPSolver::computeConstraintViolation(const Trajectory& traj) const
{
    double violation = 0.0;
    int global_k = 0;
    for (const auto& segment : ocp_->segments()) {
        for (int i = 0; i < segment.N; ++i) {
            const Vector& x = traj.x[global_k];
            const Vector& u = traj.u[global_k];
            // 一般约束正部
            const Vector& p_viol = getStageParameter(segment, i);
            for (const auto& constraint : segment.constraints) {
                if (!constraint ||
                    !evaluateConstraintValue(*constraint, x, u, p_viol, global_k,
                        /*in_parallel=*/false, mutable_g_)) {
                    return std::numeric_limits<double>::infinity();
                }
                violation += mutable_g_.cwiseMax(0.0).sum();
            }
            // box bound 正部
            violation += (segment.x_min - x).cwiseMax(0.0).sum();
            violation += (x - segment.x_max).cwiseMax(0.0).sum();
            violation += (segment.u_min - u).cwiseMax(0.0).sum();
            violation += (u - segment.u_max).cwiseMax(0.0).sum();
            ++global_k;
        }
        // 当前 segment 的末端状态 box bound
        const Vector& xN = traj.x[global_k];
        violation += (segment.x_min - xN).cwiseMax(0.0).sum();
        violation += (xN - segment.x_max).cwiseMax(0.0).sum();
    }
    return violation;
}

double SQPSolver::computeMerit(const Trajectory& traj) const
{
    double cost_sum = 0.0;
    int global_k = 0;
    for (const auto& segment : ocp_->segments()) {
        if (!segment.cost) {
            continue;
        }
        for (int i = 0; i < segment.N; ++i) {
            double stage_cost = 0.0;
            try {
                segment.cost->evaluate(traj.x[global_k], traj.u[global_k], stage_cost);
            } catch (const std::exception& e) {
                LOG_ERROR("computeMerit at step", global_k, " cost callback threw exception: ", e.what());
                return std::numeric_limits<double>::infinity();
            }
            if (!std::isfinite(stage_cost)) {
                LOG_ERROR("computeMerit at step ", global_k, " received non-finite cost");
                return std::numeric_limits<double>::infinity();
            }
            cost_sum += stage_cost;
            ++global_k;
        }
    }
    // 终端代价
    if (!ocp_->segments().empty()) {
        const auto& last_segment = ocp_->segments().back();
        if (!last_segment.cost) {
            return cost_sum + options_.merit_penalty * computeConstraintViolation(traj);
        }
        double terminal_cost = 0.0;
        try {
            last_segment.cost->evaluate(traj.x[global_k], zero_u_, terminal_cost);
        } catch (const std::exception& e) {
            LOG_ERROR("computeMerit terminal cost callback threw exception:", e.what());
            return std::numeric_limits<double>::infinity();
        }
        if (!std::isfinite(terminal_cost)) {
            LOG_ERROR("computeMerit received non-finite terminal cost");
            return std::numeric_limits<double>::infinity();
        }
        cost_sum += terminal_cost;
    }
    return cost_sum + options_.merit_penalty * computeConstraintViolation(traj);
}

double SQPSolver::computeDirectionalDerivative() const
{
    const int N = qp_data_->N;
    double deriv = 0.0;
    for (int k = 0; k <= N; ++k) {
        deriv += qp_data_->q[k].dot(delta_traj_.x[k]);
    }
    for (int k = 0; k < N; ++k) {
        deriv += qp_data_->r[k].dot(delta_traj_.u[k]);
    }
    // l1 merit function 的约束项方向导数近似：QP 预测 violation 降为 0，
    // 因此贡献为 -penalty * violation。
    deriv -= options_.merit_penalty * computeConstraintViolation(current_traj_);
    return deriv;
}

bool SQPSolver::lineSearch(double& alpha)
{
    const double phi0 = computeMerit(current_traj_);
    if (!std::isfinite(phi0)) {
        LOG_ERROR("line search initial merit non-finite, refusing to apply delta");
        return false;
    }

    double dir_deriv = computeDirectionalDerivative();

    // 计算当前 QP 步的最大绝对值。
    const int N = qp_data_->N;
    double max_delta = 0.0;
    for (int k = 0; k <= N; ++k) {
        max_delta = std::max(max_delta, delta_traj_.x[k].cwiseAbs().maxCoeff());
    }
    for (int k = 0; k < N; ++k) {
        max_delta = std::max(max_delta, delta_traj_.u[k].cwiseAbs().maxCoeff());
    }

    // 若 QP 步已低于 stationarity_tol 且 merit 有限，说明已到达/接近最优点。
    // 此时直接接受单位步长（只要 merit 未显著恶化），避免在 LQR 等一次迭代即达
    // 精确最优的问题中，因 Armijo 数值噪声拒绝有效步。
    if (max_delta < options_.stationarity_tol && std::isfinite(phi0)) {
        Trajectory trial;
        trial.resize(N, qp_data_->nx, qp_data_->nu);
        if (applyRetraction(*ocp_, current_traj_, 1.0, delta_traj_, trial)) {
            const double phi1 = computeMerit(trial);
            const double tol = std::max(1.0, std::abs(phi0)) * options_.stationarity_tol;
            if (std::isfinite(phi1) && phi1 <= phi0 + tol) {
                alpha = 1.0;
                return true;
            }
        }
    }

    // 若 QP 方向导数为正，说明当前方向不是严格的 merit 下降方向。
    // Full SQP 直接拒绝该迭代；RTI 作为单步 QP 无法重新线性化，启用保守策略：
    // 用一个极小的负导数继续尝试 Armijo 回退，若仍失败则退回到"merit 不恶化"兜底。
    // 导数等于 0 时保留 0（常见于 delta=0 的平稳点），此时 alpha=1 的 merit 不变，
    // Armijo 条件退化为 phi_alpha <= phi0，应被接受。
    if (dir_deriv > 0.0) {
        if (!rti_mode_active_) {
            LOG_ERROR("Full SQP mode: QP directional derivative positive, not a descent direction, refusing to apply delta");
            return false;
        }
        LOG_WARN("RTI mode: QP directional derivative positive, enabling conservative fallback strategy");
        dir_deriv = -1e-6;
    }

    Trajectory trial;
    trial.resize(qp_data_->N, qp_data_->nx, qp_data_->nu);

    alpha = 1.0;
    const double c = options_.line_search_c;
    const double rho = options_.line_search_rho;
    const double alpha_min = options_.line_search_alpha_min;
    constexpr int max_backtracks = 20;

    for (int iter = 0; iter < max_backtracks; ++iter) {
        if (!applyRetraction(*ocp_, current_traj_, alpha, delta_traj_, trial)) {
            LOG_ERROR("line search at backtrack step ", iter, " retract application failed");
            return false;
        }
        const double phi_alpha = computeMerit(trial);
        // Armijo 条件；同时拒绝非有限 merit
        if (std::isfinite(phi_alpha) && phi_alpha <= phi0 + c * alpha * dir_deriv) {
            return true;
        }
        alpha *= rho;
        if (alpha < alpha_min) {
            alpha = alpha_min;
            if (!applyRetraction(*ocp_, current_traj_, alpha, delta_traj_, trial)) {
                LOG_ERROR("line search retract application failed at minimum step size");
                return false;
            }
            const double phi_min = computeMerit(trial);
            if (std::isfinite(phi_min) && phi_min <= phi0 + c * alpha * dir_deriv) {
                return true;
            }
            // RTI 模式安全兜底：最小步长仍不满足 Armijo 时，
            // 至少要求 trial merit 有限且不恶化（phi_min <= phi0），否则同样拒绝。
            if (rti_mode_active_ && std::isfinite(phi_min) && phi_min <= phi0 + 1e-12) {
                LOG_WARN("RTI mode: minimum step size does not satisfy Armijo but merit did not worsen, applying alpha_min=", alpha);
                return true;
            }
            LOG_ERROR("line search backtracked to minimum step size but Armijo condition still not satisfied, refusing to apply delta");
            return false;
        }
    }

    // RTI 模式兜底：最大回退次数后仍检查 merit 有限且不恶化
    if (rti_mode_active_) {
        alpha = alpha_min;
        if (!applyRetraction(*ocp_, current_traj_, alpha, delta_traj_, trial)) {
            LOG_ERROR("RTI mode retract application failed after maximum backtracking");
            return false;
        }
        const double phi_min = computeMerit(trial);
        if (std::isfinite(phi_min) && phi_min <= phi0 + 1e-12) {
            LOG_WARN("RTI mode: merit did not worsen after maximum backtracking, applying alpha_min=", alpha);
            return true;
        }
    }

    LOG_ERROR("line search reached maximum backtracking iterations without satisfying Armijo condition, refusing to apply delta");
    return false;
}

bool SQPSolver::checkConvergence()
{
    const int N = qp_data_->N;

    // 1) 步长平稳性：delta_x / delta_u 最大绝对值
    double max_delta = 0.0;
    for (int k = 0; k <= N; ++k) {
        max_delta = std::max(max_delta, delta_traj_.x[k].cwiseAbs().maxCoeff());
    }
    for (int k = 0; k < N; ++k) {
        max_delta = std::max(max_delta, delta_traj_.u[k].cwiseAbs().maxCoeff());
    }
    if (max_delta >= options_.stationarity_tol) {
        return false;
    }

    // 2) 动力学残差：基于当前轨迹重新计算 x_next - x[k+1]，
    //    避免使用 applyRetraction 之前旧线性化的 b[k]。
    //    使用轻量 discretize() 接口，避免重复计算 A/B。
    const int nx = qp_data_->nx;
    const int nu = qp_data_->nu;
    (void)nu; // discretize() 默认实现内部使用，此处仅用于线性化路径一致性
    double max_dyn_residual = 0.0;
    int global_k = 0;
    for (const auto& segment : ocp_->segments()) {
        for (int i = 0; i < segment.N; ++i) {
            Vector x_next;
            try {
                segment.dynamics->discretize(current_traj_.x[global_k],
                    current_traj_.u[global_k], segment.stepSize(i), segment.v_sign,
                    x_next);
            } catch (const std::exception& e) {
                LOG_ERROR("checkConvergence at step ", global_k, " dynamics callback threw exception: ",
                    e.what());
                return false;
            }
            // 防御性校验：只关心 x_next 维度与有限性
            if (x_next.size() != nx) {
                LOG_ERROR("checkConvergence at step", global_k, " received invalid dimension output");
                return false;
            }
            if (!x_next.allFinite()) {
                LOG_ERROR("checkConvergence at step", global_k, " received non-finite values");
                return false;
            }
            const Vector defect = x_next - current_traj_.x[global_k + 1];
            max_dyn_residual = std::max(max_dyn_residual, defect.cwiseAbs().maxCoeff());
            ++global_k;
        }
    }
    if (max_dyn_residual >= options_.kkt_tol) {
        return false;
    }

    // 3) 一般约束违反：g(x, u) <= 0
    double max_constr_viol = 0.0;
    global_k = 0;
    for (const auto& segment : ocp_->segments()) {
        for (int i = 0; i < segment.N; ++i) {
            const Vector& p_conv = getStageParameter(segment, i);
            for (const auto& constraint : segment.constraints) {
                if (!constraint ||
                    !evaluateConstraintValue(*constraint, current_traj_.x[global_k],
                        current_traj_.u[global_k], p_conv, global_k, /*in_parallel=*/false,
                        mutable_g_)) {
                    return false;
                }
                max_constr_viol = std::max(max_constr_viol, mutable_g_.cwiseMax(0.0).maxCoeff());
            }
            ++global_k;
        }
    }
    if (max_constr_viol >= options_.constr_viol_tol) {
        return false;
    }

    // 4) box 边界违反（含末端状态）
    double max_box_viol = 0.0;
    global_k = 0;
    for (const auto& segment : ocp_->segments()) {
        for (int i = 0; i < segment.N; ++i) {
            const Vector& x = current_traj_.x[global_k];
            const Vector& u = current_traj_.u[global_k];
            max_box_viol = std::max(max_box_viol,
                (segment.x_min - x).cwiseMax(0.0).maxCoeff());
            max_box_viol = std::max(max_box_viol,
                (x - segment.x_max).cwiseMax(0.0).maxCoeff());
            max_box_viol = std::max(max_box_viol,
                (segment.u_min - u).cwiseMax(0.0).maxCoeff());
            max_box_viol = std::max(max_box_viol,
                (u - segment.u_max).cwiseMax(0.0).maxCoeff());
            ++global_k;
        }
        // 当前 segment 的末端状态边界
        const Vector& xN = current_traj_.x[global_k];
        max_box_viol = std::max(max_box_viol,
            (segment.x_min - xN).cwiseMax(0.0).maxCoeff());
        max_box_viol = std::max(max_box_viol,
            (xN - segment.x_max).cwiseMax(0.0).maxCoeff());
    }
    if (max_box_viol >= options_.constr_viol_tol) {
        return false;
    }

    return true;
}

bool SQPSolver::applyRetraction(const MultiStageOCP& ocp,
    const Trajectory& current, double alpha, const Trajectory& delta,
    Trajectory& result)
{
    int global_k = 0;
    for (const auto& segment : ocp.segments()) {
        const auto& dynamics = *segment.dynamics;
        for (int i = 0; i < segment.N; ++i) {
            // 关键：状态更新必须走动力学模型的 retract，确保 theta 在 SO2 上流形更新
            try {
                dynamics.retract(current.x[global_k], alpha, delta.x[global_k],
                    result.x[global_k]);
            } catch (const std::exception& e) {
                LOG_ERROR("applyRetraction at step ", global_k, " model retract threw exception: ", e.what());
                return false;
            }
            result.u[global_k] = current.u[global_k] + alpha * delta.u[global_k];
            if (!result.x[global_k].allFinite() || !result.u[global_k].allFinite()) {
                LOG_ERROR("applyRetraction at step", global_k, " got non-finite result");
                return false;
            }
            ++global_k;
        }
    }
    // 末端状态同样走 retract（避免 N=0 等边界情况）
    if (!ocp.segments().empty()) {
        const auto& last_segment = ocp.segments().back();
        try {
            last_segment.dynamics->retract(current.x[global_k], alpha, delta.x[global_k],
                result.x[global_k]);
        } catch (const std::exception& e) {
            LOG_ERROR("applyRetraction terminal state model retract threw exception:", e.what());
            return false;
        }
        if (!result.x[global_k].allFinite()) {
            LOG_ERROR("applyRetraction got non-finite result at terminal state");
            return false;
        }
    }
    return true;
}
} // namespace stc_SQP
