#include "apa_ddp_solver.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace apa_post_processor {
ApaDdpSolver::ApaDdpSolver(ApaDdpSolverConfig config,
                           const BicycleDynamics* dynamics,
                           const DdpCostEvaluator* cost_evaluator)
    : config_(std::move(config)),
      dynamics_(dynamics),
      cost_evaluator_(cost_evaluator),
      outer_loop_(config_.outer, config_.cost),
      inner_solver_(std::make_unique<MsIlqrSolver>(config_.inner, dynamics_,
                                                   cost_evaluator_)) {
    if (dynamics_ == nullptr || cost_evaluator_ == nullptr) {
        throw std::invalid_argument("ApaDdpSolver: 动力学与代价求值层必须非空");
    }
}

ApaDdpStageOneResult ApaDdpSolver::solveStageOne(
    const DdpReference& reference) {
    // 冷启动入口：首轮以参考自带的前端初值启动（委托给热启动实现）
    return solveStageOne(reference, reference.initial_states,
                         reference.initial_controls);
}

MsIlqrResult ApaDdpSolver::solveInnerResilient(
    const DdpReference& reference, AlOuterLoop* outer,
    DdpCostMultiplierState* multipliers, const DdpCostInput& cost_input,
    const DdpAlignedVec<DdpState>& warm_states,
    const DdpAlignedVec<DdpControl>& warm_controls, double mu_round,
    ApaDdpReport* report) {
    const auto raise_merit_floor = [&]() {
        if (config_.inner.merit_mu_al_ratio > 0.0) {
            inner_solver_->raiseMeritMuFloor(
                config_.inner.merit_mu_al_ratio *
                MeritAlHook(mu_round, outer->muAmplitude()));
        }
    };
    raise_merit_floor();
    MsIlqrResult result = inner_solver_->solve(
        reference, *multipliers, cost_input, warm_states, warm_controls);
    if (result.status != MsIlqrStatus::REGULARIZATION_OVERFLOW) {
        return result;
    }
    // 冷重启兜底：溢出部分源于陈旧 QP 活动集热启动与衰减到地板的
    inner_solver_ = std::make_unique<MsIlqrSolver>(config_.inner, dynamics_,
                                                   cost_evaluator_);
    ++report->inner_restarts;
    raise_merit_floor();
    result = inner_solver_->solve(reference, *multipliers, cost_input,
                                  warm_states, warm_controls);
    return result;
}

void ApaDdpSolver::runOuterLoop(
    const DdpReference& reference, AlOuterLoop* outer,
    DdpCostMultiplierState* multipliers,
    const DdpAlignedVec<DdpState>& warm_start_states,
    const DdpAlignedVec<DdpControl>& warm_start_controls, int max_rounds,
    double tracking_weight, const std::vector<bool>* anneal_exempt_mask,
    const DdpGatingPlan* gating_plan, ApaDdpReport* report,
    double* out_max_sign_violation, double* out_max_dwell_violation,
    double* out_max_seam_speed, bool* out_gating_ok) {
    const std::size_t num_steps = reference.poses.size() - 1;
    const bool has_gating = (gating_plan != nullptr);
    report->history.reserve(static_cast<std::size_t>(max_rounds));
    // 零乘子求值：阶段二需要门控零向量在场，否则求值层拒绝
    const auto zero_multipliers = [&]() {
        if (has_gating) {
            return DdpCostMultiplierState::MakeStageTwoZero(
                num_steps, gating_plan->seam_indices.size());
        }
        return DdpCostMultiplierState::MakeZero(num_steps);
    }();
    // 门控状态（仅阶段二使用）
    double gating_mu = config_.gating_mu_initial;
    double prev_gating_violation = -1.0;
    // 热启动轨迹
    auto warm_states = warm_start_states;
    auto warm_controls = warm_start_controls;
    for (int round = 0; round < max_rounds; ++round) {
        DdpCostInput cost_input;
        cost_input.tracking_weight =
            anneal_exempt_mask != nullptr ? outer->trackingWeight()
                                          : tracking_weight;
        cost_input.anneal_exempt_mask = anneal_exempt_mask;
        cost_input.gating_plan = gating_plan;
        cost_input.esdf_scale = outer->esdfScale();
        const double mu_round = outer->mu();
        MsIlqrResult inner_result = solveInnerResilient(
            reference, outer, multipliers, cost_input, warm_states,
            warm_controls, mu_round, report);
        report->total_inner_iterations += inner_result.iterations;
        report->domain_guard_rejections += inner_result.domain_guard_rejections;
        if (inner_result.status == MsIlqrStatus::REGULARIZATION_OVERFLOW) {
            report->status = ApaDdpStatus::INNER_SOLVER_FAILED;
            break;
        }
        const auto snapshot = outer->measure(
            reference, inner_solver_->states(), inner_solver_->defects());
        // 阶段二专属：门控量测
        GatingSnapshot gating_snapshot;
        if (has_gating) {
            gating_snapshot =
                measureGating(*gating_plan, inner_solver_->states());
        }
        const double base_cost =
            cost_evaluator_
                ->evaluate(reference, inner_solver_->states(),
                           inner_solver_->controls(), zero_multipliers,
                           cost_input)
                .total_cost;
        ApaDdpOuterRecord record;
        record.outer_index = round;
        record.tracking_weight = cost_input.tracking_weight;
        record.mu = mu_round;
        record.base_cost = base_cost;
        record.augmented_cost = inner_result.final_cost;
        record.terminal_position_error = snapshot.terminal_position_error;
        record.terminal_heading_error_deg = snapshot.terminal_heading_error_deg;
        record.max_amplitude_violation = snapshot.max_amplitude_violation;
        record.defect_norm_inf = snapshot.defect_norm_inf;
        record.inner_status = inner_result.status;
        record.inner_iterations = inner_result.iterations;
        const auto check = outer->checkTermination(snapshot);
        const bool gating_ok =
            !has_gating ||
            (gating_snapshot.max_sign_violation <= config_.gating_tol &&
             gating_snapshot.max_dwell_violation <= config_.gating_tol &&
             gating_snapshot.max_seam_speed <= config_.gating_tol);
        // 终态指标随每轮刷新
        report->terminal_position_error = snapshot.terminal_position_error;
        report->terminal_heading_error_deg =
            snapshot.terminal_heading_error_deg;
        report->max_amplitude_violation = snapshot.max_amplitude_violation;
        report->defect_norm_inf = snapshot.defect_norm_inf;
        report->final_cost = base_cost;
        report->terminal_ok = check.terminal_ok;
        report->inequality_ok = check.inequality_ok;
        report->defect_ok = check.defect_ok;
        report->outer_iterations = round + 1;
        // 阶段二门控输出（每轮覆写，末轮值即为终态）
        if (out_max_sign_violation != nullptr) {
            *out_max_sign_violation = gating_snapshot.max_sign_violation;
        }
        if (out_max_dwell_violation != nullptr) {
            *out_max_dwell_violation = gating_snapshot.max_dwell_violation;
        }
        if (out_max_seam_speed != nullptr) {
            *out_max_seam_speed = gating_snapshot.max_seam_speed;
        }
        if (out_gating_ok != nullptr) {
            *out_gating_ok = gating_ok;
        }
        if (check.converged() && gating_ok) {
            report->status = ApaDdpStatus::CONVERGED;
            report->history.push_back(std::move(record));
            break;
        }
        if (round + 1 >= max_rounds) {
            report->status = ApaDdpStatus::MAX_OUTER_ITERATIONS;
            report->history.push_back(std::move(record));
            break;
        }
        record.mu_increased =
            outer->update(snapshot, base_cost, multipliers);
        report->history.push_back(std::move(record));
        // 阶段二专属：门控乘子更新
        if (has_gating) {
            updateGating(gating_snapshot, multipliers, &gating_mu,
                         &prev_gating_violation);
        }
        // δ 投影热启动
        warm_states = inner_solver_->states();
        for (auto& state : warm_states) {
            state(DDP_IDX_DELTA) = std::min(
                std::max(state(DDP_IDX_DELTA), -config_.cost.delta_max),
                config_.cost.delta_max);
        }
        warm_controls = inner_solver_->controls();
    }
    report->mu_initial_calibrated = outer->calibratedMu();
    report->mu_final = outer->mu();
}

ApaDdpStageOneResult ApaDdpSolver::solveStageOne(
    const DdpReference& reference,
    const DdpAlignedVec<DdpState>& warm_start_states,
    const DdpAlignedVec<DdpControl>& warm_start_controls) {
    const std::size_t num_poses = reference.poses.size();
    if (num_poses < 2) {
        throw std::invalid_argument("ApaDdpSolver: 参考位姿数量必须 >= 2");
    }
    const std::size_t num_steps = num_poses - 1;
    if (reference.initial_states.size() != num_poses ||
        reference.initial_controls.size() != num_steps) {
        throw std::invalid_argument(
            "ApaDdpSolver: 状态/控制初值尺寸必须为 N+1 / N");
    }
    if (warm_start_states.size() != num_poses ||
        warm_start_controls.size() != num_steps) {
        throw std::invalid_argument(
            "ApaDdpSolver: 热启动状态/控制尺寸必须为 N+1 / N");
    }
    ApaDdpStageOneResult result;
    outer_loop_.reset();
    auto multipliers = outer_loop_.makeInitialMultipliers(num_steps);
    const auto exempt_mask = outer_loop_.makeAnnealExemptMask(reference);
    runOuterLoop(reference, &outer_loop_, &multipliers, warm_start_states,
                 warm_start_controls, config_.outer.max_outer_iterations,
                 /*tracking_weight=*/0.0, &exempt_mask,
                 /*gating_plan=*/nullptr, &result.report,
                 /*out_max_sign_violation=*/nullptr,
                 /*out_max_dwell_violation=*/nullptr,
                 /*out_max_seam_speed=*/nullptr,
                 /*out_gating_ok=*/nullptr);
    result.states = inner_solver_->states();
    result.controls = inner_solver_->controls();
    result.final_multipliers = multipliers;
    return result;
}

ApaDdpStageTwoResult ApaDdpSolver::solveStageTwo(
    const DdpReference& reference, const DdpGatingPlan& gating_plan,
    const DdpAlignedVec<DdpState>& warm_states,
    const DdpAlignedVec<DdpControl>& warm_controls, double tracking_weight,
    const DdpCostMultiplierState* dual_seed) {
    const std::size_t num_poses = reference.poses.size();
    if (num_poses < 2) {
        throw std::invalid_argument("ApaDdpSolver: 参考位姿数量必须 >= 2");
    }
    const std::size_t num_steps = num_poses - 1;
    if (warm_states.size() != num_poses || warm_controls.size() != num_steps) {
        throw std::invalid_argument(
            "ApaDdpSolver: 阶段二热启动尺寸必须为 N+1 / N");
    }
    if (!std::isfinite(tracking_weight) || tracking_weight < 0.0) {
        throw std::invalid_argument(
            "ApaDdpSolver: 阶段二跟踪权重必须为非负有限值");
    }
    // 门控计划完整性：尺寸/查表自洽（残缺计划会让三类门控静默错位）
    if (gating_plan.sign_gate.size() != num_poses ||
        gating_plan.seam_lookup.size() != num_poses ||
        gating_plan.dwell_v_cap.size() != num_poses) {
        throw std::invalid_argument("ApaDdpSolver: 门控计划尺寸必须为 N+1");
    }
    for (std::size_t j = 0; j < gating_plan.seam_indices.size(); ++j) {
        const std::size_t seam = gating_plan.seam_indices[j];
        if (seam >= num_poses ||
            gating_plan.seam_lookup[seam] != static_cast<int>(j)) {
            throw std::invalid_argument(
                "ApaDdpSolver: 门控计划接缝索引与查表不自洽");
        }
    }
    for (std::size_t k = 0; k < num_poses; ++k) {
        if (gating_plan.seam_lookup[k] >= 0 &&
            gating_plan.seam_indices[static_cast<std::size_t>(
                gating_plan.seam_lookup[k])] != k) {
            throw std::invalid_argument(
                "ApaDdpSolver: 门控计划接缝查表存在游离条目");
        }
    }
    AlOuterLoopConfig outer_config = config_.outer;
    outer_config.amplitude_mu_per_element =
        config_.outer.amplitude_mu_per_element_stage_two;
    if (dual_seed != nullptr && dual_seed->terminal_mu.size() > 0) {
        const double seed_mu = dual_seed->terminal_mu.maxCoeff();
        outer_config.first_round_mu = seed_mu;
        outer_config.mu_min = std::max(outer_config.mu_min, seed_mu);
    }
    AlOuterLoop outer(outer_config, config_.cost);
    auto multipliers = DdpCostMultiplierState::MakeStageTwoZero(
        num_steps, gating_plan.seam_indices.size());
    multipliers.amplitude_mu.setConstant(outer_config.amplitude_mu_initial);
    multipliers.terminal_mu.setConstant(outer_config.first_round_mu);
    if (dual_seed != nullptr) {
        if (dual_seed->terminal_lambda.size() > 0) {
            multipliers.terminal_lambda = dual_seed->terminal_lambda;
            multipliers.terminal_mu = dual_seed->terminal_mu;
        }
        const auto amplitude_size =
            static_cast<Eigen::Index>(DDP_AMPLITUDE_CONSTRAINT_DIM * num_steps);
        if (dual_seed->amplitude_lambda.size() == amplitude_size &&
            dual_seed->amplitude_mu.size() == amplitude_size) {
            multipliers.amplitude_lambda = dual_seed->amplitude_lambda;
            multipliers.amplitude_mu = dual_seed->amplitude_mu;
        }
    }
    multipliers.gating_sign_mu.setConstant(config_.gating_mu_initial);
    multipliers.gating_seam_mu.setConstant(config_.gating_mu_initial);
    multipliers.gating_dwell_mu.setConstant(config_.gating_mu_initial);
    ApaDdpStageTwoResult result;
    runOuterLoop(reference, &outer, &multipliers, warm_states, warm_controls,
                 config_.stage_two_max_outer_iterations, tracking_weight,
                 /*anneal_exempt_mask=*/nullptr, &gating_plan, &result.report,
                 &result.max_sign_violation, &result.max_dwell_violation,
                 &result.max_seam_speed, &result.gating_ok);
    result.states = inner_solver_->states();
    result.controls = inner_solver_->controls();
    return result;
}

ApaDdpSolver::GatingSnapshot ApaDdpSolver::measureGating(
    const DdpGatingPlan& plan, const DdpAlignedVec<DdpState>& states) const {
    const std::size_t num_poses = states.size();
    if (num_poses < 2 || plan.sign_gate.size() != num_poses ||
        plan.seam_lookup.size() != num_poses ||
        plan.dwell_v_cap.size() != num_poses) {
        throw std::invalid_argument(
            "ApaDdpSolver: 门控量测的状态/计划尺寸必须为 N+1");
    }
    GatingSnapshot snapshot;
    const auto nodes = static_cast<Eigen::Index>(num_poses);
    snapshot.sign_g = Eigen::VectorXd::Zero(nodes);
    snapshot.dwell_g = Eigen::VectorXd::Zero(nodes);
    snapshot.seam_c = Eigen::VectorXd::Zero(
        static_cast<Eigen::Index>(plan.seam_indices.size()));
    double violation_sq = 0.0;
    for (std::size_t k = 0; k < num_poses; ++k) {
        const double v = states[k](DDP_IDX_V);
        // 残差公式与代价求值层同一来源（量测与惩罚失配会破坏 AL 收敛性）
        const int sign = plan.sign_gate[k];
        if (sign != 0) {
            const double g = DdpCostEvaluator::SignGateResidual(sign, v);
            snapshot.sign_g(static_cast<Eigen::Index>(k)) = g;
            snapshot.max_sign_violation =
                std::max(snapshot.max_sign_violation, std::max(0.0, g));
            violation_sq += std::pow(std::max(0.0, g), 2.0);
        }
        const int seam = plan.seam_lookup[k];
        if (seam >= 0) {
            const double c = DdpCostEvaluator::SeamResidual(v);
            snapshot.seam_c(seam) = c;
            snapshot.max_seam_speed =
                std::max(snapshot.max_seam_speed, std::abs(c));
            violation_sq += c * c;
        }
        const double cap = plan.dwell_v_cap[k];
        if (cap > 0.0) {
            const double g = DdpCostEvaluator::DwellResidual(v, cap);
            snapshot.dwell_g(static_cast<Eigen::Index>(k)) = g;
            snapshot.max_dwell_violation = std::max(
                snapshot.max_dwell_violation, std::max(0.0, std::abs(v) - cap));
            violation_sq += std::pow(std::max(0.0, g), 2.0);
        }
    }
    snapshot.violation_norm = std::sqrt(violation_sq);
    return snapshot;
}

bool ApaDdpSolver::updateGating(const GatingSnapshot& snapshot,
                                DdpCostMultiplierState* multipliers,
                                double* gating_mu,
                                double* prev_violation) const {
    if (multipliers == nullptr || gating_mu == nullptr ||
        prev_violation == nullptr ||
        multipliers->gating_sign_lambda.size() != snapshot.sign_g.size() ||
        multipliers->gating_dwell_lambda.size() != snapshot.dwell_g.size() ||
        multipliers->gating_seam_lambda.size() != snapshot.seam_c.size()) {
        throw std::invalid_argument("ApaDdpSolver: 门控乘子尺寸必须与快照一致");
    }
    // 乘子更新（Hestenes-Powell）：不等式投影恒非负，等式符号自由
    multipliers->gating_sign_lambda =
        (multipliers->gating_sign_lambda + *gating_mu * snapshot.sign_g)
            .cwiseMax(0.0);
    multipliers->gating_dwell_lambda =
        (multipliers->gating_dwell_lambda + *gating_mu * snapshot.dwell_g)
            .cwiseMax(0.0);
    multipliers->gating_seam_lambda += *gating_mu * snapshot.seam_c;
    // 门控 μ 增长（单组门控）：首轮只记录不增长（与外层标定轮同一约定），
    bool increased = false;
    if (*prev_violation >= 0.0 &&
        snapshot.violation_norm >
            config_.outer.mu_gate_kappa * *prev_violation) {
        *gating_mu = std::min(config_.outer.mu_growth_factor * *gating_mu,
                              config_.gating_mu_max);
        increased = true;
    }
    *prev_violation = snapshot.violation_norm;
    multipliers->gating_sign_mu.setConstant(*gating_mu);
    multipliers->gating_dwell_mu.setConstant(*gating_mu);
    multipliers->gating_seam_mu.setConstant(*gating_mu);
    return increased;
}
}  // namespace apa_post_processor
