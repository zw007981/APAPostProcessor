#include "apa_ilqr_solver.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace apa_post_processor {
bool EscalationStuck(const EscalationState& s) {
    if (!s.inner_stationary) {
        return false;
    }
    if (!s.terminal_ok && !(s.terminal_wanted && s.terminal_pinned)) {
        return false;
    }
    if (!s.inequality_ok && !(s.amplitude_wanted && s.amplitude_pinned)) {
        return false;
    }
    if (s.has_gating && !s.gating_ok &&
        !(s.gating_wanted && s.gating_pinned)) {
        return false;
    }
    return true;
}
ApaILQRSolver::ApaILQRSolver(const iLQRConfig& config,
                           const BicycleDynamics* dynamics,
                           const iLQRCostEvaluator* cost_evaluator)
    : config_(config),
      dynamics_(dynamics),
      cost_evaluator_(cost_evaluator),
      outer_loop_(config_) {
    resetInnerSolver();
    if (dynamics_ == nullptr || cost_evaluator_ == nullptr) {
        throw std::invalid_argument("ApaILQRSolver: 动力学与代价求值层必须非空");
    }
}

void ApaILQRSolver::resetInnerSolver() {
    if (config_.inner_use_virtual_control) {
        inner_solver_ = std::make_unique<MsIlqrSolverVirtualControl>(
            config_, dynamics_, cost_evaluator_);
    } else {
        inner_solver_ = std::make_unique<MsIlqrSolver>(config_, dynamics_,
                                                      cost_evaluator_);
    }
}

ApaILQRStageOneResult ApaILQRSolver::solveStageOne(
    const iLQRReference& reference) {
    // 冷启动入口：首轮以参考自带的前端初值启动（委托给热启动实现）
    return solveStageOne(reference, reference.initial_states,
                         reference.initial_controls);
}

MsIlqrResult ApaILQRSolver::solveInnerResilient(
    const iLQRReference& reference, AlOuterLoop* outer,
    iLQRCostMultiplierState* multipliers, const iLQRCostInput& cost_input,
    const iLQRAlignedVec<iLQRState>& warm_states,
    const iLQRAlignedVec<iLQRControl>& warm_controls,
    iLQRAlignedVec<iLQRState>* virtual_controls, double mu_round,
    ApaILQRReport* report) {
    const auto raise_merit_floor = [&]() {
        if (config_.inner_merit_mu_al_ratio > 0.0) {
            inner_solver_->raiseMeritMuFloor(
                config_.inner_merit_mu_al_ratio *
                MeritAlHook(mu_round, outer->muAmplitude()));
        }
    };
    // 虚拟控制热启动：空数组让内层反解初始化（首轮 rollout 复现热启动
    // 轨迹、初始缺陷恒零），非空则逐轮续接
    const iLQRAlignedVec<iLQRState>* w_in =
        config_.inner_use_virtual_control && virtual_controls != nullptr &&
                !virtual_controls->empty()
            ? virtual_controls
            : nullptr;
    const auto write_back = [&]() {
        if (config_.inner_use_virtual_control && virtual_controls != nullptr) {
            *virtual_controls = inner_solver_->virtualControls();
        }
    };
    raise_merit_floor();
    MsIlqrResult result = inner_solver_->solve(
        reference, *multipliers, cost_input, warm_states, warm_controls, w_in);
    if (result.status != MsIlqrStatus::REGULARIZATION_OVERFLOW) {
        write_back();
        return result;
    }
    // 冷重启兜底：溢出部分源于陈旧 QP 活动集热启动与衰减到地板的
    resetInnerSolver();
    ++report->inner_restarts;
    raise_merit_floor();
    result = inner_solver_->solve(reference, *multipliers, cost_input,
                                  warm_states, warm_controls, w_in);
    write_back();
    return result;
}

void ApaILQRSolver::runOuterLoop(
    const iLQRReference& reference, AlOuterLoop* outer,
    iLQRCostMultiplierState* multipliers,
    const iLQRAlignedVec<iLQRState>& warm_start_states,
    const iLQRAlignedVec<iLQRControl>& warm_start_controls,
    iLQRAlignedVec<iLQRState>* warm_virtual_controls, int max_rounds,
    double tracking_weight, const std::vector<bool>* anneal_exempt_mask,
    const iLQRGatingPlan* gating_plan, ApaILQRReport* report,
    double* out_max_sign_violation, double* out_max_dwell_violation,
    double* out_max_seam_speed, bool* out_gating_ok) {
    const std::size_t num_steps = reference.poses.size() - 1;
    const bool has_gating = (gating_plan != nullptr);
    report->history.reserve(static_cast<std::size_t>(max_rounds));
    // 零乘子求值：阶段二需要门控零向量在场，否则求值层拒绝
    const auto zero_multipliers = [&]() {
        if (has_gating) {
            return iLQRCostMultiplierState::MakeStageTwoZero(
                num_steps, gating_plan->seam_indices.size());
        }
        return iLQRCostMultiplierState::MakeZero(num_steps);
    }();
    // 门控状态（仅阶段二使用）
    double gating_mu = config_.gating_mu_initial;
    double prev_gating_violation = -1.0;
    // 升级耗尽连续轮计数（阶段二提前退出判据的观测窗）
    int exhausted_streak = 0;
    // 热启动轨迹
    auto warm_states = warm_start_states;
    auto warm_controls = warm_start_controls;
    for (int round = 0; round < max_rounds; ++round) {
        iLQRCostInput cost_input;
        cost_input.tracking_weight =
            anneal_exempt_mask != nullptr ? outer->trackingWeight()
                                          : tracking_weight;
        cost_input.anneal_exempt_mask = anneal_exempt_mask;
        cost_input.gating_plan = gating_plan;
        cost_input.esdf_scale = outer->esdfScale();
        const double mu_round = outer->mu();
        MsIlqrResult inner_result = solveInnerResilient(
            reference, outer, multipliers, cost_input, warm_states,
            warm_controls, warm_virtual_controls, mu_round, report);
        report->total_inner_iterations += inner_result.iterations;
        report->domain_guard_rejections += inner_result.domain_guard_rejections;
        if (inner_result.status == MsIlqrStatus::REGULARIZATION_OVERFLOW) {
            report->status = ApaILQRStatus::INNER_SOLVER_FAILED;
            break;
        }
        // 虚拟控制残余 ‖w‖∞（开关关闭时为 0）：增广问题收敛判据的附加项
        double w_inf = 0.0;
        if (config_.inner_use_virtual_control) {
            for (const auto& w : inner_solver_->virtualControls()) {
                w_inf = std::max(w_inf, w.cwiseAbs().maxCoeff());
            }
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
        ApaILQROuterRecord record;
        record.outer_index = round;
        record.tracking_weight = cost_input.tracking_weight;
        record.mu = mu_round;
        record.mu_amplitude = multipliers->amplitude_mu.maxCoeff();
        record.mu_gating = gating_mu;
        record.base_cost = base_cost;
        record.augmented_cost = inner_result.final_cost;
        record.terminal_position_error = snapshot.terminal_position_error;
        record.terminal_heading_error_deg = snapshot.terminal_heading_error_deg;
        record.max_amplitude_violation = snapshot.max_amplitude_violation;
        record.defect_norm_inf = snapshot.defect_norm_inf;
        record.virtual_control_norm_inf = w_inf;
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
        report->virtual_control_norm_inf = w_inf;
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
        // 增广路径不设额外收敛判据：内层 w 稳态随对偶残差缩放、
        // 不要求趋零（软代价权重 1/R 决定扰动量级）；外层原始三组
        // 判据承担可行性验收，内层溢出已由上方分支拦截
        const bool virtual_ok = true;
        if (check.converged() && gating_ok && virtual_ok) {
            report->status = ApaILQRStatus::CONVERGED;
            report->history.push_back(std::move(record));
            break;
        }
        if (round + 1 >= max_rounds) {
            report->status = ApaILQRStatus::MAX_OUTER_ITERATIONS;
            report->history.push_back(std::move(record));
            break;
        }
        record.mu_increased =
            outer->update(snapshot, base_cost, multipliers);
        report->history.push_back(std::move(record));
        // 阶段二专属：门控乘子更新
        bool gating_wanted = false;
        if (has_gating) {
            updateGating(gating_snapshot, multipliers, &gating_mu,
                         &prev_gating_violation, &gating_wanted);
        }
        // 阶段二提前退出（仅目标逐轮冻结的外层：阶段一含退火，不动点
        // 论证不成立）：连续两轮「升级耗尽」即停——第一轮证明 μ 已无
        // 空间且 λ 增量已注入，第二轮证明 λ 漂移也未带来 μ_gate_kappa
        // 级下降；任何一组仍有升级空间都会复位计数，自适应于求解难度
        if (anneal_exempt_mask == nullptr) {
            const EscalationState state{
                check.terminal_ok, outer->wantedTerminalGrowth(),
                outer->mu() >= config_.outer_mu_max,
                check.inequality_ok, outer->wantedAmplitudeGrowth(),
                multipliers->amplitude_mu.maxCoeff() >= config_.outer_mu_max,
                gating_ok, gating_wanted,
                gating_mu >= config_.gating_mu_max, has_gating,
                inner_result.status == MsIlqrStatus::CONVERGED_COST ||
                    inner_result.status == MsIlqrStatus::CONVERGED_GRADIENT};
            if (EscalationStuck(state)) {
                ++exhausted_streak;
                if (exhausted_streak >= 2) {
                    report->status = ApaILQRStatus::ESCALATION_EXHAUSTED;
                    break;
                }
            } else {
                exhausted_streak = 0;
            }
        }
        // δ 投影热启动
        warm_states = inner_solver_->states();
        for (auto& state : warm_states) {
            state(ILQR_IDX_DELTA) = std::min(
                std::max(state(ILQR_IDX_DELTA), -config_.cost_delta_max),
                config_.cost_delta_max);
        }
        warm_controls = inner_solver_->controls();
    }
    report->mu_initial_calibrated = outer->calibratedMu();
    report->mu_final = outer->mu();
    report->mu_amplitude_final = multipliers->amplitude_mu.maxCoeff();
    report->mu_gating_final = gating_mu;
}

ApaILQRStageOneResult ApaILQRSolver::solveStageOne(
    const iLQRReference& reference,
    const iLQRAlignedVec<iLQRState>& warm_start_states,
    const iLQRAlignedVec<iLQRControl>& warm_start_controls) {
    const std::size_t num_poses = reference.poses.size();
    if (num_poses < 2) {
        throw std::invalid_argument("ApaILQRSolver: 参考位姿数量必须 >= 2");
    }
    const std::size_t num_steps = num_poses - 1;
    if (reference.initial_states.size() != num_poses ||
        reference.initial_controls.size() != num_steps) {
        throw std::invalid_argument(
            "ApaILQRSolver: 状态/控制初值尺寸必须为 N+1 / N");
    }
    if (warm_start_states.size() != num_poses ||
        warm_start_controls.size() != num_steps) {
        throw std::invalid_argument(
            "ApaILQRSolver: 热启动状态/控制尺寸必须为 N+1 / N");
    }
    ApaILQRStageOneResult result;
    outer_loop_.reset();
    auto multipliers = outer_loop_.makeInitialMultipliers(num_steps);
    const auto exempt_mask = outer_loop_.makeAnnealExemptMask(reference);
    iLQRAlignedVec<iLQRState> warm_virtual;
    runOuterLoop(reference, &outer_loop_, &multipliers, warm_start_states,
                 warm_start_controls, &warm_virtual,
                 config_.outer_max_outer_iterations,
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

ApaILQRStageTwoResult ApaILQRSolver::solveStageTwo(
    const iLQRReference& reference, const iLQRGatingPlan& gating_plan,
    const iLQRAlignedVec<iLQRState>& warm_states,
    const iLQRAlignedVec<iLQRControl>& warm_controls, double tracking_weight,
    const iLQRCostMultiplierState* dual_seed) {
    const std::size_t num_poses = reference.poses.size();
    if (num_poses < 2) {
        throw std::invalid_argument("ApaILQRSolver: 参考位姿数量必须 >= 2");
    }
    const std::size_t num_steps = num_poses - 1;
    if (warm_states.size() != num_poses || warm_controls.size() != num_steps) {
        throw std::invalid_argument(
            "ApaILQRSolver: 阶段二热启动尺寸必须为 N+1 / N");
    }
    if (!std::isfinite(tracking_weight) || tracking_weight < 0.0) {
        throw std::invalid_argument(
            "ApaILQRSolver: 阶段二跟踪权重必须为非负有限值");
    }
    // 门控计划完整性：尺寸/查表自洽（残缺计划会让三类门控静默错位）
    if (gating_plan.sign_gate.size() != num_poses ||
        gating_plan.seam_lookup.size() != num_poses ||
        gating_plan.dwell_v_cap.size() != num_poses) {
        throw std::invalid_argument("ApaILQRSolver: 门控计划尺寸必须为 N+1");
    }
    for (std::size_t j = 0; j < gating_plan.seam_indices.size(); ++j) {
        const std::size_t seam = gating_plan.seam_indices[j];
        if (seam >= num_poses ||
            gating_plan.seam_lookup[seam] != static_cast<int>(j)) {
            throw std::invalid_argument(
                "ApaILQRSolver: 门控计划接缝索引与查表不自洽");
        }
    }
    for (std::size_t k = 0; k < num_poses; ++k) {
        if (gating_plan.seam_lookup[k] >= 0 &&
            gating_plan.seam_indices[static_cast<std::size_t>(
                gating_plan.seam_lookup[k])] != k) {
            throw std::invalid_argument(
                "ApaILQRSolver: 门控计划接缝查表存在游离条目");
        }
    }
    // 阶段二局部配置：逐元素门控按阶段二开关、对偶种子提升首轮 μ 下限
    iLQRConfig stage_two_config = config_;
    stage_two_config.outer_amplitude_mu_per_element =
        config_.outer_amplitude_mu_per_element_stage_two;
    if (dual_seed != nullptr && dual_seed->terminal_mu.size() > 0) {
        const double seed_mu = dual_seed->terminal_mu.maxCoeff();
        stage_two_config.outer_first_round_mu = seed_mu;
        stage_two_config.outer_mu_min =
            std::max(stage_two_config.outer_mu_min, seed_mu);
    }
    AlOuterLoop outer(stage_two_config);
    auto multipliers = iLQRCostMultiplierState::MakeStageTwoZero(
        num_steps, gating_plan.seam_indices.size());
    multipliers.amplitude_mu.setConstant(
        stage_two_config.outer_amplitude_mu_initial);
    multipliers.terminal_mu.setConstant(
        stage_two_config.outer_first_round_mu);
    if (dual_seed != nullptr) {
        if (dual_seed->terminal_lambda.size() > 0) {
            multipliers.terminal_lambda = dual_seed->terminal_lambda;
            multipliers.terminal_mu = dual_seed->terminal_mu;
        }
        const auto amplitude_size =
            static_cast<Eigen::Index>(ILQR_AMPLITUDE_CONSTRAINT_DIM * num_steps);
        if (dual_seed->amplitude_lambda.size() == amplitude_size &&
            dual_seed->amplitude_mu.size() == amplitude_size) {
            multipliers.amplitude_lambda = dual_seed->amplitude_lambda;
            multipliers.amplitude_mu = dual_seed->amplitude_mu;
        }
    }
    multipliers.gating_sign_mu.setConstant(config_.gating_mu_initial);
    multipliers.gating_seam_mu.setConstant(config_.gating_mu_initial);
    multipliers.gating_dwell_mu.setConstant(config_.gating_mu_initial);
    ApaILQRStageTwoResult result;
    iLQRAlignedVec<iLQRState> warm_virtual;
    runOuterLoop(reference, &outer, &multipliers, warm_states, warm_controls,
                 &warm_virtual, config_.stage_two_max_outer_iterations,
                 tracking_weight,
                 /*anneal_exempt_mask=*/nullptr, &gating_plan, &result.report,
                 &result.max_sign_violation, &result.max_dwell_violation,
                 &result.max_seam_speed, &result.gating_ok);
    result.states = inner_solver_->states();
    result.controls = inner_solver_->controls();
    return result;
}

ApaILQRSolver::GatingSnapshot ApaILQRSolver::measureGating(
    const iLQRGatingPlan& plan, const iLQRAlignedVec<iLQRState>& states) const {
    const std::size_t num_poses = states.size();
    if (num_poses < 2 || plan.sign_gate.size() != num_poses ||
        plan.seam_lookup.size() != num_poses ||
        plan.dwell_v_cap.size() != num_poses) {
        throw std::invalid_argument(
            "ApaILQRSolver: 门控量测的状态/计划尺寸必须为 N+1");
    }
    GatingSnapshot snapshot;
    const auto nodes = static_cast<Eigen::Index>(num_poses);
    snapshot.sign_g = Eigen::VectorXd::Zero(nodes);
    snapshot.dwell_g = Eigen::VectorXd::Zero(nodes);
    snapshot.seam_c = Eigen::VectorXd::Zero(
        static_cast<Eigen::Index>(plan.seam_indices.size()));
    double violation_sq = 0.0;
    for (std::size_t k = 0; k < num_poses; ++k) {
        const double v = states[k](ILQR_IDX_V);
        // 残差公式与代价求值层同一来源（量测与惩罚失配会破坏 AL 收敛性）
        const int sign = plan.sign_gate[k];
        if (sign != 0) {
            const double g = iLQRCostEvaluator::SignGateResidual(sign, v);
            snapshot.sign_g(static_cast<Eigen::Index>(k)) = g;
            snapshot.max_sign_violation =
                std::max(snapshot.max_sign_violation, std::max(0.0, g));
            violation_sq += std::pow(std::max(0.0, g), 2.0);
        }
        const int seam = plan.seam_lookup[k];
        if (seam >= 0) {
            const double c = iLQRCostEvaluator::SeamResidual(v);
            snapshot.seam_c(seam) = c;
            snapshot.max_seam_speed =
                std::max(snapshot.max_seam_speed, std::abs(c));
            violation_sq += c * c;
        }
        const double cap = plan.dwell_v_cap[k];
        if (cap > 0.0) {
            const double g = iLQRCostEvaluator::DwellResidual(v, cap);
            snapshot.dwell_g(static_cast<Eigen::Index>(k)) = g;
            snapshot.max_dwell_violation = std::max(
                snapshot.max_dwell_violation, std::max(0.0, std::abs(v) - cap));
            violation_sq += std::pow(std::max(0.0, g), 2.0);
        }
    }
    snapshot.violation_norm = std::sqrt(violation_sq);
    return snapshot;
}

bool ApaILQRSolver::updateGating(const GatingSnapshot& snapshot,
                                iLQRCostMultiplierState* multipliers,
                                double* gating_mu,
                                double* prev_violation,
                                bool* out_wanted) const {
    if (multipliers == nullptr || gating_mu == nullptr ||
        prev_violation == nullptr ||
        multipliers->gating_sign_lambda.size() != snapshot.sign_g.size() ||
        multipliers->gating_dwell_lambda.size() != snapshot.dwell_g.size() ||
        multipliers->gating_seam_lambda.size() != snapshot.seam_c.size()) {
        throw std::invalid_argument("ApaILQRSolver: 门控乘子尺寸必须与快照一致");
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
    const bool wanted =
        *prev_violation >= 0.0 &&
        snapshot.violation_norm >
            config_.outer_mu_gate_kappa * *prev_violation;
    bool increased = false;
    if (wanted) {
        *gating_mu = std::min(config_.outer_mu_growth_factor * *gating_mu,
                              config_.gating_mu_max);
        increased = true;
    }
    if (out_wanted != nullptr) {
        *out_wanted = wanted;
    }
    *prev_violation = snapshot.violation_norm;
    multipliers->gating_sign_mu.setConstant(*gating_mu);
    multipliers->gating_dwell_mu.setConstant(*gating_mu);
    multipliers->gating_seam_mu.setConstant(*gating_mu);
    return increased;
}
}  // namespace apa_post_processor
