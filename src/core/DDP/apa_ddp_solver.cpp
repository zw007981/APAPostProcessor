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
    ApaDdpReport& report = result.report;
    report.history.reserve(
        static_cast<std::size_t>(config_.outer.max_outer_iterations));
    // 外层调度复位（同一实例多次求解复用）+ 首轮乘子与退火豁免掩码
    outer_loop_.reset();
    auto multipliers = outer_loop_.makeInitialMultipliers(num_steps);
    const auto exempt_mask = outer_loop_.makeAnnealExemptMask(reference);
    // 候选待融段掩码（按融化平衡式临界比生成，整轮求解期间不变）
    const auto candidate_mask = outer_loop_.makeMeltCandidateMask(reference);
    // 零乘子求值用于逐轮基础代价 J_s′（μ⁰ 标定与诊断）
    const auto zero_multipliers = DdpCostMultiplierState::MakeZero(num_steps);
    // 首轮以调用方给定的轨迹启动（冷启动入口传入的是参考自带的前端
    // 初值），后续轮次以上轮收敛轨迹（经 δ 投影）热启动
    DdpAlignedVec<DdpState> warm_states = warm_start_states;
    DdpAlignedVec<DdpControl> warm_controls = warm_start_controls;
    // 逃逸指标：上一轮解长度（环比基线，首轮为 0 表示尚无历史）
    double prev_solution_length = 0.0;
    for (int round = 0; round < config_.outer.max_outer_iterations; ++round) {
        DdpCostInput cost_input;
        cost_input.tracking_weight = outer_loop_.trackingWeight();
        cost_input.anneal_exempt_mask = &exempt_mask;
        // 换挡代理门宽随外层退火（weight_shift=0 时该字段不被求值层消费，
        // 赋值无副作用）
        cost_input.shift_beta = outer_loop_.shiftBeta();
        // 候选待融段的深退火权重（掩码为空时求值层不消费，无副作用）
        cost_input.melt_candidate_mask = &candidate_mask;
        cost_input.candidate_tracking_weight =
            outer_loop_.candidateTrackingWeight();
        // 内层求解（固定 λ/μ 的增广问题）
        const double mu_round = outer_loop_.mu();
        MsIlqrResult inner_result = inner_solver_->solve(
            reference, multipliers, cost_input, warm_states, warm_controls);
        if (inner_result.status == MsIlqrStatus::REGULARIZATION_OVERFLOW) {
            // 冷重启兜底：溢出部分源于陈旧 QP 活动集热启动与衰减到地板
            // 的 ρ_reg（box-DDP 固有僵局，见 known-limitations 登记），
            // 全新实例在同一起点/乘子下重解通常可通过；μ_m 与 ρ_reg 随
            // 实例重置回初值。重试仍失败才判定致命失败
            inner_solver_ = std::make_unique<MsIlqrSolver>(
                config_.inner, dynamics_, cost_evaluator_);
            ++report.inner_restarts;
            inner_result = inner_solver_->solve(
                reference, multipliers, cost_input, warm_states, warm_controls);
        }
        report.total_inner_iterations += inner_result.iterations;
        report.domain_guard_rejections += inner_result.domain_guard_rejections;
        if (inner_result.status == MsIlqrStatus::REGULARIZATION_OVERFLOW) {
            // anytime 性质：内层名义轨迹仍是最后接受的可行迭代点
            report.status = ApaDdpStatus::INNER_SOLVER_FAILED;
            break;
        }
        // 内层迭代超限不致命：沿用当前迭代点继续外层调度（诊断入记录）
        const auto snapshot = outer_loop_.measure(
            reference, inner_solver_->states(), inner_solver_->defects());
        // 退火终点自适应：逃逸指标量测与上报（默认全关、零开销）——解长度
        // 环比、相对参考位姿的最大横向偏离、打靶缺陷范数，任一越阈即
        // 冻结退火（守护「解仍在同伦类内」）
        if (config_.outer.anneal_freeze_length_growth > 0.0 ||
            config_.outer.anneal_freeze_lateral_deviation > 0.0 ||
            config_.outer.anneal_freeze_defect > 0.0 ||
            config_.outer.anneal_freeze_ref_length_ratio > 0.0) {
            double solution_length = 0.0;
            double max_deviation = 0.0;
            for (std::size_t k = 1; k < inner_solver_->states().size(); ++k) {
                solution_length +=
                    std::hypot(inner_solver_->states()[k](DDP_IDX_X) -
                                   inner_solver_->states()[k - 1](DDP_IDX_X),
                               inner_solver_->states()[k](DDP_IDX_Y) -
                                   inner_solver_->states()[k - 1](DDP_IDX_Y));
            }
            for (std::size_t k = 0; k < reference.poses.size(); ++k) {
                max_deviation =
                    std::max(max_deviation,
                             std::hypot(inner_solver_->states()[k](DDP_IDX_X) -
                                            reference.poses[k].x,
                                        inner_solver_->states()[k](DDP_IDX_Y) -
                                            reference.poses[k].y));
            }
            const double growth = prev_solution_length > 0.0
                                      ? solution_length / prev_solution_length
                                      : 1.0;
            prev_solution_length = solution_length;
            // 参考总长 = 网格步数 × 实际间距（全长归一后的均匀网格）
            const double reference_length =
                static_cast<double>(num_steps) * reference.ds;
            const double ref_length_ratio =
                reference_length > 0.0 ? solution_length / reference_length
                                       : 1.0;
            outer_loop_.reportEscapeIndicators(growth, max_deviation,
                                               snapshot.defect_norm_inf,
                                               ref_length_ratio);
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
        // 终态指标随每轮刷新：循环任意出口都保留最后一轮量测
        const auto check = outer_loop_.checkTermination(snapshot);
        report.terminal_position_error = snapshot.terminal_position_error;
        report.terminal_heading_error_deg = snapshot.terminal_heading_error_deg;
        report.max_amplitude_violation = snapshot.max_amplitude_violation;
        report.defect_norm_inf = snapshot.defect_norm_inf;
        report.final_cost = base_cost;
        report.terminal_ok = check.terminal_ok;
        report.inequality_ok = check.inequality_ok;
        report.defect_ok = check.defect_ok;
        report.outer_iterations = round + 1;
        if (check.converged()) {
            report.status = ApaDdpStatus::CONVERGED;
            report.history.push_back(std::move(record));
            break;
        }
        if (round + 1 >= config_.outer.max_outer_iterations) {
            report.status = ApaDdpStatus::MAX_OUTER_ITERATIONS;
            report.history.push_back(std::move(record));
            break;
        }
        // 未达标且仍有预算：λ 更新 + 分组门控 μ 增长 + 退火推进，
        // 下轮内层以本轮收敛轨迹热启动
        record.mu_increased =
            outer_loop_.update(snapshot, base_cost, &multipliers);
        report.history.push_back(std::move(record));
        // 热启动投影：把 δ 裁剪进物理边界。换挡区 v≈0 时 tanδ 允许
        // 优化器经 ±π/2 奇异区"原地免费转头"（AL 不等式在 g<0 时
        // 零曲率零梯度、对该漏洞不可见），一旦落入奇异盆地内层线性化
        // 失效、后续轮次卡死。投影把下轮起点拉回良态区，已被违反的
        // 节点则经本轮 λ 累积（λ>0 时约束对 g<0 亦产生内屏障）在
        // 重解中保持有界；投影引入的缺陷由 MS 打靶机制自然吸收
        warm_states = inner_solver_->states();
        for (auto& state : warm_states) {
            state(DDP_IDX_DELTA) = std::min(
                std::max(state(DDP_IDX_DELTA), -config_.cost.delta_max),
                config_.cost.delta_max);
        }
        warm_controls = inner_solver_->controls();
    }
    report.mu_initial_calibrated = outer_loop_.calibratedMu();
    report.mu_final = outer_loop_.mu();
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
    if (!std::isfinite(config_.seed_mu_cap_ratio) ||
        config_.seed_mu_cap_ratio < 0.0) {
        throw std::invalid_argument(
            "ApaDdpSolver: 种子 μ 上限比率必须为非负有限值（0 = 关闭）");
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
    ApaDdpStageTwoResult result;
    ApaDdpReport& report = result.report;
    report.history.reserve(
        static_cast<std::size_t>(config_.stage_two_max_outer_iterations));
    // 阶段二外层调度：与阶段一同源（自适应 μ⁰ 标定 + 终点/幅值分组门控
    // 增长），但跟踪权重恒定不退火——精化阶段不再融化 maneuver，跟踪项
    // 只承担对热启动邻域的温和保持，冻结在调用方给定的量级；若回退到
    // w_ref,0 的强跟踪，会与标定后的终端罚权重形成失衡平衡（实测终端
    // 误差被拖到 0.1 m 量级、收敛轮数翻倍）
    AlOuterLoopConfig outer_config = config_.outer;
    if (dual_seed != nullptr && dual_seed->terminal_mu.size() > 0) {
        // 对偶热启动：首轮临时罚权重与 μ⁰ 标定下限都抬到种子水平——
        // 阶段一建立的终端平衡由 λ/μ 承载，标定公式在小尺度问题上会把
        // μ⁰ 重新压回下限（实测把已收敛的终端平衡重新打开）
        const double seed_mu = dual_seed->terminal_mu.maxCoeff();
        double effective_seed = seed_mu;
        if (config_.seed_mu_cap_ratio > 0.0) {
            // 量级自适应上限：以阶段二自身在热启动轨迹上按 ALTRO clip
            // 公式量测的标定初值 μ̂（不经种子抬升的版本）为参照，只截断
            // 「种子已达 μ_max」的病态情形，正常量级的种子照常抬升。
            // 量测与逐轮诊断的 base_cost 同口径（零乘子 + 门控计划在场）
            const auto zero_multipliers =
                DdpCostMultiplierState::MakeStageTwoZero(
                    num_steps, gating_plan.seam_indices.size());
            DdpCostInput cal_input;
            cal_input.tracking_weight = tracking_weight;
            cal_input.gating_plan = &gating_plan;
            const double base_cost0 =
                cost_evaluator_
                    ->evaluate(reference, warm_states, warm_controls,
                               zero_multipliers, cal_input)
                    .total_cost;
            // 终端残差范数与外层量测同一公式（c=[x−xg,y−yg,wrap(θ−θg),v,a]）
            const DdpState& x_final = warm_states.back();
            const Pose& goal = reference.poses.back();
            const double c_norm = std::sqrt(
                std::pow(x_final(DDP_IDX_X) - goal.x, 2.0) +
                std::pow(x_final(DDP_IDX_Y) - goal.y, 2.0) +
                std::pow(WrapAngle(x_final(DDP_IDX_THETA) - goal.theta), 2.0) +
                std::pow(x_final(DDP_IDX_V), 2.0) +
                std::pow(x_final(DDP_IDX_A), 2.0));
            const double mu_hat = std::min(
                std::max(base_cost0 /
                             std::max(c_norm * c_norm, outer_config.epsilon_mu),
                         outer_config.mu_min),
                outer_config.mu_max);
            effective_seed =
                std::min(seed_mu, config_.seed_mu_cap_ratio * mu_hat);
        }
        outer_config.first_round_mu = effective_seed;
        outer_config.mu_min = std::max(outer_config.mu_min, effective_seed);
    }
    AlOuterLoop outer(outer_config, config_.cost);
    auto multipliers = DdpCostMultiplierState::MakeStageTwoZero(
        num_steps, gating_plan.seam_indices.size());
    multipliers.amplitude_mu.setConstant(outer_config.amplitude_mu_initial);
    multipliers.terminal_mu.setConstant(outer_config.first_round_mu);
    if (dual_seed != nullptr) {
        // 终端乘子直接续接（尺寸恒为 5 维）；幅值乘子仅在网格一致时续接
        // （修剪会删除网格点导致 5N 尺寸变化，失配时从零重建——其违反度
        // 在热启动附近很小，λ 累积恢复很快）
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
    // 零乘子求值用于逐轮基础代价 J_s′（μ⁰ 标定与诊断；门控向量必须为
    // 零值在场，否则触发求值层的半配置拒绝）
    auto zero_multipliers = DdpCostMultiplierState::MakeStageTwoZero(
        num_steps, gating_plan.seam_indices.size());
    double gating_mu = config_.gating_mu_initial;
    double prev_gating_violation = -1.0;
    // 首轮以调用方给定的修剪后轨迹热启动，后续轮次以上轮收敛轨迹
    // （经 δ 投影）热启动
    DdpAlignedVec<DdpState> round_warm_states = warm_states;
    DdpAlignedVec<DdpControl> round_warm_controls = warm_controls;
    for (int round = 0; round < config_.stage_two_max_outer_iterations;
         ++round) {
        DdpCostInput cost_input;
        cost_input.tracking_weight = tracking_weight;
        cost_input.gating_plan = &gating_plan;
        const double mu_round = outer.mu();
        MsIlqrResult inner_result =
            inner_solver_->solve(reference, multipliers, cost_input,
                                 round_warm_states, round_warm_controls);
        if (inner_result.status == MsIlqrStatus::REGULARIZATION_OVERFLOW) {
            // 冷重启兜底（与阶段一同一约定：陈旧 QP 活动集僵局）
            inner_solver_ = std::make_unique<MsIlqrSolver>(
                config_.inner, dynamics_, cost_evaluator_);
            ++report.inner_restarts;
            inner_result =
                inner_solver_->solve(reference, multipliers, cost_input,
                                     round_warm_states, round_warm_controls);
        }
        report.total_inner_iterations += inner_result.iterations;
        report.domain_guard_rejections += inner_result.domain_guard_rejections;
        if (inner_result.status == MsIlqrStatus::REGULARIZATION_OVERFLOW) {
            report.status = ApaDdpStatus::INNER_SOLVER_FAILED;
            break;
        }
        // 内层迭代超限不致命：沿用当前迭代点继续外层调度
        const auto snapshot = outer.measure(reference, inner_solver_->states(),
                                            inner_solver_->defects());
        const auto gating_snapshot =
            measureGating(gating_plan, inner_solver_->states());
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
        // 联合终止判据：阶段一三项判据 + 门控违反度（线性量测）
        const auto check = outer.checkTermination(snapshot);
        const bool gating_ok =
            gating_snapshot.max_sign_violation <= config_.gating_tol &&
            gating_snapshot.max_dwell_violation <= config_.gating_tol &&
            gating_snapshot.max_seam_speed <= config_.gating_tol;
        // 终态指标随每轮刷新：循环任意出口都保留最后一轮量测
        report.terminal_position_error = snapshot.terminal_position_error;
        report.terminal_heading_error_deg = snapshot.terminal_heading_error_deg;
        report.max_amplitude_violation = snapshot.max_amplitude_violation;
        report.defect_norm_inf = snapshot.defect_norm_inf;
        report.final_cost = base_cost;
        report.terminal_ok = check.terminal_ok;
        report.inequality_ok = check.inequality_ok;
        report.defect_ok = check.defect_ok;
        report.outer_iterations = round + 1;
        result.max_sign_violation = gating_snapshot.max_sign_violation;
        result.max_dwell_violation = gating_snapshot.max_dwell_violation;
        result.max_seam_speed = gating_snapshot.max_seam_speed;
        result.gating_ok = gating_ok;
        if (check.converged() && gating_ok) {
            report.status = ApaDdpStatus::CONVERGED;
            report.history.push_back(std::move(record));
            break;
        }
        if (round + 1 >= config_.stage_two_max_outer_iterations) {
            report.status = ApaDdpStatus::MAX_OUTER_ITERATIONS;
            report.history.push_back(std::move(record));
            break;
        }
        // 终点/幅值乘子与 μ 调度（与阶段一同源），随后门控乘子更新
        record.mu_increased = outer.update(snapshot, base_cost, &multipliers);
        report.history.push_back(std::move(record));
        updateGating(gating_snapshot, &multipliers, &gating_mu,
                     &prev_gating_violation);
        // 热启动投影：δ 裁剪进物理边界（与阶段一同一防线）
        round_warm_states = inner_solver_->states();
        for (auto& state : round_warm_states) {
            state(DDP_IDX_DELTA) = std::min(
                std::max(state(DDP_IDX_DELTA), -config_.cost.delta_max),
                config_.cost.delta_max);
        }
        round_warm_controls = inner_solver_->controls();
    }
    report.mu_initial_calibrated = outer.calibratedMu();
    report.mu_final = outer.mu();
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
    // 此后仅当联合违反度未充分下降（>κ·上轮）才按 φ 倍提升。
    // 边界语义说明：取严格 `>`，即违反度恰好下降到 κ·上轮（含 ±ULP 级
    // 抖动）时判为"已充分下降"、本轮不增长。门控阈值 κ 本身是工程启发
    // 常数，边界落点无论判向哪一侧，效果都只是一轮 μ 是否提升——判
    // "充分"则推迟一轮硬化、判"不足"则提前一轮硬化，下一轮量测会
    // 自动修正排程方向，无累积误差；故不引入额外绝对容差（反而会把
    // 排程行为绑定到违反度的具体量级上）
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
