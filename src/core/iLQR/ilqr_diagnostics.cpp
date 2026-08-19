#include "ilqr_diagnostics.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <utility>

#include "../../util/logger.h"
#include "../NMPC/vehicle_circle_geometry.h"

namespace apa_post_processor {
namespace {

const char* iLQRPostStageStatusName(iLQRPostStageStatus status) {
    switch (status) {
        case iLQRPostStageStatus::SUCCESS:
            return "SUCCESS";
        case iLQRPostStageStatus::SUCCESS_STAGE_ONE_ONLY:
            return "SUCCESS_STAGE_ONE_ONLY";
        case iLQRPostStageStatus::STAGE_ONE_NOT_CONVERGED:
            return "STAGE_ONE_NOT_CONVERGED";
        case iLQRPostStageStatus::PIVOT_DETECTED:
            return "PIVOT_DETECTED";
        case iLQRPostStageStatus::PRUNED_PATH_DEGENERATE:
            return "PRUNED_PATH_DEGENERATE";
        case iLQRPostStageStatus::STAGE_TWO_NOT_CONVERGED:
            return "STAGE_TWO_NOT_CONVERGED";
        case iLQRPostStageStatus::VALIDATION_FAILED:
            return "VALIDATION_FAILED";
    }
    return "UNKNOWN";
}
}  // namespace

std::string iLQRDiagnostics::BuildFailureMessage(
    const iLQRPostStageResult& post) {
    std::string msg =
        std::string("iLQR post stage ") + iLQRPostStageStatusName(post.status) +
        ": " + post.diagnostics.failed_check +
        " measured=" + std::to_string(post.diagnostics.measured_value) +
        " threshold=" + std::to_string(post.diagnostics.threshold);
    if (!post.diagnostics.degraded_reason.empty()) {
        msg += ", degraded_reason=" + post.diagnostics.degraded_reason;
    }
    msg += ", fell back to original path";
    return msg;
}

double iLQRDiagnostics::MeasureIntrusion(
    const iLQRAlignedVec<iLQRState>& states,
    const VehicleFootprintModel& footprint_model, const ESDFMap& esdf_map) {
    const double outer_radius = footprint_model.getOuterRadius();
    const auto outer_circles =
        vehicle_circle_geometry::ExtractLocalCircleCenters(footprint_model,
                                                           CircleType::OUTER);
    double intrusion = 0.0;
    for (const auto& state : states) {
        const double cos_theta = std::cos(state(ILQR_IDX_THETA));
        const double sin_theta = std::sin(state(ILQR_IDX_THETA));
        for (const auto& local : outer_circles) {
            const double wx = state(ILQR_IDX_X) + local.x() * cos_theta -
                              local.y() * sin_theta;
            const double wy = state(ILQR_IDX_Y) + local.x() * sin_theta +
                              local.y() * cos_theta;
            intrusion =
                std::max(intrusion, outer_radius - esdf_map.getDist(wx, wy));
        }
    }
    return intrusion;
}

void iLQRDiagnostics::LogWorstAmplitudeViolation(
    const iLQRAlignedVec<iLQRState>& states, const iLQRConfig& config) {
    double worst_violation = 0.0;
    std::size_t worst_index = 0;
    std::string worst_name;
    double worst_state_value = 0.0;
    for (std::size_t k = 0; k < states.size(); ++k) {
        const auto& x = states[k];
        const std::array<std::pair<const char*, double>, 5> checks{{
            {"v", x(ILQR_IDX_V) * x(ILQR_IDX_V) - config.cost_v_max * config.cost_v_max},
            {"a", x(ILQR_IDX_A) * x(ILQR_IDX_A) - config.cost_a_max * config.cost_a_max},
            {"omega", x(ILQR_IDX_OMEGA) * x(ILQR_IDX_OMEGA) -
                          config.cost_omega_max * config.cost_omega_max},
            {"delta+", x(ILQR_IDX_DELTA) - config.cost_delta_max},
            {"delta-", -x(ILQR_IDX_DELTA) - config.cost_delta_max},
        }};
        for (const auto& [name, g] : checks) {
            if (g > worst_violation) {
                worst_violation = g;
                worst_index = k;
                worst_name = name;
            }
        }
    }
    if (worst_violation > 0.0) {
        const auto& x = states[worst_index];
        worst_state_value =
            worst_name[0] == 'v'
                ? x(ILQR_IDX_V)
                : (worst_name[0] == 'a'
                       ? x(ILQR_IDX_A)
                       : (worst_name[0] == 'o' ? x(ILQR_IDX_OMEGA)
                                               : x(ILQR_IDX_DELTA)));
        LOG_FMT_WARN(
            "iLQR stage one worst amplitude: constraint={}, "
            "node={}/{}, violation={:.4f}, state_value={:.4f}, "
            "v={:.4f}, pose=({:.3f}, {:.3f}, {:.3f})",
            worst_name, worst_index, states.size(), worst_violation,
            worst_state_value, x(ILQR_IDX_V), x(ILQR_IDX_X), x(ILQR_IDX_Y),
            x(ILQR_IDX_THETA));
    }
}

void iLQRDiagnostics::LogStageOneReport(
    const ApaILQRStageOneResult& stage_one, const iLQRConfig& config,
    const VehicleFootprintModel& footprint_model, const ESDFMap& esdf_map) {
    LOG_FMT_INFO(
        "iLQR stage one: status={}, outer={}, inner_total={}, restarts={}, "
        "term_err={:.4f} m/{:.3f} deg, ineq={:.4f}, "
        "defect={:.2e}, mu_final={:.1f}",
        static_cast<int>(stage_one.report.status),
        stage_one.report.outer_iterations,
        stage_one.report.total_inner_iterations,
        stage_one.report.inner_restarts,
        stage_one.report.terminal_position_error,
        stage_one.report.terminal_heading_error_deg,
        stage_one.report.max_amplitude_violation,
        stage_one.report.defect_norm_inf, stage_one.report.mu_final);

    if (stage_one.report.status == ApaILQRStatus::CONVERGED) {
        LOG_FMT_INFO("iLQR stage one collision intrusion: {:.4f} m",
                     MeasureIntrusion(stage_one.states, footprint_model,
                                      esdf_map));
    }

    if (stage_one.report.status != ApaILQRStatus::CONVERGED) {
        for (const auto& round : stage_one.report.history) {
            LOG_FMT_WARN(
                "iLQR stage one round {}: w_ref={:.3f}, mu={:.1f}, "
                "term_err={:.4f} m/{:.3f} deg, ineq={:.4f}, "
                "defect={:.2e}, inner_status={}, inner_iter={}",
                round.outer_index, round.tracking_weight, round.mu,
                round.terminal_position_error, round.terminal_heading_error_deg,
                round.max_amplitude_violation, round.defect_norm_inf,
                static_cast<int>(round.inner_status), round.inner_iterations);
        }
        LogWorstAmplitudeViolation(stage_one.states, config);
    }
}

void iLQRDiagnostics::LogPostStageReport(
    const iLQRPostStageResult& post, const ApaILQRStageOneResult& stage_one,
    const ESDFMap& esdf_map) {
    LOG_FMT_INFO(
        "iLQR post stage: status={}, maneuvers {}->{}, seams={}",
        static_cast<int>(post.status), post.diagnostics.input_maneuver_count,
        post.diagnostics.output_maneuver_count, post.diagnostics.seams.size());
    LOG_FMT_INFO(
        "iLQR domain diagnostics: out_of_map_queries={}, "
        "domain_guard_rejections={}",
        esdf_map.outOfMapQueryCount(),
        stage_one.report.domain_guard_rejections);
    // 阶段二逐轮诊断：降级输出（阶段一候选被采用）时同样需要取证——
    // 阶段二不收敛即丢弃其解，但 μ 轨迹说明「旋钮是否已耗尽」
    if (post.stage_two.has_value()) {
        const auto& report = post.stage_two->report;
        LOG_FMT_INFO(
            "iLQR stage two diagnostics: status={}, outer={}, "
            "mu_final={:.1f}, mu_amp_final={:.1f}, mu_gating_final={:.1f}, "
            "term_err={:.4f} m/{:.3f} deg, ineq={:.4f}, "
            "defect={:.2e}, sign_viol={:.4f}, dwell_viol={:.4f}, "
            "seam_speed={:.4f}, gating_ok={}",
            static_cast<int>(report.status), report.outer_iterations,
            report.mu_final, report.mu_amplitude_final,
            report.mu_gating_final, report.terminal_position_error,
            report.terminal_heading_error_deg, report.max_amplitude_violation,
            report.defect_norm_inf, post.stage_two->max_sign_violation,
            post.stage_two->max_dwell_violation, post.stage_two->max_seam_speed,
            post.stage_two->gating_ok);
        if (report.status != ApaILQRStatus::CONVERGED) {
            for (const auto& round : report.history) {
                LOG_FMT_WARN(
                    "iLQR stage two round {}: w_ref={:.3f}, mu={:.1f}, "
                    "mu_amp={:.1f}, mu_gating={:.1f}, "
                    "term_err={:.4f} m/{:.3f} deg, ineq={:.4f}, "
                    "defect={:.2e}, inner_status={}, inner_iter={}",
                    round.outer_index, round.tracking_weight, round.mu,
                    round.mu_amplitude, round.mu_gating,
                    round.terminal_position_error,
                    round.terminal_heading_error_deg,
                    round.max_amplitude_violation, round.defect_norm_inf,
                    static_cast<int>(round.inner_status),
                    round.inner_iterations);
            }
        }
    }
}

void iLQRDiagnostics::LogFailureDiagnostics(
    const iLQRPostStageResult& post,
    const VehicleFootprintModel& footprint_model, const ESDFMap& esdf_map) {
    LOG_FMT_WARN("{}", BuildFailureMessage(post));

    if (post.stage_two.has_value()) {
        const auto& report = post.stage_two->report;
        LOG_FMT_WARN(
            "iLQR stage two diagnostics: status={}, outer={}, "
            "term_err={:.4f} m/{:.3f} deg, ineq={:.4f}, "
            "defect={:.2e}, sign_viol={:.4f}, dwell_viol={:.4f}, "
            "seam_speed={:.4f}, gating_ok={}",
            static_cast<int>(report.status), report.outer_iterations,
            report.terminal_position_error, report.terminal_heading_error_deg,
            report.max_amplitude_violation, report.defect_norm_inf,
            post.stage_two->max_sign_violation,
            post.stage_two->max_dwell_violation, post.stage_two->max_seam_speed,
            post.stage_two->gating_ok);

        LogStageTwoIntrusion(post, footprint_model, esdf_map);
        if (report.status != ApaILQRStatus::CONVERGED) {
            for (const auto& round : report.history) {
                LOG_FMT_WARN(
                    "iLQR stage two round {}: w_ref={:.3f}, mu={:.1f}, "
                    "mu_amp={:.1f}, mu_gating={:.1f}, "
                    "term_err={:.4f} m/{:.3f} deg, ineq={:.4f}, "
                    "defect={:.2e}, inner_status={}, inner_iter={}",
                    round.outer_index, round.tracking_weight, round.mu,
                    round.mu_amplitude, round.mu_gating,
                    round.terminal_position_error,
                    round.terminal_heading_error_deg,
                    round.max_amplitude_violation, round.defect_norm_inf,
                    static_cast<int>(round.inner_status),
                    round.inner_iterations);
            }
        }
    }
    for (const auto& seam : post.diagnostics.seams) {
        LOG_FMT_WARN(
            "iLQR seam report: index={}, delta_delta={:.3f}, "
            "t_resteer={:.2f}, t_dwell={:.2f}, dwell={:.2f}, "
            "seam_speed={:.4f}, window_max_speed={:.4f}, "
            "window_end_omega={:.4f}",
            seam.seam_index, seam.delta_delta, seam.t_resteer, seam.t_dwell,
            seam.dwell_duration, seam.seam_speed, seam.window_max_speed,
            seam.window_end_omega);
    }
}

void iLQRDiagnostics::LogStageTwoIntrusion(
    const iLQRPostStageResult& post,
    const VehicleFootprintModel& footprint_model, const ESDFMap& esdf_map) {
    const double outer_radius = footprint_model.getOuterRadius();
    const auto outer_circles =
        vehicle_circle_geometry::ExtractLocalCircleCenters(footprint_model,
                                                           CircleType::OUTER);
    double stage_two_intrusion = 0.0;
    std::size_t worst_node = 0;
    double worst_wx = 0.0;
    double worst_wy = 0.0;
    for (std::size_t k = 0; k < post.stage_two->states.size(); ++k) {
        const auto& state = post.stage_two->states[k];
        const double cos_theta = std::cos(state(ILQR_IDX_THETA));
        const double sin_theta = std::sin(state(ILQR_IDX_THETA));
        for (const auto& local : outer_circles) {
            const double wx = state(ILQR_IDX_X) + local.x() * cos_theta -
                              local.y() * sin_theta;
            const double wy = state(ILQR_IDX_Y) + local.x() * sin_theta +
                              local.y() * cos_theta;
            const double intrusion = outer_radius - esdf_map.getDist(wx, wy);
            if (intrusion > stage_two_intrusion) {
                stage_two_intrusion = intrusion;
                worst_node = k;
                worst_wx = wx;
                worst_wy = wy;
            }
        }
    }
    LOG_FMT_WARN(
        "iLQR stage two collision intrusion: {:.4f} m "
        "(node={}, circle_center=({:.3f}, {:.3f}))",
        stage_two_intrusion, worst_node, worst_wx, worst_wy);
}
}  // namespace apa_post_processor
