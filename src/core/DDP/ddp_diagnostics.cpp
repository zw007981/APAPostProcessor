#include "ddp_diagnostics.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <utility>

#include "../../util/logger.h"
#include "../NMPC/vehicle_circle_geometry.h"

namespace apa_post_processor {
namespace {

const char* DdpPostStageStatusName(DdpPostStageStatus status) {
    switch (status) {
        case DdpPostStageStatus::SUCCESS:
            return "SUCCESS";
        case DdpPostStageStatus::SUCCESS_STAGE_ONE_ONLY:
            return "SUCCESS_STAGE_ONE_ONLY";
        case DdpPostStageStatus::STAGE_ONE_NOT_CONVERGED:
            return "STAGE_ONE_NOT_CONVERGED";
        case DdpPostStageStatus::PIVOT_DETECTED:
            return "PIVOT_DETECTED";
        case DdpPostStageStatus::PRUNED_PATH_DEGENERATE:
            return "PRUNED_PATH_DEGENERATE";
        case DdpPostStageStatus::STAGE_TWO_NOT_CONVERGED:
            return "STAGE_TWO_NOT_CONVERGED";
        case DdpPostStageStatus::VALIDATION_FAILED:
            return "VALIDATION_FAILED";
    }
    return "UNKNOWN";
}
}  // namespace

std::string DdpDiagnostics::BuildFailureMessage(
    const DdpPostStageResult& post) {
    std::string msg =
        std::string("DDP post stage ") + DdpPostStageStatusName(post.status) +
        ": " + post.diagnostics.failed_check +
        " measured=" + std::to_string(post.diagnostics.measured_value) +
        " threshold=" + std::to_string(post.diagnostics.threshold);
    if (!post.diagnostics.degraded_reason.empty()) {
        msg += ", degraded_reason=" + post.diagnostics.degraded_reason;
    }
    msg += ", fell back to original path";
    return msg;
}

double DdpDiagnostics::MeasureIntrusion(
    const DdpAlignedVec<DdpState>& states,
    const VehicleFootprintModel& footprint_model, const ESDFMap& esdf_map) {
    const double outer_radius = footprint_model.getOuterRadius();
    const auto outer_circles =
        vehicle_circle_geometry::ExtractLocalCircleCenters(footprint_model,
                                                           CircleType::OUTER);
    double intrusion = 0.0;
    for (const auto& state : states) {
        const double cos_theta = std::cos(state(DDP_IDX_THETA));
        const double sin_theta = std::sin(state(DDP_IDX_THETA));
        for (const auto& local : outer_circles) {
            const double wx = state(DDP_IDX_X) + local.x() * cos_theta -
                              local.y() * sin_theta;
            const double wy = state(DDP_IDX_Y) + local.x() * sin_theta +
                              local.y() * cos_theta;
            intrusion =
                std::max(intrusion, outer_radius - esdf_map.getDist(wx, wy));
        }
    }
    return intrusion;
}

void DdpDiagnostics::LogWorstAmplitudeViolation(
    const DdpAlignedVec<DdpState>& states, const DdpCostConfig& cost) {
    double worst_violation = 0.0;
    std::size_t worst_index = 0;
    std::string worst_name;
    double worst_state_value = 0.0;
    for (std::size_t k = 0; k < states.size(); ++k) {
        const auto& x = states[k];
        const std::array<std::pair<const char*, double>, 5> checks{{
            {"v", x(DDP_IDX_V) * x(DDP_IDX_V) - cost.v_max * cost.v_max},
            {"a", x(DDP_IDX_A) * x(DDP_IDX_A) - cost.a_max * cost.a_max},
            {"omega", x(DDP_IDX_OMEGA) * x(DDP_IDX_OMEGA) -
                          cost.omega_max * cost.omega_max},
            {"delta+", x(DDP_IDX_DELTA) - cost.delta_max},
            {"delta-", -x(DDP_IDX_DELTA) - cost.delta_max},
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
                ? x(DDP_IDX_V)
                : (worst_name[0] == 'a'
                       ? x(DDP_IDX_A)
                       : (worst_name[0] == 'o' ? x(DDP_IDX_OMEGA)
                                               : x(DDP_IDX_DELTA)));
        LOG_FMT_WARN(
            "DDP stage one worst amplitude: constraint={}, "
            "node={}/{}, violation={:.4f}, state_value={:.4f}, "
            "v={:.4f}, pose=({:.3f}, {:.3f}, {:.3f})",
            worst_name, worst_index, states.size(), worst_violation,
            worst_state_value, x(DDP_IDX_V), x(DDP_IDX_X), x(DDP_IDX_Y),
            x(DDP_IDX_THETA));
    }
}

void DdpDiagnostics::LogStageOneReport(
    const ApaDdpStageOneResult& stage_one, const DdpConfig& config,
    const VehicleFootprintModel& footprint_model, const ESDFMap& esdf_map) {
    LOG_FMT_INFO(
        "DDP stage one: status={}, outer={}, inner_total={}, restarts={}, "
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

    if (stage_one.report.status == ApaDdpStatus::CONVERGED) {
        LOG_FMT_INFO("DDP stage one collision intrusion: {:.4f} m",
                     MeasureIntrusion(stage_one.states, footprint_model,
                                      esdf_map));
    }

    if (stage_one.report.status != ApaDdpStatus::CONVERGED) {
        for (const auto& round : stage_one.report.history) {
            LOG_FMT_WARN(
                "DDP stage one round {}: w_ref={:.3f}, mu={:.1f}, "
                "term_err={:.4f} m/{:.3f} deg, ineq={:.4f}, "
                "defect={:.2e}, inner_status={}, inner_iter={}",
                round.outer_index, round.tracking_weight, round.mu,
                round.terminal_position_error, round.terminal_heading_error_deg,
                round.max_amplitude_violation, round.defect_norm_inf,
                static_cast<int>(round.inner_status), round.inner_iterations);
        }
        LogWorstAmplitudeViolation(stage_one.states, config.solver.cost);
    }
}

void DdpDiagnostics::LogPostStageReport(
    const DdpPostStageResult& post, const ApaDdpStageOneResult& stage_one,
    const ESDFMap& esdf_map) {
    LOG_FMT_INFO(
        "DDP post stage: status={}, maneuvers {}->{}, seams={}",
        static_cast<int>(post.status), post.diagnostics.input_maneuver_count,
        post.diagnostics.output_maneuver_count, post.diagnostics.seams.size());
    LOG_FMT_INFO(
        "DDP domain diagnostics: out_of_map_queries={}, "
        "domain_guard_rejections={}",
        esdf_map.outOfMapQueryCount(),
        stage_one.report.domain_guard_rejections);
}

void DdpDiagnostics::LogFailureDiagnostics(
    const DdpPostStageResult& post,
    const VehicleFootprintModel& footprint_model, const ESDFMap& esdf_map) {
    LOG_FMT_WARN("{}", BuildFailureMessage(post));

    if (post.stage_two.has_value()) {
        const auto& report = post.stage_two->report;
        LOG_FMT_WARN(
            "DDP stage two diagnostics: status={}, outer={}, "
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
        if (report.status != ApaDdpStatus::CONVERGED) {
            for (const auto& round : report.history) {
                LOG_FMT_WARN(
                    "DDP stage two round {}: w_ref={:.3f}, mu={:.1f}, "
                    "term_err={:.4f} m/{:.3f} deg, ineq={:.4f}, "
                    "defect={:.2e}, inner_status={}, inner_iter={}",
                    round.outer_index, round.tracking_weight, round.mu,
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
            "DDP seam report: index={}, delta_delta={:.3f}, "
            "t_resteer={:.2f}, t_dwell={:.2f}, dwell={:.2f}, "
            "seam_speed={:.4f}, window_max_speed={:.4f}, "
            "window_end_omega={:.4f}",
            seam.seam_index, seam.delta_delta, seam.t_resteer, seam.t_dwell,
            seam.dwell_duration, seam.seam_speed, seam.window_max_speed,
            seam.window_end_omega);
    }
}

void DdpDiagnostics::LogStageTwoIntrusion(
    const DdpPostStageResult& post,
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
        const double cos_theta = std::cos(state(DDP_IDX_THETA));
        const double sin_theta = std::sin(state(DDP_IDX_THETA));
        for (const auto& local : outer_circles) {
            const double wx = state(DDP_IDX_X) + local.x() * cos_theta -
                              local.y() * sin_theta;
            const double wy = state(DDP_IDX_Y) + local.x() * sin_theta +
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
        "DDP stage two collision intrusion: {:.4f} m "
        "(node={}, circle_center=({:.3f}, {:.3f}))",
        stage_two_intrusion, worst_node, worst_wx, worst_wy);
}
}  // namespace apa_post_processor
