#include "nmpc_config.h"

#include "../../util/data_loader.hpp"
#include "../../util/logger.h"

namespace apa_post_processor {
// 顶层字段：proto 字段名与 C++ 成员名一致，仅需两个参数
#define LOAD_(proto, field)             \
    if ((proto).has_##field()) {        \
        field = (proto).field();        \
    }
// 子配置字段：proto 字段名与 C++ 成员路径不同，需要三个参数
#define LOAD_SUB_(proto, proto_field, member) \
    if ((proto).has_##proto_field()) {        \
        member = (proto).proto_field();       \
    }

void NMPCConfig::loadFromProto(
    const ::apa::post_processor::NMPCConfigProto& proto) {
    LOAD_(proto, max_collision_depth);
    LOAD_(proto, terminal_position_error_threshold);
    LOAD_(proto, terminal_heading_error_threshold_deg);
    LOAD_(proto, control_effort_accel_weight);
    LOAD_(proto, control_effort_steer_rate_weight);
    LOAD_(proto, smoothing_jerk_weight);
    LOAD_(proto, smoothing_steer_accel_weight);
    LOAD_(proto, interior_speed_weight);
    LOAD_(proto, interior_steer_weight);
    LOAD_(proto, global_target_position_weight);
    LOAD_(proto, global_target_heading_weight);
    LOAD_(proto, terminal_position_weight);
    LOAD_(proto, terminal_heading_weight);
    LOAD_(proto, terminal_speed_weight);
    LOAD_(proto, terminal_steer_weight);
    LOAD_(proto, position_tracking_weight);
    LOAD_(proto, max_position_deviation_from_ref);
    LOAD_(proto, theta_tracking_weight);
    LOAD_(proto, max_theta_deviation_from_ref);
    LOAD_(proto, esdf_safety_margin);
    LOAD_(proto, esdf_penalty_weight);
    LOAD_(proto, max_iter);
    LOAD_(proto, dt);
    LOAD_(proto, enable_debug_output);
    LOAD_(proto, corridor_hard_margin);
    LOAD_(proto, corridor_soft_quadratic_weight);
    LOAD_(proto, corridor_soft_linear_weight);
    LOAD_(proto, hpipm_block_size);
    LOAD_(proto, short_n_threshold);
    LOAD_(proto, use_line_search);
    LOAD_(proto, sqp_hessian_regularization);
    LOAD_(proto, hpipm_tol);
    LOAD_(proto, skip_last_step_corridor);
    LOAD_(proto, use_static_corridor_soft_constraint);
    LOAD_(proto, static_corridor_soft_weight);
    LOAD_(proto, use_static_corridor);
    LOAD_(proto, collision_safety_margin);
    // 子配置
    if (proto.has_bspline()) {
        const auto& b = proto.bspline();
        LOAD_SUB_(b, dense_step_dist, bspline.dense_step_dist);
        LOAD_SUB_(b, control_point_spacing, bspline.control_point_spacing);
        LOAD_SUB_(b, weight_data, bspline.weight_data);
        LOAD_SUB_(b, weight_smooth_d1, bspline.weight_smooth_d1);
        LOAD_SUB_(b, weight_smooth_d2, bspline.weight_smooth_d2);
        LOAD_SUB_(b, weight_smooth_d3, bspline.weight_smooth_d3);
        LOAD_SUB_(b, weight_collision, bspline.weight_collision);
        LOAD_SUB_(b, weight_kappa, bspline.weight_kappa);
        LOAD_SUB_(b, max_kappa, bspline.max_kappa);
        LOAD_SUB_(b, weight_reg, bspline.weight_reg);
        LOAD_SUB_(b, collision_margin, bspline.collision_margin);
        LOAD_SUB_(b, anchor_extension_length, bspline.anchor_extension_length);
        LOAD_SUB_(b, min_segment_arc_length_for_degradation, bspline.min_segment_arc_length_for_degradation);
        LOAD_SUB_(b, collision_validation_tolerance, bspline.collision_validation_tolerance);
        LOAD_SUB_(b, lbfgs_max_iterations, bspline.lbfgs_max_iterations);
        LOAD_SUB_(b, lbfgs_epsilon, bspline.lbfgs_epsilon);
        LOAD_SUB_(b, lbfgs_epsilon_rel, bspline.lbfgs_epsilon_rel);
        LOAD_SUB_(b, lbfgs_m, bspline.lbfgs_m);
        LOAD_SUB_(b, lbfgs_max_linesearch, bspline.lbfgs_max_linesearch);
        LOAD_SUB_(b, lbfgs_linesearch_algo, bspline.lbfgs_linesearch_algo);
        LOAD_SUB_(b, lbfgs_ftol, bspline.lbfgs_ftol);
        LOAD_SUB_(b, lbfgs_wolfe, bspline.lbfgs_wolfe);
        LOAD_SUB_(b, arc_length_table_step, bspline.arc_length_table_step);
    }
    if (proto.has_speed()) {
        const auto& s = proto.speed();
        LOAD_SUB_(s, max_v_forward, speed.max_v_forward);
        LOAD_SUB_(s, max_v_reverse, speed.max_v_reverse);
        LOAD_SUB_(s, max_jerk_proxy, speed.max_jerk_proxy);
        LOAD_SUB_(s, weight_v_ref, speed.weight_v_ref);
        LOAD_SUB_(s, weight_a_sq, speed.weight_a_sq);
        LOAD_SUB_(s, weight_jerk_sq, speed.weight_jerk_sq);
        LOAD_SUB_(s, time_reintegration_epsilon, speed.time_reintegration_epsilon);
        LOAD_SUB_(s, max_lateral_accel, speed.max_lateral_accel);
        LOAD_SUB_(s, esdf_danger_margin, speed.esdf_danger_margin);
    }
    if (proto.has_diff_flat()) {
        const auto& df = proto.diff_flat();
        LOAD_SUB_(df, curvature_denominator_epsilon, diff_flat.curvature_denominator_epsilon);
    }
    if (proto.has_resampler()) {
        const auto& r = proto.resampler();
        LOAD_SUB_(r, n_min_active_per_segment, resampler.n_min_active_per_segment);
        LOAD_SUB_(r, nominal_step_s, resampler.nominal_step_s);
        LOAD_SUB_(r, density_w_base, resampler.density_w_base);
        LOAD_SUB_(r, density_w_kappa, resampler.density_w_kappa);
        LOAD_SUB_(r, density_w_obs, resampler.density_w_obs);
        LOAD_SUB_(r, dense_step_dist, resampler.dense_step_dist);
        LOAD_SUB_(r, steer_padding_epsilon, resampler.steer_padding_epsilon);
        LOAD_SUB_(r, steer_safe_rate_ratio, resampler.steer_safe_rate_ratio);
        LOAD_SUB_(r, min_segment_arc_length_for_degradation, resampler.min_segment_arc_length_for_degradation);
        LOAD_SUB_(r, time_reintegration_epsilon, resampler.time_reintegration_epsilon);
        LOAD_SUB_(r, obstacle_density_margin, resampler.obstacle_density_margin);
    }
    if (proto.has_corridor()) {
        const auto& c = proto.corridor();
        LOAD_SUB_(c, soft_margin, corridor.soft_margin);
    }
}
#undef LOAD_SUB_
#undef LOAD_
}  // namespace apa_post_processor
