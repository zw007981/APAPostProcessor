#include "path_to_ocp_converter.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

#include <costs/quadratic_tracking.h>
#include <models/bicycle_model_delta.h>

#include "../../util/constants.h"

namespace apa_post_processor {
PathToOcpConverter::PathToOcpConverter(const VehicleParams& vehicle_params,
                                       const PathToOcpConfig& config)
    : vehicle_params_(vehicle_params), config_(config) {
    if (vehicle_params_.wheelbase <= 0.0 ||
        vehicle_params_.max_steer_angle <= 0.0) {
        throw std::invalid_argument(
            "PathToOcpConverter: vehicle_params must have positive wheelbase "
            "and max_steer_angle!!!");
    }
    if (config_.dt <= 0.0 || config_.target_peak_speed <= 0.0 ||
        config_.max_speed <= 0.0 || config_.accel_limit <= 0.0 ||
        config_.steer_rate_limit <= 0.0 || config_.pose_bound <= 0.0 ||
        config_.boundary_velocity_slack < 0.0) {
        throw std::invalid_argument(
            "PathToOcpConverter: config contains non-positive or negative "
            "field that must be positive/non-negative!!!");
    }
}

PathToOcpConverter::Result PathToOcpConverter::convert(const Path& path) const {
    if (path.empty()) {
        throw std::invalid_argument("PathToOcpConverter: path is empty!!!");
    }
    const auto& maneuvers = path.getManeuvers();
    Result result;
    auto dynamics =
        std::make_shared<stc_SQP::BicycleModelDelta>(vehicle_params_.wheelbase);
    for (std::size_t m = 0; m < maneuvers.size(); ++m) {
        const auto& maneuver = maneuvers[m];
        if (maneuver.direction != Direction::FORWARD &&
            maneuver.direction != Direction::BACKWARD) {
            throw std::invalid_argument(
                "PathToOcpConverter: only FORWARD/BACKWARD maneuvers are "
                "currently supported, PIVOT/UNKNOWN not handled yet!!!");
        }
        if (maneuver.points.size() < 2) {
            throw std::invalid_argument(
                "PathToOcpConverter: maneuver must contain at least 2 "
                "points!!!");
        }
        const auto samples = sampleManeuver(maneuver);
        if (m == 0) {
            result.init_guess.x.push_back(samples.states.front());
        }
        for (std::size_t i = 1; i < samples.states.size(); ++i) {
            result.init_guess.x.push_back(samples.states[i]);
        }
        for (const auto& u : samples.controls) {
            result.init_guess.u.push_back(u);
        }
        const bool is_terminal_segment = (m + 1 == maneuvers.size());
        result.ocp.addSegment(
            buildSegment(maneuver, dynamics, samples, is_terminal_segment));
    }
    return result;
}

PathToOcpConverter::VelocityProfile PathToOcpConverter::buildVelocityProfile(
    double arc_length) const {
    const double tf_ideal = 1.5 * arc_length / config_.target_peak_speed;
    const int step_num =
        std::max(1, static_cast<int>(std::llround(tf_ideal / config_.dt)));
    const double tf = step_num * config_.dt;
    const double tf_cubed = tf * tf * tf, tf_squared = tf * tf;
    VelocityProfile profile;
    profile.step_num = step_num;
    profile.tf = tf;
    profile.b = -2.0 * arc_length / tf_cubed;
    profile.c = 3.0 * arc_length / tf_squared;
    return profile;
}

PathToOcpConverter::ManeuverSamples PathToOcpConverter::sampleManeuver(
    const Maneuver& maneuver) const {
    const double v_sign = (maneuver.direction == Direction::FORWARD) ? 1.0 : -1.0;
    const double arc_length = maneuver.length();
    const auto profile = buildVelocityProfile(arc_length);
    const auto cumulative = buildCumulativeArcLength(maneuver.points);

    ManeuverSamples samples;
    samples.states.reserve(profile.step_num + 1);
    samples.controls.reserve(profile.step_num);
    std::vector<double> v_signed(profile.step_num + 1);
    std::vector<double> delta_values(profile.step_num + 1);
    for (int i = 0; i <= profile.step_num; ++i) {
        const double t = i * config_.dt;
        const double s = std::clamp(
            profile.b * t * t * t + profile.c * t * t, 0.0, arc_length);
        const double v_mag =
            std::max(0.0, 3.0 * profile.b * t * t + 2.0 * profile.c * t);
        const PathPoint pose =
            interpolateAtArcLength(maneuver.points, cumulative, s);
        const double kappa =
            pose.hasKappa() ? pose.getKappa() : 0.0;
        const double raw_delta = std::atan(kappa * vehicle_params_.wheelbase);
        const double delta = std::clamp(raw_delta, -vehicle_params_.max_steer_angle,
                                        vehicle_params_.max_steer_angle);
        stc_SQP::Vector x(5);
        x << pose.x, pose.y, pose.theta, v_sign * v_mag, delta;
        samples.states.push_back(std::move(x));
        v_signed[i] = v_sign * v_mag;
        delta_values[i] = delta;
    }
    for (int i = 0; i < profile.step_num; ++i) {
        stc_SQP::Vector u(2);
        u << (v_signed[i + 1] - v_signed[i]) / config_.dt,
            (delta_values[i + 1] - delta_values[i]) / config_.dt;
        samples.controls.push_back(std::move(u));
    }
    return samples;
}

stc_SQP::StageSegment PathToOcpConverter::buildSegment(
    const Maneuver& maneuver,
    const std::shared_ptr<stc_SQP::DynamicalSystem>& dynamics,
    const ManeuverSamples& samples, bool is_terminal_segment) const {
    const double v_sign = (maneuver.direction == Direction::FORWARD) ? 1.0 : -1.0;
    constexpr int kNx = 5, kNu = 2;
    stc_SQP::StageSegment segment;
    segment.dynamics = dynamics;
    segment.N = static_cast<int>(samples.controls.size());
    segment.dt = config_.dt;
    segment.v_sign = v_sign;
    segment.x_min = stc_SQP::Vector::Constant(kNx, -config_.pose_bound);
    segment.x_max = stc_SQP::Vector::Constant(kNx, config_.pose_bound);
    segment.x_min(3) =
        (v_sign > 0.0) ? -config_.boundary_velocity_slack : -config_.max_speed;
    segment.x_max(3) =
        (v_sign > 0.0) ? config_.max_speed : config_.boundary_velocity_slack;
    segment.x_min(4) = -vehicle_params_.max_steer_angle;
    segment.x_max(4) = vehicle_params_.max_steer_angle;
    segment.u_min =
        (stc_SQP::Vector(kNu) << -config_.accel_limit, -config_.steer_rate_limit)
            .finished();
    segment.u_max =
        (stc_SQP::Vector(kNu) << config_.accel_limit, config_.steer_rate_limit)
            .finished();
    // 状态代价Q：内部机动段仅在v/delta两维施加小权重（v^2是路径长度代价sum|v|dt在
    // v=0处的光滑二次近似，delta^2抑制不必要的大转向），x/y/theta保持自由；
    // 终端机动段额外在x/y/theta/v/delta施加跟踪权重，近似终端位姿硬约束
    // （受限于CostTerm接口尚未区分stage/terminal，该权重会施加到最后一段的每一步，
    // 这是当前框架下的已知近似，详见类注释）。
    stc_SQP::Matrix Q = stc_SQP::Matrix::Zero(kNx, kNx);
    stc_SQP::Vector x_ref = stc_SQP::Vector::Zero(kNx);
    if (is_terminal_segment) {
        Q(0, 0) = config_.terminal_position_weight;
        Q(1, 1) = config_.terminal_position_weight;
        Q(2, 2) = config_.terminal_heading_weight;
        Q(3, 3) = config_.terminal_speed_weight;
        Q(4, 4) = config_.terminal_steer_weight;
        x_ref = samples.states.back();
    } else {
        Q(3, 3) = config_.interior_speed_weight;
        Q(4, 4) = config_.interior_steer_weight;
    }
    stc_SQP::Matrix R = stc_SQP::Matrix::Zero(kNu, kNu);
    R(0, 0) = config_.control_effort_accel_weight;
    R(1, 1) = config_.control_effort_steer_rate_weight;
    segment.cost = std::make_shared<stc_SQP::QuadraticTrackingCost>(
        x_ref, Q, R, /*theta_idx=*/2);
    return segment;
}

std::vector<double> PathToOcpConverter::buildCumulativeArcLength(
    const std::vector<PathPoint>& points) {
    std::vector<double> cumulative(points.size(), 0.0);
    for (std::size_t i = 1; i < points.size(); ++i) {
        cumulative[i] = cumulative[i - 1] +
                       std::hypot(points[i].x - points[i - 1].x,
                                  points[i].y - points[i - 1].y);
    }
    return cumulative;
}

PathPoint PathToOcpConverter::interpolateAtArcLength(
    const std::vector<PathPoint>& points, const std::vector<double>& cumulative,
    double s) {
    const double s_clamped = std::clamp(s, cumulative.front(), cumulative.back());
    const auto it =
        std::lower_bound(cumulative.begin(), cumulative.end(), s_clamped);
    auto upper_idx = static_cast<std::size_t>(std::distance(cumulative.begin(), it));
    upper_idx = std::clamp<std::size_t>(upper_idx, 1, points.size() - 1);
    const std::size_t lower_idx = upper_idx - 1;
    const double seg_len = cumulative[upper_idx] - cumulative[lower_idx];
    const double ratio = (seg_len > EPSILON_PRECISE)
                             ? (s_clamped - cumulative[lower_idx]) / seg_len
                             : 0.0;
    const PathPoint &p0 = points[lower_idx], &p1 = points[upper_idx];
    const double x = p0.x + (p1.x - p0.x) * ratio;
    const double y = p0.y + (p1.y - p0.y) * ratio;
    const double theta =
        p0.theta + std::remainder(p1.theta - p0.theta, 2.0 * PI) * ratio;
    PathPoint result(x, y, theta);
    const double kappa0 = p0.hasKappa() ? p0.getKappa() : 0.0;
    const double kappa1 = p1.hasKappa() ? p1.getKappa() : 0.0;
    result.setKappa(kappa0 + (kappa1 - kappa0) * ratio);
    return result;
}
}  // namespace apa_post_processor
