#include "path_to_ocp_converter.h"

#include <costs/quadratic_tracking.h>
#include <models/bicycle_model_delta.h>
#include <util/constants.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

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
    const auto profiles = computeSegmentProfiles(path);
    Result result;
    result.init_guess = generateInitialGuess(path, profiles);
    result.ocp = buildOcp(path, profiles, result.init_guess);
    return result;
}

std::vector<SegmentProfile> PathToOcpConverter::computeSegmentProfiles(
    const Path& path) const {
    const auto& maneuvers = path.getManeuvers();
    std::vector<SegmentProfile> profiles;
    profiles.reserve(maneuvers.size());
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
        const double arc_length = maneuver.length();
        const auto velocity_profile = buildVelocityProfile(arc_length);
        SegmentProfile profile;
        profile.N = velocity_profile.step_num;
        profile.dt = velocity_profile.tf / velocity_profile.step_num;
        profile.v_sign =
            (maneuver.direction == Direction::FORWARD) ? 1.0 : -1.0;
        profile.is_terminal = (m + 1 == maneuvers.size());
        profiles.push_back(std::move(profile));
    }
    return profiles;
}

stc_SQP::Trajectory PathToOcpConverter::generateInitialGuess(
    const Path& path, const std::vector<SegmentProfile>& profiles) const {
    const auto& maneuvers = path.getManeuvers();
    if (maneuvers.size() != profiles.size()) {
        throw std::invalid_argument(
            "PathToOcpConverter::generateInitialGuess: profiles count (" +
            std::to_string(profiles.size()) + ") must match maneuver count (" +
            std::to_string(maneuvers.size()) + ")!!!");
    }
    stc_SQP::Trajectory init_guess;
    for (std::size_t m = 0; m < maneuvers.size(); ++m) {
        const auto& maneuver = maneuvers[m];
        const auto& profile = profiles[m];
        if (!profile.dt_array.empty()) {
            throw std::invalid_argument(
                "PathToOcpConverter::generateInitialGuess: non-uniform "
                "dt_array is not supported in the default cubic velocity "
                "profile generator; use a custom initial guess or leave "
                "dt_array empty!!!");
        }
        const auto velocity_profile = buildVelocityProfile(maneuver.length());
        const auto samples =
            sampleManeuver(maneuver, profile, velocity_profile);
        if (m == 0) {
            init_guess.x.push_back(samples.states.front());
        }
        for (std::size_t i = 1; i < samples.states.size(); ++i) {
            init_guess.x.push_back(samples.states[i]);
        }
        for (const auto& u : samples.controls) {
            init_guess.u.push_back(u);
        }
    }
    return init_guess;
}

stc_SQP::MultiStageOCP PathToOcpConverter::buildOcp(
    const Path& path, const std::vector<SegmentProfile>& profiles,
    const stc_SQP::Trajectory& ref_trajectory) const {
    const auto& maneuvers = path.getManeuvers();
    if (maneuvers.size() != profiles.size()) {
        throw std::invalid_argument(
            "PathToOcpConverter::buildOcp: profiles count (" +
            std::to_string(profiles.size()) + ") must match maneuver count (" +
            std::to_string(maneuvers.size()) + ")!!!");
    }
    // 校验 ref_trajectory 总维度与 profiles 一致
    int total_steps = 0;
    for (const auto& profile : profiles) {
        total_steps += profile.N;
    }
    if (static_cast<int>(ref_trajectory.x.size()) != total_steps + 1) {
        throw std::invalid_argument(
            "PathToOcpConverter::buildOcp: ref_trajectory.x size (" +
            std::to_string(ref_trajectory.x.size()) +
            ") must equal total steps + 1 (" + std::to_string(total_steps + 1) +
            ")!!!");
    }
    if (static_cast<int>(ref_trajectory.u.size()) != total_steps) {
        throw std::invalid_argument(
            "PathToOcpConverter::buildOcp: ref_trajectory.u size (" +
            std::to_string(ref_trajectory.u.size()) +
            ") must equal total steps (" + std::to_string(total_steps) +
            ")!!!");
    }
    stc_SQP::MultiStageOCP ocp;
    auto dynamics =
        std::make_shared<stc_SQP::BicycleModelDelta>(vehicle_params_.wheelbase);
    // 全程目标牵引代价：目标位姿是整条路径的终点
    const stc_SQP::Vector global_target_x_ref = ref_trajectory.x.back();
    int global_x_offset = 0;
    for (std::size_t m = 0; m < maneuvers.size(); ++m) {
        const auto& maneuver = maneuvers[m];
        const auto& profile = profiles[m];
        // 提取该段在 ref_trajectory 中的 theta_ref/x_ref/y_ref
        std::vector<double> theta_refs;
        std::vector<double> x_refs;
        std::vector<double> y_refs;
        theta_refs.reserve(static_cast<std::size_t>(profile.N));
        x_refs.reserve(static_cast<std::size_t>(profile.N));
        y_refs.reserve(static_cast<std::size_t>(profile.N));
        for (int i = 0; i < profile.N; ++i) {
            const auto& state =
                ref_trajectory.x[static_cast<std::size_t>(global_x_offset + i)];
            theta_refs.push_back(state(2));
            x_refs.push_back(state(0));
            y_refs.push_back(state(1));
        }
        // 终端x_ref：非终端段传零向量（内部权重不跟踪位置），终端段传该段末端参考状态
        stc_SQP::Vector terminal_x_ref = stc_SQP::Vector::Zero(5);
        if (profile.is_terminal) {
            terminal_x_ref =
                ref_trajectory
                    .x[static_cast<std::size_t>(global_x_offset + profile.N)];
        }
        ocp.addSegment(buildSegment(maneuver, dynamics, profile, terminal_x_ref,
                                    global_target_x_ref, theta_refs, x_refs,
                                    y_refs));
        global_x_offset += profile.N;
    }
    return ocp;
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
    const Maneuver& maneuver, const SegmentProfile& profile,
    const VelocityProfile& velocity_profile) const {
    const double v_sign = profile.v_sign;
    const double arc_length = maneuver.length();
    const auto cumulative = buildCumulativeArcLength(maneuver.points);
    const double dt = profile.dt;
    const int step_num = profile.N;
    // b/c 需与 buildVelocityProfile 保持同步
    const double b = velocity_profile.b;
    const double c = velocity_profile.c;
    // 末尾校验弧长一致性
    const double s_at_tf =
        b * velocity_profile.tf * velocity_profile.tf * velocity_profile.tf +
        c * velocity_profile.tf * velocity_profile.tf;
    if (std::abs(s_at_tf - arc_length) > 1e-9) {
        throw std::invalid_argument(
            "PathToOcpConverter::sampleManeuver: velocity_profile coefficients "
            "are inconsistent with arc_length (s(tf)=" +
            std::to_string(s_at_tf) +
            ", arc_length=" + std::to_string(arc_length) + ")!!!");
    }

    ManeuverSamples samples;
    samples.states.reserve(step_num + 1);
    samples.controls.reserve(step_num);
    std::vector<double> v_signed(step_num + 1);
    std::vector<double> delta_values(step_num + 1);
    for (int i = 0; i <= step_num; ++i) {
        const double t = i * dt;
        const double s = std::clamp(b * t * t * t + c * t * t, 0.0, arc_length);
        const double v_mag = std::max(0.0, 3.0 * b * t * t + 2.0 * c * t);
        const TrajectoryPoint pose =
            interpolateAtArcLength(maneuver.points, cumulative, s);
        const double kappa = pose.hasKappa() ? pose.getKappa() : 0.0;
        const double raw_delta = std::atan(kappa * vehicle_params_.wheelbase);
        const double delta =
            std::clamp(raw_delta, -vehicle_params_.max_steer_angle,
                       vehicle_params_.max_steer_angle);
        stc_SQP::Vector x(5);
        x << pose.x, pose.y, pose.theta, v_sign * v_mag, delta;
        samples.states.push_back(std::move(x));
        v_signed[i] = v_sign * v_mag;
        delta_values[i] = delta;
    }
    for (int i = 0; i < step_num; ++i) {
        stc_SQP::Vector u(2);
        u << (v_signed[i + 1] - v_signed[i]) / dt,
            (delta_values[i + 1] - delta_values[i]) / dt;
        samples.controls.push_back(std::move(u));
    }
    return samples;
}

stc_SQP::StageSegment PathToOcpConverter::buildSegment(
    const Maneuver& maneuver,
    const std::shared_ptr<stc_SQP::DynamicalSystem>& dynamics,
    const SegmentProfile& profile, const stc_SQP::Vector& terminal_x_ref,
    const stc_SQP::Vector& global_target_x_ref,
    const std::vector<double>& theta_refs, const std::vector<double>& x_refs,
    const std::vector<double>& y_refs) const {
    const double v_sign = profile.v_sign;
    if (std::abs(v_sign - 1.0) > 1e-12 && std::abs(v_sign + 1.0) > 1e-12) {
        throw std::invalid_argument(
            "PathToOcpConverter::buildSegment: v_sign must be +1.0 or -1.0, "
            "got " +
            std::to_string(v_sign) + "!!!");
    }
    constexpr int kNx = 5, kNu = 2;
    if (static_cast<int>(theta_refs.size()) != profile.N) {
        throw std::invalid_argument(
            "PathToOcpConverter::buildSegment: theta_refs size (" +
            std::to_string(theta_refs.size()) + ") must match profile.N (" +
            std::to_string(profile.N) + ")!!!");
    }
    if (static_cast<int>(x_refs.size()) != profile.N ||
        static_cast<int>(y_refs.size()) != profile.N) {
        throw std::invalid_argument(
            "PathToOcpConverter::buildSegment: x_refs/y_refs size must match "
            "profile.N (" +
            std::to_string(profile.N) + ")!!!");
    }
    stc_SQP::StageSegment segment;
    segment.dynamics = dynamics;
    segment.N = profile.N;
    segment.dt = profile.dt;
    segment.dt_array = profile.dt_array;
    segment.v_sign = v_sign;
    segment.x_min = stc_SQP::Vector::Constant(kNx, -config_.pose_bound);
    segment.x_max = stc_SQP::Vector::Constant(kNx, config_.pose_bound);
    segment.x_min(3) =
        (v_sign > 0.0) ? -config_.boundary_velocity_slack : -config_.max_speed;
    segment.x_max(3) =
        (v_sign > 0.0) ? config_.max_speed : config_.boundary_velocity_slack;
    segment.x_min(4) = -vehicle_params_.max_steer_angle;
    segment.x_max(4) = vehicle_params_.max_steer_angle;
    segment.u_min = (stc_SQP::Vector(kNu) << -config_.accel_limit,
                     -config_.steer_rate_limit)
                        .finished();
    segment.u_max =
        (stc_SQP::Vector(kNu) << config_.accel_limit, config_.steer_rate_limit)
            .finished();
    // 终端段额外在 x/y/theta 施加跟踪权重
    stc_SQP::Matrix Q = stc_SQP::Matrix::Zero(kNx, kNx);
    stc_SQP::Vector x_ref = stc_SQP::Vector::Zero(kNx);
    x_ref(0) = global_target_x_ref(0);
    x_ref(1) = global_target_x_ref(1);
    x_ref(2) = global_target_x_ref(2);
    Q(0, 0) = config_.global_target_position_weight;
    Q(1, 1) = config_.global_target_position_weight;
    Q(2, 2) = config_.global_target_heading_weight;
    if (profile.is_terminal) {
        // 终端段在全程目标牵引之上叠加更强的终端跟踪权重（两者指向同一个目标）。
        Q(0, 0) += config_.terminal_position_weight;
        Q(1, 1) += config_.terminal_position_weight;
        Q(2, 2) += config_.terminal_heading_weight;
        Q(3, 3) = config_.terminal_speed_weight;
        Q(4, 4) = config_.terminal_steer_weight;
        x_ref(3) = terminal_x_ref(3);
        x_ref(4) = terminal_x_ref(4);
    } else {
        Q(3, 3) = config_.interior_speed_weight;
        Q(4, 4) = config_.interior_steer_weight;
    }
    stc_SQP::Matrix R = stc_SQP::Matrix::Zero(kNu, kNu);
    R(0, 0) = config_.control_effort_accel_weight;
    R(1, 1) = config_.control_effort_steer_rate_weight;
    segment.cost = std::make_shared<stc_SQP::QuadraticTrackingCost>(
        x_ref, Q, R, /*theta_idx=*/2);
    // 填充 stage_params：每步存储局部步索引 p(0)、参考航向 p(1)、参考位置 p(3)/p(4)
    segment.stage_params.resize(static_cast<std::size_t>(segment.N));
    for (int i = 0; i < segment.N; ++i) {
        stc_SQP::StageParameters sp;
        sp.p = stc_SQP::Vector::Zero(stc_SQP::STAGE_PARAM_DIM);
        sp.p(0) = static_cast<double>(i);
        // p(1): theta_ref, p(3)/p(4): x_ref/y_ref
        sp.p(1) = theta_refs[static_cast<std::size_t>(i)];
        sp.p(3) = x_refs[static_cast<std::size_t>(i)];
        sp.p(4) = y_refs[static_cast<std::size_t>(i)];
        segment.stage_params[static_cast<std::size_t>(i)] = std::move(sp);
    }
    return segment;
}

std::vector<double> PathToOcpConverter::buildCumulativeArcLength(
    const std::vector<TrajectoryPoint>& points) {
    std::vector<double> cumulative(points.size(), 0.0);
    for (std::size_t i = 1; i < points.size(); ++i) {
        cumulative[i] =
            cumulative[i - 1] + std::hypot(points[i].x - points[i - 1].x,
                                           points[i].y - points[i - 1].y);
    }
    return cumulative;
}

TrajectoryPoint PathToOcpConverter::interpolateAtArcLength(
    const std::vector<TrajectoryPoint>& points,
    const std::vector<double>& cumulative, double s) {
    const double s_clamped =
        std::clamp(s, cumulative.front(), cumulative.back());
    const auto it =
        std::lower_bound(cumulative.begin(), cumulative.end(), s_clamped);
    auto upper_idx =
        static_cast<std::size_t>(std::distance(cumulative.begin(), it));
    upper_idx = std::clamp<std::size_t>(upper_idx, 1, points.size() - 1);
    const std::size_t lower_idx = upper_idx - 1;
    const double seg_len = cumulative[upper_idx] - cumulative[lower_idx];
    const double ratio = (seg_len > EPSILON_PRECISE)
                             ? (s_clamped - cumulative[lower_idx]) / seg_len
                             : 0.0;
    const TrajectoryPoint &p0 = points[lower_idx], &p1 = points[upper_idx];
    const double x = p0.x + (p1.x - p0.x) * ratio;
    const double y = p0.y + (p1.y - p0.y) * ratio;
    const double theta =
        p0.theta + std::remainder(p1.theta - p0.theta, 2.0 * PI) * ratio;
    TrajectoryPoint result(x, y, theta);
    const double kappa0 = p0.hasKappa() ? p0.getKappa() : 0.0;
    const double kappa1 = p1.hasKappa() ? p1.getKappa() : 0.0;
    result.setKappa(kappa0 + (kappa1 - kappa0) * ratio);
    return result;
}
}  // namespace apa_post_processor
