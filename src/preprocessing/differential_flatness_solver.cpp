#include "differential_flatness_solver.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

#include "../util/constants.h"

namespace apa_post_processor {

DifferentialFlatnessSolver::DifferentialFlatnessSolver(
    const DifferentialFlatnessSolverConfig& config)
    : config_(config) {
    if (config_.curvature_denominator_epsilon <= 0.0) {
        throw std::invalid_argument(
            "DifferentialFlatnessSolverConfig::curvature_denominator_epsilon "
            "must be positive");
    }
}

void DifferentialFlatnessSolver::validateInputs(
    const DifferentialFlatnessInput& input,
    const VehicleParams& vehicle_params) const {
    const auto n_points = input.x.size();
    if (n_points < 2) {
        throw std::invalid_argument(
            "DifferentialFlatnessSolver requires at least 2 points");
    }
    if (input.y.size() != n_points || input.theta.size() != n_points ||
        input.x_d1.size() != n_points || input.x_d2.size() != n_points ||
        input.x_d3.size() != n_points || input.y_d1.size() != n_points ||
        input.y_d2.size() != n_points || input.y_d3.size() != n_points ||
        input.v.size() != n_points || input.a.size() != n_points ||
        input.t.size() != n_points) {
        throw std::invalid_argument(
            "DifferentialFlatnessSolver input vectors must have the same size");
    }
    if (vehicle_params.wheelbase <= 0.0) {
        throw std::invalid_argument(
            "DifferentialFlatnessSolver requires positive wheelbase");
    }
    for (std::size_t i = 0; i < n_points; ++i) {
        if (!std::isfinite(input.x[i]) || !std::isfinite(input.y[i]) ||
            !std::isfinite(input.theta[i]) || !std::isfinite(input.x_d1[i]) ||
            !std::isfinite(input.x_d2[i]) || !std::isfinite(input.x_d3[i]) ||
            !std::isfinite(input.y_d1[i]) || !std::isfinite(input.y_d2[i]) ||
            !std::isfinite(input.y_d3[i]) || !std::isfinite(input.v[i]) ||
            !std::isfinite(input.a[i]) || !std::isfinite(input.t[i])) {
            throw std::invalid_argument(
                "DifferentialFlatnessSolver input contains non-finite value "
                "at index " +
                std::to_string(i));
        }
    }
    for (std::size_t i = 1; i < n_points; ++i) {
        if (input.t[i] < input.t[i - 1] - EPSILON) {
            throw std::invalid_argument(
                "DifferentialFlatnessSolver input time stamps must be "
                "non-decreasing");
        }
    }
}

std::pair<double, double> DifferentialFlatnessSolver::computeSinglePoint(
    double x_d1, double x_d2, double x_d3, double y_d1, double y_d2,
    double y_d3, double v, double wheelbase,
    double denominator_epsilon) const {
    // 参数速度模方 x'^2 + y'^2，是后续所有曲率/曲率导数公式的公共分母。
    const double speed_sq = x_d1 * x_d1 + y_d1 * y_d1;
    // 对 (x'^2 + y'^2)^3 做死区保护，避免极低速或驻点处除零。
    const double denom_cubed =
        std::max(speed_sq * speed_sq * speed_sq, denominator_epsilon);
    // 有向曲率分子：x'y'' - y'x''
    const double curvature_numerator = x_d1 * y_d2 - y_d1 * x_d2;
    // 有向曲率 kappa = (x'y'' - y'x'') / (x'^2 + y'^2)^{3/2}
    // 分母用 sqrt(speed_sq^3) 并同样施加死区保护。
    const double speed_sq_sqrt = std::sqrt(speed_sq);
    const double denom_for_kappa =
        std::max(speed_sq * speed_sq_sqrt, denominator_epsilon);
    const double kappa = curvature_numerator / denom_for_kappa;
    // 前轮偏角：delta = atan(kappa * L)
    const double delta = std::atan(kappa * wheelbase);
    // 曲率弧长导数 dkappa/ds 的分子：
    // (x'y''' - y'x''')(x'^2 + y'^2) - 3(x'y'' - y'x'')(x'x'' + y'y'')
    const double cross3 = x_d1 * y_d3 - y_d1 * x_d3;
    const double dot12 = x_d1 * x_d2 + y_d1 * y_d2;
    const double dkappa_ds_numerator =
        cross3 * speed_sq - 3.0 * curvature_numerator * dot12;
    const double dkappa_ds = dkappa_ds_numerator / denom_cubed;
    // 前轮偏角变化率：d(delta)/dt = L * v / (1 + (kappa*L)^2) * dkappa/ds
    const double one_plus_kappa_l_sq = 1.0 + (kappa * wheelbase) * (kappa * wheelbase);
    const double delta_dot =
        (wheelbase * v / one_plus_kappa_l_sq) * dkappa_ds;
    return {delta, delta_dot};
}

DifferentialFlatnessResult DifferentialFlatnessSolver::computeFlatOutputs(
    const DifferentialFlatnessInput& input,
    const VehicleParams& vehicle_params) const {
    const auto n_points = input.x.size();
    DifferentialFlatnessResult result;
    result.points.reserve(n_points);
    result.success = true;
    result.status_msg =
        "OK: " + std::to_string(n_points) + " points computed";
    for (std::size_t i = 0; i < n_points; ++i) {
        const auto [delta, delta_dot] = computeSinglePoint(
            input.x_d1[i], input.x_d2[i], input.x_d3[i], input.y_d1[i],
            input.y_d2[i], input.y_d3[i], input.v[i],
            vehicle_params.wheelbase,
            config_.curvature_denominator_epsilon);
        PathPoint point(input.x[i], input.y[i], input.theta[i]);
        point.setV(input.v[i]);
        point.setA(input.a[i]);
        point.setDelta(delta);
        point.setDeltaDot(delta_dot);
        result.points.push_back(point);
    }
    return result;
}

DifferentialFlatnessResult DifferentialFlatnessSolver::solve(
    const DifferentialFlatnessInput& input,
    const VehicleParams& vehicle_params) const {
    validateInputs(input, vehicle_params);
    return computeFlatOutputs(input, vehicle_params);
}

}  // namespace apa_post_processor
