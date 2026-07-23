#include "time_profile.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "constants.h"

namespace apa_post_processor {
TimeProfile ComputeTimeProfile(const std::vector<double>& s,
                               const std::vector<double>& kappa,
                               const std::vector<int>& sigma,
                               const std::vector<std::size_t>& cusps,
                               const VehicleParams& vehicle_params,
                               const TimeProfileConfig& config) {
    if (!std::isfinite(config.max_v_forward) || config.max_v_forward <= 0.0 ||
        !std::isfinite(config.max_v_reverse) || config.max_v_reverse <= 0.0 ||
        !std::isfinite(config.max_lateral_accel) ||
        config.max_lateral_accel <= 0.0 ||
        !std::isfinite(config.time_reintegration_epsilon) ||
        config.time_reintegration_epsilon <= 0.0) {
        throw std::invalid_argument(
            "ComputeTimeProfile: config contains invalid values!!!");
    }
    if (vehicle_params.max_accel <= 0.0 || vehicle_params.max_decel >= 0.0) {
        throw std::invalid_argument(
            "ComputeTimeProfile: max_accel must be positive and max_decel "
            "must be negative!!!");
    }
    if (kappa.size() != s.size() || sigma.size() != s.size()) {
        throw std::invalid_argument(
            "ComputeTimeProfile: s/kappa/sigma must have the same size!!!");
    }
    for (const auto cusp : cusps) {
        if (cusp == 0 || cusp + 1 >= s.size()) {
            throw std::invalid_argument(
                "ComputeTimeProfile: cusp index must be strictly interior!!!");
        }
    }
    const std::size_t n = s.size();
    TimeProfile output;
    output.v.assign(n, 0.0);
    output.a.assign(n, 0.0);
    output.t.assign(n, 0.0);
    if (n < 2) {
        return output;
    }
    // 零速边界序列：首点、各换挡点、末点；相邻边界间为一个匀向分段
    std::vector<std::size_t> bounds;
    bounds.reserve(cusps.size() + 2);
    bounds.push_back(0);
    bounds.insert(bounds.end(), cusps.begin(), cusps.end());
    bounds.push_back(n - 1);
    // w_i = |v_i|：前向-后向两遍扫描求 bang-bang 时间最优速率剖面
    std::vector<double> w(n, 0.0);
    for (std::size_t seg = 0; seg + 1 < bounds.size(); ++seg) {
        const std::size_t begin = bounds[seg];
        const std::size_t end = bounds[seg + 1];
        // 前向：由段首零速按加速度上限推进，受极限速度与曲率侧向加速度
        // 上限封顶
        for (std::size_t i = begin + 1; i <= end; ++i) {
            const double ds = s[i] - s[i - 1];
            const double v_ref =
                sigma[i] > 0 ? config.max_v_forward : config.max_v_reverse;
            double v_kappa = std::numeric_limits<double>::infinity();
            if (std::abs(kappa[i]) > EPSILON) {
                v_kappa =
                    std::sqrt(config.max_lateral_accel / std::abs(kappa[i]));
            }
            w[i] = std::min({v_ref, v_kappa,
                             std::sqrt(w[i - 1] * w[i - 1] +
                                       2.0 * vehicle_params.max_accel * ds)});
        }
        // 后向：由段末零速按减速度上限回推
        w[end] = 0.0;
        for (std::size_t i = end; i-- > begin;) {
            const double ds = s[i + 1] - s[i];
            w[i] = std::min(
                w[i], std::sqrt(w[i + 1] * w[i + 1] +
                                2.0 * std::abs(vehicle_params.max_decel) * ds));
        }
    }
    // 带符号速度 u_i = σ_i·w_i
    for (std::size_t i = 0; i < n; ++i) {
        output.v[i] = sigma[i] * w[i];
    }
    // 时刻反积分：dt = 2Δs/(w_i+w_{i+1})，全停点对由死区保护兜底
    for (std::size_t i = 0; i + 1 < n; ++i) {
        const double ds = s[i + 1] - s[i];
        const double denom =
            std::max(w[i] + w[i + 1], config.time_reintegration_epsilon);
        output.t[i + 1] = output.t[i] + 2.0 * ds / denom;
    }
    // 带符号加速度：du/dt 的差分（内部中心、端点单侧，分母非正置 0），
    // 与 Trajectory 构造侧 δ̇=dδ/dt 的差分做法一致
    for (std::size_t i = 0; i < n; ++i) {
        const std::size_t lo = (i > 0) ? i - 1 : i;
        const std::size_t hi = (i + 1 < n) ? i + 1 : i;
        const double dt = output.t[hi] - output.t[lo];
        output.a[i] = dt > 0.0 ? (output.v[hi] - output.v[lo]) / dt : 0.0;
    }
    return output;
}
}  // namespace apa_post_processor
