#include "adaptive_resampler.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <string>

#include "../util/constants.h"

namespace apa_post_processor {
namespace {
// 单段 Maneuver 最少有效样本点
constexpr int kHardMinActivePerSegment = 4;
// 原地打轮补丁的最少点数
constexpr int kMinPaddingCount = 1;
// 全退化场景兜底 delta_t_min (s)
constexpr double kFallbackDeltaTMin = 0.1;
}  // namespace

AdaptiveResampler::AdaptiveResampler(const AdaptiveResamplerConfig& config)
    : config_(config) {
    if (config_.n_max_pool <= 0 || config_.n_min_active_per_segment < 1 ||
        config_.nominal_step_s <= 0.0 || config_.density_w_base < 0.0 ||
        config_.density_w_kappa < 0.0 || config_.density_w_obs < 0.0 ||
        config_.dense_step_dist <= 0.0 || config_.steer_padding_epsilon < 0.0 ||
        config_.steer_safe_rate_ratio <= 0.0 ||
        config_.steer_safe_rate_ratio > 1.0 ||
        config_.min_segment_arc_length_for_degradation < 0.0 ||
        config_.time_reintegration_epsilon <= 0.0 ||
        config_.memory_pool_margin < 0 ||
        config_.obstacle_density_margin < 0.0) {
        throw std::invalid_argument(
            "AdaptiveResamplerConfig contains invalid values");
    }
}

void AdaptiveResampler::validateInputs(
    const std::vector<AdaptiveResamplerSegmentInput>& segments,
    const VehicleParams& vehicle_params) const {
    if (segments.empty()) {
        throw std::invalid_argument(
            "AdaptiveResampler requires at least one segment");
    }
    if (vehicle_params.wheelbase <= 0.0 ||
        vehicle_params.max_steer_rate <= 0.0) {
        throw std::invalid_argument(
            "AdaptiveResampler requires positive wheelbase and max_steer_rate");
    }
    for (std::size_t j = 0; j < segments.size(); ++j) {
        const auto& seg = segments[j];
        if (seg.states.size() != seg.dense_points.size()) {
            throw std::invalid_argument(
                "AdaptiveResampler segment " + std::to_string(j) +
                ": states and dense_points sizes mismatch");
        }
        if (seg.states.size() < 2) {
            throw std::invalid_argument("AdaptiveResampler segment " +
                                        std::to_string(j) +
                                        " requires at least 2 points");
        }
        for (std::size_t i = 0; i < seg.states.size(); ++i) {
            if (!seg.states[i].hasV() || !seg.states[i].hasA() ||
                !seg.states[i].hasDelta() || !seg.states[i].hasDeltaDot()) {
                throw std::invalid_argument("AdaptiveResampler segment " +
                                            std::to_string(j) + " point " +
                                            std::to_string(i) +
                                            " is missing v/a/delta/delta_dot");
            }
            if (!std::isfinite(seg.states[i].x) ||
                !std::isfinite(seg.states[i].y) ||
                !std::isfinite(seg.states[i].theta) ||
                !std::isfinite(seg.states[i].getV()) ||
                !std::isfinite(seg.states[i].getA()) ||
                !std::isfinite(seg.states[i].getDelta()) ||
                !std::isfinite(seg.states[i].getDeltaDot())) {
                throw std::invalid_argument("AdaptiveResampler segment " +
                                            std::to_string(j) + " point " +
                                            std::to_string(i) +
                                            " contains non-finite value");
            }
        }
        for (std::size_t i = 1; i < seg.dense_points.size(); ++i) {
            if (seg.dense_points[i].s < seg.dense_points[i - 1].s - EPSILON) {
                throw std::invalid_argument(
                    "AdaptiveResampler segment " + std::to_string(j) +
                    " dense_points arc length s must be non-decreasing");
            }
        }
    }
}

double AdaptiveResampler::computeSegmentArcLength(
    const AdaptiveResamplerSegmentInput& segment) const {
    const auto& dp = segment.dense_points;
    return dp.back().s - dp.front().s;
}

int AdaptiveResampler::computeBaseTotalDimension(double total_arc_length,
                                                 int segment_count) const {
    if (segment_count <= 0 || total_arc_length <= 0.0) {
        return 0;
    }
    const double nominal_count =
        std::round(total_arc_length / config_.nominal_step_s);
    const double lower_bound =
        static_cast<double>(segment_count * kHardMinActivePerSegment);
    const double upper_bound =
        static_cast<double>(config_.n_max_pool - config_.memory_pool_margin);
    const double clamped =
        std::max(lower_bound, std::min(nominal_count, upper_bound));
    return static_cast<int>(clamped);
}

std::vector<int> AdaptiveResampler::distributeActivePoints(
    const std::vector<double>& segment_arc_lengths, int base_total) const {
    const int segment_count = static_cast<int>(segment_arc_lengths.size());
    std::vector<int> active_counts(segment_count, kHardMinActivePerSegment);
    if (base_total <= segment_count * kHardMinActivePerSegment) {
        return active_counts;
    }
    const double total_length = std::accumulate(segment_arc_lengths.begin(),
                                                segment_arc_lengths.end(), 0.0);
    // 极端退化场景下直接返回硬下限
    if (total_length <= 1e-12) {
        return active_counts;
    }
    int remaining = base_total;
    int longest_idx = 0;
    double longest_length = -1.0;
    for (int j = 0; j < segment_count; ++j) {
        const double ratio = segment_arc_lengths[j] / total_length;
        const int allocated =
            std::max(kHardMinActivePerSegment,
                     static_cast<int>(
                         std::round(static_cast<double>(base_total) * ratio)));
        active_counts[j] = allocated;
        remaining -= allocated;
        if (segment_arc_lengths[j] > longest_length) {
            longest_length = segment_arc_lengths[j];
            longest_idx = j;
        }
    }
    // 将舍入误差补给/扣除自最长段
    active_counts[longest_idx] += remaining;
    active_counts[longest_idx] =
        std::max(kHardMinActivePerSegment, active_counts[longest_idx]);
    return active_counts;
}

std::vector<TrajectoryPoint> AdaptiveResampler::resampleDegenerateSegment(
    const AdaptiveResamplerSegmentInput& segment) const {
    std::vector<TrajectoryPoint> result;
    result.reserve(2);
    result.push_back(segment.states.front());
    result.push_back(segment.states.back());
    return result;
}

double AdaptiveResampler::deltaToKappa(double delta, double wheelbase) const {
    return std::tan(delta) / wheelbase;
}

std::vector<double> AdaptiveResampler::computeDensityFunction(
    const AdaptiveResamplerSegmentInput& segment,
    const VehicleParams& vehicle_params) const {
    const auto n_points = segment.dense_points.size();
    std::vector<double> rho;
    rho.reserve(n_points);
    for (std::size_t i = 0; i < n_points; ++i) {
        const double delta = segment.states[i].getDelta();
        const double kappa = deltaToKappa(delta, vehicle_params.wheelbase);
        double obs_density = 0.0;
        // 多圆一致性：对挂载的全部子圆求和
        for (const auto& circle : segment.dense_points[i].circles) {
            const double intrusion =
                std::max(0.0, config_.obstacle_density_margin - circle.dist);
            obs_density += intrusion * intrusion;
        }
        const double value = config_.density_w_base +
                             config_.density_w_kappa * std::abs(kappa) +
                             config_.density_w_obs * obs_density;
        rho.push_back(value);
    }
    return rho;
}

std::vector<double> AdaptiveResampler::integrateCdf(
    const std::vector<double>& s, const std::vector<double>& rho) const {
    const auto n_points = s.size();
    std::vector<double> cdf;
    cdf.reserve(n_points);
    cdf.push_back(0.0);
    for (std::size_t i = 1; i < n_points; ++i) {
        const double ds = s[i] - s[i - 1];
        const double integrated = 0.5 * (rho[i] + rho[i - 1]) * ds;
        cdf.push_back(cdf.back() + integrated);
    }
    return cdf;
}

double AdaptiveResampler::interpolateAngle(double a0, double a1,
                                           double alpha) const {
    const double diff = std::remainder(a1 - a0, 2.0 * PI);
    return a0 + alpha * diff;
}

std::vector<TrajectoryPoint> AdaptiveResampler::extractSamplesByCdf(
    const AdaptiveResamplerSegmentInput& segment, const std::vector<double>& s,
    const std::vector<double>& cdf, int n_active,
    std::vector<double>& sampled_s_out) const {
    std::vector<TrajectoryPoint> sampled;
    sampled.reserve(n_active);
    sampled_s_out.clear();
    sampled_s_out.reserve(n_active);
    if (n_active <= 0) {
        return sampled;
    }
    if (n_active == 1) {
        sampled.push_back(segment.states.front());
        sampled_s_out.push_back(s.front());
        return sampled;
    }
    const double cdf_end = cdf.back();
    if (cdf_end <= 0.0) {
        // CDF 退化为 0 时直接线性插值
        for (int k = 0; k < n_active; ++k) {
            const double alpha =
                static_cast<double>(k) / static_cast<double>(n_active - 1);
            const std::size_t idx = static_cast<std::size_t>(std::round(
                alpha * static_cast<double>(segment.states.size() - 1)));
            sampled.push_back(segment.states[idx]);
            sampled_s_out.push_back(s.front() + alpha * (s.back() - s.front()));
        }
        return sampled;
    }
    const double delta_cdf = cdf_end / static_cast<double>(n_active - 1);
    for (int k = 0; k < n_active; ++k) {
        const double target_cdf = static_cast<double>(k) * delta_cdf;
        auto it = std::lower_bound(cdf.begin(), cdf.end(), target_cdf);
        if (it == cdf.begin()) {
            sampled.push_back(segment.states.front());
            sampled_s_out.push_back(s.front());
            continue;
        }
        if (it == cdf.end()) {
            sampled.push_back(segment.states.back());
            sampled_s_out.push_back(s.back());
            continue;
        }
        const std::size_t right = static_cast<std::size_t>(it - cdf.begin());
        const std::size_t left = right - 1;
        const double denom = cdf[right] - cdf[left];
        const double alpha =
            (denom > 1e-12) ? ((target_cdf - cdf[left]) / denom) : 0.0;
        const TrajectoryPoint& p_left = segment.states[left];
        const TrajectoryPoint& p_right = segment.states[right];
        TrajectoryPoint point;
        point.x = p_left.x + alpha * (p_right.x - p_left.x);
        point.y = p_left.y + alpha * (p_right.y - p_left.y);
        point.theta = interpolateAngle(p_left.theta, p_right.theta, alpha);
        point.setV(p_left.getV() + alpha * (p_right.getV() - p_left.getV()));
        point.setA(p_left.getA() + alpha * (p_right.getA() - p_left.getA()));
        const double delta_left = p_left.getDelta();
        const double delta_right = p_right.getDelta();
        point.setDelta(interpolateAngle(delta_left, delta_right, alpha));
        point.setDeltaDot(p_left.getDeltaDot() +
                          alpha *
                              (p_right.getDeltaDot() - p_left.getDeltaDot()));
        sampled.push_back(point);
        sampled_s_out.push_back(s[left] + alpha * (s[right] - s[left]));
    }
    return sampled;
}

void AdaptiveResampler::recoverSegmentTimeSteps(
    const std::vector<TrajectoryPoint>& sampled_points,
    const std::vector<double>& sampled_s,
    std::vector<double>& delta_t_out) const {
    const auto n_sampled = sampled_points.size();
    if (n_sampled < 2) {
        return;
    }
    delta_t_out.reserve(n_sampled - 1);
    for (std::size_t k = 0; k + 1 < n_sampled; ++k) {
        const double ds = sampled_s[k + 1] - sampled_s[k];
        const double denom =
            std::max(std::abs(sampled_points[k].getV()) +
                         std::abs(sampled_points[k + 1].getV()),
                     config_.time_reintegration_epsilon);
        delta_t_out.push_back(2.0 * ds / denom);
    }
}

std::vector<TrajectoryPoint> AdaptiveResampler::resampleSegmentByDensity(
    const AdaptiveResamplerSegmentInput& segment, int n_active,
    const VehicleParams& vehicle_params,
    std::vector<double>& sampled_s_out) const {
    const auto n_points = segment.dense_points.size();
    std::vector<double> s;
    s.reserve(n_points);
    for (const auto& dpd : segment.dense_points) {
        s.push_back(dpd.s);
    }
    const auto rho = computeDensityFunction(segment, vehicle_params);
    const auto cdf = integrateCdf(s, rho);
    auto sampled =
        extractSamplesByCdf(segment, s, cdf, n_active, sampled_s_out);
    return sampled;
}

AdaptiveResampler::SteerPaddingSegment
AdaptiveResampler::buildSteerPaddingSegment(const TrajectoryPoint& anchor,
                                            double delta_start,
                                            double delta_end,
                                            double steer_safe_rate,
                                            double delta_t_min) const {
    SteerPaddingSegment padding;
    const double raw_delta = std::remainder(delta_end - delta_start, 2.0 * PI);
    const double delta_delta = std::abs(raw_delta);
    if (delta_delta <= config_.steer_padding_epsilon) {
        return padding;
    }
    const double t_steer = delta_delta / steer_safe_rate;
    // n_pad 为时间步数，至少为 1；当 t_steer 很小时也保证至少 1 步
    int n_pad = kMinPaddingCount;
    if (delta_t_min > 1e-12 && t_steer > delta_t_min) {
        n_pad = static_cast<int>(std::ceil(t_steer / delta_t_min));
        n_pad = std::max(kMinPaddingCount, n_pad);
    }
    const double delta_t_pad = t_steer / static_cast<double>(n_pad);
    const int sign = (raw_delta >= 0.0) ? 1 : -1;
    // n_pad 个时间步把 [delta_start, delta_end] 分成 n_pad 段，
    // 内部过渡点数量为 n_pad - 1，前后端点分别由相邻常规段端点承载，
    // 避免输出序列中出现完全重复的状态点。
    const int n_internal_points = std::max(0, n_pad - 1);
    padding.points.reserve(n_internal_points);
    padding.delta_t.reserve(n_pad);
    for (int i = 0; i < n_internal_points; ++i) {
        const double alpha =
            static_cast<double>(i + 1) / static_cast<double>(n_pad);
        TrajectoryPoint point(anchor.x, anchor.y, anchor.theta);
        point.setV(0.0);
        point.setA(0.0);
        point.setDelta(interpolateAngle(delta_start, delta_end, alpha));
        point.setDeltaDot(static_cast<double>(sign) * steer_safe_rate);
        padding.points.push_back(point);
    }
    for (int i = 0; i < n_pad; ++i) {
        padding.delta_t.push_back(delta_t_pad);
    }
    padding.pad_count = n_pad;
    padding.delta_start = delta_start;
    padding.delta_end = delta_end;
    return padding;
}

AdaptiveResampler::SteerPaddingSegment
AdaptiveResampler::buildCuspPaddingIfNeeded(const TrajectoryPoint& end_prev,
                                            const TrajectoryPoint& start_next,
                                            double steer_safe_rate,
                                            double delta_t_min) const {
    return buildSteerPaddingSegment(start_next, end_prev.getDelta(),
                                    start_next.getDelta(), steer_safe_rate,
                                    delta_t_min);
}

AdaptiveResampler::SteerPaddingSegment
AdaptiveResampler::buildStartPaddingIfNeeded(const TrajectoryPoint& first_point,
                                             double initial_steer_angle,
                                             double steer_safe_rate,
                                             double delta_t_min) const {
    return buildSteerPaddingSegment(first_point, initial_steer_angle,
                                    first_point.getDelta(), steer_safe_rate,
                                    delta_t_min);
}

AdaptiveResamplerResult AdaptiveResampler::assembleFinalTrajectory(
    std::vector<std::vector<TrajectoryPoint>>& segment_points,
    std::vector<std::vector<double>>& segment_delta_t,
    std::vector<SteerPaddingSegment>& cusp_paddings,
    SteerPaddingSegment& start_padding) const {
    AdaptiveResamplerResult result;
    result.success = true;
    result.points.reserve(config_.n_max_pool);
    result.delta_t.reserve(config_.n_max_pool);
    const int segment_count = static_cast<int>(segment_points.size());
    // 起始对齐补丁放在段 0 之前；其内部点连接 virtual initial_steer_angle
    // 与段 0 第一点。由于虚拟前驱不在输出序列中，第一条 delta_t 连接的是
    // 虚拟点 → 第一个内部点，应丢弃，否则 delta_t 会比 points 多 1。
    if (!start_padding.points.empty()) {
        result.points.insert(result.points.end(), start_padding.points.begin(),
                             start_padding.points.end());
        // n_pad >= 2 时 delta_t 至少有 2 条；跳过第一条（虚拟→内部）后，
        // 剩余 delta_t 正好与内部点 + 段 0 第一点构成连续区间。
        result.delta_t.insert(result.delta_t.end(),
                              start_padding.delta_t.begin() + 1,
                              start_padding.delta_t.end());
    }
    for (int j = 0; j < segment_count; ++j) {
        result.points.insert(result.points.end(), segment_points[j].begin(),
                             segment_points[j].end());
        result.delta_t.insert(result.delta_t.end(), segment_delta_t[j].begin(),
                              segment_delta_t[j].end());
        if (j + 1 < segment_count) {
            if (!cusp_paddings[j].points.empty()) {
                // 段间换挡补丁：n_pad 条 delta_t 连接"前一段终点 -> 内部点 ->
                // 下一段起点"，内部点 n_pad-1 个。
                result.points.insert(result.points.end(),
                                     cusp_paddings[j].points.begin(),
                                     cusp_paddings[j].points.end());
                result.delta_t.insert(result.delta_t.end(),
                                      cusp_paddings[j].delta_t.begin(),
                                      cusp_paddings[j].delta_t.end());
            } else {
                // 相邻常规段空间连续（真实泊车数据段间共享换挡点）时，
                // 需要补一条连接 delta_t，否则最终 delta_t.size() 会比
                // points.size() - 1 少 segment_count - 1。
                const auto& p0 = segment_points[j].back();
                const auto& p1 = segment_points[j + 1].front();
                constexpr double kSegmentConnectionEpsilon = 1e-3;
                const double gap = std::hypot(p1.x - p0.x, p1.y - p0.y);
                if (gap < kSegmentConnectionEpsilon) {
                    const double v_avg =
                        0.5 * (std::abs(p0.getV()) + std::abs(p1.getV()));
                    constexpr double kMinConnectSpeed = 1e-3;
                    const double dt = (v_avg > kMinConnectSpeed)
                                          ? gap / v_avg
                                          : kFallbackDeltaTMin;
                    result.delta_t.push_back(dt);
                }
            }
        }
    }
    result.final_dimension = static_cast<int>(result.points.size());
    result.status_msg =
        "OK: final dimension = " + std::to_string(result.final_dimension);
    return result;
}

AdaptiveResamplerResult AdaptiveResampler::enforceDimensionLimit(
    std::vector<std::vector<TrajectoryPoint>>& segment_points,
    std::vector<std::vector<double>>& segment_delta_t,
    const std::vector<AdaptiveResamplerSegmentInput>& segments,
    const VehicleParams& vehicle_params,
    std::vector<SteerPaddingSegment>& cusp_paddings,
    SteerPaddingSegment& start_padding,
    const std::vector<int>& active_points) const {
    AdaptiveResamplerResult result;
    result.success = true;
    // 计算补丁内部点总数
    auto countPaddingPoints = [&]() {
        int total = static_cast<int>(start_padding.points.size());
        for (const auto& pad : cusp_paddings) {
            total += static_cast<int>(pad.points.size());
        }
        return total;
    };
    int padding_total = countPaddingPoints();
    std::vector<int> adjusted_active = active_points;
    // 一级兜底：丢弃 Margin 富余，压缩常规段到 N_max_pool - padding_total
    int available_for_regular = config_.n_max_pool - padding_total;
    int regular_total = 0;
    for (const auto count : adjusted_active) {
        regular_total += count;
    }
    if (regular_total > available_for_regular) {
        const int deficit = regular_total - available_for_regular;
        std::vector<double> arc_lengths;
        arc_lengths.reserve(segments.size());
        for (const auto& seg : segments) {
            arc_lengths.push_back(computeSegmentArcLength(seg));
        }
        const int min_regular_total =
            static_cast<int>(segments.size()) * kHardMinActivePerSegment;
        int target_regular_total =
            std::max(min_regular_total, regular_total - deficit);
        adjusted_active =
            distributeActivePoints(arc_lengths, target_regular_total);
        // 重新生成常规段
        for (std::size_t j = 0; j < segments.size(); ++j) {
            std::vector<double> sampled_s;
            if (arc_lengths[j] <
                config_.min_segment_arc_length_for_degradation) {
                segment_points[j] = resampleDegenerateSegment(segments[j]);
            } else {
                segment_points[j] = resampleSegmentByDensity(
                    segments[j], adjusted_active[j], vehicle_params, sampled_s);
            }
            if (segment_points[j].size() == 2) {
                // 退化段按端点 s 线性分配
                sampled_s = {segments[j].dense_points.front().s,
                             segments[j].dense_points.back().s};
            }
            segment_delta_t[j].clear();
            recoverSegmentTimeSteps(segment_points[j], sampled_s,
                                    segment_delta_t[j]);
        }
        regular_total = 0;
        for (const auto count : adjusted_active) {
            regular_total += count;
        }
    }
    // 二级兜底：时间域动态压缩，循环减少补丁段点数直到满足维度限制。
    // 实现策略：保持转向总角度不变，等效放大单步物理时长（即放宽角速度
    // 假设），通过增大 delta_t_min 使 buildSteerPaddingSegment 重新计算的
    // n_pad 线性减少，从而将补丁内部点总数压入剩余额度。
    constexpr int kMaxCompressionIterations = 10;
    const double original_steer_rate =
        config_.steer_safe_rate_ratio * vehicle_params.max_steer_rate;
    for (int iteration = 0; iteration < kMaxCompressionIterations;
         ++iteration) {
        padding_total = countPaddingPoints();
        available_for_regular = config_.n_max_pool - padding_total;
        if (regular_total <= available_for_regular) {
            break;
        }
        const int min_regular_total =
            static_cast<int>(segments.size()) * kHardMinActivePerSegment;
        const int max_padding_allowed = config_.n_max_pool - min_regular_total;
        int current_padding = padding_total;
        double compression_ratio = 1.0;
        if (max_padding_allowed > 0 && current_padding > max_padding_allowed) {
            compression_ratio = static_cast<double>(current_padding) /
                                static_cast<double>(max_padding_allowed);
        }
        // 压缩起始补丁：使用段 0 第一点作为 anchor，保留原始 delta 端点
        if (!start_padding.points.empty()) {
            const double new_delta_t_min =
                start_padding.delta_t.front() * compression_ratio;
            start_padding = buildSteerPaddingSegment(
                segment_points.front().front(), start_padding.delta_start,
                start_padding.delta_end, original_steer_rate, new_delta_t_min);
        }
        // 压缩换挡补丁：使用段 j+1 第一点作为 anchor，保留原始 delta 端点
        for (std::size_t j = 0; j < cusp_paddings.size(); ++j) {
            if (cusp_paddings[j].points.empty()) {
                continue;
            }
            const double new_delta_t_min =
                cusp_paddings[j].delta_t.front() * compression_ratio;
            cusp_paddings[j] = buildSteerPaddingSegment(
                segment_points[j + 1].front(), cusp_paddings[j].delta_start,
                cusp_paddings[j].delta_end, original_steer_rate,
                new_delta_t_min);
        }
    }
    result = assembleFinalTrajectory(segment_points, segment_delta_t,
                                     cusp_paddings, start_padding);
    return result;
}

AdaptiveResamplerResult AdaptiveResampler::resample(
    const std::vector<AdaptiveResamplerSegmentInput>& segments,
    const VehicleParams& vehicle_params, double initial_steer_angle) const {
    validateInputs(segments, vehicle_params);
    const double steer_safe_rate =
        config_.steer_safe_rate_ratio * vehicle_params.max_steer_rate;
    // 计算每段弧长
    std::vector<double> arc_lengths;
    arc_lengths.reserve(segments.size());
    for (const auto& seg : segments) {
        arc_lengths.push_back(computeSegmentArcLength(seg));
    }
    const double total_arc_length =
        std::accumulate(arc_lengths.begin(), arc_lengths.end(), 0.0);
    const int base_total = computeBaseTotalDimension(
        total_arc_length, static_cast<int>(segments.size()));
    std::vector<int> active_points =
        distributeActivePoints(arc_lengths, base_total);
    // 逐段重采样
    std::vector<std::vector<TrajectoryPoint>> segment_points(segments.size());
    std::vector<std::vector<double>> segment_delta_t(segments.size());
    for (std::size_t j = 0; j < segments.size(); ++j) {
        std::vector<double> sampled_s;
        if (arc_lengths[j] < config_.min_segment_arc_length_for_degradation) {
            segment_points[j] = resampleDegenerateSegment(segments[j]);
            sampled_s = {segments[j].dense_points.front().s,
                         segments[j].dense_points.back().s};
        } else {
            segment_points[j] = resampleSegmentByDensity(
                segments[j], active_points[j], vehicle_params, sampled_s);
        }
        recoverSegmentTimeSteps(segment_points[j], sampled_s,
                                segment_delta_t[j]);
    }
    // 计算全局最小时间步长作为补丁段 delta_t_min 的参考。
    // 原地打轮不需要高时间分辨率，因此设置独立下限避免 n_pad 过度膨胀。
    constexpr double kSteerPaddingDeltaTMin = 0.05;
    double delta_t_min = kFallbackDeltaTMin;
    for (const auto& dt_vec : segment_delta_t) {
        for (const double dt : dt_vec) {
            if (dt > 1e-12 && dt < delta_t_min) {
                delta_t_min = dt;
            }
        }
    }
    delta_t_min = std::max(delta_t_min, kSteerPaddingDeltaTMin);
    // 评估段间换挡补丁
    std::vector<SteerPaddingSegment> cusp_paddings;
    cusp_paddings.reserve(segments.size() - 1);
    for (std::size_t j = 0; j + 1 < segments.size(); ++j) {
        cusp_paddings.push_back(buildCuspPaddingIfNeeded(
            segment_points[j].back(), segment_points[j + 1].front(),
            steer_safe_rate, delta_t_min));
    }
    // 评估起始转向对齐补丁
    SteerPaddingSegment start_padding = buildStartPaddingIfNeeded(
        segment_points[0].front(), initial_steer_angle, steer_safe_rate,
        delta_t_min);
    // 维度最终锁死
    auto result = enforceDimensionLimit(segment_points, segment_delta_t,
                                        segments, vehicle_params, cusp_paddings,
                                        start_padding, active_points);
    if (result.final_dimension > config_.n_max_pool) {
        result.success = false;
        result.status_msg = "Failed: final dimension " +
                            std::to_string(result.final_dimension) +
                            " exceeds n_max_pool " +
                            std::to_string(config_.n_max_pool);
    }
    return result;
}
}  // namespace apa_post_processor
