#include "alm_maneuver_segmenter.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <stdexcept>
#include <utility>

#include "../../util/constants.h"

namespace apa_post_processor {
AlmManeuverSegmenter::AlmManeuverSegmenter(AlmManeuverSegmenterConfig config)
    : config_(config) {
    if (!std::isfinite(config.nominal_segment_length) ||
        config.nominal_segment_length <= 0.0) {
        throw std::invalid_argument("nominal_segment_length 必须为正有限值");
    }
    if (!std::isfinite(config.nominal_speed) || config.nominal_speed <= 0.0) {
        throw std::invalid_argument("nominal_speed 必须为正有限值");
    }
    if (!std::isfinite(config.nominal_turn_rate) ||
        config.nominal_turn_rate <= 0.0) {
        throw std::invalid_argument("nominal_turn_rate 必须为正有限值");
    }
    if (!std::isfinite(config.min_segment_duration) ||
        config.min_segment_duration <= 0.0) {
        throw std::invalid_argument("min_segment_duration 必须为正有限值");
    }
    if (!std::isfinite(config.fuse_arc_threshold) ||
        config.fuse_arc_threshold < 0.0) {
        throw std::invalid_argument("fuse_arc_threshold 必须为非负有限值");
    }
    if (!std::isfinite(config.fuse_heading_threshold) ||
        config.fuse_heading_threshold <= 0.0) {
        throw std::invalid_argument("fuse_heading_threshold 必须为正有限值");
    }
}

std::vector<AlmManeuverEstimate> AlmManeuverSegmenter::segment(
    const Path& path) const {
    if (path.empty()) {
        throw std::invalid_argument("输入 Path 为空，无法解析");
    }
    std::vector<AlmManeuverEstimate> estimates;
    estimates.reserve(path.numManeuvers());
    // 带符号累积弧长与解缠绕朝向均跨 Maneuver 连续传递，保证全局 θ-s 链一致
    double cumulative_arc = 0.0;
    double prev_theta = path.front().theta;
    for (const auto& maneuver : path.getManeuvers()) {
        estimates.emplace_back(
            segmentManeuver(maneuver, &cumulative_arc, &prev_theta));
    }
    // 微段融合（默认关闭）：移除内部摆动微段，θ-s 优化的初始结构不再
    // 携带微抖动换挡
    if (config_.fuse_arc_threshold > 0.0) {
        estimates = FuseShortManeuvers(std::move(estimates));
    }
    return estimates;
}

std::vector<AlmManeuverEstimate> AlmManeuverSegmenter::FuseShortManeuvers(
    std::vector<AlmManeuverEstimate> estimates) const {
    const std::size_t n = estimates.size();
    if (n <= 2) {
        // 首末段绝对保护，序列过短时无可融对象
        return estimates;
    }
    // 第一遍：标记融合候选（内部段、方向 FORWARD/BACKWARD、|Δs| 与 |Δθ|
    // 均低于阈值）；判据量取 θ-s 链上的精确值（段终点 − 段起点），与
    // melter 的融化判据同构
    std::vector<bool> fused(n, false);
    for (std::size_t i = 1; i + 1 < n; ++i) {
        const auto& estimate = estimates[i];
        if (estimate.direction != Direction::FORWARD &&
            estimate.direction != Direction::BACKWARD) {
            continue;
        }
        const double arc_span = std::abs(estimate.segments.back().arc_length -
                                         estimate.start_arc_length);
        const double heading_change =
            std::abs(estimate.segments.back().theta - estimate.start_theta);
        if (arc_span < config_.fuse_arc_threshold &&
            heading_change < config_.fuse_heading_threshold) {
            fused[i] = true;
        }
    }
    // 第二遍：重建——移除融合段、后续段的累积弧长整体平移重锚（融合段的
    // 带符号跨度被后续段吸收，保持 θ-s 链连续）、同向邻段合并
    std::vector<AlmManeuverEstimate> result;
    result.reserve(n);
    double arc_shift = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        if (fused[i]) {
            // 融合段的带符号弧长跨度（含方向符号，后退为负）
            arc_shift -= estimates[i].segments.back().arc_length -
                         estimates[i].start_arc_length;
            continue;
        }
        AlmManeuverEstimate kept = std::move(estimates[i]);
        kept.start_arc_length += arc_shift;
        for (auto& segment : kept.segments) {
            segment.arc_length += arc_shift;
        }
        // 同向邻段合并：重锚后弧长已连续，直接拼接段序列
        if (!result.empty() && result.back().direction == kept.direction &&
            (kept.direction == Direction::FORWARD ||
             kept.direction == Direction::BACKWARD)) {
            auto& tail = result.back().segments;
            tail.insert(tail.end(),
                        std::make_move_iterator(kept.segments.begin()),
                        std::make_move_iterator(kept.segments.end()));
            continue;
        }
        result.push_back(std::move(kept));
    }
    return result;
}

AlmManeuverEstimate AlmManeuverSegmenter::segmentManeuver(
    const Maneuver& maneuver, double* cumulative_arc,
    double* prev_theta) const {
    const auto& points = maneuver.points;
    const int num_points = static_cast<int>(points.size());
    if (num_points == 0) {
        throw std::invalid_argument("Maneuver 不含任何路径点");
    }
    AlmManeuverEstimate estimate;
    estimate.direction = maneuver.direction;
    estimate.start_arc_length = *cumulative_arc;
    estimate.start_theta = UnwrapAngle(points.front().theta, *prev_theta);
    *prev_theta = estimate.start_theta;
    const double sign = DirectionSign(maneuver.direction);
    if (num_points == 1) {
        // 单点 Maneuver：无位移、无朝向变化，退化为一段（时长取下限）
        AlmSegmentEstimate segment;
        segment.desired_position = {points.front().x, points.front().y};
        segment.theta = estimate.start_theta;
        segment.arc_length = *cumulative_arc;
        segment.duration = config_.min_segment_duration;
        estimate.segments.push_back(segment);
        return estimate;
    }
    // 空间等距降采样：M=ceil(L/d_seg)，K_step=floor((N-1)/M)；锚点取
    // 0, K, 2K, ..., (M-1)K, N-1，末锚点强制为路径终点（末段可能略长）
    const double length = maneuver.length();
    const int num_segments = std::max(
        1,
        static_cast<int>(std::ceil(length / config_.nominal_segment_length)));
    const int k_step = std::max(1, (num_points - 1) / num_segments);
    std::vector<int> anchors;
    anchors.reserve(num_segments + 1);
    for (int i = 0; i < num_segments; ++i) {
        anchors.push_back(std::min(i * k_step, num_points - 1));
    }
    if (anchors.back() != num_points - 1) {
        anchors.push_back(num_points - 1);
    }
    estimate.segments.reserve(anchors.size() - 1);
    double segment_start_theta = estimate.start_theta;
    for (std::vector<int>::size_type j = 0; j + 1 < anchors.size(); ++j) {
        // 段内实际折线弧长（带方向符号累积）
        double dist = 0.0;
        for (int k = anchors[j]; k < anchors[j + 1]; ++k) {
            dist += std::hypot(points[k + 1].x - points[k].x,
                               points[k + 1].y - points[k].y);
        }
        const auto& end_point = points[anchors[j + 1]];
        AlmSegmentEstimate segment;
        *cumulative_arc += sign * dist;
        segment.arc_length = *cumulative_arc;
        segment.theta = UnwrapAngle(end_point.theta, *prev_theta);
        *prev_theta = segment.theta;
        segment.desired_position = {end_point.x, end_point.y};
        const double delta_theta =
            std::abs(segment.theta - segment_start_theta);
        // 时长初值取行驶/转向/下限三者的最大值，保证任何退化输入下都为正
        segment.duration = std::max({dist / config_.nominal_speed,
                                     delta_theta / config_.nominal_turn_rate,
                                     config_.min_segment_duration});
        segment_start_theta = segment.theta;
        estimate.segments.push_back(segment);
    }
    return estimate;
}

double AlmManeuverSegmenter::DirectionSign(Direction direction) {
    switch (direction) {
        case Direction::FORWARD:
            return 1.0;
        case Direction::BACKWARD:
            return -1.0;
        case Direction::PIVOT:
            return 0.0;
        default:
            return 1.0;
    }
}

double AlmManeuverSegmenter::UnwrapAngle(double theta, double reference) {
    return theta - 2.0 * PI * std::floor((theta - reference + PI) / (2.0 * PI));
}
}  // namespace apa_post_processor
