#include "alm_trajectory_sampler.h"

#include <cmath>
#include <stdexcept>

namespace apa_post_processor {
std::vector<Maneuver> SampleMincoTrajectory(
    const MincoTrajectory& trajectory,
    const std::vector<AlmManeuverEstimate>& estimates,
    const Eigen::Vector2d& start_position,
    const BicycleKinematicsExtractor& kinematics, int samples_per_segment) {
    CheckMincoSampleStructure(trajectory, estimates);
    if (!start_position.allFinite()) {
        throw std::invalid_argument("起点世界坐标必须为有限值");
    }
    if (samples_per_segment < 2) {
        throw std::invalid_argument("每段采样点数须 >= 2");
    }
    const int num_samples = samples_per_segment;
    std::vector<Maneuver> maneuvers;
    maneuvers.reserve(estimates.size());
    // 跨 Maneuver 连续积分的位置状态与上一采样点的运动状态（梯形积分）
    Eigen::Vector2d position = start_position;
    double segment_start_time = 0.0;
    double prev_sample_time = 0.0;
    double prev_s_dot = 0.0;
    Eigen::Vector2d prev_direction(1.0, 0.0);
    bool has_prev_sample = false;
    int segment_offset = 0;
    const int total_segments = trajectory.numSegments();
    // 由轨迹求值组装一个采样点并推进积分位置（lambda 捕获上述积分状态）
    auto append_sample = [&](std::vector<TrajectoryPoint>* points,
                             int global_segment, double local_time,
                             double global_time) {
        ThetaSSample sample;
        sample.theta =
            trajectory.evaluateSegment(global_segment, local_time, 0).x();
        sample.theta_dot =
            trajectory.evaluateSegment(global_segment, local_time, 1).x();
        sample.theta_ddot =
            trajectory.evaluateSegment(global_segment, local_time, 2).x();
        sample.s =
            trajectory.evaluateSegment(global_segment, local_time, 0).y();
        sample.s_dot =
            trajectory.evaluateSegment(global_segment, local_time, 1).y();
        sample.s_ddot =
            trajectory.evaluateSegment(global_segment, local_time, 2).y();
        const Eigen::Vector2d direction(std::cos(sample.theta),
                                        std::sin(sample.theta));
        if (has_prev_sample) {
            position +=
                0.5 * (global_time - prev_sample_time) *
                (prev_s_dot * prev_direction + sample.s_dot * direction);
        }
        const AckermannState state = kinematics.extract(sample);
        TrajectoryPoint point;
        point.x = position.x();
        point.y = position.y();
        point.theta = sample.theta;
        point.setV(state.v);
        point.setA(state.a);
        point.setDelta(state.delta);
        point.setDeltaDot(state.delta_dot);
        // θ-s 轨迹本身是时间参数化的多项式，采样点携带真实全局时刻，
        // 使梯形配点残差运动学校验对离散化产出直接可用
        point.setT(global_time);
        points->push_back(point);
        prev_sample_time = global_time;
        prev_s_dot = sample.s_dot;
        prev_direction = direction;
        has_prev_sample = true;
    };
    for (std::size_t m = 0; m < estimates.size(); ++m) {
        std::vector<TrajectoryPoint> points;
        const int segment_count =
            static_cast<int>(estimates[m].segments.size());
        points.reserve(segment_count * num_samples + 1);
        for (int seg = 0; seg < segment_count; ++seg) {
            const int global_segment = segment_offset + seg;
            const double duration_g = trajectory.duration(global_segment);
            // 半开区间采样：段末端点由下一段首点覆盖，避免连接处重复
            for (int j = 0; j < num_samples; ++j) {
                const double local_time = duration_g * j / num_samples;
                append_sample(&points, global_segment, local_time,
                              segment_start_time + local_time);
            }
            segment_start_time += duration_g;
        }
        // 全局末段补终点采样（全局轨迹终点，非半开区间覆盖范围）
        if (segment_offset + segment_count == total_segments) {
            append_sample(&points, total_segments - 1,
                          trajectory.duration(total_segments - 1),
                          segment_start_time);
        }
        segment_offset += segment_count;
        maneuvers.emplace_back(std::move(points), estimates[m].direction);
    }
    return maneuvers;
}

void CheckMincoSampleStructure(
    const MincoTrajectory& trajectory,
    const std::vector<AlmManeuverEstimate>& estimates) {
    if (estimates.empty()) {
        throw std::invalid_argument("初值估计不能为空");
    }
    // 段数求和保持在 size_t 域，仅在最终比较时一次性转换（numSegments 恒 >=
    // 0）， 避免逐元素 static_cast<int> 在理论极端输入下的截断路径
    std::size_t total_segments = 0;
    for (const auto& estimate : estimates) {
        if (estimate.segments.empty()) {
            throw std::invalid_argument("每个 Maneuver 至少需要一个微观段");
        }
        total_segments += estimate.segments.size();
    }
    if (total_segments != static_cast<std::size_t>(trajectory.numSegments())) {
        throw std::invalid_argument("初值估计与轨迹的段数不一致");
    }
}
}  // namespace apa_post_processor
