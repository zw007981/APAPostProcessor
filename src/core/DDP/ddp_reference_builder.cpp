#include "ddp_reference_builder.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace apa_post_processor {
namespace {
// 把 Path 展平为连续 Pose 序列（forEach 已剔除 maneuver 间共享的重复边界点）
std::vector<Pose> FlattenPath(const Path& path) {
    std::vector<Pose> points;
    points.reserve(path.size());
    path.forEach([&points](const TrajectoryPoint& point) {
        points.emplace_back(point.x, point.y, point.theta);
    });
    return points;
}

// 由方向枚举映射运动符号：前进 +1 / 后退 -1 / 原地转向或未知 0
int DirectionSign(Direction direction) {
    if (direction == Direction::FORWARD) {
        return 1;
    }
    if (direction == Direction::BACKWARD) {
        return -1;
    }
    return 0;
}

// 裁剪到对称盒 [-bound, bound]
double ClampSymmetric(double value, double bound) {
    return std::clamp(value, -bound, bound);
}
}  // namespace

DdpReferenceBuilder::DdpReferenceBuilder(DdpReferenceBuilderConfig config,
                                         const VehicleParams& vehicle_params)
    : config_(config), wheelbase_(vehicle_params.wheelbase) {
    if (!(config_.sample_dist > 0.0) || !(config_.dt > 0.0) ||
        config_.shooting_interval < 1 || !(config_.v_max > 0.0) ||
        !(config_.a_max > 0.0) || !(config_.delta_max > 0.0) ||
        !(config_.omega_max > 0.0)) {
        throw std::invalid_argument(
            "DdpReferenceBuilder: sample_dist/dt/shooting_interval/box bounds "
            "must be positive!!!");
    }
    if (!(wheelbase_ > EPSILON)) {
        throw std::invalid_argument(
            "DdpReferenceBuilder: vehicle wheelbase must be positive!!!");
    }
}

DdpReference DdpReferenceBuilder::build(const Path& path) const {
    const std::vector<Pose> points = FlattenPath(path);
    if (points.size() < 2) {
        throw std::invalid_argument(
            "DdpReferenceBuilder: path contains fewer than 2 points!!!");
    }
    // 累积无符号弧长（θ-only 的零位移段产生零增量，由重采样 ratio 守卫吸收）
    std::vector<double> arc_length(points.size(), 0.0);
    for (std::size_t i = 1; i < points.size(); ++i) {
        arc_length[i] =
            arc_length[i - 1] + std::hypot(points[i].x - points[i - 1].x,
                                           points[i].y - points[i - 1].y);
    }
    const double total_length = arc_length.back();
    if (total_length < config_.sample_dist) {
        throw std::invalid_argument(
            "DdpReferenceBuilder: path total length is shorter than one "
            "sample spacing!!!");
    }
    // 全长归一：段数按标称间距四舍五入，实际间距 = L / N，保证均匀覆盖终点
    const std::size_t num_steps = std::max<std::size_t>(
        1, static_cast<std::size_t>(
               std::lround(total_length / config_.sample_dist)));
    const double ds = total_length / static_cast<double>(num_steps);
    // 等弧长重采样：双指针扫描，θ 经统一 wrap 后线性插值
    DdpReference reference;
    reference.ds = ds;
    reference.dt = config_.dt;
    reference.poses.reserve(num_steps + 1);
    std::size_t segment = 0;
    for (std::size_t k = 0; k <= num_steps; ++k) {
        const double target =
            std::min(static_cast<double>(k) * ds, total_length);
        while (segment + 1 < points.size() &&
               arc_length[segment + 1] < target) {
            ++segment;
        }
        if (segment + 1 >= points.size()) {
            reference.poses.push_back(points.back());
            continue;
        }
        const double segment_length =
            arc_length[segment + 1] - arc_length[segment];
        const double ratio =
            segment_length > 0.0
                ? (target - arc_length[segment]) / segment_length
                : 0.0;
        const Pose& from = points[segment];
        const Pose& to = points[segment + 1];
        reference.poses.emplace_back(
            from.x + (to.x - from.x) * ratio, from.y + (to.y - from.y) * ratio,
            WrapAngle(from.theta + WrapAngle(to.theta - from.theta) * ratio));
    }
    // maneuver 元数据：边界弧长经四舍五入映射到网格索引（单调不减），
    // 相邻 maneuver 共享边界点索引；跨网格不足一个间距的微型 maneuver 可能
    // 不独占任何网格点，其元数据仍完整保留
    const auto& src_maneuvers = path.getManeuvers();
    reference.maneuvers.reserve(src_maneuvers.size());
    std::vector<std::size_t> end_indices;
    end_indices.reserve(src_maneuvers.size());
    double boundary_arc = 0.0;
    for (const auto& maneuver : src_maneuvers) {
        boundary_arc += maneuver.length();
        const long grid_index = std::clamp(std::lround(boundary_arc / ds), 0L,
                                           static_cast<long>(num_steps));
        end_indices.push_back(static_cast<std::size_t>(grid_index));
        const auto& m_points = maneuver.points;
        reference.maneuvers.push_back(DdpReferenceManeuver{
            DirectionSign(maneuver.direction), maneuver.length(),
            WrapAngle(m_points.back().theta - m_points.front().theta), 0, 0});
    }
    for (std::size_t m = 0; m < reference.maneuvers.size(); ++m) {
        reference.maneuvers[m].begin_index = (m == 0) ? 0 : end_indices[m - 1];
        reference.maneuvers[m].end_index = end_indices[m];
    }
    // cusp 检测：仅登记方向反号（符号均非零且相异）的 maneuver 边界；
    // 涉及原地转向/未知方向（符号 0）的边界不构成换挡尖点
    for (std::size_t m = 1; m < reference.maneuvers.size(); ++m) {
        if (reference.maneuvers[m - 1].sign * reference.maneuvers[m].sign ==
            -1) {
            reference.cusp_indices.push_back(end_indices[m - 1]);
        }
    }
    // 逐网格点 maneuver 归属：cusp 点归后一段，末点固定归最后一段
    // （末点 N 不参与下方 [begin, end) 循环赋值，由构造默认值直接给出）
    std::vector<std::size_t> point_maneuver(num_steps + 1,
                                            src_maneuvers.size() - 1);
    for (std::size_t m = 0; m < reference.maneuvers.size(); ++m) {
        for (std::size_t k = reference.maneuvers[m].begin_index;
             k < reference.maneuvers[m].end_index; ++k) {
            point_maneuver[k] = m;
        }
    }
    // 初值提取：v 由名义车速带符号给出，κ 由参考朝向差分得到并反解 δ，
    // a/ω 由 v/δ 经三点中心差分（端点单侧差分）得到，全部裁剪进盒约束
    const double v_nominal = ds / config_.dt;
    std::vector<double> v_init(num_steps + 1, 0.0);
    std::vector<double> delta_init(num_steps + 1, 0.0);
    for (std::size_t k = 0; k <= num_steps; ++k) {
        v_init[k] = ClampSymmetric(
            static_cast<double>(reference.maneuvers[point_maneuver[k]].sign) *
                v_nominal,
            config_.v_max);
    }
    for (std::size_t k = 0; k < num_steps; ++k) {
        const double kappa =
            WrapAngle(reference.poses[k + 1].theta - reference.poses[k].theta) /
            ds;
        delta_init[k] =
            ClampSymmetric(std::atan(wheelbase_ * kappa), config_.delta_max);
    }
    delta_init[num_steps] = delta_init[num_steps - 1];
    auto central_difference = [this](const std::vector<double>& values,
                                     double bound, std::vector<double>* out) {
        const std::size_t last = values.size() - 1;
        out->reserve(values.size());
        for (std::size_t k = 0; k <= last; ++k) {
            const double diff =
                (k == 0) ? (values[1] - values[0]) / config_.dt
                : (k == last)
                    ? (values[last] - values[last - 1]) / config_.dt
                    : (values[k + 1] - values[k - 1]) / (2.0 * config_.dt);
            out->push_back(ClampSymmetric(diff, bound));
        }
    };
    std::vector<double> a_init;
    std::vector<double> omega_init;
    central_difference(v_init, config_.a_max, &a_init);
    central_difference(delta_init, config_.omega_max, &omega_init);
    reference.initial_states.reserve(num_steps + 1);
    for (std::size_t k = 0; k <= num_steps; ++k) {
        DdpState state;
        state << reference.poses[k].x, reference.poses[k].y,
            reference.poses[k].theta, v_init[k], a_init[k], delta_init[k],
            omega_init[k];
        reference.initial_states.push_back(state);
    }
    reference.initial_controls.resize(num_steps, DdpControl::Zero());
    // 打靶节点布设：{每 n_s 步} ∪ {cusp} ∪ {末点 N}，排序去重
    reference.shooting_nodes.reserve(
        (num_steps + config_.shooting_interval - 1) /
            config_.shooting_interval +
        reference.cusp_indices.size() + 2);
    for (std::size_t node = 0; node < num_steps;
         node += config_.shooting_interval) {
        reference.shooting_nodes.push_back(node);
    }
    reference.shooting_nodes.insert(reference.shooting_nodes.end(),
                                    reference.cusp_indices.begin(),
                                    reference.cusp_indices.end());
    reference.shooting_nodes.push_back(num_steps);
    std::sort(reference.shooting_nodes.begin(), reference.shooting_nodes.end());
    reference.shooting_nodes.erase(std::unique(reference.shooting_nodes.begin(),
                                               reference.shooting_nodes.end()),
                                   reference.shooting_nodes.end());
    std::sort(reference.cusp_indices.begin(), reference.cusp_indices.end());
    reference.cusp_indices.erase(std::unique(reference.cusp_indices.begin(),
                                             reference.cusp_indices.end()),
                                 reference.cusp_indices.end());
    return reference;
}
}  // namespace apa_post_processor
