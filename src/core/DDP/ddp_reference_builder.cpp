#include "ddp_reference_builder.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include "../NMPC/vehicle_circle_geometry.h"

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

// 由位姿序列重建 Path：逐点追加由 addPoint 完成去重/方向推断，
// finalize 统一完成曲率估计（与数据加载同一条构造路径）
Path RebuildPath(const std::vector<Pose>& points) {
    Path result;
    for (const Pose& pose : points) {
        result.addPoint(pose);
    }
    result.finalize();
    return result;
}

// 取 RS 路径首段与末段的行驶方向（true = 前进）：零长基元不表达方向，
// 必须跳过，否则接缝处的换挡计数会凭空多算一次
std::pair<bool, bool> FirstAndLastDirection(const RsPath& rs) {
    bool first = true;
    bool last = true;
    bool found_first = false;
    for (int i = 0; i < rs.num_segments; ++i) {
        const double length = rs.segments[i].length;
        if (std::abs(length) <= 1e-12) {
            continue;
        }
        if (!found_first) {
            first = length > 0.0;
            found_first = true;
        }
        last = length > 0.0;
    }
    return {first, last};
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

Path ShortcutShiftPoints(const Path& path, const ESDFMap& esdf_map,
                         const VehicleFootprintModel& footprint_model,
                         double wheelbase, double delta_max,
                         const DdpRsShortcutConfig& config) {
    if (!std::isfinite(config.cap_ratio) || config.cap_ratio < 0.0 ||
        config.cap_ratio > 1.0) {
        throw std::invalid_argument(
            "ShortcutShiftPoints: 曲率上限比例必须落在 [0,1]");
    }
    if (!std::isfinite(config.collision_margin) ||
        config.collision_margin < 0.0) {
        throw std::invalid_argument(
            "ShortcutShiftPoints: 碰撞裕度必须为非负有限值");
    }
    if (!std::isfinite(config.max_length_growth) ||
        config.max_length_growth < 0.0) {
        throw std::invalid_argument(
            "ShortcutShiftPoints: 长度增长上限必须为非负有限值");
    }
    if (config.index_stride < 1 || config.max_rounds < 0 ||
        !(config.sample_dist > 0.0)) {
        throw std::invalid_argument(
            "ShortcutShiftPoints: 扫描步长/轮数上限/采样间距非法");
    }
    if (!(wheelbase > 0.0) || !(delta_max > 0.0)) {
        throw std::invalid_argument(
            "ShortcutShiftPoints: 轴距与 δ_max 必须为正");
    }
    if (config.cap_ratio == 0.0 || config.max_rounds == 0 || path.empty()) {
        return path;
    }
    const double turning_radius =
        wheelbase / (config.cap_ratio * std::tan(delta_max));
    // 展平为「位姿 + 行驶方向」序列：换挡段数即方向翻转次数加一，全部
    // 判据都建立在这个序列上
    std::vector<Pose> poses;
    std::vector<char> forward;
    poses.reserve(path.size());
    forward.reserve(path.size());
    const auto& src_maneuvers = path.getManeuvers();
    for (std::size_t m = 0; m < src_maneuvers.size(); ++m) {
        const char dir =
            (src_maneuvers[m].direction == Direction::BACKWARD) ? 0 : 1;
        const auto& points = src_maneuvers[m].points;
        for (std::size_t i = 0; i < points.size(); ++i) {
            if (m > 0 && i == 0) {
                continue;
            }
            poses.emplace_back(points[i].x, points[i].y, points[i].theta);
            forward.push_back(dir);
        }
    }
    if (poses.size() < 3) {
        return path;
    }
    const auto outer_circles =
        vehicle_circle_geometry::ExtractLocalCircleCenters(footprint_model,
                                                           CircleType::OUTER);
    const double outer_radius = footprint_model.getOuterRadius();
    // 单条 RS 采样序列的安全判据：任一覆盖圆越界或侵入超裕度即拒绝
    const auto samples_safe = [&](const std::vector<RsSamplePoint>& samples) {
        for (const auto& sample : samples) {
            const double cos_theta = std::cos(sample.pose.theta);
            const double sin_theta = std::sin(sample.pose.theta);
            for (const auto& local : outer_circles) {
                const double wx = sample.pose.x + local.x() * cos_theta -
                                  local.y() * sin_theta;
                const double wy = sample.pose.y + local.x() * sin_theta +
                                  local.y() * cos_theta;
                if (!esdf_map.inMap(wx, wy)) {
                    return false;
                }
                if (outer_radius - esdf_map.getDist(wx, wy) >
                    config.collision_margin) {
                    return false;
                }
            }
        }
        return true;
    };
    // 原始总长是长度守卫的固定基准（不随贪心轮次放宽，避免逐轮累积膨胀）
    double original_length = 0.0;
    for (std::size_t i = 1; i < poses.size(); ++i) {
        original_length += std::hypot(poses[i].x - poses[i - 1].x,
                                      poses[i].y - poses[i - 1].y);
    }
    const double length_budget =
        original_length * (1.0 + config.max_length_growth);
    bool modified = false;
    for (int round = 0; round < config.max_rounds; ++round) {
        const int num_points = static_cast<int>(poses.size());
        // 前缀翻转数与前缀折线长：把候选评估中的段数与长度判据降到 O(1)，
        // 这样昂贵的逐点碰撞校验只需对少数通过廉价判据的候选执行
        std::vector<int> flip_prefix(poses.size(), 0);
        std::vector<double> length_prefix(poses.size(), 0.0);
        for (int i = 1; i < num_points; ++i) {
            flip_prefix[i] =
                flip_prefix[i - 1] + (forward[i] != forward[i - 1] ? 1 : 0);
            length_prefix[i] =
                length_prefix[i - 1] + std::hypot(poses[i].x - poses[i - 1].x,
                                                  poses[i].y - poses[i - 1].y);
        }
        const int current_flips = flip_prefix[num_points - 1];
        // 候选端点集：全部换挡点（本级存在的目标就是移动它们）并上
        // 每 stride 个采样点
        std::vector<int> candidates;
        candidates.reserve(static_cast<std::size_t>(num_points) /
                               static_cast<std::size_t>(config.index_stride) +
                           static_cast<std::size_t>(current_flips) + 2);
        for (int i = 0; i < num_points; ++i) {
            const bool is_shift = (i > 0 && forward[i] != forward[i - 1]);
            if (i % config.index_stride == 0 || is_shift ||
                i == num_points - 1) {
                candidates.push_back(i);
            }
        }
        int best_i = -1;
        int best_j = -1;
        int best_flips = current_flips;
        double best_length = 0.0;
        std::vector<RsSamplePoint> best_samples;
        for (const int i : candidates) {
            for (const int j : candidates) {
                if (j <= i + 1) {
                    continue;
                }
                // 欧氏下界预检：任何 i→j 曲线的弧长不小于端点直线距离，
                // 拼接总长下界已超预算的候选必被下方长度门拒绝，
                // RS 词（数十次三角函数）不必计算——精确剪枝不改变选取
                const double length_lower =
                    length_prefix[i] +
                    std::hypot(poses[j].x - poses[i].x,
                               poses[j].y - poses[i].y) +
                    (length_prefix[num_points - 1] - length_prefix[j]);
                if (length_lower > length_budget) {
                    continue;
                }
                const auto rs = ComputeShortestReedsShepp(poses[i], poses[j],
                                                          turning_radius);
                if (!rs.valid) {
                    continue;
                }
                // 拼接后的总长：前缀（含进入 i 的那条边）+ RS 弧长 + 后缀
                const double length =
                    length_prefix[i] + rs.arcLength(turning_radius) +
                    (length_prefix[num_points - 1] - length_prefix[j]);
                if (length > length_budget) {
                    continue;
                }
                // 拼接后的翻转数：前后缀内部翻转 + 两处接缝翻转 + RS 自身尖点
                const auto rs_dir = FirstAndLastDirection(rs);
                const int head_seam =
                    (i > 0 && (forward[i - 1] != 0) != rs_dir.first) ? 1 : 0;
                const int tail_seam = (j + 1 < num_points &&
                                       rs_dir.second != (forward[j + 1] != 0))
                                          ? 1
                                          : 0;
                const int flips =
                    (i > 0 ? flip_prefix[i - 1] : 0) + head_seam +
                    rs.numCusps() + tail_seam +
                    (j + 1 < num_points
                         ? flip_prefix[num_points - 1] - flip_prefix[j + 1]
                         : 0);
                if (flips > best_flips) {
                    continue;
                }
                if (flips == best_flips && best_i >= 0 &&
                    length >= best_length) {
                    continue;
                }
                // 段数未下降且长度也未改善时不值得更换同伦类
                if (flips == best_flips && best_i < 0 &&
                    length >= length_prefix[num_points - 1]) {
                    continue;
                }
                const auto samples = SampleReedsShepp(
                    rs, poses[i], turning_radius, config.sample_dist);
                if (!samples_safe(samples)) {
                    continue;
                }
                best_i = i;
                best_j = j;
                best_flips = flips;
                best_length = length;
                best_samples = samples;
            }
        }
        if (best_i < 0) {
            break;
        }
        std::vector<Pose> spliced_poses;
        std::vector<char> spliced_forward;
        const std::size_t spliced_size =
            static_cast<std::size_t>(best_i) + best_samples.size() +
            static_cast<std::size_t>(num_points - best_j - 1);
        spliced_poses.reserve(spliced_size);
        spliced_forward.reserve(spliced_size);
        for (int k = 0; k < best_i; ++k) {
            spliced_poses.push_back(poses[k]);
            spliced_forward.push_back(forward[k]);
        }
        for (const auto& sample : best_samples) {
            spliced_poses.push_back(sample.pose);
            spliced_forward.push_back(sample.forward ? 1 : 0);
        }
        for (int k = best_j + 1; k < num_points; ++k) {
            spliced_poses.push_back(poses[k]);
            spliced_forward.push_back(forward[k]);
        }
        poses = std::move(spliced_poses);
        forward = std::move(spliced_forward);
        modified = true;
    }
    if (!modified) {
        return path;
    }
    return RebuildPath(poses);
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
    reference.step_dt.assign(num_steps, config_.dt);
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
