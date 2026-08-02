#include "ddp_reference_builder.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

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

// 剪枝工作点：位姿 + 所属 maneuver 下标（剔除迭代期间维护归属关系）
struct TaggedPoint {
    Pose pose;
    std::size_t maneuver{0};
};

// 由位姿序列重建 Path：逐点追加由 addPoint 完成去重/方向推断，
// finalize 统一完成曲率估计（与 PruneRedundantCusps 末尾同一构造路径）
Path RebuildPath(const std::vector<Pose>& points) {
    Path result;
    for (const Pose& pose : points) {
        result.addPoint(pose);
    }
    result.finalize();
    return result;
}
}  // namespace

Path PruneRedundantCusps(const Path& path, const ESDFMap& esdf_map,
                         const VehicleFootprintModel& footprint_model,
                         const DdpCuspPruneConfig& config) {
    // 配置错误会让剪枝行为不可预期（负阈值无意义、负重叠系数/碰撞裕度
    // 使判据方向反转），必须显式拒绝
    if (!std::isfinite(config.max_prune_arc) || config.max_prune_arc < 0.0) {
        throw std::invalid_argument(
            "PruneRedundantCusps: 弧长阈值必须为非负有限值");
    }
    if (!std::isfinite(config.overlap_ratio) || config.overlap_ratio <= 0.0) {
        throw std::invalid_argument(
            "PruneRedundantCusps: 重叠判据系数必须为正有限值");
    }
    if (!std::isfinite(config.collision_margin) ||
        config.collision_margin < 0.0) {
        throw std::invalid_argument(
            "PruneRedundantCusps: 碰撞裕度必须为非负有限值");
    }
    // 阈值 0 = 关闭：原样返回（含少于 3 段无从剔除的情形）
    const auto& src_maneuvers = path.getManeuvers();
    if (config.max_prune_arc == 0.0 || src_maneuvers.size() < 3) {
        return path;
    }
    // 展平为带 maneuver 归属的工作点列（相邻 maneuver 的共享边界点只
    // 保留一次，归属后一段——与参考构建器的网格归属同一约定）
    std::vector<TaggedPoint> work;
    work.reserve(path.size());
    for (std::size_t m = 0; m < src_maneuvers.size(); ++m) {
        const auto& points = src_maneuvers[m].points;
        for (std::size_t i = 0; i < points.size(); ++i) {
            if (m > 0 && i == 0) {
                continue;
            }
            const auto& p = points[i];
            work.push_back(TaggedPoint{Pose{p.x, p.y, p.theta}, m});
        }
    }
    // ESDF 安全检查的外圆集合（局部坐标）与半径
    const auto outer_circles =
        vehicle_circle_geometry::ExtractLocalCircleCenters(footprint_model,
                                                           CircleType::OUTER);
    const double outer_radius = footprint_model.getOuterRadius();
    // 缝合桥安全检查：桥上按 ~0.05 m 取样（含两端点），逐样本位姿评估
    // 全部外圆的侵入深度，超过裕度即判不安全（调用方回滚本次剔除）
    const auto bridge_safe = [&](const Pose& from, const Pose& to) {
        const double span = std::hypot(to.x - from.x, to.y - from.y);
        const auto samples = static_cast<std::size_t>(
            std::max(1.0, std::ceil(span / DELTA_DIST)));
        for (std::size_t s = 0; s <= samples; ++s) {
            const double ratio =
                static_cast<double>(s) / static_cast<double>(samples);
            const double px = from.x + (to.x - from.x) * ratio;
            const double py = from.y + (to.y - from.y) * ratio;
            const double ptheta = WrapAngle(
                from.theta + WrapAngle(to.theta - from.theta) * ratio);
            const double cos_theta = std::cos(ptheta);
            const double sin_theta = std::sin(ptheta);
            for (const auto& local : outer_circles) {
                const double wx =
                    px + local.x() * cos_theta - local.y() * sin_theta;
                const double wy =
                    py + local.x() * sin_theta + local.y() * cos_theta;
                if (outer_radius - esdf_map.getDist(wx, wy) >
                    config.collision_margin) {
                    return false;
                }
            }
        }
        return true;
    };
    // 迭代剔除：每轮找到第一个通过全部守卫的冗余折返并应用，随后重扫
    // （剔除改变邻接关系，单次从左到右扫描会漏掉新形成的可剔除对）
    for (bool changed = true; changed;) {
        changed = false;
        // 由归属标记重建 maneuver 区间表（连续同 id 段）
        struct Span {
            std::size_t begin;
            std::size_t end;
            std::size_t maneuver;
        };
        std::vector<Span> spans;
        for (std::size_t k = 0; k < work.size(); ++k) {
            if (spans.empty() || work[k].maneuver != spans.back().maneuver) {
                spans.push_back(Span{k, k, work[k].maneuver});
            } else {
                spans.back().end = k;
            }
        }
        for (std::size_t s = 1; s + 1 < spans.size(); ++s) {
            const auto& span = spans[s];
            if (span.end == span.begin) {
                continue;  // 单点退化跨段（剔除残端）永远不是冗余折返
            }
            const int sign =
                DirectionSign(src_maneuvers[span.maneuver].direction);
            if (sign == 0) {
                continue;  // 原地转向段不参与（其朝向变化无法被缝合承载）
            }
            double arc = 0.0;
            for (std::size_t k = span.begin; k < span.end; ++k) {
                arc += std::hypot(work[k + 1].pose.x - work[k].pose.x,
                                  work[k + 1].pose.y - work[k].pose.y);
            }
            if (!(arc < config.max_prune_arc)) {
                continue;
            }
            // 重叠判据：折返终点 B 距前缀已覆盖区域最近点 Q 的距离
            const Pose& endpoint = work[span.end].pose;
            std::size_t nearest = 0;
            double nearest_dist = std::numeric_limits<double>::max();
            for (std::size_t k = 0; k <= span.begin; ++k) {
                const double dist = std::hypot(work[k].pose.x - endpoint.x,
                                               work[k].pose.y - endpoint.y);
                if (dist < nearest_dist) {
                    nearest_dist = dist;
                    nearest = k;
                }
            }
            if (nearest_dist > config.overlap_ratio * arc) {
                continue;  // 终点落在未探索新区域：真实位移，不剔除
            }
            // 航向一致性守卫：缝合桥 Q→B 的方向必须与两端点的航向轴
            // 共线（夹角 ≈0 或 ≈π；横向桥接对自行车模型不可达）；
            // 退化桥（|QB|≈0）则要求朝向几乎不变（否则等价于原地转向）
            const Pose& join = work[nearest].pose;
            if (nearest_dist > 1e-6) {
                const double bridge_heading =
                    std::atan2(endpoint.y - join.y, endpoint.x - join.x);
                const auto heading_gap = [bridge_heading](double theta) {
                    const double phi =
                        std::abs(WrapAngle(bridge_heading - theta));
                    return std::min(phi, std::abs(phi - PI));
                };
                if (heading_gap(join.theta) > 0.5 ||
                    heading_gap(endpoint.theta) > 0.5) {
                    continue;
                }
            } else if (std::abs(WrapAngle(endpoint.theta - join.theta)) > 0.1) {
                continue;
            }
            // ESDF 安全检查：缝合桥（含接缝点）任一外圆侵入超限即放弃
            // 本次剔除（被剪的段可能是绕障必需）
            if (!bridge_safe(join, endpoint)) {
                continue;
            }
            // 应用剔除：保留 [0..nearest] 与 [span.end..]，删除中间
            // （m_{i-1} 的三重覆盖尾段 + 整个冗余折返段）；折返终点 B
            // 保留但必须改挂后一段的归属——否则它作为被剔 maneuver 的
            // 单点残端留在区间表头部，会被后续迭代误剔
            const std::size_t next_maneuver = spans[s + 1].maneuver;
            work.erase(work.begin() + static_cast<long>(nearest) + 1,
                       work.begin() + static_cast<long>(span.end));
            if (nearest + 1 < work.size()) {
                work[nearest + 1].maneuver = next_maneuver;
            }
            changed = true;
            break;
        }
    }
    // 防御：剔除后路径退化（不足两点或总长不足一个采样间距）时放弃全部
    // 剪枝、原样返回——下游参考构建对退化输入抛异常，剪枝不得引入
    if (work.size() < 2) {
        return path;
    }
    double total = 0.0;
    for (std::size_t k = 1; k < work.size(); ++k) {
        total += std::hypot(work[k].pose.x - work[k - 1].pose.x,
                            work[k].pose.y - work[k - 1].pose.y);
    }
    if (total < DELTA_DIST) {
        return path;
    }
    // 重建 Path：逐点追加由 addPoint 完成去重/插值/方向推断，
    // finalize 统一完成曲率估计（与数据加载同一条构造路径）
    Path result;
    for (const auto& tagged : work) {
        result.addPoint(tagged.pose);
    }
    result.finalize();
    return result;
}

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

Path ProjectReferenceCurvature(const Path& path, double wheelbase,
                               double delta_max,
                               const DdpCurvatureProjectionConfig& config) {
    // 配置校验：比例阈值必须落在 [0,1]、车辆参数为正
    if (!std::isfinite(config.cap_ratio) || config.cap_ratio < 0.0 ||
        config.cap_ratio > 1.0) {
        throw std::invalid_argument(
            "ProjectReferenceCurvature: 曲率上限比例必须落在 [0,1]");
    }
    if (!(wheelbase > 0.0) || !(delta_max > 0.0)) {
        throw std::invalid_argument(
            "ProjectReferenceCurvature: 轴距与 δ_max 必须为正");
    }
    if (config.cap_ratio == 0.0 || path.empty()) {
        return path;
    }
    const double kappa_cap = config.cap_ratio * std::tan(delta_max) / wheelbase;
    if (!(kappa_cap > 0.0)) {
        return path;
    }
    // 展平为带 maneuver 归属的工作点列（相邻 maneuver 的共享边界点只
    // 保留一次，归属前一段——与预剪枝同一约定）
    std::vector<TaggedPoint> work;
    work.reserve(path.size());
    const auto& src_maneuvers = path.getManeuvers();
    for (std::size_t m = 0; m < src_maneuvers.size(); ++m) {
        const auto& points = src_maneuvers[m].points;
        for (std::size_t i = 0; i < points.size(); ++i) {
            if (m > 0 && i == 0) {
                continue;
            }
            const auto& p = points[i];
            work.push_back(TaggedPoint{Pose{p.x, p.y, p.theta}, m});
        }
    }
    if (work.size() < 2) {
        return path;
    }
    // 段长与段曲率：κ_i = wrap(Δθ_i)/ds_i（i 为段索引，连接
    // work[i]→work[i+1]）——与参考构建器 δ = atan(L·κ) 反解同一 θ 差分
    // 口径，只有这个口径的可行才等价于初值 δ 的可行。零位移段（原地
    // 转向）不产生运动曲率，其 Δθ 原样保留、不参与钳制与摊派
    const std::size_t num_segments = work.size() - 1;
    std::vector<double> seg_len(num_segments, 0.0);
    std::vector<double> seg_kappa(num_segments, 0.0);
    for (std::size_t i = 0; i < num_segments; ++i) {
        seg_len[i] = std::hypot(work[i + 1].pose.x - work[i].pose.x,
                                work[i + 1].pose.y - work[i].pose.y);
        if (seg_len[i] > 1e-9) {
            seg_kappa[i] =
                WrapAngle(work[i + 1].pose.theta - work[i].pose.theta) /
                seg_len[i];
        }
    }
    // 逐 maneuver 执行「钳制 + 航向守恒再分配」：超限段钳到 ±κ_cap，
    // 被钳掉的带符号航向按单位弧长均匀摊到本 maneuver 的未钳段上
    // （总航向变化严格守恒 ⟹ maneuver 端点航向逐位不变，跨 maneuver
    // 不出现连锁偏移）。段归属 = 终点所属 maneuver（跨界段是后一段的
    // 首次位移）。摊派后任何未钳段 |κ| 超限、或无未钳段可摊（整段贴限
    // 的参考没有上游几何裕度），该 maneuver 整段回滚保持原样
    std::vector<double> adjusted = seg_kappa;
    bool modified = false;
    std::size_t seg_begin = 0;
    for (std::size_t m = 0; m < src_maneuvers.size(); ++m) {
        std::size_t seg_end = seg_begin;
        while (seg_end < num_segments && work[seg_end + 1].maneuver == m) {
            ++seg_end;
        }
        double removed = 0.0;   // 被钳掉的带符号航向 (rad)
        double slack_ds = 0.0;  // 未钳正常段的总弧长 (m)
        for (std::size_t i = seg_begin; i < seg_end; ++i) {
            if (seg_len[i] <= 1e-9) {
                continue;
            }
            if (std::abs(seg_kappa[i]) > kappa_cap) {
                removed +=
                    (seg_kappa[i] - std::copysign(kappa_cap, seg_kappa[i])) *
                    seg_len[i];
            } else {
                slack_ds += seg_len[i];
            }
        }
        if (removed != 0.0 && slack_ds > 1e-9) {
            const double add = removed / slack_ds;
            bool feasible = true;
            for (std::size_t i = seg_begin; i < seg_end; ++i) {
                if (seg_len[i] <= 1e-9 || std::abs(seg_kappa[i]) > kappa_cap) {
                    continue;
                }
                if (std::abs(seg_kappa[i] + add) > kappa_cap + 1e-12) {
                    feasible = false;
                    break;
                }
            }
            if (feasible) {
                for (std::size_t i = seg_begin; i < seg_end; ++i) {
                    if (seg_len[i] <= 1e-9) {
                        continue;
                    }
                    adjusted[i] = std::abs(seg_kappa[i]) > kappa_cap
                                      ? std::copysign(kappa_cap, seg_kappa[i])
                                      : seg_kappa[i] + add;
                }
                modified = true;
            }
        }
        seg_begin = seg_end;
    }
    // 未做任何调整：逐位透传（回滚语义保证输出与输入一致）
    if (!modified) {
        return path;
    }
    // 由调整后的段曲率重积 θ（位置一律不动）：首点航向保持，逐段累进；
    // 零位移段的 Δθ 原样保留。守恒性保证各 maneuver 末点航向与原值一致
    std::vector<Pose> projected(work.size());
    projected[0] = work[0].pose;
    for (std::size_t i = 0; i < num_segments; ++i) {
        projected[i + 1] = work[i + 1].pose;
        const double dtheta =
            seg_len[i] > 1e-9
                ? adjusted[i] * seg_len[i]
                : WrapAngle(work[i + 1].pose.theta - work[i].pose.theta);
        projected[i + 1].theta = WrapAngle(projected[i].theta + dtheta);
    }
    return RebuildPath(projected);
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
                               : (values[k + 1] - values[k - 1]) /
                                     (2.0 * config_.dt);
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
