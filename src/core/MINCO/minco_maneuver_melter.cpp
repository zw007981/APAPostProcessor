#include "minco_maneuver_melter.h"

#include <cmath>
#include <stdexcept>

#include "../../util/topology_cleaner.h"
#include "minco_trajectory_sampler.h"

namespace apa_post_processor {
MincoManeuverMelter::MincoManeuverMelter(const MincoConfig& config)
    : config_(config) {
    if (!(config_.melt_arc_threshold > 0.0) ||
        !std::isfinite(config_.melt_arc_threshold) ||
        !(config_.melt_heading_threshold > 0.0) ||
        !std::isfinite(config_.melt_heading_threshold)) {
        throw std::invalid_argument(
            "MincoConfig 融化阈值必须为正有限值");
    }
    if (config_.samples_per_segment < 2) {
        throw std::invalid_argument("MincoConfig 采样数须 >= 2");
    }
}

MincoMeltResult MincoManeuverMelter::meltAndPrune(
    const MincoTrajectory& trajectory,
    const std::vector<MincoManeuverEstimate>& estimates,
    const Eigen::Vector2d& start_position,
    const BicycleKinematicsExtractor& kinematics) const {
    CheckMincoSampleStructure(trajectory, estimates);
    if (!start_position.allFinite()) {
        throw std::invalid_argument("起点世界坐标必须为有限值");
    }
    MincoMeltResult result;
    result.maneuver_infos = detectMelting(trajectory, estimates);
    // 离散化委托给共享的纯函数工具，与"预处理粗优化轨迹"的离散化保持同一
    // 套管线，本类只叠加融化/剔除判断，不重复实现采样逻辑
    std::vector<Maneuver> maneuvers =
        SampleMincoTrajectory(trajectory, estimates, start_position, kinematics,
                              config_.samples_per_segment);
    for (std::size_t m = 0; m < maneuvers.size(); ++m) {
        switch (result.maneuver_infos[m].classification) {
            case MincoMeltClass::MELTED:
                // 复用 topology_cleaner 的删除标记约定：UNKNOWN 段在第二遍
                // 重建时被整体剔除
                maneuvers[m].direction = Direction::UNKNOWN;
                ++result.removed_count;
                break;
            case MincoMeltClass::PIVOT:
                // PIVOT 只作为信息性分类标签：段内采样点保留连续优化产出的
                // 真实折返轨迹（x,y,v,a,δ,δ̇ 全部原样），与 NORMAL 段完全
                // 同等对待。压平位置/清零速度会产生 v≡0 但 θ 变化的状态，
                // 与全链路唯一承认的运动学关系 θ̇=v·tanδ/L_base 直接矛盾；
                // 物理本质是一次压缩到极短时间/极小空间内的多点掉头式微动，
                // 而非真正的原地旋转
                maneuvers[m].direction = Direction::PIVOT;
                ++result.pivot_count;
                break;
            case MincoMeltClass::NORMAL:
                break;
        }
    }
    // 第二遍重建：剔除 UNKNOWN 段、同向相邻段合并（方向相反段绝不合并）
    result.path = ReconstructPath(maneuvers);
    result.pruned = result.removed_count > 0;
    return result;
}

std::vector<MincoManeuverMeltInfo> MincoManeuverMelter::detectMelting(
    const MincoTrajectory& trajectory,
    const std::vector<MincoManeuverEstimate>& estimates) const {
    CheckMincoSampleStructure(trajectory, estimates);
    std::vector<MincoManeuverMeltInfo> infos;
    infos.reserve(estimates.size());
    int segment_offset = 0;
    for (std::size_t m = 0; m < estimates.size(); ++m) {
        const int first_segment = segment_offset;
        const int last_segment =
            segment_offset + static_cast<int>(estimates[m].segments.size()) - 1;
        // θ/s 位置量跨段连续，Maneuver 的净位移直接由首末段端点精确取值
        const double s_start =
            trajectory.evaluateSegment(first_segment, 0.0, 0).y();
        const double s_end =
            trajectory
                .evaluateSegment(last_segment,
                                 trajectory.duration(last_segment), 0)
                .y();
        const double theta_start =
            trajectory.evaluateSegment(first_segment, 0.0, 0).x();
        const double theta_end =
            trajectory
                .evaluateSegment(last_segment,
                                 trajectory.duration(last_segment), 0)
                .x();
        MincoManeuverMeltInfo info;
        info.arc_displacement = std::abs(s_end - s_start);
        info.heading_change = std::abs(theta_end - theta_start);
        const bool tiny_arc =
            info.arc_displacement < config_.melt_arc_threshold;
        // 首尾段保护：只允许参与同向合并，既不允许被当作废段剔除，也不允许
        // 被重分类为 PIVOT——首段承载车辆当前位姿锚点、末段承载 MINCO 精确收敛
        // 的终点 x,y,θ 等式约束（微调必须保持已收敛的终点等式约束不被扰动），
        // 其真实运动方向是输出语义的一部分；改写为
        // PIVOT 标签会丢弃 FORWARD/BACKWARD 信息并阻断其参与同向合并
        const bool endpoint = (m == 0 || m + 1 == estimates.size());
        if (!endpoint && tiny_arc &&
            info.heading_change > config_.melt_heading_threshold) {
            info.classification = MincoMeltClass::PIVOT;
        } else if (tiny_arc && !endpoint) {
            info.classification = MincoMeltClass::MELTED;
        }
        infos.push_back(info);
        segment_offset = last_segment + 1;
    }
    return infos;
}

void ResegmentByVelocityDirection(Path* path, double v_epsilon) {
    if (path == nullptr || path->empty()) {
        return;
    }
    if (!std::isfinite(v_epsilon) || v_epsilon < 0.0) {
        throw std::invalid_argument(
            "ResegmentByVelocityDirection received invalid v_epsilon");
    }
    std::vector<Maneuver> out;
    out.reserve(path->numManeuvers());
    for (auto& m : path->getManeuvers()) {
        const auto& pts = m.points;
        if (pts.size() < 2) {
            out.push_back(std::move(m));
            continue;
        }
        // 按 v 符号切分点序列为方向游程：停驻点并入当前游程，前导停驻点
        // 暂存并挂到首个游程
        struct Run {
            Direction direction{Direction::UNKNOWN};
            std::vector<TrajectoryPoint> points;
        };
        std::vector<Run> runs;
        std::vector<TrajectoryPoint> pending;
        int cur_sign = 0;
        for (const auto& p : pts) {
            const int sign = (p.hasV() && std::abs(p.getV()) >= v_epsilon)
                                 ? (p.getV() > 0 ? 1 : -1)
                                 : 0;
            if (sign == 0) {
                if (runs.empty()) {
                    pending.push_back(p);
                } else {
                    runs.back().points.push_back(p);
                }
                continue;
            }
            if (runs.empty() || sign != cur_sign) {
                runs.push_back(
                    {sign > 0 ? Direction::FORWARD : Direction::BACKWARD, {}});
                cur_sign = sign;
            }
            runs.back().points.push_back(p);
        }
        if (runs.empty()) {
            // 整段无有效方向（全部停驻或无 v 数据）：原样保留
            out.push_back(std::move(m));
            continue;
        }
        // 组装为带共享边界点的机动段（后段首点 = 前段末点，Path::addPoint
        // 语义），方向由实际 v 符号决定，忠实于几何特征
        for (std::size_t r = 0; r < runs.size(); ++r) {
            Maneuver sub;
            sub.direction = runs[r].direction;
            if (r > 0) {
                sub.points.push_back(runs[r - 1].points.back());
            } else {
                sub.points.insert(sub.points.end(), pending.begin(),
                                  pending.end());
            }
            sub.points.insert(sub.points.end(), runs[r].points.begin(),
                              runs[r].points.end());
            out.push_back(std::move(sub));
        }
    }
    // 防御性同向合并 + 重算曲率（切分后相邻段方向相反，正常无合并）
    *path = ReconstructPath(out);
}
}  // namespace apa_post_processor
