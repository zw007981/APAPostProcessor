#include "topology_cleaner.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace apa_post_processor {

void ClassifyAndResetManeuvers(std::vector<Maneuver>& maneuvers,
                               const TopologyCleanupConfig& config) {
    for (auto& maneuver : maneuvers) {
        const double arc_length = maneuver.length();
        if (arc_length >= config.min_arc_length) {
            // 弧长足够，正常机动段
            continue;
        }
        if (maneuver.points.size() < 2) {
            // 点数不足，无法计算 Δδ
            continue;
        }
        // 计算首尾前轮转角变化量
        double delta_first = 0.0;
        double delta_last = 0.0;
        if (maneuver.points.front().hasDelta() &&
            maneuver.points.back().hasDelta()) {
            delta_first = maneuver.points.front().getDelta();
            delta_last = maneuver.points.back().getDelta();
        }
        const double abs_delta_delta = std::abs(delta_last - delta_first);
        if (abs_delta_delta > config.pivot_delta_threshold) {
            // 极小段 + 大 Δδ → 原地打轮(PIVOT)
            maneuver.direction = Direction::PIVOT;
            const double x0 = maneuver.points.front().x;
            const double y0 = maneuver.points.front().y;
            for (auto& pt : maneuver.points) {
                pt.x = x0;
                pt.y = y0;
                // θ 保留 NMPC 原值
                if (pt.hasV()) {
                    pt.setV(0.0);
                }
                if (pt.hasA()) {
                    pt.setA(0.0);
                }
                // δ 与 delta_dot 保留原值
            }
        } else {
            // 极小段 + 小 Δδ → 标记为待删除
            maneuver.direction = Direction::UNKNOWN;
        }
    }
}

Path ReconstructPath(const std::vector<Maneuver>& input_maneuvers) {
    // 第一遍：剔除 UNKNOWN 段
    std::vector<Maneuver> valid_maneuvers;
    valid_maneuvers.reserve(input_maneuvers.size());
    for (const auto& maneuver : input_maneuvers) {
        if (maneuver.direction == Direction::UNKNOWN) {
            continue;
        }
        valid_maneuvers.push_back(maneuver);
    }
    // 兜底：所有段均被剔除时，至少保留第一个段
    if (valid_maneuvers.empty() && !input_maneuvers.empty()) {
        valid_maneuvers.push_back(input_maneuvers.front());
    }
    // 第二遍：同向合并
    Path path;
    auto& out_maneuvers = path.getManeuvers();
    for (auto& maneuver : valid_maneuvers) {
        if (out_maneuvers.empty()) {
            out_maneuvers.push_back(std::move(maneuver));
            continue;
        }
        auto& last = out_maneuvers.back();
        if (last.direction == maneuver.direction) {
            // 同向合并：前段尾点与后段首点物理重合，pop_back 去重
            if (!last.points.empty()) {
                last.points.pop_back();
            }
            last.points.insert(last.points.end(),
                               std::make_move_iterator(maneuver.points.begin()),
                               std::make_move_iterator(maneuver.points.end()));
        } else {
            out_maneuvers.push_back(std::move(maneuver));
        }
    }
    path.finalize();
    return path;
}

}  // namespace apa_post_processor
