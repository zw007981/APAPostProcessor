#pragma once

#include <vector>

#include "maneuver.h"
#include "path.h"

namespace apa_post_processor {

// 拓扑清洗配置
struct TopologyCleanupConfig {
    // 极小段弧长阈值 (m)
    double min_arc_length = 0.05;
    // 前轮转角变化阈值 (rad)，超过此值转为PIVOT
    double pivot_delta_threshold = 0.1;
};

// 第一遍：对极小段分类并转化为原地打轮或标记剔除
void ClassifyAndResetManeuvers(std::vector<Maneuver>& maneuvers,
                               const TopologyCleanupConfig& config);

// 第二遍：剔除标记段、合并同向相邻段、构造新Path
Path ReconstructPath(const std::vector<Maneuver>& input_maneuvers);

}  // namespace apa_post_processor
