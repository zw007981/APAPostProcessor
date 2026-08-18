#pragma once

#include <vector>

#include "../../util/maneuver.h"
#include "minco_config.h"

namespace apa_post_processor {
// 停驻窗口改写统计
struct MincoSteerPaddingStats {
    // 被合法化改写的窗口数
    int windows_legalized = 0;
    // 因净 Δθ 超阈值而保持原样的窗口数
    int windows_skipped = 0;
    // 改写实际使用的最大 |δ̇| 过渡斜率 (rad/s)（供评估是否满足硬件
    // 转向速率上限；超过说明窗口时间跨度不足以完成有界过渡）
    double max_steer_rate_used = 0.0;
};
// 对全部 Maneuver 的停驻窗口做"低速蠕行打轮"合法化改写：
// θ-s 换挡邻域的停驻窗口（|v|≈0）内轨迹表现为 θ̇≠0 且 δ 在 atan 值域内
// 翻转——v≈0 时 θ̇=v·tanδ/L≈0 与 δ 解耦，该状态物理上不可执行（普通
// 阿克曼车辆没有原地转向能力）。净 Δθ 很小的窗口全部点（含首末）原位
// 改写：x,y/v/a/t 保持原值（速度/位置/航向残差不受影响），θ 冻结为
// 窗口首点值（消除 pivot 旋转），δ 从窗口前最后一个驱动点的限幅值向
// 窗口后第一个驱动点的限幅值线性过渡（消除 atan 值域翻转伪影，且边界
// 对 Δδ 最小），δ̇ 取过渡斜率。净 Δθ 超阈值的窗口（真实 pivot 旋转
// 需求）保持原样并计数。
MincoSteerPaddingStats ApplySteerPadding(std::vector<Maneuver>& maneuvers,
                                       const MincoConfig& config);
}  // namespace apa_post_processor
