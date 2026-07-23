#pragma once

#include <vector>

#include "../../util/maneuver.h"

namespace apa_post_processor {
// 停驻窗口"停-打轮-走"合法化改写配置
struct AlmSteerPaddingConfig {
    // 停驻判定速度上限 (m/s)：|v| 低于该值的连续点段视为停驻窗口
    double v_epsilon = 0.05;
    // 可冻结窗口的最大净朝向变化 (rad)：窗口净 Δθ 低于该值才允许改写——
    // 冻结会把 Δθ 转嫁到下游航向（最终计入终点航向误差），阈值需与终点
    // 航向容差（1.5°）保持足够余量；超过该值的窗口含真实旋转需求（多点
    // 掉头机动），冻结会破坏几何，保持原样
    double max_freeze_dtheta = 0.02;
    // 前轮最大转角 δ_max (rad)（替换序列的转角限幅）
    double max_steer_angle = 0.48;
    // 前轮最大转角速度 δ̇_max (rad/s)（替换序列的过渡速率上限，决定窗口
    // 最短分配时长，不足时在窗口内插入更多点并顺延后续时间戳）
    double max_steer_rate = 0.4;
    // 替换序列的采样间隔 (s)
    double sample_dt = 0.1;
};
// 停驻窗口改写统计
struct AlmSteerPaddingStats {
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
AlmSteerPaddingStats ApplySteerPadding(
    std::vector<Maneuver>& maneuvers, const AlmSteerPaddingConfig& config);
}  // namespace apa_post_processor
