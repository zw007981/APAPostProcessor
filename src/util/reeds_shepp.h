#pragma once

#include <array>
#include <cstddef>
#include <vector>

#include "pose.h"

namespace apa_post_processor {
// Reeds-Shepp 曲线：连接两个位姿的最短「有界曲率 + 允许倒车」路径。
// 本模块只做纯几何生成，不涉及碰撞与动力学；使用方负责校验可行性。
//
// 为什么需要它：换挡点被路径搜索前端固定后，两个换挡点之间的几何形状也
// 就被钉死了，端点固定时曲率占用率无法再降低（降曲率必须增弧长，而弧长
// 被端点约束）。要真正减少换挡段数，必须允许换挡位姿本身移动、并用
// 有界曲率曲线重新连接——RS 曲线正是该问题的闭式最优解。

// RS 基元的转向类型（左转 / 直行 / 右转）
enum class RsSteer { LEFT = 0, STRAIGHT = 1, RIGHT = 2 };

// RS 基元：转向类型 + 归一化有符号长度。
// 长度以「单位转弯半径」为尺度：转向段的长度即转过的圆心角 (rad)，直行段
// 的长度即归一化弧长；符号表示行驶方向（正 = 前进，负 = 倒车）
struct RsSegment {
    RsSteer steer{RsSteer::STRAIGHT};
    double length{0.0};
};

// 一条完整的 RS 路径。RS 定理保证最短路径至多由 5 段基元组成
struct RsPath {
    // 至多 5 段基元，仅前 num_segments 个有效
    std::array<RsSegment, 5> segments{};
    int num_segments{0};
    // 归一化总长（各段 |length| 之和），乘以转弯半径即为实际弧长 (m)
    double normalized_length{0.0};
    // 求解是否成功（无解时为 false，所有其它字段无意义）
    bool valid{false};
    // 实际弧长 (m)
    double arcLength(double turning_radius) const {
        return normalized_length * turning_radius;
    }
    // 方向反转（尖点）次数：相邻基元的行驶方向符号发生变化的次数
    int numCusps() const;
};

// 采样得到的 RS 路径点：位姿 + 该点所属基元的行驶方向（true = 前进）
struct RsSamplePoint {
    Pose pose{};
    bool forward{true};
};

// 求解连接 start 与 goal 的最短 RS 路径。
// turning_radius 为允许的最小转弯半径 (m)，必须为正；非正值抛出
RsPath ComputeShortestReedsShepp(const Pose& start, const Pose& goal,
                                 double turning_radius);

// 按弧长步长 sample_dist (m) 离散 RS 路径为位姿序列（含首尾端点）。
// path 无效或步长非正时抛出；返回的相邻点间距不超过 sample_dist
std::vector<RsSamplePoint> SampleReedsShepp(const RsPath& path,
                                            const Pose& start,
                                            double turning_radius,
                                            double sample_dist);
}  // namespace apa_post_processor
