#pragma once

#include <cstddef>

namespace apa_post_processor {
// 后处理算法通用配置基类（纯数据容器）。
// 派生类（NMPCConfig / IlqrSolverConfig 等）在此基础上追加各自专有参数。
// 禁止拷贝，仅允许通过指针/引用传递，避免切片。
struct Config {
    virtual ~Config() = default;
    // 允许拷贝（派生类拷贝时不会切片，基类直接拷贝的风险由调用方负责）
    Config(const Config&) = default;
    Config& operator=(const Config&) = default;
    Config(Config&&) = default;
    Config& operator=(Config&&) = default;

    // ============================================================
    // 第一组：轨迹合法性门禁阈值（决定"什么是合法轨迹"）
    // ============================================================
    // 最大允许碰撞深度 (m)
    double max_collision_depth = 0.02;
    // 终点位置误差上限 (m)，<0 表示不启用
    double terminal_position_error_threshold = 0.05;
    // 终点航向误差上限 (°)，<0 表示不启用
    double terminal_heading_error_threshold_deg = 1.5;

    // ============================================================
    // 第二组：Cost 权重（不同算法共享代价函数结构）
    // ============================================================
    // 控制代价：加速度 a
    double control_effort_accel_weight = 1e-2;
    // 控制代价：前轮转角变化率 δ_dot
    double control_effort_steer_rate_weight = 1e-2;
    // 顺滑代价：纵向加加速度 jerk
    double smoothing_jerk_weight = 1.0;
    // 顺滑代价：转向角加速度
    double smoothing_steer_accel_weight = 1.0;
    // 内部机动段状态代价：速度 v
    double interior_speed_weight = 1e-2;
    // 内部机动段状态代价：前轮转角 δ
    double interior_steer_weight = 1e-3;
    // 全程目标牵引代价：x, y 位置
    double global_target_position_weight = 1e-3;
    // 全程目标牵引代价：航向 θ
    double global_target_heading_weight = 1e-3;
    // 终端段状态代价：x, y 位置（权重远大于全局牵引）
    double terminal_position_weight = 1e5;
    // 终端段状态代价：航向 θ
    double terminal_heading_weight = 1e5;
    // 终端段状态代价：速度 v
    double terminal_speed_weight = 5.0;
    // 终端段状态代价：前轮转角 δ
    double terminal_steer_weight = 1.0;

    // ============================================================
    // 第三组：轨迹跟踪软代价（偏离参考轨迹的惩罚，默认关闭）
    // ============================================================
    // 位置跟踪软代价权重，0.0 表示关闭
    double position_tracking_weight = 0.0;
    // 位置跟踪软代价死区宽度 (m)
    double max_position_deviation_from_ref = 0.01;
    // 航向跟踪软代价权重，0.0 表示关闭
    double theta_tracking_weight = 0.0;
    // 航向跟踪软代价死区宽度 (rad)
    double max_theta_deviation_from_ref = 0.06;

    // ============================================================
    // 第四组：碰撞安全
    // ============================================================
    // ESDF 舒适间隙偏好 (m)，低于此距离时软代价开始生效
    double esdf_safety_margin = 0.1;
    // ESDF 碰撞惩罚软代价权重，提供 ~10cm 舒适间隙
    double esdf_penalty_weight = 3.0;

    // ============================================================
    // 第五组：求解器收敛
    // ============================================================
    // 最大迭代次数
    int max_iter = 100;

    // ============================================================
    // 第六组：打靶步长
    // ============================================================
    // OCP 离散化时间步长 (s)
    double dt = 0.1;

    // ============================================================
    // 调试输出开关
    // ============================================================
    bool enable_debug_output = false;

   protected:
    // 仅允许派生类构造
    Config() = default;
};
}  // namespace apa_post_processor