#pragma once

#include <cstddef>
#include <vector>

#include "../vehicle/vehicle_params.h"

namespace apa_post_processor {
// 梯形加减速时间参数化配置：约束参数默认值与 SpeedProfilePlannerConfig
// 同名字段保持一致
struct TimeProfileConfig {
    // 前进极限速度 (m/s)
    double max_v_forward = 1.389;
    // 倒车极限速度 (m/s)
    double max_v_reverse = 1.0;
    // 最大侧向加速度 (m/s^2)，曲率速度上限 v² <= a_lat_max/|κ|
    double max_lateral_accel = 1.0;
    // 时间戳反积分死区保护 (m/s)
    double time_reintegration_epsilon = 1e-3;
};
// 梯形加减速时序：与输入点序列一一对应的纵向时序量
struct TimeProfile {
    // 带符号纵向速度 v_i (m/s)
    std::vector<double> v;
    // 带符号纵向加速度 a_i (m/s^2)
    std::vector<double> a;
    // 各点时刻 t_i (s)，首点为 0，非递减
    std::vector<double> t;
};
// 由路径点几何量（已剔除换挡边界重复点）计算"最快走完"前提的梯形
// 加减速（bang-bang）时序：以首末点/换挡点为零速分段边界，每段前向
// 扫描按 max_accel 推进、后向扫描按 |max_decel| 回推，受极限速度与曲率
// 侧向加速度上限封顶；v=σ·w（w 为速率剖面），t 按 2Δs/(w_i+w_{i+1})
// 反积分，a 由 du/dt 差分给出（内部中心、端点单侧，分母非正置 0）。
// 加速度约定与 SpeedProfilePlanner 的 box 约束一致：空间加速度
// σ·a ∈ [max_decel, max_accel]（沿运动方向加速受油门极限、制动受刹车
// 极限，倒车制动时带符号 a 为正、幅值可达 |max_decel|）。
// s 为非递减累计弧长 (m)，kappa 为运动方向签名曲率 (1/m)，sigma ∈ {±1}，
// cusps 为严格内部的换挡点下标（该点车速为 0）；s/kappa/sigma 长度须
// 一致，cusps 越界、配置或车辆纵向极限非法时抛 std::invalid_argument
TimeProfile ComputeTimeProfile(const std::vector<double>& s,
                               const std::vector<double>& kappa,
                               const std::vector<int>& sigma,
                               const std::vector<std::size_t>& cusps,
                               const VehicleParams& vehicle_params,
                               const TimeProfileConfig& config = {});
}  // namespace apa_post_processor
