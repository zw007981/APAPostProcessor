#pragma once

namespace apa_post_processor {

// 相邻点默认距离 (m)
inline constexpr double DELTA_DIST = 0.05;
// 浮点数比较阈值
inline constexpr double EPSILON = 1e-3;
// 精确比较阈值
inline constexpr double EPSILON_PRECISE = 1e-6;
// 碰撞检测浮点容差 (m)
inline constexpr double kCollisionEpsilon = 1e-6;
// 日志打印精度
inline constexpr int PRINT_PRECISION = 2;
// 圆周率
inline constexpr double PI = 3.14159265358979323846;
// 角度转弧度乘子
inline constexpr double DEG2RAD = PI / 180.0;
// 弧度转角度乘子
inline constexpr double RAD2DEG = 180.0 / PI;
}  // namespace apa_post_processor