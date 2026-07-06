#pragma once

namespace apa_post_processor {
// 用于定义项目中使用的常量以避免重复定义

// 相邻两点之间的默认距离（m）
inline constexpr double DELTA_DIST = 0.05;
// 浮点数比较阈值
inline constexpr double EPSILON = 1e-3;
// 精确浮点数比较阈值
inline constexpr double EPSILON_PRECISE = 1e-6;
// 日志打印时的数值精度
inline constexpr int PRINT_PRECISION = 2;
// 圆周率高精度常量
inline constexpr double PI = 3.14159265358979323846;
// 角度转弧度乘子 (约等于 0.0174533)
inline constexpr double DEG2RAD = PI / 180.0;
// 弧度转角度乘子 (约等于 57.2958)
inline constexpr double RAD2DEG = 180.0 / PI;
}  // namespace apa_post_processor