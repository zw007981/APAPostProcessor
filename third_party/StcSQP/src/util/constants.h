#pragma once

namespace stc_SQP {
// 用于定义项目中使用的常量以避免重复定义

// 相邻两点之间的默认距离（m）
inline constexpr double DELTA_DIST = 0.05;
// 浮点数比较默认阈值
inline constexpr double EPSILON = 1e-3;
// 浮点数比较精确阈值
inline constexpr double EPSILON_PRECISE = 1e-9;
// 日志打印时的数值精度
inline constexpr int PRINT_PRECISION = 2;
// 圆周率高精度常量
inline constexpr double PI = 3.14159265358979323846;
// 角度转弧度乘子 (约等于 0.0174533)
inline constexpr double DEG2RAD = PI / 180.0;
// 弧度转角度乘子 (约等于 57.2958)
inline constexpr double RAD2DEG = 180.0 / PI;
// 2π常量
inline constexpr double TWO_PI = 2.0 * PI;
// 1/(2π)常量
inline constexpr double INV_TWO_PI = 1.0 / TWO_PI;
// 通用 per-step 参数向量 p 的固定维度（与 autogen/common.py::P_DIM、
// generated/corridor.h 中的 CORRIDOR_P_DIM 以及 examples/parking 业务层保持一致）。
inline constexpr int STAGE_PARAM_DIM = 150;
}