#pragma once

#include "math_util.hpp"

namespace stc_SQP {
namespace so2 {
    // SO2 流形更新：给定当前角度 angle 与切空间增量 delta，
    // 返回 angle + delta 经 math_util::NormalizeAngle 规范化后的结果
    inline double Retract(double angle, double delta)
    {
        return math_util::NormalizeAngle(angle + delta);
    }
} // namespace so2
} // namespace stc_SQP
