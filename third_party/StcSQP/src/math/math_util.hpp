#pragma once

#include <cmath>

#include "../util/constants.h"

namespace stc_SQP {
namespace math_util {
    // 将角度规范化到 (-π, π]；±π 统一映射到 π
    inline double NormalizeAngle(double angle)
    {
        return angle - TWO_PI * std::ceil((angle - PI) * INV_TWO_PI);
    }
} // namespace math_util
} // namespace stc_SQP
