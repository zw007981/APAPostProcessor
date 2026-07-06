#pragma once

#include <cmath>

#include "../core/types.h"
#include "math_util.hpp"

namespace stc_SQP {
namespace se2 {
    // 由位姿 [x, y, theta] 构造 3x3 齐次变换矩阵T
    inline Matrix TransformFromPose(const Vector& pose)
    {
        Matrix T = Matrix::Identity(3, 3);
        const double c = std::cos(pose(2)), s = std::sin(pose(2));
        T(0, 0) = c;
        T(0, 1) = -s;
        T(1, 0) = s;
        T(1, 1) = c;
        T(0, 2) = pose(0);
        T(1, 2) = pose(1);
        return T;
    }

    // 将 SE2 变换作用于二维点 p（世界坐标），返回变换后的点
    inline Vector ApplyTransform(const Vector& pose, const Vector& p)
    {
        Vector result(2);
        const double c = std::cos(pose(2)), s = std::sin(pose(2));
        result(0) = pose(0) + c * p(0) - s * p(1);
        result(1) = pose(1) + s * p(0) + c * p(1);
        return result;
    }

    // 将 SE2 逆变换作用于二维点 p（即把世界点转换到 pose 局部坐标系）
    inline Vector ApplyInverseTransform(const Vector& pose, const Vector& p)
    {
        Vector result(2);
        const double dx = p(0) - pose(0), dy = p(1) - pose(1);
        const double c = std::cos(pose(2)), s = std::sin(pose(2));
        result(0) = c * dx + s * dy;
        result(1) = -s * dx + c * dy;
        return result;
    }

    // 位姿组合：result = pose_a ⊕ pose_b（先 b 后 a）。
    // 组合后的 theta 通过 math_util::NormalizeAngle 规范化到 (-π, π]，保证 SE2 表示唯一性。
    inline Vector Compose(const Vector& pose_a, const Vector& pose_b)
    {
        Vector result(3);
        const double theta_a = pose_a(2), theta_b = pose_b(2);
        const double c_a = std::cos(theta_a), s_a = std::sin(theta_a);
        result(0) = pose_a(0) + c_a * pose_b(0) - s_a * pose_b(1);
        result(1) = pose_a(1) + s_a * pose_b(0) + c_a * pose_b(1);
        result(2) = math_util::NormalizeAngle(theta_a + theta_b);
        return result;
    }

} // namespace se2
} // namespace stc_SQP
