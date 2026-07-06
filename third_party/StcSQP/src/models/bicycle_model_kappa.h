#pragma once

#include <cmath>

#include "../math/math_util.hpp"
#include "../math/so2.hpp"
#include "dynamical_system.h"

namespace stc_SQP {
// 曲率控制版阿克曼自行车模型
// 状态 x = [x_pos, y_pos, theta, v, kappa]^T，控制 u = [a, kappa_dot]^T
class BicycleModelKappa : public DynamicalSystem {
protected:
    // 状态维度：x, y, theta, v, kappa
    static constexpr int NX = 5;
    // 控制维度：a, kappa_dot
    static constexpr int NU = 2;
    // x 坐标索引
    static constexpr int IDX_X = 0;
    // y 坐标索引
    static constexpr int IDX_Y = 1;
    // 航向角索引（SO2 流形）
    static constexpr int IDX_THETA = 2;
    // 纵向速度索引
    static constexpr int IDX_V = 3;
    // 曲率索引
    static constexpr int IDX_KAPPA = 4;
    // 纵向加速度控制索引
    static constexpr int IDX_A = 0;
    // 曲率变化率控制索引
    static constexpr int IDX_KAPPA_DOT = 1;

public:
    BicycleModelKappa() = default;
    ~BicycleModelKappa() override = default;
    // 状态维度 nx = 5
    int nx() const override { return NX; }
    // 控制维度 nu = 2
    int nu() const override { return NU; }
    // 连续动力学 f(x, u)：输出 x_dot，维度为 nx
    void evaluate(const Vector& x, const Vector& u, Vector& x_dot) const override;
    // 离散化并线性化：基于当前 (x, u)、时间步长 dt 与速度方向 v_sign，
    // RK4 同步积分状态与变分方程，得到 x_next 及离散 Jacobian A/B。
    // 仅用于兼容 DynamicalSystem 接口并供上层 OCP 进行换挡检测。
    void discretizeAndLinearize(const Vector& x, const Vector& u, double dt,
        double v_sign, Vector& x_next, Matrix& A, Matrix& B) const override;
    // 流形更新：x_new = x ⊕ (alpha * delta)
    void retract(const Vector& x, double alpha, const Vector& delta,
        Vector& x_new) const override;

protected:
    // 连续时间 Jacobian A_c = df/dx、B_c = df/du，供 RK4 变分方程积分使用。
    void computeContinuousJacobians(const Vector& x, Matrix& A_c, Matrix& B_c) const;
};

} // namespace stc_SQP
