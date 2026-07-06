#pragma once

#include <cmath>

#include "../math/math_util.hpp"
#include "../math/so2.hpp"
#include "dynamical_system.h"

namespace stc_SQP {
// 前轮转角控制版阿克曼自行车模型。
// 状态 x = [x_pos, y_pos, theta, v, delta]^T，控制 u = [a, delta_dot]^T
class BicycleModelDelta : public DynamicalSystem {
protected:
    // 状态维度：x, y, theta, v, delta
    static constexpr int NX = 5;
    // 控制维度：a, delta_dot
    static constexpr int NU = 2;
    // x 坐标索引
    static constexpr int IDX_X = 0;
    // y 坐标索引
    static constexpr int IDX_Y = 1;
    // 航向角索引（SO2 流形）
    static constexpr int IDX_THETA = 2;
    // 纵向速度索引
    static constexpr int IDX_V = 3;
    // 前轮转角索引
    static constexpr int IDX_DELTA = 4;
    // 纵向加速度控制索引
    static constexpr int IDX_A = 0;
    // 前轮转角变化率控制索引
    static constexpr int IDX_DELTA_DOT = 1;

public:
    // 构造函数：wheelbase 为车辆轴距，必须大于 0。
    explicit BicycleModelDelta(double wheelbase = 2.8);
    ~BicycleModelDelta() override = default;
    // 状态维度 nx = 5
    int nx() const override { return NX; }
    // 控制维度 nu = 2
    int nu() const override { return NU; }
    // 连续动力学 f(x, u)：输出 x_dot，维度为 nx
    void evaluate(const Vector& x, const Vector& u, Vector& x_dot) const override;
    // 离散化并线性化：基于当前 (x, u)、时间步长 dt 与速度方向 v_sign，
    // 计算下一时刻状态 x_next 以及离散 Jacobian A = dx_next/dx、B = dx_next/du。
    // 仅用于兼容 DynamicalSystem 接口并供上层 OCP 进行换挡检测。
    void discretizeAndLinearize(const Vector& x, const Vector& u, double dt,
        double v_sign, Vector& x_next, Matrix& A, Matrix& B) const override;
    // 流形更新：x_new = x ⊕ (alpha * delta)
    void retract(const Vector& x, double alpha, const Vector& delta,
        Vector& x_new) const override;

protected:
    // 连续时间 Jacobian A_c = df/dx、B_c = df/du，供 RK4 变分方程积分使用。
    void computeContinuousJacobians(const Vector& x, Matrix& A_c, Matrix& B_c) const;

protected:
    double wheelbase_ = 2.8; // 车辆轴距 L
};
} // namespace stc_SQP
