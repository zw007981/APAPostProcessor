#pragma once

#include "../core/types.h"

namespace stc_SQP {
// 动力学系统抽象接口：SQP 引擎与具体车辆模型的唯一耦合点
class DynamicalSystem {
public:
    virtual ~DynamicalSystem() = default;
    // 状态维度
    virtual int nx() const = 0;
    // 控制维度
    virtual int nu() const = 0;
    // 连续动力学 f(x, u)：输出 x_dot，维度为 nx
    virtual void evaluate(const Vector& x, const Vector& u, Vector& x_dot) const = 0;
    // 离散化并线性化：基于当前 (x, u)、时间步长 dt 与速度方向 v_sign，
    // 计算下一时刻状态 x_next 以及离散 Jacobian A = dx_next/dx、B = dx_next/du。
    // v_sign 取 +1.0 表示前进，-1.0 表示后退；模型可选择是否解释该符号。
    // 实现通常采用 RK4 对状态及变分方程进行同步积分。
    virtual void discretizeAndLinearize(const Vector& x, const Vector& u, double dt,
        double v_sign, Vector& x_next, Matrix& A, Matrix& B) const
        = 0;
    // 轻量离散化：仅返回下一时刻状态 x_next，不计算 Jacobian。
    // 默认实现复用 discretizeAndLinearize() 并丢弃 A/B；对计算敏感模型应重写为
    // 纯 RK4 状态传播，以节省变分方程积分开销。
    virtual void discretize(const Vector& x, const Vector& u, double dt,
        double v_sign, Vector& x_next) const
    {
        Matrix A(nx(), nx()), B(nx(), nu());
        discretizeAndLinearize(x, u, dt, v_sign, x_next, A, B);
    }
    // 流形更新：x_new = x ⊕ (alpha * delta)。
    // 默认实现为欧氏空间相加；含 theta 的模型必须重写，对 theta 维度调用 so2::Retract。
    virtual void retract(const Vector& x, double alpha, const Vector& delta,
        Vector& x_new) const
    {
        x_new = x + alpha * delta;
    }
};
} // namespace stc_SQP
