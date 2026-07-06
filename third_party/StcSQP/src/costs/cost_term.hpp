#pragma once

#include "core/types.h"

namespace stc_SQP {
// 代价项纯虚接口：SQP 引擎通过该接口统一访问各类代价
// 每个代价项负责在给定 (x, u) 处计算标量代价、一阶梯度以及二阶 Hessian。
class CostTerm {
public:
    virtual ~CostTerm() = default;
    // 计算标量代价 cost = L(x, u)
    virtual void evaluate(const Vector& x, const Vector& u, double& cost) const = 0;
    // 计算梯度：q = dL/dx in R^(nx), r = dL/du in R^(nu)
    virtual void gradient(const Vector& x, const Vector& u, Vector& q, Vector& r) const = 0;
    // 计算 Hessian：Q = d²L/dx² in R^(nx x nx), R = d²L/du² in R^(nu x nu), S = d²L/(du dx) in R^(nu x nx)
    virtual void hessian(const Vector& x, const Vector& u, Matrix& Q, Matrix& R, Matrix& S) const = 0;
};
} // namespace stc_SQP
