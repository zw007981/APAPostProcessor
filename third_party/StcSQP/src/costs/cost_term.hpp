#pragma once

#include <memory>
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
    // 一次调用同时得到标量代价、梯度与 Hessian；默认实现转发到 evaluate/gradient/hessian，
    // 保证已有子类不修改即可继续编译通过，行为完全不变。
    // 对内部持有昂贵计算（如 ESDF 地图查询）的代价，覆写此方法可消除重复计算。
    virtual void evaluateGradientAndHessian(const Vector& x, const Vector& u, double& cost,
        Vector& q, Vector& r, Matrix& Q, Matrix& R, Matrix& S) const
    {
        evaluate(x, u, cost);
        gradient(x, u, q, r);
        hessian(x, u, Q, R, S);
    }
    // 创建独立副本，供多线程并行 assembleQP/assembleCost 使用。
    // 含非拥有引用（如 EsdfMapInterface）的代价只需拷贝引用本身，
    // 不需要深拷贝地图数据（与 Constraint::clone() 约定一致）。
    virtual std::shared_ptr<CostTerm> clone() const = 0;
};
} // namespace stc_SQP
