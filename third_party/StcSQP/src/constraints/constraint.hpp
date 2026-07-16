#pragma once

#include <memory>

#include "../core/types.h"

namespace stc_SQP {
// 约束项纯虚接口：SQP 引擎通过该接口统一访问各类约束。
// 约束形式统一为 g(x, u, p) <= 0；p 为每步通用参数，由调用方显式传入，
// 约束对象内部不再持有运行时可变参数，从而消除跨调用隐藏状态与线程安全隐患。
class Constraint {
public:
    virtual ~Constraint() = default;
    // 约束维度
    virtual int ng() const = 0;
    // 施加约束 g(x, u, p) <= 0；p 维度为 0 表示本步无参数
    virtual void evaluate(const Vector& x, const Vector& u, const Vector& p, Vector& g) const = 0;
    // 计算 Jacobian：Cx = dg/dx, Cu = dg/du（p 作为已知参数，不进入求导）
    virtual void jacobian(const Vector& x, const Vector& u, const Vector& p, Matrix& Cx,
        Matrix& Cu) const = 0;
    // 一次调用同时得到约束值与 Jacobian；默认实现为 evaluate + jacobian
    virtual void evaluateAndJacobian(const Vector& x, const Vector& u, const Vector& p, Vector& g,
        Matrix& Cx, Matrix& Cu) const
    {
        evaluate(x, u, p, g);
        jacobian(x, u, p, Cx, Cu);
    }
    // 是否支持 HPIPM 软约束（默认不支持）
    virtual bool supportsSlack() const { return false; }
    // 创建独立副本，供多线程并行 linearize 使用。
    // 含非拥有引用或 CasADi 工作区等可变内部状态的子类必须深拷贝对应状态，
    // 保证副本在不同线程中独立求值。
    virtual std::shared_ptr<Constraint> clone() const = 0;
};
} // namespace stc_SQP
