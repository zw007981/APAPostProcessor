#pragma once

#include <memory>
#include <vector>

#include "cost_term.hpp"

namespace stc_SQP {
// 组合代价：把多个 CostTerm 按线性叠加组合成一个 CostTerm。
// 用于需要将若干代价打包为单一 shared_ptr<CostTerm> 的场景
// （如 StageSegment::cost 需要同时包含 QuadraticTrackingCost 与 ESDF 软惩罚）。
class CompositeCost : public CostTerm {
public:
    // terms：参与组合的子代价列表，不能为空，元素不能为空指针
    explicit CompositeCost(std::vector<std::shared_ptr<CostTerm>> terms);
    // 添加一个子代价；允许在构造后追加，元素不能为空指针
    void addTerm(std::shared_ptr<CostTerm> term);
    // 计算标量代价：各子代价之和
    void evaluate(const Vector& x, const Vector& u, double& cost) const override;
    // 计算梯度：各子代价梯度之和
    void gradient(const Vector& x, const Vector& u, Vector& q, Vector& r) const override;
    // 计算Hessian：各子代价Hessian之和
    void hessian(const Vector& x, const Vector& u, Matrix& Q, Matrix& R, Matrix& S) const override;
    // 组合求值：依次调用各子代价的 evaluateGradientAndHessian（若子代价已覆写则复用其内部缓存）
    void evaluateGradientAndHessian(const Vector& x, const Vector& u, double& cost,
        Vector& q, Vector& r, Matrix& Q, Matrix& R, Matrix& S) const override;
    // 创建副本；对每个子代价调用 clone()，保证多线程场景下各线程持有独立实例
    std::shared_ptr<CostTerm> clone() const;

protected:
    // 参与组合的子代价列表
    std::vector<std::shared_ptr<CostTerm>> terms_;
};
} // namespace stc_SQP
