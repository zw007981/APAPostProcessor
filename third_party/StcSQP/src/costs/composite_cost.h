#pragma once

#include <memory>
#include <vector>

#include "cost_term.hpp"

namespace stc_SQP {
// 组合代价：把多个CostTerm按线性叠加组合成一个CostTerm，用于同一个StageSegment需要
// 同时施加多种代价（如：跟踪代价 + ESDF碰撞惩罚代价）的场景——因为StageSegment每步
// 只能挂载一个CostTerm，无法直接叠加多个。
class CompositeCost : public CostTerm {
public:
    // terms：参与组合的子代价列表，不能为空，元素不能为空指针
    explicit CompositeCost(std::vector<std::shared_ptr<CostTerm>> terms);
    // 计算标量代价：各子代价之和
    void evaluate(const Vector& x, const Vector& u, double& cost) const override;
    // 计算梯度：各子代价梯度之和
    void gradient(const Vector& x, const Vector& u, Vector& q, Vector& r) const override;
    // 计算Hessian：各子代价Hessian之和
    void hessian(const Vector& x, const Vector& u, Matrix& Q, Matrix& R, Matrix& S) const override;

protected:
    // 参与组合的子代价列表
    std::vector<std::shared_ptr<CostTerm>> terms_;
};
} // namespace stc_SQP
