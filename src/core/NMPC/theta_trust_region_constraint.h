#pragma once

#include <constraints/constraint.hpp>
#include <stdexcept>
#include <string>

namespace apa_post_processor {
// 航向信赖域约束：对每个打靶点施加 |theta - theta_ref| <= delta_theta_max。
class ThetaTrustRegionConstraint : public stc_SQP::Constraint {
   public:
    // 使用最大允许航向偏差构造约束，delta_theta_max >= 0。
    explicit ThetaTrustRegionConstraint(double delta_theta_max);
    // 约束维度：ng = 2
    int ng() const override;
    // 施加约束 g(x, u, p) <= 0
    void evaluate(const stc_SQP::Vector& x, const stc_SQP::Vector& u,
                  const stc_SQP::Vector& p, stc_SQP::Vector& g) const override;
    // 计算 Jacobian：Cx = dg/dx, Cu = dg/du
    void jacobian(const stc_SQP::Vector& x, const stc_SQP::Vector& u,
                  const stc_SQP::Vector& p, stc_SQP::Matrix& Cx,
                  stc_SQP::Matrix& Cu) const override;
    // 深拷贝
    std::shared_ptr<stc_SQP::Constraint> clone() const;

   protected:
    // 校验输入维度
    void validateInputs(const stc_SQP::Vector& x,
                        const stc_SQP::Vector& p) const;

   protected:
    // 最大允许航向偏差 (rad)
    double delta_theta_max_;
};
}  // namespace apa_post_processor
