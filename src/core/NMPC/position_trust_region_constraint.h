#pragma once

#include <constraints/constraint.hpp>
#include <stdexcept>
#include <string>

namespace apa_post_processor {
// 位置信赖域约束：对每个打靶点施加 |x-x_ref|<=delta_xy_max,
// |y-y_ref|<=delta_xy_max。
class PositionTrustRegionConstraint : public stc_SQP::Constraint {
   public:
    // 使用最大允许位置偏差构造约束，delta_xy_max >= 0。
    explicit PositionTrustRegionConstraint(double delta_xy_max);
    // 约束维度：ng = 4
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
    void validateInputs(const stc_SQP::Vector& x,
                        const stc_SQP::Vector& p) const;

   protected:
    double delta_xy_max_ = 0.0;
};
}  // namespace apa_post_processor
