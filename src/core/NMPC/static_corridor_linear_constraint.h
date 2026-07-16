#pragma once

#include <constraints/constraint.hpp>
#include <stdexcept>
#include <string>

namespace apa_post_processor {
// 静态舒适走廊线性不等式约束：注入 C_matrix*Z <= d_vector 作为 HPIPM 软约束。
class StaticCorridorLinearConstraint : public stc_SQP::Constraint {
   public:
    // 使用全局 C、d 矩阵及段参数构造。
    StaticCorridorLinearConstraint(const stc_SQP::Matrix& C,
                                   const stc_SQP::Vector& d,
                                   int global_start_idx,
                                   int constraints_per_step, int segment_steps,
                                   bool skip_last_step = false);
    int ng() const override;
    void evaluate(const stc_SQP::Vector& x, const stc_SQP::Vector& u,
                  const stc_SQP::Vector& p, stc_SQP::Vector& g) const override;
    void jacobian(const stc_SQP::Vector& x, const stc_SQP::Vector& u,
                  const stc_SQP::Vector& p, stc_SQP::Matrix& Cx,
                  stc_SQP::Matrix& Cu) const override;
    std::shared_ptr<stc_SQP::Constraint> clone() const;

   protected:
    // 校验步索引在有效范围内。
    void validateStepIndex(const stc_SQP::Vector& p, int local_step_idx) const;

   protected:
    stc_SQP::Matrix C_;
    stc_SQP::Vector d_;
    int global_start_idx_;
    int constraints_per_step_;
    int segment_steps_;
    bool skip_last_step_;
};
}  // namespace apa_post_processor
