#pragma once

#include "constraint.hpp"
#include "../core/types.h"

namespace stc_SQP {
// 盒式约束：将状态与控制量的上下界统一表示为一般形式的不等式约束 g(x, u) <= 0
// g = [x_min - x; x - x_max; u_min - u; u - u_max]，维度 ng = 2 * nx + 2 * nu
class BoxConstraint : public Constraint {
public:
    // 使用状态与控制量的上下界构造盒式约束
    BoxConstraint(const Vector& x_min, const Vector& x_max,
        const Vector& u_min, const Vector& u_max);
    // 约束维度
    int ng() const override;
    // 施加约束 g(x, u, p) <= 0；BoxConstraint 不消费 p
    void evaluate(const Vector& x, const Vector& u, const Vector& p, Vector& g) const override;
    // 计算 Jacobian：Cx = dg/dx, Cu = dg/du
    void jacobian(const Vector& x, const Vector& u, const Vector& p, Matrix& Cx,
        Matrix& Cu) const override;
    std::shared_ptr<Constraint> clone() const;
    // 返回状态约束的下界
    const Vector& xMin() const { return x_min_; }
    // 返回状态约束的上界
    const Vector& xMax() const { return x_max_; }
    // 返回控制量约束的下界
    const Vector& uMin() const { return u_min_; }
    // 返回控制量约束的上界
    const Vector& uMax() const { return u_max_; }

protected:
    // 检查运行期入参维度是否与构造期一致
    void validateInputDimensions(const Vector& x, const Vector& u) const;

protected:
    // 状态约束下界
    Vector x_min_;
    // 状态约束上界
    Vector x_max_;
    // 控制量约束下界
    Vector u_min_;
    // 控制量约束上界
    Vector u_max_;
};
} // namespace stc_SQP
