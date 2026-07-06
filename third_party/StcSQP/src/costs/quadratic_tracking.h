#pragma once

#include "core/types.h"
#include "cost_term.hpp"
#include "../util/constants.h"

namespace stc_SQP {

// 二次跟踪代价：L(x, u) = 0.5 * (x - x_ref)^T Q (x - x_ref) + 0.5 * u^T R u
class QuadraticTrackingCost : public CostTerm {
public:
    // 使用参考状态、状态偏差权重矩阵、控制量权重矩阵以及可选的角度维度索引构造二次跟踪代价
    QuadraticTrackingCost(const Vector& x_ref, const Matrix& Q, const Matrix& R,
        int theta_idx = -1);
    // 计算标量代价 cost = L(x, u)
    void evaluate(const Vector& x, const Vector& u, double& cost) const override;
    // 计算梯度：q = dL/dx in R^(nx), r = dL/du in R^(nu)
    void gradient(const Vector& x, const Vector& u, Vector& q, Vector& r) const override;
    // 计算 Hessian：Q = d²L/dx² in R^(nx x nx), R = d²L/du² in R^(nu x nu), S = d²L/(du dx) in R^(nu x nx)
    void hessian(const Vector& x, const Vector& u, Matrix& Q, Matrix& R, Matrix& S) const override;
    // 返回参考状态
    const Vector& xRef() const { return x_ref_; }
    // 返回状态偏差权重矩阵
    const Matrix& Q() const { return Q_; }
    // 返回控制量权重矩阵
    const Matrix& R() const { return R_; }
    // 返回状态向量中角度维度的索引，-1 表示不存在角度状态
    int thetaIdx() const { return theta_idx_; }

protected:
    // 检查对称矩阵是否半正定
    void validatePositiveSemidefinite(const Matrix& M, const char* name) const;
    // 检查运行期入参维度是否与构造期一致
    void validateInputDimensions(const Vector& x, const Vector& u) const;
    // 计算状态误差；若存在 theta_idx，对角度使用 SO2 差值并规范化到 (-π, π]
    Vector computeStateError(const Vector& x) const;

protected:
    // 参考状态
    Vector x_ref_;
    // 状态偏差权重矩阵
    Matrix Q_;
    // 控制量权重矩阵
    Matrix R_;
    // 状态向量中角度维度的索引，-1 表示不存在角度状态
    int theta_idx_ = -1;
};
} // namespace stc_SQP
