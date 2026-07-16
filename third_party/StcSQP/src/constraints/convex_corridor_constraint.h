#pragma once

#include "../core/types.h"
#include "../generated/corridor.h"
#include "../models/casadi_wrapper.h"
#include "constraint.hpp"

namespace stc_SQP {
// 凸走廊约束：将车辆四个角点约束在一组由参数 p 描述的凸半空间交集内。
// 约束形式：g(x, u, p) <= 0，其中 g 的维度为 CORRIDOR_G_DIM（4 角点 × 10 半空间）。
// 注意：本类实例持有 CasADi 工作区不是线程安全的；OpenMP 等多线程场景必须先调用 clone() 获取独立副本。
class ConvexCorridorConstraint : public Constraint {
public:
    // 使用控制维度 nu 构造约束；参数 p 在 evaluate/jacobian 时显式传入。
    explicit ConvexCorridorConstraint(int nu);
    // 保留带 p 的构造函数以兼容既有调用点，但 p 不再作为成员持有。
    ConvexCorridorConstraint(const Vector& p, int nu);
    // 约束维度，固定为 CORRIDOR_G_DIM
    int ng() const override;
    // 计算约束值 g(x, u, p) <= 0
    void evaluate(const Vector& x, const Vector& u, const Vector& p, Vector& g) const override;
    // 计算 Jacobian：Cx = dg/dx，Cu = dg/du（Cu 恒为零）
    void jacobian(const Vector& x, const Vector& u, const Vector& p, Matrix& Cx,
        Matrix& Cu) const override;
    // 一次调用同时得到约束值与 Jacobian，避免重复执行 CasADi 函数
    void evaluateAndJacobian(const Vector& x, const Vector& u, const Vector& p, Vector& g,
        Matrix& Cx, Matrix& Cu) const override;
    // 返回控制维度
    int nu() const { return nu_; }
    std::shared_ptr<Constraint> clone() const;

protected:
    // 检查输入 x/u 的维度是否符合预期
    void validateInputDimensions(const Vector& x, const Vector& u) const;
    // 检查参数 p 维度
    static void validateParameters(const Vector& p);
    // 复用的底层求值：cx_buffer 长度至少为 CORRIDOR_CX_NNZ
    void doEvaluate(const Vector& x, const Vector& p, Vector& g, double* cx_buffer) const;

protected:
    // 控制维度（用于运行期维度校验）
    int nu_ = 0;
    // CasADi 生成函数的轻量包装器，构造时预分配 iw_/w_ 工作区
    casadi::CasADiFunction func_;
    // 预分配 scratch 缓冲区，避免每次 evaluate 动态分配
    mutable Vector cx_dummy_;
    mutable Vector g_tmp_;
    mutable std::vector<const double*> arg_scratch_;
    mutable std::vector<double*> res_scratch_;
};
} // namespace stc_SQP
