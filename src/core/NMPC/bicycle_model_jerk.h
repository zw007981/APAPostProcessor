#pragma once

#include <models/dynamical_system.h>

#include <math/math_util.hpp>
#include <math/so2.hpp>

namespace apa_post_processor {
// 控制量升阶版阿克曼自行车模型：a/delta_dot 升级为状态，jerk/ddelta_dot
// 为控制量。
class BicycleModelJerk : public stc_SQP::DynamicalSystem {
   protected:
    // 状态维度：x, y, theta, v, delta, a, ddelta
    static constexpr int NX = 7;
    // 控制维度：jerk, ddelta_dot
    static constexpr int NU = 2;
    // x 坐标索引
    static constexpr int IDX_X = 0;
    // y 坐标索引
    static constexpr int IDX_Y = 1;
    // 航向角索引（SO2 流形）
    static constexpr int IDX_THETA = 2;
    // 纵向速度索引
    static constexpr int IDX_V = 3;
    // 前轮转角索引
    static constexpr int IDX_DELTA = 4;
    // 纵向加速度状态索引
    static constexpr int IDX_A = 5;
    // 前轮转角变化率状态索引
    static constexpr int IDX_DDELTA = 6;
    // 纵向加加速度（jerk）控制索引
    static constexpr int IDX_JERK = 0;
    // 前轮转角加速度控制索引
    static constexpr int IDX_DDELTA_DOT = 1;

   public:
    // 构造函数：wheelbase 为车辆轴距，必须大于 0。
    explicit BicycleModelJerk(double wheelbase = 2.8);
    ~BicycleModelJerk() override = default;
    // 状态维度 nx = 7
    int nx() const override { return NX; }
    // 控制维度 nu = 2
    int nu() const override { return NU; }
    // 连续动力学 f(x, u)：输出 x_dot，维度为 nx
    void evaluate(const stc_SQP::Vector& x, const stc_SQP::Vector& u,
                  stc_SQP::Vector& x_dot) const override;
    // 离散化并线性化。
    void discretizeAndLinearize(const stc_SQP::Vector& x,
                                const stc_SQP::Vector& u, double dt,
                                double v_sign, stc_SQP::Vector& x_next,
                                stc_SQP::Matrix& A,
                                stc_SQP::Matrix& B) const override;
    // 流形更新：x_new = x ⊕ (alpha * delta)
    void retract(const stc_SQP::Vector& x, double alpha,
                 const stc_SQP::Vector& delta,
                 stc_SQP::Vector& x_new) const override;

   protected:
    // 连续时间 Jacobian，供 RK4 变分方程积分使用。
    void computeContinuousJacobians(const stc_SQP::Vector& x,
                                    stc_SQP::Matrix& A_c,
                                    stc_SQP::Matrix& B_c) const;

   protected:
    double wheelbase_ = 2.8;
};
}  // namespace apa_post_processor
