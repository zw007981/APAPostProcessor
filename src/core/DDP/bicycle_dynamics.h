#pragma once

#include <array>

#include "ddp_reference_builder.h"

namespace apa_post_processor {
// 控制分量布局索引：u = [j, η]（纵向跃度、前轮转角加加速度）
inline constexpr int DDP_IDX_JERK = 0;
inline constexpr int DDP_IDX_ETA = 1;
// 状态/控制雅可比定长类型：栈上分配，热路径严禁堆分配
using DdpStateJacobian = Eigen::Matrix<double, DDP_STATE_DIM, DDP_STATE_DIM>;
using DdpControlJacobian =
    Eigen::Matrix<double, DDP_STATE_DIM, DDP_CONTROL_DIM>;
// 完整二阶动力学张量：逐输出行存储——f_xx[r] 为 ∂²f_r/∂x²（7×7）、
// f_uu[r] 为 ∂²f_r/∂u²（2×2）、f_ux[r] 为 ∂²f_r/(∂u∂x)（2×7）；
// v⁺/a⁺/δ⁺/ω⁺ 四行对 (x,u) 线性，切片恒为零矩阵
using DdpStateHessianTensor = std::array<DdpStateJacobian, DDP_STATE_DIM>;
using DdpControlHessianTensor =
    std::array<Eigen::Matrix<double, DDP_CONTROL_DIM, DDP_CONTROL_DIM>,
               DDP_STATE_DIM>;
using DdpMixedHessianTensor =
    std::array<Eigen::Matrix<double, DDP_CONTROL_DIM, DDP_STATE_DIM>,
               DDP_STATE_DIM>;
// 完整二阶动力学项（true MS-DDP）的编译期开关状态：默认关闭；开启方式为编译时
// 定义宏 DDP_ENABLE_FULL_HESSIAN。张量求值能力本身始终可用并经有限差分全切片
// 验证，开关只决定内层求解器回推时是否补回 s'·f_xx 类张量收缩项
inline constexpr bool DDP_FULL_HESSIAN_ENABLED =
#ifdef DDP_ENABLE_FULL_HESSIAN
    true;
#else
    false;
#endif
class BicycleDynamics {
   public:
    // 构造：轴距必须为正（δ⁺→θ 运动学反解的唯一车辆参数依赖）
    explicit BicycleDynamics(double wheelbase);
    // 单步离散动力学：半隐式 Euler 链式更新（先 a⁺/ω⁺，再 v⁺/δ⁺，再 Δθ/θ⁺），
    // 位移更新用中点朝向角 θ_mid=θ+Δθ/2（无偏、二阶旋转精度）；
    // 调用方必须保证 dt>0——dt=0 时产出恒等映射、dt<0 时按负步长反向积分，
    // 均非报错路径但语义不正确（dt 校验放在配置/调用层，热路径不重复检查）
    DdpState step(const DdpState& x, const DdpControl& u, double dt) const;
    // 解析雅可比 A=∂f/∂x、B=∂f/∂u：求导链阶与动力学更新顺序严格一致，
    // 含中点角引入的全部非零元（g₁/g₂ 辅助量），输出指针必须非空
    void jacobians(const DdpState& x, const DdpControl& u, double dt,
                   DdpStateJacobian* A, DdpControlJacobian* B) const;
    // 完整二阶张量 f_xx/f_uu/f_ux：供 true MS-DDP 回推的 s'·f_xx 张量收缩使用，
    // 非零元仅出现在 x⁺/y⁺/θ⁺ 三行，输出指针必须非空
    void hessians(const DdpState& x, const DdpControl& u, double dt,
                  DdpStateHessianTensor* f_xx, DdpControlHessianTensor* f_uu,
                  DdpMixedHessianTensor* f_ux) const;

   protected:
    // 链式中间量：step/jacobians/hessians
    // 共享同一份数值，保证导数与动力学严格一致
    struct ChainValues {
        double a_plus{0.0};
        double omega_plus{0.0};
        double v_plus{0.0};
        double delta_plus{0.0};
        double tan_delta{0.0};
        double sec2_delta{0.0};
        double dtheta{0.0};
        double theta_mid{0.0};
        double cos_mid{1.0};
        double sin_mid{0.0};
    };
    static ChainValues EvaluateChain(const DdpState& x, const DdpControl& u,
                                     double dt, double wheelbase);

   protected:
    // 轴距 (m)
    double wheelbase_;
};
}  // namespace apa_post_processor
