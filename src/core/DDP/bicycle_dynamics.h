#pragma once

#include <array>

#include "ddp_reference_builder.h"

namespace apa_post_processor {
// u = [j, η]（纵向跃度、前轮转角加加速度）
inline constexpr int DDP_IDX_JERK = 0;
inline constexpr int DDP_IDX_ETA = 1;
using DdpStateJacobian = Eigen::Matrix<double, DDP_STATE_DIM, DDP_STATE_DIM>;
using DdpControlJacobian =
    Eigen::Matrix<double, DDP_STATE_DIM, DDP_CONTROL_DIM>;
// 七维自行车动力学模型：半隐式 Euler 积分 +
// 解析雅可比，位移用中点朝向角保证旋转精度
class BicycleDynamics {
   public:
    // 构造时校验轴距必须为正
    explicit BicycleDynamics(double wheelbase);
    // 半隐式 Euler，位移用中点朝向角 θ_mid（无偏、二阶旋转精度）
    DdpState step(const DdpState& x, const DdpControl& u, double dt) const;
    // 解析雅可比 A/B，求导链阶与动力学更新顺序严格一致
    void jacobians(const DdpState& x, const DdpControl& u, double dt,
                   DdpStateJacobian* A, DdpControlJacobian* B) const;

   protected:
    // step/jacobians 共享同一份中间量，保证导数与动力学严格一致
    struct ChainValues {
        // a⁺ = a + j·dt
        double a_plus{0.0};
        // ω⁺ = ω + η·dt
        double omega_plus{0.0};
        // v⁺ = v + a⁺·dt
        double v_plus{0.0};
        // δ⁺ = δ + ω⁺·dt
        double delta_plus{0.0};
        // tan(δ⁺)
        double tan_delta{0.0};
        // sec²(δ⁺)
        double sec2_delta{0.0};
        // Δθ
        double dtheta{0.0};
        // 中点朝向 θ+Δθ/2
        double theta_mid{0.0};
        // cos(θ_mid)
        double cos_mid{1.0};
        // sin(θ_mid)
        double sin_mid{0.0};
    };
    static ChainValues EvaluateChain(const DdpState& x, const DdpControl& u,
                                     double dt, double wheelbase);

   protected:
    // 轴距 (m)
    double wheelbase_;
};
}  // namespace apa_post_processor
