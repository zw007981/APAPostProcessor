#include "bicycle_dynamics.h"

#include <cmath>
#include <stdexcept>

namespace apa_post_processor {
BicycleDynamics::BicycleDynamics(double wheelbase) : wheelbase_(wheelbase) {
    if (!(wheelbase_ > EPSILON)) {
        throw std::invalid_argument(
            "BicycleDynamics: wheelbase must be positive!!!");
    }
}

BicycleDynamics::ChainValues BicycleDynamics::EvaluateChain(const iLQRState& x,
                                                            const iLQRControl& u,
                                                            double dt,
                                                            double wheelbase) {
    ChainValues chain;
    // 半隐式 Euler 链式更新：先积分链顶端 a⁺、ω⁺，再逐级代入 v⁺、δ⁺
    chain.a_plus = x(ILQR_IDX_A) + u(ILQR_IDX_JERK) * dt;
    chain.omega_plus = x(ILQR_IDX_OMEGA) + u(ILQR_IDX_ETA) * dt;
    chain.v_plus = x(ILQR_IDX_V) + chain.a_plus * dt;
    chain.delta_plus = x(ILQR_IDX_DELTA) + chain.omega_plus * dt;
    // 朝向增量与中点朝向角：位移更新用 θ_mid 而非旧 θ，消除曲线运动的
    // 带符号累积截断偏置（轨迹向外侧螺旋漂移）
    chain.tan_delta = std::tan(chain.delta_plus);
    chain.sec2_delta = 1.0 + chain.tan_delta * chain.tan_delta;
    chain.dtheta = chain.v_plus * chain.tan_delta * dt / wheelbase;
    chain.theta_mid = x(ILQR_IDX_THETA) + chain.dtheta * 0.5;
    chain.cos_mid = std::cos(chain.theta_mid);
    chain.sin_mid = std::sin(chain.theta_mid);
    return chain;
}

iLQRState BicycleDynamics::step(const iLQRState& x, const iLQRControl& u,
                               double dt) const {
    const ChainValues chain = EvaluateChain(x, u, dt, wheelbase_);
    iLQRState x_next;
    x_next << x(ILQR_IDX_X) + chain.v_plus * chain.cos_mid * dt,
        x(ILQR_IDX_Y) + chain.v_plus * chain.sin_mid * dt,
        x(ILQR_IDX_THETA) + chain.dtheta, chain.v_plus, chain.a_plus,
        chain.delta_plus, chain.omega_plus;
    return x_next;
}

void BicycleDynamics::jacobians(const iLQRState& x, const iLQRControl& u,
                                double dt, iLQRStateJacobian* A,
                                iLQRControlJacobian* B) const {
    const ChainValues chain = EvaluateChain(x, u, dt, wheelbase_);
    // 中点角对链上变量的标量偏导：g₁=∂θ_mid/∂v、g₂=∂θ_mid/∂δ；
    // 链阶规律——a↔g₁·dt、j↔g₁·dt²、ω↔g₂·dt、η↔g₂·dt²
    const double g1 = chain.tan_delta / (2.0 * wheelbase_) * dt;
    const double g2 = chain.v_plus * chain.sec2_delta / (2.0 * wheelbase_) * dt;
    const double tan_over_l = chain.tan_delta / wheelbase_;
    const double v_sec2_over_l = chain.v_plus * chain.sec2_delta / wheelbase_;
    const double dt2 = dt * dt;
    const double dt3 = dt2 * dt;
    A->setZero();
    B->setZero();
    // x⁺ 行：∂x⁺/∂· = c_m·∂v⁺·dt − v⁺·s_m·∂θ_mid·dt
    (*A)(ILQR_IDX_X, ILQR_IDX_X) = 1.0;
    (*A)(ILQR_IDX_X, ILQR_IDX_THETA) = -chain.v_plus * chain.sin_mid * dt;
    (*A)(ILQR_IDX_X, ILQR_IDX_V) =
        chain.cos_mid * dt - chain.v_plus * chain.sin_mid * g1 * dt;
    (*A)(ILQR_IDX_X, ILQR_IDX_A) =
        chain.cos_mid * dt2 - chain.v_plus * chain.sin_mid * g1 * dt2;
    (*A)(ILQR_IDX_X, ILQR_IDX_DELTA) = -chain.v_plus * chain.sin_mid * g2 * dt;
    (*A)(ILQR_IDX_X, ILQR_IDX_OMEGA) = -chain.v_plus * chain.sin_mid * g2 * dt2;
    (*B)(ILQR_IDX_X, ILQR_IDX_JERK) =
        chain.cos_mid * dt3 - chain.v_plus * chain.sin_mid * g1 * dt3;
    (*B)(ILQR_IDX_X, ILQR_IDX_ETA) = -chain.v_plus * chain.sin_mid * g2 * dt3;
    // y⁺ 行：∂y⁺/∂· = s_m·∂v⁺·dt + v⁺·c_m·∂θ_mid·dt
    (*A)(ILQR_IDX_Y, ILQR_IDX_Y) = 1.0;
    (*A)(ILQR_IDX_Y, ILQR_IDX_THETA) = chain.v_plus * chain.cos_mid * dt;
    (*A)(ILQR_IDX_Y, ILQR_IDX_V) =
        chain.sin_mid * dt + chain.v_plus * chain.cos_mid * g1 * dt;
    (*A)(ILQR_IDX_Y, ILQR_IDX_A) =
        chain.sin_mid * dt2 + chain.v_plus * chain.cos_mid * g1 * dt2;
    (*A)(ILQR_IDX_Y, ILQR_IDX_DELTA) = chain.v_plus * chain.cos_mid * g2 * dt;
    (*A)(ILQR_IDX_Y, ILQR_IDX_OMEGA) = chain.v_plus * chain.cos_mid * g2 * dt2;
    (*B)(ILQR_IDX_Y, ILQR_IDX_JERK) =
        chain.sin_mid * dt3 + chain.v_plus * chain.cos_mid * g1 * dt3;
    (*B)(ILQR_IDX_Y, ILQR_IDX_ETA) = chain.v_plus * chain.cos_mid * g2 * dt3;
    // θ⁺ 行：∂θ⁺/∂· = (tanδ⁺·∂v⁺ + v⁺·sec²δ⁺·∂δ⁺)·dt/L
    (*A)(ILQR_IDX_THETA, ILQR_IDX_THETA) = 1.0;
    (*A)(ILQR_IDX_THETA, ILQR_IDX_V) = tan_over_l * dt;
    (*A)(ILQR_IDX_THETA, ILQR_IDX_A) = tan_over_l * dt2;
    (*A)(ILQR_IDX_THETA, ILQR_IDX_DELTA) = v_sec2_over_l * dt;
    (*A)(ILQR_IDX_THETA, ILQR_IDX_OMEGA) = v_sec2_over_l * dt2;
    (*B)(ILQR_IDX_THETA, ILQR_IDX_JERK) = tan_over_l * dt3;
    (*B)(ILQR_IDX_THETA, ILQR_IDX_ETA) = v_sec2_over_l * dt3;
    // v⁺/a⁺/δ⁺/ω⁺ 四行：对 (x,u) 线性，链式代入的常数元
    (*A)(ILQR_IDX_V, ILQR_IDX_V) = 1.0;
    (*A)(ILQR_IDX_V, ILQR_IDX_A) = dt;
    (*B)(ILQR_IDX_V, ILQR_IDX_JERK) = dt2;
    (*A)(ILQR_IDX_A, ILQR_IDX_A) = 1.0;
    (*B)(ILQR_IDX_A, ILQR_IDX_JERK) = dt;
    (*A)(ILQR_IDX_DELTA, ILQR_IDX_DELTA) = 1.0;
    (*A)(ILQR_IDX_DELTA, ILQR_IDX_OMEGA) = dt;
    (*B)(ILQR_IDX_DELTA, ILQR_IDX_ETA) = dt2;
    (*A)(ILQR_IDX_OMEGA, ILQR_IDX_OMEGA) = 1.0;
    (*B)(ILQR_IDX_OMEGA, ILQR_IDX_ETA) = dt;
}
}  // namespace apa_post_processor
