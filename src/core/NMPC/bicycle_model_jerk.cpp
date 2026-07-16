#include "bicycle_model_jerk.h"

#include <stdexcept>

namespace apa_post_processor {
BicycleModelJerk::BicycleModelJerk(double wheelbase) : wheelbase_(wheelbase) {
    if (wheelbase_ <= 0.0) {
        throw std::invalid_argument(
            "BicycleModelJerk: wheelbase must be greater than 0");
    }
}

void BicycleModelJerk::evaluate(const stc_SQP::Vector& x,
                                const stc_SQP::Vector& u,
                                stc_SQP::Vector& x_dot) const {
    // x = [x_pos, y_pos, theta, v, delta, a, ddelta]
    // u = [jerk, ddelta_dot]
    x_dot.resize(NX);
    const double theta = x(IDX_THETA), v = x(IDX_V), delta = x(IDX_DELTA),
                 a = x(IDX_A), ddelta = x(IDX_DDELTA);
    const double jerk = u(IDX_JERK), ddelta_dot = u(IDX_DDELTA_DOT);
    const double c = std::cos(theta), s = std::sin(theta);
    x_dot(IDX_X) = v * c;
    x_dot(IDX_Y) = v * s;
    x_dot(IDX_THETA) = v * std::tan(delta) / wheelbase_;
    x_dot(IDX_V) = a;
    x_dot(IDX_DELTA) = ddelta;
    x_dot(IDX_A) = jerk;
    x_dot(IDX_DDELTA) = ddelta_dot;
}

void BicycleModelJerk::computeContinuousJacobians(const stc_SQP::Vector& x,
                                                  stc_SQP::Matrix& A_c,
                                                  stc_SQP::Matrix& B_c) const {
    // 连续雅可比：与 BicycleModelDelta 非零项一致，新增两条积分链
    A_c.setZero(NX, NX);
    B_c.setZero(NX, NU);
    const double theta = x(IDX_THETA), v = x(IDX_V), delta = x(IDX_DELTA);
    const double c = std::cos(theta), s = std::sin(theta);
    const double cos_delta = std::cos(delta),
                 sec_delta_sq = 1.0 / (cos_delta * cos_delta);
    A_c(IDX_X, IDX_THETA) = -v * s;
    A_c(IDX_X, IDX_V) = c;
    A_c(IDX_Y, IDX_THETA) = v * c;
    A_c(IDX_Y, IDX_V) = s;
    A_c(IDX_THETA, IDX_V) = std::tan(delta) / wheelbase_;
    A_c(IDX_THETA, IDX_DELTA) = v * sec_delta_sq / wheelbase_;
    A_c(IDX_V, IDX_A) = 1.0;
    A_c(IDX_DELTA, IDX_DDELTA) = 1.0;
    B_c(IDX_A, IDX_JERK) = 1.0;
    B_c(IDX_DDELTA, IDX_DDELTA_DOT) = 1.0;
}

void BicycleModelJerk::discretizeAndLinearize(const stc_SQP::Vector& x,
                                              const stc_SQP::Vector& u,
                                              double dt, double v_sign,
                                              stc_SQP::Vector& x_next,
                                              stc_SQP::Matrix& A,
                                              stc_SQP::Matrix& B) const {
    (void)v_sign;
    // RK4 + 变分方程同步积分
    x_next.resize(NX);
    A.setZero(NX, NX);
    B.setZero(NX, NU);
    stc_SQP::Matrix Phi = stc_SQP::Matrix::Identity(NX, NX);
    stc_SQP::Matrix Psi = stc_SQP::Matrix::Zero(NX, NU);
    stc_SQP::Matrix A_c(NX, NX);
    stc_SQP::Matrix B_c(NX, NU);
    stc_SQP::Vector k1_x(NX), k2_x(NX), k3_x(NX), k4_x(NX);
    stc_SQP::Matrix k1_Phi(NX, NX), k2_Phi(NX, NX), k3_Phi(NX, NX),
        k4_Phi(NX, NX);
    stc_SQP::Matrix k1_Psi(NX, NU), k2_Psi(NX, NU), k3_Psi(NX, NU),
        k4_Psi(NX, NU);
    // 阶段 1: t=0
    evaluate(x, u, k1_x);
    computeContinuousJacobians(x, A_c, B_c);
    k1_Phi = A_c * Phi;
    k1_Psi = A_c * Psi + B_c;
    // 阶段 2: t=dt/2
    stc_SQP::Vector x_stage = x + 0.5 * dt * k1_x;
    evaluate(x_stage, u, k2_x);
    computeContinuousJacobians(x_stage, A_c, B_c);
    stc_SQP::Matrix Phi_stage = Phi + 0.5 * dt * k1_Phi;
    stc_SQP::Matrix Psi_stage = Psi + 0.5 * dt * k1_Psi;
    k2_Phi = A_c * Phi_stage;
    k2_Psi = A_c * Psi_stage + B_c;
    // 阶段 3: t=dt/2 (使用 k2 斜率)
    x_stage = x + 0.5 * dt * k2_x;
    evaluate(x_stage, u, k3_x);
    computeContinuousJacobians(x_stage, A_c, B_c);
    Phi_stage = Phi + 0.5 * dt * k2_Phi;
    Psi_stage = Psi + 0.5 * dt * k2_Psi;
    k3_Phi = A_c * Phi_stage;
    k3_Psi = A_c * Psi_stage + B_c;
    // 阶段 4: t=dt
    x_stage = x + dt * k3_x;
    evaluate(x_stage, u, k4_x);
    computeContinuousJacobians(x_stage, A_c, B_c);
    Phi_stage = Phi + dt * k3_Phi;
    Psi_stage = Psi + dt * k3_Psi;
    k4_Phi = A_c * Phi_stage;
    k4_Psi = A_c * Psi_stage + B_c;
    // RK4 组合
    x_next = x + dt / 6.0 * (k1_x + 2.0 * k2_x + 2.0 * k3_x + k4_x);
    Phi = Phi + dt / 6.0 * (k1_Phi + 2.0 * k2_Phi + 2.0 * k3_Phi + k4_Phi);
    Psi = Psi + dt / 6.0 * (k1_Psi + 2.0 * k2_Psi + 2.0 * k3_Psi + k4_Psi);
    A = Phi;
    B = Psi;
    // theta SO2 规范化
    x_next(IDX_THETA) = stc_SQP::math_util::NormalizeAngle(x_next(IDX_THETA));
}

void BicycleModelJerk::retract(const stc_SQP::Vector& x, double alpha,
                               const stc_SQP::Vector& delta,
                               stc_SQP::Vector& x_new) const {
    // retract: 先对 theta 做规范化再叠加，避免别名时叠加两次
    const double theta_new =
        stc_SQP::so2::Retract(x(IDX_THETA), alpha * delta(IDX_THETA));
    x_new = x + alpha * delta;
    x_new(IDX_THETA) = theta_new;
}
}  // namespace apa_post_processor
