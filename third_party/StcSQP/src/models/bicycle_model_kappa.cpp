#include "bicycle_model_kappa.h"

namespace stc_SQP {
void BicycleModelKappa::evaluate(const Vector& x, const Vector& u, Vector& x_dot) const
{
    // x_dot = [v*cos(theta), v*sin(theta), v*kappa, a, kappa_dot]
    x_dot.resize(NX);
    const double theta = x(IDX_THETA), v = x(IDX_V), kappa = x(IDX_KAPPA),
                 a = u(IDX_A), kappa_dot = u(IDX_KAPPA_DOT);
    const double c = std::cos(theta), s = std::sin(theta);
    x_dot(IDX_X) = v * c;
    x_dot(IDX_Y) = v * s;
    x_dot(IDX_THETA) = v * kappa;
    x_dot(IDX_V) = a;
    x_dot(IDX_KAPPA) = kappa_dot;
}

void BicycleModelKappa::computeContinuousJacobians(const Vector& x, Matrix& A_c,
    Matrix& B_c) const
{
    A_c.setZero(NX, NX);
    B_c.setZero(NX, NU);
    const double theta = x(IDX_THETA), v = x(IDX_V), kappa = x(IDX_KAPPA);
    const double c = std::cos(theta), s = std::sin(theta);
    A_c(IDX_X, IDX_THETA) = -v * s;
    A_c(IDX_X, IDX_V) = c;
    A_c(IDX_Y, IDX_THETA) = v * c;
    A_c(IDX_Y, IDX_V) = s;
    A_c(IDX_THETA, IDX_V) = kappa;
    A_c(IDX_THETA, IDX_KAPPA) = v;
    B_c(IDX_V, IDX_A) = 1.0;
    B_c(IDX_KAPPA, IDX_KAPPA_DOT) = 1.0;
}

void BicycleModelKappa::discretizeAndLinearize(const Vector& x, const Vector& u, double dt,
    double v_sign, Vector& x_next, Matrix& A,
    Matrix& B) const
{
    (void)v_sign;
    // 数学背景：
    //   设 Phi(t) = dx(t)/dx(0), Psi(t) = dx(t)/du 为状态/输入灵敏度矩阵，
    //   则它们满足变分 ODE：
    //     Phi_dot = A_c(x(t), u) * Phi,   Phi(0) = I
    //     Psi_dot = A_c(x(t), u) * Psi + B_c(x(t), u),   Psi(0) = 0
    //   将状态 x 与 [Phi, Psi] 组成增广系统，用同一组 RK4 阶段同时积分，
    //   得到离散状态 x_next 与离散 Jacobian A = Phi(dt), B = Psi(dt)。
    //
    // 实现细节：
    //   每一 RK4 阶段先计算该阶段状态 x_stage，再计算该点处的 A_c/B_c，
    //   最后计算该阶段 Phi/Psi 的斜率 k*_Phi/k*_Psi。这样可保持与状态传播
    //   相同的非线性轨迹，提高灵敏度精度。
    x_next.resize(NX);
    A.setZero(NX, NX);
    B.setZero(NX, NU);
    // 灵敏度矩阵初值
    Matrix Phi = Matrix::Identity(NX, NX);
    Matrix Psi = Matrix::Zero(NX, NU);
    Matrix A_c(NX, NX);
    Matrix B_c(NX, NU);
    // RK4 阶段的状态斜率
    Vector k1_x(NX), k2_x(NX), k3_x(NX), k4_x(NX);
    // RK4 阶段的灵敏度斜率
    Matrix k1_Phi(NX, NX), k2_Phi(NX, NX), k3_Phi(NX, NX), k4_Phi(NX, NX);
    Matrix k1_Psi(NX, NU), k2_Psi(NX, NU), k3_Psi(NX, NU), k4_Psi(NX, NU);
    // 阶段 1：在 t=0 处计算
    evaluate(x, u, k1_x);
    computeContinuousJacobians(x, A_c, B_c);
    k1_Phi = A_c * Phi;
    k1_Psi = A_c * Psi + B_c;
    // 阶段 2：在 t=dt/2 处计算
    Vector x_stage = x + 0.5 * dt * k1_x;
    evaluate(x_stage, u, k2_x);
    computeContinuousJacobians(x_stage, A_c, B_c);
    Matrix Phi_stage = Phi + 0.5 * dt * k1_Phi;
    Matrix Psi_stage = Psi + 0.5 * dt * k1_Psi;
    k2_Phi = A_c * Phi_stage;
    k2_Psi = A_c * Psi_stage + B_c;
    // 阶段 3：在 t=dt/2 处计算（使用 k2 斜率）
    x_stage = x + 0.5 * dt * k2_x;
    evaluate(x_stage, u, k3_x);
    computeContinuousJacobians(x_stage, A_c, B_c);
    Phi_stage = Phi + 0.5 * dt * k2_Phi;
    Psi_stage = Psi + 0.5 * dt * k2_Psi;
    k3_Phi = A_c * Phi_stage;
    k3_Psi = A_c * Psi_stage + B_c;
    // 阶段 4：在 t=dt 处计算
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
    // theta 作为 SO2 量，在离散传播后保持规范化，便于后续 retract 与约束评估
    x_next(IDX_THETA) = math_util::NormalizeAngle(x_next(IDX_THETA));
}

void BicycleModelKappa::retract(const Vector& x, double alpha, const Vector& delta,
    Vector& x_new) const
{
    // 先基于原始 x 计算规范化后的新 theta，再执行欧氏相加，避免 x_new 与 x 别名时
    // 先相加修改了 theta 又用它去计算 retract，导致角度被叠加两次。
    const double theta_new = so2::Retract(x(IDX_THETA), alpha * delta(IDX_THETA));
    x_new = x + alpha * delta;
    x_new(IDX_THETA) = theta_new;
}
} // namespace stc_SQP
