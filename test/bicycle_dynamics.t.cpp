#include <gtest/gtest.h>

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "core/iLQR/bicycle_dynamics.h"
#include "util/constants.h"

namespace apa_post_processor {
namespace {

// 测试用固定步长与轴距：与 iLQR 参考构建的标称值一致
constexpr double kDt = 0.1;
constexpr double kWheelbase = 3.0;

// 有限差分混合判据：|解析 − 数值| ≤ rel_tol · max(|数值|, abs_floor)，
// 显著元素等价于相对误差 < 1e-6；零/微小元素的"相对误差"无定义，
// 由 abs_floor 派生的 1e-9 级绝对容差兜底
constexpr double kRelTol = 1e-6;
constexpr double kAbsFloor = 1e-3;

// 一个有代表性的运行点：直行巡航、左转加速、倒车右转、cusp 过零、大转角近边界
struct OperatingPoint {
    std::string name;
    iLQRState x;
    iLQRControl u;
};

std::vector<OperatingPoint> MakeOperatingPoints() {
    std::vector<OperatingPoint> points;
    points.reserve(5);
    iLQRState x1;
    x1 << 1.2, -0.7, 0.0, 0.5, 0.0, 0.0, 0.0;
    points.push_back({"straight_cruise", x1, iLQRControl::Zero()});
    iLQRState x2;
    x2 << -3.0, 2.5, 0.7, 0.4, 0.1, 0.3, 0.05;
    points.push_back(
        {"left_turn_accel", x2, (iLQRControl() << 0.2, 0.3).finished()});
    iLQRState x3;
    x3 << 0.4, 5.0, -2.1, -0.45, -0.05, -0.25, -0.08;
    points.push_back(
        {"reverse_right_turn", x3, (iLQRControl() << -0.15, -0.25).finished()});
    iLQRState x4;
    x4 << 0.0, 0.0, 0.2, 0.001, -0.3, 0.35, -0.1;
    points.push_back(
        {"cusp_crossing", x4, (iLQRControl() << -0.5, 0.8).finished()});
    iLQRState x5;
    x5 << -1.0, -1.0, 3.0, 0.35, 0.2, 0.5, 0.15;
    points.push_back(
        {"large_steer_near_limit", x5, (iLQRControl() << 0.4, -0.6).finished()});
    return points;
}

// 对 step() 做中心差分，得到数值雅可比 A/B（每列对一个输入分量扰动）
void NumericJacobians(const BicycleDynamics& dynamics, const iLQRState& x,
                      const iLQRControl& u, double dt, iLQRStateJacobian* num_a,
                      iLQRControlJacobian* num_b) {
    for (int i = 0; i < ILQR_STATE_DIM; ++i) {
        const double h = 1e-6 * std::max(1.0, std::abs(x(i)));
        iLQRState x_plus = x, x_minus = x;
        x_plus(i) += h;
        x_minus(i) -= h;
        num_a->col(i) =
            (dynamics.step(x_plus, u, dt) - dynamics.step(x_minus, u, dt)) /
            (2.0 * h);
    }
    for (int i = 0; i < ILQR_CONTROL_DIM; ++i) {
        const double h = 1e-6 * std::max(1.0, std::abs(u(i)));
        iLQRControl u_plus = u, u_minus = u;
        u_plus(i) += h;
        u_minus(i) -= h;
        num_b->col(i) =
            (dynamics.step(x, u_plus, dt) - dynamics.step(x, u_minus, dt)) /
            (2.0 * h);
    }
}

// 全元素对拍：逐元素应用混合判据，失败时打印元素坐标与数值
template <typename TDerivedA, typename TDerivedB>
void ExpectMatrixAllNear(const Eigen::MatrixBase<TDerivedA>& analytic,
                         const Eigen::MatrixBase<TDerivedB>& numeric,
                         const std::string& label) {
    ASSERT_EQ(analytic.rows(), numeric.rows());
    ASSERT_EQ(analytic.cols(), numeric.cols());
    for (int r = 0; r < analytic.rows(); ++r) {
        for (int c = 0; c < analytic.cols(); ++c) {
            const double tolerance =
                kRelTol * std::max(std::abs(numeric(r, c)), kAbsFloor);
            EXPECT_NEAR(analytic(r, c), numeric(r, c), tolerance)
                << label << " (" << r << ", " << c << ")";
        }
    }
}

// 测试解析雅可比与有限差分的全元素对拍（本 Milestone 的核心验收）。
// 因为漏写中点角新增非零元与写错链阶 dt 幂次是最常见的实现 bug，
// 所以必须在多个代表性运行点（含 cusp 过零与大转角）上逐元素对照，
// 而非只抽查对角块。
TEST(BicycleDynamicsTest, JacobiansMatchFiniteDifferenceAtOperatingPoints) {
    const BicycleDynamics dynamics(kWheelbase);
    for (const auto& point : MakeOperatingPoints()) {
        iLQRStateJacobian analytic_a, numeric_a;
        iLQRControlJacobian analytic_b, numeric_b;
        dynamics.jacobians(point.x, point.u, kDt, &analytic_a, &analytic_b);
        NumericJacobians(dynamics, point.x, point.u, kDt, &numeric_a,
                         &numeric_b);
        ExpectMatrixAllNear(analytic_a, numeric_a, point.name + " A");
        ExpectMatrixAllNear(analytic_b, numeric_b, point.name + " B");
    }
}

// 测试直线运动的位置更新精确性：δ=0 且 ω=0 时 θ_mid=θ 恒成立，
// 位移必须等于 v⁺·cos/sin(θ)·dt 的闭式结果（浮点舍入内）。
TEST(BicycleDynamicsTest, StraightLineMotionIsExact) {
    const BicycleDynamics dynamics(kWheelbase);
    iLQRState x;
    x << 1.0, 2.0, 0.3, 0.5, 0.1, 0.0, 0.0;
    const iLQRControl u = (iLQRControl() << 0.4, 0.0).finished();
    const iLQRState next = dynamics.step(x, u, kDt);
    const double a_plus = 0.1 + 0.4 * kDt;
    const double v_plus = 0.5 + a_plus * kDt;
    EXPECT_NEAR(next(ILQR_IDX_X), 1.0 + v_plus * std::cos(0.3) * kDt, 1e-12);
    EXPECT_NEAR(next(ILQR_IDX_Y), 2.0 + v_plus * std::sin(0.3) * kDt, 1e-12);
    EXPECT_NEAR(next(ILQR_IDX_THETA), 0.3, 1e-15);
    EXPECT_NEAR(next(ILQR_IDX_V), v_plus, 1e-15);
    EXPECT_NEAR(next(ILQR_IDX_A), a_plus, 1e-15);
    EXPECT_NEAR(next(ILQR_IDX_DELTA), 0.0, 1e-15);
    EXPECT_NEAR(next(ILQR_IDX_OMEGA), 0.0, 1e-15);
}

// 旧 θ 格式（位移更新用上时刻朝向）的对照实现：仅用于量化中点格式的优势
iLQRState StepWithOldTheta(double wheelbase, const iLQRState& x,
                          const iLQRControl& u, double dt) {
    const double a_plus = x(ILQR_IDX_A) + u(ILQR_IDX_JERK) * dt;
    const double omega_plus = x(ILQR_IDX_OMEGA) + u(ILQR_IDX_ETA) * dt;
    const double v_plus = x(ILQR_IDX_V) + a_plus * dt;
    const double delta_plus = x(ILQR_IDX_DELTA) + omega_plus * dt;
    const double dtheta = v_plus * std::tan(delta_plus) * dt / wheelbase;
    iLQRState next;
    next << x(ILQR_IDX_X) + v_plus * std::cos(x(ILQR_IDX_THETA)) * dt,
        x(ILQR_IDX_Y) + v_plus * std::sin(x(ILQR_IDX_THETA)) * dt,
        x(ILQR_IDX_THETA) + dtheta, v_plus, a_plus, delta_plus, omega_plus;
    return next;
}

// 测试定 δ 圆弧行驶一周回到起点邻域。
// 因为恒定 v/δ 下中点格式的位移方向恰好是真实弦方向，轨迹构成正多边形，
// 整周后闭合误差只来自步数对周期的整除残差（厘米级以内），
// 该用例钉住中点格式在最基本曲线场景下的数值稳定性。
TEST(BicycleDynamicsTest, ConstantSteerCircleClosesToStart) {
    const BicycleDynamics dynamics(kWheelbase);
    const double steer = 0.3;
    const double speed = 0.5;
    // 整周时长：角速率 r = v·tanδ/L，T = 2π/r
    const double rate = speed * std::tan(steer) / kWheelbase;
    const int steps = static_cast<int>(std::ceil(2.0 * PI / rate / kDt));
    iLQRState state = iLQRState::Zero();
    state(ILQR_IDX_V) = speed;
    state(ILQR_IDX_DELTA) = steer;
    const iLQRControl u = iLQRControl::Zero();
    for (int i = 0; i < steps; ++i) {
        state = dynamics.step(state, u, kDt);
    }
    const double closure_error = std::hypot(state(ILQR_IDX_X), state(ILQR_IDX_Y));
    std::cout << "[CIRCLE] steps=" << steps
              << " closure_error=" << closure_error << std::endl;
    EXPECT_LT(closure_error, 0.05);
}

// 精确圆弧积分对照：沿用与离散动力学相同的 (v⁺,δ⁺,Δθ) 链值，但步内按恒定
// 速率圆弧精确旋转（弦方向 θ+Δθ/2、弦长 2R·sin(Δθ/2)）。作为参考真值时，
// 它与被测格式共享完全相同的控制/状态序列，从而把位移更新格式误差与朝向链的
// 公共截断误差干净剥离——粗/细网格对比会被朝向链的右黎曼一阶误差污染，
// 而本对照只暴露位移格式本身的阶数差异
iLQRState StepWithExactArc(double wheelbase, const iLQRState& x,
                          const iLQRControl& u, double dt) {
    const double a_plus = x(ILQR_IDX_A) + u(ILQR_IDX_JERK) * dt;
    const double omega_plus = x(ILQR_IDX_OMEGA) + u(ILQR_IDX_ETA) * dt;
    const double v_plus = x(ILQR_IDX_V) + a_plus * dt;
    const double delta_plus = x(ILQR_IDX_DELTA) + omega_plus * dt;
    const double tan_delta = std::tan(delta_plus);
    const double dtheta = v_plus * tan_delta * dt / wheelbase;
    const double theta = x(ILQR_IDX_THETA);
    double x_next, y_next;
    if (std::abs(tan_delta) < 1e-12) {
        x_next = x(ILQR_IDX_X) + v_plus * std::cos(theta) * dt;
        y_next = x(ILQR_IDX_Y) + v_plus * std::sin(theta) * dt;
    } else {
        const double radius = wheelbase / tan_delta;
        const double center_x = x(ILQR_IDX_X) - radius * std::sin(theta);
        const double center_y = x(ILQR_IDX_Y) + radius * std::cos(theta);
        const double rel_x = x(ILQR_IDX_X) - center_x;
        const double rel_y = x(ILQR_IDX_Y) - center_y;
        const double cos_d = std::cos(dtheta);
        const double sin_d = std::sin(dtheta);
        x_next = center_x + cos_d * rel_x - sin_d * rel_y;
        y_next = center_y + sin_d * rel_x + cos_d * rel_y;
    }
    iLQRState next;
    next << x_next, y_next, theta + dtheta, v_plus, a_plus, delta_plus,
        omega_plus;
    return next;
}

// 沿 δ*(t)=δ_f·t/T 线性爬坡剖面行驶：通过反解 η 使 δ 逐步精确跟踪目标剖面，
// 三种步进格式共享完全相同的状态序列，仅位移更新的角度处理不同
enum class SteerStepMode { kMidpoint, kOldTheta, kExactArc };
iLQRState TrackSteerProfile(double wheelbase, SteerStepMode mode,
                           const iLQRState& x0, double steer_final,
                           double period, double dt, int steps) {
    const BicycleDynamics dynamics(wheelbase);
    iLQRState x = x0;
    for (int k = 0; k < steps; ++k) {
        const double delta_target = steer_final * (k + 1) * dt / period;
        iLQRControl u;
        u(ILQR_IDX_JERK) = 0.0;
        // δ⁺ = δ + ω·dt + η·dt² ≡ δ* ⟹ η = ((δ*−δ)/dt − ω)/dt
        u(ILQR_IDX_ETA) =
            ((delta_target - x(ILQR_IDX_DELTA)) / dt - x(ILQR_IDX_OMEGA)) / dt;
        if (mode == SteerStepMode::kMidpoint) {
            x = dynamics.step(x, u, dt);
        } else if (mode == SteerStepMode::kOldTheta) {
            x = StepWithOldTheta(wheelbase, x, u, dt);
        } else {
            x = StepWithExactArc(wheelbase, x, u, dt);
        }
    }
    return x;
}

// 测试变曲率路径上中点格式无系统性螺旋漂移（中点格式二阶精度的判别场景）。
// 因为恒定曲率下两种格式的轨迹是全等正多边形、对称剖面下偏置积分对消，
// 只有曲率单调变化（持续同向转向）时旧 θ 格式的位移方向滞后 Δθ/2 才形成
// 带符号累积偏置；所以用线性爬坡转向剖面 δ*(t)=δ_f·t/T（螺旋弧），
// 以共享同一状态序列的精确圆弧积分为参考真值，断言中点格式终点偏差量级
// 显著优于旧 θ 格式（弦方向精确+弦长 O(Δθ²) 偏差 vs 方向 O(Δθ) 带符号偏置）。
TEST(BicycleDynamicsTest, VaryingSteerMidpointOutperformsOldTheta) {
    iLQRState x0 = iLQRState::Zero();
    x0(ILQR_IDX_V) = 0.5;
    const double steer_final = 0.5;
    const double period = 12.0;
    const int steps = static_cast<int>(period / kDt);
    const iLQRState exact =
        TrackSteerProfile(kWheelbase, SteerStepMode::kExactArc, x0, steer_final,
                          period, kDt, steps);
    const iLQRState mid = TrackSteerProfile(kWheelbase, SteerStepMode::kMidpoint,
                                           x0, steer_final, period, kDt, steps);
    const iLQRState old = TrackSteerProfile(kWheelbase, SteerStepMode::kOldTheta,
                                           x0, steer_final, period, kDt, steps);
    const double mid_error = std::hypot(mid(ILQR_IDX_X) - exact(ILQR_IDX_X),
                                        mid(ILQR_IDX_Y) - exact(ILQR_IDX_Y));
    const double old_error = std::hypot(old(ILQR_IDX_X) - exact(ILQR_IDX_X),
                                        old(ILQR_IDX_Y) - exact(ILQR_IDX_Y));
    std::cout << "[DRIFT] mid_error=" << mid_error << " old_error=" << old_error
              << std::endl;
    // 旧 θ 格式的系统性漂移必须真实存在（否则对照无意义），
    // 中点格式终点偏差必须显著更小（弦方向精确，仅残留弦长 O(Δθ²) 偏差）
    EXPECT_GT(old_error, 1e-3);
    EXPECT_LT(mid_error, old_error / 4.0);
    EXPECT_LT(mid_error, 1e-3);
    // 三种格式的朝向更新链完全一致：朝向终点应与精确参考逐步一致
    EXPECT_NEAR(
        std::remainder(mid(ILQR_IDX_THETA) - exact(ILQR_IDX_THETA), 2.0 * PI),
        0.0, 1e-12);
}

// 测试换挡场景：v 过零处动力学光滑、无分支、无 NaN。
// 因为 δ 是显式状态、由 η 积分而来（对照 ALM 的 0/0 奇异反解），
// 所以 v=0 邻域不需要任何正则化常数，v=±ε 的单步结果必须连续，
// 且持续倒车加速度驱动的过零 rollout 全程有限。
TEST(BicycleDynamicsTest, VelocitySignCrossingIsSmoothAndFinite) {
    const BicycleDynamics dynamics(kWheelbase);
    iLQRState x_pos;
    x_pos << 0.0, 0.0, 0.1, 1e-9, -0.2, 0.1, 0.05;
    iLQRState x_neg = x_pos;
    x_neg(ILQR_IDX_V) = -1e-9;
    const iLQRControl u = (iLQRControl() << -0.3, 0.4).finished();
    const iLQRState next_pos = dynamics.step(x_pos, u, kDt);
    const iLQRState next_neg = dynamics.step(x_neg, u, kDt);
    for (int i = 0; i < ILQR_STATE_DIM; ++i) {
        EXPECT_TRUE(std::isfinite(next_pos(i)));
        EXPECT_TRUE(std::isfinite(next_neg(i)));
        EXPECT_NEAR(next_pos(i), next_neg(i), 1e-6) << "component " << i;
    }
    // 从 v=0.02 出发持续施加倒车跃度：v 过零后进入倒车，全程有限
    iLQRState x;
    x << 0.0, 0.0, 0.1, 0.02, -0.2, 0.1, 0.05;
    bool crossed_zero = false;
    for (int step_idx = 0; step_idx < 20; ++step_idx) {
        x = dynamics.step(x, u, kDt);
        for (int i = 0; i < ILQR_STATE_DIM; ++i) {
            ASSERT_TRUE(std::isfinite(x(i))) << "step " << step_idx;
        }
        if (x(ILQR_IDX_V) < 0.0) {
            crossed_zero = true;
        }
    }
    EXPECT_TRUE(crossed_zero);
}

// 转角切线线性化的参考实现：阈值内取精确 tanδ，超阈后沿断点切线做
// 奇偶对称的 C¹ 线性延拓
double ReferenceTan(double delta, double threshold) {
    if (threshold > 0.0 && std::abs(delta) > threshold) {
        const double tan_a = std::tan(threshold);
        const double sec2_a = 1.0 + tan_a * tan_a;
        return std::copysign(tan_a + sec2_a * (std::abs(delta) - threshold),
                             delta);
    }
    return std::tan(delta);
}

// 从 step() 结果反解本步实际使用的 tanδ⁺：Δθ = v⁺·tanδ⁺·dt/L
double ExtractTanFromStep(const BicycleDynamics& dynamics, const iLQRState& x,
                          const iLQRControl& u) {
    const iLQRState next = dynamics.step(x, u, kDt);
    const double v_plus = next(ILQR_IDX_V);
    const double dtheta = next(ILQR_IDX_THETA) - x(ILQR_IDX_THETA);
    return dtheta * kWheelbase / (v_plus * kDt);
}

// 构造一个 δ⁺ 落在指定值、且 v⁺ 非零的运行点（ω=0/加速度链为零，
// 使 δ⁺=δ、v⁺=v，便于直接对照解析式）
iLQRState MakeSteerPoint(double delta, double v) {
    iLQRState x;
    x << 0.0, 0.0, 0.3, v, 0.0, delta, 0.0;
    return x;
}

}  // namespace
}  // namespace apa_post_processor
