#include "bicycle_kinematics_extractor.h"

#include <cmath>
#include <stdexcept>

#include "../../util/constants.h"

namespace apa_post_processor {
BicycleKinematicsExtractor::BicycleKinematicsExtractor(
    const MincoConfig& config)
    : config_(config),
      tan_steer_max_sq_(std::tan(config.max_steer_angle) *
                        std::tan(config.max_steer_angle)) {
    // 物理参数错误会静默污染全部下游计算，必须在构造期显式拒绝
    if (!std::isfinite(config.wheelbase) || config.wheelbase <= 0.0) {
        throw std::invalid_argument("wheelbase 必须为正有限值");
    }
    if (!std::isfinite(config.max_velocity) || config.max_velocity <= 0.0) {
        throw std::invalid_argument("max_velocity 必须为正有限值");
    }
    if (!std::isfinite(config.max_acceleration) ||
        config.max_acceleration <= 0.0) {
        throw std::invalid_argument("max_acceleration 必须为正有限值");
    }
    if (!std::isfinite(config.max_steer_angle) ||
        config.max_steer_angle <= 0.0 || config.max_steer_angle >= 0.5 * PI) {
        throw std::invalid_argument("max_steer_angle 必须在 (0, π/2) 内");
    }
    if (!std::isfinite(config.max_steer_rate) || config.max_steer_rate <= 0.0) {
        throw std::invalid_argument("max_steer_rate 必须为正有限值");
    }
    if (!std::isfinite(config.epsilon_g) || config.epsilon_g < 0.0) {
        throw std::invalid_argument("epsilon_g 必须为非负有限值");
    }
    if (!std::isfinite(config.steer_hinge_epsilon) ||
        config.steer_hinge_epsilon <= 0.0) {
        throw std::invalid_argument("steer_hinge_epsilon 必须为正有限值");
    }
}

AckermannState BicycleKinematicsExtractor::extract(
    const ThetaSSample& sample) const {
    const double wheelbase = config_.wheelbase;
    const double epsilon_g = config_.epsilon_g;
    AckermannState state;
    state.v = sample.s_dot;
    state.a = sample.s_ddot;
    // δ = atan(L·θ̇/ṡ)。对宗量改写成 L·θ̇·ṡ/(ṡ²+ε_g)：ṡ≠0 且 ε_g→0 时严格
    // 退化为原始公式；ε_g>0 时 (ṡ,θ̇)=(0,0) 处给出有限值 0，避免 0/0 NaN。
    // 与 δ̇ 的分母正则化同源，二者偏差同阶、随 ε_g 同步消失
    const double steer_ratio = wheelbase * sample.theta_dot * sample.s_dot /
                               (sample.s_dot * sample.s_dot + epsilon_g);
    state.delta = std::atan(steer_ratio);
    // δ̇ = L(θ̈ṡ-θ̇s̈)/(ṡ²+L²θ̇²+ε_g)：ε_g 作为分母的一部分，使换挡点
    // （ṡ,θ̇ 同步趋于 0）处数值有界
    const double numerator =
        sample.theta_ddot * sample.s_dot - sample.theta_dot * sample.s_ddot;
    const double denominator =
        sample.s_dot * sample.s_dot +
        wheelbase * wheelbase * sample.theta_dot * sample.theta_dot + epsilon_g;
    state.delta_dot = wheelbase * numerator / denominator;
    return state;
}

SteerGradients BicycleKinematicsExtractor::steerGradients(
    const ThetaSSample& sample) const {
    const double wheelbase = config_.wheelbase;
    const double epsilon_g = config_.epsilon_g;
    const double theta_dot = sample.theta_dot;
    const double s_dot = sample.s_dot;
    SteerGradients gradients;
    // δ = atan(z)，z = L·θ̇·ṡ/(ṡ²+ε_g)；∂δ/∂x = (∂z/∂x)/(1+z²)，仅依赖
    // θ̇ 与 ṡ，对 θ̈、s̈ 的梯度恒为 0
    const double s_dot_sq_plus_eps = s_dot * s_dot + epsilon_g;
    const double steer_ratio =
        wheelbase * theta_dot * s_dot / s_dot_sq_plus_eps;
    const double atan_factor = 1.0 / (1.0 + steer_ratio * steer_ratio);
    gradients.delta.d_theta_dot =
        atan_factor * wheelbase * s_dot / s_dot_sq_plus_eps;
    gradients.delta.d_s_dot = atan_factor * wheelbase * theta_dot *
                              (epsilon_g - s_dot * s_dot) /
                              (s_dot_sq_plus_eps * s_dot_sq_plus_eps);
    // δ̇ = L·r/(m+ε_g)，r = θ̈ṡ-θ̇s̈，m = ṡ²+L²θ̇²；
    // ∂δ̇/∂x = L·[(∂r/∂x)(m+ε_g) - r(∂m/∂x)]/(m+ε_g)²
    const double numerator =
        sample.theta_ddot * s_dot - theta_dot * sample.s_ddot;
    const double denominator = s_dot * s_dot +
                               wheelbase * wheelbase * theta_dot * theta_dot +
                               epsilon_g;
    const double denominator_sq = denominator * denominator;
    gradients.delta_dot.d_theta_dot =
        wheelbase *
        (-sample.s_ddot * denominator -
         numerator * 2.0 * wheelbase * wheelbase * theta_dot) /
        denominator_sq;
    gradients.delta_dot.d_theta_ddot = wheelbase * s_dot / denominator;
    gradients.delta_dot.d_s_dot =
        wheelbase *
        (sample.theta_ddot * denominator - numerator * 2.0 * s_dot) /
        denominator_sq;
    gradients.delta_dot.d_s_ddot = -wheelbase * theta_dot / denominator;
    return gradients;
}

PhysicalConstraintPenalties BicycleKinematicsExtractor::evaluatePenalties(
    const ThetaSSample& sample) const {
    const double wheelbase_sq = config_.wheelbase * config_.wheelbase;
    const double theta_dot = sample.theta_dot;
    const double theta_ddot = sample.theta_ddot;
    const double s_dot = sample.s_dot;
    const double s_ddot = sample.s_ddot;
    PhysicalConstraintPenalties penalties;
    // C_v = ṡ² - v_max²
    penalties.velocity =
        MakePenalty(s_dot * s_dot - config_.max_velocity * config_.max_velocity,
                    SampleGradient{0.0, 0.0, 2.0 * s_dot, 0.0});
    // C_a = s̈² - a_max²
    penalties.acceleration = MakePenalty(
        s_ddot * s_ddot - config_.max_acceleration * config_.max_acceleration,
        SampleGradient{0.0, 0.0, 0.0, 2.0 * s_ddot});
    // C_δ = L²θ̇² - ṡ²tan²(δ_max)（防奇异平方形态，避免直接计算 δ）；
    // 惩罚取光滑 hinge 形态：三次形态在小违反区梯度 ∝C² 消失，换挡尖点
    // 邻域的转向角违反因此对优化器不可见
    penalties.steer_angle =
        MakeHingePenalty(wheelbase_sq * theta_dot * theta_dot -
                             s_dot * s_dot * tan_steer_max_sq_,
                         SampleGradient{2.0 * wheelbase_sq * theta_dot, 0.0,
                                        -2.0 * s_dot * tan_steer_max_sq_, 0.0},
                         config_.steer_hinge_epsilon);
    // C_δ̇ = L²r² - δ̇_max²m²（防奇异交叉乘积形态），r = θ̈ṡ-θ̇s̈，
    // m = ṡ²+L²θ̇²；约束本身为纯多项式，无需 ε_g
    const double cross = theta_ddot * s_dot - theta_dot * s_ddot;
    const double norm = s_dot * s_dot + wheelbase_sq * theta_dot * theta_dot;
    const double steer_rate_max_sq =
        config_.max_steer_rate * config_.max_steer_rate;
    SampleGradient steer_rate_grad;
    steer_rate_grad.d_theta_dot =
        -2.0 * wheelbase_sq * cross * s_ddot -
        4.0 * steer_rate_max_sq * norm * wheelbase_sq * theta_dot;
    steer_rate_grad.d_theta_ddot = 2.0 * wheelbase_sq * cross * s_dot;
    steer_rate_grad.d_s_dot = 2.0 * wheelbase_sq * cross * theta_ddot -
                              4.0 * steer_rate_max_sq * norm * s_dot;
    steer_rate_grad.d_s_ddot = -2.0 * wheelbase_sq * cross * theta_dot;
    penalties.steer_rate = MakePenalty(
        wheelbase_sq * cross * cross - steer_rate_max_sq * norm * norm,
        steer_rate_grad);
    return penalties;
}

ConstraintPenalty BicycleKinematicsExtractor::MakePenalty(
    double constraint, const SampleGradient& grad_c) {
    ConstraintPenalty result;
    result.constraint = constraint;
    if (constraint <= 0.0) {
        return result;
    }
    // 三次光滑外点罚：penalty = C³，∇penalty = 3C²·∇C（C<=0 时值与梯度
    // 均为 0，保证 C² 连续）
    result.penalty = constraint * constraint * constraint;
    const double factor = 3.0 * constraint * constraint;
    result.gradient.d_theta_dot = factor * grad_c.d_theta_dot;
    result.gradient.d_theta_ddot = factor * grad_c.d_theta_ddot;
    result.gradient.d_s_dot = factor * grad_c.d_s_dot;
    result.gradient.d_s_ddot = factor * grad_c.d_s_ddot;
    return result;
}

ConstraintPenalty BicycleKinematicsExtractor::MakeHingePenalty(
    double constraint, const SampleGradient& grad_c, double hinge_epsilon) {
    ConstraintPenalty result;
    result.constraint = constraint;
    if (constraint <= 0.0) {
        return result;
    }
    // 光滑 hinge：0<C≤ε_h 时 penalty=C²/(2ε_h)、∇=(C/ε_h)·∇C；
    // C>ε_h 时 penalty=C−ε_h/2、∇=1·∇C（分段点处值与梯度连续）
    double factor = 1.0;
    if (constraint <= hinge_epsilon) {
        result.penalty = constraint * constraint / (2.0 * hinge_epsilon);
        factor = constraint / hinge_epsilon;
    } else {
        result.penalty = constraint - 0.5 * hinge_epsilon;
    }
    result.gradient.d_theta_dot = factor * grad_c.d_theta_dot;
    result.gradient.d_theta_ddot = factor * grad_c.d_theta_ddot;
    result.gradient.d_s_dot = factor * grad_c.d_s_dot;
    result.gradient.d_s_ddot = factor * grad_c.d_s_ddot;
    return result;
}
}  // namespace apa_post_processor
