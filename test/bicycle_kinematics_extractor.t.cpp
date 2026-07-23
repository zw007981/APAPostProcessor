#include "core/ALM/bicycle_kinematics_extractor.h"

#include <gtest/gtest.h>

#include <cmath>
#include <functional>
#include <stdexcept>

namespace apa_post_processor {
namespace {

constexpr double kWheelbase = 2.8;

// 默认上限配置：wheelbase=2.8，epsilon_g 按测试需要单独指定
BicycleKinematicsConfig MakeConfig(double epsilon_g) {
    BicycleKinematicsConfig config;
    config.wheelbase = kWheelbase;
    config.max_velocity = 2.0;
    config.max_acceleration = 1.5;
    config.max_steer_angle = 0.65;
    config.max_steer_rate = 0.4;
    config.epsilon_g = epsilon_g;
    return config;
}

// 数值中心差分：对 sample 的指定分量（指针成员）扰动 ±h 求 func 的偏导
double CentralDifference(const std::function<double(const ThetaSSample&)>& func,
                         const ThetaSSample& sample,
                         double ThetaSSample::*member, double h) {
    ThetaSSample plus = sample;
    ThetaSSample minus = sample;
    plus.*member += h;
    minus.*member -= h;
    return (func(plus) - func(minus)) / (2.0 * h);
}

// 按索引取 SampleGradient 分量：0=θ̇, 1=θ̈, 2=ṡ, 3=s̈
double GradientComponent(const SampleGradient& grad, int index) {
    switch (index) {
        case 0:
            return grad.d_theta_dot;
        case 1:
            return grad.d_theta_ddot;
        case 2:
            return grad.d_s_dot;
        default:
            return grad.d_s_ddot;
    }
}

// 梯度对拍：相对误差 < 1e-6，近零分量退化为绝对误差 1e-12
void ExpectGradientClose(double analytic, double numeric) {
    EXPECT_LE(std::abs(analytic - numeric),
              1e-6 * std::max(std::abs(numeric), 1e-6))
        << "analytic=" << analytic << " numeric=" << numeric;
}

// 测试常加速度直线的状态提取。
// 因为直线行驶 θ 恒定，所以 δ 与 δ̇ 都必须为 0，v/a 直接等于 ṡ/s̈。
TEST(BicycleKinematicsExtractorTest,
     StraightLineConstantAccelerationExtractsExpectedState) {
    const BicycleKinematicsExtractor extractor(MakeConfig(0.0));
    // θ(t)=0.2 恒定；s(t)=0.5t²+0.1t，在 t=0.8 处 ṡ=0.9、s̈=1.0
    const ThetaSSample sample{0.2, 0.0, 0.0, 0.42, 0.9, 1.0};

    const AckermannState state = extractor.extract(sample);

    EXPECT_NEAR(state.v, 0.9, 1e-12);
    EXPECT_NEAR(state.a, 1.0, 1e-12);
    EXPECT_NEAR(state.delta, 0.0, 1e-12);
    EXPECT_NEAR(state.delta_dot, 0.0, 1e-12);
}

// 测试常曲率圆弧的状态提取。
// 因为 θ̇/ṡ 恒定对应固定曲率，所以 δ 为常值 atan(L·θ̇/ṡ)、δ̇=0。
TEST(BicycleKinematicsExtractorTest,
     ConstantCurvatureArcExtractsExpectedSteer) {
    const BicycleKinematicsExtractor extractor(MakeConfig(0.0));
    // θ(t)=0.3t，s(t)=1.2t：θ̇=0.3、θ̈=0、ṡ=1.2、s̈=0
    const ThetaSSample sample{0.36, 0.3, 0.0, 1.44, 1.2, 0.0};

    const AckermannState state = extractor.extract(sample);

    EXPECT_NEAR(state.v, 1.2, 1e-12);
    EXPECT_NEAR(state.a, 0.0, 1e-12);
    EXPECT_NEAR(state.delta, std::atan(kWheelbase * 0.3 / 1.2), 1e-12);
    EXPECT_NEAR(state.delta_dot, 0.0, 1e-12);
}

// 测试时变转向场景下 δ/δ̇ 与手推解析解一致。
// 因为 δ(t)=atan(L·t/2) 可解析求导，所以实现值必须与手推闭式解吻合。
TEST(BicycleKinematicsExtractorTest, TimeVaryingSteerMatchesHandDerivedValues) {
    const BicycleKinematicsExtractor extractor(MakeConfig(0.0));
    // θ(t)=0.5t²、s(t)=2t，在 t=0.7 处 θ̇=0.7、θ̈=1、ṡ=2、s̈=0
    const ThetaSSample sample{0.245, 0.7, 1.0, 1.4, 2.0, 0.0};

    const AckermannState state = extractor.extract(sample);

    // δ = atan(L·θ̇/ṡ) = atan(2.8·0.7/2.0)
    EXPECT_NEAR(state.delta, std::atan(kWheelbase * 0.7 / 2.0), 1e-12);
    // δ̇ = d/dt atan(L·t/2) = (L/2)/(1+(Lt/2)²) = 2L/(4+0.49L²)
    const double expected_delta_dot =
        2.0 * kWheelbase / (4.0 + kWheelbase * kWheelbase * 0.49);
    EXPECT_NEAR(state.delta_dot, expected_delta_dot, 1e-12);
}

// 测试换挡点附近（ṡ→0）扫描不发散。
// 因为换挡尖点是硬边界，所以 ε_g=0（正则化前，不含精确 0 点）与 ε_g>0
// （正则化后，含精确 0 点）两种配置下 δ/δ̇ 都必须保持有限。
TEST(BicycleKinematicsExtractorTest, ShiftPointScanStaysFinite) {
    for (const double epsilon_g : {0.0, 1e-8}) {
        const BicycleKinematicsExtractor extractor(MakeConfig(epsilon_g));
        for (int k = 1; k <= 12; ++k) {
            ThetaSSample sample{0.0, 0.3, 0.1, 0.0, std::pow(10.0, -k), 0.2};
            const AckermannState state = extractor.extract(sample);
            EXPECT_TRUE(std::isfinite(state.delta))
                << "epsilon_g=" << epsilon_g << " s_dot=1e-" << k;
            EXPECT_TRUE(std::isfinite(state.delta_dot))
                << "epsilon_g=" << epsilon_g << " s_dot=1e-" << k;
        }
    }
    // 精确退化点 (ṡ,θ̇)=(0,0)：正则化配置必须给出有限值而非 0/0 NaN
    const BicycleKinematicsExtractor extractor(MakeConfig(1e-8));
    const AckermannState state = extractor.extract(ThetaSSample{});
    EXPECT_TRUE(std::isfinite(state.delta));
    EXPECT_TRUE(std::isfinite(state.delta_dot));
    EXPECT_DOUBLE_EQ(state.delta, 0.0);
    EXPECT_DOUBLE_EQ(state.delta_dot, 0.0);
}

// 测试 SampleGradient 默认构造的零初始化。
// 因为 MakePenalty 在 C<=0 时直接返回默认构造的零梯度（可行点不对优化目标
// 产生贡献），所以该零值语义必须有显式防御性断言，防止未来结构变更打破。
TEST(BicycleKinematicsExtractorTest, SampleGradientDefaultInitializesToZero) {
    const SampleGradient gradient;
    EXPECT_DOUBLE_EQ(gradient.d_theta_dot, 0.0);
    EXPECT_DOUBLE_EQ(gradient.d_theta_ddot, 0.0);
    EXPECT_DOUBLE_EQ(gradient.d_s_dot, 0.0);
    EXPECT_DOUBLE_EQ(gradient.d_s_ddot, 0.0);
}

// 测试正则化偏差随 ε_g→0 单调收敛到未正则化解析值。
// 因为 ε_g 只是数值安全阀而非物理模型的一部分，所以它引入的偏差必须随
// ε_g 减小而消失。
TEST(BicycleKinematicsExtractorTest,
     RegularizationBiasConvergesToUnregularizedValues) {
    const ThetaSSample sample{0.0, 0.4, 0.6, 0.0, 0.7, 0.3};
    const AckermannState reference =
        BicycleKinematicsExtractor(MakeConfig(0.0)).extract(sample);
    double prev_delta_bias = std::numeric_limits<double>::max();
    double prev_delta_dot_bias = std::numeric_limits<double>::max();
    for (const double epsilon_g : {1e-4, 1e-6, 1e-8, 1e-10}) {
        const AckermannState state =
            BicycleKinematicsExtractor(MakeConfig(epsilon_g)).extract(sample);
        const double delta_bias = std::abs(state.delta - reference.delta);
        const double delta_dot_bias =
            std::abs(state.delta_dot - reference.delta_dot);
        EXPECT_LT(delta_bias, prev_delta_bias);
        EXPECT_LT(delta_dot_bias, prev_delta_dot_bias);
        prev_delta_bias = delta_bias;
        prev_delta_dot_bias = delta_dot_bias;
    }
}

// 测试四个约束在边界内时惩罚恒为 0。
// 因为 max(0,C)^3 在 C<=0 时取 0 且梯度也为 0，所以可行点不应对优化
// 目标产生任何贡献。
TEST(BicycleKinematicsExtractorTest, PenaltiesAreZeroInsideLimits) {
    const BicycleKinematicsExtractor extractor(MakeConfig(1e-8));
    const ThetaSSample sample{0.0, 0.1, 0.1, 0.0, 1.0, 0.5};

    const PhysicalConstraintPenalties penalties =
        extractor.evaluatePenalties(sample);

    const auto check_zero = [](const ConstraintPenalty& penalty) {
        EXPECT_LT(penalty.constraint, 0.0);
        EXPECT_DOUBLE_EQ(penalty.penalty, 0.0);
        EXPECT_DOUBLE_EQ(penalty.gradient.d_theta_dot, 0.0);
        EXPECT_DOUBLE_EQ(penalty.gradient.d_theta_ddot, 0.0);
        EXPECT_DOUBLE_EQ(penalty.gradient.d_s_dot, 0.0);
        EXPECT_DOUBLE_EQ(penalty.gradient.d_s_ddot, 0.0);
    };
    check_zero(penalties.velocity);
    check_zero(penalties.acceleration);
    check_zero(penalties.steer_angle);
    check_zero(penalties.steer_rate);
}

// 测试速度/加速度/转角速度约束在边界外时惩罚为约束值的立方。
// 这三项软惩罚定义为 max(0,C)^3，所以越界点惩罚值必须严格为正且等于 C³。
// 前轮转角约束已改造为光滑 hinge 形态，不在此断言范围（见下方专项测试）。
TEST(BicycleKinematicsExtractorTest, PenaltiesArePositiveOutsideLimits) {
    const BicycleKinematicsExtractor extractor(MakeConfig(1e-8));
    const ThetaSSample sample{0.0, 0.8, 2.0, 0.0, 2.5, 2.0};

    const PhysicalConstraintPenalties penalties =
        extractor.evaluatePenalties(sample);

    const auto check_positive = [](const ConstraintPenalty& penalty) {
        EXPECT_GT(penalty.constraint, 0.0);
        EXPECT_NEAR(
            penalty.penalty,
            penalty.constraint * penalty.constraint * penalty.constraint, 1e-9);
    };
    check_positive(penalties.velocity);
    check_positive(penalties.acceleration);
    check_positive(penalties.steer_rate);
}

// 测试前轮转角约束的光滑 hinge 惩罚形态（C¹ 连续、违反区梯度不消失）。
// 三次方形态在小违反区（C→0⁺）梯度 ∝C² 消失，换挡尖点邻域的转向角违反
// 因此对优化器不可见；hinge 形态在 C>ε_h 时 penalty=C−ε_h/2、梯度=1·∇C，
// 在 0<C≤ε_h 时以 C²/(2ε_h) 平滑过渡，C≤0 时恒为 0。
TEST(BicycleKinematicsExtractorTest, SteerAnglePenaltyUsesSmoothHingeForm) {
    const auto make_sample = [](double theta_dot, double s_dot) {
        ThetaSSample sample{0.0, theta_dot, 0.0, 0.0, s_dot, 0.0};
        return sample;
    };
    const BicycleKinematicsExtractor extractor(MakeConfig(1e-8));
    const double eps = extractor.config().steer_hinge_epsilon;
    ASSERT_GT(eps, 0.0);
    // 大违反（C >> ε_h）：线性区，penalty = C − ε_h/2
    {
        const ThetaSSample sample = make_sample(0.8, 2.5);
        const auto penalty = extractor.evaluatePenalties(sample).steer_angle;
        ASSERT_GT(penalty.constraint, eps);
        EXPECT_NEAR(penalty.penalty, penalty.constraint - eps / 2.0, 1e-12);
    }
    // 小违反（0 < C < ε_h）：二次过渡区，且梯度不消失（与 C³ 形态的关键区别）
    {
        // 构造 C = ε_h/2 的采样点：L²θ̇² − ṡ²tan²(δ_max) = ε_h/2
        // 取 ṡ=0，θ̇ = sqrt(ε_h/2)/L
        const double theta_dot = std::sqrt(eps / 2.0) / kWheelbase;
        const ThetaSSample sample = make_sample(theta_dot, 0.0);
        const auto penalty = extractor.evaluatePenalties(sample).steer_angle;
        ASSERT_GT(penalty.constraint, 0.0);
        ASSERT_LT(penalty.constraint, eps);
        EXPECT_NEAR(penalty.penalty,
                    penalty.constraint * penalty.constraint / (2.0 * eps),
                    1e-15);
        // 梯度模长 = (C/ε_h)·|∇C| > 0；C³ 形态下同点为 3C²|∇C|≈0
        const double grad_norm = std::abs(penalty.gradient.d_theta_dot);
        EXPECT_GT(grad_norm, 0.0);
    }
    // 边界内（C < 0）：严格为 0（与三次方形态一致的可行侧语义）
    {
        const ThetaSSample sample = make_sample(0.0, 1.0);
        const auto penalty = extractor.evaluatePenalties(sample).steer_angle;
        ASSERT_LT(penalty.constraint, 0.0);
        EXPECT_DOUBLE_EQ(penalty.penalty, 0.0);
        EXPECT_DOUBLE_EQ(penalty.gradient.d_theta_dot, 0.0);
    }
}

// 测试四个约束惩罚的解析梯度与中心差分一致。
// 因为 L-BFGS 直接消费这些梯度，所以任一分量错误都会导致虚假下降方向。
TEST(BicycleKinematicsExtractorTest, PenaltyGradientsMatchCentralDifference) {
    const BicycleKinematicsExtractor extractor(MakeConfig(1e-8));
    const ThetaSSample sample{0.0, 0.8, 2.0, 0.0, 2.5, 2.0};
    const PhysicalConstraintPenalties penalties =
        extractor.evaluatePenalties(sample);
    const std::function<double(const ThetaSSample&)> penalty_funcs[] = {
        [&extractor](const ThetaSSample& smp) {
            return extractor.evaluatePenalties(smp).velocity.penalty;
        },
        [&extractor](const ThetaSSample& smp) {
            return extractor.evaluatePenalties(smp).acceleration.penalty;
        },
        [&extractor](const ThetaSSample& smp) {
            return extractor.evaluatePenalties(smp).steer_angle.penalty;
        },
        [&extractor](const ThetaSSample& smp) {
            return extractor.evaluatePenalties(smp).steer_rate.penalty;
        }};
    const SampleGradient gradients[] = {
        penalties.velocity.gradient, penalties.acceleration.gradient,
        penalties.steer_angle.gradient, penalties.steer_rate.gradient};
    double ThetaSSample::*members[] = {
        &ThetaSSample::theta_dot, &ThetaSSample::theta_ddot,
        &ThetaSSample::s_dot, &ThetaSSample::s_ddot};

    for (int penalty_index = 0; penalty_index < 4; ++penalty_index) {
        for (int component = 0; component < 4; ++component) {
            const double numeric = CentralDifference(
                penalty_funcs[penalty_index], sample, members[component], 1e-6);
            ExpectGradientClose(
                GradientComponent(gradients[penalty_index], component),
                numeric);
        }
    }
}

// 测试 δ/δ̇ 的解析梯度与中心差分一致（值与梯度同一正则化公式）。
// 因为设计文档要求 ε_g 同时作用于值与梯度，所以二者必须互洽。
TEST(BicycleKinematicsExtractorTest, SteerGradientsMatchCentralDifference) {
    const BicycleKinematicsExtractor extractor(MakeConfig(1e-8));
    const ThetaSSample sample{0.0, 0.5, 0.8, 0.0, 1.5, 0.7};
    const SteerGradients gradients = extractor.steerGradients(sample);
    const std::function<double(const ThetaSSample&)> value_funcs[] = {
        [&extractor](const ThetaSSample& smp) {
            return extractor.extract(smp).delta;
        },
        [&extractor](const ThetaSSample& smp) {
            return extractor.extract(smp).delta_dot;
        }};
    const SampleGradient analytic[] = {gradients.delta, gradients.delta_dot};
    double ThetaSSample::*members[] = {
        &ThetaSSample::theta_dot, &ThetaSSample::theta_ddot,
        &ThetaSSample::s_dot, &ThetaSSample::s_ddot};

    for (int value_index = 0; value_index < 2; ++value_index) {
        for (int component = 0; component < 4; ++component) {
            const double numeric = CentralDifference(
                value_funcs[value_index], sample, members[component], 1e-6);
            ExpectGradientClose(
                GradientComponent(analytic[value_index], component), numeric);
        }
    }
}

// 测试非法配置的拒绝行为。
// 因为错误物理参数会静默污染全部下游计算，所以必须在构造期显式失败。
TEST(BicycleKinematicsExtractorTest, InvalidConfigThrows) {
    const auto expect_throw = [](BicycleKinematicsConfig config) {
        EXPECT_THROW(BicycleKinematicsExtractor{config}, std::invalid_argument);
    };
    expect_throw([] {
        auto c = MakeConfig(0.0);
        c.wheelbase = 0.0;
        return c;
    }());
    expect_throw([] {
        auto c = MakeConfig(0.0);
        c.wheelbase = -1.0;
        return c;
    }());
    expect_throw([] {
        auto c = MakeConfig(0.0);
        c.wheelbase = std::numeric_limits<double>::quiet_NaN();
        return c;
    }());
    expect_throw([] {
        auto c = MakeConfig(0.0);
        c.max_velocity = 0.0;
        return c;
    }());
    expect_throw([] {
        auto c = MakeConfig(0.0);
        c.max_acceleration = -1.0;
        return c;
    }());
    expect_throw([] {
        auto c = MakeConfig(0.0);
        c.max_steer_angle = 0.0;
        return c;
    }());
    expect_throw([] {
        auto c = MakeConfig(0.0);
        c.max_steer_angle = 1.6;
        return c;
    }());
    expect_throw([] {
        auto c = MakeConfig(0.0);
        c.max_steer_rate = -0.4;
        return c;
    }());
    expect_throw([] {
        auto c = MakeConfig(0.0);
        c.epsilon_g = -1e-9;
        return c;
    }());
    EXPECT_NO_THROW(BicycleKinematicsExtractor{});
}

}  // namespace
}  // namespace apa_post_processor
