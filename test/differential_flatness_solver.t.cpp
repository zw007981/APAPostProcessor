#include "preprocessing/differential_flatness_solver.h"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "vehicle/vehicle_params.h"

namespace apa_post_processor {
namespace {

// 典型车辆参数：与 NMPC 测试保持一致。
VehicleParams MakeVehicleParams() {
    return VehicleParams(/*length=*/4.3, /*width=*/1.8, /*wheelbase=*/2.7,
                         /*max_steer_angle=*/0.6, /*rear_overhang=*/0.8,
                         /*max_accel=*/1.0, /*max_decel=*/-1.5,
                         /*max_steer_rate=*/0.4);
}

// 默认微分平坦补全配置。
DifferentialFlatnessSolverConfig MakeConfig() {
    DifferentialFlatnessSolverConfig config;
    config.curvature_denominator_epsilon = 1e-6;
    return config;
}

// 判断双精度浮点数是否为有限值（非 NaN、非 Inf）。
bool IsFinite(double value) {
    return std::isfinite(value);
}

}  // namespace

// 圆弧场景：曲率恒定，delta 为常数，delta_dot 应为 0。
// 触发原因：验证前轮偏角公式 delta = atan(kappa * L) 的正确性。
// 预期行为：所有点的 delta 接近 atan(L / R)，delta_dot 接近 0，无 NaN/Inf。
TEST(DifferentialFlatnessSolverTest, ComputesConstantCurvatureForCircularArc) {
    const auto vehicle_params = MakeVehicleParams();
    const auto config = MakeConfig();
    DifferentialFlatnessSolver solver(config);

    constexpr double kRadius = 5.0;
    constexpr double kTotalAngle = PI * 0.5;
    constexpr std::size_t kNumPoints = 51;
    constexpr double kV = 0.5;

    DifferentialFlatnessInput input;
    input.x.reserve(kNumPoints);
    input.y.reserve(kNumPoints);
    input.theta.reserve(kNumPoints);
    input.x_d1.reserve(kNumPoints);
    input.x_d2.reserve(kNumPoints);
    input.x_d3.reserve(kNumPoints);
    input.y_d1.reserve(kNumPoints);
    input.y_d2.reserve(kNumPoints);
    input.y_d3.reserve(kNumPoints);
    input.v.reserve(kNumPoints);
    input.a.reserve(kNumPoints);
    input.t.reserve(kNumPoints);

    for (std::size_t i = 0; i < kNumPoints; ++i) {
        const double u =
            static_cast<double>(i) / static_cast<double>(kNumPoints - 1);
        const double theta = u * kTotalAngle;
        const double dtheta_du = kTotalAngle;
        const double d2theta_du2 = 0.0;
        const double d3theta_du3 = 0.0;

        input.x.push_back(kRadius * std::sin(theta));
        input.y.push_back(kRadius * (1.0 - std::cos(theta)));
        input.theta.push_back(theta);

        input.x_d1.push_back(kRadius * std::cos(theta) * dtheta_du);
        input.y_d1.push_back(kRadius * std::sin(theta) * dtheta_du);
        input.x_d2.push_back(-kRadius * std::sin(theta) * dtheta_du * dtheta_du +
                             kRadius * std::cos(theta) * d2theta_du2);
        input.y_d2.push_back(kRadius * std::cos(theta) * dtheta_du * dtheta_du +
                             kRadius * std::sin(theta) * d2theta_du2);
        input.x_d3.push_back(-kRadius * std::cos(theta) * dtheta_du * dtheta_du *
                                 dtheta_du -
                             3.0 * kRadius * std::sin(theta) * dtheta_du *
                                 d2theta_du2 +
                             kRadius * std::cos(theta) * d3theta_du3);
        input.y_d3.push_back(-kRadius * std::sin(theta) * dtheta_du * dtheta_du *
                                 dtheta_du +
                             3.0 * kRadius * std::cos(theta) * dtheta_du *
                                 d2theta_du2 +
                             kRadius * std::sin(theta) * d3theta_du3);

        input.v.push_back(kV);
        input.a.push_back(0.0);
        input.t.push_back(static_cast<double>(i) * 0.1);
    }

    const auto result = solver.solve(input, vehicle_params);

    EXPECT_TRUE(result.success);
    ASSERT_EQ(result.points.size(), kNumPoints);

    const double expected_delta =
        std::atan(vehicle_params.wheelbase / kRadius);
    for (const auto& point : result.points) {
        EXPECT_TRUE(IsFinite(point.getDelta()));
        EXPECT_TRUE(IsFinite(point.getDeltaDot()));
        EXPECT_NEAR(point.getDelta(), expected_delta, 1e-6);
        EXPECT_NEAR(point.getDeltaDot(), 0.0, 1e-6);
    }
}

// 抛物线场景：曲率随弧长变化，delta_dot 不为 0，用于验证链式求导公式。
// 触发原因：曲率导数项 dkappa/ds 的代数展开是 3.3 节核心，需要与解析手算值对齐。
// 预期行为：delta 与 delta_dot 均与解析公式一致。
TEST(DifferentialFlatnessSolverTest, ComputesVaryingCurvatureForParabola) {
    const auto vehicle_params = MakeVehicleParams();
    const auto config = MakeConfig();
    DifferentialFlatnessSolver solver(config);

    constexpr std::size_t kNumPoints = 51;
    constexpr double kV = 0.5;

    DifferentialFlatnessInput input;
    input.x.reserve(kNumPoints);
    input.y.reserve(kNumPoints);
    input.theta.reserve(kNumPoints);
    input.x_d1.reserve(kNumPoints);
    input.x_d2.reserve(kNumPoints);
    input.x_d3.reserve(kNumPoints);
    input.y_d1.reserve(kNumPoints);
    input.y_d2.reserve(kNumPoints);
    input.y_d3.reserve(kNumPoints);
    input.v.reserve(kNumPoints);
    input.a.reserve(kNumPoints);
    input.t.reserve(kNumPoints);

    for (std::size_t i = 0; i < kNumPoints; ++i) {
        const double u =
            static_cast<double>(i) / static_cast<double>(kNumPoints - 1);
        const double speed_sq = 1.0 + 4.0 * u * u;
        const double kappa = 2.0 / std::pow(speed_sq, 1.5);
        const double dkappa_ds = -24.0 * u / std::pow(speed_sq, 3.0);

        input.x.push_back(u);
        input.y.push_back(u * u);
        input.theta.push_back(std::atan2(2.0 * u, 1.0));
        input.x_d1.push_back(1.0);
        input.y_d1.push_back(2.0 * u);
        input.x_d2.push_back(0.0);
        input.y_d2.push_back(2.0);
        input.x_d3.push_back(0.0);
        input.y_d3.push_back(0.0);
        input.v.push_back(kV);
        input.a.push_back(0.0);
        input.t.push_back(static_cast<double>(i) * 0.1);

        const double expected_delta =
            std::atan(kappa * vehicle_params.wheelbase);
        const double expected_delta_dot =
            (vehicle_params.wheelbase * kV /
             (1.0 + (kappa * vehicle_params.wheelbase) *
                        (kappa * vehicle_params.wheelbase))) *
            dkappa_ds;

        // 在循环内逐点断言，失败时更容易定位索引。
        EXPECT_TRUE(IsFinite(expected_delta));
        EXPECT_TRUE(IsFinite(expected_delta_dot));
    }

    const auto result = solver.solve(input, vehicle_params);

    EXPECT_TRUE(result.success);
    ASSERT_EQ(result.points.size(), kNumPoints);

    for (std::size_t i = 0; i < kNumPoints; ++i) {
        const double u =
            static_cast<double>(i) / static_cast<double>(kNumPoints - 1);
        const double speed_sq = 1.0 + 4.0 * u * u;
        const double kappa = 2.0 / std::pow(speed_sq, 1.5);
        const double dkappa_ds = -24.0 * u / std::pow(speed_sq, 3.0);
        const double expected_delta =
            std::atan(kappa * vehicle_params.wheelbase);
        const double expected_delta_dot =
            (vehicle_params.wheelbase * kV /
             (1.0 + (kappa * vehicle_params.wheelbase) *
                        (kappa * vehicle_params.wheelbase))) *
            dkappa_ds;

        EXPECT_TRUE(IsFinite(result.points[i].getDelta()));
        EXPECT_TRUE(IsFinite(result.points[i].getDeltaDot()));
        EXPECT_NEAR(result.points[i].getDelta(), expected_delta, 1e-9);
        EXPECT_NEAR(result.points[i].getDeltaDot(), expected_delta_dot, 1e-9);
    }
}

// 直线段场景：曲率恒为 0，delta 与 delta_dot 均应为 0。
// 触发原因：验证 kappa = 0 时不会出现 atan(0) 歧义或符号错误。
// 预期行为：所有点的 delta = 0，delta_dot = 0。
TEST(DifferentialFlatnessSolverTest, ProducesZeroSteeringForStraightLine) {
    const auto vehicle_params = MakeVehicleParams();
    const auto config = MakeConfig();
    DifferentialFlatnessSolver solver(config);

    constexpr std::size_t kNumPoints = 21;
    constexpr double kV = 0.5;

    DifferentialFlatnessInput input;
    input.x.reserve(kNumPoints);
    input.y.reserve(kNumPoints);
    input.theta.reserve(kNumPoints);
    input.x_d1.reserve(kNumPoints);
    input.x_d2.reserve(kNumPoints);
    input.x_d3.reserve(kNumPoints);
    input.y_d1.reserve(kNumPoints);
    input.y_d2.reserve(kNumPoints);
    input.y_d3.reserve(kNumPoints);
    input.v.reserve(kNumPoints);
    input.a.reserve(kNumPoints);
    input.t.reserve(kNumPoints);

    for (std::size_t i = 0; i < kNumPoints; ++i) {
        const double u =
            static_cast<double>(i) / static_cast<double>(kNumPoints - 1);
        input.x.push_back(u * 10.0);
        input.y.push_back(0.0);
        input.theta.push_back(0.0);
        input.x_d1.push_back(10.0);
        input.y_d1.push_back(0.0);
        input.x_d2.push_back(0.0);
        input.y_d2.push_back(0.0);
        input.x_d3.push_back(0.0);
        input.y_d3.push_back(0.0);
        input.v.push_back(kV);
        input.a.push_back(0.0);
        input.t.push_back(static_cast<double>(i) * 0.1);
    }

    const auto result = solver.solve(input, vehicle_params);

    EXPECT_TRUE(result.success);
    ASSERT_EQ(result.points.size(), kNumPoints);
    for (const auto& point : result.points) {
        EXPECT_NEAR(point.getDelta(), 0.0, 1e-12);
        EXPECT_NEAR(point.getDeltaDot(), 0.0, 1e-12);
    }
}

// 极低速/接近零速场景：参数速度模长极小，验证死区保护生效。
// 触发原因：泊车起步/刹停处 x'、y' 可能同时接近 0，原始公式分母会除零。
// 预期行为：输出有限值（不会 NaN/Inf），delta 由分子决定或趋近于 0。
TEST(DifferentialFlatnessSolverTest, HandlesNearZeroSpeedWithoutNaN) {
    const auto vehicle_params = MakeVehicleParams();
    const auto config = MakeConfig();
    DifferentialFlatnessSolver solver(config);

    constexpr std::size_t kNumPoints = 11;

    DifferentialFlatnessInput input;
    input.x.reserve(kNumPoints);
    input.y.reserve(kNumPoints);
    input.theta.reserve(kNumPoints);
    input.x_d1.reserve(kNumPoints);
    input.x_d2.reserve(kNumPoints);
    input.x_d3.reserve(kNumPoints);
    input.y_d1.reserve(kNumPoints);
    input.y_d2.reserve(kNumPoints);
    input.y_d3.reserve(kNumPoints);
    input.v.reserve(kNumPoints);
    input.a.reserve(kNumPoints);
    input.t.reserve(kNumPoints);

    for (std::size_t i = 0; i < kNumPoints; ++i) {
        const double scale = std::pow(10.0, -static_cast<double>(i));
        input.x.push_back(0.0);
        input.y.push_back(0.0);
        input.theta.push_back(0.0);
        input.x_d1.push_back(scale);
        input.y_d1.push_back(0.0);
        input.x_d2.push_back(0.0);
        input.y_d2.push_back(scale);
        input.x_d3.push_back(0.0);
        input.y_d3.push_back(0.0);
        input.v.push_back(scale);
        input.a.push_back(0.0);
        input.t.push_back(static_cast<double>(i) * 0.01);
    }

    const auto result = solver.solve(input, vehicle_params);

    EXPECT_TRUE(result.success);
    ASSERT_EQ(result.points.size(), kNumPoints);
    for (const auto& point : result.points) {
        EXPECT_TRUE(IsFinite(point.getDelta()))
            << "delta is not finite: " << point.getDelta();
        EXPECT_TRUE(IsFinite(point.getDeltaDot()))
            << "delta_dot is not finite: " << point.getDeltaDot();
    }
}

// 方向符号传递：倒车时 v 为负，delta_dot 符号应随 v 反向。
// 触发原因：验证 delta_dot 公式中 v 的符号被正确传递。
// 预期行为：前进与倒车同一点的 delta 相同，delta_dot 符号相反。
TEST(DifferentialFlatnessSolverTest, PreservesReverseVelocitySign) {
    const auto vehicle_params = MakeVehicleParams();
    const auto config = MakeConfig();
    DifferentialFlatnessSolver solver(config);

    constexpr std::size_t kNumPoints = 21;
    constexpr double kV = 0.5;

    auto buildInput = [&](double v_sign) {
        DifferentialFlatnessInput input;
        input.x.reserve(kNumPoints);
        input.y.reserve(kNumPoints);
        input.theta.reserve(kNumPoints);
        input.x_d1.reserve(kNumPoints);
        input.x_d2.reserve(kNumPoints);
        input.x_d3.reserve(kNumPoints);
        input.y_d1.reserve(kNumPoints);
        input.y_d2.reserve(kNumPoints);
        input.y_d3.reserve(kNumPoints);
        input.v.reserve(kNumPoints);
        input.a.reserve(kNumPoints);
        input.t.reserve(kNumPoints);

        for (std::size_t i = 0; i < kNumPoints; ++i) {
            const double u =
                static_cast<double>(i) / static_cast<double>(kNumPoints - 1);
            input.x.push_back(u);
            input.y.push_back(u * u);
            input.theta.push_back(std::atan2(2.0 * u, 1.0));
            input.x_d1.push_back(1.0);
            input.y_d1.push_back(2.0 * u);
            input.x_d2.push_back(0.0);
            input.y_d2.push_back(2.0);
            input.x_d3.push_back(0.0);
            input.y_d3.push_back(0.0);
            input.v.push_back(v_sign * kV);
            input.a.push_back(0.0);
            input.t.push_back(static_cast<double>(i) * 0.1);
        }
        return input;
    };

    const auto forward_result = solver.solve(buildInput(1.0), vehicle_params);
    const auto reverse_result = solver.solve(buildInput(-1.0), vehicle_params);

    ASSERT_EQ(forward_result.points.size(), kNumPoints);
    ASSERT_EQ(reverse_result.points.size(), kNumPoints);

    for (std::size_t i = 0; i < kNumPoints; ++i) {
        EXPECT_NEAR(forward_result.points[i].getDelta(),
                    reverse_result.points[i].getDelta(), 1e-12);
        EXPECT_NEAR(forward_result.points[i].getDeltaDot(),
                    -reverse_result.points[i].getDeltaDot(), 1e-12);
    }
}

// 非法输入：维度不一致应在入口处抛出异常。
// 触发原因：防御调用方误传长度不等的向量。
// 预期行为：std::invalid_argument。
TEST(DifferentialFlatnessSolverTest, RejectsMismatchedInputSizes) {
    const auto vehicle_params = MakeVehicleParams();
    const auto config = MakeConfig();
    DifferentialFlatnessSolver solver(config);

    DifferentialFlatnessInput input;
    input.x = {0.0, 1.0, 2.0};
    input.y = {0.0, 0.0, 0.0};
    input.theta = {0.0, 0.0, 0.0};
    input.x_d1 = {1.0, 1.0};
    input.x_d2 = {0.0, 0.0, 0.0};
    input.x_d3 = {0.0, 0.0, 0.0};
    input.y_d1 = {0.0, 0.0, 0.0};
    input.y_d2 = {0.0, 0.0, 0.0};
    input.y_d3 = {0.0, 0.0, 0.0};
    input.v = {0.0, 0.0, 0.0};
    input.a = {0.0, 0.0, 0.0};
    input.t = {0.0, 0.1, 0.2};

    EXPECT_THROW(solver.solve(input, vehicle_params), std::invalid_argument);
}

// 非法输入：轴距非正应在入口处抛出异常。
// 触发原因：车辆参数不完整时不能继续计算。
// 预期行为：std::invalid_argument。
TEST(DifferentialFlatnessSolverTest, RejectsNonPositiveWheelbase) {
    const auto config = MakeConfig();
    DifferentialFlatnessSolver solver(config);

    DifferentialFlatnessInput input;
    input.x = {0.0, 1.0};
    input.y = {0.0, 0.0};
    input.theta = {0.0, 0.0};
    input.x_d1 = {1.0, 1.0};
    input.x_d2 = {0.0, 0.0};
    input.x_d3 = {0.0, 0.0};
    input.y_d1 = {0.0, 0.0};
    input.y_d2 = {0.0, 0.0};
    input.y_d3 = {0.0, 0.0};
    input.v = {0.0, 0.0};
    input.a = {0.0, 0.0};
    input.t = {0.0, 0.1};

    VehicleParams bad_params = MakeVehicleParams();
    bad_params.wheelbase = 0.0;

    EXPECT_THROW(solver.solve(input, bad_params), std::invalid_argument);
}

// 非法输入：时间戳递减应在入口处抛出异常。
// 触发原因：时间序列必须单调，否则后续 NMPC 积分会异常。
// 预期行为：std::invalid_argument。
TEST(DifferentialFlatnessSolverTest, RejectsDecreasingTimeStamps) {
    const auto vehicle_params = MakeVehicleParams();
    const auto config = MakeConfig();
    DifferentialFlatnessSolver solver(config);

    DifferentialFlatnessInput input;
    input.x = {0.0, 1.0, 2.0};
    input.y = {0.0, 0.0, 0.0};
    input.theta = {0.0, 0.0, 0.0};
    input.x_d1 = {1.0, 1.0, 1.0};
    input.x_d2 = {0.0, 0.0, 0.0};
    input.x_d3 = {0.0, 0.0, 0.0};
    input.y_d1 = {0.0, 0.0, 0.0};
    input.y_d2 = {0.0, 0.0, 0.0};
    input.y_d3 = {0.0, 0.0, 0.0};
    input.v = {0.0, 0.0, 0.0};
    input.a = {0.0, 0.0, 0.0};
    input.t = {0.0, 0.2, 0.1};

    EXPECT_THROW(solver.solve(input, vehicle_params), std::invalid_argument);
}

// 边界输入：0 点与 1 点输入应在入口处抛出异常。
// 触发原因：覆盖 n_points < 2 分支，保证分支覆盖率。
// 预期行为：std::invalid_argument。
TEST(DifferentialFlatnessSolverTest, RejectsTooFewPoints) {
    const auto vehicle_params = MakeVehicleParams();
    const auto config = MakeConfig();
    DifferentialFlatnessSolver solver(config);

    DifferentialFlatnessInput empty_input;
    EXPECT_THROW(solver.solve(empty_input, vehicle_params),
                 std::invalid_argument);

    DifferentialFlatnessInput single_input;
    single_input.x = {0.0};
    single_input.y = {0.0};
    single_input.theta = {0.0};
    single_input.x_d1 = {1.0};
    single_input.x_d2 = {0.0};
    single_input.x_d3 = {0.0};
    single_input.y_d1 = {0.0};
    single_input.y_d2 = {0.0};
    single_input.y_d3 = {0.0};
    single_input.v = {0.0};
    single_input.a = {0.0};
    single_input.t = {0.0};
    EXPECT_THROW(solver.solve(single_input, vehicle_params),
                 std::invalid_argument);
}

// 非法输入：NaN 值应在入口处抛出异常。
// 触发原因：上游 B 样条数值异常时，Fail-fast 比静默输出 NaN 更安全。
// 预期行为：std::invalid_argument。
TEST(DifferentialFlatnessSolverTest, RejectsNonFiniteInputValues) {
    const auto vehicle_params = MakeVehicleParams();
    const auto config = MakeConfig();
    DifferentialFlatnessSolver solver(config);

    DifferentialFlatnessInput input;
    input.x = {0.0, 1.0};
    input.y = {0.0, 0.0};
    input.theta = {0.0, 0.0};
    input.x_d1 = {1.0, 1.0};
    input.x_d2 = {0.0, 0.0};
    input.x_d3 = {0.0, 0.0};
    input.y_d1 = {0.0, std::numeric_limits<double>::quiet_NaN()};
    input.y_d2 = {0.0, 0.0};
    input.y_d3 = {0.0, 0.0};
    input.v = {0.0, 0.0};
    input.a = {0.0, 0.0};
    input.t = {0.0, 0.1};

    EXPECT_THROW(solver.solve(input, vehicle_params), std::invalid_argument);
}

// 成功路径：status_msg 应包含计算点数摘要。
// 触发原因：验证建议 1 的落地，避免下游误以为 status_msg 始终为空。
// 预期行为：result.status_msg 非空且包含点数信息。
TEST(DifferentialFlatnessSolverTest, ReportsStatusMessageOnSuccess) {
    const auto vehicle_params = MakeVehicleParams();
    const auto config = MakeConfig();
    DifferentialFlatnessSolver solver(config);

    constexpr std::size_t kNumPoints = 5;
    DifferentialFlatnessInput input;
    input.x.assign(kNumPoints, 0.0);
    input.y.assign(kNumPoints, 0.0);
    input.theta.assign(kNumPoints, 0.0);
    input.x_d1.assign(kNumPoints, 1.0);
    input.x_d2.assign(kNumPoints, 0.0);
    input.x_d3.assign(kNumPoints, 0.0);
    input.y_d1.assign(kNumPoints, 0.0);
    input.y_d2.assign(kNumPoints, 0.0);
    input.y_d3.assign(kNumPoints, 0.0);
    input.v.assign(kNumPoints, 0.0);
    input.a.assign(kNumPoints, 0.0);
    input.t.resize(kNumPoints);
    for (std::size_t i = 0; i < kNumPoints; ++i) {
        input.t[i] = static_cast<double>(i) * 0.1;
    }

    const auto result = solver.solve(input, vehicle_params);

    EXPECT_TRUE(result.success);
    EXPECT_FALSE(result.status_msg.empty());
    EXPECT_NE(result.status_msg.find(std::to_string(kNumPoints)),
              std::string::npos);
}

}  // namespace apa_post_processor
