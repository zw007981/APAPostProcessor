#include "preprocessing/speed_profile_planner.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "vehicle/vehicle_params.h"

namespace apa_post_processor {
namespace {

// 典型车辆参数：与 NMPC 测试保持一致，并带本阶段需要的加减速极限。
VehicleParams MakeVehicleParams() {
    return VehicleParams(/*length=*/4.3, /*width=*/1.8, /*wheelbase=*/2.7,
                         /*max_steer_angle=*/0.6, /*rear_overhang=*/0.8,
                         /*max_accel=*/1.0, /*max_decel=*/-1.5,
                         /*max_steer_rate=*/0.4);
}

// 默认速度规划配置
SpeedProfilePlannerConfig MakeConfig() {
    SpeedProfilePlannerConfig config;
    config.max_v_forward = 1.389;
    config.max_v_reverse = 1.0;
    config.max_jerk_proxy = 3.0;
    config.weight_v_ref = 10.0;
    config.weight_a_sq = 2.0;
    config.weight_jerk_sq = 80.0;
    config.time_reintegration_epsilon = 1e-3;
    config.max_lateral_accel = 1.0;
    config.esdf_danger_margin = 0.5;
    return config;
}

// 构造等距弧长输入
SpeedProfileInput MakeStraightInput(double length, double step,
                                    double kappa = 0.0,
                                    double esdf_dist = 10.0) {
    SpeedProfileInput input;
    for (double s = 0.0; s <= length + 1e-9; s += step) {
        input.s.push_back(std::min(s, length));
        input.kappa.push_back(kappa);
        input.min_esdf_dist.push_back(esdf_dist);
    }
    return input;
}

}  // namespace

// 平直无障碍前进段：速度剖面应满足边界条件、参考速度、加速度 box bound。
// 触发原因：SpeedProfilePlanner 的 Happy Path，验证 QP 组装与求解正确性。
// 预期行为：success=true，v0=0，vN=0，速度非负，时间戳单调递增，加速度在 box
// 内。
TEST(SpeedProfilePlannerTest, PlansForwardStraightSegment) {
    const auto vehicle_params = MakeVehicleParams();
    const auto config = MakeConfig();
    SpeedProfilePlanner planner(config);

    const auto input = MakeStraightInput(/*length=*/10.0, /*step=*/0.5);
    const std::vector<int> signs(input.s.size(), 1);
    const auto result = planner.plan(input, vehicle_params, signs);

    EXPECT_TRUE(result.success) << "status: " << result.status_msg;
    ASSERT_FALSE(result.v.empty());
    ASSERT_FALSE(result.t.empty());
    ASSERT_FALSE(result.a.empty());

    EXPECT_NEAR(result.v.front(), 0.0, 1e-3);
    EXPECT_NEAR(result.v.back(), 0.0, 1e-3);
    EXPECT_GE(result.b.front(), 0.0);
    EXPECT_NEAR(result.b.front(), 0.0, 1e-4);

    for (std::size_t i = 0; i + 1 < result.t.size(); ++i) {
        EXPECT_LE(result.t[i], result.t[i + 1]);
    }
    for (const double v : result.v) {
        EXPECT_GE(v, -1e-4);
        EXPECT_LE(v, config.max_v_forward + 1e-3);
    }
    for (const double a : result.a) {
        EXPECT_GE(a, vehicle_params.max_decel - 1e-3);
        EXPECT_LE(a, vehicle_params.max_accel + 1e-3);
    }
}

// 非零初始速度：验证 b0 = initial_velocity^2 且不影响终点刹停。
// 触发原因：重规划/非静止起步场景需要暴露 initial_velocity 参数。
// 预期行为：v0 等于传入的 initial_velocity，vN 仍为 0。
TEST(SpeedProfilePlannerTest, HonorsNonZeroInitialVelocity) {
    const auto vehicle_params = MakeVehicleParams();
    const auto config = MakeConfig();
    SpeedProfilePlanner planner(config);

    const auto input = MakeStraightInput(/*length=*/10.0, /*step=*/0.5);
    const std::vector<int> signs(input.s.size(), 1);
    constexpr double kInitialV = 0.6;
    const auto result =
        planner.plan(input, vehicle_params, signs, {}, kInitialV);

    EXPECT_TRUE(result.success);
    EXPECT_NEAR(result.v.front(), kInitialV, 1e-3);
    EXPECT_NEAR(result.b.front(), kInitialV * kInitialV, 1e-4);
    EXPECT_NEAR(result.v.back(), 0.0, 1e-3);
}

// 中间换挡尖点约束：跨前进-倒车两段拼接时，尖点处 b_k = 0。
// 触发原因：验证多 Maneuver 全局拼接时的强制静止约束。
// 预期行为：尖点索引处 v=0，前后段速度符号正确。
TEST(SpeedProfilePlannerTest, EnforcesCuspZeroVelocity) {
    const auto vehicle_params = MakeVehicleParams();
    const auto config = MakeConfig();
    SpeedProfilePlanner planner(config);

    // 0~10m 前进，10~20m 倒车，共 41 个点，尖点在索引 20。
    SpeedProfileInput input = MakeStraightInput(/*length=*/20.0, /*step=*/0.5);
    std::vector<int> signs(input.s.size(), 1);
    for (std::size_t i = 21; i < signs.size(); ++i) {
        signs[i] = -1;
    }
    const std::vector<std::size_t> cusp_indices = {20};

    const auto result =
        planner.plan(input, vehicle_params, signs, cusp_indices);

    EXPECT_TRUE(result.success);
    EXPECT_NEAR(result.v[20], 0.0, 1e-4);
    EXPECT_NEAR(result.b[20], 0.0, 1e-6);

    for (std::size_t i = 0; i < 20; ++i) {
        EXPECT_GE(result.v[i], -1e-4);
    }
    for (std::size_t i = 21; i < result.v.size(); ++i) {
        EXPECT_LE(result.v[i], 1e-4);
    }
}

// 曲率收紧速度上限：高曲率段的最大速度受 max_lateral_accel / |kappa| 限制。
// 触发原因：验证 V_limit^2[i] 中的曲率项生效。
// 预期行为：最大速度不超过 sqrt(max_lateral_accel / kappa)。
TEST(SpeedProfilePlannerTest, RespectsCurvatureSpeedLimit) {
    const auto vehicle_params = MakeVehicleParams();
    const auto config = MakeConfig();
    SpeedProfilePlanner planner(config);

    constexpr double kKappa = 1.0;
    const auto input = MakeStraightInput(/*length=*/5.0, /*step=*/0.1, kKappa);
    const std::vector<int> signs(input.s.size(), 1);
    const auto result = planner.plan(input, vehicle_params, signs);

    EXPECT_TRUE(result.success);
    const double expected_v_max = std::sqrt(config.max_lateral_accel / kKappa);
    double v_max = 0.0;
    for (const double v : result.v) {
        v_max = std::max(v_max, std::abs(v));
    }
    EXPECT_LE(v_max, expected_v_max + 1e-3);
}

// ESDF 危险度收紧速度上限：靠近障碍物时速度被线性压低。
// 触发原因：验证 V_limit^2[i] 中的 ESDF 项生效。
// 预期行为：危险区域的速度低于无障碍区域。
TEST(SpeedProfilePlannerTest, RespectsEsdfDangerSpeedLimit) {
    const auto vehicle_params = MakeVehicleParams();
    const auto config = MakeConfig();
    SpeedProfilePlanner planner(config);

    auto input = MakeStraightInput(/*length=*/10.0, /*step=*/0.2);
    // 中间一段距离很近
    for (std::size_t i = 15; i < 25 && i < input.min_esdf_dist.size(); ++i) {
        input.min_esdf_dist[i] = 0.1;
    }
    const std::vector<int> signs(input.s.size(), 1);
    const auto result = planner.plan(input, vehicle_params, signs);

    EXPECT_TRUE(result.success);
    const double safe_speed =
        *std::max_element(result.v.begin(), result.v.begin() + 15);
    const double danger_speed =
        *std::max_element(result.v.begin() + 15, result.v.begin() + 25);
    EXPECT_LT(danger_speed, safe_speed);
}

// 非法尖点索引：首尾索引会冲突或冗余，应在 plan() 入口抛出异常。
// 触发原因：防御调用方误传 0 或 n_points-1 作为 cusp_indices。
// 预期行为：std::invalid_argument。
TEST(SpeedProfilePlannerTest, RejectsInvalidCuspIndices) {
    const auto vehicle_params = MakeVehicleParams();
    const auto config = MakeConfig();
    SpeedProfilePlanner planner(config);

    const auto input = MakeStraightInput(/*length=*/5.0, /*step=*/0.5);
    const std::vector<int> signs(input.s.size(), 1);

    EXPECT_THROW(planner.plan(input, vehicle_params, signs, {0},
                              /*initial_velocity=*/0.5),
                 std::invalid_argument);
    EXPECT_THROW(planner.plan(input, vehicle_params, signs,
                              {input.s.size() - 1}, /*initial_velocity=*/0.5),
                 std::invalid_argument);
}

// 禁用 jerk 约束：max_jerk_proxy = 0 时不应生成 jerk 行，求解仍成功。
// 触发原因：验证 jerk 约束的可选性。
// 预期行为：result.success=true，速度非空。
TEST(SpeedProfilePlannerTest, DisablesJerkConstraintWhenMaxJerkProxyIsZero) {
    auto config = MakeConfig();
    config.max_jerk_proxy = 0.0;
    const SpeedProfilePlanner planner(config);
    const auto vehicle_params = MakeVehicleParams();

    const auto input = MakeStraightInput(/*length=*/5.0, /*step=*/0.5);
    const std::vector<int> signs(input.s.size(), 1);
    const auto result = planner.plan(input, vehicle_params, signs);

    EXPECT_TRUE(result.success);
    EXPECT_FALSE(result.v.empty());
}

// QP 不可行场景：过弱的减速度能力无法让车辆从非零初速度刹停，应返回
// success=false。 触发原因：验证求解器失败时的降级行为（不崩溃、不抛异常）。
// 预期行为：result.success=false。
TEST(SpeedProfilePlannerTest, ReportsFailureOnInfeasibleProblem) {
    auto vehicle_params = MakeVehicleParams();
    vehicle_params.max_accel = 0.1;
    vehicle_params.max_decel = -0.1;
    const SpeedProfilePlanner planner(MakeConfig());

    const auto input = MakeStraightInput(/*length=*/1.0, /*step=*/0.1);
    const std::vector<int> signs(input.s.size(), 1);
    const auto result = planner.plan(input, vehicle_params, signs, {},
                                     /*initial_velocity=*/1.0);

    EXPECT_FALSE(result.success);
}

// 倒车方向符号：全段方向为 -1 时应输出非正速度，加速度符号与方向一致。
// 触发原因：验证 v_i = sign * sqrt(b_i) 的方向复原逻辑。
// 预期行为：所有 v <= 0，时间戳仍单调递增。
TEST(SpeedProfilePlannerTest, RestoresReverseDirectionSign) {
    const auto vehicle_params = MakeVehicleParams();
    const auto config = MakeConfig();
    SpeedProfilePlanner planner(config);

    const auto input = MakeStraightInput(/*length=*/5.0, /*step=*/0.25);
    const std::vector<int> signs(input.s.size(), -1);
    const auto result = planner.plan(input, vehicle_params, signs);

    EXPECT_TRUE(result.success);
    for (const double v : result.v) {
        EXPECT_LE(v, 1e-4);
        EXPECT_GE(v, -config.max_v_reverse - 1e-3);
    }
    for (std::size_t i = 0; i + 1 < result.t.size(); ++i) {
        EXPECT_LE(result.t[i], result.t[i + 1]);
    }
}

}  // namespace apa_post_processor
