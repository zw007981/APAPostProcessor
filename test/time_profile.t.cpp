#include "util/time_profile.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

#include "vehicle/vehicle_params.h"

namespace apa_post_processor {
namespace {

// 测试辅助：纵向极限齐全的车辆参数
VehicleParams MakeTimeProfileVehicleParams() {
    return VehicleParams(/*length=*/4.8, /*width=*/1.9, /*wheelbase=*/2.8,
                         /*max_steer_angle=*/0.65, /*rear_overhang=*/0.0,
                         /*max_accel=*/1.5, /*max_decel=*/-3.0,
                         /*max_steer_rate=*/0.4);
}

// 测试辅助：时间参数化输入（弧长/曲率/方向符号/换挡点下标四序列）
struct ProfileInput {
    std::vector<double> s;
    std::vector<double> kappa;
    std::vector<int> sigma;
    std::vector<std::size_t> cusps;
};

// 测试辅助：构造等距弧长的匀向输入（σ=+1，κ 可指定）
ProfileInput MakeStraightInput(double length, double step, double kappa = 0.0) {
    ProfileInput input;
    const int count = static_cast<int>(std::round(length / step));
    input.s.reserve(count + 1);
    input.kappa.reserve(count + 1);
    input.sigma.reserve(count + 1);
    for (int i = 0; i <= count; ++i) {
        input.s.push_back(std::min(i * step, length));
        input.kappa.push_back(kappa);
        input.sigma.push_back(1);
    }
    return input;
}

// 测试辅助：构造 前进1.0m(σ=+1) → 后退0.5m(σ=-1) 的换挡输入（步长0.05m，
// 换挡点为下标 20 的停驻点）
ProfileInput MakeGearShiftInput() {
    ProfileInput input;
    input.s.reserve(31);
    input.kappa.reserve(31);
    input.sigma.reserve(31);
    for (int i = 0; i <= 20; ++i) {
        input.s.push_back(i * 0.05);
        input.kappa.push_back(0.0);
        input.sigma.push_back(1);
    }
    for (int i = 1; i <= 10; ++i) {
        input.s.push_back(1.0 + i * 0.05);
        input.kappa.push_back(0.0);
        input.sigma.push_back(-1);
    }
    input.cusps.push_back(20);
    return input;
}

}  // namespace

// 测试场景：前进直线输入下的梯形加减速剖面。
// 预期行为：首末零速、内部全正、峰值贴限速平台、加速度有界、t 严格递增，
// 时长落在"最快走完"的合理区间（不短于纯巡航、不长于巡航+加减速惩罚
// 的宽松上界）。
TEST(TimeProfileTest, StopsAtEndsAndRespectsLimits) {
    const auto params = MakeTimeProfileVehicleParams();
    const auto input = MakeStraightInput(2.0, 0.05);
    const auto out = ComputeTimeProfile(input.s, input.kappa, input.sigma,
                                        input.cusps, params);
    ASSERT_EQ(out.v.size(), input.s.size());
    EXPECT_DOUBLE_EQ(out.v.front(), 0.0);
    EXPECT_DOUBLE_EQ(out.v.back(), 0.0);
    double max_v = 0.0;
    for (std::size_t i = 1; i + 1 < out.v.size(); ++i) {
        EXPECT_GT(out.v[i], 0.0);
        max_v = std::max(max_v, out.v[i]);
    }
    const double cap = TimeProfileConfig{}.max_v_forward;
    EXPECT_LE(max_v, cap + 1e-9);
    EXPECT_NEAR(max_v, cap, 1e-6);
    for (const double a : out.a) {
        EXPECT_GE(a, params.max_decel - 1e-6);
        EXPECT_LE(a, params.max_accel + 1e-6);
    }
    EXPECT_GT(out.a.front(), 0.0);
    EXPECT_LT(out.a.back(), 0.0);
    for (std::size_t i = 1; i < out.t.size(); ++i) {
        EXPECT_GT(out.t[i], out.t[i - 1]);
    }
    const double duration = out.t.back();
    EXPECT_GE(duration, 2.0 / cap);
    EXPECT_LT(duration, 2.0 / cap +
                            cap * (1.0 / params.max_accel +
                                   1.0 / std::abs(params.max_decel)) +
                            0.5);
}

// 测试场景：κ=1（半径 1m）输入，侧向加速度上限 1.0 m/s²。
// 预期行为：速度被曲率上限 v²<=a_lat/|κ|=1.0 封顶，平台贴 1.0 m/s。
TEST(TimeProfileTest, RespectsCurvatureSpeedCap) {
    const auto input = MakeStraightInput(2.0, 0.05, /*kappa=*/1.0);
    const auto out =
        ComputeTimeProfile(input.s, input.kappa, input.sigma, input.cusps,
                           MakeTimeProfileVehicleParams());
    double max_v = 0.0;
    for (const double v : out.v) {
        max_v = std::max(max_v, v);
    }
    EXPECT_LE(max_v, 1.0 + 1e-9);
    EXPECT_NEAR(max_v, 1.0, 1e-6);
}

// 测试场景：换挡输入（前进 1.0m → 后退 0.5m，换挡点下标 20）。
// 预期行为：首末与换挡点零速，换挡前为正、换挡后为负，t 严格递增；
// 空间加速度 σ·a 落在 [max_decel, max_accel] box 内（与
// SpeedProfilePlanner 的加速度约束约定一致：沿运动方向加速受油门极限、
// 制动受刹车极限，倒车制动时带符号 a 为正且可达 |max_decel|）。
TEST(TimeProfileTest, StopsAtCuspAndAppliesDirectionSigns) {
    const auto params = MakeTimeProfileVehicleParams();
    const auto input = MakeGearShiftInput();
    const auto out = ComputeTimeProfile(input.s, input.kappa, input.sigma,
                                        input.cusps, params);
    ASSERT_EQ(out.v.size(), input.s.size());
    EXPECT_DOUBLE_EQ(out.v.front(), 0.0);
    EXPECT_DOUBLE_EQ(out.v[20], 0.0);
    EXPECT_DOUBLE_EQ(out.v.back(), 0.0);
    for (std::size_t i = 1; i < 20; ++i) {
        EXPECT_GT(out.v[i], 0.0);
    }
    for (std::size_t i = 21; i + 1 < out.v.size(); ++i) {
        EXPECT_LT(out.v[i], 0.0);
    }
    for (std::size_t i = 0; i < out.a.size(); ++i) {
        const double a_spatial = input.sigma[i] * out.a[i];
        EXPECT_GE(a_spatial, params.max_decel - 1e-6);
        EXPECT_LE(a_spatial, params.max_accel + 1e-6);
    }
    for (std::size_t i = 1; i < out.t.size(); ++i) {
        EXPECT_GT(out.t[i], out.t[i - 1]);
    }
}

// 测试场景：单点输入。
// 预期行为：v/a/t 均为 0，不抛异常。
TEST(TimeProfileTest, SinglePointYieldsZeros) {
    const ProfileInput input{{0.0}, {0.0}, {1}, {}};
    const auto out =
        ComputeTimeProfile(input.s, input.kappa, input.sigma, input.cusps,
                           MakeTimeProfileVehicleParams());
    ASSERT_EQ(out.v.size(), 1u);
    EXPECT_DOUBLE_EQ(out.v[0], 0.0);
    EXPECT_DOUBLE_EQ(out.a[0], 0.0);
    EXPECT_DOUBLE_EQ(out.t[0], 0.0);
}

// 测试场景：非法配置（限速非正、侧向加速度非正）。
// 预期行为：抛 std::invalid_argument。
TEST(TimeProfileTest, RejectsInvalidConfig) {
    const auto params = MakeTimeProfileVehicleParams();
    const auto input = MakeStraightInput(1.0, 0.05);
    TimeProfileConfig config;
    config.max_v_forward = 0.0;
    EXPECT_THROW(ComputeTimeProfile(input.s, input.kappa, input.sigma,
                                    input.cusps, params, config),
                 std::invalid_argument);
    config = TimeProfileConfig{};
    config.max_lateral_accel = -1.0;
    EXPECT_THROW(ComputeTimeProfile(input.s, input.kappa, input.sigma,
                                    input.cusps, params, config),
                 std::invalid_argument);
}

// 测试场景：非法车辆纵向极限（max_decel 非正）。
// 预期行为：抛 std::invalid_argument。
TEST(TimeProfileTest, RejectsInvalidVehicleParams) {
    auto bad_params = MakeTimeProfileVehicleParams();
    bad_params.max_decel = 1.0;
    const auto input = MakeStraightInput(1.0, 0.05);
    EXPECT_THROW(ComputeTimeProfile(input.s, input.kappa, input.sigma,
                                    input.cusps, bad_params),
                 std::invalid_argument);
}

// 测试场景：输入向量长度不一致、换挡点下标越界。
// 预期行为：抛 std::invalid_argument。
TEST(TimeProfileTest, RejectsMalformedInputs) {
    const auto params = MakeTimeProfileVehicleParams();
    auto input = MakeStraightInput(1.0, 0.05);
    input.kappa.pop_back();
    EXPECT_THROW(ComputeTimeProfile(input.s, input.kappa, input.sigma,
                                    input.cusps, params),
                 std::invalid_argument);
    input = MakeGearShiftInput();
    input.cusps = {0};
    EXPECT_THROW(ComputeTimeProfile(input.s, input.kappa, input.sigma,
                                    input.cusps, params),
                 std::invalid_argument);
    input.cusps = {input.s.size() - 1};
    EXPECT_THROW(ComputeTimeProfile(input.s, input.kappa, input.sigma,
                                    input.cusps, params),
                 std::invalid_argument);
}

}  // namespace apa_post_processor
