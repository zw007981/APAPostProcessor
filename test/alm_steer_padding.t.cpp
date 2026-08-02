#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "core/ALM/alm_steer_padding.h"
#include "spatial/esdf_map.h"
#include "spatial/grid_map.h"
#include "util/maneuver.h"
#include "util/trajectory.h"
#include "vehicle/vehicle_footprint_model.h"
#include "vehicle/vehicle_params.h"

namespace apa_post_processor {
namespace {

constexpr double kDt = 0.1;

// 测试辅助：不含障碍物的大地图与标准 footprint（与运动学校验测试一致）
ESDFMap MakeEmptyEsdfMap() {
    const GridMap grid_map(0.1, 400, 300, Position{-10.0, -10.0}, {});
    return ESDFMap(grid_map);
}

VehicleFootprintModel MakeFootprint() {
    return VehicleFootprintModel(
        VehicleParams(/*length=*/4.3, /*width=*/1.8, /*wheelbase=*/2.7,
                      /*max_steer_angle=*/0.6, /*rear_overhang=*/0.8,
                      /*max_accel=*/1.0, /*max_decel=*/-1.5,
                      /*max_steer_rate=*/0.4),
        /*heading_sample_num=*/233, /*inner_row_num=*/2,
        /*outer_row_num=*/1);
}

TrajectoryPoint MakePoint(double x, double y, double theta, double v, double a,
                          double delta, double delta_dot, double t) {
    TrajectoryPoint pt(x, y, theta);
    pt.setV(v);
    pt.setA(a);
    pt.setDelta(delta);
    pt.setDeltaDot(delta_dot);
    pt.setT(t);
    return pt;
}

// 由速度序列递推构造梯形配点严格一致的点列（a 按
// a_k = 2Δv/dt − a_{k-1} 取得、x 按梯形增量推进），θ/δ/δ̇ 由调用方给出
std::vector<TrajectoryPoint> BuildConsistentPoints(
    const std::vector<double>& vs, const std::vector<double>& thetas,
    const std::vector<double>& deltas, const std::vector<double>& delta_dots) {
    std::vector<TrajectoryPoint> points;
    points.reserve(vs.size());
    double x = 0.0, a = 0.0;
    for (std::size_t k = 0; k < vs.size(); ++k) {
        if (k > 0) {
            x += 0.5 * kDt * (vs[k - 1] + vs[k]);
            a = 2.0 * (vs[k] - vs[k - 1]) / kDt - a;
        }
        points.push_back(MakePoint(x, 0.0, thetas[k], vs[k], a, deltas[k],
                                   delta_dots[k], k * kDt));
    }
    return points;
}

// 由速度序列与逐区间时长序列递推构造梯形配点一致的点列（dts[k] 为
// points[k] 与 points[k+1] 的区间时长），用于构造交界长 dt 的测试场景
std::vector<TrajectoryPoint> BuildConsistentPointsWithDts(
    const std::vector<double>& vs, const std::vector<double>& dts,
    const std::vector<double>& thetas, const std::vector<double>& deltas,
    const std::vector<double>& delta_dots) {
    std::vector<TrajectoryPoint> points;
    points.reserve(vs.size());
    double x = 0.0, a = 0.0, t = 0.0;
    for (std::size_t k = 0; k < vs.size(); ++k) {
        if (k > 0) {
            const double dt = dts[k - 1];
            x += 0.5 * dt * (vs[k - 1] + vs[k]);
            a = 2.0 * (vs[k] - vs[k - 1]) / dt - a;
            t += dt;
        }
        points.push_back(MakePoint(x, 0.0, thetas[k], vs[k], a, deltas[k],
                                   delta_dots[k], t));
    }
    return points;
}

// 测试辅助：逐点对计算转向梯形配点残差（与 Trajectory::validate() 同一
// 公式与低速跳过规则）的最大值
double MaxSteerResidual(const std::vector<TrajectoryPoint>& points) {
    double max_residual = 0.0;
    for (std::size_t i = 0; i + 1 < points.size(); ++i) {
        const auto& p0 = points[i];
        const auto& p1 = points[i + 1];
        const double dt = p1.getT() - p0.getT();
        if (!(dt > 0.0) || dt > 0.5) {
            continue;
        }
        if (std::abs(p0.getV()) < 0.05 && std::abs(p1.getV()) < 0.05) {
            continue;
        }
        const double residual =
            std::abs((p1.getDelta() - p0.getDelta()) -
                     0.5 * dt * (p0.getDeltaDot() + p1.getDeltaDot()));
        max_residual = std::max(max_residual, residual);
    }
    return max_residual;
}

// 测试场景：停驻窗口（净 Δθ≈0 的伪影摆动）被整体替换为合法的
// "停-打轮-走"序列：窗口内 v=0、θ 冻结、|δ|≤δ_max、|δ̇|≤δ̇_max·(1+ε)，
// 且替换后整条轨迹通过 Trajectory::validate() 全部三门（含运动学门）。
TEST(AlmSteerPaddingTest, SmallDthetaWindowIsLegalized) {
    // 速度剖面：平滑减速进入停驻窗口（索引 4..8：0.03/1e-4×3/0.03），
    // 平滑加速驶出，a 按递推与梯形配点严格一致
    const std::vector<double> vs = {0.2,  0.15, 0.1,  0.06, 0.03, 1e-4,
                                    1e-4, 1e-4, 0.03, 0.06, 0.12, 0.2};
    // θ：行驶段恒 0，窗口伪影摆动（净 Δθ=0.0015），窗口后 0.003
    const std::vector<double> thetas = {0.0,   0.0,    0.0,   0.0,
                                        0.001, 0.0015, 0.002, 0.0025,
                                        0.003, 0.003,  0.003, 0.003};
    // δ：行驶段有界，窗口内部全幅翻转（θ-s 提取伪影）
    const std::vector<double> deltas = {0.2,  0.2, 0.2,  0.2,  0.2,  1.5,
                                        -1.5, 1.5, -0.2, -0.2, -0.2, -0.2};
    const std::vector<double> delta_dots(vs.size(), 0.0);
    std::vector<Maneuver> maneuvers;
    maneuvers.emplace_back(
        BuildConsistentPoints(vs, thetas, deltas, delta_dots),
        Direction::FORWARD);
    AlmSteerPaddingConfig config;
    config.max_steer_angle = 0.48;
    config.max_steer_rate = 0.4;
    const auto stats = ApplySteerPadding(maneuvers, config);
    EXPECT_EQ(stats.windows_legalized, 1);
    EXPECT_EQ(stats.windows_skipped, 0);
    const auto& out = maneuvers.front().points;
    // 窗口（索引 4..8）θ 冻结 + δ 过渡带；过渡带按 δ̇_max=0.4 斜率向后
    // 延伸（本构造的窗口跨度 0.4s 不足以完成 0.2→-0.2 过渡，延伸带
    // 覆盖索引 9..11 直到轨迹末尾），交界经滑动平均平滑后斜坡中段保持
    // -0.4、两端斜率渐变收窄，逐对残差保持有界
    for (std::size_t k = 4; k <= 8; ++k) {
        EXPECT_DOUBLE_EQ(out[k].theta, 0.001);
    }
    // 平滑后的 δ 剖面：自 0.2 单调下降至 -0.0667（两端被滑动平均收窄）
    const std::vector<double> expected_delta = {
        0.2, 0.1867, 0.16, 0.12, 0.08, 0.04, 0.0, -0.04, -0.0667};
    // 平滑后的 δ̇ 剖面：斜坡中段（索引 6..9）保持 -0.4，两端渐变收窄
    const std::vector<double> expected_delta_dot = {
        -0.0667, -0.2, -0.3333, -0.4, -0.4, -0.4, -0.4, -0.3333, -0.2667};
    for (std::size_t k = 3; k < out.size(); ++k) {
        EXPECT_NEAR(out[k].getDelta(), expected_delta[k - 3], 1e-3);
        EXPECT_NEAR(out[k].getDeltaDot(), expected_delta_dot[k - 3], 1e-3);
        EXPECT_LE(std::abs(out[k].getDelta()), 0.48 + 1e-9);
        EXPECT_LE(std::abs(out[k].getDeltaDot()), 0.4 + 1e-9);
    }
    // 窗口外（索引 9 起）θ 保持原值
    for (std::size_t k = 9; k < out.size(); ++k) {
        EXPECT_DOUBLE_EQ(out[k].theta, 0.003);
    }
    EXPECT_NEAR(stats.max_steer_rate_used, 0.4, 1e-9);
    EXPECT_EQ(out.size(), vs.size());
    // 时间戳严格递增
    for (std::size_t k = 1; k < out.size(); ++k) {
        EXPECT_GT(out[k].getT(), out[k - 1].getT());
    }
    // 替换后整条轨迹通过完整三门校验（碰撞/终点/运动学）
    Trajectory traj;
    traj.reserve(out.size());
    for (const auto& pt : out) {
        traj.push_back(pt);
    }
    const auto esdf = MakeEmptyEsdfMap();
    const auto footprint = MakeFootprint();
    const TrajectoryPoint goal(traj.back().x, traj.back().y, traj.back().theta);
    const auto validation = traj.validate(goal, esdf, footprint);
    EXPECT_TRUE(validation.kinematic_feasible) << validation.kinematic_detail;
    EXPECT_TRUE(validation.all_passed) << FormatValidationResult(validation);
}

// 测试场景：停驻窗口净 Δθ 超过阈值时不改写——冻结会损失真实旋转量并
// 破坏终点航向预算，保持原样并计入 skipped。
TEST(AlmSteerPaddingTest, LargeDthetaWindowIsSkipped) {
    const std::vector<double> vs = {0.2, 0.1, 1e-4, 1e-4, 1e-4, -0.1, -0.2};
    const std::vector<double> thetas = {0.0,  0.0,  0.01, 0.025,
                                        0.04, 0.04, 0.04};
    const std::vector<double> deltas = {0.1, 0.1, 1.5, -1.5, 1.5, -0.1, -0.1};
    const std::vector<double> delta_dots(vs.size(), 0.0);
    std::vector<Maneuver> maneuvers;
    maneuvers.emplace_back(
        BuildConsistentPoints(vs, thetas, deltas, delta_dots),
        Direction::FORWARD);
    const double before_theta = maneuvers.front().points[3].theta;
    const double before_delta = maneuvers.front().points[3].getDelta();
    AlmSteerPaddingConfig config;
    const auto stats = ApplySteerPadding(maneuvers, config);
    EXPECT_EQ(stats.windows_legalized, 0);
    EXPECT_EQ(stats.windows_skipped, 1);
    EXPECT_EQ(maneuvers.front().points.size(), vs.size());
    EXPECT_DOUBLE_EQ(maneuvers.front().points[3].theta, before_theta);
    EXPECT_DOUBLE_EQ(maneuvers.front().points[3].getDelta(), before_delta);
}

// 测试场景：过渡带与原剖面交界处的 δ̇ 折角在长 dt 点对下不再超标。
// 构造 padding 斜坡恰在交界对前一点到达原剖面值（Δδ=0）且原剖面
// δ̇≈0——交界对残差 ~0.5·dt·|slope−δ̇_orig| = 0.067 超过 0.05 门（与
// 真实数据（爬行区时间拉伸）同形态）；平滑后折角被分摊，最大残差必须
// 低于 0.05 rad。
TEST(AlmSteerPaddingTest, JunctionKinkIsTaperedUnderLongDt) {
    // 速度：平滑减速进入停驻窗口（索引 3..6），0.335s 长 dt 爬行驶出
    const std::vector<double> vs = {0.2,  0.1,  0.06, 1e-4, 1e-4,
                                    1e-4, 1e-4, 0.06, 0.12};
    const std::vector<double> dts = {0.1, 0.1, 0.1, 0.1, 0.1, 0.05, 0.335, 0.1};
    // θ：行驶段恒 0，窗口伪影摆动（净 Δθ<0.02），窗口后 0.003
    const std::vector<double> thetas = {0.0,   0.0,   0.001, 0.0015, 0.002,
                                        0.002, 0.002, 0.003, 0.003};
    // δ：行驶段 0.2，窗口内部全幅翻转（伪影），窗口后原剖面 0.3 平坦
    const std::vector<double> deltas = {0.2,  0.2, 0.2, 1.5, -1.5,
                                        -0.2, 0.3, 0.3, 0.3};
    const std::vector<double> delta_dots(vs.size(), 0.0);
    std::vector<Maneuver> maneuvers;
    maneuvers.emplace_back(
        BuildConsistentPointsWithDts(vs, dts, thetas, deltas, delta_dots),
        Direction::FORWARD);
    AlmSteerPaddingConfig config;
    config.max_steer_angle = 0.48;
    config.max_steer_rate = 0.4;
    const auto stats = ApplySteerPadding(maneuvers, config);
    EXPECT_EQ(stats.windows_legalized, 1);
    const auto& out = maneuvers.front().points;
    // 硬件约束保持：|δ|≤δ_max、|δ̇|≤δ̇_max·(1+ε)、时间戳严格递增
    for (const auto& pt : out) {
        EXPECT_LE(std::abs(pt.getDelta()), 0.48 + 1e-9);
        EXPECT_LE(std::abs(pt.getDeltaDot()), 0.4 + 1e-6);
    }
    for (std::size_t k = 1; k < out.size(); ++k) {
        EXPECT_GT(out[k].getT(), out[k - 1].getT());
    }
    // 交界折角被平滑后，最大转向梯形配点残差必须低于 0.05 rad 门
    EXPECT_LT(MaxSteerResidual(out), 0.05);
    // 整条轨迹通过完整三门校验
    Trajectory traj;
    traj.reserve(out.size());
    for (const auto& pt : out) {
        traj.push_back(pt);
    }
    const auto esdf = MakeEmptyEsdfMap();
    const auto footprint = MakeFootprint();
    const TrajectoryPoint goal(traj.back().x, traj.back().y, traj.back().theta);
    const auto validation = traj.validate(goal, esdf, footprint);
    EXPECT_TRUE(validation.all_passed) << FormatValidationResult(validation);
}

}  // namespace
}  // namespace apa_post_processor
