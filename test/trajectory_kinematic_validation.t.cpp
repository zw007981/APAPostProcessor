// Trajectory::validate() 运动学可行性校验（梯形配点残差）的独立测试文件。
// 与 test/trajectory.t.cpp 分开存放：本文件覆盖配点残差的各边界分支与
// 合成轨迹的容差行为，编译依赖与耗时特征不同。
#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include "spatial/esdf_map.h"
#include "spatial/grid_map.h"
#include "util/trajectory.h"
#include "vehicle/vehicle_footprint_model.h"
#include "vehicle/vehicle_params.h"

namespace apa_post_processor {
namespace {

// 测试辅助：不含障碍物的大地图，车辆永远不碰撞
ESDFMap MakeEmptyEsdfMap() {
    const GridMap grid_map(0.1, 400, 300, Position{-10.0, -10.0}, {});
    return ESDFMap(grid_map);
}

// 测试辅助：标准车辆参数（轴距 2.7m，与既有 trajectory.t.cpp 一致）
VehicleParams MakeKinematicVehicleParams() {
    return VehicleParams(/*length=*/4.3, /*width=*/1.8, /*wheelbase=*/2.7,
                         /*max_steer_angle=*/0.6, /*rear_overhang=*/0.8,
                         /*max_accel=*/1.0, /*max_decel=*/-1.5,
                         /*max_steer_rate=*/0.4);
}

// 测试辅助：标准外圆 footprint 模型
VehicleFootprintModel MakeKinematicFootprintModel() {
    return VehicleFootprintModel(MakeKinematicVehicleParams(),
                                 /*heading_sample_num=*/233,
                                 /*inner_row_num=*/2, /*outer_row_num=*/1);
}

constexpr double kWheelbase = 2.7;
constexpr double kDt = 0.1;
constexpr int kNumPoints = 21;

// 构造一个完整携带运动学量的轨迹点
TrajectoryPoint MakeKinematicPoint(double x, double y, double theta, double v,
                                   double a, double delta, double delta_dot,
                                   double t) {
    TrajectoryPoint pt(x, y, theta);
    pt.setV(v);
    pt.setA(a);
    pt.setDelta(delta);
    pt.setDeltaDot(delta_dot);
    pt.setT(t);
    return pt;
}

// 匀速直线：v=1 m/s、θ=0、δ=0、a=0、δ̇=0。梯形残差解析上恒为 0
Trajectory BuildUniformStraightTrajectory() {
    Trajectory traj;
    traj.reserve(kNumPoints);
    for (int k = 0; k < kNumPoints; ++k) {
        traj.push_back(MakeKinematicPoint(k * kDt * 1.0, 0.0, 0.0, 1.0, 0.0,
                                          0.0, 0.0, k * kDt));
    }
    return traj;
}

// 匀速圆弧：v=1 m/s、δ=0.2 rad 恒值，半径 R=L/tanδ。x/y 的梯形残差为弦弧
// 截断误差（O(R·Δθ³)，非零但极小），θ/v/δ 残差解析上恒为 0
Trajectory BuildCircularArcTrajectory() {
    Trajectory traj;
    traj.reserve(kNumPoints);
    const double delta = 0.2;
    const double radius = kWheelbase / std::tan(delta);
    const double omega = 1.0 * std::tan(delta) / kWheelbase;
    for (int k = 0; k < kNumPoints; ++k) {
        const double theta = omega * k * kDt;
        traj.push_back(MakeKinematicPoint(
            radius * std::sin(theta), radius * (1.0 - std::cos(theta)), theta,
            1.0, 0.0, delta, 0.0, k * kDt));
    }
    return traj;
}

// 匀加速直线：v0=0.5、a=1.0 恒值，x 取精确积分。v/x 的梯形残差解析上恒为 0
Trajectory BuildConstantAccelTrajectory() {
    Trajectory traj;
    traj.reserve(kNumPoints);
    for (int k = 0; k < kNumPoints; ++k) {
        const double t = k * kDt;
        traj.push_back(MakeKinematicPoint(0.5 * t + 0.5 * t * t, 0.0, 0.0,
                                          0.5 + t, 1.0, 0.0, 0.0, t));
    }
    return traj;
}

// 测试场景：匀速直线轨迹的全部 5 项梯形残差在解析上恒为 0。
// 预期行为：kinematic_feasible=true，四项最大残差均不超过 1e-12，
// 且其它门同时通过时 all_passed=true。
TEST(TrajectoryKinematicValidationTest,
     UniformStraightLinePassesWithZeroResidual) {
    const auto traj = BuildUniformStraightTrajectory();
    const auto esdf = MakeEmptyEsdfMap();
    const auto footprint = MakeKinematicFootprintModel();
    const TrajectoryPoint goal(traj.back().x, 0.0, 0.0);
    const auto result = traj.validate(goal, esdf, footprint);
    EXPECT_TRUE(result.kinematic_feasible);
    EXPECT_LE(result.max_kinematic_position_residual, 1e-12);
    EXPECT_LE(result.max_kinematic_heading_residual_deg, 1e-10);
    EXPECT_LE(result.max_kinematic_velocity_residual, 1e-12);
    EXPECT_LE(result.max_kinematic_steer_residual, 1e-12);
    EXPECT_TRUE(result.collision_safe);
    EXPECT_TRUE(result.all_passed);
}

// 测试场景：匀速圆弧轨迹是"已知运动学可行"的合成基准。x/y 梯形残差为弦弧
// 截断误差而非不可行证据：Δθ=ω·dt≈0.0077 rad 时理论量级 R·Δθ³/24≈2e-7 m。
// 预期行为：kinematic_feasible=true；位置残差落在截断误差量级（<1e-5 m，
// 既防止校验把截断误差误判为不可行，也防止实现把残差错误算成 0），
// 其余三项残差不超过 1e-10。
TEST(TrajectoryKinematicValidationTest,
     UniformCircularArcPassesWithTruncationScaleResidual) {
    const auto traj = BuildCircularArcTrajectory();
    const auto esdf = MakeEmptyEsdfMap();
    const auto footprint = MakeKinematicFootprintModel();
    const TrajectoryPoint goal(traj.back().x, traj.back().y, traj.back().theta);
    const auto result = traj.validate(goal, esdf, footprint);
    EXPECT_TRUE(result.kinematic_feasible);
    EXPECT_GT(result.max_kinematic_position_residual, 0.0);
    EXPECT_LT(result.max_kinematic_position_residual, 1e-5);
    EXPECT_LE(result.max_kinematic_heading_residual_deg, 1e-10);
    EXPECT_LE(result.max_kinematic_velocity_residual, 1e-12);
    EXPECT_LE(result.max_kinematic_steer_residual, 1e-12);
}

// 测试场景：匀加速直线轨迹（v 线性变化、a 恒值）的 v/x 梯形残差解析上恒为 0。
// 预期行为：kinematic_feasible=true，速度与位置残差均不超过 1e-12。
TEST(TrajectoryKinematicValidationTest,
     ConstantAccelerationPassesWithZeroResidual) {
    const auto traj = BuildConstantAccelTrajectory();
    const auto esdf = MakeEmptyEsdfMap();
    const auto footprint = MakeKinematicFootprintModel();
    const TrajectoryPoint goal(traj.back().x, 0.0, 0.0);
    const auto result = traj.validate(goal, esdf, footprint);
    EXPECT_TRUE(result.kinematic_feasible);
    EXPECT_LE(result.max_kinematic_position_residual, 1e-12);
    EXPECT_LE(result.max_kinematic_velocity_residual, 1e-12);
}

// 测试场景：人为在中部采样点插入 0.5m 位置跳变（明显违反 ẋ=v·cosθ）。
// 预期行为：kinematic_feasible=false，最大位置残差远超阈值，detail 报告
// position 项超标；即使碰撞/终点两门均通过，all_passed 也必须为 false
// （运动学门真正计入汇总）。
TEST(TrajectoryKinematicValidationTest, PositionJumpIsRejected) {
    auto traj = BuildUniformStraightTrajectory();
    traj[10].x += 0.5;
    const auto esdf = MakeEmptyEsdfMap();
    const auto footprint = MakeKinematicFootprintModel();
    const TrajectoryPoint goal(traj.back().x, 0.0, 0.0);
    const auto result = traj.validate(goal, esdf, footprint);
    EXPECT_FALSE(result.kinematic_feasible);
    EXPECT_GT(result.max_kinematic_position_residual, 0.1);
    EXPECT_NE(result.kinematic_detail.find("position"), std::string::npos);
    EXPECT_TRUE(result.collision_safe);
    EXPECT_TRUE(result.terminal_position_ok);
    EXPECT_FALSE(result.all_passed);
}

// 测试场景：轨迹点逐帧前进但速度恒为 0（"v≡0 但位置持续变化"的自相矛盾
// 状态，与 PIVOT 修复记录的运动学矛盾同源）。
// 预期行为：位置残差等于逐帧位移 0.1m，kinematic_feasible=false。
TEST(TrajectoryKinematicValidationTest, ZeroVelocityButMovingIsRejected) {
    Trajectory traj;
    traj.reserve(kNumPoints);
    for (int k = 0; k < kNumPoints; ++k) {
        traj.push_back(MakeKinematicPoint(k * kDt * 1.0, 0.0, 0.0, 0.0, 0.0,
                                          0.0, 0.0, k * kDt));
    }
    const auto esdf = MakeEmptyEsdfMap();
    const auto footprint = MakeKinematicFootprintModel();
    const TrajectoryPoint goal(traj.back().x, 0.0, 0.0);
    const auto result = traj.validate(goal, esdf, footprint);
    EXPECT_FALSE(result.kinematic_feasible);
    EXPECT_NEAR(result.max_kinematic_position_residual, 0.1, 1e-12);
}

// 测试场景：直线行驶（θ 恒为 0）但前轮转角恒为 0.3 rad（明显违反
// θ̇=v·tanδ/L：按此转角 θ 应持续变化）。取 Δt=0.5s（仍在默认 dt 门
// 上限内）使航向残差 ≈ 3.28° 超过标定阈值 3.0°。
// 预期行为：kinematic_feasible=false，航向残差与解析值一致。
TEST(TrajectoryKinematicValidationTest,
     SteerInconsistentWithHeadingIsRejected) {
    constexpr double kBigDt = 0.5;
    Trajectory traj;
    traj.reserve(kNumPoints);
    for (int k = 0; k < kNumPoints; ++k) {
        traj.push_back(MakeKinematicPoint(k * kBigDt * 1.0, 0.0, 0.0, 1.0, 0.0,
                                          0.3, 0.0, k * kBigDt));
    }
    const auto esdf = MakeEmptyEsdfMap();
    const auto footprint = MakeKinematicFootprintModel();
    const TrajectoryPoint goal(traj.back().x, 0.0, 0.0);
    const auto result = traj.validate(goal, esdf, footprint);
    EXPECT_FALSE(result.kinematic_feasible);
    EXPECT_NEAR(result.max_kinematic_heading_residual_deg,
                kBigDt * 1.0 * std::tan(0.3) / kWheelbase * RAD2DEG, 1e-9);
}

// 测试场景：换挡尖点处的低速转向跳变不误伤。θ-s 参数化下 ṡ→0 时 δ 可在
// atan 值域内跳变（本例停驻点对间 Δδ=±3.0 rad），但此时 v≈0、
// θ̇=v·tanδ/L≈0，δ 与车辆运动解耦，该跳变不携带可行性信号。
// 预期行为：低速跳变点对的前轮转角残差被跳过，kinematic_feasible=true，
// 且最大转角残差保持为 0（其余点对残差均为 0）。
TEST(TrajectoryKinematicValidationTest,
     CuspSteerFlipAtStandstillIsNotPenalized) {
    // v 序列：前进（v=1）→ 停驻 4 点（v=0）→ 后退（v=-1）；a 由递推
    // a_{k+1}=2Δv/Δt−a_k 取得使速度残差恒为 0；x 按梯形增量推进使位置
    // 残差恒为 0；δ 仅在停驻段内部翻转（±1.5 rad），含回归 0 的过渡在内
    // 的全部跳变点对两端 v 均为 0
    std::vector<double> vs(kNumPoints, 0.0), xs(kNumPoints, 0.0),
        as(kNumPoints, 0.0);
    for (int k = 0; k < kNumPoints; ++k) {
        if (k <= 9) {
            vs[k] = 1.0;
        } else if (k >= 14) {
            vs[k] = -1.0;
        }
    }
    for (int k = 1; k < kNumPoints; ++k) {
        xs[k] = xs[k - 1] + 0.5 * kDt * (vs[k - 1] + vs[k]);
        as[k] = 2.0 * (vs[k] - vs[k - 1]) / kDt - as[k - 1];
    }
    Trajectory traj;
    traj.reserve(kNumPoints);
    for (int k = 0; k < kNumPoints; ++k) {
        double delta = 0.0;
        if (k == 11) {
            delta = 1.5;
        } else if (k == 12) {
            delta = -1.5;
        }
        traj.push_back(MakeKinematicPoint(xs[k], 0.0, 0.0, vs[k], as[k], delta,
                                          0.0, k * kDt));
    }
    const auto esdf = MakeEmptyEsdfMap();
    const auto footprint = MakeKinematicFootprintModel();
    const TrajectoryPoint goal(traj.back().x, 0.0, 0.0);
    const auto result = traj.validate(goal, esdf, footprint);
    EXPECT_TRUE(result.kinematic_feasible);
    EXPECT_LE(result.max_kinematic_steer_residual, 1e-12);
    EXPECT_LE(result.max_kinematic_position_residual, 1e-12);
    EXPECT_LE(result.max_kinematic_velocity_residual, 1e-9);
}

// 测试场景（对照）：同样的 δ 全幅跳变若发生在行驶速度下（|v|=1 m/s），
// 必须被转角残差门拒绝——低速跳过不削弱行驶状态下的检出能力。
TEST(TrajectoryKinematicValidationTest, SteerFlipAtDrivingSpeedIsRejected) {
    Trajectory traj;
    traj.reserve(kNumPoints);
    for (int k = 0; k < kNumPoints; ++k) {
        // 匀速直线（θ 恒为 0），δ 在中途从 +1.5 跳到 -1.5
        const double delta = (k <= 10) ? 1.5 : -1.5;
        traj.push_back(MakeKinematicPoint(k * kDt * 1.0, 0.0, 0.0, 1.0, 0.0,
                                          delta, 0.0, k * kDt));
    }
    const auto esdf = MakeEmptyEsdfMap();
    const auto footprint = MakeKinematicFootprintModel();
    const TrajectoryPoint goal(traj.back().x, 0.0, 0.0);
    const auto result = traj.validate(goal, esdf, footprint);
    EXPECT_FALSE(result.kinematic_feasible);
    EXPECT_NEAR(result.max_kinematic_steer_residual, 3.0, 1e-9);
}

// 测试场景：相邻点 Δt 超过 max_kinematic_dt（默认 0.5s）的点对被跳过——
// 长 Δt 点对的梯形截断误差 O(Δt³) 主导残差，不携带可行性判别信号。
// 即使轨迹存在"v≡0 但位置持续变化"的严重矛盾，只要全部点对 Δt 超上限，
// 运动学门也不证伪（该行为的代价已在接口文档登记）。
// 预期行为：kinematic_feasible=true，detail 包含 "skipped"。
TEST(TrajectoryKinematicValidationTest, LongDtPairsAreSkippedByDtGate) {
    Trajectory traj;
    traj.reserve(kNumPoints);
    for (int k = 0; k < kNumPoints; ++k) {
        // v≡0 但每步位置前进 1m、Δt=1.0s（> 默认上限 0.5s）
        traj.push_back(
            MakeKinematicPoint(k * 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, k * 1.0));
    }
    const auto esdf = MakeEmptyEsdfMap();
    const auto footprint = MakeKinematicFootprintModel();
    const TrajectoryPoint goal(traj.back().x, 0.0, 0.0);
    const auto result = traj.validate(goal, esdf, footprint);
    EXPECT_TRUE(result.kinematic_feasible);
    EXPECT_NE(result.kinematic_detail.find("skipped"), std::string::npos);
}

// 测试场景：速度在相邻帧间跳变 1.0 m/s 但加速度恒为 0（违反 v̇=a）。
// 位置增量按两端点速度梯形精确构造，位置/航向残差保持为 0，单独隔离出
// 速度残差这一项。
// 预期行为：速度残差 1.0 m/s，kinematic_feasible=false。
TEST(TrajectoryKinematicValidationTest,
     VelocityJumpWithoutAccelerationIsRejected) {
    Trajectory traj;
    traj.reserve(kNumPoints);
    double x = 0.0;
    double prev_v = 1.0;
    for (int k = 0; k < kNumPoints; ++k) {
        const double v = (k <= 10) ? 1.0 : 2.0;
        if (k > 0) {
            // 梯形一致的位移增量：Δx = Δt/2·(v_{k-1} + v_k)
            x += 0.5 * kDt * (prev_v + v);
        }
        prev_v = v;
        traj.push_back(
            MakeKinematicPoint(x, 0.0, 0.0, v, 0.0, 0.0, 0.0, k * kDt));
    }
    const auto esdf = MakeEmptyEsdfMap();
    const auto footprint = MakeKinematicFootprintModel();
    const TrajectoryPoint goal(traj.back().x, 0.0, 0.0);
    const auto result = traj.validate(goal, esdf, footprint);
    EXPECT_FALSE(result.kinematic_feasible);
    EXPECT_NEAR(result.max_kinematic_velocity_residual, 1.0, 1e-12);
    EXPECT_LE(result.max_kinematic_position_residual, 1e-12);
}

// 测试场景：前轮转角在相邻帧间跳变 0.3 rad 但转角速度恒为 0（违反 δ̇=δdot）。
// 其余量保持匀速直线构造（θ 恒为 0），跳变点处的航向残差
// （≈0.33°，见 SteerInconsistentWithHeadingIsRejected 的标量来源）远小于
// 默认阈值，位置残差为 0，单独隔离出转角残差这一项。
// 预期行为：转角残差 0.3 rad，kinematic_feasible=false。
TEST(TrajectoryKinematicValidationTest, SteerJumpWithoutRateIsRejected) {
    Trajectory traj;
    traj.reserve(kNumPoints);
    for (int k = 0; k < kNumPoints; ++k) {
        const double delta = (k <= 10) ? 0.0 : 0.3;
        traj.push_back(MakeKinematicPoint(k * kDt * 1.0, 0.0, 0.0, 1.0, 0.0,
                                          delta, 0.0, k * kDt));
    }
    const auto esdf = MakeEmptyEsdfMap();
    const auto footprint = MakeKinematicFootprintModel();
    const TrajectoryPoint goal(traj.back().x, 0.0, 0.0);
    const auto result = traj.validate(goal, esdf, footprint);
    EXPECT_FALSE(result.kinematic_feasible);
    EXPECT_NEAR(result.max_kinematic_steer_residual, 0.3, 1e-12);
    EXPECT_LE(result.max_kinematic_position_residual, 1e-12);
}

// 测试场景：轨迹点数 < 2（无相邻点对可比较）。
// 预期行为：运动学门真空通过（没有可证伪的区间），kinematic_feasible=true，
// detail 说明无相邻点对。
TEST(TrajectoryKinematicValidationTest, SinglePointTrajectoryPassesVacuously) {
    Trajectory traj;
    traj.push_back(MakeKinematicPoint(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0));
    const auto esdf = MakeEmptyEsdfMap();
    const auto footprint = MakeKinematicFootprintModel();
    const TrajectoryPoint goal(0.0, 0.0, 0.0);
    const auto result = traj.validate(goal, esdf, footprint);
    EXPECT_TRUE(result.kinematic_feasible);
    EXPECT_NE(result.kinematic_detail.find("pair"), std::string::npos);
}

// 测试场景：时间戳未设置（hasT()==false）时的显式处理——与 validate() 既有的
// 非异常错误处理风格一致，运动学门跳过（不证伪），detail 注明跳过原因。
// 预期行为：kinematic_feasible=true，detail 包含 "skipped"；其它门正常评估。
TEST(TrajectoryKinematicValidationTest, MissingTimestampsSkipsKinematicCheck) {
    Trajectory traj;
    traj.reserve(kNumPoints);
    for (int k = 0; k < kNumPoints; ++k) {
        // 全部运动学量齐全但时间戳缺失
        TrajectoryPoint pt(k * kDt, 0.0, 0.0);
        pt.setV(1.0);
        pt.setA(0.0);
        pt.setDelta(0.0);
        pt.setDeltaDot(0.0);
        traj.push_back(pt);
    }
    const auto esdf = MakeEmptyEsdfMap();
    const auto footprint = MakeKinematicFootprintModel();
    const TrajectoryPoint goal(traj.back().x, 0.0, 0.0);
    const auto result = traj.validate(goal, esdf, footprint);
    EXPECT_TRUE(result.kinematic_feasible);
    EXPECT_NE(result.kinematic_detail.find("skipped"), std::string::npos);
    EXPECT_TRUE(result.all_passed);
}

// 测试场景：空轨迹的既有行为不被破坏。
// 预期行为：运动学门与其余各门一样判定失败，detail 包含 "empty"。
TEST(TrajectoryKinematicValidationTest, EmptyTrajectoryFailsKinematicGate) {
    const Trajectory empty;
    const auto esdf = MakeEmptyEsdfMap();
    const auto footprint = MakeKinematicFootprintModel();
    const TrajectoryPoint goal(5.0, 0.0, 0.0);
    const auto result = empty.validate(goal, esdf, footprint);
    EXPECT_FALSE(result.kinematic_feasible);
    EXPECT_NE(result.kinematic_detail.find("empty"), std::string::npos);
    EXPECT_FALSE(result.all_passed);
}

}  // namespace
}  // namespace apa_post_processor
