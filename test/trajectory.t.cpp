#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include "spatial/esdf_map.h"
#include "spatial/grid_map.h"
#include "util/path.h"
#include "util/time_profile.h"
#include "util/trajectory.h"
#include "vehicle/vehicle_footprint_model.h"
#include "vehicle/vehicle_params.h"

namespace apa_post_processor {
namespace {

// 测试辅助：构造含时间戳的四点轨迹 (0,0,0)→(1,0,0)→(2,0,0)→(3,0,0)，每步 1m/s
Trajectory MakeSimpleTrajectory() {
    Trajectory traj;
    auto pt0 = TrajectoryPoint(0.0, 0.0, 0.0);
    pt0.setT(0.0);
    pt0.setV(1.0);
    traj.push_back(pt0);
    auto pt1 = TrajectoryPoint(1.0, 0.0, 0.0);
    pt1.setT(1.0);
    pt1.setV(1.0);
    traj.push_back(pt1);
    auto pt2 = TrajectoryPoint(2.0, 0.0, 0.0);
    pt2.setT(2.0);
    pt2.setV(1.0);
    traj.push_back(pt2);
    auto pt3 = TrajectoryPoint(3.0, 0.0, 0.0);
    pt3.setT(3.0);
    pt3.setV(1.0);
    traj.push_back(pt3);
    return traj;
}

// ===== 默认构造与空轨迹 =====

// 测试场景：默认构造的 Trajectory 应为空。
// 预期行为：empty()=true、size()=0、front()/back() 抛出异常。
TEST(TrajectoryTest, DefaultConstructedIsEmpty) {
    const Trajectory traj;
    EXPECT_TRUE(traj.empty());
    EXPECT_EQ(traj.size(), 0u);
    EXPECT_THROW(traj.front(), std::runtime_error);
    EXPECT_THROW(traj.back(), std::runtime_error);
}

// ===== 从向量构造 =====

// 测试场景：从 TrajectoryPoint 向量构造轨迹。
// 预期行为：size 正确、首尾点可访问、弧长正确。
TEST(TrajectoryTest, ConstructFromVector) {
    std::vector<TrajectoryPoint> pts{
        TrajectoryPoint(0.0, 0.0, 0.0),
        TrajectoryPoint(1.0, 0.0, 0.0),
        TrajectoryPoint(2.0, 0.0, 0.0),
    };
    const Trajectory traj(std::move(pts));
    EXPECT_EQ(traj.size(), 3u);
    EXPECT_FALSE(traj.empty());
    EXPECT_DOUBLE_EQ(traj.front().x, 0.0);
    EXPECT_DOUBLE_EQ(traj.back().x, 2.0);
    EXPECT_DOUBLE_EQ(traj.length(), 2.0);
}

// ===== clear / reserve =====

// 测试场景：clear 后轨迹为空，reserve 预留容量。
// 预期行为：clear 后 empty()=true、size()=0、length()=0。
TEST(TrajectoryTest, ClearEmptiesTrajectory) {
    auto traj = MakeSimpleTrajectory();
    EXPECT_FALSE(traj.empty());
    traj.clear();
    EXPECT_TRUE(traj.empty());
    EXPECT_EQ(traj.size(), 0u);
    EXPECT_DOUBLE_EQ(traj.length(), 0.0);
}

// 测试场景：reserve 后 capacity 至少为指定值。
TEST(TrajectoryTest, ReserveDoesNotChangeSize) {
    Trajectory traj;
    traj.reserve(100);
    EXPECT_EQ(traj.size(), 0u);
    EXPECT_TRUE(traj.empty());
}

// ===== 元素访问 =====

// 测试场景：通过 front/back/operator[] 访问轨迹点。
// 预期行为：返回值与构造时一致。
TEST(TrajectoryTest, ElementAccessReturnsCorrectPoints) {
    const auto traj = MakeSimpleTrajectory();
    EXPECT_DOUBLE_EQ(traj.front().x, 0.0);
    EXPECT_DOUBLE_EQ(traj.front().y, 0.0);
    EXPECT_DOUBLE_EQ(traj.back().x, 3.0);
    EXPECT_DOUBLE_EQ(traj.back().y, 0.0);
    EXPECT_DOUBLE_EQ(traj[1].x, 1.0);
    EXPECT_DOUBLE_EQ(traj[2].x, 2.0);
}

// ===== 弧长计算 =====

// 测试场景：沿 x 轴直线轨迹弧长 = 3.0m。
// 预期行为：length() 返回 3.0。
TEST(TrajectoryTest, LengthOfStraightLine) {
    const auto traj = MakeSimpleTrajectory();
    EXPECT_DOUBLE_EQ(traj.length(), 3.0);
}

// 测试场景：单点轨迹弧长为 0。
// 预期行为：length() 返回 0.0。
TEST(TrajectoryTest, LengthOfSinglePointIsZero) {
    Trajectory traj;
    traj.push_back(TrajectoryPoint(1.0, 2.0, 0.0));
    EXPECT_DOUBLE_EQ(traj.length(), 0.0);
}

// 测试场景：(0,0)→(3,4) 弧长 = 5.0。
// 预期行为：length() 返回 5.0。
TEST(TrajectoryTest, LengthOfDiagonal) {
    Trajectory traj;
    traj.push_back(TrajectoryPoint(0.0, 0.0, 0.0));
    traj.push_back(TrajectoryPoint(3.0, 4.0, 0.0));
    EXPECT_DOUBLE_EQ(traj.length(), 5.0);
}

// 测试场景：push_back 后弧长缓存失效并重新计算。
// 预期行为：追加点后 length() 正确更新。
TEST(TrajectoryTest, LengthUpdatesAfterPushBack) {
    auto traj = MakeSimpleTrajectory();
    EXPECT_DOUBLE_EQ(traj.length(), 3.0);
    auto pt = TrajectoryPoint(4.0, 0.0, 0.0);
    pt.setT(4.0);
    traj.push_back(pt);
    EXPECT_DOUBLE_EQ(traj.length(), 4.0);
}

// ===== 时长计算 =====

// 测试场景：首点 t=0.0、末点 t=3.0，时长为 3.0s。
// 预期行为：duration() 返回 3.0。
TEST(TrajectoryTest, DurationComputesCorrectly) {
    const auto traj = MakeSimpleTrajectory();
    EXPECT_DOUBLE_EQ(traj.duration(), 3.0);
}

// 测试场景：时间戳未设置的轨迹时长应为 0。
// 预期行为：duration() 返回 0.0。
TEST(TrajectoryTest, DurationWithoutTimestampsIsZero) {
    Trajectory traj;
    traj.push_back(TrajectoryPoint(0.0, 0.0, 0.0));
    traj.push_back(TrajectoryPoint(1.0, 0.0, 0.0));
    EXPECT_DOUBLE_EQ(traj.duration(), 0.0);
}

// 测试场景：单点轨迹时长为 0。
// 预期行为：duration() 返回 0.0。
TEST(TrajectoryTest, DurationOfSinglePointIsZero) {
    Trajectory traj;
    auto pt = TrajectoryPoint(1.0, 2.0, 0.0);
    pt.setT(5.0);
    traj.push_back(pt);
    EXPECT_DOUBLE_EQ(traj.duration(), 0.0);
}

// ===== 迭代器 =====

// 测试场景：通过范围 for 遍历所有点并累加 x 坐标。
// 预期行为：sum_x = 0+1+2+3 = 6.0。
TEST(TrajectoryTest, RangeBasedForIteration) {
    const auto traj = MakeSimpleTrajectory();
    double sum_x = 0.0;
    for (const auto& pt : traj) {
        sum_x += pt.x;
    }
    EXPECT_DOUBLE_EQ(sum_x, 6.0);
}

// 测试场景：通过 cbegin/cend 遍历所有点。
// 预期行为：遍历点数为 4。
TEST(TrajectoryTest, ConstIteratorCount) {
    const auto traj = MakeSimpleTrajectory();
    std::size_t count = 0;
    for (auto it = traj.cbegin(); it != traj.cend(); ++it) {
        ++count;
    }
    EXPECT_EQ(count, 4u);
}

// ===== emplace_back =====

// 测试场景：emplace_back 就地构造轨迹点。
// 预期行为：点正确加入，size 增加。
TEST(TrajectoryTest, EmplaceBackAddsPoint) {
    Trajectory traj;
    traj.emplace_back(1.0, 2.0, 0.5);
    EXPECT_EQ(traj.size(), 1u);
    EXPECT_DOUBLE_EQ(traj.front().x, 1.0);
    EXPECT_DOUBLE_EQ(traj.front().y, 2.0);
    EXPECT_DOUBLE_EQ(traj.front().theta, 0.5);
}

// ===== points() 只读访问 =====

// 测试场景：points() 返回内部向量的只读引用。
// 预期行为：返回向量与轨迹内容一致。
TEST(TrajectoryTest, PointsAccessorReturnsInternalVector) {
    const auto traj = MakeSimpleTrajectory();
    const auto& pts = traj.points();
    EXPECT_EQ(pts.size(), 4u);
    EXPECT_DOUBLE_EQ(pts[0].x, 0.0);
    EXPECT_DOUBLE_EQ(pts[3].x, 3.0);
}

// ===== toString =====

// 测试场景：toString 包含 size、length、duration 信息。
// 预期行为：返回字符串包含 Trajectory 关键字与数值。
TEST(TrajectoryTest, ToStringContainsKeyInfo) {
    const auto traj = MakeSimpleTrajectory();
    const auto s = traj.toString();
    EXPECT_NE(s.find("Trajectory"), std::string::npos);
    EXPECT_NE(s.find("size=4"), std::string::npos);
    EXPECT_NE(s.find("length=3"), std::string::npos);
    EXPECT_NE(s.find("duration=3"), std::string::npos);
}

// ===== 移动语义 =====

// 测试场景：移动构造后原轨迹为空。
// 预期行为：移动后 new_traj.size()=4、traj.empty()=true。
TEST(TrajectoryTest, MoveConstructorTransfersOwnership) {
    auto traj = MakeSimpleTrajectory();
    const Trajectory new_traj(std::move(traj));
    EXPECT_EQ(new_traj.size(), 4u);
    EXPECT_TRUE(traj.empty());
    EXPECT_DOUBLE_EQ(new_traj.length(), 3.0);
}

// ===== 由 Path 构造（微分平坦补全） =====

// 测试辅助：Path 构造用车辆参数（轴距 2.8m）
VehicleParams MakeRefTrajVehicleParams() {
    return VehicleParams(/*length=*/4.8, /*width=*/1.9, /*wheelbase=*/2.8,
                         /*max_steer_angle=*/0.65);
}

// 测试辅助：向 Path 追加沿 x 轴的点列（theta 固定），从 x_from 到 x_to，
// 步长 0.05m（与前端路径点距同量级）
void AppendXLinePoints(Path* path, double x_from, double x_to, double theta) {
    const int count =
        static_cast<int>(std::round(std::abs(x_to - x_from) / 0.05));
    for (int i = 1; i <= count; ++i) {
        path->addPoint({x_from + (x_to - x_from) * i / count, 0.0, theta});
    }
}

// 测试辅助：向 Path 追加从 (x0,y0) 出发、初始切向沿 +x 的圆弧点列
// （半径 radius、每步 dtheta rad、heading = 切向 + heading_offset），
// 首点 (x0,y0) 由调用方先行 addPoint
void AppendArcPoints(Path* path, double x0, double y0, double radius,
                     double dtheta, int count, double heading_offset) {
    for (int i = 1; i <= count; ++i) {
        const double theta = dtheta * i;
        path->addPoint({x0 + radius * std::sin(theta),
                        y0 + radius * (1.0 - std::cos(theta)),
                        theta + heading_offset});
    }
}

// 测试场景：空 Path 经微分平坦补全构造轨迹。
// 预期行为：产出空轨迹，不抛异常。
TEST(TrajectoryTest, FromPathEmptyPathYieldsEmptyTrajectory) {
    const Path path;
    const Trajectory traj(path, MakeRefTrajVehicleParams());
    EXPECT_TRUE(traj.empty());
    EXPECT_EQ(traj.size(), 0u);
}

// 测试场景：非法运动学输入（轴距非法、时间参数化配置非法）。
// 预期行为：一律抛 std::invalid_argument。
TEST(TrajectoryTest, FromPathRejectsInvalidKinematicInputs) {
    Path path;
    path.addPoint({0.0, 0.0, 0.0});
    AppendXLinePoints(&path, 0.0, 1.0, 0.0);
    path.finalize();
    // 默认构造的 VehicleParams 轴距为 0，非法
    EXPECT_THROW(Trajectory(path, VehicleParams{}), std::invalid_argument);
    // 时间参数化配置非法
    TimeProfileConfig bad_config;
    bad_config.max_v_forward = 0.0;
    EXPECT_THROW(Trajectory(path, MakeRefTrajVehicleParams(), bad_config),
                 std::invalid_argument);
}

// 测试场景：沿 x 轴前进直线（κ=0），梯形加减速时间参数化。
// 预期行为：κ=δ=δ̇=0；首末 v=0、内部 v>0 且不超限速平台；t 自 0 严格
// 递增；时长不短于纯巡航时间（"最快走完"下界）。
TEST(TrajectoryTest, FromPathStraightForwardFillsKinematics) {
    Path path;
    path.addPoint({0.0, 0.0, 0.0});
    AppendXLinePoints(&path, 0.0, 2.0, 0.0);
    path.finalize();
    const Trajectory traj(path, MakeRefTrajVehicleParams());
    ASSERT_EQ(traj.size(), path.size());
    EXPECT_DOUBLE_EQ(traj.front().getT(), 0.0);
    EXPECT_DOUBLE_EQ(traj.front().getV(), 0.0);
    EXPECT_DOUBLE_EQ(traj.back().getV(), 0.0);
    double max_v = 0.0;
    for (const auto& pt : traj) {
        EXPECT_TRUE(pt.hasV());
        EXPECT_TRUE(pt.hasA());
        EXPECT_TRUE(pt.hasDelta());
        EXPECT_TRUE(pt.hasDeltaDot());
        EXPECT_TRUE(pt.hasKappa());
        EXPECT_TRUE(pt.hasT());
        EXPECT_NEAR(pt.getKappa(), 0.0, 1e-12);
        EXPECT_NEAR(pt.getDelta(), 0.0, 1e-12);
        EXPECT_NEAR(pt.getDeltaDot(), 0.0, 1e-12);
        max_v = std::max(max_v, pt.getV());
    }
    for (std::size_t i = 1; i + 1 < traj.size(); ++i) {
        EXPECT_GT(traj[i].getV(), 0.0);
    }
    EXPECT_LE(max_v, TimeProfileConfig{}.max_v_forward + 1e-9);
    for (std::size_t i = 1; i < traj.size(); ++i) {
        EXPECT_GT(traj[i].getT(), traj[i - 1].getT());
    }
    EXPECT_GE(traj.duration(), 2.0 / TimeProfileConfig{}.max_v_forward);
    EXPECT_NEAR(traj.length(), 2.0, 1e-9);
}

// 测试场景：半径 10m 的前进圆弧（恒定曲率）。
// 预期行为：κ≈1/R、δ≈atan(L/R)（微分平坦关系）、δ̇≈0；v 首末为 0、
// 内部为正；时长为正。
TEST(TrajectoryTest, FromPathForwardArcFillsDeltaFromFlatness) {
    Path path;
    path.addPoint({0.0, 0.0, 0.0});
    constexpr double kRadius = 10.0;
    AppendArcPoints(&path, 0.0, 0.0, kRadius, 0.01, 30,
                    /*heading_offset=*/0.0);
    path.finalize();
    const auto params = MakeRefTrajVehicleParams();
    const Trajectory traj(path, params);
    ASSERT_EQ(traj.size(), path.size());
    const double expect_delta = std::atan(params.wheelbase / kRadius);
    EXPECT_DOUBLE_EQ(traj.front().getV(), 0.0);
    EXPECT_DOUBLE_EQ(traj.back().getV(), 0.0);
    for (std::size_t i = 1; i + 1 < traj.size(); ++i) {
        EXPECT_GT(traj[i].getV(), 0.0);
    }
    for (const auto& pt : traj) {
        EXPECT_NEAR(pt.getKappa(), 1.0 / kRadius, 1e-4);
        EXPECT_NEAR(pt.getDelta(), expect_delta, 1e-4);
        EXPECT_NEAR(pt.getDeltaDot(), 0.0, 1e-6);
    }
    EXPECT_GT(traj.duration(), 0.0);
}

// 测试场景：半径 10m 的倒车圆弧（航向 = 切向 + π，几何曲率 κ_geom>0）。
// 预期行为：识别为 BACKWARD，内部 v 恒负、首末为 0；运动方向签名曲率与
// δ 取负（σ=-1 ⇒ κ=-1/R、δ=atan(-L/R)），δ̇≈0。
TEST(TrajectoryTest, FromPathBackwardArcFlipsSteerSign) {
    Path path;
    path.addPoint({0.0, 0.0, PI});
    AppendArcPoints(&path, 0.0, 0.0, 10.0, 0.01, 30,
                    /*heading_offset=*/PI);
    path.finalize();
    ASSERT_EQ(path.numManeuvers(), 1u);
    ASSERT_EQ(path.getManeuvers().front().direction, Direction::BACKWARD);
    const auto params = MakeRefTrajVehicleParams();
    const Trajectory traj(path, params);
    ASSERT_EQ(traj.size(), path.size());
    EXPECT_DOUBLE_EQ(traj.front().getV(), 0.0);
    EXPECT_DOUBLE_EQ(traj.back().getV(), 0.0);
    for (std::size_t i = 1; i + 1 < traj.size(); ++i) {
        EXPECT_LT(traj[i].getV(), 0.0);
    }
    for (const auto& pt : traj) {
        EXPECT_NEAR(pt.getKappa(), -0.1, 1e-4);
        EXPECT_NEAR(pt.getDelta(), std::atan(-params.wheelbase / 10.0), 1e-4);
        EXPECT_NEAR(pt.getDeltaDot(), 0.0, 1e-6);
    }
}

// 测试场景：前进 1.0m 再后退 0.5m 的单次换挡路径（2 个机动段）。
// 预期行为：换挡边界重复点只发射一次（size 与 Path::size() 一致）；首末
// 与换挡边界点 v=0；段内离开端点后 v 按机动段变号；t 不减且时长为正。
TEST(TrajectoryTest, FromPathGearShiftAssignsVelocitySignPerManeuver) {
    Path path;
    path.addPoint({0.0, 0.0, 0.0});
    AppendXLinePoints(&path, 0.0, 1.0, 0.0);
    AppendXLinePoints(&path, 1.0, 0.5, 0.0);
    path.finalize();
    ASSERT_EQ(path.numManeuvers(), 2u);
    const Trajectory traj(path, MakeRefTrajVehicleParams());
    ASSERT_EQ(traj.size(), path.size());
    const std::size_t boundary = path.getManeuvers().front().points.size() - 1;
    EXPECT_DOUBLE_EQ(traj[boundary].x, 1.0);
    EXPECT_DOUBLE_EQ(traj.front().getV(), 0.0);
    EXPECT_DOUBLE_EQ(traj[boundary].getV(), 0.0);
    EXPECT_DOUBLE_EQ(traj.back().getV(), 0.0);
    EXPECT_GT(traj[1].getV(), 0.0);
    EXPECT_LT(traj[boundary + 2].getV(), 0.0);
    for (std::size_t i = 1; i < traj.size(); ++i) {
        EXPECT_GE(traj[i].getT(), traj[i - 1].getT());
    }
    EXPECT_GT(traj.duration(), 0.0);
}

// 测试场景：直线接圆弧的曲率过渡路径（κ 由 0 单调升至 0.1）。
// 预期行为：δ̇=dδ/dt 在交界附近取正值，稳态弧段末端回归 0。
TEST(TrajectoryTest, FromPathSteerRateTracksCurvatureVariation) {
    Path path;
    path.addPoint({0.0, 0.0, 0.0});
    AppendXLinePoints(&path, 0.0, 1.0, 0.0);
    AppendArcPoints(&path, 1.0, 0.0, 10.0, 0.01, 20, /*heading_offset=*/0.0);
    path.finalize();
    const Trajectory traj(path, MakeRefTrajVehicleParams());
    ASSERT_EQ(traj.size(), path.size());
    double max_delta_dot = 0.0;
    for (const auto& pt : traj) {
        max_delta_dot = std::max(max_delta_dot, pt.getDeltaDot());
    }
    EXPECT_GT(max_delta_dot, 0.0);
    EXPECT_NEAR(traj.back().getDeltaDot(), 0.0, 1e-3);
}

// 测试场景：路径未 finalize（路径点 κ 未设置）时构造轨迹。
// 预期行为：κ 按 0 回退处理，δ 随之恒为 0，不抛异常。
TEST(TrajectoryTest, FromPathUnfinalizedPathTreatsKappaAsZero) {
    Path path;
    path.addPoint({0.0, 0.0, 0.0});
    AppendArcPoints(&path, 0.0, 0.0, 10.0, 0.01, 30, /*heading_offset=*/0.0);
    // 故意不调用 finalize()
    const Trajectory traj(path, MakeRefTrajVehicleParams());
    ASSERT_EQ(traj.size(), path.size());
    for (const auto& pt : traj) {
        EXPECT_DOUBLE_EQ(pt.getKappa(), 0.0);
        EXPECT_DOUBLE_EQ(pt.getDelta(), 0.0);
    }
}

// ===== 物理方向段数（countDirectionRuns） =====

// 测试辅助：由 (x, v) 序列构造轨迹（位移与速度是统计的全部输入）
Trajectory MakeVelocityTrajectory(
    const std::vector<std::pair<double, double>>& xvs) {
    Trajectory traj;
    traj.reserve(xvs.size());
    for (const auto& [x, v] : xvs) {
        TrajectoryPoint pt(x, 0.0, 0.0);
        pt.setV(v);
        traj.push_back(pt);
    }
    return traj;
}

// 测试场景：物理方向段数统计——按明确符号切段后丢弃位移不足的抖动段；
// 停驻点不改变方向状态、不产生段边界。
// 预期行为：各序列给出表中段数。
TEST(TrajectoryTest, CountDirectionRunsIgnoresStandstills) {
    // 空轨迹与未设置 v 的轨迹
    EXPECT_EQ(Trajectory{}.countDirectionRuns(), 0);
    // 全正/全负：1 段
    EXPECT_EQ(MakeVelocityTrajectory({{0.0, 0.5}, {0.1, 1.0}, {0.2, 0.8}})
                  .countDirectionRuns(),
              1);
    EXPECT_EQ(
        MakeVelocityTrajectory({{0.0, -0.5}, {0.1, -1.0}}).countDirectionRuns(),
        1);
    // 正-负-正-负：4 段
    EXPECT_EQ(MakeVelocityTrajectory(
                  {{0.0, 0.5}, {0.1, -0.3}, {0.2, 0.2}, {0.3, -0.4}})
                  .countDirectionRuns(),
              4);
    // 换挡停驻不产生额外段：正-停-负为 2 段
    EXPECT_EQ(MakeVelocityTrajectory({{0.0, 0.5}, {0.1, 1e-4}, {0.2, -0.5}})
                  .countDirectionRuns(),
              2);
    // 同向夹停驻仍为 1 段：正-停-正
    EXPECT_EQ(MakeVelocityTrajectory({{0.0, 0.5}, {0.1, 1e-4}, {0.2, 0.3}})
                  .countDirectionRuns(),
              1);
    // 首末停驻不单独成段
    EXPECT_EQ(
        MakeVelocityTrajectory({{0.0, 0.0}, {0.1, 0.5}, {0.2, 0.3}, {0.3, 0.0}})
            .countDirectionRuns(),
        1);
    EXPECT_EQ(
        MakeVelocityTrajectory(
            {{0.0, 1e-4}, {0.1, -0.5}, {0.2, -0.3}, {0.3, 1e-4}, {0.4, 0.5}})
            .countDirectionRuns(),
        2);
    // 全部停驻：0 段（无任何明确运动方向）
    EXPECT_EQ(MakeVelocityTrajectory({{0.0, 0.0}, {0.1, 1e-4}, {0.2, 0.0}})
                  .countDirectionRuns(),
              0);
    // 自定义停驻阈值
    EXPECT_EQ(MakeVelocityTrajectory({{0.0, 0.04}, {0.1, -0.04}})
                  .countDirectionRuns(0.05),
              0);
}

// 测试场景：低速数值抖动过滤——反号 excursion 位移不足 min_arc 时判为
// 抖动丢弃（离散求解器停驻区 ±cm/s 毛刺的典型形态），位移足够的真实
// 小幅度段保留。
// 预期行为：毛刺不计段，真实段保留。
TEST(TrajectoryTest, CountDirectionRunsFiltersDisplacementJitter) {
    // 前进主段中夹 0.01m 位移的后退毛刺：毛刺丢弃，合并为 1 段
    EXPECT_EQ(MakeVelocityTrajectory(
                  {{0.0, 0.5}, {0.1, 0.5}, {0.11, -0.5}, {0.12, 0.5}})
                  .countDirectionRuns(),
              1);
    // 0.06m 位移的后退 excursion 与其后 0.07m 位移的前进段均为真实
    // 小幅度段：计为 3 段
    EXPECT_EQ(MakeVelocityTrajectory({{0.0, 0.5},
                                      {0.1, 0.5},
                                      {0.13, -0.5},
                                      {0.16, -0.5},
                                      {0.17, 0.5},
                                      {0.23, 0.5}})
                  .countDirectionRuns(),
              3);
}

// ====== validate() 验证功能 ======

// 测试辅助：不含障碍物的大地图，车辆永远不碰撞
ESDFMap MakeEmptyEsdfMap() {
    const GridMap grid_map(0.1, 400, 300, Position{-10.0, -10.0}, {});
    return ESDFMap(grid_map);
}

// 测试辅助：标准车辆参数
VehicleParams MakeValidationVehicleParams() {
    return VehicleParams(/*length=*/4.3, /*width=*/1.8, /*wheelbase=*/2.7,
                         /*max_steer_angle=*/0.6, /*rear_overhang=*/0.8,
                         /*max_accel=*/1.0, /*max_decel=*/-1.5,
                         /*max_steer_rate=*/0.4);
}

// 测试辅助：标准外圆 footprint 模型
VehicleFootprintModel MakeValidationFootprintModel() {
    return VehicleFootprintModel(MakeValidationVehicleParams(),
                                 /*heading_sample_num=*/233,
                                 /*inner_row_num=*/2, /*outer_row_num=*/1);
}

// 测试辅助：构造沿 x 轴从 (0,0,0) 到 (5,0,0) 的长直轨迹
Trajectory MakeStraightTrajectory() {
    Trajectory traj;
    for (double x = 0.0; x <= 5.0 + 1e-9; x += 0.1) {
        traj.push_back(TrajectoryPoint{std::min(x, 5.0), 0.0, 0.0});
    }
    return traj;
}

// 测试场景：空轨迹不合法的边界条件。
// 预期行为：collision_safe/terminal_position_ok/terminal_heading_ok 全部为
// false，all_passed 为 false，detail 字段包含 "trajectory is empty"。
TEST(TrajectoryTest, ValidateEmptyTrajectoryAllGatesFail) {
    const Trajectory empty;
    const auto esdf = MakeEmptyEsdfMap();
    const auto footprint = MakeValidationFootprintModel();
    const TrajectoryPoint goal(5.0, 0.0, 0.0);
    const auto result = empty.validate(goal, esdf, footprint);
    EXPECT_FALSE(result.collision_safe);
    EXPECT_FALSE(result.terminal_position_ok);
    EXPECT_FALSE(result.terminal_heading_ok);
    EXPECT_FALSE(result.all_passed);
    EXPECT_NE(result.collision_detail.find("empty"), std::string::npos);
    EXPECT_NE(result.terminal_position_detail.find("empty"), std::string::npos);
    EXPECT_NE(result.terminal_heading_detail.find("empty"), std::string::npos);
}

// 测试场景：车辆在大范围空地中沿 x 轴行驶，碰撞深度应为 0。
// 预期行为：collision_safe=true，max_intrusion_depth≈0。
TEST(TrajectoryTest, ValidateStraightTrajectoryCollisionSafe) {
    const auto traj = MakeStraightTrajectory();
    const auto esdf = MakeEmptyEsdfMap();
    const auto footprint = MakeValidationFootprintModel();
    const TrajectoryPoint goal(5.0, 0.0, 0.0);
    const auto result = traj.validate(goal, esdf, footprint);
    EXPECT_TRUE(result.collision_safe);
    EXPECT_LE(result.max_intrusion_depth, 0.0);
}

// 测试场景：轨迹终点与目标位姿完全一致。
// 预期行为：terminal_position_ok=true、terminal_heading_ok=true。
TEST(TrajectoryTest, ValidateExactTerminalMatchPasses) {
    const auto traj = MakeStraightTrajectory();
    const auto esdf = MakeEmptyEsdfMap();
    const auto footprint = MakeValidationFootprintModel();
    const TrajectoryPoint goal(5.0, 0.0, 0.0);
    const auto result = traj.validate(goal, esdf, footprint);
    EXPECT_TRUE(result.terminal_position_ok);
    EXPECT_TRUE(result.terminal_heading_ok);
    EXPECT_NEAR(result.terminal_position_error, 0.0, 1e-6);
    EXPECT_NEAR(result.terminal_heading_error_deg, 0.0, 1e-6);
}

// 测试场景：轨迹终点在 (5,0,0)，但目标在 (5,1,π/4)，位置误差 1.0m、航向误差
// 45°。 预期行为：terminal_position_ok=false、terminal_heading_ok=false。
TEST(TrajectoryTest, ValidateTerminalMismatchFails) {
    const auto traj = MakeStraightTrajectory();
    const auto esdf = MakeEmptyEsdfMap();
    const auto footprint = MakeValidationFootprintModel();
    const TrajectoryPoint goal(5.0, 1.0, M_PI / 4.0);
    const auto result = traj.validate(goal, esdf, footprint);
    EXPECT_FALSE(result.terminal_position_ok);
    EXPECT_FALSE(result.terminal_heading_ok);
    EXPECT_FALSE(result.all_passed);
    EXPECT_NEAR(result.terminal_position_error, 1.0, 1e-6);
    EXPECT_NEAR(result.terminal_heading_error_deg, 45.0, 1e-2);
    EXPECT_NE(result.terminal_position_detail.find("exceeds"),
              std::string::npos);
    EXPECT_NE(result.terminal_heading_detail.find("exceeds"),
              std::string::npos);
}

// 测试场景：终点位置误差 0.03m（<0.05m）、航向误差 1.0°（<3.0°）。
// 预期行为：terminal_position_ok=true、terminal_heading_ok=true。
TEST(TrajectoryTest, ValidateTerminalSmallDeviationWithinTolerance) {
    const auto esdf = MakeEmptyEsdfMap();
    const auto footprint = MakeValidationFootprintModel();
    Trajectory traj;
    for (double x = 0.0; x <= 5.0 + 1e-9; x += 0.1) {
        traj.push_back(TrajectoryPoint{std::min(x, 5.0), 0.0, 0.0});
    }
    traj.back() = TrajectoryPoint{5.03, 0.0, M_PI / 180.0};
    const TrajectoryPoint goal(5.0, 0.0, 0.0);
    const auto result = traj.validate(goal, esdf, footprint);
    EXPECT_TRUE(result.terminal_position_ok);
    EXPECT_TRUE(result.terminal_heading_ok);
}

// 测试场景：终点航向 -1°（等价于 359°），目标航向 0°，偏差 1°。
// 预期行为：terminal_heading_ok=true、误差 ≈ 1.0°。
TEST(TrajectoryTest, ValidateTerminalHeadingWrapAround) {
    const auto esdf = MakeEmptyEsdfMap();
    const auto footprint = MakeValidationFootprintModel();
    Trajectory traj;
    for (double x = 0.0; x <= 5.0 + 1e-9; x += 0.1) {
        traj.push_back(TrajectoryPoint{std::min(x, 5.0), 0.0, 0.0});
    }
    traj.back() = TrajectoryPoint{5.0, 0.0, -M_PI / 180.0};
    const TrajectoryPoint goal(5.0, 0.0, 0.0);
    const auto result = traj.validate(goal, esdf, footprint);
    EXPECT_TRUE(result.terminal_heading_ok);
    EXPECT_NEAR(result.terminal_heading_error_deg, 1.0, 0.01);
}

// 测试场景：空地长直轨迹 + 终点精确匹配。
// 预期行为：all_passed=true。
TEST(TrajectoryTest, ValidateAllGatesPassOnValidTrajectory) {
    const auto traj = MakeStraightTrajectory();
    const auto esdf = MakeEmptyEsdfMap();
    const auto footprint = MakeValidationFootprintModel();
    const TrajectoryPoint goal(5.0, 0.0, 0.0);
    const auto result = traj.validate(goal, esdf, footprint);
    EXPECT_TRUE(result.all_passed);
}

// 测试场景：验证通过时格式化字符串以 "[PASS]" 开头。
TEST(TrajectoryTest, ValidateFormatPassResult) {
    TrajectoryValidationResult r;
    r.all_passed = true;
    r.collision_safe = true;
    r.terminal_position_ok = true;
    r.terminal_heading_ok = true;
    r.max_intrusion_depth = 0.0;
    r.terminal_position_error = 0.001;
    r.terminal_heading_error_deg = 0.5;
    const auto s = FormatValidationResult(r);
    EXPECT_NE(s.find("[PASS]"), std::string::npos);
    EXPECT_NE(s.find("collision=0.0000"), std::string::npos);
    EXPECT_NE(s.find("pos_err=0.001"), std::string::npos);
}

// 测试场景：验证失败时格式化字符串以 "[FAIL" 开头并注明超标项。
TEST(TrajectoryTest, ValidateFormatFailResult) {
    TrajectoryValidationResult r;
    r.all_passed = false;
    r.collision_safe = false;
    r.terminal_position_ok = true;
    r.terminal_heading_ok = true;
    r.max_intrusion_depth = 0.031;
    r.terminal_position_error = 0.01;
    r.terminal_heading_error_deg = 0.5;
    const auto s = FormatValidationResult(r);
    EXPECT_NE(s.find("[FAIL"), std::string::npos);
    EXPECT_NE(s.find("collision=0.031"), std::string::npos);
}

// 测试场景：使用更严格的碰撞阈值配置。
// 预期行为：宽松默认通过但严格配置下碰撞失败。
TEST(TrajectoryTest, ValidateCustomCollisionThreshold) {
    const auto traj = MakeStraightTrajectory();
    const auto esdf = MakeEmptyEsdfMap();
    const auto footprint = MakeValidationFootprintModel();
    const TrajectoryPoint goal(5.0, 0.0, 0.0);
    const auto result_default = traj.validate(goal, esdf, footprint);
    EXPECT_TRUE(result_default.collision_safe);
    TrajectoryValidationConfig strict_config;
    strict_config.max_collision_depth = -0.01;
    const auto result_strict =
        traj.validate(goal, esdf, footprint, strict_config);
    EXPECT_FALSE(result_strict.collision_safe);
    EXPECT_GT(result_strict.max_intrusion_depth,
              strict_config.max_collision_depth);
}

}  // namespace
}  // namespace apa_post_processor
