#include "util/trajectory.h"

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include "spatial/esdf_map.h"
#include "spatial/grid_map.h"
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

// 测试场景：轨迹终点在 (5,0,0)，但目标在 (5,1,π/4)，位置误差 1.0m、航向误差 45°。
// 预期行为：terminal_position_ok=false、terminal_heading_ok=false。
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
    EXPECT_NE(result.terminal_position_detail.find("exceeds"), std::string::npos);
    EXPECT_NE(result.terminal_heading_detail.find("exceeds"), std::string::npos);
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
    const auto result_strict = traj.validate(goal, esdf, footprint, strict_config);
    EXPECT_FALSE(result_strict.collision_safe);
    EXPECT_GT(result_strict.max_intrusion_depth, strict_config.max_collision_depth);
}

}  // namespace
}  // namespace apa_post_processor
