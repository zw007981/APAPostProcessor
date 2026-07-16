#include "util/path_validator.h"

#include <gtest/gtest.h>

#include <cmath>

#include "spatial/esdf_map.h"
#include "spatial/grid_map.h"
#include "util/maneuver.h"
#include "util/path.h"
#include "util/trajectory_point.h"
#include "vehicle/vehicle_footprint_model.h"
#include "vehicle/vehicle_params.h"

namespace apa_post_processor {
namespace {

// 测试辅助：不含障碍物的大地图，车辆永远不碰撞
ESDFMap MakeEmptyEsdfMap() {
    const GridMap grid_map(0.1, 400, 300, Position{-10.0, -10.0}, {});
    return ESDFMap(grid_map);
}

// 测试辅助：标准车辆参数
VehicleParams MakeVehicleParams() {
    return VehicleParams(/*length=*/4.3, /*width=*/1.8, /*wheelbase=*/2.7,
                         /*max_steer_angle=*/0.6, /*rear_overhang=*/0.8,
                         /*max_accel=*/1.0, /*max_decel=*/-1.5,
                         /*max_steer_rate=*/0.4);
}

// 测试辅助：标准外圆 footprint 模型
VehicleFootprintModel MakeFootprintModel() {
    return VehicleFootprintModel(MakeVehicleParams(),
                                 /*heading_sample_num=*/233,
                                 /*inner_row_num=*/2, /*outer_row_num=*/1);
}

// 测试辅助：构造沿 x 轴从 (0,0,0) 到 (5,0,0) 的长直路径
Path MakeStraightPath() {
    Path path;
    for (double x = 0.0; x <= 5.0 + 1e-9; x += 0.1) {
        path.addPoint(TrajectoryPoint{std::min(x, 5.0), 0.0, 0.0});
    }
    path.finalize();
    return path;
}

// ===== 空路径：所有门禁应失败 =====

// 测试场景：空路径不合法的边界条件。
// 预期行为：collision_safe/terminal_position_ok/terminal_heading_ok 全部为
// false，
//   all_passed 为 false，detail 字段包含 "path is empty"。
TEST(PathValidatorTest, EmptyPathAllGatesFail) {
    const Path empty;
    const auto esdf = MakeEmptyEsdfMap();
    const auto footprint = MakeFootprintModel();
    const TrajectoryPoint goal(5.0, 0.0, 0.0);
    const auto result = ValidatePath(empty, goal, esdf, footprint);
    EXPECT_FALSE(result.collision_safe);
    EXPECT_FALSE(result.terminal_position_ok);
    EXPECT_FALSE(result.terminal_heading_ok);
    EXPECT_FALSE(result.all_passed);
    EXPECT_NE(result.collision_detail.find("empty"), std::string::npos);
    EXPECT_NE(result.terminal_position_detail.find("empty"), std::string::npos);
    EXPECT_NE(result.terminal_heading_detail.find("empty"), std::string::npos);
}

// ===== 碰撞安全：无障碍地图中长直路径应通过 =====

// 测试场景：车辆在大范围空地中沿 x 轴行驶，所有外圆圆心到最近障碍物距离远大于
//   外圆半径，碰撞深度应为 0。
// 预期行为：collision_safe=true，max_intrusion_depth≈0。
TEST(PathValidatorTest, StraightPathInEmptyMapCollisionSafe) {
    const auto path = MakeStraightPath();
    const auto esdf = MakeEmptyEsdfMap();
    const auto footprint = MakeFootprintModel();
    const TrajectoryPoint goal(5.0, 0.0, 0.0);
    const auto result = ValidatePath(path, goal, esdf, footprint);
    EXPECT_TRUE(result.collision_safe);
    EXPECT_LE(result.max_intrusion_depth, 0.0);
}

// ===== 终点收敛：终点恰好匹配目标 → 通过 =====

// 测试场景：路径终点与目标位姿完全一致。
// 预期行为：terminal_position_ok=true、terminal_heading_ok=true，
//   terminal_position_error≈0、terminal_heading_error_deg≈0。
TEST(PathValidatorTest, ExactTerminalMatchPasses) {
    const auto path = MakeStraightPath();
    const auto esdf = MakeEmptyEsdfMap();
    const auto footprint = MakeFootprintModel();
    const TrajectoryPoint goal(5.0, 0.0, 0.0);
    const auto result = ValidatePath(path, goal, esdf, footprint);
    EXPECT_TRUE(result.terminal_position_ok);
    EXPECT_TRUE(result.terminal_heading_ok);
    EXPECT_NEAR(result.terminal_position_error, 0.0, 1e-6);
    EXPECT_NEAR(result.terminal_heading_error_deg, 0.0, 1e-6);
}

// ===== 终点收敛：终点偏离目标 → 不通过 =====

// 测试场景：路径终点在 (5,0,0)，但目标在 (5,1,π/4)，
//   位置误差 1.0m >> 0.05m 阈值，航向误差 45° >> 3° 阈值。
// 预期行为：terminal_position_ok=false、terminal_heading_ok=false。
TEST(PathValidatorTest, TerminalMismatchFails) {
    const auto path = MakeStraightPath();
    const auto esdf = MakeEmptyEsdfMap();
    const auto footprint = MakeFootprintModel();
    // 目标与路径终点 (5,0,0) 有明显偏差
    const TrajectoryPoint goal(5.0, 1.0, M_PI / 4.0);
    const auto result = ValidatePath(path, goal, esdf, footprint);
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

// ===== 终点收敛：终点略微偏离但在门禁内 → 通过 =====

// 测试场景：终点位置误差 0.03m（<0.05m）、航向误差 1.0°（<3.0°）。
// 预期行为：terminal_position_ok=true、terminal_heading_ok=true。
TEST(PathValidatorTest, TerminalSmallDeviationWithinTolerance) {
    const auto esdf = MakeEmptyEsdfMap();
    const auto footprint = MakeFootprintModel();
    // 构造终点为 (5.03, 0.0, 1.0°) 的路径
    Path path;
    for (double x = 0.0; x <= 5.0 + 1e-9; x += 0.1) {
        path.addPoint(TrajectoryPoint{std::min(x, 5.0), 0.0, 0.0});
    }
    // 最后一个点改为略微偏离
    path.getManeuvers().back().points.back() =
        TrajectoryPoint{5.03, 0.0, M_PI / 180.0};
    path.finalize();
    const TrajectoryPoint goal(5.0, 0.0, 0.0);
    const auto result = ValidatePath(path, goal, esdf, footprint);
    EXPECT_TRUE(result.terminal_position_ok);
    EXPECT_TRUE(result.terminal_heading_ok);
}

// ===== 终点收敛：航向角缠绕正确处理（remainder 语义） =====

// 测试场景：终点航向 359°（-1° 的等价表示），目标航向 0°，
//   实际偏差 1°，应在 3° 门禁内通过。
// 预期行为：terminal_heading_ok=true、误差 ≈ 1.0°。
TEST(PathValidatorTest, TerminalHeadingWrapAround) {
    const auto esdf = MakeEmptyEsdfMap();
    const auto footprint = MakeFootprintModel();
    Path path;
    for (double x = 0.0; x <= 5.0 + 1e-9; x += 0.1) {
        path.addPoint(TrajectoryPoint{std::min(x, 5.0), 0.0, 0.0});
    }
    // 终点航向设为 -1°（等价于 359°），目标为 0°
    path.getManeuvers().back().points.back() =
        TrajectoryPoint{5.0, 0.0, -M_PI / 180.0};
    path.finalize();
    const TrajectoryPoint goal(5.0, 0.0, 0.0);
    const auto result = ValidatePath(path, goal, esdf, footprint);
    EXPECT_TRUE(result.terminal_heading_ok);
    EXPECT_NEAR(result.terminal_heading_error_deg, 1.0, 0.01);
}

// ===== 全部通过 =====

// 测试场景：空地长直路径 + 终点精确匹配。
// 预期行为：all_passed=true。
TEST(PathValidatorTest, AllGatesPassOnValidPath) {
    const auto path = MakeStraightPath();
    const auto esdf = MakeEmptyEsdfMap();
    const auto footprint = MakeFootprintModel();
    const TrajectoryPoint goal(5.0, 0.0, 0.0);
    const auto result = ValidatePath(path, goal, esdf, footprint);
    EXPECT_TRUE(result.all_passed);
}

// ===== FormatValidationResult 格式化输出 =====

// 测试场景：验证通过/失败两种情况的格式化字符串。
// 预期行为：通过时以 "[PASS]" 开头；失败时以 "[FAIL" 开头并注明具体超标项。
TEST(PathValidatorTest, FormatPassResult) {
    PathValidationResult r;
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

TEST(PathValidatorTest, FormatFailResult) {
    PathValidationResult r;
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

}  // namespace
}  // namespace apa_post_processor
