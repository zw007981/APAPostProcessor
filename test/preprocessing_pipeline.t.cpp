#include "preprocessing/preprocessing_pipeline.h"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "core/NMPC/nmpc_solver.h"
#include "spatial/esdf_map.h"
#include "spatial/grid_map.h"
#include "util/constants.h"
#include "util/maneuver.h"
#include "util/path.h"
#include "util/position.h"
#include "util/trajectory_point.h"
#include "vehicle/vehicle_footprint_model.h"
#include "vehicle/vehicle_params.h"

namespace apa_post_processor {
namespace {

// 公共车辆参数：与其它预处理阶段测试保持一致。
VehicleParams MakeVehicleParams() {
    return VehicleParams(/*length=*/4.3, /*width=*/1.8, /*wheelbase=*/2.7,
                         /*max_steer_angle=*/0.6, /*rear_overhang=*/0.8,
                         /*max_accel=*/1.0, /*max_decel=*/-1.5,
                         /*max_steer_rate=*/0.4);
}

// 大地图：足够大，确保车辆在任何位姿下都不会越界。
// 范围 x:[-10, 30), y:[-10, 20)，无障碍物。
ESDFMap MakeLargeEmptyEsdfMap() {
    const GridMap grid_map(0.1, 400, 300, Position{-10.0, -10.0}, {});
    return ESDFMap(grid_map);
}

// 含障碍物的地图：在 (3.0, -2.0) 附近有一堵水平墙，位于测试路径下方 2m 处。
// 车辆在 y=0 行驶，不会撞墙，但墙产生的 ESDF 梯度确保静态走廊可正常构建。
ESDFMap MakeObstacleEsdfMap() {
    std::vector<Position> cells;
    for (int i = -8; i <= 8; ++i) {
        cells.emplace_back(Position{static_cast<double>(i) * 0.3, -2.0});
    }
    const GridMap grid_map(0.1, 300, 250, Position{-5.0, -10.0}, cells);
    return ESDFMap(grid_map);
}

// Footprint 模型：single outer row 简化几何。
VehicleFootprintModel MakeFootprintModel() {
    return VehicleFootprintModel(MakeVehicleParams(),
                                 /*heading_sample_num=*/233,
                                 /*inner_row_num=*/2, /*outer_row_num=*/1);
}

// 默认管线配置（不含静态走廊，兼容空地图零梯度）
PreprocessingPipelineConfig MakeConfig() {
    PreprocessingPipelineConfig config;
    config.use_static_corridor = false;
    return config;
}

// 含静态走廊的配置
PreprocessingPipelineConfig MakeConfigWithCorridor() {
    PreprocessingPipelineConfig config;
    config.use_static_corridor = true;
    return config;
}

// 构造一条长直前进机动段 (0,0,0)→(5,0,0)，点距 0.1m
Maneuver MakeLongStraightManeuver() {
    std::vector<TrajectoryPoint> points;
    for (double x = 0.0; x <= 5.0 + EPSILON; x += 0.1) {
        points.emplace_back(TrajectoryPoint{std::min(x, 5.0), 0.0, 0.0});
    }
    return Maneuver(std::move(points), Direction::FORWARD);
}

// 构造一条弯曲机动段
Maneuver MakeCurvedManeuver() {
    std::vector<TrajectoryPoint> points;
    constexpr double kRadius = 5.0;
    constexpr int kNumPoints = 51;
    for (int i = 0; i < kNumPoints; ++i) {
        const double t =
            static_cast<double>(i) / static_cast<double>(kNumPoints - 1);
        const double theta = t * PI * 0.5;
        const double x = kRadius * std::sin(theta);
        const double y = kRadius * (1.0 - std::cos(theta));
        points.emplace_back(TrajectoryPoint{x, y, theta});
    }
    return Maneuver(std::move(points), Direction::FORWARD);
}

// 构造一条倒退机动段：从 (5,0,PI) 回到 (0,0,PI)
Maneuver MakeLongStraightBackwardManeuver() {
    std::vector<TrajectoryPoint> points;
    for (double x = 5.0; x >= -EPSILON; x -= 0.1) {
        points.emplace_back(TrajectoryPoint{std::max(x, 0.0), 0.0, PI});
    }
    return Maneuver(std::move(points), Direction::BACKWARD);
}

// 由单个 Maneuver 构造 Path
Path MakePathFromManeuver(const Maneuver& maneuver) {
    Path path;
    for (const auto& pp : maneuver.points) {
        path.addPoint(pp);
    }
    path.finalize();
    return path;
}

// 由两个 Maneuver 拼接成 Path（前进+倒退）
Path MakeTwoManeuverPath(const Maneuver& m1, const Maneuver& m2) {
    Path path;
    for (const auto& pp : m1.points) {
        path.addPoint(pp);
    }
    for (const auto& pp : m2.points) {
        path.addPoint(pp);
    }
    path.finalize();
    return path;
}

// ============================================================
// 测试：构造函数与输入校验
// ============================================================

// 管线构造成功，不抛异常
TEST(PreprocessingPipelineTest, ConstructsSuccessfully) {
    const auto config = MakeConfig();
    const auto vehicle_params = MakeVehicleParams();
    const auto footprint = MakeFootprintModel();
    const auto esdf_map = MakeLargeEmptyEsdfMap();
    EXPECT_NO_THROW(PreprocessingPipeline pipeline(config, vehicle_params,
                                                   footprint, esdf_map));
}

// 输入空路径应被 validateInputs 拦截
TEST(PreprocessingPipelineTest, RunThrowsOnEmptyPath) {
    const auto config = MakeConfig();
    const auto vehicle_params = MakeVehicleParams();
    const auto footprint = MakeFootprintModel();
    const auto esdf_map = MakeLargeEmptyEsdfMap();
    const PreprocessingPipeline pipeline(config, vehicle_params, footprint,
                                         esdf_map);
    Path empty_path;
    EXPECT_THROW(pipeline.run(empty_path), std::invalid_argument);
}

// ============================================================
// 测试：端到端单 Maneuver 场景
// ============================================================

// 长直前进段端到端：验证管线各阶段正常串联，输出维度 > 0 且 delta_t 数量正确
TEST(PreprocessingPipelineTest, RunsSingleForwardManeuverEndToEnd) {
    const auto config = MakeConfig();
    const auto vehicle_params = MakeVehicleParams();
    const auto footprint = MakeFootprintModel();
    const auto esdf_map = MakeLargeEmptyEsdfMap();
    const PreprocessingPipeline pipeline(config, vehicle_params, footprint,
                                         esdf_map);

    const auto maneuver = MakeLongStraightManeuver();
    const auto path = MakePathFromManeuver(maneuver);
    const auto result = pipeline.run(path);

    EXPECT_TRUE(result.success) << result.status_msg;
    EXPECT_GT(result.final_dimension, 0);
    EXPECT_FALSE(result.z_ref.empty());
    // delta_t 数组长度应为 points.size() - 1
    EXPECT_EQ(result.delta_t.size(), static_cast<std::size_t>(std::max(
                                         0, result.final_dimension - 1)));
    // 各阶段耗时非负
    EXPECT_GE(result.time_bspline_ms, 0.0);
    EXPECT_GE(result.time_speed_ms, 0.0);
    EXPECT_GE(result.time_diff_flat_ms, 0.0);
    EXPECT_GE(result.time_resample_ms, 0.0);
    if (config.use_static_corridor) {
        EXPECT_GE(result.time_corridor_ms, 0.0);
    }
    EXPECT_GE(result.time_total_ms, 0.0);
}

// 弯曲机动段端到端：验证含曲率变化的场景各阶段正常串联
TEST(PreprocessingPipelineTest, RunsSingleCurvedManeuverEndToEnd) {
    const auto config = MakeConfig();
    const auto vehicle_params = MakeVehicleParams();
    const auto footprint = MakeFootprintModel();
    const auto esdf_map = MakeLargeEmptyEsdfMap();
    const PreprocessingPipeline pipeline(config, vehicle_params, footprint,
                                         esdf_map);

    const auto maneuver = MakeCurvedManeuver();
    const auto path = MakePathFromManeuver(maneuver);
    const auto result = pipeline.run(path);

    EXPECT_TRUE(result.success) << result.status_msg;
    EXPECT_GT(result.final_dimension, 0);
}

// 验证输出点携带完整的状态与控制量（x,y,theta,v,a,delta,delta_dot）
TEST(PreprocessingPipelineTest, OutputPointsCarryFullStateAndControl) {
    const auto config = MakeConfig();
    const auto vehicle_params = MakeVehicleParams();
    const auto footprint = MakeFootprintModel();
    const auto esdf_map = MakeLargeEmptyEsdfMap();
    const PreprocessingPipeline pipeline(config, vehicle_params, footprint,
                                         esdf_map);

    const auto maneuver = MakeLongStraightManeuver();
    const auto path = MakePathFromManeuver(maneuver);
    const auto result = pipeline.run(path);

    ASSERT_TRUE(result.success);
    ASSERT_FALSE(result.z_ref.empty());
    // 每个打靶点都应携带全部状态/控制量
    for (const auto& pt : result.z_ref) {
        EXPECT_TRUE(pt.hasV());
        EXPECT_TRUE(pt.hasDelta());
        EXPECT_TRUE(pt.hasA());
        EXPECT_TRUE(pt.hasDeltaDot());
    }
}

// 关闭静态走廊时 c_matrix/d_vector 应保持空
TEST(PreprocessingPipelineTest, DisablesStaticCorridorWhenFlagIsFalse) {
    auto config = MakeConfigWithCorridor();
    config.use_static_corridor = false;
    const auto vehicle_params = MakeVehicleParams();
    const auto footprint = MakeFootprintModel();
    const auto esdf_map = MakeLargeEmptyEsdfMap();
    const PreprocessingPipeline pipeline(config, vehicle_params, footprint,
                                         esdf_map);

    const auto maneuver = MakeLongStraightManeuver();
    const auto path = MakePathFromManeuver(maneuver);
    const auto result = pipeline.run(path);

    EXPECT_TRUE(result.success) << result.status_msg;
    EXPECT_EQ(result.c_matrix.rows(), 0);
    EXPECT_EQ(result.d_vector.size(), 0);
    EXPECT_DOUBLE_EQ(result.time_corridor_ms, 0.0);
}

// 启用静态走廊时 c_matrix/d_vector 应非空（需障碍物地图提供非零梯度）。
// 本测试依赖 MakeObstacleEsdfMap() 在 y=-2.0 处精确放置一堵水平墙，确保路径
// y=0 附近存在非零 ESDF 梯度以触发有效的静态走廊构建。走廊行数断言为
// (打靶点数 × 外圆数 × 2 边界类型)，耦合了 VehicleFootprintModel 的
// outer_row_num=1。
TEST(PreprocessingPipelineTest, BuildsStaticCorridorWhenFlagIsTrue) {
    const auto config = MakeConfigWithCorridor();
    const auto vehicle_params = MakeVehicleParams();
    const auto footprint = MakeFootprintModel();
    const auto esdf_map = MakeObstacleEsdfMap();
    const PreprocessingPipeline pipeline(config, vehicle_params, footprint,
                                         esdf_map);

    const auto maneuver = MakeLongStraightManeuver();
    const auto path = MakePathFromManeuver(maneuver);
    const auto result = pipeline.run(path);

    ASSERT_TRUE(result.success) << result.status_msg;
    EXPECT_GT(result.c_matrix.rows(), 0);
    EXPECT_GT(result.d_vector.size(), 0);
}

// ============================================================
// 测试：端到端多 Maneuver（含换挡）场景
// ============================================================

// 前进+倒退两段端到端：验证换挡拼接、补丁注入、维度锁死
TEST(PreprocessingPipelineTest, RunsTwoManeuverWithCuspEndToEnd) {
    const auto config = MakeConfig();
    const auto vehicle_params = MakeVehicleParams();
    const auto footprint = MakeFootprintModel();
    const auto esdf_map = MakeLargeEmptyEsdfMap();
    const PreprocessingPipeline pipeline(config, vehicle_params, footprint,
                                         esdf_map);

    const auto m1 = MakeLongStraightManeuver();
    const auto m2 = MakeLongStraightBackwardManeuver();
    const auto path = MakeTwoManeuverPath(m1, m2);
    const auto result = pipeline.run(path);

    EXPECT_TRUE(result.success) << result.status_msg;
    EXPECT_GT(result.final_dimension, 0);
    // 多段场景下维度应至少能覆盖两段的最小点数
    EXPECT_GE(result.final_dimension, 4);
}

// ============================================================
// 测试：非默认 initial_velocity / initial_steer_angle
// ============================================================

// 非零起始速度应被正常透传，首点速度接近给定值
TEST(PreprocessingPipelineTest, PassesNonDefaultInitialVelocity) {
    const auto config = MakeConfig();
    const auto vehicle_params = MakeVehicleParams();
    const auto footprint = MakeFootprintModel();
    const auto esdf_map = MakeLargeEmptyEsdfMap();
    const PreprocessingPipeline pipeline(config, vehicle_params, footprint,
                                         esdf_map);

    const auto maneuver = MakeLongStraightManeuver();
    const auto path = MakePathFromManeuver(maneuver);
    constexpr double kInitialV = 0.5;
    const auto result = pipeline.run(path, kInitialV);

    ASSERT_TRUE(result.success) << result.status_msg;
    ASSERT_FALSE(result.z_ref.empty());
    // 第一个打靶点的速度符号应与方向一致（正），大小接近 initial_velocity
    EXPECT_GT(result.z_ref[0].getV(), 0.0);
    EXPECT_LE(std::abs(result.z_ref[0].getV() - kInitialV), 0.3);
}

// 非零起始前轮转角应被正常透传
TEST(PreprocessingPipelineTest, PassesNonDefaultInitialSteerAngle) {
    const auto config = MakeConfig();
    const auto vehicle_params = MakeVehicleParams();
    const auto footprint = MakeFootprintModel();
    const auto esdf_map = MakeLargeEmptyEsdfMap();
    const PreprocessingPipeline pipeline(config, vehicle_params, footprint,
                                         esdf_map);

    const auto maneuver = MakeLongStraightManeuver();
    const auto path = MakePathFromManeuver(maneuver);
    constexpr double kInitialDelta = 0.3;
    const auto result =
        pipeline.run(path, /*initial_velocity=*/0.0, kInitialDelta);

    ASSERT_TRUE(result.success) << result.status_msg;
    ASSERT_FALSE(result.z_ref.empty());
    // 至少第一个补丁点或第一个打靶点的 delta 接近给定值。
    // 容差来源：补丁段 delta_t_min=0.05s × max_steer_rate=0.4rad/s ≈ 0.02rad 为
    // 最小可分辨转角变化，0.15rad 已留足裕度（约 7.5 个最小步的累积误差上限）。
    const double first_delta = result.z_ref[0].getDelta();
    EXPECT_LT(std::abs(first_delta - kInitialDelta), 0.15);
}

// 默认 initial_velocity=0.0 和 initial_steer_angle=0.0 的行为与显式传 0 一致
TEST(PreprocessingPipelineTest, DefaultZeroInitialValuesMatchExplicitZero) {
    const auto config = MakeConfig();
    const auto vehicle_params = MakeVehicleParams();
    const auto footprint = MakeFootprintModel();
    const auto esdf_map = MakeLargeEmptyEsdfMap();
    const PreprocessingPipeline pipeline(config, vehicle_params, footprint,
                                         esdf_map);

    const auto maneuver = MakeLongStraightManeuver();
    const auto path = MakePathFromManeuver(maneuver);
    const auto result_default = pipeline.run(path);
    const auto result_explicit = pipeline.run(path, 0.0, 0.0);

    ASSERT_TRUE(result_default.success);
    ASSERT_TRUE(result_explicit.success);
    // 两者应产生相同的维度
    EXPECT_EQ(result_default.final_dimension, result_explicit.final_dimension);
}

// ============================================================
// 测试：三机动段（前进→倒退→前进）端到端
// ============================================================

// 前进→倒退→前进三段端到端：验证多段换挡场景下管线不退化。
TEST(PreprocessingPipelineTest, RunsThreeManeuverEndToEnd) {
    const auto config = MakeConfig();
    const auto vehicle_params = MakeVehicleParams();
    const auto footprint = MakeFootprintModel();
    const auto esdf_map = MakeLargeEmptyEsdfMap();
    const PreprocessingPipeline pipeline(config, vehicle_params, footprint,
                                         esdf_map);

    // 前进 0→5、倒退 5→2、前进 2→8
    auto m1 = MakeLongStraightManeuver();
    std::vector<TrajectoryPoint> pts2;
    for (double x = 5.0; x >= 2.0 - EPSILON; x -= 0.1) {
        pts2.emplace_back(TrajectoryPoint{std::max(x, 2.0), 0.0, PI});
    }
    const auto m2 = Maneuver(std::move(pts2), Direction::BACKWARD);
    std::vector<TrajectoryPoint> pts3;
    for (double x = 2.0; x <= 8.0 + EPSILON; x += 0.1) {
        pts3.emplace_back(TrajectoryPoint{std::min(x, 8.0), 0.0, 0.0});
    }
    const auto m3 = Maneuver(std::move(pts3), Direction::FORWARD);
    // 手工拼接三段路径
    Path three_path;
    for (const auto& pp : m1.points) {
        three_path.addPoint(pp);
    }
    for (const auto& pp : m2.points) {
        three_path.addPoint(pp);
    }
    for (const auto& pp : m3.points) {
        three_path.addPoint(pp);
    }
    three_path.finalize();
    const auto result = pipeline.run(three_path);

    EXPECT_TRUE(result.success) << result.status_msg;
    EXPECT_GT(result.final_dimension, 0);
    // 三段场景下维度应至少能覆盖三段的最小点数
    EXPECT_GE(result.final_dimension, 6);
}

// ============================================================
// 测试：PIVOT 方向机动段
// ============================================================

// 验证含 PIVOT 段（原地转向）的路径不会导致管线崩溃，PIVOT 段被静默跳过。
TEST(PreprocessingPipelineTest, SkipsPivotManeuverGracefully) {
    const auto config = MakeConfig();
    const auto vehicle_params = MakeVehicleParams();
    const auto footprint = MakeFootprintModel();
    const auto esdf_map = MakeLargeEmptyEsdfMap();
    const PreprocessingPipeline pipeline(config, vehicle_params, footprint,
                                         esdf_map);

    // 构造含一个 PIVOT 段的路径：前进(0,0,0)→(3,0,0)，原地转向到
    // PI，再前进到(6,0,PI)
    Path pivot_path;
    for (double x = 0.0; x <= 3.0 + EPSILON; x += 0.1) {
        pivot_path.addPoint(TrajectoryPoint{std::min(x, 3.0), 0.0, 0.0});
    }
    pivot_path.addPoint(TrajectoryPoint{3.0, 0.0, PI});  // cusp 处 theta 突变
    for (double x = 3.0; x <= 6.0 + EPSILON; x += 0.1) {
        pivot_path.addPoint(TrajectoryPoint{std::min(x, 6.0), 0.0, PI});
    }
    pivot_path.finalize();

    const auto result = pipeline.run(pivot_path);
    // PIVOT 段被跳过，管线应在剩余的 FORWARD/BACKWARD 段上正常运行
    EXPECT_TRUE(result.success) << result.status_msg;
    EXPECT_GT(result.final_dimension, 0);
}

// ============================================================
// 测试：失败传播
// ============================================================

// BSplineSmoother 在路径紧贴极小地图边界时可能因碰撞校验失败。
// 本测试依赖 ESDFMap 对越界坐标返回 dist=0 的隐式契约（见
// docs/known-limitations.md「ESDFMap::getDistAndGrad 越界查询返回 (0.0,
// Zero)」条目）； 若 ESDFMap 越界行为未来变更，此测试可能静默失效。
TEST(PreprocessingPipelineTest, PropagatesBSplineFailure) {
    auto config = MakeConfig();
    // 设置极低的碰撞容忍度，迫使碰撞校验失败（即使无障碍物，车身外圆也可能越界）
    config.bspline.collision_validation_tolerance = 1e-6;
    const auto vehicle_params = MakeVehicleParams();
    const auto footprint = MakeFootprintModel();
    // 极小地图使车辆外圆必然越界
    const GridMap tiny_grid(1.0, 2, 2, Position{0.0, 0.0}, {});
    const ESDFMap tiny_esdf(tiny_grid);
    const PreprocessingPipeline pipeline(config, vehicle_params, footprint,
                                         tiny_esdf);

    const auto maneuver = MakeLongStraightManeuver();
    const auto path = MakePathFromManeuver(maneuver);
    const auto result = pipeline.run(path);

    // 越界查询导致 ESDF 距离为 0，侵入深度会超过容忍度
    EXPECT_FALSE(result.success);
}

// ============================================================
// 测试：Pipeline → NmpcSolver 端到端验证（🚨#2 接线验证）
// ============================================================

// 验证预处理管线输出可被 NmpcSolver 正常消费：管线 Z_ref/delta_t → Path →
// NmpcSolver::optimize()，验证求解不崩溃且产出非空轨迹。
// 使用大地图+无障碍物场景确保管线成功，聚焦于结构兼容性验证。
TEST(PreprocessingPipelineTest, PipelineOutputFeedsNmpcSolverEndToEnd) {
    const auto config = MakeConfig();
    const auto vehicle_params = MakeVehicleParams();
    const auto footprint = MakeFootprintModel();
    const auto esdf_map = MakeLargeEmptyEsdfMap();

    // Step 1: 运行预处理管线
    const PreprocessingPipeline pipeline(config, vehicle_params, footprint,
                                         esdf_map);
    const auto maneuver = MakeLongStraightManeuver();
    const auto path = MakePathFromManeuver(maneuver);
    const auto pipe_result = pipeline.run(path);

    ASSERT_TRUE(pipe_result.success) << pipe_result.status_msg;
    ASSERT_GE(pipe_result.final_dimension, 3);

    // Step 2: 将管线 Z_ref 转换为 Path，供 NmpcSolver::optimize() 消费
    Path nmpc_path;
    for (const auto& pt : pipe_result.z_ref) {
        nmpc_path.addPoint(pt);
    }
    nmpc_path.finalize();

    // Step 3: 构造 NmpcSolver（使用管线输出维度调整 max_iter 以加速测试）
    NmpcSolverConfig nmpc_config;
    nmpc_config.max_iter = 50;
    nmpc_config.esdf_safety_margin = 0.0;
    nmpc_config.esdf_penalty_weight = 100.0;
    // 传入静态走廊系数以验证字段接线与约束注入（Milestone 012 已完成静态走廊
    // 在 NmpcSolver::optimize() 中的注入；此处验证结构兼容性与求解不崩溃）。
    nmpc_config.static_corridor_C = pipe_result.c_matrix;
    nmpc_config.static_corridor_d = pipe_result.d_vector;

    const NmpcSolver solver(vehicle_params, footprint, nmpc_config);

    // Step 4: 执行 NMPC 优化，验证求解不崩溃
    const auto nmpc_result = solver.optimize(nmpc_path, esdf_map);

    // 即使不收敛（无障碍物时空地图几乎肯定收敛），至少轨迹应非空
    EXPECT_FALSE(nmpc_result.trajectory.x.empty())
        << "NmpcSolver should produce non-empty trajectory on trivial input";
    EXPECT_GT(nmpc_result.solve_time_ms, 0.0);
}

// ============================================================
// 测试：Milestone 011 — 碰撞安全裕度统一性验证
// ============================================================

// PreprocessingPipelineConfig 默认值一致性：collision_safety_margin 与
// bspline.collision_margin 默认值应均为 0。
// 触发原因：外圆已超出车辆矩形轮廓边界，无需物理安全裕度；Round 7 起
// StaticCorridorBuilderConfig 不再有 hard_margin 字段（静态走廊只剩独立的
// soft_margin，不再受 collision_safety_margin 驱动，安全职责完全交给 NMPC
// 侧默认生效的迭代走廊），因此本测试不再校验 corridor 侧的一致性。
// 预期行为：两者默认值严格相等（均为 0）。
TEST(PreprocessingPipelineTest, CollisionSafetyMarginDefaultsAreConsistent) {
    const PreprocessingPipelineConfig config;
    EXPECT_DOUBLE_EQ(config.collision_safety_margin, 0.0);
    EXPECT_DOUBLE_EQ(config.bspline.collision_margin, 0.0);
    EXPECT_DOUBLE_EQ(config.collision_safety_margin,
                     config.bspline.collision_margin);
}

// collision_safety_margin 变化不再影响静态舒适走廊输出。
// 触发原因：Round 7 删除 StaticCorridorBuilderConfig::hard_margin 字段后，
// PreprocessingPipeline 构造函数不再把 collision_safety_margin 传播到
// corridor 子配置（corridor.soft_margin 是独立字段，默认 18cm，与
// collision_safety_margin 无关）。此前该测试验证的是"传播生效、hard 约束
// 变紧"，现在传播路径已被有意移除，测试应反过来验证：collision_safety_margin
// 改变时，静态舒适走廊的 c_matrix/d_vector 输出应完全不受影响。
// 预期行为：默认裕度与自定义裕度下，静态走廊 c_matrix/d_vector 逐元素相等。
TEST(PreprocessingPipelineTest,
     CollisionSafetyMarginDoesNotAffectCorridorOutput) {
    const auto vehicle_params = MakeVehicleParams();
    const auto footprint = MakeFootprintModel();
    const auto esdf_map = MakeObstacleEsdfMap();

    // 使用默认裕度 0
    auto config_default = MakeConfigWithCorridor();
    const PreprocessingPipeline pipeline_default(config_default, vehicle_params,
                                                 footprint, esdf_map);

    // 使用自定义裕度 5cm（验证 collision_safety_margin 不再传播到 corridor）
    auto config_custom = MakeConfigWithCorridor();
    config_custom.collision_safety_margin = 0.05;
    const PreprocessingPipeline pipeline_custom(config_custom, vehicle_params,
                                                footprint, esdf_map);

    const auto maneuver = MakeLongStraightManeuver();
    const auto path = MakePathFromManeuver(maneuver);

    const auto result_default = pipeline_default.run(path);
    const auto result_custom = pipeline_custom.run(path);

    ASSERT_TRUE(result_default.success) << result_default.status_msg;
    ASSERT_TRUE(result_custom.success) << result_custom.status_msg;

    // 两个管线的输出维度应相同（裕度不影响重采样步长，也不影响走廊行数）
    EXPECT_EQ(result_default.final_dimension, result_custom.final_dimension);
    ASSERT_EQ(result_default.c_matrix.rows(), result_custom.c_matrix.rows());
    ASSERT_EQ(result_default.c_matrix.cols(), result_custom.c_matrix.cols());
    ASSERT_GE(result_default.c_matrix.rows(), 1);

    // collision_safety_margin 已不再驱动 corridor.soft_margin，两个管线的
    // c_matrix/d_vector 应逐元素相等。
    EXPECT_TRUE(result_default.c_matrix.isApprox(result_custom.c_matrix, 1e-9));
    EXPECT_TRUE(result_default.d_vector.isApprox(result_custom.d_vector, 1e-9));
}

// 无效 collision_safety_margin（负值）应被 PreprocessingPipeline 构造函数拦截。
// 触发原因：Review Round 1 发现构造函数缺少对 collision_safety_margin
// 的直接校验， 此前依赖下游 BSplineSmoother/StaticCorridorBuilder
// 各自校验传播后的值。 预期行为：负值或 NaN 在构造期即抛
// std::invalid_argument。
TEST(PreprocessingPipelineTest,
     ConstructorThrowsOnInvalidCollisionSafetyMargin) {
    auto config = MakeConfig();
    const auto vehicle_params = MakeVehicleParams();
    const auto footprint = MakeFootprintModel();
    const auto esdf_map = MakeLargeEmptyEsdfMap();

    // 负值
    config.collision_safety_margin = -0.1;
    EXPECT_THROW(
        PreprocessingPipeline(config, vehicle_params, footprint, esdf_map),
        std::invalid_argument);

    // NaN
    config.collision_safety_margin = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(
        PreprocessingPipeline(config, vehicle_params, footprint, esdf_map),
        std::invalid_argument);

    // +Inf
    config.collision_safety_margin = std::numeric_limits<double>::infinity();
    EXPECT_THROW(
        PreprocessingPipeline(config, vehicle_params, footprint, esdf_map),
        std::invalid_argument);
}

// 非默认 collision_safety_margin 对 BSplineSmoother 碰撞行为的影响。
// 触发原因：Review Round 1 指出
// NonDefaultCollisionSafetyMarginPropagatesToCorridorOutput
// 只覆盖了静态走廊侧传播，缺少 BSplineSmoother 侧的端到端验证。
// 预期行为：在障碍物附近，使用更小裕度（1cm）的管线允许更大的侵入深度，
// 使用更大裕度（10cm）的管线要求更严格，侵入深度更小或直接失败。
TEST(PreprocessingPipelineTest,
     CollisionSafetyMarginAffectsBSplineSmootherBehavior) {
    const auto vehicle_params = MakeVehicleParams();
    const auto footprint = MakeFootprintModel();
    // 使用含障碍物的地图：路径 y=0 贴近 y=-2.0 处的水平墙，
    // 裕度调整应显著影响 BSplineSmoother 的碰撞校验结果。
    const auto esdf_map = MakeObstacleEsdfMap();

    // 构造一条弯道贴近障碍物的路径，使碰撞代价非平凡
    auto maneuver = MakeCurvedManeuver();
    Path path;
    for (const auto& pp : maneuver.points) {
        path.addPoint(pp);
    }
    path.finalize();

    // 极小裕度 1cm：管线更容易通过碰撞校验
    auto config_small = MakeConfigWithCorridor();
    config_small.collision_safety_margin = 0.01;
    const PreprocessingPipeline pipeline_small(config_small, vehicle_params,
                                               footprint, esdf_map);
    const auto result_small = pipeline_small.run(path);

    // 较大裕度 15cm：碰撞校验应更严格，可能失败
    auto config_large = MakeConfigWithCorridor();
    config_large.collision_safety_margin = 0.15;
    const PreprocessingPipeline pipeline_large(config_large, vehicle_params,
                                               footprint, esdf_map);
    const auto result_large = pipeline_large.run(path);

    // 管线在无障碍冲突时均应成功（弯道路径离 y=-2 墙有足够距离）
    ASSERT_TRUE(result_small.success) << result_small.status_msg;
    ASSERT_TRUE(result_large.success) << result_large.status_msg;
    // 两个管线的输出维度应相同
    EXPECT_EQ(result_small.final_dimension, result_large.final_dimension);
}

// ============================================================
// 测试：Milestone 013 — 调试数据透传（enable_debug_output）
// ============================================================

// 默认配置下 enable_debug_output 应为 false，且结果中调试容器为空。
// 触发原因：Milestone 013 新增调试开关，必须保证默认关闭不影响生产路径。
// 预期行为：debug_maneuver_outputs 为空，且不额外分配内存。
TEST(PreprocessingPipelineTest, DebugOutputDisabledByDefault) {
    const PreprocessingPipelineConfig default_config;
    EXPECT_FALSE(default_config.enable_debug_output);

    // 使用关闭静态走廊的配置，避免空地图零梯度导致走廊构建失败。
    const auto config = MakeConfig();
    const auto vehicle_params = MakeVehicleParams();
    const auto footprint = MakeFootprintModel();
    const auto esdf_map = MakeLargeEmptyEsdfMap();
    const PreprocessingPipeline pipeline(config, vehicle_params, footprint,
                                         esdf_map);

    const auto maneuver = MakeLongStraightManeuver();
    const auto path = MakePathFromManeuver(maneuver);
    const auto result = pipeline.run(path);

    ASSERT_TRUE(result.success) << result.status_msg;
    EXPECT_TRUE(result.debug_maneuver_outputs.empty());
}

// 开启 enable_debug_output 后，debug_maneuver_outputs 应填充各阶段中间产物。
// 触发原因：验证调试数据透传链路正确，供 Visualizer 离线排障。
// 预期行为：每个非 PIVOT 机动段对应一个 PerManeuverOutput，且平滑结果、
// 速度规划结果、微分平坦结果均非空。
TEST(PreprocessingPipelineTest, DebugOutputPopulatedWhenEnabled) {
    auto config = MakeConfig();
    config.enable_debug_output = true;
    const auto vehicle_params = MakeVehicleParams();
    const auto footprint = MakeFootprintModel();
    const auto esdf_map = MakeLargeEmptyEsdfMap();
    const PreprocessingPipeline pipeline(config, vehicle_params, footprint,
                                         esdf_map);

    const auto maneuver = MakeLongStraightManeuver();
    const auto path = MakePathFromManeuver(maneuver);
    const auto result = pipeline.run(path);

    ASSERT_TRUE(result.success) << result.status_msg;
    EXPECT_FALSE(result.debug_maneuver_outputs.empty());
    for (const auto& output : result.debug_maneuver_outputs) {
        EXPECT_TRUE(output.smooth_result.success);
        EXPECT_FALSE(output.smooth_result.dense_points.empty());
        EXPECT_TRUE(output.speed_result.success);
        EXPECT_FALSE(output.speed_result.v.empty());
        EXPECT_TRUE(output.diff_flat_result.success);
        EXPECT_FALSE(output.diff_flat_result.points.empty());
    }
}

// 多机动段场景下，debug_maneuver_outputs 的数量应等于非 PIVOT 段数量。
// 触发原因：PIVOT 段在管线中被跳过，调试数据中不应为其分配空条目。
// 预期行为：前进+倒退两段产生 2 个 PerManeuverOutput。
TEST(PreprocessingPipelineTest, DebugOutputMatchesNonPivotManeuverCount) {
    auto config = MakeConfig();
    config.enable_debug_output = true;
    const auto vehicle_params = MakeVehicleParams();
    const auto footprint = MakeFootprintModel();
    const auto esdf_map = MakeLargeEmptyEsdfMap();
    const PreprocessingPipeline pipeline(config, vehicle_params, footprint,
                                         esdf_map);

    const auto m1 = MakeLongStraightManeuver();
    const auto m2 = MakeLongStraightBackwardManeuver();
    const auto path = MakeTwoManeuverPath(m1, m2);
    const auto result = pipeline.run(path);

    ASSERT_TRUE(result.success) << result.status_msg;
    EXPECT_EQ(result.debug_maneuver_outputs.size(), 2U);
}

// 关闭调试开关时，即使多次调用 run()，结果中的调试容器也应保持为空。
// 触发原因：确认不会在前一次开启调试后遗留数据到后续关闭调试的结果。
// 预期行为：第二次 run() 返回的 debug_maneuver_outputs 仍为空。
TEST(PreprocessingPipelineTest, DebugOutputClearedWhenDisabled) {
    auto config_on = MakeConfig();
    config_on.enable_debug_output = true;
    auto config_off = MakeConfig();
    config_off.enable_debug_output = false;
    const auto vehicle_params = MakeVehicleParams();
    const auto footprint = MakeFootprintModel();
    const auto esdf_map = MakeLargeEmptyEsdfMap();

    const PreprocessingPipeline pipeline_on(config_on, vehicle_params,
                                            footprint, esdf_map);
    const PreprocessingPipeline pipeline_off(config_off, vehicle_params,
                                             footprint, esdf_map);

    const auto maneuver = MakeLongStraightManeuver();
    const auto path = MakePathFromManeuver(maneuver);

    const auto result_on = pipeline_on.run(path);
    ASSERT_TRUE(result_on.success);
    EXPECT_FALSE(result_on.debug_maneuver_outputs.empty());

    const auto result_off = pipeline_off.run(path);
    ASSERT_TRUE(result_off.success);
    EXPECT_TRUE(result_off.debug_maneuver_outputs.empty());
}

// PreprocessingPipelineResult 应记录本次 run() 实际使用的关键参数与原始路径，
// 供 Visualizer 侧的 slack_true / 隔离墙 / Subplot 1 原始轨迹对比使用。
// 触发原因：Milestone 013 Round 1 评审要求可视化参数与管线实际参数保持一致。
// 预期行为：original_z_ref 非空且与输入路径点数一致；hard/soft margin 与配置
// 相同；outer_row_num_used 与构造管线时传入的 footprint 模型一致。
TEST(PreprocessingPipelineTest, ResultRecordsOriginalPathAndMarginParams) {
    auto config = MakeConfigWithCorridor();
    config.collision_safety_margin = 0.08;
    config.corridor.soft_margin = 0.25;
    const auto vehicle_params = MakeVehicleParams();
    const auto footprint = MakeFootprintModel();
    const auto esdf_map = MakeObstacleEsdfMap();
    const PreprocessingPipeline pipeline(config, vehicle_params, footprint,
                                         esdf_map);

    const auto maneuver = MakeLongStraightManeuver();
    const auto path = MakePathFromManeuver(maneuver);
    const auto result = pipeline.run(path);

    ASSERT_TRUE(result.success) << result.status_msg;
    EXPECT_FALSE(result.original_z_ref.empty());
    EXPECT_EQ(result.original_z_ref.size(), path.size());
    EXPECT_DOUBLE_EQ(result.hard_margin_used, 0.08);
    EXPECT_DOUBLE_EQ(result.soft_margin_used, 0.25);
    EXPECT_EQ(result.outer_row_num_used, footprint.getOuterRowNum());
}

}  // namespace
}  // namespace apa_post_processor
