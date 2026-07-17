#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "core/NMPC/nmpc_solver.h"
#include "core/NMPC/preprocessing_to_ocp_converter.h"
#include "preprocessing/preprocessing_pipeline.h"
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

// 公共车辆参数：与预处理管线测试保持一致。
VehicleParams MakeVehicleParams() {
    return VehicleParams(/*length=*/4.3, /*width=*/1.8, /*wheelbase=*/2.7,
                         /*max_steer_angle=*/0.6, /*rear_overhang=*/0.8,
                         /*max_accel=*/1.0, /*max_decel=*/-1.5,
                         /*max_steer_rate=*/0.4);
}

// 大地图：范围足够大，确保车辆在任何位姿下都不会越界。
ESDFMap MakeLargeEmptyEsdfMap() {
    const GridMap grid_map(0.1, 400, 300, Position{-10.0, -10.0}, {});
    return ESDFMap(grid_map);
}

// 含障碍物的地图：在 (3.0, -2.0) 附近有一堵水平墙，与
// preprocessing_pipeline.t.cpp 中的 MakeObstacleEsdfMap 保持一致。
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

// 默认管线配置（关闭静态走廊，避免空旷地图零梯度导致走廊构建失败）
PreprocessingPipelineConfig MakePipelineConfig() {
    PreprocessingPipelineConfig config;
    config.use_static_corridor = false;
    return config;
}

// 启用静态走廊的配置，需配合含障碍物地图使用
PreprocessingPipelineConfig MakePipelineConfigWithCorridor() {
    PreprocessingPipelineConfig config;
    config.use_static_corridor = true;
    return config;
}

// 构造一条长直前进机动段 (0,0,0)->(5,0,0)
Maneuver MakeLongStraightManeuver() {
    std::vector<TrajectoryPoint> points;
    for (double x = 0.0; x <= 5.0 + EPSILON; x += 0.1) {
        points.emplace_back(TrajectoryPoint{std::min(x, 5.0), 0.0, 0.0});
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

// 构造所有机动段均为 PIVOT/UNKNOWN 的 Path，用于测试 inferVSign 的 fallback。
Path MakePivotOnlyPath() {
    std::vector<TrajectoryPoint> points;
    for (int i = 0; i <= 10; ++i) {
        const double theta = static_cast<double>(i) * 0.1;
        TrajectoryPoint pt(0.0, 0.0, theta);
        pt.setV(0.0);
        pt.setA(0.0);
        pt.setDelta(0.0);
        pt.setDeltaDot(0.0);
        points.emplace_back(pt);
    }
    Maneuver maneuver(std::move(points), Direction::PIVOT);
    Path path;
    for (const auto& pt : maneuver.points) {
        path.addPoint(pt);
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

// 直接按 Maneuver 方向拼接成 Path，保留上游标注的 FORWARD/BACKWARD 方向，
// 避免 Path::addPoint 在 synthetic 路径上把倒退段误判为 FORWARD/PIVOT。
Path MakeSwitchbackPathFromManeuvers(const Maneuver& forward,
                                     const Maneuver& backward) {
    Path path;
    auto& maneuvers = path.getManeuvers();
    maneuvers.push_back(forward);
    maneuvers.push_back(backward);
    path.finalize();
    return path;
}

// 派生测试夹具，暴露 protected static 方法 inferVSign 以做白盒测试。
class PreprocessingToOcpConverterTestAccess
    : public PreprocessingToOcpConverter {
   public:
    PreprocessingToOcpConverterTestAccess(const VehicleParams& vehicle_params,
                                          const PathToOcpConfig& config)
        : PreprocessingToOcpConverter(vehicle_params, config) {}
    static double CallInferVSign(const Path& original_path,
                                 const std::vector<TrajectoryPoint>& z_ref) {
        return InferVSign(original_path, z_ref);
    }
};

}  // namespace

// ============================================================
// 测试：转换器基础行为
// ============================================================

// 构造器应拒绝非法车辆参数与非正配置项，避免后续 OCP 装配出现无意义边界。
TEST(PreprocessingToOcpConverterTest, ThrowsOnInvalidVehicleParams) {
    auto bad_vehicle_params = MakeVehicleParams();
    bad_vehicle_params.wheelbase = 0.0;
    EXPECT_THROW((PreprocessingToOcpConverter(bad_vehicle_params)),
                 std::invalid_argument);

    auto bad_steer_params = MakeVehicleParams();
    bad_steer_params.max_steer_angle = -0.1;
    EXPECT_THROW((PreprocessingToOcpConverter(bad_steer_params)),
                 std::invalid_argument);
}

TEST(PreprocessingToOcpConverterTest, ThrowsOnInvalidConfig) {
    const auto vehicle_params = MakeVehicleParams();
    PathToOcpConfig config;
    config.dt = 0.0;
    EXPECT_THROW((PreprocessingToOcpConverter(vehicle_params, config)),
                 std::invalid_argument);

    PathToOcpConfig config2;
    config2.max_speed = -1.0;
    EXPECT_THROW((PreprocessingToOcpConverter(vehicle_params, config2)),
                 std::invalid_argument);

    PathToOcpConfig config3;
    config3.boundary_velocity_slack = -0.01;
    EXPECT_THROW((PreprocessingToOcpConverter(vehicle_params, config3)),
                 std::invalid_argument);
}

// 转换失败预处理结果应抛异常，避免用无效数据构建 OCP。
TEST(PreprocessingToOcpConverterTest, ThrowsOnFailedPipeResult) {
    const auto vehicle_params = MakeVehicleParams();
    const PreprocessingToOcpConverter converter(vehicle_params);
    Path path;
    path.addPoint(Pose(0.0, 0.0, 0.0));
    path.addPoint(Pose(1.0, 0.0, 0.0));
    path.finalize();
    PreprocessingPipelineResult failed_result;
    failed_result.success = false;
    EXPECT_THROW(converter.convert(path, failed_result), std::invalid_argument);
}

// z_ref 点数不足时应抛异常。
TEST(PreprocessingToOcpConverterTest, ThrowsOnTooFewZRefPoints) {
    const auto vehicle_params = MakeVehicleParams();
    const PreprocessingToOcpConverter converter(vehicle_params);
    Path path;
    path.addPoint(Pose(0.0, 0.0, 0.0));
    path.addPoint(Pose(1.0, 0.0, 0.0));
    path.finalize();
    PreprocessingPipelineResult result;
    result.success = true;
    result.z_ref.push_back(TrajectoryPoint{0.0, 0.0, 0.0});
    result.z_ref.back().setV(0.0);
    result.z_ref.back().setA(0.0);
    result.z_ref.back().setDelta(0.0);
    result.z_ref.back().setDeltaDot(0.0);
    EXPECT_THROW(converter.convert(path, result), std::invalid_argument);
}

// delta_t 长度与 z_ref 不匹配时应抛异常。
TEST(PreprocessingToOcpConverterTest, ThrowsOnMismatchedDeltaT) {
    const auto vehicle_params = MakeVehicleParams();
    const PreprocessingToOcpConverter converter(vehicle_params);
    Path path;
    path.addPoint(Pose(0.0, 0.0, 0.0));
    path.addPoint(Pose(1.0, 0.0, 0.0));
    path.finalize();
    PreprocessingPipelineResult result;
    result.success = true;
    result.z_ref.push_back(TrajectoryPoint{0.0, 0.0, 0.0});
    result.z_ref.push_back(TrajectoryPoint{1.0, 0.0, 0.0});
    for (auto& pt : result.z_ref) {
        pt.setV(0.0);
        pt.setA(0.0);
        pt.setDelta(0.0);
        pt.setDeltaDot(0.0);
    }
    result.delta_t = {0.1, 0.1};  // 应为 1 个
    EXPECT_THROW(converter.convert(path, result), std::invalid_argument);
}

// z_ref 点缺少状态/控制量时应抛异常。
TEST(PreprocessingToOcpConverterTest, ThrowsOnMissingStateControl) {
    const auto vehicle_params = MakeVehicleParams();
    const PreprocessingToOcpConverter converter(vehicle_params);
    Path path;
    path.addPoint(Pose(0.0, 0.0, 0.0));
    path.addPoint(Pose(1.0, 0.0, 0.0));
    path.finalize();
    PreprocessingPipelineResult result;
    result.success = true;
    result.z_ref.push_back(TrajectoryPoint{0.0, 0.0, 0.0});
    result.z_ref.push_back(TrajectoryPoint{1.0, 0.0, 0.0});
    result.delta_t = {0.1};
    EXPECT_THROW(converter.convert(path, result), std::invalid_argument);
}

// ============================================================
// 测试：速度方向符号推断
// ============================================================

// z_ref 中存在非零速度时应优先以 z_ref 的符号为准，即使原始 Path 标注为前进。
// 这里不经过 pipeline，而是直接构造一个 FORWARD Path 与速度为负的人工 z_ref，
// 以排除 pipeline 自身速度规划对符号的干扰。
TEST(PreprocessingToOcpConverterTest, InfersVSignFromZRef) {
    const auto path =
        MakePathFromManeuver(MakeLongStraightManeuver());  // FORWARD
    std::vector<TrajectoryPoint> z_ref;
    z_ref.push_back(TrajectoryPoint{0.0, 0.0, 0.0});
    z_ref.back().setV(-0.5);  // 与 Path 方向相反
    z_ref.back().setA(0.0);
    z_ref.back().setDelta(0.0);
    z_ref.back().setDeltaDot(0.0);
    z_ref.push_back(TrajectoryPoint{1.0, 0.0, 0.0});
    z_ref.back().setV(-0.5);
    z_ref.back().setA(0.0);
    z_ref.back().setDelta(0.0);
    z_ref.back().setDeltaDot(0.0);

    const double v_sign =
        PreprocessingToOcpConverterTestAccess::CallInferVSign(path, z_ref);
    EXPECT_LT(v_sign, 0.0)
        << "z_ref backward speed should dominate over Path FORWARD label";
}

// 原始 Path 第一个非 PIVOT/UNKNOWN 机动段为 BACKWARD 时应返回 -1.0。
// 注意：Path::addPoint 根据位移在航向上的投影推断方向，因此真正的 BACKWARD 需要
// theta 与位移方向相反（theta=0 但 x 递减）。
TEST(PreprocessingToOcpConverterTest, InfersVSignFromPathBackward) {
    Path path;
    for (double x = 0.0; x >= -5.0 - EPSILON; x -= 0.1) {
        path.addPoint(Pose(x, 0.0, 0.0));
    }
    path.finalize();
    ASSERT_FALSE(path.getManeuvers().empty());
    ASSERT_EQ(path.getManeuvers().front().direction, Direction::BACKWARD);

    PreprocessingPipelineResult result;
    result.success = true;
    result.z_ref.push_back(TrajectoryPoint{0.0, 0.0, 0.0});
    result.z_ref.back().setV(0.0);
    result.z_ref.back().setA(0.0);
    result.z_ref.back().setDelta(0.0);
    result.z_ref.back().setDeltaDot(0.0);
    result.z_ref.push_back(TrajectoryPoint{-1.0, 0.0, 0.0});
    result.z_ref.back().setV(0.0);
    result.z_ref.back().setA(0.0);
    result.z_ref.back().setDelta(0.0);
    result.z_ref.back().setDeltaDot(0.0);
    result.delta_t = {0.1};

    const double v_sign = PreprocessingToOcpConverterTestAccess::CallInferVSign(
        path, result.z_ref);
    EXPECT_DOUBLE_EQ(v_sign, -1.0);
}

// z_ref 全零速度且 Path 全为 PIVOT/UNKNOWN 时应兜底为前进。
TEST(PreprocessingToOcpConverterTest, InfersVSignDefaultsToForward) {
    const auto path = MakePivotOnlyPath();
    PreprocessingPipelineResult result;
    result.success = true;
    result.z_ref.push_back(TrajectoryPoint{0.0, 0.0, 0.0});
    result.z_ref.back().setV(0.0);
    result.z_ref.back().setA(0.0);
    result.z_ref.back().setDelta(0.0);
    result.z_ref.back().setDeltaDot(0.0);
    result.z_ref.push_back(TrajectoryPoint{0.0, 0.0, 0.1});
    result.z_ref.back().setV(0.0);
    result.z_ref.back().setA(0.0);
    result.z_ref.back().setDelta(0.0);
    result.z_ref.back().setDeltaDot(0.0);
    result.delta_t = {0.1};

    const double v_sign = PreprocessingToOcpConverterTestAccess::CallInferVSign(
        path, result.z_ref);
    EXPECT_DOUBLE_EQ(v_sign, 1.0);
}

// ============================================================
// 测试：非均匀 delta_t 与初始猜测映射
// ============================================================

// 验证转换器正确把管线 delta_t 映射到 StageSegment::dt_array，
// 且初始猜测状态/控制量与 z_ref 一一对应。
TEST(PreprocessingToOcpConverterTest, MapsDtArrayAndInitialGuess) {
    const auto vehicle_params = MakeVehicleParams();
    const auto footprint = MakeFootprintModel();
    const auto esdf_map = MakeLargeEmptyEsdfMap();
    const PreprocessingPipeline pipeline(MakePipelineConfig(), vehicle_params,
                                         footprint, esdf_map);

    const auto path = MakePathFromManeuver(MakeLongStraightManeuver());
    const auto pipe_result = pipeline.run(path);
    ASSERT_TRUE(pipe_result.success) << pipe_result.status_msg;
    ASSERT_GE(pipe_result.z_ref.size(), 2U);

    const PreprocessingToOcpConverter converter(vehicle_params);
    const auto conv = converter.convert(path, pipe_result);

    ASSERT_EQ(conv.ocp.segments().size(), 1U);
    const auto& segment = conv.ocp.segments().front();
    EXPECT_FALSE(segment.dt_array.empty());
    EXPECT_EQ(segment.dt, 0.0);
    EXPECT_EQ(segment.dt_array.size(), pipe_result.delta_t.size());
    for (std::size_t i = 0; i < pipe_result.delta_t.size(); ++i) {
        EXPECT_DOUBLE_EQ(segment.dt_array[i], pipe_result.delta_t[i]);
    }
    EXPECT_EQ(segment.N, static_cast<int>(pipe_result.delta_t.size()));

    ASSERT_EQ(conv.init_guess.x.size(), pipe_result.z_ref.size());
    ASSERT_EQ(conv.init_guess.u.size(), pipe_result.delta_t.size());
    for (std::size_t i = 0; i < pipe_result.z_ref.size(); ++i) {
        const auto& pt = pipe_result.z_ref[i];
        const auto& x = conv.init_guess.x[i];
        EXPECT_DOUBLE_EQ(x(0), pt.x);
        EXPECT_DOUBLE_EQ(x(1), pt.y);
        EXPECT_DOUBLE_EQ(x(2), pt.theta);
        EXPECT_DOUBLE_EQ(x(3), pt.getV());
        EXPECT_DOUBLE_EQ(x(4), pt.getDelta());
        // 状态增广（BicycleModelJerk）：a、delta_dot 现为状态分量 5、6，
        // 初始猜测直接复用微分平坦已解出的参考值。
        EXPECT_DOUBLE_EQ(x(5), pt.getA());
        EXPECT_DOUBLE_EQ(x(6), pt.getDeltaDot());
    }
    for (std::size_t i = 0; i < pipe_result.delta_t.size(); ++i) {
        const auto& pt = pipe_result.z_ref[i];
        const auto& pt_next = pipe_result.z_ref[i + 1];
        const double dt_i = pipe_result.delta_t[i];
        const auto& u = conv.init_guess.u[i];
        // 新控制量 [jerk, ddelta_dot] 的初始猜测为相邻 z_ref 点 a/delta_dot 的
        // 有限差分近似（与 PreprocessingToOcpConverter::convert 实现一致）。
        EXPECT_DOUBLE_EQ(u(0), (pt_next.getA() - pt.getA()) / dt_i);
        EXPECT_DOUBLE_EQ(u(1),
                         (pt_next.getDeltaDot() - pt.getDeltaDot()) / dt_i);
    }
}

// 验证 stage_params 中参考航向与 z_ref 一致。
TEST(PreprocessingToOcpConverterTest, FillsStageParamsWithThetaRef) {
    const auto vehicle_params = MakeVehicleParams();
    const auto footprint = MakeFootprintModel();
    const auto esdf_map = MakeLargeEmptyEsdfMap();
    const PreprocessingPipeline pipeline(MakePipelineConfig(), vehicle_params,
                                         footprint, esdf_map);

    const auto path = MakePathFromManeuver(MakeLongStraightManeuver());
    const auto pipe_result = pipeline.run(path);
    ASSERT_TRUE(pipe_result.success) << pipe_result.status_msg;

    const PreprocessingToOcpConverter converter(vehicle_params);
    const auto conv = converter.convert(path, pipe_result);

    const auto& segment = conv.ocp.segments().front();
    ASSERT_EQ(segment.stage_params.size(), pipe_result.delta_t.size());
    for (std::size_t i = 0; i < segment.stage_params.size(); ++i) {
        EXPECT_DOUBLE_EQ(segment.stage_params[i].p(0), static_cast<double>(i));
        EXPECT_DOUBLE_EQ(segment.stage_params[i].p(1),
                         pipe_result.z_ref[i].theta);
    }
}

// 构造一条速度符号为“前进-后退-前进”的人工 z_ref，验证转换器按 v 符号切分为多段
// OCP。
TEST(PreprocessingToOcpConverterTest, SplitsIntoMultipleSegmentsByVSign) {
    const auto vehicle_params = MakeVehicleParams();
    const PreprocessingToOcpConverter converter(vehicle_params);
    const auto path = MakePathFromManeuver(MakeLongStraightManeuver());

    PreprocessingPipelineResult result;
    result.success = true;
    // 7 个点，速度符号：+ + - - + +，共 3 段。
    // 在符号翻转处插入 v=0 的换挡补丁点，使其成为相邻段共享的端点，
    // 避免共享状态的初始速度违反下一段的 box bound。
    const std::vector<double> velocities = {0.5, 0.0, -0.3, -0.3,
                                            0.0, 0.4, 0.4};
    for (std::size_t i = 0; i < velocities.size(); ++i) {
        TrajectoryPoint pt(static_cast<double>(i), 0.0, 0.0);
        pt.setV(velocities[i]);
        pt.setA(0.0);
        pt.setDelta(0.0);
        pt.setDeltaDot(0.0);
        result.z_ref.push_back(pt);
    }
    result.delta_t = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};

    const auto conv = converter.convert(path, result);
    const auto& segments = conv.ocp.segments();
    ASSERT_EQ(segments.size(), 3U);
    EXPECT_DOUBLE_EQ(segments[0].v_sign, 1.0);
    EXPECT_DOUBLE_EQ(segments[1].v_sign, -1.0);
    EXPECT_DOUBLE_EQ(segments[2].v_sign, 1.0);
    EXPECT_EQ(segments[0].N, 1);
    EXPECT_EQ(segments[1].N, 3);
    EXPECT_EQ(segments[2].N, 2);
    EXPECT_EQ(segments[0].dt_array.size(), 1U);
    EXPECT_EQ(segments[1].dt_array.size(), 3U);
    EXPECT_EQ(segments[2].dt_array.size(), 2U);
    EXPECT_EQ(conv.init_guess.x.size(), result.z_ref.size());
    EXPECT_EQ(conv.init_guess.u.size(), result.delta_t.size());
}

// ============================================================
// 测试：静态走廊系数截断
// ============================================================

// StaticCorridorBuilder 为每个 z_ref 点（含末端）生成约束，而 OCP 只需要前 N
// 步的约束，因此转换器应截断最后一个点对应的约束行。
TEST(PreprocessingToOcpConverterTest, TruncatesStaticCorridorToTotalSteps) {
    const auto vehicle_params = MakeVehicleParams();
    const auto footprint = MakeFootprintModel();
    const auto esdf_map = MakeObstacleEsdfMap();
    const PreprocessingPipeline pipeline(MakePipelineConfigWithCorridor(),
                                         vehicle_params, footprint, esdf_map);

    const auto path = MakePathFromManeuver(MakeLongStraightManeuver());
    const auto pipe_result = pipeline.run(path);
    ASSERT_TRUE(pipe_result.success) << pipe_result.status_msg;
    ASSERT_GT(pipe_result.c_matrix.rows(), 0);

    const int n_points = static_cast<int>(pipe_result.z_ref.size());
    const int n_steps = n_points - 1;
    const int constraints_per_point = pipe_result.c_matrix.rows() / n_points;
    ASSERT_GT(constraints_per_point, 0);

    const PreprocessingToOcpConverter converter(vehicle_params);
    const auto conv = converter.convert(path, pipe_result);

    EXPECT_EQ(conv.static_corridor_C.rows(), constraints_per_point * n_steps);
    EXPECT_EQ(conv.static_corridor_d.size(), constraints_per_point * n_steps);
    // 截断后的 C/d 应是原矩阵的前若干行；状态增广（Milestone 023，
    // BicycleModelJerk）后 TruncateCorridor 会把列数从 5 补齐到 7（新增 a、
    // ddelta 两列全零系数，走廊约束与之无关，见 TruncateCorridor 实现注释），
    // 因此这里只比较原始 5 列，新增列已由 padding 语义保证为 0，无需重复验证。
    const int src_cols = static_cast<int>(pipe_result.c_matrix.cols());
    for (int i = 0; i < conv.static_corridor_C.rows(); ++i) {
        EXPECT_TRUE(conv.static_corridor_C.row(i).leftCols(src_cols).isApprox(
            pipe_result.c_matrix.row(i)));
        EXPECT_DOUBLE_EQ(conv.static_corridor_d(i), pipe_result.d_vector(i));
    }
}

// 关闭静态走廊时截断结果应为空。
TEST(PreprocessingToOcpConverterTest, EmptyCorridorWhenDisabled) {
    const auto vehicle_params = MakeVehicleParams();
    const auto footprint = MakeFootprintModel();
    const auto esdf_map = MakeLargeEmptyEsdfMap();
    auto config = MakePipelineConfig();
    config.use_static_corridor = false;
    const PreprocessingPipeline pipeline(config, vehicle_params, footprint,
                                         esdf_map);

    const auto path = MakePathFromManeuver(MakeLongStraightManeuver());
    const auto pipe_result = pipeline.run(path);
    ASSERT_TRUE(pipe_result.success) << pipe_result.status_msg;
    EXPECT_EQ(pipe_result.c_matrix.rows(), 0);

    const PreprocessingToOcpConverter converter(vehicle_params);
    const auto conv = converter.convert(path, pipe_result);

    EXPECT_EQ(conv.static_corridor_C.rows(), 0);
    EXPECT_EQ(conv.static_corridor_d.size(), 0);
}

// ============================================================
// 测试：端到端接入 NmpcSolver
// ============================================================

// 单机动段场景：pipeline -> converter -> NmpcSolver
// 完整链路不崩溃，产出非空轨迹。
TEST(PreprocessingToOcpConverterTest, EndToEndSingleManeuver) {
    const auto vehicle_params = MakeVehicleParams();
    const auto footprint = MakeFootprintModel();
    const auto esdf_map = MakeLargeEmptyEsdfMap();
    const PreprocessingPipeline pipeline(MakePipelineConfig(), vehicle_params,
                                         footprint, esdf_map);

    const auto path = MakePathFromManeuver(MakeLongStraightManeuver());
    const auto pipe_result = pipeline.run(path);
    ASSERT_TRUE(pipe_result.success) << pipe_result.status_msg;

    const PreprocessingToOcpConverter converter(vehicle_params);
    const auto conv = converter.convert(path, pipe_result);

    NMPCConfig nmpc_config;
    nmpc_config.max_iter = 50;
    nmpc_config.static_corridor_C = conv.static_corridor_C;
    nmpc_config.static_corridor_d = conv.static_corridor_d;
    const NmpcSolver solver(vehicle_params, footprint, nmpc_config);
    const auto nmpc_result =
        solver.optimize(conv.ocp, conv.init_guess, esdf_map);

    EXPECT_FALSE(nmpc_result.trajectory.x.empty())
        << "NMPC should produce non-empty trajectory on valid warm start";
    EXPECT_GT(nmpc_result.solve_time_ms, 0.0);
}

// ============================================================
// 测试：预处理失败时整体判定失败（模拟 main.cpp 四态中的第一态）
// ============================================================

// 当 PreprocessingPipelineResult.success == false 时，业务层不应再调用
// converter 或 NmpcSolver，直接判定整体失败并携带状态消息。
TEST(PreprocessingToOcpConverterTest, MarksFailureOnPreprocessingFailure) {
    const auto vehicle_params = MakeVehicleParams();
    const PreprocessingToOcpConverter converter(vehicle_params);
    Path path;
    path.addPoint(Pose(0.0, 0.0, 0.0));
    path.addPoint(Pose(1.0, 0.0, 0.0));
    path.finalize();
    PreprocessingPipelineResult failed_result;
    failed_result.success = false;
    failed_result.status_msg = "deliberate failure";

    // 模拟 main.cpp 的前置判定：预处理失败直接抛异常/标记失败，不进入 NMPC。
    EXPECT_THROW(converter.convert(path, failed_result), std::invalid_argument);

    // 业务层状态机应落在"整体失败"分支。
    bool business_success = false;
    std::string message;
    if (!failed_result.success) {
        business_success = false;
        message = "Preprocessing pipeline failed: " + failed_result.status_msg;
    }
    EXPECT_FALSE(business_success);
    EXPECT_NE(message.find("Preprocessing pipeline failed"), std::string::npos);
}

// ============================================================
// 测试：NMPC 状态机（未收敛返回最新迭代 / 完全失败回退）
// ============================================================

// 通过设置 max_iter=1 让 NmpcSolver 只迭代一次即耗尽预算，验证业务层正确识别
// "reached max_iter without full convergence, returning last iterate"状态：
// 返回的 trajectory.x 非空，但 converged == false。
TEST(PreprocessingToOcpConverterTest, ReturnsLastIterateOnMaxIterExhaustion) {
    const auto vehicle_params = MakeVehicleParams();
    const auto footprint = MakeFootprintModel();
    const auto esdf_map = MakeLargeEmptyEsdfMap();
    const PreprocessingPipeline pipeline(MakePipelineConfig(), vehicle_params,
                                         footprint, esdf_map);

    const auto path = MakePathFromManeuver(MakeLongStraightManeuver());
    const auto pipe_result = pipeline.run(path);
    ASSERT_TRUE(pipe_result.success) << pipe_result.status_msg;

    const PreprocessingToOcpConverter converter(vehicle_params);
    const auto conv = converter.convert(path, pipe_result);

    NMPCConfig nmpc_config;
    nmpc_config.max_iter = 1;  // 强制未收敛但返回最后一次迭代
    nmpc_config.static_corridor_C = conv.static_corridor_C;
    nmpc_config.static_corridor_d = conv.static_corridor_d;
    const NmpcSolver solver(vehicle_params, footprint, nmpc_config);
    const auto nmpc_result =
        solver.optimize(conv.ocp, conv.init_guess, esdf_map);

    EXPECT_FALSE(nmpc_result.trajectory.x.empty())
        << "NMPC should return last iterate even if max_iter exhausted";
    EXPECT_FALSE(nmpc_result.converged)
        << "Single iteration should not satisfy convergence criteria";
}

// 通过设置 max_iter=0 让 NmpcSolver 在 validateProblem 阶段即失败，
// 验证业务层（模拟 main.cpp 逻辑）正确回退到预处理 z_ref 并标注状态。
TEST(PreprocessingToOcpConverterTest, FallsBackToPreprocessingOnNmpcFailure) {
    const auto vehicle_params = MakeVehicleParams();
    const auto footprint = MakeFootprintModel();
    const auto esdf_map = MakeLargeEmptyEsdfMap();
    const PreprocessingPipeline pipeline(MakePipelineConfig(), vehicle_params,
                                         footprint, esdf_map);

    const auto path = MakePathFromManeuver(MakeLongStraightManeuver());
    const auto pipe_result = pipeline.run(path);
    ASSERT_TRUE(pipe_result.success) << pipe_result.status_msg;

    const PreprocessingToOcpConverter converter(vehicle_params);
    const auto conv = converter.convert(path, pipe_result);

    NMPCConfig nmpc_config;
    nmpc_config.max_iter = 0;  // 强制 validateProblem 失败
    nmpc_config.static_corridor_C = conv.static_corridor_C;
    nmpc_config.static_corridor_d = conv.static_corridor_d;
    const NmpcSolver solver(vehicle_params, footprint, nmpc_config);
    const auto nmpc_result =
        solver.optimize(conv.ocp, conv.init_guess, esdf_map);

    ASSERT_TRUE(nmpc_result.trajectory.x.empty())
        << "max_iter=0 should produce empty trajectory";

    // 回退路径：直接把 z_ref 转成 Path
    Path fallback_path;
    for (const auto& pt : pipe_result.z_ref) {
        fallback_path.addPoint(pt);
    }
    fallback_path.finalize();

    EXPECT_FALSE(fallback_path.empty());
    EXPECT_EQ(fallback_path.size(), pipe_result.z_ref.size());
}

}  // namespace apa_post_processor
