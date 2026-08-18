#include <gtest/gtest.h>

#include <cmath>
#include <string>

#include "core/collision_check.h"
#include "core/post_processor.h"
#include "spatial/esdf_map.h"
#include "spatial/grid_map.h"
#include "util/constants.h"
#include "util/path.h"
#include "vehicle/vehicle_footprint_model.h"
#include "vehicle/vehicle_params.h"

namespace apa_post_processor {
namespace {

// 公共车辆参数（与 post_processor.t.cpp 的合成场景一致）
VehicleParams MakeVehicleParams() {
    return VehicleParams(/*length=*/4.3, /*width=*/1.8, /*wheelbase=*/2.7,
                         /*max_steer_angle=*/0.6, /*rear_overhang=*/0.8,
                         /*max_accel=*/1.0, /*max_decel=*/-1.5,
                         /*max_steer_rate=*/0.4);
}

// 无障碍大地图：确保车辆包络完整落在地图内
ESDFMap MakeLargeEmptyEsdfMap() {
    const GridMap grid_map(0.1, 400, 300, Position{-10.0, -10.0}, {});
    return ESDFMap(grid_map);
}

// Footprint 模型：single outer row 简化几何
VehicleFootprintModel MakeFootprintModel(const VehicleParams& params) {
    return VehicleFootprintModel(params,
                                 /*heading_sample_num=*/233,
                                 /*inner_row_num=*/2, /*outer_row_num=*/1);
}

// 白盒访问器：暴露 protected 静态方法
class PostProcessorMincoTestAccess : public PostProcessor {
   public:
    using PostProcessor::PostProcessor;
    static MincoConfig CallDeriveKinematicsConfig(
        const VehicleParams& vehicle_params, const MincoConfig& minco_config) {
        return DeriveKinematicsConfig(vehicle_params, minco_config);
    }
};

// 从当前路径末端沿 x 轴追加直线路径点（步长 0.05 m，与 A* 点距一致）
void AppendXLine(Path* path, double x_from, double x_to, double theta) {
    const int count =
        static_cast<int>(std::round(std::abs(x_to - x_from) / 0.05));
    for (int i = 1; i <= count; ++i) {
        const double x = x_from + (x_to - x_from) * i / count;
        path->addPoint({x, 0.0, theta});
    }
}

// 构造从原点出发沿 x 轴的直线前进路径
Path BuildStraightPath(double length) {
    Path path;
    path.addPoint({0.0, 0.0, 0.0});
    AppendXLine(&path, 0.0, length, 0.0);
    path.finalize();
    return path;
}

// 构造 前进1.0m → 后退0.5m 的单次换挡路径（2 个机动段）
Path BuildGearShiftPath() {
    Path path;
    path.addPoint({0.0, 0.0, 0.0});
    AppendXLine(&path, 0.0, 1.0, 0.0);
    AppendXLine(&path, 1.0, 0.5, 0.0);
    path.finalize();
    return path;
}

// 终点位置误差 (m)：优化路径末点与初始路径末点的欧氏距离
double TerminalPositionError(const Path& optimized, const Path& init) {
    return std::hypot(optimized.back().x - init.back().x,
                      optimized.back().y - init.back().y);
}

// 终点航向误差 (deg)：优化路径末点与初始路径末点的归一化角度差
double TerminalHeadingErrorDeg(const Path& optimized, const Path& init) {
    return std::abs(
               NormalizeAngle(optimized.back().theta - init.back().theta)) *
           180.0 / PI;
}

}  // namespace

// ============================================================
// 测试：运动学配置由车辆物理参数派生
// ============================================================

// 轴距/最大前轮转角/转角速度必须与 VehicleParams 同源；加速度上限取
// max_accel 与 |max_decel| 的较小值；速度上限取自 MINCO 配置。
TEST(PostProcessorMincoTest, DeriveKinematicsConfigMatchesVehicleParams) {
    const auto vehicle_params = MakeVehicleParams();
    MincoConfig minco_config;
    minco_config.max_velocity = 1.8;
    const auto kinematics_config =
        PostProcessorMincoTestAccess::CallDeriveKinematicsConfig(vehicle_params,
                                                                minco_config);
    EXPECT_DOUBLE_EQ(kinematics_config.wheelbase, 2.7);
    EXPECT_DOUBLE_EQ(kinematics_config.max_steer_angle, 0.6);
    EXPECT_DOUBLE_EQ(kinematics_config.max_steer_rate, 0.4);
    // min(max_accel=1.0, |max_decel|=1.5) = 1.0
    EXPECT_DOUBLE_EQ(kinematics_config.max_acceleration, 1.0);
    EXPECT_DOUBLE_EQ(kinematics_config.max_velocity, 1.8);
}

// ============================================================
// 测试：MINCO 路径不污染配置、不与 NMPC 路径互相干扰
// ============================================================

// 调用方传入的 MincoConfig 对象在 optimizeMinco 后必须保持原值。
TEST(PostProcessorMincoTest, DoesNotMutateMincoConfig) {
    const auto vehicle_params = MakeVehicleParams();
    const auto footprint = MakeFootprintModel(vehicle_params);
    const auto esdf_map = MakeLargeEmptyEsdfMap();
    const PostProcessor processor(vehicle_params, footprint, esdf_map);
    MincoConfig minco_config;
    minco_config.nominal_segment_length = 0.45;
    minco_config.max_outer_iterations = 3;
    minco_config.solver_weight_gear_cusp = 500.0;
    minco_config.margin_safe = 0.03;
    minco_config.melt_arc_threshold = 0.08;
    minco_config.max_velocity = 1.5;
    const auto path = BuildStraightPath(2.0);
    processor.optimizeMinco(path, minco_config);
    EXPECT_DOUBLE_EQ(minco_config.nominal_segment_length, 0.45);
    EXPECT_EQ(minco_config.max_outer_iterations, 3);
    EXPECT_DOUBLE_EQ(minco_config.solver_weight_gear_cusp, 500.0);
    EXPECT_DOUBLE_EQ(minco_config.margin_safe, 0.03);
    EXPECT_DOUBLE_EQ(minco_config.melt_arc_threshold, 0.08);
    EXPECT_DOUBLE_EQ(minco_config.max_velocity, 1.5);
}

// 先跑一次 NMPC 路径（max_iter=0 强制回退），再跑 MINCO 路径，最后再跑
// NMPC 路径：两次 NMPC 结果必须完全一致，且 NMPC 配置对象未被触碰。
TEST(PostProcessorMincoTest, MincoPathDoesNotInterfereWithNmpcPath) {
    const auto vehicle_params = MakeVehicleParams();
    const auto footprint = MakeFootprintModel(vehicle_params);
    const auto esdf_map = MakeLargeEmptyEsdfMap();
    const PostProcessor processor(vehicle_params, footprint, esdf_map);
    NMPCConfig nmpc_config;
    nmpc_config.max_iter = 0;  // 强制 NMPC 失败，走确定性的预处理回退分支
    nmpc_config.use_static_corridor = false;
    const auto path = BuildStraightPath(5.0);
    const auto before = processor.optimize(path, nmpc_config);
    ASSERT_TRUE(before.success);
    const auto minco_result = processor.optimizeMinco(path, MincoConfig{});
    ASSERT_TRUE(minco_result.success) << minco_result.message;
    const auto after = processor.optimize(path, nmpc_config);
    ASSERT_TRUE(after.success);
    EXPECT_EQ(before.message, after.message);
    EXPECT_EQ(before.final_maneuvers, after.final_maneuvers);
    EXPECT_DOUBLE_EQ(before.final_length, after.final_length);
    // NMPC 配置对象未被 MINCO 路径触碰
    EXPECT_EQ(nmpc_config.max_iter, 0);
    EXPECT_FALSE(nmpc_config.use_static_corridor);
}

// ============================================================
// 测试：minco_traj 结果轨迹填充
// ============================================================

// 成功路径必须同步填充 minco_traj（点数与输出 Path 一致，逐点携带状态/控制量）；
// 失败路径 minco_traj 必须为空。
TEST(PostProcessorMincoTest, MincoTrajFilledOnSuccessAndEmptyOnFailure) {
    const auto vehicle_params = MakeVehicleParams();
    const auto footprint = MakeFootprintModel(vehicle_params);
    const auto esdf_map = MakeLargeEmptyEsdfMap();
    const PostProcessor processor(vehicle_params, footprint, esdf_map);
    const auto path = BuildStraightPath(2.0);
    const auto result = processor.optimizeMinco(path, MincoConfig{});
    ASSERT_TRUE(result.success) << result.message;
    std::size_t path_points = 0;
    for (const auto& maneuver : result.optimized_path.getManeuvers()) {
        path_points += maneuver.points.size();
    }
    ASSERT_EQ(result.optimized_trajectory.size(), path_points);
    for (const auto& pt : result.optimized_trajectory) {
        EXPECT_TRUE(std::isfinite(pt.x));
        EXPECT_TRUE(std::isfinite(pt.y));
        EXPECT_TRUE(std::isfinite(pt.theta));
        EXPECT_TRUE(pt.hasV());
        EXPECT_TRUE(pt.hasDelta());
    }
    const Path empty_path;
    const auto failed = processor.optimizeMinco(empty_path, MincoConfig{});
    EXPECT_FALSE(failed.success);
    EXPECT_TRUE(failed.optimized_trajectory.empty());
}

// ============================================================
// 测试：minco_preprocessed_traj 预处理粗优化轨迹填充
// ============================================================

// 预处理成功后必须填充 intermediate_traces["minco_preprocessed"]（预处理
// 粗优化轨迹的离散化结果，作为"优化前"对比基线）：首点锚定初始路径首点，
// 末点在预处理收敛容差内贴近初始路径末点，逐点携带状态/控制量；预处理
// 失败时 intermediate_traces 中无此项。
TEST(PostProcessorMincoTest,
     MincoPreprocessedTrajFilledOnSuccessAndEmptyOnFailure) {
    const auto vehicle_params = MakeVehicleParams();
    const auto footprint = MakeFootprintModel(vehicle_params);
    const auto esdf_map = MakeLargeEmptyEsdfMap();
    const PostProcessor processor(vehicle_params, footprint, esdf_map);
    const auto path = BuildStraightPath(2.0);
    const auto result = processor.optimizeMinco(path, MincoConfig{});
    ASSERT_TRUE(result.success) << result.message;
    // 从 intermediate_traces 中查找预处理轨迹
    const Trajectory* preprocessed = nullptr;
    for (const auto& [name, traj] : result.intermediate_traces) {
        if (name == "minco_preprocessed") {
            preprocessed = &traj;
            break;
        }
    }
    ASSERT_NE(preprocessed, nullptr);
    ASSERT_FALSE(preprocessed->empty());
    // 离散化锚定初始路径首点（世界坐标还原积分的锚点）
    EXPECT_DOUBLE_EQ(preprocessed->front().x, path.front().x);
    EXPECT_DOUBLE_EQ(preprocessed->front().y, path.front().y);
    for (const auto& pt : *preprocessed) {
        EXPECT_TRUE(std::isfinite(pt.x));
        EXPECT_TRUE(std::isfinite(pt.y));
        EXPECT_TRUE(std::isfinite(pt.theta));
        EXPECT_TRUE(pt.hasV());
        EXPECT_TRUE(pt.hasDelta());
    }
    // 预处理收敛容差默认 0.1 m，叠加离散化积分还原余量后取 0.15 m 上限
    EXPECT_NEAR(preprocessed->back().x, path.back().x, 0.15);
    EXPECT_NEAR(preprocessed->back().y, path.back().y, 0.15);
    // 预处理失败：intermediate_traces 中无 minco_preprocessed 项
    MincoConfig fail_config;
    fail_config.convergence_position_tolerance = 1e-9;
    const auto failed = processor.optimizeMinco(path, fail_config);
    ASSERT_FALSE(failed.success);
    bool has_preprocessed = false;
    for (const auto& [name, traj] : failed.intermediate_traces) {
        if (name == "minco_preprocessed") {
            has_preprocessed = true;
            break;
        }
    }
    EXPECT_FALSE(has_preprocessed);
}

// ============================================================
// 测试：合成场景端到端
// ============================================================

// 空路径输入必须显式失败，不抛异常、不返回半成品。
TEST(PostProcessorMincoTest, EmptyPathReturnsFailure) {
    const auto vehicle_params = MakeVehicleParams();
    const auto footprint = MakeFootprintModel(vehicle_params);
    const auto esdf_map = MakeLargeEmptyEsdfMap();
    const PostProcessor processor(vehicle_params, footprint, esdf_map);
    const Path empty_path;
    const auto result = processor.optimizeMinco(empty_path, MincoConfig{});
    EXPECT_FALSE(result.success);
    EXPECT_TRUE(result.optimized_path.empty());
    EXPECT_FALSE(result.message.empty());
}

// 预处理失败必须显式走失败分支（第 2 道质量门），不抛异常、不返回半成品。
// 把预处理跟踪容差收紧到物理上不可达的量级（合成场景收敛后跟踪误差在
// 1e-3 m 量级，远大于 1e-9），确定性触发预处理失败。
TEST(PostProcessorMincoTest, PreprocessFailureReturnsFailure) {
    const auto vehicle_params = MakeVehicleParams();
    const auto footprint = MakeFootprintModel(vehicle_params);
    const auto esdf_map = MakeLargeEmptyEsdfMap();
    const PostProcessor processor(vehicle_params, footprint, esdf_map);
    MincoConfig minco_config;
    minco_config.convergence_position_tolerance = 1e-9;
    const auto path = BuildStraightPath(2.0);
    const auto result = processor.optimizeMinco(path, minco_config);
    EXPECT_FALSE(result.success);
    EXPECT_TRUE(result.optimized_path.empty());
    EXPECT_NE(result.message.find("preprocessing failed"), std::string::npos);
    EXPECT_TRUE(result.optimized_trajectory.empty());
}

// 空旷直线场景应收敛：终点双指标满足 MINCO 收敛判据，轨迹无碰撞、无奇异。
TEST(PostProcessorMincoTest, EndToEndStraightLineConverges) {
    const auto vehicle_params = MakeVehicleParams();
    const auto footprint = MakeFootprintModel(vehicle_params);
    const auto esdf_map = MakeLargeEmptyEsdfMap();
    const PostProcessor processor(vehicle_params, footprint, esdf_map);
    const auto path = BuildStraightPath(2.0);
    const auto result = processor.optimizeMinco(path, MincoConfig{});
    ASSERT_TRUE(result.success) << result.message;
    EXPECT_EQ(result.final_maneuvers, 1);
    EXPECT_GT(result.final_length, 0.0);
    EXPECT_GT(result.total_time_ms, 0.0);
    EXPECT_TRUE(IsPathFinite(result.optimized_path));
    EXPECT_LE(TerminalPositionError(result.optimized_path, path), 0.05);
    EXPECT_LE(TerminalHeadingErrorDeg(result.optimized_path, path), 1.5);
    EXPECT_LE(
        ComputeMaxCollisionDepth(result.optimized_path, esdf_map, footprint),
        0.02);
}

// 单次真实换挡场景应收敛且输出合法。物理方向段口径下，优化器在空旷场景
// 可合法消除冗余换挡（直接从起点爬向终点，长度 1.5m→~0.55m，终点误差
// 0.04 m 在 0.05 m 容差内），也可保留换挡——两种形态均为合法输出；真实
// 数据集的换挡由障碍物几何强制，不会出现这种退化
TEST(PostProcessorMincoTest, EndToEndGearShiftConverges) {
    const auto vehicle_params = MakeVehicleParams();
    const auto footprint = MakeFootprintModel(vehicle_params);
    const auto esdf_map = MakeLargeEmptyEsdfMap();
    const PostProcessor processor(vehicle_params, footprint, esdf_map);
    const auto path = BuildGearShiftPath();
    const auto result = processor.optimizeMinco(path, MincoConfig{});
    ASSERT_TRUE(result.success) << result.message;
    EXPECT_GE(result.final_maneuvers, 1);
    EXPECT_LE(result.final_maneuvers, 2);
    EXPECT_TRUE(IsPathFinite(result.optimized_path));
    EXPECT_LE(TerminalPositionError(result.optimized_path, path), 0.05);
    EXPECT_LE(TerminalHeadingErrorDeg(result.optimized_path, path), 1.5);
    EXPECT_LE(
        ComputeMaxCollisionDepth(result.optimized_path, esdf_map, footprint),
        0.02);
}

// ============================================================
// 测试：PostProcessorResult 重构新字段（Phase 1 双写验证）
// ============================================================

// 成功路径：optimized_trajectory 包含完整运动学量
TEST(PostProcessorMincoTest, OptimizedTrajectoryContainsFullKinematics) {
    const auto vehicle_params = MakeVehicleParams();
    const auto footprint = MakeFootprintModel(vehicle_params);
    const auto esdf_map = MakeLargeEmptyEsdfMap();
    const PostProcessor processor(vehicle_params, footprint, esdf_map);
    const auto path = BuildStraightPath(2.0);
    const auto result = processor.optimizeMinco(path, MincoConfig{});
    ASSERT_TRUE(result.success) << result.message;
    ASSERT_FALSE(result.optimized_trajectory.empty());
    for (const auto& pt : result.optimized_trajectory) {
        EXPECT_TRUE(std::isfinite(pt.x));
        EXPECT_TRUE(std::isfinite(pt.y));
        EXPECT_TRUE(std::isfinite(pt.theta));
        EXPECT_TRUE(pt.hasV());
        EXPECT_TRUE(pt.hasDelta());
    }
}

// algorithm 字段反映实际运行的求解器
TEST(PostProcessorMincoTest, AlgorithmFieldIsMinco) {
    const auto vehicle_params = MakeVehicleParams();
    const auto footprint = MakeFootprintModel(vehicle_params);
    const auto esdf_map = MakeLargeEmptyEsdfMap();
    const PostProcessor processor(vehicle_params, footprint, esdf_map);
    const auto path = BuildStraightPath(2.0);
    const auto result = processor.optimizeMinco(path, MincoConfig{});
    ASSERT_TRUE(result.success);
    EXPECT_EQ(result.algorithm, "minco");
}

// intermediate_traces 包含预处理轨迹（成功路径）
TEST(PostProcessorMincoTest, IntermediateTracesContainMincoPreprocessed) {
    const auto vehicle_params = MakeVehicleParams();
    const auto footprint = MakeFootprintModel(vehicle_params);
    const auto esdf_map = MakeLargeEmptyEsdfMap();
    const PostProcessor processor(vehicle_params, footprint, esdf_map);
    const auto path = BuildStraightPath(2.0);
    const auto result = processor.optimizeMinco(path, MincoConfig{});
    ASSERT_TRUE(result.success) << result.message;
    // intermediate_traces 应包含 "minco_preprocessed" 条目
    bool found = false;
    for (const auto& [name, traj] : result.intermediate_traces) {
        if (name == "minco_preprocessed") {
            found = true;
            EXPECT_FALSE(traj.empty());
        }
    }
    EXPECT_TRUE(found) << "intermediate_traces should contain minco_preprocessed";
}

// output_level 分级语义
TEST(PostProcessorMincoTest, OutputLevelReflectsResultQuality) {
    const auto vehicle_params = MakeVehicleParams();
    const auto footprint = MakeFootprintModel(vehicle_params);
    const auto esdf_map = MakeLargeEmptyEsdfMap();
    const PostProcessor processor(vehicle_params, footprint, esdf_map);
    // 直线场景：应收敛，输出级别为完全成功
    {
        const auto path = BuildStraightPath(2.0);
        const auto result = processor.optimizeMinco(path, MincoConfig{});
        ASSERT_TRUE(result.success);
        EXPECT_EQ(result.output_level, OutputLevel::kFullSuccess);
    }
    // 空路径：直接失败
    {
        const Path empty_path;
        const auto result = processor.optimizeMinco(empty_path, MincoConfig{});
        EXPECT_FALSE(result.success);
        EXPECT_EQ(result.output_level, OutputLevel::kFallback);
    }
}

}  // namespace apa_post_processor
