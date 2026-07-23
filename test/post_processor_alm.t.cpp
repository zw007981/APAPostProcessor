#include <gtest/gtest.h>

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "core/NMPC/vehicle_circle_geometry.h"
#include "core/collision_check.h"
#include "core/post_processor.h"
#include "spatial/esdf_map.h"
#include "spatial/grid_map.h"
#include "util/constants.h"
#include "util/data_loader.hpp"
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
class PostProcessorAlmTestAccess : public PostProcessor {
   public:
    using PostProcessor::PostProcessor;
    static BicycleKinematicsConfig CallDeriveKinematicsConfig(
        const VehicleParams& vehicle_params, double max_velocity) {
        return DeriveKinematicsConfig(vehicle_params, max_velocity);
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
// max_accel 与 |max_decel| 的较小值；速度上限取自 ALM 配置。
TEST(PostProcessorAlmTest, DeriveKinematicsConfigMatchesVehicleParams) {
    const auto vehicle_params = MakeVehicleParams();
    const auto kinematics_config =
        PostProcessorAlmTestAccess::CallDeriveKinematicsConfig(vehicle_params,
                                                               1.8);
    EXPECT_DOUBLE_EQ(kinematics_config.wheelbase, 2.7);
    EXPECT_DOUBLE_EQ(kinematics_config.max_steer_angle, 0.6);
    EXPECT_DOUBLE_EQ(kinematics_config.max_steer_rate, 0.4);
    // min(max_accel=1.0, |max_decel|=1.5) = 1.0
    EXPECT_DOUBLE_EQ(kinematics_config.max_acceleration, 1.0);
    EXPECT_DOUBLE_EQ(kinematics_config.max_velocity, 1.8);
}

// ============================================================
// 测试：ALM 路径不污染配置、不与 NMPC 路径互相干扰
// ============================================================

// 调用方传入的 AlmConfig 对象在 optimizeAlm 后必须保持原值。
TEST(PostProcessorAlmTest, DoesNotMutateAlmConfig) {
    const auto vehicle_params = MakeVehicleParams();
    const auto footprint = MakeFootprintModel(vehicle_params);
    const auto esdf_map = MakeLargeEmptyEsdfMap();
    const PostProcessor processor(vehicle_params, footprint, esdf_map);
    AlmConfig alm_config;
    alm_config.segmenter.nominal_segment_length = 0.45;
    alm_config.solver.max_outer_iterations = 3;
    alm_config.solver.weight_gear_cusp = 500.0;
    alm_config.esdf_penalty.margin_safe = 0.03;
    alm_config.melter.melt_arc_threshold = 0.08;
    alm_config.max_velocity = 1.5;
    const auto path = BuildStraightPath(2.0);
    processor.optimizeAlm(path, alm_config);
    EXPECT_DOUBLE_EQ(alm_config.segmenter.nominal_segment_length, 0.45);
    EXPECT_EQ(alm_config.solver.max_outer_iterations, 3);
    EXPECT_DOUBLE_EQ(alm_config.solver.weight_gear_cusp, 500.0);
    EXPECT_DOUBLE_EQ(alm_config.esdf_penalty.margin_safe, 0.03);
    EXPECT_DOUBLE_EQ(alm_config.melter.melt_arc_threshold, 0.08);
    EXPECT_DOUBLE_EQ(alm_config.max_velocity, 1.5);
}

// 先跑一次 NMPC 路径（max_iter=0 强制回退），再跑 ALM 路径，最后再跑
// NMPC 路径：两次 NMPC 结果必须完全一致，且 NMPC 配置对象未被触碰。
TEST(PostProcessorAlmTest, AlmPathDoesNotInterfereWithNmpcPath) {
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
    const auto alm_result = processor.optimizeAlm(path, AlmConfig{});
    ASSERT_TRUE(alm_result.success) << alm_result.message;
    const auto after = processor.optimize(path, nmpc_config);
    ASSERT_TRUE(after.success);
    EXPECT_EQ(before.message, after.message);
    EXPECT_EQ(before.final_maneuvers, after.final_maneuvers);
    EXPECT_DOUBLE_EQ(before.final_length, after.final_length);
    // NMPC 配置对象未被 ALM 路径触碰
    EXPECT_EQ(nmpc_config.max_iter, 0);
    EXPECT_FALSE(nmpc_config.use_static_corridor);
}

// ============================================================
// 测试：alm_traj 结果轨迹填充
// ============================================================

// 成功路径必须同步填充 alm_traj（点数与输出 Path 一致，逐点携带状态/控制量）；
// 失败路径 alm_traj 必须为空。
TEST(PostProcessorAlmTest, AlmTrajFilledOnSuccessAndEmptyOnFailure) {
    const auto vehicle_params = MakeVehicleParams();
    const auto footprint = MakeFootprintModel(vehicle_params);
    const auto esdf_map = MakeLargeEmptyEsdfMap();
    const PostProcessor processor(vehicle_params, footprint, esdf_map);
    const auto path = BuildStraightPath(2.0);
    const auto result = processor.optimizeAlm(path, AlmConfig{});
    ASSERT_TRUE(result.success) << result.message;
    std::size_t path_points = 0;
    for (const auto& maneuver : result.optimized_path.getManeuvers()) {
        path_points += maneuver.points.size();
    }
    ASSERT_EQ(result.alm_traj.size(), path_points);
    for (const auto& pt : result.alm_traj) {
        EXPECT_TRUE(std::isfinite(pt.x));
        EXPECT_TRUE(std::isfinite(pt.y));
        EXPECT_TRUE(std::isfinite(pt.theta));
        EXPECT_TRUE(pt.hasV());
        EXPECT_TRUE(pt.hasDelta());
    }
    const Path empty_path;
    const auto failed = processor.optimizeAlm(empty_path, AlmConfig{});
    EXPECT_FALSE(failed.success);
    EXPECT_TRUE(failed.alm_traj.empty());
}

// ============================================================
// 测试：alm_preprocessed_traj 预处理粗优化轨迹填充
// ============================================================

// 预处理成功后必须同步填充 alm_preprocessed_traj（预处理粗优化轨迹的离散化
// 结果，作为"优化前"对比基线）：首点锚定初始路径首点，末点在预处理收敛容差
// 内贴近初始路径末点，逐点携带状态/控制量；预处理失败时必须为空。
TEST(PostProcessorAlmTest,
     AlmPreprocessedTrajFilledOnSuccessAndEmptyOnFailure) {
    const auto vehicle_params = MakeVehicleParams();
    const auto footprint = MakeFootprintModel(vehicle_params);
    const auto esdf_map = MakeLargeEmptyEsdfMap();
    const PostProcessor processor(vehicle_params, footprint, esdf_map);
    const auto path = BuildStraightPath(2.0);
    const auto result = processor.optimizeAlm(path, AlmConfig{});
    ASSERT_TRUE(result.success) << result.message;
    ASSERT_FALSE(result.alm_preprocessed_traj.empty());
    // 离散化锚定初始路径首点（世界坐标还原积分的锚点）
    EXPECT_DOUBLE_EQ(result.alm_preprocessed_traj.front().x, path.front().x);
    EXPECT_DOUBLE_EQ(result.alm_preprocessed_traj.front().y, path.front().y);
    for (const auto& pt : result.alm_preprocessed_traj) {
        EXPECT_TRUE(std::isfinite(pt.x));
        EXPECT_TRUE(std::isfinite(pt.y));
        EXPECT_TRUE(std::isfinite(pt.theta));
        EXPECT_TRUE(pt.hasV());
        EXPECT_TRUE(pt.hasDelta());
    }
    // 预处理收敛容差默认 0.1 m，叠加离散化积分还原余量后取 0.15 m 上限
    EXPECT_NEAR(result.alm_preprocessed_traj.back().x, path.back().x, 0.15);
    EXPECT_NEAR(result.alm_preprocessed_traj.back().y, path.back().y, 0.15);
    // 预处理失败：无任何离散化产物
    AlmConfig fail_config;
    fail_config.preprocessor.convergence_position_tolerance = 1e-9;
    const auto failed = processor.optimizeAlm(path, fail_config);
    ASSERT_FALSE(failed.success);
    EXPECT_TRUE(failed.alm_preprocessed_traj.empty());
}

// ============================================================
// 测试：合成场景端到端
// ============================================================

// 空路径输入必须显式失败，不抛异常、不返回半成品。
TEST(PostProcessorAlmTest, EmptyPathReturnsFailure) {
    const auto vehicle_params = MakeVehicleParams();
    const auto footprint = MakeFootprintModel(vehicle_params);
    const auto esdf_map = MakeLargeEmptyEsdfMap();
    const PostProcessor processor(vehicle_params, footprint, esdf_map);
    const Path empty_path;
    const auto result = processor.optimizeAlm(empty_path, AlmConfig{});
    EXPECT_FALSE(result.success);
    EXPECT_TRUE(result.optimized_path.empty());
    EXPECT_FALSE(result.message.empty());
}

// 预处理失败必须显式走失败分支（第 2 道质量门），不抛异常、不返回半成品。
// 把预处理跟踪容差收紧到物理上不可达的量级（合成场景收敛后跟踪误差在
// 1e-3 m 量级，远大于 1e-9），确定性触发预处理失败。
TEST(PostProcessorAlmTest, PreprocessFailureReturnsFailure) {
    const auto vehicle_params = MakeVehicleParams();
    const auto footprint = MakeFootprintModel(vehicle_params);
    const auto esdf_map = MakeLargeEmptyEsdfMap();
    const PostProcessor processor(vehicle_params, footprint, esdf_map);
    AlmConfig alm_config;
    alm_config.preprocessor.convergence_position_tolerance = 1e-9;
    const auto path = BuildStraightPath(2.0);
    const auto result = processor.optimizeAlm(path, alm_config);
    EXPECT_FALSE(result.success);
    EXPECT_TRUE(result.optimized_path.empty());
    EXPECT_NE(result.message.find("preprocessing failed"), std::string::npos);
    EXPECT_TRUE(result.alm_traj.empty());
}

// 空旷直线场景应收敛：终点双指标满足 ALM 收敛判据，轨迹无碰撞、无奇异。
TEST(PostProcessorAlmTest, EndToEndStraightLineConverges) {
    const auto vehicle_params = MakeVehicleParams();
    const auto footprint = MakeFootprintModel(vehicle_params);
    const auto esdf_map = MakeLargeEmptyEsdfMap();
    const PostProcessor processor(vehicle_params, footprint, esdf_map);
    const auto path = BuildStraightPath(2.0);
    const auto result = processor.optimizeAlm(path, AlmConfig{});
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
TEST(PostProcessorAlmTest, EndToEndGearShiftConverges) {
    const auto vehicle_params = MakeVehicleParams();
    const auto footprint = MakeFootprintModel(vehicle_params);
    const auto esdf_map = MakeLargeEmptyEsdfMap();
    const PostProcessor processor(vehicle_params, footprint, esdf_map);
    const auto path = BuildGearShiftPath();
    const auto result = processor.optimizeAlm(path, AlmConfig{});
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
// 四数据集端到端验收（集成测试，允许较长耗时）
// ============================================================

// 数据集描述：显示名 + 文件路径 + 机动段数验收上限
struct AlmDatasetCase {
    std::string name;
    std::string file;
    // 最终机动段数上限断言：pin 住已验证的融化能力（防回归），非劣化下界
    int max_final_maneuvers;
};

class AlmDatasetAcceptanceTest
    : public ::testing::TestWithParam<AlmDatasetCase> {};

// 每个数据集：ALM 路径必须产出无碰撞、无奇异（全部采样点有限）、终点精度
// 达标的轨迹；同时跑 NMPC 路径打印对比指标（NMPC 结果不做门禁）。
TEST_P(AlmDatasetAcceptanceTest, ProducesCollisionFreeSaneTrajectory) {
    const auto& dataset = GetParam();
    ::apa::post_processor::OptimizeRequest request;
    ASSERT_EQ(DataLoader::LoadProtoFromJsonFile(dataset.file, request),
              LoadResult::SUCCESS);
    const auto vehicle_params = VehicleParams::FromProto(request.vehicle());
    // 与生产入口一致的 footprint 默认构造（233/2/4）
    const VehicleFootprintModel footprint_model(vehicle_params);
    const auto grid_map = GridMap::FromProto(request.environment());
    const ESDFMap esdf_map(grid_map);
    const auto init_path = Path::FromProto(request.initial_path());
    ASSERT_FALSE(init_path.empty());
    const PostProcessor processor(vehicle_params, footprint_model, esdf_map);
    const int init_maneuvers = static_cast<int>(init_path.numManeuvers());
    const double init_length = init_path.length();

    // NMPC 路径（默认配置，对比基线）
    const NMPCConfig nmpc_config;
    const auto nmpc_result = processor.optimize(init_path, nmpc_config);
    // ALM 路径（默认配置）
    const AlmConfig alm_config;
    const auto alm_result = processor.optimizeAlm(init_path, alm_config);
    // 打印对比指标行（供验收报告采集）
    const double nmpc_collision =
        nmpc_result.optimized_path.empty()
            ? -1.0
            : ComputeMaxCollisionDepth(nmpc_result.optimized_path, esdf_map,
                                       footprint_model);
    std::cout << "[ALM-CMP] dataset=" << dataset.name << " alg=NMPC"
              << " success=" << nmpc_result.success
              << " maneuvers=" << init_maneuvers << "->"
              << nmpc_result.final_maneuvers << " length=" << init_length
              << "->" << nmpc_result.final_length << " term_pos_err="
              << (nmpc_result.optimized_path.empty()
                      ? -1.0
                      : TerminalPositionError(nmpc_result.optimized_path,
                                              init_path))
              << " term_head_err_deg="
              << (nmpc_result.optimized_path.empty()
                      ? -1.0
                      : TerminalHeadingErrorDeg(nmpc_result.optimized_path,
                                                init_path))
              << " collision=" << nmpc_collision
              << " time_ms=" << nmpc_result.total_time_ms << " msg=\""
              << nmpc_result.message << "\"" << std::endl;
    const double alm_collision =
        alm_result.optimized_path.empty()
            ? -1.0
            : ComputeMaxCollisionDepth(alm_result.optimized_path, esdf_map,
                                       footprint_model);
    std::cout
        << "[ALM-CMP] dataset=" << dataset.name << " alg=ALM"
        << " success=" << alm_result.success << " maneuvers=" << init_maneuvers
        << "->" << alm_result.final_maneuvers << " length=" << init_length
        << "->" << alm_result.final_length << " term_pos_err="
        << (alm_result.optimized_path.empty()
                ? -1.0
                : TerminalPositionError(alm_result.optimized_path, init_path))
        << " term_head_err_deg="
        << (alm_result.optimized_path.empty()
                ? -1.0
                : TerminalHeadingErrorDeg(alm_result.optimized_path, init_path))
        << " collision=" << alm_collision
        << " time_ms=" << alm_result.total_time_ms << " msg=\""
        << alm_result.message << "\"" << std::endl;

    // 验收门禁：ALM 路径必须成功且轨迹无碰撞、无奇异、终点精度达标
    ASSERT_TRUE(alm_result.success) << alm_result.message;
    ASSERT_FALSE(alm_result.optimized_path.empty());
    // 预处理粗优化轨迹（优化前基线）必须同步产出
    EXPECT_FALSE(alm_result.alm_preprocessed_traj.empty());
    // 合法性门禁：validate() 三门（碰撞安全 + 终点收敛 + 运动学可行）
    // 全部通过——运动学可行性取梯形配点残差，ALM 采样点携带时间戳后
    // 该门对 ALM 产出直接生效
    const auto& goal_pt = init_path.back();
    const TrajectoryPoint goal(goal_pt.x, goal_pt.y, goal_pt.theta);
    const auto validation =
        alm_result.alm_traj.validate(goal, esdf_map, footprint_model);
    EXPECT_TRUE(validation.kinematic_feasible) << validation.kinematic_detail;
    EXPECT_TRUE(validation.all_passed) << FormatValidationResult(validation);
    EXPECT_TRUE(IsPathFinite(alm_result.optimized_path));
    EXPECT_LE(alm_collision, 0.02);
    EXPECT_LE(TerminalPositionError(alm_result.optimized_path, init_path),
              0.05);
    EXPECT_LE(TerminalHeadingErrorDeg(alm_result.optimized_path, init_path),
              1.5);
    EXPECT_GT(alm_result.final_length, 0.0);
    // 机动段数不超过该数据集的验收上限：按物理方向段口径（v 变号统计，
    // 含多项式段内过冲）实测段数（data3→7、data1→4、data7→4、data6→4）
    // 收紧并留 1 段余量，防止压缩能力静默回归
    EXPECT_LE(alm_result.final_maneuvers, dataset.max_final_maneuvers);
}

INSTANTIATE_TEST_SUITE_P(
    FourDatasets, AlmDatasetAcceptanceTest,
    ::testing::Values(
        AlmDatasetCase{"data3_mid_park", "data/mid_park/data3.json", 8},
        AlmDatasetCase{"data1_rub_park", "data/rub_park/data1.json", 5},
        AlmDatasetCase{"data7_rub_park", "data/rub_park/data7.json", 5},
        AlmDatasetCase{"data6_long_park", "data/long_park/data6.json", 5}),
    [](const ::testing::TestParamInfo<AlmDatasetCase>& info) {
        return info.param.name;
    });

}  // namespace apa_post_processor
