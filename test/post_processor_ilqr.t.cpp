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

// 公共车辆参数（与 post_processor_minco.t.cpp 的合成场景一致）
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

// 合成小尺度场景的 iLQR 配置：mu_min 降到 1.0，防止小代价量级下 μ⁰ 自适应
// 标定被下限 clip 到过强罚权重（λ=100·c 比问题代价本身大 3 个数量级会把
// 内层首轮淹死）；真实数据集 J_s′/‖c‖² 量级在 1e2 以上、不触及下限，
// 生产默认配置不受影响
iLQRConfig MakeSyntheticiLQRConfig() {
    iLQRConfig config;
    config.solver.outer.mu_min = 1.0;
    return config;
}

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

// 构造 前进1.0m → 倒退2.0m 的单次换挡路径（2 个机动段）
Path BuildGearShiftPath() {
    Path path;
    path.addPoint({0.0, 0.0, 0.0});
    AppendXLine(&path, 0.0, 1.0, 0.0);
    AppendXLine(&path, 1.0, -1.0, 0.0);
    path.finalize();
    return path;
}

// 构造 前进1.0m → 倒退0.5m 的短倒退换挡路径（倒退段仅 0.5 m，驻留窗占
// 比过半，阶段二门控重解在该形态下实测不收敛，用于钉住回退诊断语义）
Path BuildShortReversalPath() {
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
// 测试：失败语义（显式失败、不抛异常、不返回半成品）
// ============================================================

// 空路径输入必须显式失败，不抛异常、不返回半成品，统计量取空路径口径
// （final_length/final_maneuvers 均为 0，钉住空路径显式守卫）。
TEST(PostProcessoriLQRTest, EmptyPathReturnsFailure) {
    const auto vehicle_params = MakeVehicleParams();
    const auto footprint = MakeFootprintModel(vehicle_params);
    const auto esdf_map = MakeLargeEmptyEsdfMap();
    const PostProcessor processor(vehicle_params, footprint, esdf_map);
    const Path empty_path;
    const auto result = processor.optimizeiLQR(empty_path, iLQRConfig{});
    EXPECT_FALSE(result.success);
    EXPECT_TRUE(result.optimized_path.empty());
    EXPECT_TRUE(result.optimized_trajectory.empty());
    EXPECT_FALSE(result.message.empty());
    EXPECT_DOUBLE_EQ(result.final_length, 0.0);
    EXPECT_EQ(result.final_maneuvers, 0);
}

// 退化路径（总长 0.02 m，不足一个重采样间距 0.05 m）无法构建参考——参考
// 构建器抛 std::invalid_argument，编排层必须捕获并转为显式失败，异常不得
// 逃逸到调用方；统计量同样取空路径口径。
TEST(PostProcessoriLQRTest, DegeneratePathReturnsFailure) {
    const auto vehicle_params = MakeVehicleParams();
    const auto footprint = MakeFootprintModel(vehicle_params);
    const auto esdf_map = MakeLargeEmptyEsdfMap();
    const PostProcessor processor(vehicle_params, footprint, esdf_map);
    Path path;
    path.addPoint({0.0, 0.0, 0.0});
    path.addPoint({0.02, 0.0, 0.0});
    path.finalize();
    const auto result = processor.optimizeiLQR(path, iLQRConfig{});
    EXPECT_FALSE(result.success);
    EXPECT_TRUE(result.optimized_path.empty());
    EXPECT_TRUE(result.optimized_trajectory.empty());
    EXPECT_FALSE(result.message.empty());
    EXPECT_DOUBLE_EQ(result.final_length, 0.0);
    EXPECT_EQ(result.final_maneuvers, 0);
}

// ============================================================
// 测试：iLQR 路径不污染配置、不与 MINCO 路径互相干扰
// ============================================================

// 调用方传入的 iLQRConfig 对象在 optimizeiLQR 后必须保持原值（幅值边界同步
// 只允许发生在编排层局部副本上）。
TEST(PostProcessoriLQRTest, DoesNotMutateiLQRConfig) {
    const auto vehicle_params = MakeVehicleParams();
    const auto footprint = MakeFootprintModel(vehicle_params);
    const auto esdf_map = MakeLargeEmptyEsdfMap();
    const PostProcessor processor(vehicle_params, footprint, esdf_map);
    iLQRConfig ilqr_config;
    ilqr_config.reference.v_max = 1.2;
    ilqr_config.solver.cost.v_max = 9.9;  // 故意制造未同步状态
    ilqr_config.solver.outer.mu_min = 1.0;
    ilqr_config.solver.outer.max_outer_iterations = 3;
    ilqr_config.esdf.weight_safe = 250.0;
    ilqr_config.post_stage.kappa_pad = 1.1;
    const auto path = BuildStraightPath(2.0);
    processor.optimizeiLQR(path, ilqr_config);
    EXPECT_DOUBLE_EQ(ilqr_config.reference.v_max, 1.2);
    EXPECT_DOUBLE_EQ(ilqr_config.solver.cost.v_max, 9.9);
    EXPECT_DOUBLE_EQ(ilqr_config.solver.outer.mu_min, 1.0);
    EXPECT_EQ(ilqr_config.solver.outer.max_outer_iterations, 3);
    EXPECT_DOUBLE_EQ(ilqr_config.esdf.weight_safe, 250.0);
    EXPECT_DOUBLE_EQ(ilqr_config.post_stage.kappa_pad, 1.1);
}

// 先跑一次 MINCO 路径，再跑 iLQR 路径，最后再跑 MINCO 路径：两次 MINCO 结果必须
// 完全一致，且 MINCO 配置对象未被触碰。
TEST(PostProcessoriLQRTest, iLQRPathDoesNotInterfereWithMincoPath) {
    const auto vehicle_params = MakeVehicleParams();
    const auto footprint = MakeFootprintModel(vehicle_params);
    const auto esdf_map = MakeLargeEmptyEsdfMap();
    const PostProcessor processor(vehicle_params, footprint, esdf_map);
    const auto path = BuildStraightPath(2.0);
    const auto before = processor.optimizeMinco(path, MincoConfig{});
    ASSERT_TRUE(before.success) << before.message;
    const auto ilqr_result =
        processor.optimizeiLQR(path, MakeSyntheticiLQRConfig());
    ASSERT_TRUE(ilqr_result.success) << ilqr_result.message;
    const auto after = processor.optimizeMinco(path, MincoConfig{});
    ASSERT_TRUE(after.success) << after.message;
    EXPECT_EQ(before.message, after.message);
    EXPECT_EQ(before.final_maneuvers, after.final_maneuvers);
    EXPECT_DOUBLE_EQ(before.final_length, after.final_length);
}

// ============================================================
// 测试：ilqr_traj 结果轨迹填充
// ============================================================

// 成功路径必须同步填充 ilqr_traj（含驻留与时间戳，逐点携带状态/控制量）；
// 失败路径 ilqr_traj 必须为空。
TEST(PostProcessoriLQRTest, iLQRTrajFilledOnSuccessAndEmptyOnFailure) {
    const auto vehicle_params = MakeVehicleParams();
    const auto footprint = MakeFootprintModel(vehicle_params);
    const auto esdf_map = MakeLargeEmptyEsdfMap();
    const PostProcessor processor(vehicle_params, footprint, esdf_map);
    const auto path = BuildStraightPath(2.0);
    const auto result = processor.optimizeiLQR(path, MakeSyntheticiLQRConfig());
    ASSERT_TRUE(result.success) << result.message;
    ASSERT_FALSE(result.optimized_trajectory.empty());
    for (const auto& pt : result.optimized_trajectory) {
        EXPECT_TRUE(std::isfinite(pt.x));
        EXPECT_TRUE(std::isfinite(pt.y));
        EXPECT_TRUE(std::isfinite(pt.theta));
        EXPECT_TRUE(pt.hasV());
        EXPECT_TRUE(pt.hasDelta());
        EXPECT_TRUE(pt.hasT());
    }
    // 时间戳严格单调（驻留插入只拉伸时间轴）
    for (std::size_t i = 1; i < result.optimized_trajectory.size(); ++i) {
        EXPECT_GT(result.optimized_trajectory[i].getT(), result.optimized_trajectory[i - 1].getT());
    }
    const Path empty_path;
    const auto failed = processor.optimizeiLQR(empty_path, iLQRConfig{});
    EXPECT_FALSE(failed.success);
    EXPECT_TRUE(failed.optimized_trajectory.empty());
}

// ============================================================
// 测试：合成场景端到端
// ============================================================

// 空旷直线场景应收敛：终点双指标达标，轨迹无碰撞、无奇异，时间戳完备。
TEST(PostProcessoriLQRTest, EndToEndStraightLineConverges) {
    const auto vehicle_params = MakeVehicleParams();
    const auto footprint = MakeFootprintModel(vehicle_params);
    const auto esdf_map = MakeLargeEmptyEsdfMap();
    const PostProcessor processor(vehicle_params, footprint, esdf_map);
    const auto path = BuildStraightPath(2.0);
    const auto result = processor.optimizeiLQR(path, MakeSyntheticiLQRConfig());
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

// 单次真实换挡场景（前进 1.0 m → 倒退 2.0 m）：阶段二门控重解收敛，
// 接缝零速 + 驻留插入后输出合法轨迹；物理方向段数 1~2（空旷场景允许
// 合法融化冗余换挡，也允许保留换挡），终点双指标与碰撞门达标。
TEST(PostProcessoriLQRTest, EndToEndGearShiftCompletes) {
    const auto vehicle_params = MakeVehicleParams();
    const auto footprint = MakeFootprintModel(vehicle_params);
    const auto esdf_map = MakeLargeEmptyEsdfMap();
    const PostProcessor processor(vehicle_params, footprint, esdf_map);
    const auto path = BuildGearShiftPath();
    const auto result = processor.optimizeiLQR(path, MakeSyntheticiLQRConfig());
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

// 短倒退换挡场景（倒退段仅 0.5 m）：实测阶段二门控重解内层未收敛
// （ρ_reg 溢出），但阶段一解干净收敛——分级降级结构应输出阶段一降级
// 候选（阶段一解 + 修剪 + 驻留插入，过同一合法性门）而非整体回退。
// 钉住"降级不伪装"语义：显式成功但 message 如实反映降级级别与原因，
// 输出轨迹过碰撞/终点双指标门，物理方向段数不增。
// 若端到端调参后该场景阶段二转为收敛，本用例应改写为阶段二成功路径断言
TEST(PostProcessoriLQRTest, ShortReversalOutputsStageOneCandidate) {
    const auto vehicle_params = MakeVehicleParams();
    const auto footprint = MakeFootprintModel(vehicle_params);
    const auto esdf_map = MakeLargeEmptyEsdfMap();
    const PostProcessor processor(vehicle_params, footprint, esdf_map);
    const auto path = BuildShortReversalPath();
    const auto result = processor.optimizeiLQR(path, MakeSyntheticiLQRConfig());
    ASSERT_TRUE(result.success) << result.message;
    EXPECT_NE(result.message.find("stage-one candidate"), std::string::npos)
        << result.message;
    EXPECT_NE(result.message.find("stage_two_convergence"), std::string::npos)
        << result.message;
    EXPECT_FALSE(result.optimized_path.empty());
    EXPECT_FALSE(result.optimized_trajectory.empty());
    EXPECT_TRUE(IsPathFinite(result.optimized_path));
    EXPECT_LE(TerminalPositionError(result.optimized_path, path), 0.05);
    EXPECT_LE(TerminalHeadingErrorDeg(result.optimized_path, path), 1.5);
    EXPECT_LE(
        ComputeMaxCollisionDepth(result.optimized_path, esdf_map, footprint),
        0.02);
    EXPECT_GE(result.final_maneuvers, 1);
    EXPECT_LE(result.final_maneuvers, 2);
}

// 双候选择优规则真值表（L7.2）：成功优先 → maneuver 数少优先 → 长度短
// 优先；完全持平保持融化候选（现状语义）。规则必须对 data7 型对照
// （同 maneuver、对照更短）选出关融化候选、对 data3/data1 型对照
// （对照回退）保持融化候选
TEST(PostProcessoriLQRTest, PreferControlCandidateTruthTable) {
    PostProcessorResult melt;
    melt.success = true;
    melt.final_maneuvers = 4;
    melt.final_length = 16.74;
    PostProcessorResult control;
    control.success = true;
    control.final_maneuvers = 4;
    control.final_length = 15.54;
    // 同 maneuver、对照更短 → 选对照（data7 型）
    EXPECT_TRUE(PostProcessor::PreferControlCandidate(melt, control));
    // 对照失败 → 保持融化候选（data3/data1 型）
    control.success = false;
    EXPECT_FALSE(PostProcessor::PreferControlCandidate(melt, control));
    control.success = true;
    // 融化 maneuver 更少 → 保持融化候选（哪怕对照更短）
    melt.final_maneuvers = 3;
    EXPECT_FALSE(PostProcessor::PreferControlCandidate(melt, control));
    // 对照 maneuver 更少 → 选对照
    melt.final_maneuvers = 4;
    control.final_maneuvers = 3;
    control.final_length = 20.0;
    EXPECT_TRUE(PostProcessor::PreferControlCandidate(melt, control));
    control.final_maneuvers = 4;
    control.final_length = 15.54;
    // 融化失败 → 选对照（诊断不更差）
    melt.success = false;
    EXPECT_TRUE(PostProcessor::PreferControlCandidate(melt, control));
    // 完全持平 → 保持融化候选
    melt.success = true;
    control.final_length = melt.final_length;
    EXPECT_FALSE(PostProcessor::PreferControlCandidate(melt, control));
}

// ============================================================
// 测试：PostProcessorResult 重构新字段（Phase 1 双写验证）
// ============================================================

// 成功路径：optimized_trajectory 包含完整运动学量与时间戳
TEST(PostProcessoriLQRTest, OptimizedTrajectoryContainsFullKinematics) {
    const auto vehicle_params = MakeVehicleParams();
    const auto footprint = MakeFootprintModel(vehicle_params);
    const auto esdf_map = MakeLargeEmptyEsdfMap();
    const PostProcessor processor(vehicle_params, footprint, esdf_map);
    const auto path = BuildStraightPath(2.0);
    const auto result = processor.optimizeiLQR(path, MakeSyntheticiLQRConfig());
    ASSERT_TRUE(result.success) << result.message;
    ASSERT_FALSE(result.optimized_trajectory.empty());
    for (const auto& pt : result.optimized_trajectory) {
        EXPECT_TRUE(std::isfinite(pt.x));
        EXPECT_TRUE(std::isfinite(pt.y));
        EXPECT_TRUE(std::isfinite(pt.theta));
        EXPECT_TRUE(pt.hasV());
        EXPECT_TRUE(pt.hasDelta());
        EXPECT_TRUE(pt.hasT());
    }
    // 时间戳严格单调（驻留插入只拉伸时间轴）
    for (std::size_t i = 1; i < result.optimized_trajectory.size(); ++i) {
        EXPECT_GT(result.optimized_trajectory[i].getT(),
                  result.optimized_trajectory[i - 1].getT());
    }
}

// algorithm 字段必须反映实际运行的求解器
TEST(PostProcessoriLQRTest, AlgorithmFieldIsiLQR) {
    const auto vehicle_params = MakeVehicleParams();
    const auto footprint = MakeFootprintModel(vehicle_params);
    const auto esdf_map = MakeLargeEmptyEsdfMap();
    const PostProcessor processor(vehicle_params, footprint, esdf_map);
    const auto path = BuildStraightPath(2.0);
    const auto result = processor.optimizeiLQR(path, MakeSyntheticiLQRConfig());
    ASSERT_TRUE(result.success);
    EXPECT_EQ(result.algorithm, "ilqr");
}

// output_level 分级语义：成功=2, 降级=1, 失败=0
TEST(PostProcessoriLQRTest, OutputLevelReflectsResultQuality) {
    const auto vehicle_params = MakeVehicleParams();
    const auto footprint = MakeFootprintModel(vehicle_params);
    const auto esdf_map = MakeLargeEmptyEsdfMap();
    const PostProcessor processor(vehicle_params, footprint, esdf_map);
    // 直线场景：阶段二应收敛，输出级别为完全成功
    {
        const auto path = BuildStraightPath(2.0);
        const auto result =
            processor.optimizeiLQR(path, MakeSyntheticiLQRConfig());
        ASSERT_TRUE(result.success);
        EXPECT_EQ(result.output_level, OutputLevel::kFullSuccess);
    }
    // 短倒退场景：阶段二不收敛，降级到阶段一候选
    {
        const auto path = BuildShortReversalPath();
        const auto result =
            processor.optimizeiLQR(path, MakeSyntheticiLQRConfig());
        ASSERT_TRUE(result.success);
        EXPECT_EQ(result.output_level, OutputLevel::kDegraded);
    }
    // 空路径：直接失败
    {
        const Path empty_path;
        const auto result = processor.optimizeiLQR(empty_path, iLQRConfig{});
        EXPECT_FALSE(result.success);
        EXPECT_EQ(result.output_level, OutputLevel::kFallback);
    }
}

}  // namespace apa_post_processor
