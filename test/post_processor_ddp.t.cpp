#include <gtest/gtest.h>

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

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

// 公共车辆参数（与 post_processor_alm.t.cpp 的合成场景一致）
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

// 合成小尺度场景的 DDP 配置：mu_min 降到 1.0，防止小代价量级下 μ⁰ 自适应
// 标定被下限 clip 到过强罚权重（λ=100·c 比问题代价本身大 3 个数量级会把
// 内层首轮淹死）；真实数据集 J_s′/‖c‖² 量级在 1e2 以上、不触及下限，
// 生产默认配置不受影响
DdpConfig MakeSyntheticDdpConfig() {
    DdpConfig config;
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
TEST(PostProcessorDdpTest, EmptyPathReturnsFailure) {
    const auto vehicle_params = MakeVehicleParams();
    const auto footprint = MakeFootprintModel(vehicle_params);
    const auto esdf_map = MakeLargeEmptyEsdfMap();
    const PostProcessor processor(vehicle_params, footprint, esdf_map);
    const Path empty_path;
    const auto result = processor.optimizeDdp(empty_path, DdpConfig{});
    EXPECT_FALSE(result.success);
    EXPECT_TRUE(result.optimized_path.empty());
    EXPECT_TRUE(result.ddp_traj.empty());
    EXPECT_FALSE(result.message.empty());
    EXPECT_DOUBLE_EQ(result.final_length, 0.0);
    EXPECT_EQ(result.final_maneuvers, 0);
}

// 退化路径（总长 0.02 m，不足一个重采样间距 0.05 m）无法构建参考——参考
// 构建器抛 std::invalid_argument，编排层必须捕获并转为显式失败，异常不得
// 逃逸到调用方；统计量同样取空路径口径。
TEST(PostProcessorDdpTest, DegeneratePathReturnsFailure) {
    const auto vehicle_params = MakeVehicleParams();
    const auto footprint = MakeFootprintModel(vehicle_params);
    const auto esdf_map = MakeLargeEmptyEsdfMap();
    const PostProcessor processor(vehicle_params, footprint, esdf_map);
    Path path;
    path.addPoint({0.0, 0.0, 0.0});
    path.addPoint({0.02, 0.0, 0.0});
    path.finalize();
    const auto result = processor.optimizeDdp(path, DdpConfig{});
    EXPECT_FALSE(result.success);
    EXPECT_TRUE(result.optimized_path.empty());
    EXPECT_TRUE(result.ddp_traj.empty());
    EXPECT_FALSE(result.message.empty());
    EXPECT_DOUBLE_EQ(result.final_length, 0.0);
    EXPECT_EQ(result.final_maneuvers, 0);
}

// ============================================================
// 测试：DDP 路径不污染配置、不与 ALM 路径互相干扰
// ============================================================

// 调用方传入的 DdpConfig 对象在 optimizeDdp 后必须保持原值（幅值边界同步
// 只允许发生在编排层局部副本上）。
TEST(PostProcessorDdpTest, DoesNotMutateDdpConfig) {
    const auto vehicle_params = MakeVehicleParams();
    const auto footprint = MakeFootprintModel(vehicle_params);
    const auto esdf_map = MakeLargeEmptyEsdfMap();
    const PostProcessor processor(vehicle_params, footprint, esdf_map);
    DdpConfig ddp_config;
    ddp_config.reference.v_max = 1.2;
    ddp_config.solver.cost.v_max = 9.9;  // 故意制造未同步状态
    ddp_config.solver.outer.mu_min = 1.0;
    ddp_config.solver.outer.max_outer_iterations = 3;
    ddp_config.esdf.weight_safe = 250.0;
    ddp_config.post_stage.kappa_pad = 1.1;
    const auto path = BuildStraightPath(2.0);
    processor.optimizeDdp(path, ddp_config);
    EXPECT_DOUBLE_EQ(ddp_config.reference.v_max, 1.2);
    EXPECT_DOUBLE_EQ(ddp_config.solver.cost.v_max, 9.9);
    EXPECT_DOUBLE_EQ(ddp_config.solver.outer.mu_min, 1.0);
    EXPECT_EQ(ddp_config.solver.outer.max_outer_iterations, 3);
    EXPECT_DOUBLE_EQ(ddp_config.esdf.weight_safe, 250.0);
    EXPECT_DOUBLE_EQ(ddp_config.post_stage.kappa_pad, 1.1);
}

// 先跑一次 ALM 路径，再跑 DDP 路径，最后再跑 ALM 路径：两次 ALM 结果必须
// 完全一致，且 ALM 配置对象未被触碰。
TEST(PostProcessorDdpTest, DdpPathDoesNotInterfereWithAlmPath) {
    const auto vehicle_params = MakeVehicleParams();
    const auto footprint = MakeFootprintModel(vehicle_params);
    const auto esdf_map = MakeLargeEmptyEsdfMap();
    const PostProcessor processor(vehicle_params, footprint, esdf_map);
    const auto path = BuildStraightPath(2.0);
    const auto before = processor.optimizeAlm(path, AlmConfig{});
    ASSERT_TRUE(before.success) << before.message;
    const auto ddp_result =
        processor.optimizeDdp(path, MakeSyntheticDdpConfig());
    ASSERT_TRUE(ddp_result.success) << ddp_result.message;
    const auto after = processor.optimizeAlm(path, AlmConfig{});
    ASSERT_TRUE(after.success) << after.message;
    EXPECT_EQ(before.message, after.message);
    EXPECT_EQ(before.final_maneuvers, after.final_maneuvers);
    EXPECT_DOUBLE_EQ(before.final_length, after.final_length);
}

// ============================================================
// 测试：ddp_traj 结果轨迹填充
// ============================================================

// 成功路径必须同步填充 ddp_traj（含驻留与时间戳，逐点携带状态/控制量）；
// 失败路径 ddp_traj 必须为空。
TEST(PostProcessorDdpTest, DdpTrajFilledOnSuccessAndEmptyOnFailure) {
    const auto vehicle_params = MakeVehicleParams();
    const auto footprint = MakeFootprintModel(vehicle_params);
    const auto esdf_map = MakeLargeEmptyEsdfMap();
    const PostProcessor processor(vehicle_params, footprint, esdf_map);
    const auto path = BuildStraightPath(2.0);
    const auto result = processor.optimizeDdp(path, MakeSyntheticDdpConfig());
    ASSERT_TRUE(result.success) << result.message;
    ASSERT_FALSE(result.ddp_traj.empty());
    for (const auto& pt : result.ddp_traj) {
        EXPECT_TRUE(std::isfinite(pt.x));
        EXPECT_TRUE(std::isfinite(pt.y));
        EXPECT_TRUE(std::isfinite(pt.theta));
        EXPECT_TRUE(pt.hasV());
        EXPECT_TRUE(pt.hasDelta());
        EXPECT_TRUE(pt.hasT());
    }
    // 时间戳严格单调（驻留插入只拉伸时间轴）
    for (std::size_t i = 1; i < result.ddp_traj.size(); ++i) {
        EXPECT_GT(result.ddp_traj[i].getT(), result.ddp_traj[i - 1].getT());
    }
    const Path empty_path;
    const auto failed = processor.optimizeDdp(empty_path, DdpConfig{});
    EXPECT_FALSE(failed.success);
    EXPECT_TRUE(failed.ddp_traj.empty());
}

// ============================================================
// 测试：合成场景端到端
// ============================================================

// 空旷直线场景应收敛：终点双指标达标，轨迹无碰撞、无奇异，时间戳完备。
TEST(PostProcessorDdpTest, EndToEndStraightLineConverges) {
    const auto vehicle_params = MakeVehicleParams();
    const auto footprint = MakeFootprintModel(vehicle_params);
    const auto esdf_map = MakeLargeEmptyEsdfMap();
    const PostProcessor processor(vehicle_params, footprint, esdf_map);
    const auto path = BuildStraightPath(2.0);
    const auto result = processor.optimizeDdp(path, MakeSyntheticDdpConfig());
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
TEST(PostProcessorDdpTest, EndToEndGearShiftCompletes) {
    const auto vehicle_params = MakeVehicleParams();
    const auto footprint = MakeFootprintModel(vehicle_params);
    const auto esdf_map = MakeLargeEmptyEsdfMap();
    const PostProcessor processor(vehicle_params, footprint, esdf_map);
    const auto path = BuildGearShiftPath();
    const auto result = processor.optimizeDdp(path, MakeSyntheticDdpConfig());
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
// （ρ_reg 溢出），后处理按设计回退原始路径——钉住"回退必须带诊断"语义：
// 显式失败、不产出半成品、统计量取空路径口径、message 携带失败阶段 +
// 失败项 + 量化值/阈值。
// 若端到端调参后该场景转为收敛，本用例应改写为成功路径断言
TEST(PostProcessorDdpTest, ShortReversalFallsBackWithDiagnostics) {
    const auto vehicle_params = MakeVehicleParams();
    const auto footprint = MakeFootprintModel(vehicle_params);
    const auto esdf_map = MakeLargeEmptyEsdfMap();
    const PostProcessor processor(vehicle_params, footprint, esdf_map);
    const auto path = BuildShortReversalPath();
    const auto result = processor.optimizeDdp(path, MakeSyntheticDdpConfig());
    ASSERT_FALSE(result.success);
    EXPECT_TRUE(result.optimized_path.empty());
    EXPECT_TRUE(result.ddp_traj.empty());
    EXPECT_DOUBLE_EQ(result.final_length, 0.0);
    EXPECT_EQ(result.final_maneuvers, 0);
    EXPECT_NE(result.message.find("STAGE_TWO_NOT_CONVERGED"), std::string::npos)
        << result.message;
    EXPECT_NE(result.message.find("stage_two_convergence"), std::string::npos)
        << result.message;
    EXPECT_NE(result.message.find("measured="), std::string::npos)
        << result.message;
    EXPECT_NE(result.message.find("threshold="), std::string::npos)
        << result.message;
}

// ============================================================
// 四数据集端到端验收（集成测试，允许较长耗时）
// ============================================================

// 数据集描述：显示名 + 文件路径
struct DdpDatasetCase {
    std::string name;
    std::string file;
};

class DdpDatasetAcceptanceTest
    : public ::testing::TestWithParam<DdpDatasetCase> {};

// 每个数据集：DDP 路径必须完整跑通（阶段一 → 后处理/阶段二 → 校验/回退），
// 不抛异常、状态消息非空、耗时为正；成功时轨迹必须无碰撞、无奇异、终点
// 精度达标；回退时必须带结构化诊断（失败项 + 量化值/阈值）。
// 注意：本用例为重型端到端测试（完整阶段一 + 阶段二求解，含 ESDF 求值），
// Debug 构建下四数据集合计约分钟级；如需快速反馈可用 --gtest_filter 排除
TEST_P(DdpDatasetAcceptanceTest, CompletesWithClearStatusAndDiagnostics) {
    const auto& dataset = GetParam();
    ::apa::post_processor::OptimizeRequest request;
    ASSERT_EQ(DataLoader::LoadProtoFromJsonFile(dataset.file, request),
              LoadResult::SUCCESS);
    const auto vehicle_params = VehicleParams::FromProto(request.vehicle());
    // footprint 取 233/2/2：与 DDP 真实数据集冒烟验证及生产配置
    // （data/ddp_config.json 的 outer_row_num=2）一致
    const VehicleFootprintModel footprint_model(vehicle_params, 233, 2, 2);
    const auto grid_map = GridMap::FromProto(request.environment());
    const ESDFMap esdf_map(grid_map);
    const auto init_path = Path::FromProto(request.initial_path());
    ASSERT_FALSE(init_path.empty());
    const PostProcessor processor(vehicle_params, footprint_model, esdf_map);
    const int init_maneuvers = static_cast<int>(init_path.numManeuvers());
    const double init_length = init_path.length();

    const DdpConfig ddp_config;
    const auto result = processor.optimizeDdp(init_path, ddp_config);
    // 打印对比指标行（供验收报告采集）
    const double collision =
        result.optimized_path.empty()
            ? -1.0
            : ComputeMaxCollisionDepth(result.optimized_path, esdf_map,
                                       footprint_model);
    std::cout << "[DDP-CMP] dataset=" << dataset.name << " alg=DDP"
              << " success=" << result.success
              << " maneuvers=" << init_maneuvers << "->"
              << result.final_maneuvers << " length=" << init_length << "->"
              << result.final_length << " term_pos_err="
              << (result.optimized_path.empty()
                      ? -1.0
                      : TerminalPositionError(result.optimized_path, init_path))
              << " term_head_err_deg="
              << (result.optimized_path.empty()
                      ? -1.0
                      : TerminalHeadingErrorDeg(result.optimized_path,
                                                init_path))
              << " collision=" << collision
              << " time_ms=" << result.total_time_ms << " msg=\""
              << result.message << "\"" << std::endl;

    // 通用门禁：完整跑通（不抛异常、状态消息非空、耗时为正）
    EXPECT_FALSE(result.message.empty());
    EXPECT_GT(result.total_time_ms, 0.0);
    if (result.success) {
        // 成功：轨迹无碰撞、无奇异、终点双指标达标、ddp_traj 同步填充
        ASSERT_FALSE(result.optimized_path.empty());
        EXPECT_FALSE(result.ddp_traj.empty());
        EXPECT_TRUE(IsPathFinite(result.optimized_path));
        EXPECT_LE(collision, 0.02);
        EXPECT_LE(TerminalPositionError(result.optimized_path, init_path),
                  0.05);
        EXPECT_LE(TerminalHeadingErrorDeg(result.optimized_path, init_path),
                  1.5);
        EXPECT_GT(result.final_length, 0.0);
        return;
    }
    // 回退：不产出半成品（optimized_path/ddp_traj 均为空），且必须携带
    // 结构化诊断（失败项 + 量化值/阈值）
    EXPECT_TRUE(result.optimized_path.empty());
    EXPECT_TRUE(result.ddp_traj.empty());
    EXPECT_NE(result.message.find("measured="), std::string::npos)
        << result.message;
    EXPECT_NE(result.message.find("threshold="), std::string::npos)
        << result.message;
}

INSTANTIATE_TEST_SUITE_P(
    FourDatasets, DdpDatasetAcceptanceTest,
    ::testing::Values(
        DdpDatasetCase{"data3_mid_park", "data/mid_park/data3.json"},
        DdpDatasetCase{"data1_rub_park", "data/rub_park/data1.json"},
        DdpDatasetCase{"data7_rub_park", "data/rub_park/data7.json"},
        DdpDatasetCase{"data6_long_park", "data/long_park/data6.json"}),
    [](const ::testing::TestParamInfo<DdpDatasetCase>& info) {
        return info.param.name;
    });

}  // namespace apa_post_processor
