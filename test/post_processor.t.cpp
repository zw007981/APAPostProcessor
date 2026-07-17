#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "core/NMPC/nmpc_solver.h"
#include "core/post_processor.h"
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

// 公共车辆参数
VehicleParams MakeVehicleParams() {
    return VehicleParams(/*length=*/4.3, /*width=*/1.8, /*wheelbase=*/2.7,
                         /*max_steer_angle=*/0.6, /*rear_overhang=*/0.8,
                         /*max_accel=*/1.0, /*max_decel=*/-1.5,
                         /*max_steer_rate=*/0.4);
}

// 大地图：确保车辆不越界
ESDFMap MakeLargeEmptyEsdfMap() {
    const GridMap grid_map(0.1, 400, 300, Position{-10.0, -10.0}, {});
    return ESDFMap(grid_map);
}

// Footprint 模型：single outer row 简化几何
VehicleFootprintModel MakeFootprintModel() {
    return VehicleFootprintModel(MakeVehicleParams(),
                                 /*heading_sample_num=*/233,
                                 /*inner_row_num=*/2, /*outer_row_num=*/1);
}

// 构造一条长直前进机动段 (0,0,0)->(5,0,0)
Maneuver MakeLongStraightManeuver() {
    std::vector<TrajectoryPoint> points;
    for (double x = 0.0; x <= 5.0 + EPSILON; x += 0.1) {
        points.emplace_back(TrajectoryPoint{std::min(x, 5.0), 0.0, 0.0});
    }
    return Maneuver(std::move(points), Direction::FORWARD);
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

// 构造一条含真实换挡的往返路径：前进5m -> 后退3m，共2个机动段。
// 预处理管线会在换挡点注入零速转向补丁，用于验证 NMPC 失败回退到预处理轨迹时
// 拓扑清洗仍会正确合并这些补丁段，不让最终机动段数超过输入的2段。
Path MakeSwitchbackPath() {
    Path path;
    for (double x = 0.0; x <= 5.0 + EPSILON; x += 0.1) {
        path.addPoint(Pose(std::min(x, 5.0), 0.0, 0.0));
    }
    for (double x = 5.0; x >= 2.0 - EPSILON; x -= 0.1) {
        path.addPoint(Pose(std::max(x, 2.0), 0.0, PI));
    }
    return path;
}

// 派生测试夹具，暴露 protected 静态方法以做白盒测试。
class PostProcessorTestAccess : public PostProcessor {
   public:
    PostProcessorTestAccess(const VehicleParams& vehicle_params,
                            const VehicleFootprintModel& footprint_model,
                            const ESDFMap& esdf_map)
        : PostProcessor(vehicle_params, footprint_model, esdf_map) {}
    static NMPCConfig CallApplyRetryConfig(
        const NMPCConfig& base_config, const AdaptiveRetryConfig& retry_config,
        int retry_idx) {
        return applyRetryConfig(base_config, retry_config, retry_idx);
    }
};

}  // namespace

// ============================================================
// 测试：applyRetrySpacing 正确修改局部副本
// ============================================================

// 第 0 次重试应将 dense_step_dist 乘以第一个乘数，nominal_step_s
// 乘以第一个乘数，并保持静态走廊开关。
TEST(PostProcessorTest, ApplyRetryConfigScalesDenseAndNominal) {
    NMPCConfig base_config;
    base_config.bspline.dense_step_dist = 0.05;
    base_config.resampler.nominal_step_s = 0.15;
    base_config.use_static_corridor = true;
    AdaptiveRetryConfig retry_config;
    retry_config.dense_step_dist_multipliers = {1.0, 1.0};
    retry_config.nominal_step_s_multipliers = {2.0, 3.0};
    retry_config.use_static_corridor_flags = {true, false};

    const auto scaled = PostProcessorTestAccess::CallApplyRetryConfig(
        base_config, retry_config, 0);

    EXPECT_DOUBLE_EQ(scaled.bspline.dense_step_dist, 0.05);
    EXPECT_DOUBLE_EQ(scaled.resampler.nominal_step_s, 0.30);
    EXPECT_TRUE(scaled.use_static_corridor);
    // 原始配置未被修改
    EXPECT_DOUBLE_EQ(base_config.bspline.dense_step_dist, 0.05);
    EXPECT_DOUBLE_EQ(base_config.resampler.nominal_step_s, 0.15);
    EXPECT_TRUE(base_config.use_static_corridor);
}

// 第 1 次重试应使用第二个乘数，并按配置关闭静态走廊。
TEST(PostProcessorTest, ApplyRetryConfigDisablesCorridorOnSecondRetry) {
    NMPCConfig base_config;
    base_config.bspline.dense_step_dist = 0.05;
    base_config.resampler.nominal_step_s = 0.15;
    base_config.use_static_corridor = true;
    AdaptiveRetryConfig retry_config;
    retry_config.dense_step_dist_multipliers = {1.0, 1.0};
    retry_config.nominal_step_s_multipliers = {2.0, 3.0};
    retry_config.use_static_corridor_flags = {true, false};

    const auto scaled = PostProcessorTestAccess::CallApplyRetryConfig(
        base_config, retry_config, 1);

    EXPECT_DOUBLE_EQ(scaled.bspline.dense_step_dist, 0.05);
    EXPECT_DOUBLE_EQ(scaled.resampler.nominal_step_s, 0.45);
    EXPECT_FALSE(scaled.use_static_corridor);
}

// 乘数列表与走廊标志越界时应回退到最后一个值。
TEST(PostProcessorTest, ApplyRetryConfigFallsBackToLastValues) {
    NMPCConfig base_config;
    base_config.bspline.dense_step_dist = 0.05;
    base_config.resampler.nominal_step_s = 0.15;
    base_config.use_static_corridor = true;
    AdaptiveRetryConfig retry_config;
    retry_config.dense_step_dist_multipliers = {1.0};
    retry_config.nominal_step_s_multipliers = {2.0};
    retry_config.use_static_corridor_flags = {true};

    const auto scaled = PostProcessorTestAccess::CallApplyRetryConfig(
        base_config, retry_config, 2);

    EXPECT_DOUBLE_EQ(scaled.bspline.dense_step_dist, 0.05);
    EXPECT_DOUBLE_EQ(scaled.resampler.nominal_step_s, 0.30);
    EXPECT_TRUE(scaled.use_static_corridor);
}

// ============================================================
// 测试：optimize 不泄漏配置修改
// ============================================================

// 默认参数下 NMPC
// 失败（max_iter=0）会触发重试；调用方传入的配置对象应保持原值。
TEST(PostProcessorTest, DoesNotLeakConfigAfterRetry) {
    const auto vehicle_params = MakeVehicleParams();
    const auto footprint = MakeFootprintModel();
    const auto esdf_map = MakeLargeEmptyEsdfMap();
    const PostProcessor processor(vehicle_params, footprint, esdf_map);

    NMPCConfig nmpc_config;
    nmpc_config.max_iter = 0;  // 强制 NMPC 在 validateProblem 阶段失败
    nmpc_config.use_static_corridor = false;
    const double original_dense = nmpc_config.bspline.dense_step_dist;
    const double original_nominal = nmpc_config.resampler.nominal_step_s;

    AdaptiveRetryConfig retry_config;
    retry_config.max_retries = 2;
    retry_config.dense_step_dist_multipliers = {2.0, 3.0};
    retry_config.nominal_step_s_multipliers = {1.0, 1.5};

    const auto path = MakePathFromManeuver(MakeLongStraightManeuver());
    const auto result = processor.optimize(path, nmpc_config, retry_config);

    // 输入配置对象必须保持原值
    EXPECT_DOUBLE_EQ(nmpc_config.bspline.dense_step_dist, original_dense);
    EXPECT_DOUBLE_EQ(nmpc_config.resampler.nominal_step_s, original_nominal);
    // NMPC 失败但预处理成功 → 直接回退到预处理轨迹（无需重试）。
    // 重构后 PostProcessor 质量门已在 runSingleAttempt 中直接兜底，
    // 不再走外层的 AdaptiveRetry 重试循环。
    EXPECT_TRUE(result.success);
    EXPECT_FALSE(result.optimized_path.empty());
    EXPECT_NE(result.message.find("falling back"), std::string::npos);
}

// 回归测试：NMPC 失败回退到预处理轨迹时，拓扑清洗必须同样应用到 fallback 路径。
// 场景：2 段真实换挡（前进5m + 后退3m，航向反转180°），预处理管线会在换挡点注入
// 零速转向补丁段；若 fallback 分支遗漏拓扑清洗（历史 bug），补丁段会被
// Path::finalize() 按方向变化拆分成比输入更多的 maneuver，直接违反"机动段数
// 不劣化"的验收要求。修复后 fallback 路径的机动段数不应超过输入路径。
TEST(PostProcessorTest,
     FallbackPathAppliesTopologyCleanupAndDoesNotInflateManeuverCount) {
    const auto vehicle_params = MakeVehicleParams();
    const auto footprint = MakeFootprintModel();
    const auto esdf_map = MakeLargeEmptyEsdfMap();
    const PostProcessor processor(vehicle_params, footprint, esdf_map);

    NMPCConfig nmpc_config;
    nmpc_config.max_iter =
        0;  // 强制 NMPC 在 validateProblem 阶段失败，触发 fallback
    nmpc_config.use_static_corridor = false;

    const auto path = MakeSwitchbackPath();
    const auto init_maneuvers = path.numManeuvers();
    const auto result = processor.optimize(path, nmpc_config);

    ASSERT_TRUE(result.success);
    EXPECT_NE(result.message.find("falling back"), std::string::npos);
    EXPECT_LE(result.final_maneuvers, static_cast<int>(init_maneuvers));
}

// ============================================================
// 测试：正常路径下的端到端链路
// ============================================================

// 默认参数在长直空旷场景应成功产出非空路径。
TEST(PostProcessorTest, EndToEndSingleManeuverConverges) {
    const auto vehicle_params = MakeVehicleParams();
    const auto footprint = MakeFootprintModel();
    const auto esdf_map = MakeLargeEmptyEsdfMap();
    const PostProcessor processor(vehicle_params, footprint, esdf_map);

    NMPCConfig nmpc_config;
    nmpc_config.max_iter = 50;
    nmpc_config.use_static_corridor = false;  // 空地图无梯度，关闭走廊

    const auto path = MakePathFromManeuver(MakeLongStraightManeuver());
    const auto result = processor.optimize(path, nmpc_config);

    EXPECT_TRUE(result.success);
    EXPECT_FALSE(result.optimized_path.empty());
    EXPECT_GT(result.final_length, 0.0);
    EXPECT_GT(result.total_time_ms, 0.0);
}
}  // namespace apa_post_processor
