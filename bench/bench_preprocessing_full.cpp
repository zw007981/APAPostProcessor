// 预处理管线统一性能基准：按 Milestone 005→006→007→008→009→010 顺序，
// 依次测量各子阶段耗时，最后测量端到端总耗时。
// 所有测试统一使用 data/config.json 指向的 data3.json 真实数据集，
// 每个用例重复 4 次取均值，不再区分长短场景。
//
// 预计算上下文在首次初始化时跑通完整管线一次以构建各阶段的输入数据，
// 后续各 benchmark 仅对目标阶段重新计时，上游输入均复用预计算结果。

#include <benchmark/benchmark.h>

#include <algorithm>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "bench_preprocessing_utils.h"
#include "preprocessing/adaptive_resampler.h"
#include "preprocessing/bspline_smoother.h"
#include "preprocessing/differential_flatness_solver.h"
#include "preprocessing/preprocessing_pipeline.h"
#include "preprocessing/speed_profile_planner.h"
#include "preprocessing/static_corridor_builder.h"
#include "spatial/esdf_map.h"
#include "spatial/grid_map.h"
#include "util/data_loader.hpp"
#include "util/path.h"
#include "vehicle/vehicle_footprint_model.h"
#include "vehicle/vehicle_params.h"

namespace apa_post_processor {
namespace {

using namespace apa_post_processor::bench_utils;

// ============================================================
// 全局上下文：一次性加载 data3.json，预计算所有阶段所需的输入数据。
// 各 benchmark 仅对目标阶段重新计时，不重复执行上游阶段。
// ============================================================
struct FullBenchmarkContext {
    // ── 车辆与环境 ──
    VehicleParams vehicle_params;
    std::unique_ptr<VehicleFootprintModel> footprint_model;
    std::unique_ptr<ESDFMap> esdf_map;
    Path path;

    // ── 各阶段预计算输入（由首次初始化构建，供各 benchmark 复用）──
    // Stage 1 输入：机动段列表（即 maneuvers 本身，无需额外存储）

    // Stage 2 输入：由 BSplineSmoother 结果构建的速度规划输入
    std::vector<BSplineSmoother::Result> smooth_results;
    std::vector<SpeedProfileInput> speed_inputs;
    std::vector<std::vector<int>> direction_signs_vec;

    // Stage 3 输入：微分平坦补全输入
    std::vector<DifferentialFlatnessInput> diff_inputs;

    // Stage 4 输入：自适应重采样段列表
    std::vector<AdaptiveResamplerSegmentInput> resampler_segments;

    // Stage 5 输入：固定维数 Z_ref
    std::vector<TrajectoryPoint> z_ref;

    // ── 统计信息（由首次初始化记录，用于 report counters）──
    int total_maneuvers = 0;
    int successful_maneuvers = 0;
    double total_path_length_m = 0.0;
};

const FullBenchmarkContext& GetFullContext() {
    static const FullBenchmarkContext ctx = []() {
        FullBenchmarkContext c;

        // ── 加载数据 ──
        // 支持通过环境变量 APA_BENCH_DATA_FILE 覆盖 config.json 中的路径，
        // 便于在多个数据集上批量跑 benchmark 而无需反复修改配置文件。
        std::string data_file_path;
        const char* env_data_path = std::getenv("APA_BENCH_DATA_FILE");
        if (env_data_path != nullptr && env_data_path[0] != '\0') {
            data_file_path = env_data_path;
        } else {
            nlohmann::json config_json;
            constexpr const char* kConfigPath = "data/config.json";
            if (DataLoader::LoadJsonFile(kConfigPath, config_json) !=
                LoadResult::SUCCESS) {
                throw std::runtime_error(
                    "bench_preprocessing_full: failed to load config.json");
            }
            data_file_path = config_json["data_file_path"].get<std::string>();
        }

        ::apa::post_processor::OptimizeRequest request;
        if (DataLoader::LoadProtoFromJsonFile(data_file_path, request) !=
            LoadResult::SUCCESS) {
            throw std::runtime_error(
                "bench_preprocessing_full: failed to load data file: " +
                data_file_path);
        }

        c.vehicle_params = VehicleParams::FromProto(request.vehicle());
        c.footprint_model =
            std::make_unique<VehicleFootprintModel>(c.vehicle_params);
        const auto grid_map = GridMap::FromProto(request.environment());
        c.esdf_map = std::make_unique<ESDFMap>(grid_map);
        c.path = Path::FromProto(request.initial_path());
        const auto& maneuvers = c.path.getManeuvers();

        if (maneuvers.empty()) {
            throw std::runtime_error(
                "bench_preprocessing_full: no maneuvers in data file");
        }

        c.total_maneuvers = static_cast<int>(maneuvers.size());

        // ── 预计算：逐机动段跑通 005→006→007 三个阶段 ──
        BSplineSmootherBenchmarkAccessor smoother(
            BSplineSmootherConfig{}, c.vehicle_params, *c.footprint_model,
            *c.esdf_map);
        SpeedProfilePlanner speed_planner(SpeedProfilePlannerConfig{});
        DifferentialFlatnessSolver diff_solver(
            DifferentialFlatnessSolverConfig{});

        c.smooth_results.reserve(maneuvers.size());
        c.speed_inputs.reserve(maneuvers.size());
        c.direction_signs_vec.reserve(maneuvers.size());
        c.diff_inputs.reserve(maneuvers.size());
        c.resampler_segments.reserve(maneuvers.size());

        for (const auto& maneuver : maneuvers) {
            // Stage 1: B 样条平滑
            auto smooth_result = smoother.smooth(maneuver);
            if (!smooth_result.success || smooth_result.dense_points.empty()) {
                continue;
            }

            // 构建 Stage 2 输入：速度规划
            auto speed_input =
                BuildSpeedProfileInput(smooth_result, smoother, *c.esdf_map);
            std::vector<int> signs(smooth_result.dense_points.size(),
                                   BenchDirectionToSign(maneuver.direction));

            // Stage 2: 速度规划
            auto speed_result =
                speed_planner.plan(speed_input, c.vehicle_params, signs);
            if (!speed_result.success) {
                c.smooth_results.push_back(std::move(smooth_result));
                continue;
            }

            // 构建 Stage 3 输入：微分平坦
            auto diff_input = BuildDifferentialFlatnessInput(
                smooth_result, smoother, speed_result);

            // Stage 3: 微分平坦补全
            auto diff_result = diff_solver.solve(diff_input, c.vehicle_params);
            if (!diff_result.success) {
                c.smooth_results.push_back(std::move(smooth_result));
                c.speed_inputs.push_back(std::move(speed_input));
                c.direction_signs_vec.push_back(std::move(signs));
                continue;
            }

            // 构建 Stage 4 输入：重采样段
            auto segment = BuildAdaptiveResamplerSegmentInput(
                smooth_result, diff_result, maneuver.direction);

            // 累计路径长度
            c.total_path_length_m += maneuver.length();

            // 保存所有中间结果
            c.smooth_results.push_back(std::move(smooth_result));
            c.speed_inputs.push_back(std::move(speed_input));
            c.direction_signs_vec.push_back(std::move(signs));
            c.diff_inputs.push_back(std::move(diff_input));
            c.resampler_segments.push_back(std::move(segment));
            c.successful_maneuvers++;
        }

        // ── Stage 4: 自适应重采样（所有机动段一起）──
        if (!c.resampler_segments.empty()) {
            AdaptiveResampler resampler(AdaptiveResamplerConfig{});
            auto resample_result =
                resampler.resample(c.resampler_segments, c.vehicle_params);
            if (resample_result.success && !resample_result.points.empty()) {
                c.z_ref = std::move(resample_result.points);
            }
        }

        if (c.successful_maneuvers == 0) {
            throw std::runtime_error(
                "bench_preprocessing_full: all maneuvers failed in "
                "pre-computation");
        }

        return c;
    }();
    return ctx;
}

// ============================================================
// 辅助：为每个 benchmark 注册统一的 4 次重复 + 仅聚合输出
// ============================================================
void ApplyStandardArgs(benchmark::internal::Benchmark* b) {
    b->Unit(benchmark::kMillisecond)
        ->Repetitions(4)
        ->ReportAggregatesOnly(true);
}

// ============================================================
// Benchmark 1: B 样条分段平滑 (Milestone 005)
// 测量 BSplineSmoother::smooth() 所有机动段的总耗时。
// ============================================================
static void BM_Stage1_BSplineSmoother(benchmark::State& state) {
    const auto& ctx = GetFullContext();
    BSplineSmootherBenchmarkAccessor smoother(
        BSplineSmootherConfig{}, ctx.vehicle_params, *ctx.footprint_model,
        *ctx.esdf_map);

    // 跨所有 repetition 累计质量遥测指标，最终取平均或最大值。
    int non_converged_sum = 0;
    int successful_sum = 0;
    double max_intrusion_depth = 0.0;

    for (auto _ : state) {
        double total_intrusion = 0.0;
        int ok_count = 0;
        int nc_count = 0;
        double max_intr = 0.0;
        const auto& maneuvers = ctx.path.getManeuvers();
        for (const auto& maneuver : maneuvers) {
            auto result = smoother.smooth(maneuver);
            if (result.success) {
                total_intrusion += result.max_intrusion_depth;
                ok_count++;
                max_intr = std::max(max_intr, result.max_intrusion_depth);
            }
            if (!result.optimizer_converged) {
                nc_count++;
            }
        }
        successful_sum += ok_count;
        non_converged_sum += nc_count;
        max_intrusion_depth = std::max(max_intrusion_depth, max_intr);
        benchmark::DoNotOptimize(total_intrusion);
        benchmark::DoNotOptimize(ok_count);
    }

    const double iterations =
        static_cast<double>(std::max<int64_t>(1, state.iterations()));
    state.counters["maneuvers"] = static_cast<double>(ctx.total_maneuvers);
    state.counters["successful"] =
        static_cast<double>(successful_sum) / iterations;
    state.counters["non_converged"] =
        static_cast<double>(non_converged_sum) / iterations;
    state.counters["max_intrusion_depth"] = max_intrusion_depth;
}
BENCHMARK(BM_Stage1_BSplineSmoother)->Apply(ApplyStandardArgs);

// ============================================================
// Benchmark 2: 纵向速度规划 (Milestone 006)
// 测量 SpeedProfilePlanner::plan() 所有机动段的总耗时。
// 输入由预计算的 BSplineSmoother 结果构建，不计入计时。
// ============================================================
static void BM_Stage2_SpeedProfilePlanner(benchmark::State& state) {
    const auto& ctx = GetFullContext();
    if (ctx.speed_inputs.empty()) {
        state.SkipWithError("No valid speed inputs");
        return;
    }

    SpeedProfilePlanner planner(SpeedProfilePlannerConfig{});

    for (auto _ : state) {
        int ok_count = 0;
        for (std::size_t i = 0; i < ctx.speed_inputs.size(); ++i) {
            auto result = planner.plan(ctx.speed_inputs[i], ctx.vehicle_params,
                                       ctx.direction_signs_vec[i]);
            if (result.success) {
                ok_count++;
                benchmark::DoNotOptimize(result.v.front());
            }
        }
        benchmark::DoNotOptimize(ok_count);
    }

    state.counters["segments"] = static_cast<double>(ctx.speed_inputs.size());
    state.counters["total_pts"] = static_cast<double>([&ctx]() {
        std::size_t n = 0;
        for (const auto& inp : ctx.speed_inputs) {
            n += inp.s.size();
        }
        return n;
    }());
}
BENCHMARK(BM_Stage2_SpeedProfilePlanner)->Apply(ApplyStandardArgs);

// ============================================================
// Benchmark 3: 微分平坦状态补全 (Milestone 007)
// 测量 DifferentialFlatnessSolver::solve() 所有机动段的总耗时。
// ============================================================
static void BM_Stage3_DifferentialFlatness(benchmark::State& state) {
    const auto& ctx = GetFullContext();
    if (ctx.diff_inputs.empty()) {
        state.SkipWithError("No valid diff inputs");
        return;
    }

    DifferentialFlatnessSolver solver(DifferentialFlatnessSolverConfig{});

    for (auto _ : state) {
        int ok_count = 0;
        for (const auto& input : ctx.diff_inputs) {
            auto result = solver.solve(input, ctx.vehicle_params);
            if (result.success) {
                ok_count++;
                benchmark::DoNotOptimize(result.points.front().getDelta());
            }
        }
        benchmark::DoNotOptimize(ok_count);
    }

    state.counters["segments"] = static_cast<double>(ctx.diff_inputs.size());
    state.counters["total_pts"] = static_cast<double>([&ctx]() {
        std::size_t n = 0;
        for (const auto& inp : ctx.diff_inputs) {
            n += inp.x.size();
        }
        return n;
    }());
}
BENCHMARK(BM_Stage3_DifferentialFlatness)->Apply(ApplyStandardArgs);

// ============================================================
// Benchmark 4: 自适应重采样与维数固化 (Milestone 008)
// 测量 AdaptiveResampler::resample() 总耗时。
// ============================================================
static void BM_Stage4_AdaptiveResampler(benchmark::State& state) {
    const auto& ctx = GetFullContext();
    if (ctx.resampler_segments.empty()) {
        state.SkipWithError("No valid resampler segments");
        return;
    }

    AdaptiveResampler resampler(AdaptiveResamplerConfig{});

    for (auto _ : state) {
        auto result =
            resampler.resample(ctx.resampler_segments, ctx.vehicle_params);
        benchmark::DoNotOptimize(result.final_dimension);
    }

    state.counters["input_segs"] =
        static_cast<double>(ctx.resampler_segments.size());
    // N_final 在首次预计算中已得到，此处通过常量计数展示
    if (!ctx.z_ref.empty()) {
        state.counters["N_final"] = static_cast<double>(ctx.z_ref.size());
    }
}
BENCHMARK(BM_Stage4_AdaptiveResampler)->Apply(ApplyStandardArgs);

// ============================================================
// Benchmark 5: 静态安全走廊构建 (Milestone 009)
// 测量 StaticCorridorBuilder::build() 总耗时。
// ============================================================
static void BM_Stage5_StaticCorridor(benchmark::State& state) {
    const auto& ctx = GetFullContext();
    if (ctx.z_ref.empty()) {
        state.SkipWithError("No valid Z_ref for corridor building");
        return;
    }

    StaticCorridorBuilder builder(StaticCorridorBuilderConfig{});

    for (auto _ : state) {
        auto result =
            builder.build(ctx.z_ref, *ctx.esdf_map, *ctx.footprint_model);
        benchmark::DoNotOptimize(result.c_matrix.data());
    }

    state.counters["N_input"] = static_cast<double>(ctx.z_ref.size());
}
BENCHMARK(BM_Stage5_StaticCorridor)->Apply(ApplyStandardArgs);

// ============================================================
// Benchmark 6: 预处理管线端到端 (Milestone 010)
// 测量 PreprocessingPipeline::run() 总耗时，并记录各阶段耗时分解。
// ============================================================
static void BM_FullPipeline(benchmark::State& state) {
    const auto& ctx = GetFullContext();

    PreprocessingPipelineConfig config;
    config.use_static_corridor = true;

    const PreprocessingPipeline pipeline(config, ctx.vehicle_params,
                                         *ctx.footprint_model, *ctx.esdf_map);

    // 各阶段累计耗时（用于在所有迭代完成后上报 counters）
    double sum_total = 0.0;
    double sum_bspline = 0.0;
    double sum_speed = 0.0;
    double sum_diff_flat = 0.0;
    double sum_resample = 0.0;
    double sum_corridor = 0.0;
    int iter_count = 0;

    for (auto _ : state) {
        auto result = pipeline.run(ctx.path);
        if (result.success) {
            sum_total += result.time_total_ms;
            sum_bspline += result.time_bspline_ms;
            sum_speed += result.time_speed_ms;
            sum_diff_flat += result.time_diff_flat_ms;
            sum_resample += result.time_resample_ms;
            sum_corridor += result.time_corridor_ms;
            iter_count++;
        }
        benchmark::DoNotOptimize(result.final_dimension);
    }

    // 上报各阶段的平均耗时 (ms) 到 counters
    if (iter_count > 0) {
        state.counters["total_ms"] = sum_total / iter_count;
        state.counters["bspline_ms"] = sum_bspline / iter_count;
        state.counters["speed_ms"] = sum_speed / iter_count;
        state.counters["diff_flat_ms"] = sum_diff_flat / iter_count;
        state.counters["resample_ms"] = sum_resample / iter_count;
        state.counters["corridor_ms"] = sum_corridor / iter_count;
    }
    state.counters["maneuvers"] = static_cast<double>(ctx.total_maneuvers);
    state.counters["path_len_m"] = ctx.total_path_length_m;
}
BENCHMARK(BM_FullPipeline)->Apply(ApplyStandardArgs);

}  // namespace
}  // namespace apa_post_processor
