#include <benchmark/benchmark.h>

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "preprocessing/bspline_smoother.h"
#include "spatial/esdf_map.h"
#include "spatial/grid_map.h"
#include "util/data_loader.hpp"
#include "util/path.h"
#include "vehicle/vehicle_footprint_model.h"
#include "vehicle/vehicle_params.h"

namespace apa_post_processor {
namespace {

// benchmark 场景描述：用于区分真实数据中的最长/最短机动段。
struct BenchmarkScenario {
    // 场景名称，仅用于输出标识
    std::string name;
    // 被测机动段（从真实数据拷贝，避免依赖 Path 临时对象）
    Maneuver maneuver;
};

// benchmark 上下文：一次性加载 data/config.json 指向的真实数据，
// 构造 footprint 模型与 ESDF 地图，并选取最长/最短两个非退化机动段。
struct BenchmarkContext {
    // 车辆参数
    VehicleParams vehicle_params;
    // 车辆 footprint 模型
    std::unique_ptr<VehicleFootprintModel> footprint_model;
    // ESDF 地图
    std::unique_ptr<ESDFMap> esdf_map;
    // 原始路径（持有 maneuvers 的内存）
    Path path;
    // 选出的两个场景
    std::vector<BenchmarkScenario> scenarios;
};

// 通过派生把 BSplineSmoother 的保护方法暴露给 benchmark，
// 从而独立测量弧长表构建与碰撞校验两阶段耗时。
class BSplineSmootherBenchmarkAccessor : public BSplineSmoother {
   public:
    using BSplineSmoother::BSplineSmoother;
    using BSplineSmoother::buildArcLengthTable;
    using BSplineSmoother::buildDenseEvalPoints;
    using BSplineSmoother::buildKnotVector;
    using BSplineSmoother::evaluateFineGrid;
    using BSplineSmoother::precomputeBasisPacks;
    using BSplineSmoother::validateCollisionFree;
};

const BenchmarkContext& GetContext() {
    static BenchmarkContext context = []() {
        BenchmarkContext ctx;

        nlohmann::json config_json;
        constexpr const char* kConfigPath = "data/config.json";
        if (DataLoader::LoadJsonFile(kConfigPath, config_json) !=
            LoadResult::SUCCESS) {
            throw std::runtime_error(
                "bench_bspline_smoother: failed to load config.json");
        }
        const auto data_file_path =
            config_json["data_file_path"].get<std::string>();

        ::apa::post_processor::OptimizeRequest request;
        if (DataLoader::LoadProtoFromJsonFile(data_file_path, request) !=
            LoadResult::SUCCESS) {
            throw std::runtime_error(
                "bench_bspline_smoother: failed to load data file");
        }

        ctx.vehicle_params = VehicleParams::FromProto(request.vehicle());
        ctx.footprint_model =
            std::make_unique<VehicleFootprintModel>(ctx.vehicle_params);
        const auto grid_map = GridMap::FromProto(request.environment());
        ctx.esdf_map = std::make_unique<ESDFMap>(grid_map);
        ctx.path = Path::FromProto(request.initial_path());

        const auto& maneuvers = ctx.path.getManeuvers();
        if (maneuvers.empty()) {
            throw std::runtime_error(
                "bench_bspline_smoother: no maneuvers in data file");
        }

        // 最长机动段
        const auto long_it =
            std::max_element(maneuvers.begin(), maneuvers.end(),
                             [](const Maneuver& a, const Maneuver& b) {
                                 return a.length() < b.length();
                             });
        ctx.scenarios.push_back({"long", *long_it});

        // 最短非退化机动段：长度低于退化阈值时 L-BFGS 不会启动，
        // 因此选择长度不小于退化阈值的最短段，保证两段都能进入优化流程。
        constexpr double kMinNonDegenerateLength = 0.1;
        const auto short_it = std::min_element(
            maneuvers.begin(), maneuvers.end(),
            [kMinNonDegenerateLength](const Maneuver& a, const Maneuver& b) {
                const double a_len =
                    (a.length() >= kMinNonDegenerateLength) ? a.length() : 1e10;
                const double b_len =
                    (b.length() >= kMinNonDegenerateLength) ? b.length() : 1e10;
                return a_len < b_len;
            });
        if (short_it == maneuvers.end() ||
            short_it->length() < kMinNonDegenerateLength) {
            ctx.scenarios.push_back({"short", *long_it});
        } else {
            ctx.scenarios.push_back({"short", *short_it});
        }

        return ctx;
    }();
    return context;
}

void RegisterScenarioArgs(benchmark::internal::Benchmark* benchmark) {
    const auto& scenarios = GetContext().scenarios;
    for (std::size_t i = 0; i < scenarios.size(); ++i) {
        benchmark->Args({static_cast<long long>(i)});
    }
    benchmark->ArgNames({"scenario"})
        ->Repetitions(8)
        ->ReportAggregatesOnly(true)
        ->Unit(benchmark::kMillisecond);
}

// 对两个场景分别测量完整 L-BFGS 平滑流程的耗时。
void SmoothManeuverBenchmark(benchmark::State& state) {
    const auto& ctx = GetContext();
    const auto scenario_idx = static_cast<std::size_t>(state.range(0));
    const auto& scenario = ctx.scenarios.at(scenario_idx);
    BSplineSmootherBenchmarkAccessor smoother(
        BSplineSmootherConfig{}, ctx.vehicle_params, *ctx.footprint_model,
        *ctx.esdf_map);

    for (auto _ : state) {
        const auto result = smoother.smooth(scenario.maneuver);
        double max_intrusion = result.max_intrusion_depth;
        benchmark::DoNotOptimize(max_intrusion);
    }
    state.SetLabel(scenario.name);
}

// 测量弧长表构建耗时：先暂停计时完成优化与细网格求值，
// 再仅对 buildArcLengthTable 循环计时。
void BuildArcLengthTableBenchmark(benchmark::State& state) {
    const auto& ctx = GetContext();
    const auto scenario_idx = static_cast<std::size_t>(state.range(0));
    const auto& scenario = ctx.scenarios.at(scenario_idx);
    BSplineSmootherBenchmarkAccessor smoother(
        BSplineSmootherConfig{}, ctx.vehicle_params, *ctx.footprint_model,
        *ctx.esdf_map);

    // 使用优化后的控制点求取细网格位置，确保弧长表测量的是真实曲线。
    const auto smooth_result = smoother.smooth(scenario.maneuver);
    const auto control_point_count =
        static_cast<int>(smooth_result.control_points.size());
    const auto knot_vector = smoother.buildKnotVector(control_point_count);
    std::vector<BSplineSmoother::BasisPack> basis_packs;
    smoother.precomputeBasisPacks(knot_vector, control_point_count,
                                  basis_packs);
    std::vector<Eigen::Vector2d> positions;
    std::vector<Eigen::Vector2d> velocities;
    smoother.evaluateFineGrid(smooth_result.control_points, basis_packs,
                              positions, velocities);

    for (auto _ : state) {
        const auto table = smoother.buildArcLengthTable(positions);
        double total_length = table.back().second;
        benchmark::DoNotOptimize(total_length);
    }
    state.SetLabel(scenario.name);
}

// 测量碰撞校验耗时：同样先暂停计时完成优化，
// 再仅对 validateCollisionFree 循环计时。
void ValidateCollisionFreeBenchmark(benchmark::State& state) {
    const auto& ctx = GetContext();
    const auto scenario_idx = static_cast<std::size_t>(state.range(0));
    const auto& scenario = ctx.scenarios.at(scenario_idx);
    BSplineSmootherBenchmarkAccessor smoother(
        BSplineSmootherConfig{}, ctx.vehicle_params, *ctx.footprint_model,
        *ctx.esdf_map);

    const auto smooth_result = smoother.smooth(scenario.maneuver);
    const auto control_point_count =
        static_cast<int>(smooth_result.control_points.size());
    const auto knot_vector = smoother.buildKnotVector(control_point_count);
    const auto dense_eval_points = smoother.buildDenseEvalPoints(
        knot_vector, control_point_count, smooth_result.arc_length_table);

    for (auto _ : state) {
        std::vector<BSplineSmoother::DensePointData> dense_points;
        double max_intrusion = 0.0;
        bool ok = smoother.validateCollisionFree(smooth_result.control_points,
                                                 dense_eval_points,
                                                 dense_points, max_intrusion);
        benchmark::DoNotOptimize(ok);
        benchmark::DoNotOptimize(max_intrusion);
    }
    state.SetLabel(scenario.name);
}

BENCHMARK(SmoothManeuverBenchmark)->Apply(RegisterScenarioArgs);
BENCHMARK(BuildArcLengthTableBenchmark)->Apply(RegisterScenarioArgs);
BENCHMARK(ValidateCollisionFreeBenchmark)->Apply(RegisterScenarioArgs);

}  // namespace
}  // namespace apa_post_processor
