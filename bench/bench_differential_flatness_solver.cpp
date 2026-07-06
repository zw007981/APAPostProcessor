#include <benchmark/benchmark.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "preprocessing/bspline_smoother.h"
#include "preprocessing/differential_flatness_solver.h"
#include "preprocessing/speed_profile_planner.h"
#include "spatial/esdf_map.h"
#include "spatial/grid_map.h"
#include "util/data_loader.hpp"
#include "util/path.h"
#include "vehicle/vehicle_footprint_model.h"
#include "vehicle/vehicle_params.h"

namespace apa_post_processor {
namespace {

// benchmark 场景描述：取自真实数据中的机动段。
struct BenchmarkScenario {
    // 场景名称，仅用于输出标识
    std::string name;
    // 被测机动段（从真实数据拷贝）
    Maneuver maneuver;
    // 微分平坦补全输入，预先由 BSplineSmoother 与 SpeedProfilePlanner 计算好
    DifferentialFlatnessInput input;
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
// 从而由控制点计算密集配点处的几何导数。
class BSplineSmootherBenchmarkAccessor : public BSplineSmoother {
   public:
    using BSplineSmoother::BSplineSmoother;
    using BSplineSmoother::buildKnotVector;
    using BSplineSmoother::computeBasisAtU;
};

// 将机动段方向枚举映射为速度规划使用的 +/-1 符号。
int DirectionToSign(Direction direction) {
    switch (direction) {
        case Direction::BACKWARD:
            return -1;
        case Direction::FORWARD:
        case Direction::UNKNOWN:
        case Direction::PIVOT:
        default:
            return 1;
    }
}

// 对单个密集配点，使用车身全部子圆查询 ESDF，返回最小距离。
double ComputeMinEsdfDistAtPoint(
    const BSplineSmoother::DensePointData& dense_point,
    const ESDFMap& esdf_map) {
    double min_dist = std::numeric_limits<double>::infinity();
    for (const auto& circle : dense_point.circles) {
        const auto [dist, grad] =
            esdf_map.getDistAndGrad(circle.center.x(), circle.center.y());
        (void)grad;
        min_dist = std::min(min_dist, dist);
    }
    return std::isfinite(min_dist) ? min_dist : 0.0;
}

// 由 BSplineSmoother 结果构造速度规划输入：弧长、曲率、最小 ESDF 距离。
// 曲率由控制点与基函数导数解析计算，与 DifferentialFlatnessSolver 保持一致。
SpeedProfileInput BuildSpeedProfileInput(
    const BSplineSmoother::Result& smooth_result,
    const BSplineSmootherBenchmarkAccessor& smoother,
    const ESDFMap& esdf_map) {
    SpeedProfileInput input;
    const auto& dense_points = smooth_result.dense_points;
    const auto control_point_count =
        static_cast<int>(smooth_result.control_points.size());
    const auto knot_vector = smoother.buildKnotVector(control_point_count);
    input.s.reserve(dense_points.size());
    input.kappa.reserve(dense_points.size());
    input.min_esdf_dist.reserve(dense_points.size());

    for (const auto& dpd : dense_points) {
        input.s.push_back(dpd.s);
        input.min_esdf_dist.push_back(ComputeMinEsdfDistAtPoint(dpd, esdf_map));

        BSplineSmoother::BasisPack bp;
        smoother.computeBasisAtU(dpd.u, knot_vector, control_point_count, bp);
        double x_d1 = 0.0;
        double y_d1 = 0.0;
        double x_d2 = 0.0;
        double y_d2 = 0.0;
        for (std::size_t k = 0; k < bp.indices.size(); ++k) {
            const int idx = bp.indices[k];
            const auto& cp = smooth_result.control_points[idx];
            x_d1 += cp.x() * bp.d1[k];
            y_d1 += cp.y() * bp.d1[k];
            x_d2 += cp.x() * bp.d2[k];
            y_d2 += cp.y() * bp.d2[k];
        }
        const double speed_sq = x_d1 * x_d1 + y_d1 * y_d1;
        const double denom = std::max(speed_sq * std::sqrt(speed_sq), 1e-12);
        const double kappa = (x_d1 * y_d2 - y_d1 * x_d2) / denom;
        input.kappa.push_back(kappa);
    }
    return input;
}

// 由 BSplineSmoother 结果构造微分平坦补全输入：几何导数与速度规划结果。
DifferentialFlatnessInput BuildDifferentialFlatnessInput(
    const BSplineSmoother::Result& smooth_result,
    const BSplineSmootherBenchmarkAccessor& smoother,
    const SpeedProfileResult& speed_result) {
    DifferentialFlatnessInput input;
    const auto& dense_points = smooth_result.dense_points;
    const auto n_points = dense_points.size();
    const auto control_point_count =
        static_cast<int>(smooth_result.control_points.size());
    const auto knot_vector = smoother.buildKnotVector(control_point_count);

    input.x.reserve(n_points);
    input.y.reserve(n_points);
    input.theta.reserve(n_points);
    input.x_d1.reserve(n_points);
    input.x_d2.reserve(n_points);
    input.x_d3.reserve(n_points);
    input.y_d1.reserve(n_points);
    input.y_d2.reserve(n_points);
    input.y_d3.reserve(n_points);
    input.v.reserve(n_points);
    input.a.reserve(n_points);
    input.t.reserve(n_points);

    for (const auto& dpd : dense_points) {
        BSplineSmoother::BasisPack bp;
        smoother.computeBasisAtU(dpd.u, knot_vector, control_point_count, bp);

        double x = 0.0;
        double y = 0.0;
        double x_d1 = 0.0;
        double y_d1 = 0.0;
        double x_d2 = 0.0;
        double y_d2 = 0.0;
        double x_d3 = 0.0;
        double y_d3 = 0.0;
        for (std::size_t k = 0; k < bp.indices.size(); ++k) {
            const int idx = bp.indices[k];
            const auto& cp = smooth_result.control_points[idx];
            x += cp.x() * bp.values[k];
            y += cp.y() * bp.values[k];
            x_d1 += cp.x() * bp.d1[k];
            y_d1 += cp.y() * bp.d1[k];
            x_d2 += cp.x() * bp.d2[k];
            y_d2 += cp.y() * bp.d2[k];
            x_d3 += cp.x() * bp.d3[k];
            y_d3 += cp.y() * bp.d3[k];
        }

        input.x.push_back(x);
        input.y.push_back(y);
        input.theta.push_back(dpd.theta);
        input.x_d1.push_back(x_d1);
        input.x_d2.push_back(x_d2);
        input.x_d3.push_back(x_d3);
        input.y_d1.push_back(y_d1);
        input.y_d2.push_back(y_d2);
        input.y_d3.push_back(y_d3);
    }

    // 速度规划结果与密集配点一一对应。
    for (std::size_t i = 0; i < n_points; ++i) {
        input.v.push_back(speed_result.v[i]);
        input.a.push_back(speed_result.a[i]);
        input.t.push_back(speed_result.t[i]);
    }

    return input;
}

const BenchmarkContext& GetContext() {
    static BenchmarkContext context = []() {
        BenchmarkContext ctx;

        nlohmann::json config_json;
        constexpr const char* kConfigPath = "data/config.json";
        if (DataLoader::LoadJsonFile(kConfigPath, config_json) !=
            LoadResult::SUCCESS) {
            throw std::runtime_error(
                "bench_differential_flatness_solver: failed to load "
                "config.json");
        }
        const auto data_file_path =
            config_json["data_file_path"].get<std::string>();

        ::apa::post_processor::OptimizeRequest request;
        if (DataLoader::LoadProtoFromJsonFile(data_file_path, request) !=
            LoadResult::SUCCESS) {
            throw std::runtime_error(
                "bench_differential_flatness_solver: failed to load data file");
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
                "bench_differential_flatness_solver: no maneuvers in data "
                "file");
        }

        // 最长机动段
        const auto long_it =
            std::max_element(maneuvers.begin(), maneuvers.end(),
                             [](const Maneuver& a, const Maneuver& b) {
                                 return a.length() < b.length();
                             });

        // 最短非退化机动段
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
        const Maneuver& short_maneuver =
            (short_it == maneuvers.end() ||
             short_it->length() < kMinNonDegenerateLength)
                ? *long_it
                : *short_it;

        BSplineSmootherBenchmarkAccessor smoother(
            BSplineSmootherConfig{}, ctx.vehicle_params, *ctx.footprint_model,
            *ctx.esdf_map);
        SpeedProfilePlanner speed_planner(SpeedProfilePlannerConfig{});

        for (const auto& maneuver : {*long_it, short_maneuver}) {
            const auto smooth_result = smoother.smooth(maneuver);
            if (!smooth_result.success || smooth_result.dense_points.empty()) {
                continue;
            }
            const auto speed_input = BuildSpeedProfileInput(
                smooth_result, smoother, *ctx.esdf_map);
            const std::vector<int> direction_signs(
                smooth_result.dense_points.size(),
                DirectionToSign(maneuver.direction));
            const auto speed_result = speed_planner.plan(
                speed_input, ctx.vehicle_params, direction_signs);
            if (!speed_result.success) {
                continue;
            }
            const auto diff_input = BuildDifferentialFlatnessInput(
                smooth_result, smoother, speed_result);
            const std::string name =
                (maneuver.length() == long_it->length()) ? "long" : "short";
            ctx.scenarios.push_back({name, maneuver, diff_input});
        }

        if (ctx.scenarios.empty()) {
            throw std::runtime_error(
                "bench_differential_flatness_solver: failed to build any "
                "scenario");
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
        ->Unit(benchmark::kMicrosecond);
}

void DifferentialFlatnessSolverBenchmark(benchmark::State& state) {
    const auto& ctx = GetContext();
    const auto scenario_idx = static_cast<std::size_t>(state.range(0));
    const auto& scenario = ctx.scenarios.at(scenario_idx);

    const DifferentialFlatnessSolverConfig config;
    const DifferentialFlatnessSolver solver(config);

    for (auto _ : state) {
        auto result = solver.solve(scenario.input, ctx.vehicle_params);
        benchmark::DoNotOptimize(result.points.front().getDelta());
    }
    state.SetLabel(scenario.name + " / " +
                   std::to_string(scenario.input.x.size()) + " pts");
}

BENCHMARK(DifferentialFlatnessSolverBenchmark)
    ->ArgName("scenario")
    ->Apply(RegisterScenarioArgs);

}  // namespace
}  // namespace apa_post_processor
