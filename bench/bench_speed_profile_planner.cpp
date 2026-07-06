#include <benchmark/benchmark.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

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
    std::string name;
    Maneuver maneuver;
    // 预计算好的速度规划输入，避免在计时循环内做几何/ESDF 计算。
    SpeedProfileInput input;
    std::vector<int> direction_signs;
};

// benchmark 上下文：一次性加载 data/config.json 指向的真实数据，
// 构造 footprint 模型与 ESDF 地图，并选取最长/最短两个非退化机动段。
struct BenchmarkContext {
    VehicleParams vehicle_params;
    std::unique_ptr<VehicleFootprintModel> footprint_model;
    std::unique_ptr<ESDFMap> esdf_map;
    Path path;
    std::vector<BenchmarkScenario> scenarios;
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

// 对单个路径点，使用车身内外子圆查询 ESDF，返回最小距离。
double ComputeMinEsdfDistAtPoint(const VehicleFootprintModel& footprint_model,
                                 const ESDFMap& esdf_map,
                                 const PathPoint& point) {
    std::vector<Eigen::Vector2d> centers;
    std::vector<Eigen::Matrix<double, 2, 3>> jacobians;

    const auto appendCenters = [&](CircleType type) {
        const auto circle_num = footprint_model.getCircleNum(type);
        centers.resize(circle_num);
        jacobians.resize(circle_num);
        footprint_model.calInterpolatedCenters(point.x, point.y, point.theta,
                                               type, centers, jacobians);
    };
    appendCenters(CircleType::INNER);
    appendCenters(CircleType::OUTER);

    double min_dist = std::numeric_limits<double>::infinity();
    for (const auto& center : centers) {
        const auto [dist, grad] =
            esdf_map.getDistAndGrad(center.x(), center.y());
        (void)grad;
        min_dist = std::min(min_dist, dist);
    }
    return std::isfinite(min_dist) ? min_dist : 0.0;
}

// 由机动段构造 SpeedProfilePlanner 输入：弧长表、曲率、最小 ESDF 距离。
SpeedProfileInput BuildSpeedProfileInput(
    const Maneuver& maneuver, const VehicleFootprintModel& footprint_model,
    const ESDFMap& esdf_map) {
    SpeedProfileInput input;
    const auto& points = maneuver.points;
    if (points.empty()) {
        return input;
    }

    input.s.reserve(points.size());
    input.kappa.reserve(points.size());
    input.min_esdf_dist.reserve(points.size());

    double cumulative_s = 0.0;
    for (std::size_t i = 0; i < points.size(); ++i) {
        if (i > 0) {
            const double dx = points[i].x - points[i - 1].x;
            const double dy = points[i].y - points[i - 1].y;
            cumulative_s += std::hypot(dx, dy);
        }
        input.s.push_back(cumulative_s);
        input.kappa.push_back(points[i].hasKappa() ? points[i].getKappa()
                                                   : 0.0);
        input.min_esdf_dist.push_back(
            ComputeMinEsdfDistAtPoint(footprint_model, esdf_map, points[i]));
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
                "bench_speed_profile_planner: failed to load config.json");
        }
        const auto data_file_path =
            config_json["data_file_path"].get<std::string>();

        ::apa::post_processor::OptimizeRequest request;
        if (DataLoader::LoadProtoFromJsonFile(data_file_path, request) !=
            LoadResult::SUCCESS) {
            throw std::runtime_error(
                "bench_speed_profile_planner: failed to load data file");
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
                "bench_speed_profile_planner: no maneuvers in data file");
        }

        const auto long_it =
            std::max_element(maneuvers.begin(), maneuvers.end(),
                             [](const Maneuver& a, const Maneuver& b) {
                                 return a.length() < b.length();
                             });

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

        ctx.scenarios.push_back(
            {"long", *long_it,
             BuildSpeedProfileInput(*long_it, *ctx.footprint_model,
                                    *ctx.esdf_map),
             std::vector<int>(long_it->points.size(),
                              DirectionToSign(long_it->direction))});
        ctx.scenarios.push_back(
            {"short", short_maneuver,
             BuildSpeedProfileInput(short_maneuver, *ctx.footprint_model,
                                    *ctx.esdf_map),
             std::vector<int>(short_maneuver.points.size(),
                              DirectionToSign(short_maneuver.direction))});

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

void SpeedProfilePlannerBenchmark(benchmark::State& state) {
    const auto& ctx = GetContext();
    const auto scenario_idx = static_cast<std::size_t>(state.range(0));
    const auto& scenario = ctx.scenarios.at(scenario_idx);

    const SpeedProfilePlannerConfig config;
    const SpeedProfilePlanner planner(config);

    for (auto _ : state) {
        auto result = planner.plan(scenario.input, ctx.vehicle_params,
                                   scenario.direction_signs);
        benchmark::DoNotOptimize(result.v.front());
    }
    state.SetLabel(scenario.name + " / " +
                   std::to_string(scenario.input.s.size()) + " pts");
}

BENCHMARK(SpeedProfilePlannerBenchmark)
    ->ArgName("scenario")
    ->Apply(RegisterScenarioArgs);

}  // namespace
}  // namespace apa_post_processor
