#include <benchmark/benchmark.h>

#include <memory>
#include <opencv2/imgproc.hpp>
#include <random>
#include <utility>
#include <vector>

#include "bench/bench_fixture.h"
#include "spatial/esdf_map.h"
#include "spatial/grid_map.h"
#include "util/position.h"

namespace apa_post_processor {
namespace {

constexpr double BENCHMARK_RESOLUTION = 0.1;

// 仅用于 benchmark 暴露受保护的距离场构建函数，避免把梯度构建时间混入对比。
class ESDFMapBenchmarkAccessor : public ESDFMap {
   public:
    using ESDFMap::BuildSignedDistData;
};

// 生成固定种子的随机地图数据与占据 cell 列表，供 query benchmark 使用。
// 逻辑与 bench_fixture.h 中的 BuildRandomMapData 一致，但允许显式指定种子。
void BuildRandomMapForQueryBenchmark(int width, int height, double resolution,
                                     double occupied_probability,
                                     std::uint32_t seed,
                                     std::vector<uint8_t>& occupancy_data,
                                     std::vector<Position>& occupied_cells) {
    const auto cell_count =
        static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
    occupancy_data.assign(cell_count, 0U);
    occupied_cells.clear();
    occupied_cells.reserve(cell_count);

    std::mt19937 rng(seed);
    std::bernoulli_distribution is_occupied(occupied_probability);
    bool has_occupied_cell = false;
    for (int row = 0; row < height; ++row) {
        for (int col = 0; col < width; ++col) {
            if (row == height - 1 && col == width - 1) {
                continue;
            }
            if (!is_occupied(rng)) {
                continue;
            }
            has_occupied_cell = true;
            const std::size_t index =
                static_cast<std::size_t>(row * width + col);
            occupancy_data[index] = 1U;
            occupied_cells.emplace_back(static_cast<double>(col) * resolution,
                                        static_cast<double>(row) * resolution);
        }
    }
    if (!has_occupied_cell) {
        occupancy_data[0] = 1U;
        occupied_cells.emplace_back(0.0, 0.0);
    }
}

void RegisterBenchmarkArgs(benchmark::internal::Benchmark* benchmark) {
    for (const auto& scenario : kBenchmarkScenarios) {
        benchmark->Args({scenario.width, scenario.height, scenario.prob_x1000});
    }
    benchmark->Repetitions(4)->ReportAggregatesOnly(true)->Unit(
        benchmark::kMillisecond);
}

// 测量自研 F&H 从占据数据构建 D_out/D_in/signed distance
// 的耗时，不包含梯度缓存。
void BuildESDFMapBenchmark(benchmark::State& state) {
    const auto width = static_cast<int>(state.range(0));
    const auto height = static_cast<int>(state.range(1));
    const auto prob_x1000 = static_cast<int>(state.range(2));
    const auto& map_data =
        BenchmarkFixture::Instance().GetMapData({width, height, prob_x1000});

    for (auto _ : state) {
        const auto signed_distance =
            ESDFMapBenchmarkAccessor::BuildSignedDistData(
                map_data.occupancy_data, width, height, BENCHMARK_RESOLUTION);
        auto center_distance = signed_distance[map_data.center_index];
        benchmark::DoNotOptimize(center_distance);
    }
}

// 测量 OpenCV 双 distanceTransform 构建 signed distance 的耗时，用于对照自研
// F&H 实现。
void BuildOpenCVSignedDistanceBenchmark(benchmark::State& state) {
    const auto width = static_cast<int>(state.range(0));
    const auto height = static_cast<int>(state.range(1));
    const auto prob_x1000 = static_cast<int>(state.range(2));
    const auto& map_data =
        BenchmarkFixture::Instance().GetMapData({width, height, prob_x1000});

    for (auto _ : state) {
        cv::Mat outside_distance;
        cv::Mat inside_distance;
        cv::Mat signed_distance;
        cv::distanceTransform(map_data.outside_mask, outside_distance,
                              cv::DIST_L2, cv::DIST_MASK_PRECISE, CV_32F);
        cv::distanceTransform(map_data.inside_mask, inside_distance,
                              cv::DIST_L2, cv::DIST_MASK_PRECISE, CV_32F);
        cv::subtract(outside_distance, inside_distance, signed_distance);
        signed_distance *= BENCHMARK_RESOLUTION;
        benchmark::DoNotOptimize(
            signed_distance.at<float>(height / 2, width / 2));
    }
}

BENCHMARK(BuildESDFMapBenchmark)->Apply(RegisterBenchmarkArgs);
BENCHMARK(BuildOpenCVSignedDistanceBenchmark)->Apply(RegisterBenchmarkArgs);

// 测量 ESDFMap 查询热路径性能：500x500 地图，100,000 个固定种子随机坐标查询。
class ESDFMapQueryBenchmark : public benchmark::Fixture {
   public:
    void SetUp(const ::benchmark::State& /*state*/) override {
        constexpr int kWidth = 500;
        constexpr int kHeight = 500;
        constexpr double kResolution = 0.1;
        constexpr double kOccupiedProbability = 0.1;
        constexpr std::uint32_t kMapSeed = 42U;
        constexpr std::uint32_t kQuerySeed = 12345U;
        constexpr int kQueryCount = 100000;

        std::vector<uint8_t> occupancy_data;
        std::vector<Position> occupied_cells;
        BuildRandomMapForQueryBenchmark(kWidth, kHeight, kResolution,
                                        kOccupiedProbability, kMapSeed,
                                        occupancy_data, occupied_cells);
        const GridMap grid_map(kResolution, kWidth, kHeight, Position{0.0, 0.0},
                               occupied_cells);
        esdf_map_ = std::make_unique<ESDFMap>(grid_map);

        queries_.reserve(kQueryCount);
        std::mt19937 rng(kQuerySeed);
        std::uniform_real_distribution<double> x_dist(
            0.0, static_cast<double>(kWidth) * kResolution);
        std::uniform_real_distribution<double> y_dist(
            0.0, static_cast<double>(kHeight) * kResolution);
        for (int i = 0; i < kQueryCount; ++i) {
            queries_.emplace_back(x_dist(rng), y_dist(rng));
        }
    }
    void TearDown(const ::benchmark::State& /*state*/) override {
        esdf_map_.reset();
        queries_.clear();
    }

   protected:
    std::unique_ptr<ESDFMap> esdf_map_;
    std::vector<std::pair<double, double>> queries_;
};

BENCHMARK_DEFINE_F(ESDFMapQueryBenchmark, GetDistAndGrad)
(benchmark::State& state) {
    for (auto _ : state) {
        for (const auto& query : queries_) {
            auto result = esdf_map_->getDistAndGrad(query.first, query.second);
            benchmark::DoNotOptimize(result);
        }
    }
    state.SetItemsProcessed(state.iterations() * queries_.size());
}

BENCHMARK_DEFINE_F(ESDFMapQueryBenchmark, GetDist)
(benchmark::State& state) {
    for (auto _ : state) {
        for (const auto& query : queries_) {
            auto dist = esdf_map_->getDist(query.first, query.second);
            benchmark::DoNotOptimize(dist);
        }
    }
    state.SetItemsProcessed(state.iterations() * queries_.size());
}

BENCHMARK_REGISTER_F(ESDFMapQueryBenchmark, GetDistAndGrad)
    ->Repetitions(4)
    ->ReportAggregatesOnly(true)
    ->Unit(benchmark::kNanosecond);
BENCHMARK_REGISTER_F(ESDFMapQueryBenchmark, GetDist)
    ->Repetitions(4)
    ->ReportAggregatesOnly(true)
    ->Unit(benchmark::kNanosecond);

}  // namespace
}  // namespace apa_post_processor
