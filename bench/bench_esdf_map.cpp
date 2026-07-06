#include <benchmark/benchmark.h>
#include <opencv2/imgproc.hpp>

#include "bench/bench_fixture.h"
#include "spatial/esdf_map.h"

namespace apa_post_processor {
namespace {

constexpr double BENCHMARK_RESOLUTION = 0.1;

// 仅用于 benchmark 暴露受保护的距离场构建函数，避免把梯度构建时间混入对比。
class ESDFMapBenchmarkAccessor : public ESDFMap {
   public:
    using ESDFMap::BuildSignedDistData;
};

void RegisterBenchmarkArgs(benchmark::internal::Benchmark* benchmark) {
    for (const auto& scenario : kBenchmarkScenarios) {
        benchmark->Args(
            {scenario.width, scenario.height, scenario.prob_x1000});
    }
    benchmark->Repetitions(8)
        ->ReportAggregatesOnly(true)
        ->Unit(benchmark::kMillisecond);
}

// 测量自研 F&H 从占据数据构建 D_out/D_in/signed distance 的耗时，不包含梯度缓存。
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

// 测量 OpenCV 双 distanceTransform 构建 signed distance 的耗时，用于对照自研 F&H 实现。
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

}  // namespace
}  // namespace apa_post_processor
