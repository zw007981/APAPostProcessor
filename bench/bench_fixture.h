#pragma once

#include <array>
#include <cstdint>
#include <map>
#include <opencv2/core.hpp>
#include <random>
#include <vector>

namespace apa_post_processor {

// 描述一组 ESDF benchmark 场景：地图宽高与占用概率（千分比）。
struct Scenario {
    int width = 0;
    int height = 0;
    // 占用概率，以千分比存储。例如 200 表示 0.2，50 表示 0.05。
    int prob_x1000 = 0;

    bool operator<(const Scenario& other) const noexcept {
        if (width != other.width) {
            return width < other.width;
        }
        if (height != other.height) {
            return height < other.height;
        }
        return prob_x1000 < other.prob_x1000;
    }
};

// 随机地图数据，供自研实现与 OpenCV 对照使用。
struct RandomMapData {
    std::vector<uint8_t> occupancy_data;
    cv::Mat outside_mask;
    cv::Mat inside_mask;
    std::size_t center_index = 0;
};

inline constexpr std::array<Scenario, 3> kBenchmarkScenarios = {{
    {200, 200, 100},   // 0.1
    {500, 500, 100},   // 0.1
    {1000, 1000, 100}  // 0.1
}};

// Benchmark fixture：在测试开始前构造所有场景所需的随机地图，
// 避免地图生成时间被计入被测函数。
class BenchmarkFixture {
   public:
    static const BenchmarkFixture& Instance();

    const RandomMapData& GetMapData(const Scenario& scenario) const {
        return maps_.at(scenario);
    }

    const std::vector<Scenario>& Scenarios() const { return scenarios_; }

   private:
    BenchmarkFixture();

    static RandomMapData BuildRandomMapData(int width, int height,
                                            double occupied_probability);

    std::vector<Scenario> scenarios_;
    std::map<Scenario, RandomMapData> maps_;
};

inline BenchmarkFixture::BenchmarkFixture() {
    scenarios_.assign(kBenchmarkScenarios.begin(), kBenchmarkScenarios.end());

    for (const auto& scenario : scenarios_) {
        maps_[scenario] = BuildRandomMapData(
            scenario.width, scenario.height,
            static_cast<double>(scenario.prob_x1000) / 1000.0);
    }
}

inline RandomMapData BenchmarkFixture::BuildRandomMapData(
    int width, int height, double occupied_probability) {
    const auto cell_count =
        static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
    std::vector<uint8_t> occupancy_data(cell_count, 0U);

    const int prob_x1000 =
        static_cast<int>(occupied_probability * 1000.0 + 0.5);
    const auto seed = static_cast<std::uint32_t>(
        static_cast<std::uint64_t>(width) * 73856093ULL ^
        static_cast<std::uint64_t>(height) * 19349663ULL ^
        static_cast<std::uint64_t>(prob_x1000) * 83492791ULL);
    std::mt19937 rng(seed);
    std::bernoulli_distribution is_occupied(occupied_probability);

    cv::Mat outside_mask(height, width, CV_8UC1, cv::Scalar(255));
    cv::Mat inside_mask(height, width, CV_8UC1, cv::Scalar(0));

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
            occupancy_data[static_cast<std::size_t>(row * width + col)] = 1U;
            outside_mask.at<uint8_t>(row, col) = 0U;
            inside_mask.at<uint8_t>(row, col) = 255U;
        }
    }

    if (!has_occupied_cell) {
        occupancy_data[0] = 1U;
        outside_mask.at<uint8_t>(0, 0) = 0U;
        inside_mask.at<uint8_t>(0, 0) = 255U;
    }

    const std::size_t center_index =
        static_cast<std::size_t>((height / 2) * width + width / 2);
    return RandomMapData{std::move(occupancy_data), outside_mask, inside_mask,
                         center_index};
}

inline const BenchmarkFixture& BenchmarkFixture::Instance() {
    static BenchmarkFixture instance;
    return instance;
}

}  // namespace apa_post_processor
