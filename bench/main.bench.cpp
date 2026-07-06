#include <benchmark/benchmark.h>

#include <algorithm>
#include <sstream>
#include <string>
#include <vector>

#include "bench/bench_fixture.h"
#include "util/logger.h"

namespace apa_post_processor {
namespace {

std::string FormatProbabilityArg(int prob_x1000) {
    std::ostringstream oss;
    oss << (static_cast<double>(prob_x1000) / 1000.0);
    return oss.str();
}

std::vector<std::string> SplitString(const std::string& text, char delimiter) {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream stream(text);
    while (std::getline(stream, token, delimiter)) {
        tokens.push_back(token);
    }
    return tokens;
}

// Google Benchmark 默认会打印 mean/median/stddev/cv 并输出 items_per_second，
// 这里只保留 mean，并把 repeats:5_mean 显示为 x5_mean，概率参数显示为 0.2/0.05。
class MeanOnlyConsoleReporter : public benchmark::ConsoleReporter {
   public:
    using benchmark::ConsoleReporter::ConsoleReporter;

    void ReportRuns(
        const std::vector<benchmark::BenchmarkReporter::Run>& reports) override {
        std::vector<benchmark::BenchmarkReporter::Run> filtered_reports;
        filtered_reports.reserve(reports.size());
        for (auto report : reports) {
            if (report.run_type !=
                    benchmark::BenchmarkReporter::Run::RT_Aggregate ||
                report.aggregate_name == "mean") {
                FormatRunName(report);
                filtered_reports.push_back(std::move(report));
                name_field_width_ = std::max(
                    name_field_width_, report.benchmark_name().size());
            }
        }
        benchmark::ConsoleReporter::ReportRuns(filtered_reports);
    }

   private:
    void FormatRunName(benchmark::BenchmarkReporter::Run& report) {
        if (report.run_type ==
                benchmark::BenchmarkReporter::Run::RT_Aggregate &&
            report.repetitions > 0) {
            report.run_name.repetitions =
                "x" + std::to_string(report.repetitions);
        }

        std::vector<std::string> tokens =
            SplitString(report.run_name.args, '/');
        if (tokens.size() == 3U) {
            try {
                const int prob_x1000 = std::stoi(tokens[2]);
                tokens[2] = FormatProbabilityArg(prob_x1000);
                report.run_name.args =
                    tokens[0] + '/' + tokens[1] + '/' + tokens[2];
            } catch (...) {
                // 解析失败时保持原样。
            }
        }
    }
};

}  // namespace
}  // namespace apa_post_processor

int main(int argc, char** argv) {
    apa_post_processor::Logger::SetLogDirectory("../log");
    apa_post_processor::Logger::SetConsoleOutputEnabled(false);

    // 在正式 benchmark 开始前实例化 fixture，确保地图已经构建完成。
    apa_post_processor::BenchmarkFixture::Instance();

    benchmark::Initialize(&argc, argv);
    if (benchmark::ReportUnrecognizedArguments(argc, argv)) {
        return 1;
    }

    apa_post_processor::MeanOnlyConsoleReporter reporter;
    benchmark::RunSpecifiedBenchmarks(&reporter);
    benchmark::Shutdown();
    return 0;
}
