// DDP 四数据集 profiling 驱动（perf 火焰图配套工具）
// 与生产入口 src/main.cpp 走完全相同的场景链路
// （PlanningScene::LoadFromFile -> optimize），但在单进程内对四个
// 数据集各重复跑若干次，供 perf record 采样定位 DDP 耗时瓶颈；
// 全程不做可视化，避免 OpenCV 绘图/写图时间污染求解采样。
// 运行：./build/Profile/apa_profile_ddp [每数据集重复次数，默认 3]
// 配套脚本 tool/profile_ddp.sh 一键完成构建/采样/火焰图生成。
#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "scene/planning_scene.h"
#include "util/logger.h"

using namespace apa_post_processor;

namespace {
// 数据集条目：名称仅用于日志与汇总表标识
struct DatasetEntry {
    // 数据集短名（如 "data1"），同时用于命名临时场景配置文件
    std::string name;
    // 数据集 proto JSON 文件路径（相对项目根目录）
    std::string file;
};

// 为指定数据集生成一份临时场景配置 JSON（与 data/config.json 同构、
// 仅替换 data_file_path），返回临时文件路径；
// 走真实链路的必要性：PlanningScene::LoadFromFile 只接受配置文件路径
std::string WriteTempSceneConfig(const DatasetEntry& dataset) {
    const std::string config_path =
        "build/Profile/profile_config_" + dataset.name + ".json";
    std::ofstream ofs(config_path);
    ofs << "{\n  \"data_file_path\": \"" << dataset.file
        << "\",\n  \"config_details_path\": \"data/ddp_config.json\"\n}\n";
    return config_path;
}

// 按生产入口同款链路跑一个数据集的一次优化：
// 读场景配置 -> 构建场景 -> optimize -> 记录耗时
// 场景每轮重建是故意的：预处理（ESDF 构建等）也在待优化范围内，
// 与生产环境单次运行的耗时构成保持一致
double RunOnce(const DatasetEntry& dataset) {
    auto scene = PlanningScene::LoadFromFile(WriteTempSceneConfig(dataset));
    if (scene == nullptr) {
        LOG_FMT_ERROR("Failed to create scene for {}!!!", dataset.file);
        return -1.0;
    }
    const auto result = scene->optimize();
    scene->printOptimizeSummary();
    return result.total_time_ms;
}

}  // namespace

int main(int argc, char** argv) {
    // 日志目录与生产入口一致；关闭控制台输出，避免刷屏干扰采样环境
    Logger::SetLogDirectory(std::string(PROJECT_ROOT_DIR) + "/log");
    Logger::SetConsoleOutputEnabled(false);
    // 每个数据集的重复次数：单次求解 0.9~2.4s，999Hz 采样下样本量
    // 仅约 1~2k，重复数次火焰图形状才稳定
    const int repeats = argc > 1 ? std::max(1, std::atoi(argv[1])) : 3;
    const std::vector<DatasetEntry> datasets = {
        {"data1", "data/rub_park/data1.json"},
        {"data3", "data/mid_park/data3.json"},
        {"data6", "data/long_park/data6.json"},
        {"data7", "data/rub_park/data7.json"},
    };
    // 按数据集聚合耗时，供 perf 采样之外直接对照各数据集求解时间
    std::vector<double> time_sums(datasets.size(), 0.0);
    std::vector<int> fail_counts(datasets.size(), 0);
    for (int round = 0; round < repeats; ++round) {
        for (size_t i = 0; i < datasets.size(); ++i) {
            const double time_ms = RunOnce(datasets[i]);
            if (time_ms < 0.0) {
                ++fail_counts[static_cast<size_t>(i)];
                continue;
            }
            time_sums[static_cast<size_t>(i)] += time_ms;
        }
    }
    std::cout << "\n===== DDP profile 驱动耗时汇总（每数据集 " << repeats
              << " 次）=====\n";
    double total_ms = 0.0;
    for (size_t i = 0; i < datasets.size(); ++i) {
        const int ok_count = repeats - fail_counts[i];
        const double mean_ms =
            ok_count > 0 ? time_sums[i] / ok_count : 0.0;
        total_ms += time_sums[i];
        std::cout << datasets[i].name << ": mean=" << mean_ms
                  << " ms, fail=" << fail_counts[i] << "/" << repeats
                  << "\n";
    }
    std::cout << "total=" << total_ms << " ms\n";
    return 0;
}
