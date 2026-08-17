// 通用算法 profiling 驱动（perf 火焰图配套工具）
// 与生产入口 src/main.cpp 走完全相同的场景链路
// （PlanningScene::LoadFromFile -> optimize），在单进程内对四个数据集各
// 重复跑若干次，供 perf record 采样定位求解耗时瓶颈；全程不做可视化，
// 避免 OpenCV 绘图/写图时间污染求解采样。
// 算法由命令行传入的配置详情 JSON 的 "algorithm" 字段（"alm"/"nmpc"/
// "ddp"）路由到对应算法场景，新增算法只需准备对应配置文件，无需新增驱动。
// 运行：./build/Profile/apa_profile <算法配置详情路径> [每数据集重复次数]
// 示例：./apa_profile data/alm_config.json 5
// 配套脚本 tool/profile.py 一键完成配置/构建/采样/火焰图生成。
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

// 为指定数据集生成一份临时场景配置 JSON（与 data/config.json 同构、仅替换
// data_file_path），返回临时文件路径；config_details_path 由命令行传入，
// 其中的 "algorithm" 字段决定路由到哪个算法场景
std::string WriteTempSceneConfig(const DatasetEntry& dataset,
                                 const std::string& config_details_path) {
    const std::string config_path =
        "build/Profile/profile_config_" + dataset.name + ".json";
    std::ofstream ofs(config_path);
    ofs << "{\n  \"data_file_path\": \"" << dataset.file
        << "\",\n  \"config_details_path\": \"" << config_details_path
        << "\"\n}\n";
    return config_path;
}

// 按生产入口同款链路跑一个数据集的一次优化：
// 读场景配置 -> 构建场景 -> optimize -> 记录耗时
// 场景每轮重建是故意的：预处理（ESDF 构建等）也在待优化范围内，
// 与生产环境单次运行的耗时构成保持一致
double RunOnce(const DatasetEntry& dataset,
               const std::string& config_details_path) {
    auto scene = PlanningScene::LoadFromFile(
        WriteTempSceneConfig(dataset, config_details_path));
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
    if (argc < 2) {
        std::cerr << "用法: apa_profile <算法配置详情路径> [每数据集重复次数]\n";
        return 1;
    }
    const std::string config_details_path = argv[1];
    // 每个数据集的重复次数：单次求解零点几秒到数秒不等，999Hz 采样下
    // 样本量偏少，重复数次火焰图形状才稳定
    const int repeats = argc > 2 ? std::max(1, std::atoi(argv[2])) : 3;
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
            const double time_ms = RunOnce(datasets[i], config_details_path);
            if (time_ms < 0.0) {
                ++fail_counts[static_cast<size_t>(i)];
                continue;
            }
            time_sums[static_cast<size_t>(i)] += time_ms;
        }
    }
    std::cout << "\n===== profile 驱动耗时汇总（每数据集 " << repeats
              << " 次，配置 " << config_details_path << "）=====\n";
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
