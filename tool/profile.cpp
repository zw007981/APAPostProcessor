// 通用算法 profiling 驱动（perf 火焰图配套工具）
// 与生产入口 src/main.cpp 走完全相同的场景链路
// （PlanningScene::LoadFromFile -> optimize），在单进程内对四个数据集各
// 重复跑若干次，供 perf record 采样定位求解耗时瓶颈；全程不做可视化，
// 避免 OpenCV 绘图/写图时间污染求解采样。
// 除耗时外同时输出质量指标（生产口径摘要行、最小离障碍距离、几何/
// 运动学曲率峰值），供优化改动前后做"耗时 vs 质量"回归对照。
// 算法由命令行传入的配置详情 JSON 的 "algorithm" 字段（"minco_theta_s"/"nmpc"/
// "ilqr"）路由到对应算法场景，新增算法只需准备对应配置文件，无需新增驱动。
// 运行：./build/Profile/apa_profile <算法配置详情路径> [每数据集重复次数]
// 示例：./apa_profile data/minco_theta_s_config.json 5
// 配套脚本 tool/profile.py 一键完成配置/构建/采样/火焰图生成。
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include "core/NMPC/vehicle_circle_geometry.h"
#include "scene/planning_scene.h"
#include "spatial/esdf_map.h"
#include "util/logger.h"
#include "util/trajectory.h"
#include "vehicle/vehicle_footprint_model.h"

using namespace apa_post_processor;

namespace {
// 数据集条目：名称仅用于日志与汇总表标识
struct DatasetEntry {
    // 数据集短名（如 "data1"），同时用于命名临时场景配置文件
    std::string name;
    // 数据集 proto JSON 文件路径（相对项目根目录）
    std::string file;
};

// 单次求解的质量与耗时指标快照（供跨轮次汇总与改动前后回归对照）
struct RunMetrics {
    // 求解是否成功；失败时质量字段无意义
    bool success{false};
    // 端到端耗时 (ms)
    double time_ms{0.0};
    // 生产口径摘要行（与生产入口同源，含优化前后长度/段数/耗时）
    std::string summary;
    // 最终轨迹最小离障碍距离 (m)
    double clearance{0.0};
    // 最终轨迹几何曲率绝对值峰值 (1/m)
    double max_kappa_geom{0.0};
    // 最终轨迹运动学曲率（tanδ/L 口径）绝对值峰值 (1/m)
    double max_kappa_kin{0.0};
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

// 计算轨迹沿线最小离障碍距离 (m)：全部外圆圆心经旋转落到世界坐标后取
// ESDF 距离减外圆半径的最小值（与生产入口 main.cpp 同口径）
double ComputeMinClearance(const Trajectory& trajectory,
                           const ESDFMap& esdf_map,
                           const VehicleFootprintModel& footprint_model) {
    const auto local_centers =
        vehicle_circle_geometry::ExtractLocalCircleCenters(footprint_model,
                                                           CircleType::OUTER);
    const double outer_radius = footprint_model.getOuterRadius();
    double min_clearance = std::numeric_limits<double>::infinity();
    for (const auto& pt : trajectory.points()) {
        const double cos_t = std::cos(pt.theta);
        const double sin_t = std::sin(pt.theta);
        for (const auto& local : local_centers) {
            const double cx = pt.x + cos_t * local.x() - sin_t * local.y();
            const double cy = pt.y + sin_t * local.x() + cos_t * local.y();
            min_clearance =
                std::min(min_clearance, esdf_map.getDist(cx, cy) - outer_radius);
        }
    }
    return min_clearance;
}

// 统计轨迹的几何/运动学曲率绝对值峰值 (1/m)：几何口径读 kappa 字段，
// 运动学口径优先读 kappa_kinematic、缺失时回退 tan(δ)/L（与生产入口同源）
void AccumulateMaxKappa(const Trajectory& trajectory, double wheelbase,
                        double* max_geom, double* max_kin) {
    for (const auto& pt : trajectory.points()) {
        if (pt.hasKappa()) {
            *max_geom = std::max(*max_geom, std::abs(pt.getKappa()));
        }
        double kin = std::numeric_limits<double>::quiet_NaN();
        if (pt.hasKappaKinematic()) {
            kin = pt.getKappaKinematic();
        } else if (pt.hasDelta()) {
            kin = std::tan(pt.getDelta()) / wheelbase;
        }
        if (std::isfinite(kin)) {
            *max_kin = std::max(*max_kin, std::abs(kin));
        }
    }
}

// 按生产入口同款链路跑一个数据集的一次优化：
// 读场景配置 -> 构建场景 -> optimize -> 采集耗时与质量指标
// 场景每轮重建是故意的：预处理（ESDF 构建等）也在待优化范围内，
// 与生产环境单次运行的耗时构成保持一致
RunMetrics RunOnce(const DatasetEntry& dataset,
                   const std::string& config_details_path) {
    RunMetrics metrics;
    auto scene = PlanningScene::LoadFromFile(
        WriteTempSceneConfig(dataset, config_details_path));
    if (scene == nullptr) {
        LOG_FMT_ERROR("Failed to create scene for {}!!!", dataset.file);
        return metrics;
    }
    const auto result = scene->optimize();
    scene->printOptimizeSummary();
    metrics.success = result.success;
    metrics.time_ms = result.total_time_ms;
    metrics.summary = scene->optimizeSummary();
    const auto& optimized = scene->optimizedTraj();
    if (result.success && !optimized.empty()) {
        metrics.clearance = ComputeMinClearance(optimized, scene->esdfMap(),
                                                scene->footprintModel());
        AccumulateMaxKappa(optimized, scene->vehicleParams().wheelbase,
                           &metrics.max_kappa_geom, &metrics.max_kappa_kin);
    }
    return metrics;
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
    // 按数据集聚合耗时与质量指标，供 perf 采样之外直接对照各数据集
    std::vector<double> time_sums(datasets.size(), 0.0);
    std::vector<double> time_mins(datasets.size(),
                                  std::numeric_limits<double>::infinity());
    std::vector<int> fail_counts(datasets.size(), 0);
    std::vector<RunMetrics> last_metrics(datasets.size());
    for (int round = 0; round < repeats; ++round) {
        for (size_t i = 0; i < datasets.size(); ++i) {
            const RunMetrics metrics = RunOnce(datasets[i], config_details_path);
            if (!metrics.success) {
                ++fail_counts[static_cast<size_t>(i)];
                last_metrics[i] = metrics;
                continue;
            }
            time_sums[static_cast<size_t>(i)] += metrics.time_ms;
            time_mins[static_cast<size_t>(i)] =
                std::min(time_mins[static_cast<size_t>(i)], metrics.time_ms);
            last_metrics[i] = metrics;
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
                  << " ms, min=" << (ok_count > 0 ? time_mins[i] : 0.0)
                  << " ms, fail=" << fail_counts[i] << "/" << repeats
                  << "\n";
    }
    std::cout << "total=" << total_ms << " ms\n";
    // 质量指标取最后一轮（成功轮次）快照：与耗时同一次运行的产物，
    // 便于"耗时 vs 质量"成对对照；同一配置下各轮应当逐位重复
    std::cout << "\n===== 质量指标（最后一轮快照）=====\n";
    for (size_t i = 0; i < datasets.size(); ++i) {
        const RunMetrics& m = last_metrics[i];
        std::cout << datasets[i].name << ": " << m.summary << "\n";
        if (!m.success) {
            continue;
        }
        std::cout << std::fixed << std::setprecision(3)
                  << "    clearance=" << m.clearance
                  << " m, kappa[geom]max=" << m.max_kappa_geom
                  << ", kappa[kin]max=" << m.max_kappa_kin << "\n";
    }
    return 0;
}
