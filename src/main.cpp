#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "core/NMPC/vehicle_circle_geometry.h"
#include "core/collision_check.h"
#include "scene/planning_scene.h"
#include "util/logger.h"
#include "util/trajectory.h"
#include "util/visualizer.hpp"

using namespace apa_post_processor;

// 计算优化轨迹沿线的最小离障碍距离 (m)：全部外圆圆心经旋转落到世界坐标
// 后取 esdf 距离减去外圆半径的最小值。碰撞质量门只保证"无侵入"（max 侵
// 入 ≤ 0.02 m），此量度量安全距离余量，用于量化"贴障碍"程度
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

// 计算优化轨迹沿线的曲率统计 (1/m)：geom 读 kappa 字段（finalizeKappa
// 加宽窗统一填充，全仓库唯一计算点），kin 读 kappa_kinematic（回退
// tanδ/L）；xchk 交叉核对只统计驱动段（|v|≥0.05 且非方向段首末点，
// 过滤依据见 docs/interfaces.md 2026-09-21 变更记录）
void LogKappaStats(const Trajectory& trajectory, double wheelbase,
                   double max_kappa) {
    // 驱动段速度阈值 (m/s)：与 iLQR post_v_dwell 豁免阈值、
    // SplitDirectionSpans 默认 v_epsilon 同值
    static constexpr double kXDwellSpeed = 0.05;
    std::vector<double> geom, kin;
    const WindowKappaConfig kappa_config;
    const auto spans = Trajectory::SplitDirectionSpans(trajectory.points(), kappa_config);
    std::vector<bool> span_edge(trajectory.size(), false);
    for (const auto& span : spans) {
        if (span.begin < span.end) {
            span_edge[span.begin] = true;
            span_edge[span.end - 1] = true;
        }
    }
    double max_abs_diff = 0.0;
    std::size_t diff_points = 0;
    std::size_t argmax_index = 0;
    double argmax_v = 0.0, argmax_geom = 0.0, argmax_kin = 0.0;
    for (std::size_t i = 0; i < trajectory.size(); ++i) {
        const auto& pt = trajectory[i];
        double g = std::numeric_limits<double>::quiet_NaN();
        double k = std::numeric_limits<double>::quiet_NaN();
        if (pt.hasKappa()) {
            g = pt.getKappa();
            geom.push_back(std::abs(g));
        }
        // 优先读交付轨迹携带的 kappa_kinematic 字段；未设置时（如中间
        // 轨迹）回退为 tanδ/L 现场换算
        if (pt.hasKappaKinematic()) {
            k = pt.getKappaKinematic();
        } else if (pt.hasDelta()) {
            k = std::tan(pt.getDelta()) / wheelbase;
        }
        if (std::isfinite(k)) {
            kin.push_back(std::abs(k));
        }
        // xchk 只统计驱动段内部点（过滤规则见函数注释）
        if (!std::isfinite(g) || !std::isfinite(k) || span_edge[i] ||
            !pt.hasV() || std::abs(pt.getV()) < kXDwellSpeed) {
            continue;
        }
        const double diff = std::abs(std::abs(g) - std::abs(k));
        if (diff >= max_abs_diff) {
            max_abs_diff = diff;
            argmax_index = i;
            argmax_v = pt.getV();
            argmax_geom = g;
            argmax_kin = k;
        }
        ++diff_points;
    }
    const auto summarize = [&](const char* name, std::vector<double> v) {
        if (v.empty()) {
            LOG_FMT_INFO("kappa[{}]: 无数据", name);
            return;
        }
        std::sort(v.begin(), v.end());
        int over = 0;
        for (double x : v) {
            if (x > max_kappa) {
                ++over;
            }
        }
        LOG_FMT_INFO(
            "kappa[{}]: max={:.4f} (×{:.2f}) p99={:.4f} 超限 {}/{} ({:.1f}%) "
            "(κ_max={:.3f})",
            name, v.back(), v.back() / max_kappa,
            v[static_cast<std::size_t>(0.99 * (v.size() - 1))], over,
            static_cast<int>(v.size()), 100.0 * over / v.size(), max_kappa);
    };
    summarize("geom", geom);
    summarize("kin ", kin);
    if (diff_points > 0) {
        LOG_FMT_INFO(
            "kappa[xchk]: max||geom|-|kin||={:.4f} (×{:.2f} κ_max, 对拍 {} "
            "点) argmax=[i={} v={:.3f} geom={:.4f} kin={:.4f}]",
            max_abs_diff, max_abs_diff / max_kappa, diff_points, argmax_index,
            argmax_v, argmax_geom, argmax_kin);
    } else {
        LOG_FMT_INFO("kappa[xchk]: 驱动段过滤后无有效对拍点");
    }
}

int main() {
    Logger::SetLogDirectory(std::string(PROJECT_ROOT_DIR) + "/log");
    try {
        // 按场景配置中的算法配置详情文件（"algorithm" 字段）路由到对应算法
        // 对比同一数据在不同算法下的效果时，只需改config.json的config_details_path无需改动本文件
        auto scene = PlanningScene::LoadFromFile(std::string(PROJECT_ROOT_DIR) +
                                                 "/data/config.json");
        if (scene == nullptr) {
            LOG_FMT_ERROR(
                "Failed to create planning scene from "
                "data/config.json!!!");
            return 1;
        }
        const auto result = scene->optimize();
        // 打印优化摘要：优化前后路径长度与机动段数变化、优化耗时
        scene->printOptimizeSummary();
        // 初始前端路径补全为全量参考轨迹（几何量经微分平坦关系与最快走完
        // 前提的梯形加减速时间参数化），供离障碍距离对比与绘图复用
        const Trajectory init_traj(scene->initPath(), scene->vehicleParams());
        // 成功时打印初始路径与优化路径的最小离障碍距离，量化"前端是否嵌入
        // 障碍、优化是否将其推出"（调参与回归对照用）
        const auto& optimized = scene->optimizedTraj();
        if (result.success && !optimized.empty()) {
            LOG_FMT_INFO(
                "init clearance: {:.3f} m -> opt clearance: {:.3f} m",
                ComputeMinClearance(init_traj, scene->esdfMap(),
                                    scene->footprintModel()),
                ComputeMinClearance(optimized, scene->esdfMap(),
                                    scene->footprintModel()));
            LogKappaStats(optimized, scene->vehicleParams().wheelbase,
                          scene->vehicleParams().max_kappa);
        }
        // 对比图绘制"原始路径 vs 优化轨迹"：红为优化前；优化失败时只画
        // 初始轨迹
        auto visualizer = Visualizer("PostProcessor", -1.0, 2.33);
        if (!init_traj.empty()) {
            visualizer.plotTrajectory(
                init_traj, scene->vehicleParams(), &scene->footprintModel(),
                &scene->esdfMap(), &scene->gridMap(),
                /*draw_swept_area=*/false,
                /*draw_start_end=*/true,
                {{"color", visualizer::Pen::RED}, {"label", "Original Path"}});
        }
        if (result.success && !optimized.empty()) {
            visualizer.plotTrajectory(
                optimized, scene->vehicleParams(), &scene->footprintModel(),
                &scene->esdfMap(), &scene->gridMap(),
                /*draw_swept_area=*/true,
                /*draw_start_end=*/false,
                {{"color", visualizer::Pen::GREEN},
                 {"label", scene->algorithmName() + " Optimized"}});
        }
        if (!init_traj.empty() || !optimized.empty()) {
            visualizer.save(std::string(PROJECT_ROOT_DIR) + "/fig");
        }
    } catch (const std::exception& e) {
        LOG_FMT_ERROR("Exception caught in main: {}!!!", e.what());
        return 1;
    }
    return 0;
}
