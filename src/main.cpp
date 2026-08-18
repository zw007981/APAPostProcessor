#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "core/NMPC/vehicle_circle_geometry.h"
#include "core/collision_check.h"
#include "scene/planning_scene.h"
#include "util/logger.h"
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

// 计算优化轨迹沿线的曲率统计 (1/m)。输出两套指标：
//   - 几何曲率 κ_geom（Path::finalize 的角度差分 Δθ/Δs，与绘图 Curvature
//     面板一致）：换挡/方向翻转折点处会因 Δθ 大而畸高
//   - 运动学曲率 κ_kin = tan(δ)/L（车辆几何，受前轮转角限幅约束）
// 均与物理上限 max_kappa 对比，输出最大 |κ|、p99 分位与超限点数
void LogKappaStats(const Trajectory& trajectory, double wheelbase,
                   double max_kappa) {
    std::vector<double> geom, kin;
    for (const auto& pt : trajectory.points()) {
        if (pt.hasKappa()) {
            geom.push_back(std::abs(pt.getKappa()));
        }
        if (pt.hasDelta()) {
            kin.push_back(std::abs(std::tan(pt.getDelta()) / wheelbase));
        }
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
