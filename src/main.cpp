#include "scene/planning_scene.h"
#include "util/logger.h"
#include "util/visualizer.hpp"

using namespace apa_post_processor;

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
        // 对比图绘制"原始路径 vs 优化轨迹"：原始前端路径只携带 x/y/θ/κ
        // 几何量，经微分平坦关系与最快走完前提的梯形加减速时间参数化
        // 补全为全量参考轨迹（红，优化前）；优化失败时只画初始轨迹
        const Trajectory init_traj(scene->initPath(), scene->vehicleParams());
        const auto& optimized = scene->optimizedTraj();
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
