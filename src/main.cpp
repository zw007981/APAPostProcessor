#include "scene/nmpc_planning_scene.h"
#include "util/logger.h"
#include "util/visualizer.hpp"

using namespace apa_post_processor;

int main() {
    Logger::SetLogDirectory("../log");
    try {
        auto scene = NMPCPlanningScene::LoadFromFile("data/config.json");
        scene->optimize();
        const auto& result = scene->lastResult();
        const auto& preprocessed = scene->preprocessedTraj();
        const auto& nmpc = scene->nmpcTraj();
        auto visualizer = Visualizer("PostProcessor", -1.0, 2.33);
        if (!preprocessed.empty()) {
            visualizer.plotTrajectory(
                preprocessed, scene->vehicleParams(), &scene->footprintModel(),
                &scene->esdfMap(), &scene->gridMap(),
                /*draw_swept_area=*/false,
                {{"color", visualizer::Pen::RED}, {"label", "Preprocessed"}});
        }
        if (!nmpc.empty()) {
            visualizer.plotTrajectory(nmpc, scene->vehicleParams(),
                                      &scene->footprintModel(),
                                      &scene->esdfMap(), &scene->gridMap(),
                                      /*draw_swept_area=*/true,
                                      {{"color", visualizer::Pen::BLUE},
                                       {"label", "NMPC Optimized"}});
        }
        if (!preprocessed.empty() || !nmpc.empty()) {
            visualizer.save("../fig");
        }
        (void)result;
    } catch (const std::exception& e) {
        LOG_FMT_ERROR("Exception caught in main: {}!!!", e.what());
        return 1;
    }
    return 0;
}