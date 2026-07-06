#include "esdf_problem_updater.h"

#include <cmath>
#include <stdexcept>
#include <string>

#include "constraints/esdf_distance_constraint.h"
#include "util/constants.h"

namespace stc_SQP {
void EsdfProblemUpdater::updateOcp(const Trajectory& current_traj, const EsdfMapInterface& map,
    MultiStageOCP& ocp)
{
    // 使用契约与 ProblemUpdater 一致：调用前若 stage_params 已预分配为 N 个空 p，
    // 归一化为空，使后续 validate() 通过；非空则要求每步 p 均为 STAGE_PARAM_DIM 维。
    for (auto& segment : ocp.segments()) {
        if (static_cast<int>(segment.stage_params.size()) == segment.N) {
            bool all_empty = true;
            for (const auto& sp : segment.stage_params) {
                if (sp.p.size() != 0) {
                    all_empty = false;
                    break;
                }
            }
            if (all_empty) {
                segment.stage_params.clear();
            }
        }
    }
    std::string reason;
    if (!ocp.validate(&reason)) {
        throw std::invalid_argument("EsdfProblemUpdater: OCP configuration invalid - " + reason);
    }
    const int total_steps = ocp.totalSteps();
    if (static_cast<int>(current_traj.x.size()) < total_steps + 1) {
        throw std::invalid_argument(
            "EsdfProblemUpdater: current_traj.x size must be at least totalSteps + 1");
    }
    for (int k = 0; k <= total_steps; ++k) {
        if (current_traj.x[k].size() < EsdfDistanceConstraint::kStateXYThetaDim) {
            throw std::invalid_argument(
                "EsdfProblemUpdater: current_traj.x[" + std::to_string(k) + "] dimension must be >= "
                + std::to_string(EsdfDistanceConstraint::kStateXYThetaDim));
        }
        if (!current_traj.x[k].head(EsdfDistanceConstraint::kStateXYThetaDim).allFinite()) {
            throw std::invalid_argument(
                "EsdfProblemUpdater: current_traj.x[" + std::to_string(k) + "] first three dimensions contain non-finite values");
        }
    }

    const auto corners_local = EsdfDistanceConstraint::cornerLocalPositions();
    int global_k = 0;
    for (auto& segment : ocp.segments()) {
        segment.stage_params.resize(segment.N);
        for (int i = 0; i < segment.N; ++i) {
            const Vector& pose = current_traj.x[global_k];
            const double theta = pose(2), c = std::cos(theta), s = std::sin(theta);
            const Eigen::Vector2d center(pose(0), pose(1));

            Vector& p = segment.stage_params[i].p;
            if (p.size() == 0) {
                p = Vector::Zero(STAGE_PARAM_DIM);
            } else if (p.size() != STAGE_PARAM_DIM) {
                throw std::invalid_argument(
                    "EsdfProblemUpdater: stage_params[" + std::to_string(i)
                    + "].p dimension invalid, expected 0 or " + std::to_string(STAGE_PARAM_DIM));
            }
            for (int corner = 0; corner < EsdfDistanceConstraint::kNumCorners; ++corner) {
                const Eigen::Vector2d& local = corners_local[corner];
                const Eigen::Vector2d corner_world(
                    center(0) + c * local(0) - s * local(1), center(1) + s * local(0) + c * local(1));
                const EsdfSample sample = map.queryDistance(corner_world);
                EsdfDistanceConstraint::packCornerSample(
                    corner, sample.distance, sample.gradient, corner_world, p);
            }
            ++global_k;
        }
    }
}
} // namespace stc_SQP
