#include "circle_footprint_esdf_problem_updater.h"

#include <cmath>
#include <stdexcept>
#include <string>

#include "constraints/circle_footprint_esdf_constraint.h"
#include "util/constants.h"

namespace stc_SQP {
CircleFootprintEsdfProblemUpdater::CircleFootprintEsdfProblemUpdater(
    std::vector<Eigen::Vector2d> circle_local_positions)
    : circle_local_positions_(std::move(circle_local_positions))
{
    if (circle_local_positions_.empty()
        || static_cast<int>(circle_local_positions_.size())
            > CircleFootprintEsdfConstraint::kMaxCircles) {
        throw std::invalid_argument(
            "CircleFootprintEsdfProblemUpdater: circle_local_positions size must be in [1, "
            + std::to_string(CircleFootprintEsdfConstraint::kMaxCircles) + "]");
    }
    for (const auto& local : circle_local_positions_) {
        if (!local.allFinite()) {
            throw std::invalid_argument(
                "CircleFootprintEsdfProblemUpdater: circle_local_positions must all be finite");
        }
    }
}

void CircleFootprintEsdfProblemUpdater::updateOcp(const Trajectory& current_traj,
    const EsdfMapInterface& map, MultiStageOCP& ocp)
{
    // 使用契约与EsdfProblemUpdater一致：调用前若stage_params已预分配为N个空p，
    // 归一化为空，使后续validate()通过；非空则要求每步p均为STAGE_PARAM_DIM维。
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
        throw std::invalid_argument(
            "CircleFootprintEsdfProblemUpdater: OCP configuration invalid - " + reason);
    }
    const int total_steps = ocp.totalSteps();
    if (static_cast<int>(current_traj.x.size()) < total_steps + 1) {
        throw std::invalid_argument(
            "CircleFootprintEsdfProblemUpdater: current_traj.x size must be at least "
            "totalSteps + 1");
    }
    for (int k = 0; k <= total_steps; ++k) {
        if (current_traj.x[k].size() < CircleFootprintEsdfConstraint::kStateXYThetaDim) {
            throw std::invalid_argument(
                "CircleFootprintEsdfProblemUpdater: current_traj.x[" + std::to_string(k)
                + "] dimension must be >= "
                + std::to_string(CircleFootprintEsdfConstraint::kStateXYThetaDim));
        }
        if (!current_traj.x[k].head(CircleFootprintEsdfConstraint::kStateXYThetaDim).allFinite()) {
            throw std::invalid_argument(
                "CircleFootprintEsdfProblemUpdater: current_traj.x[" + std::to_string(k)
                + "] first three dimensions contain non-finite values");
        }
    }
    const int circle_num = static_cast<int>(circle_local_positions_.size());
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
                    "CircleFootprintEsdfProblemUpdater: stage_params[" + std::to_string(i)
                    + "].p dimension invalid, expected 0 or " + std::to_string(STAGE_PARAM_DIM));
            }
            for (int circle = 0; circle < circle_num; ++circle) {
                const Eigen::Vector2d& local = circle_local_positions_[circle];
                const Eigen::Vector2d circle_world(
                    center(0) + c * local(0) - s * local(1), center(1) + s * local(0) + c * local(1));
                const EsdfSample sample = map.queryDistance(circle_world);
                CircleFootprintEsdfConstraint::packCircleSample(
                    circle, sample.distance, sample.gradient, circle_world, p);
            }
            ++global_k;
        }
    }
}
} // namespace stc_SQP
