#include "problem_updater.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace stc_SQP {
ProblemUpdater::ProblemUpdater(const UpdaterConfig& config)
    : config_(config)
{
    if (config_.top_k < 1 || config_.top_k > kMaxHalfSpaces) {
        throw std::invalid_argument(
            "ProblemUpdater: top_k must be in [1, " + std::to_string(kMaxHalfSpaces) + "]");
    }
    if (!std::isfinite(config_.selection_radius) || config_.selection_radius <= 0.0) {
        throw std::invalid_argument("ProblemUpdater: selection_radius must be a finite positive number");
    }
    if (!std::isfinite(config_.max_step_displacement) || config_.max_step_displacement < 0.0) {
        throw std::invalid_argument("ProblemUpdater: max_step_displacement must be a non-negative finite number");
    }
    if (!std::isfinite(config_.safety_margin) || config_.safety_margin < 0.0) {
        throw std::invalid_argument("ProblemUpdater: safety_margin must be a non-negative finite number");
    }
    assertCompleteness();
}

void ProblemUpdater::updateOcp(const Trajectory& current_traj, const MapInterface& map,
    MultiStageOCP& ocp)
{
    assertCompleteness();
    // 使用契约：调用前 stage_params 可以为空；若调用方已把 stage_params 预分配成 N 个空 p，
    // 也视为“尚未设置参数”，先归一化为空 vector，使后续 validate() 通过。
    // 除此之外，非空的 stage_params 必须每步 p 均为 kParameterDim 维且全部有限。
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
        throw std::invalid_argument("ProblemUpdater: OCP configuration invalid - " + reason);
    }
    const int total_steps = ocp.totalSteps();
    if (static_cast<int>(current_traj.x.size()) < total_steps + 1) {
        throw std::invalid_argument(
            "ProblemUpdater: current_traj.x size must be at least totalSteps + 1");
    }
    for (int k = 0; k <= total_steps; ++k) {
        if (current_traj.x[k].size() < 3) {
            throw std::invalid_argument(
                "ProblemUpdater: current_traj.x[" + std::to_string(k) + "] dimension must be >= 3");
        }
        if (!current_traj.x[k].head(3).allFinite()) {
            throw std::invalid_argument(
                "ProblemUpdater: current_traj.x[" + std::to_string(k) + "] first three dimensions contain non-finite values");
        }
    }
    int global_k = 0;
    for (auto& segment : ocp.segments()) {
        segment.stage_params.resize(segment.N);
        for (int i = 0; i < segment.N; ++i) {
            const Vector& pose = current_traj.x[global_k];
            const std::vector<HalfSpace> half_spaces =
                map.queryHalfSpaces(pose, config_.selection_radius, config_.top_k);
            for (const auto& hs : half_spaces) {
                validateHalfSpace(hs);
            }
            // 保留已有的非凸走廊参数；若该步尚无 p，则从零向量开始；维度错误则抛异常
            Vector& p = segment.stage_params[i].p;
            if (p.size() == 0) {
                p = Vector::Zero(kParameterDim);
            } else if (p.size() != kParameterDim) {
                throw std::invalid_argument(
                    "ProblemUpdater: stage_params[" + std::to_string(i)
                    + "].p dimension invalid, expected 0 or " + std::to_string(kParameterDim));
            }
            buildParameterVector(half_spaces, p);
            ++global_k;
        }
    }
}

void ProblemUpdater::assertCompleteness() const
{
    if (!(config_.selection_radius > config_.max_step_displacement + config_.safety_margin)) {
        throw std::invalid_argument(
            "FATAL: selection_radius too small, mathematical completeness violated! Ensure selection_radius is based on GJK contour distance!");
    }
}

void ProblemUpdater::validateHalfSpace(const HalfSpace& hs) const
{
    if (!isValidHalfSpace(hs)) {
        throw std::invalid_argument(
            "ProblemUpdater: invalid HalfSpace: normal must be 2D, finite and non-zero, intercept must be finite");
    }
}

void ProblemUpdater::buildParameterVector(const std::vector<HalfSpace>& half_spaces,
    Vector& p) const
{
    if (p.size() != kParameterDim) {
        throw std::invalid_argument(
            "ProblemUpdater::buildParameterVector: p dimension must be "
            + std::to_string(kParameterDim));
    }
    // ProblemUpdater 只负责更新凸走廊区间 p[15:45]；先清零，再写入本次 Top-K 半空间。
    // Dummy 补零是数学安全的：零法向量对应的半空间为 dot(0, x_world) - 0 <= 0，
    // 即 0 <= 0 恒成立；其 Jacobian 对应行亦为 0，QP 中退化为惰性约束。
    p.segment(kHalfSpaceStart, kNormalDim * kMaxHalfSpaces).setZero();
    p.segment(kInterceptStart, kMaxHalfSpaces).setZero();
    const int n_hs = std::min(config_.top_k, static_cast<int>(half_spaces.size()));
    for (int i = 0; i < n_hs; ++i) {
        const HalfSpace& hs = half_spaces[i];
        for (int d = 0; d < kNormalDim; ++d) {
            p(kHalfSpaceStart + kNormalDim * i + d) = hs.normal(d);
        }
        p(kInterceptStart + i) = hs.intercept;
    }
}
} // namespace stc_SQP
