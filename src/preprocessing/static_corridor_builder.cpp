#include "static_corridor_builder.h"

#include <omp.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "../core/NMPC/vehicle_circle_geometry.h"
#include "../util/constants.h"

namespace apa_post_processor {
namespace {
// ESDF 梯度有效性的最小模长阈值：低于该值认为无法提供可靠方向信息。
constexpr double kMinValidGradientNorm = 1e-12;
}  // namespace

StaticCorridorBuilder::StaticCorridorBuilder(
    const StaticCorridorBuilderConfig& config)
    : config_(config) {
    if (config_.soft_margin < 0.0 || !std::isfinite(config_.soft_margin)) {
        throw std::invalid_argument(
            "StaticCorridorBuilderConfig: soft_margin must be a non-negative "
            "finite value");
    }
}

void StaticCorridorBuilder::validateInputs(
    const std::vector<TrajectoryPoint>& z_ref, const ESDFMap& esdf_map,
    const VehicleFootprintModel& footprint_model) const {
    if (esdf_map.size() == 0U) {
        throw std::invalid_argument(
            "StaticCorridorBuilder requires a non-empty ESDF map");
    }
    if (z_ref.empty()) {
        throw std::invalid_argument(
            "StaticCorridorBuilder requires non-empty z_ref");
    }
    if (footprint_model.getCircleNum(CircleType::OUTER) == 0U) {
        throw std::invalid_argument(
            "StaticCorridorBuilder requires at least one outer circle");
    }
    if (footprint_model.getOuterRadius() <= 0.0) {
        throw std::invalid_argument(
            "StaticCorridorBuilder requires positive outer circle radius");
    }
    for (std::size_t k = 0; k < z_ref.size(); ++k) {
        if (!std::isfinite(z_ref[k].x) || !std::isfinite(z_ref[k].y) ||
            !std::isfinite(z_ref[k].theta)) {
            throw std::invalid_argument("StaticCorridorBuilder z_ref[" +
                                        std::to_string(k) +
                                        "] has non-finite x/y/theta");
        }
        if (!z_ref[k].hasV() || !z_ref[k].hasDelta()) {
            throw std::invalid_argument("StaticCorridorBuilder z_ref[" +
                                        std::to_string(k) +
                                        "] is missing v or delta");
        }
        if (!std::isfinite(z_ref[k].getV()) ||
            !std::isfinite(z_ref[k].getDelta())) {
            throw std::invalid_argument("StaticCorridorBuilder z_ref[" +
                                        std::to_string(k) +
                                        "] has non-finite v or delta");
        }
    }
}

double StaticCorridorBuilder::computeDScalar(double dist_ref,
                                             const Eigen::VectorXd& a_row,
                                             const Eigen::VectorXd& z_ref,
                                             double radius,
                                             double margin) const {
    if (!std::isfinite(dist_ref) || !a_row.allFinite() || !z_ref.allFinite() ||
        radius <= 0.0 || margin < 0.0 || !std::isfinite(radius) ||
        !std::isfinite(margin)) {
        throw std::invalid_argument(
            "StaticCorridorBuilder::computeDScalar received invalid inputs");
    }
    // 约束形式：-A^T * Z <= d_ref - A^T * Z_ref - R_m - margin
    // 这里 a_row 即为 A^T，因此 d = d_ref - a_row.dot(z_ref) - radius - margin
    //
    // 自洽性修正：若参考点自身已违反边界，补偿等量 violation，使约束在 Z=Z_ref
    // 处恰好取等
    const double violation = std::max(0.0, radius + margin - dist_ref);
    return dist_ref - a_row.dot(z_ref) - radius - margin + violation;
}

void StaticCorridorBuilder::assembleMatrixForm(
    StaticCorridorBuilderResult& result) const {
    // 对约束按确定性顺序排序
    std::sort(result.constraints.begin(), result.constraints.end(),
              [](const StaticCorridorConstraint& a,
                 const StaticCorridorConstraint& b) {
                  if (a.point_idx != b.point_idx) {
                      return a.point_idx < b.point_idx;
                  }
                  return a.circle_idx < b.circle_idx;
              });
    const int n_constraints = static_cast<int>(result.constraints.size());
    constexpr int kStateDim = 5;
    result.c_matrix.resize(n_constraints, kStateDim);
    result.d_vector.resize(n_constraints);
    for (int i = 0; i < n_constraints; ++i) {
        result.c_matrix.row(i) = result.constraints[i].c.transpose();
        result.d_vector(i) = result.constraints[i].d;
    }
}

StaticCorridorBuilderResult StaticCorridorBuilder::build(
    const std::vector<TrajectoryPoint>& z_ref, const ESDFMap& esdf_map,
    const VehicleFootprintModel& footprint_model) const {
    validateInputs(z_ref, esdf_map, footprint_model);
    StaticCorridorBuilderResult result;
    const auto local_centers =
        vehicle_circle_geometry::ExtractLocalCircleCenters(footprint_model,
                                                           CircleType::OUTER);
    const int circle_num = static_cast<int>(local_centers.size());
    const double radius = footprint_model.getOuterRadius();
    const int n_points = static_cast<int>(z_ref.size());
    // 每个 (point, circle) 产生一条舒适 soft 约束
    const int total_constraints = n_points * circle_num;
    result.constraints.reserve(static_cast<std::size_t>(total_constraints));
    // OpenMP 并行加速批量 ESDF 查询，按线程收集后合并
    const int n_threads = omp_get_max_threads();
    std::vector<std::vector<StaticCorridorConstraint>> thread_constraints(
        static_cast<std::size_t>(n_threads));
    for (auto& tc : thread_constraints) {
        tc.reserve(static_cast<std::size_t>(total_constraints / n_threads + 1));
    }
    // 跨线程共享失败状态
    std::atomic<bool> has_failure{false};
    std::string failure_msg;
#pragma omp parallel for schedule(static)
    for (int k = 0; k < n_points; ++k) {
        if (has_failure) {
            continue;
        }
        const int tid = omp_get_thread_num();
        const TrajectoryPoint& point = z_ref[k];
        const double cos_theta = std::cos(point.theta);
        const double sin_theta = std::sin(point.theta);
        Eigen::VectorXd z_ref_vec(5);
        z_ref_vec << point.x, point.y, point.theta, point.getV(),
            point.getDelta();
        for (int m = 0; m < circle_num; ++m) {
            if (has_failure) {
                break;
            }
            const double local_x = local_centers[m].x();
            const double local_y = local_centers[m].y();
            // 世界系圆心坐标
            const double cx =
                point.x + cos_theta * local_x - sin_theta * local_y;
            const double cy =
                point.y + sin_theta * local_x + cos_theta * local_y;
            const auto [dist_ref, grad] = esdf_map.getDistAndGrad(cx, cy);
            // 校验 ESDF 返回值
            if (!std::isfinite(dist_ref) || !grad.allFinite() ||
                grad.norm() < kMinValidGradientNorm) {
#pragma omp critical
                {
                    if (!has_failure) {
                        has_failure = true;
                        failure_msg =
                            "Invalid ESDF sample at z_ref[" +
                            std::to_string(k) + "], circle[" +
                            std::to_string(m) +
                            "]: dist=" + std::to_string(dist_ref) +
                            ", grad_norm=" + std::to_string(grad.norm());
                    }
                }
                break;
            }
            // 圆心对 theta 的偏导数，与 CircleFootprintEsdfConstraint 保持一致
            const double dcx_dtheta =
                -sin_theta * local_x - cos_theta * local_y;
            const double dcy_dtheta = cos_theta * local_x - sin_theta * local_y;
            // 超平面法向量 A^T = grad^T * J_C
            Eigen::VectorXd a_row(5);
            a_row << grad.x(), grad.y(),
                grad.x() * dcx_dtheta + grad.y() * dcy_dtheta, 0.0, 0.0;
            // 舒适边界
            StaticCorridorConstraint soft_constraint;
            soft_constraint.c = -a_row;
            soft_constraint.d = computeDScalar(dist_ref, a_row, z_ref_vec,
                                               radius, config_.soft_margin);
            soft_constraint.point_idx = k;
            soft_constraint.circle_idx = m;
            thread_constraints[tid].push_back(soft_constraint);
        }
    }
    if (has_failure) {
        result.success = false;
        result.status_msg = std::move(failure_msg);
        return result;
    }
    for (const auto& tc : thread_constraints) {
        result.constraints.insert(result.constraints.end(), tc.begin(),
                                  tc.end());
    }
    assembleMatrixForm(result);
    result.success = true;
    result.status_msg = "OK: " + std::to_string(result.constraints.size()) +
                        " corridor constraints built";
    return result;
}
}  // namespace apa_post_processor
