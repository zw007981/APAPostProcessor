#include "bspline_smoother.h"

#include <omp.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include "../util/logger.h"

namespace apa_post_processor {
namespace {
// NURBS Book A2.3：一次性计算 span 内全部非零 B 样条基函数的 0 到 n 阶导数。
// 使用 ndu 三角递推表避免递归求导在 clamped 端点出现 0/0，时间复杂度 O(p^2 +
// p*n)。
void computeBasisDerivatives(double u, int span, int p, int n,
                             const std::vector<double>& knots,
                             std::vector<std::vector<double>>& ders) {
    ders.assign(n + 1, std::vector<double>(p + 1, 0.0));
    // ndu[j][r] = 节点差分商，同时承担基函数值与导数递推分母
    std::vector<std::vector<double>> ndu(p + 1,
                                         std::vector<double>(p + 1, 0.0));
    // left[j]/right[j]：u 到 span 左右两侧节点的距离，用于 De Boor-Cox 递推权重
    std::vector<double> left(p + 1, 0.0);
    std::vector<double> right(p + 1, 0.0);

    // ---- 第一阶段：构造 ndu 三角表 ----
    // 第 0 行初始化为 1，表示 0 阶基函数在 u 处的“种子”。
    ndu[0][0] = 1.0;
    for (int j = 1; j <= p; ++j) {
        left[j] = u - knots[span + 1 - j];
        right[j] = knots[span + j] - u;
        double saved = 0.0;
        // De Boor-Cox 递推：由 j-1 阶递推到 j 阶
        for (int r = 0; r < j; ++r) {
            ndu[j][r] = right[r + 1] + left[j - r];
            const double temp = ndu[r][j - 1] / ndu[j][r];
            ndu[r][j] = saved + right[r + 1] * temp;
            saved = left[j - r] * temp;
        }
        ndu[j][j] = saved;
    }

    // 第二阶段：提取 0 阶基函数值（ndu 表对角线）
    for (int j = 0; j <= p; ++j) {
        ders[0][j] = ndu[j][p];
    }

    // 第三阶段：递推计算 1..n 阶导数（滚动数组技巧）
    std::vector<std::vector<double>> a(2, std::vector<double>(p + 1, 0.0));
    for (int r = 0; r <= p; ++r) {
        int s1 = 0;
        int s2 = 1;
        a[0][0] = 1.0;
        for (int k = 1; k <= n; ++k) {
            double d = 0.0;
            const int rk = r - k;
            const int pk = p - k;
            // 左侧边界项
            if (r >= k) {
                a[s2][0] = a[s1][0] / ndu[pk + 1][rk];
                d = a[s2][0] * ndu[rk][pk];
            }
            // 中间项：相邻系数差分递推
            const int j1 = (r >= k - 1) ? 1 : k - r;
            const int j2 = (r <= pk) ? k - 1 : p - r;
            for (int j = j1; j <= j2; ++j) {
                a[s2][j] = (a[s1][j] - a[s1][j - 1]) / ndu[pk + 1][rk + j];
                d += a[s2][j] * ndu[rk + j][pk];
            }
            // 右侧边界项
            if (r <= pk) {
                a[s2][k] = -a[s1][k - 1] / ndu[pk + 1][r];
                d += a[s2][k] * ndu[r][pk];
            }
            ders[k][r] = d;
            // 交换滚动数组下标，进入下一阶。
            std::swap(s1, s2);
        }
    }

    // ---- 第四阶段：导数阶乘缩放 ----
    // 上述递推得到的是“差分形式”的导数系数，需要按导数阶数乘以
    // p! / (p-k)! 才能得到真正的 k 阶导数。变量 r 依次保存该阶乘值。
    int r = p;
    for (int k = 1; k <= n; ++k) {
        for (int j = 0; j <= p; ++j) {
            ders[k][j] *= static_cast<double>(r);
        }
        r *= (p - k);
    }
}

// 将曲线切向角转换为车辆航向角（前进段一致，后退段相差 π），归一化到 [-π, π]
double TangentToVehicleHeading(double tangent_theta, Direction direction) {
    if (direction == Direction::BACKWARD) {
        tangent_theta += PI;
    }
    return std::remainder(tangent_theta, 2.0 * PI);
}

// 根据机动段方向获取曲线首尾处的切向方向
double HeadingToTangentDirection(double heading_theta, Direction direction) {
    return TangentToVehicleHeading(heading_theta, direction);
}

// L-BFGS 目标函数：F_data + F_smooth + F_collision + F_reg，Frozen-theta 策略
struct BSplineObjective {
    using Scalar = double;
    using Vector = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;

    const BSplineSmootherConfig& config;
    const ESDFMap& esdf_map;
    const BSplineSmoother::AnchorPoints& anchors;
    const std::vector<Eigen::Vector2d>& ref_points;
    const std::vector<Eigen::Vector2d>& ref_normals;
    const std::vector<BSplineSmoother::BasisPack>& basis_packs;
    const std::vector<BSplineSmoother::EvalPoint>& dense_eval_points;
    const std::vector<Eigen::Vector2d>& outer_circle_local_centers;
    double outer_circle_radius;
    int control_point_count;
    double avg_chord_length;
    Direction direction;
    // 短段曲率放宽：true 时 P₁/P_{N-1} 不作为硬锚定，而是自由优化变量。
    // ref_points/ref_normals 已由调用方扩展以覆盖这些额外控制点。
    bool free_p1 = false;
    bool free_pn1 = false;
    // x 分量到控制点索引的映射：x[2*j], x[2*j+1] →
    // control_points[free_cp_indices[j]]
    std::vector<int> free_cp_indices;

    Scalar operator()(const Vector& x, Vector& grad) {
        const int N = control_point_count - 1;
        const int n_free = static_cast<int>(free_cp_indices.size());
        std::vector<Eigen::Vector2d> control_points(control_point_count);
        control_points[0] = anchors.p0;
        control_points[N] = anchors.pn;
        // 固定锚定点：不在 free_cp_indices 中的首尾控制点用硬锚定值
        if (!free_p1) {
            control_points[1] = anchors.p1;
        }
        if (!free_pn1) {
            control_points[N - 1] = anchors.pn_1;
        }
        for (int j = 0; j < n_free; ++j) {
            control_points[free_cp_indices[j]] =
                Eigen::Vector2d(x[2 * j], x[2 * j + 1]);
        }
        // F_data：对所有自由控制点惩罚法向漂移。
        // ref_points 按自由控制点顺序排列：P₁(若自由), P₂, ..., P_{N-2},
        // P_{N-1}(若自由)
        double f_data = 0.0;
        std::vector<Eigen::Vector2d> data_grad(control_point_count,
                                               Eigen::Vector2d::Zero());
        for (int j = 0; j < n_free; ++j) {
            const int ci = free_cp_indices[j];
            const Eigen::Vector2d delta = control_points[ci] - ref_points[j];
            const double proj = delta.dot(ref_normals[j]);
            f_data += proj * proj;
            data_grad[ci] = 2.0 * proj * ref_normals[j];
        }
        // F_smooth：对控制多边形做一/二/三阶差分惩罚。
        double f_smooth_1 = 0.0;
        double f_smooth_2 = 0.0;
        double f_smooth_3 = 0.0;
        for (int i = 0; i <= N - 1; ++i) {
            const Eigen::Vector2d d = control_points[i + 1] - control_points[i];
            f_smooth_1 += d.squaredNorm();
        }
        for (int i = 0; i <= N - 2; ++i) {
            const Eigen::Vector2d diff = control_points[i + 2] -
                                         2.0 * control_points[i + 1] +
                                         control_points[i];
            f_smooth_2 += diff.squaredNorm();
        }
        for (int i = 0; i <= N - 3; ++i) {
            const Eigen::Vector2d diff =
                control_points[i + 3] - 3.0 * control_points[i + 2] +
                3.0 * control_points[i + 1] - control_points[i];
            f_smooth_3 += diff.squaredNorm();
        }
        std::vector<Eigen::Vector2d> smooth_grad(control_point_count,
                                                 Eigen::Vector2d::Zero());
        for (int k = 0; k <= N; ++k) {
            if (k > 0) {
                smooth_grad[k] +=
                    2.0 * (control_points[k] - control_points[k - 1]);
            }
            if (k < N) {
                smooth_grad[k] +=
                    2.0 * (control_points[k] - control_points[k + 1]);
            }
        }
        std::vector<Eigen::Vector2d> smooth_grad2(control_point_count,
                                                  Eigen::Vector2d::Zero());
        for (int i = 0; i <= N - 2; ++i) {
            const Eigen::Vector2d diff = control_points[i + 2] -
                                         2.0 * control_points[i + 1] +
                                         control_points[i];
            smooth_grad2[i] += 2.0 * diff;
            smooth_grad2[i + 1] -= 4.0 * diff;
            smooth_grad2[i + 2] += 2.0 * diff;
        }
        std::vector<Eigen::Vector2d> smooth_grad3(control_point_count,
                                                  Eigen::Vector2d::Zero());
        for (int i = 0; i <= N - 3; ++i) {
            const Eigen::Vector2d diff =
                control_points[i + 3] - 3.0 * control_points[i + 2] +
                3.0 * control_points[i + 1] - control_points[i];
            smooth_grad3[i] += -2.0 * diff;
            smooth_grad3[i + 1] += 6.0 * diff;
            smooth_grad3[i + 2] += -6.0 * diff;
            smooth_grad3[i + 3] += 2.0 * diff;
        }
        for (int k = 0; k <= N; ++k) {
            smooth_grad[k] = config.weight_smooth_d1 * smooth_grad[k] +
                             config.weight_smooth_d2 * smooth_grad2[k] +
                             config.weight_smooth_d3 * smooth_grad3[k];
        }
        const double f_smooth = config.weight_smooth_d1 * f_smooth_1 +
                                config.weight_smooth_d2 * f_smooth_2 +
                                config.weight_smooth_d3 * f_smooth_3;
        // F_reg：相邻控制点弦长与冻结平均弦长的偏差平方和。
        double f_reg = 0.0;
        std::vector<Eigen::Vector2d> reg_grad(control_point_count,
                                              Eigen::Vector2d::Zero());
        for (int i = 0; i <= N - 1; ++i) {
            const Eigen::Vector2d d = control_points[i + 1] - control_points[i];
            const double len = d.norm();
            if (len < 1e-12) {
                continue;
            }
            const double residual = len - avg_chord_length;
            f_reg += residual * residual;
            const double coeff = 2.0 * residual / len;
            reg_grad[i] -= coeff * d;
            reg_grad[i + 1] += coeff * d;
        }
        // F_collision：在密集配点处对车身子圆做三次铰链惩罚。
        // 使用完整梯度（不再冻结θ）。
        // 该循环每次 L-BFGS
        // 迭代（含每次线搜索试探）都会被完整调用一次，是热路径， 因此对
        // dense_eval_points 维度使用 OpenMP 并行化。
        double f_collision = 0.0;
        std::vector<Eigen::Vector2d> collision_grad(control_point_count,
                                                    Eigen::Vector2d::Zero());
        // 每个线程维护一份独立的梯度累加器，避免跨线程原子操作带来的竞争开销；
        // 循环结束后再逐元素合并。参考 static_corridor_builder.cpp
        // 中的同范式实现。
        const std::size_t n_dense = dense_eval_points.size();
        // 实测表明：在此数据集上，过度 fork 大量线程会因同步开销而严重劣化
        // （22 线程时耗时从 ~100ms 量级退化到 ~10s）。因此不直接使用
        // omp_get_max_threads()，而是按物理核心数的 1/4 动态选择线程数，
        // 并保证至少 2 线程（若机器支持）、不超过 omp_get_max_threads()。
        // 该策略在不同平台间更可移植，后续可基于多平台 benchmark 数据继续调优。
        const int n_threads = std::min(omp_get_max_threads(),
                                       std::max(2, omp_get_max_threads() / 4));
        std::vector<std::vector<Eigen::Vector2d>> thread_collision_grads(
            static_cast<std::size_t>(n_threads),
            std::vector<Eigen::Vector2d>(control_point_count,
                                         Eigen::Vector2d::Zero()));
        // 小循环的 fork/join 开销可能抵消并行收益，仅在配点数较多时启用。
        // 阈值由 Phase B benchmark 实测确定，取 64 作为保守初值。
#pragma omp parallel for schedule(static) reduction(+ : f_collision) \
    num_threads(n_threads) if (n_dense > 64)
        for (std::size_t di = 0; di < n_dense; ++di) {
            const auto& ep = dense_eval_points[di];
            const int tid = omp_get_thread_num();
            Eigen::Vector2d pos = Eigen::Vector2d::Zero();
            Eigen::Vector2d vel = Eigen::Vector2d::Zero();
            for (std::size_t k = 0; k < ep.indices.size(); ++k) {
                const int idx = ep.indices[k];
                pos += control_points[idx] * ep.values[k];
                vel += control_points[idx] * ep.d1[k];
            }
            const double tan_norm_sq = vel.x() * vel.x() + vel.y() * vel.y();
            const double inv_tan_norm_sq =
                (tan_norm_sq > 1e-12) ? (1.0 / tan_norm_sq) : 0.0;
            const Eigen::Vector2d normal(-vel.y(), vel.x());
            const double theta = TangentToVehicleHeading(
                std::atan2(vel.y(), vel.x()), direction);
            const double cos_th = std::cos(theta);
            const double sin_th = std::sin(theta);
            for (std::size_t k = 0; k < outer_circle_local_centers.size();
                 ++k) {
                const Eigen::Vector2d& local = outer_circle_local_centers[k];
                const double lx = local.x();
                const double ly = local.y();
                const Eigen::Vector2d center =
                    pos + Eigen::Vector2d(cos_th * lx - sin_th * ly,
                                          sin_th * lx + cos_th * ly);
                const auto [dist, grad_esdf] =
                    esdf_map.getDistAndGrad(center.x(), center.y());
                const double intrusion = std::max(
                    0.0, outer_circle_radius + config.collision_margin - dist);
                if (intrusion <= 0.0) {
                    continue;
                }
                f_collision += intrusion * intrusion * intrusion;
                const double coeff = -3.0 * intrusion * intrusion;
                // R'(theta) * local：圆心对航向的偏导数向量
                const Eigen::Vector2d rprime_local(-sin_th * lx - cos_th * ly,
                                                   cos_th * lx - sin_th * ly);
                const double grad_dot_rprime =
                    grad_esdf.x() * rprime_local.x() +
                    grad_esdf.y() * rprime_local.y();
                const double orient_factor =
                    coeff * grad_dot_rprime * inv_tan_norm_sq;
                for (std::size_t kk = 0; kk < ep.indices.size(); ++kk) {
                    const int idx = ep.indices[kk];
                    thread_collision_grads[tid][idx] +=
                        coeff * grad_esdf * ep.values[kk];
                    thread_collision_grads[tid][idx] +=
                        orient_factor * normal * ep.d1[kk];
                }
            }
        }
        for (const auto& tcg : thread_collision_grads) {
            for (int i = 0; i < control_point_count; ++i) {
                collision_grad[i] += tcg[i];
            }
        }
        // F_kappa：显式曲线曲率 L2 惩罚，仅 |κ| > max_kappa 时激活。
        // 直接约束 B 样条曲线的几何曲率（而非控制多边形的二阶差分），
        // 消除 F_smooth_d2 因节点间距 h 变化导致的 κ 等效权重漂移。
        double f_kappa = 0.0;
        std::vector<Eigen::Vector2d> kappa_grad(control_point_count,
                                                Eigen::Vector2d::Zero());
        for (const auto& ep : dense_eval_points) {
            Eigen::Vector2d vel = Eigen::Vector2d::Zero();
            Eigen::Vector2d acc = Eigen::Vector2d::Zero();
            for (std::size_t kk = 0; kk < ep.indices.size(); ++kk) {
                const int idx = ep.indices[kk];
                vel += control_points[idx] * ep.d1[kk];
                acc += control_points[idx] * ep.d2[kk];
            }
            const double vsq = vel.x() * vel.x() + vel.y() * vel.y();
            if (vsq < 1e-12) {
                continue;
            }
            const double v = std::sqrt(vsq);
            const double v3 = v * vsq;  // |C'|³
            const double cross = vel.x() * acc.y() - vel.y() * acc.x();
            const double kappa = cross / v3;
            const double abs_k = std::abs(kappa);
            if (abs_k <= config.max_kappa) {
                continue;
            }
            const double excess = abs_k - config.max_kappa;
            f_kappa += excess * excess;
            // ∂F_kappa/∂P_k = 2·excess·sign(κ)·∂κ/∂P_k
            const double sign_k = (kappa > 0.0) ? 1.0 : -1.0;
            const double coeff = 2.0 * excess * sign_k;
            // ∇_k κ = (1/v³)·(N'_k·(ay,-ax) + N''_k·(-vy,vx))
            //         - (3κ/v²)·N'_k·(vx,vy)
            const double inv_v3 = 1.0 / v3;
            const double inv_vsq = 1.0 / vsq;
            const double three_kappa_inv_vsq = 3.0 * kappa * inv_vsq;
            for (std::size_t kk = 0; kk < ep.indices.size(); ++kk) {
                const int idx = ep.indices[kk];
                const double N1 = ep.d1[kk];
                const double N2 = ep.d2[kk];
                const double dk_dx = inv_v3 * (N1 * acc.y() + N2 * (-vel.y())) -
                                     three_kappa_inv_vsq * N1 * vel.x();
                const double dk_dy = inv_v3 * (N2 * vel.x() - N1 * acc.x()) -
                                     three_kappa_inv_vsq * N1 * vel.y();
                kappa_grad[idx].x() += coeff * dk_dx;
                kappa_grad[idx].y() += coeff * dk_dy;
            }
        }
        const double total = config.weight_data * f_data + f_smooth +
                             config.weight_collision * f_collision +
                             config.weight_kappa * f_kappa +
                             config.weight_reg * f_reg;
        grad.resize(2 * n_free);
        grad.setZero();
        for (int j = 0; j < n_free; ++j) {
            const int ci = free_cp_indices[j];
            const Eigen::Vector2d g =
                config.weight_data * data_grad[ci] + smooth_grad[ci] +
                config.weight_collision * collision_grad[ci] +
                config.weight_kappa * kappa_grad[ci] +
                config.weight_reg * reg_grad[ci];
            grad[2 * j] = g.x();
            grad[2 * j + 1] = g.y();
        }
        return total;
    }
};
}  // namespace

BSplineSmoother::BSplineSmoother(const BSplineSmootherConfig& config,
                                 const VehicleParams& vehicle_params,
                                 const VehicleFootprintModel& footprint_model,
                                 const ESDFMap& esdf_map)
    : config_(config), vehicle_params_(vehicle_params), esdf_map_(esdf_map) {
    validateInputs();
    outer_circle_num_ = footprint_model.getCircleNum(CircleType::OUTER);
    outer_circle_local_centers_.resize(outer_circle_num_);
    std::vector<Eigen::Matrix<double, 2, 3>> jacobians(outer_circle_num_);
    footprint_model.calInterpolatedCenters(0.0, 0.0, 0.0, CircleType::OUTER,
                                           outer_circle_local_centers_,
                                           jacobians);
    outer_circle_radius_ = footprint_model.getOuterRadius();
}

void BSplineSmoother::validateInputs() const {
    if (config_.dense_step_dist <= 0.0 || config_.weight_data < 0.0 ||
        config_.weight_smooth_d1 < 0.0 || config_.weight_smooth_d2 < 0.0 ||
        config_.weight_smooth_d3 < 0.0 || config_.weight_collision < 0.0 ||
        config_.weight_reg < 0.0 || config_.weight_kappa < 0.0 ||
        config_.max_kappa <= 0.0 || config_.collision_margin < 0.0 ||
        config_.anchor_extension_length < 0.0 ||
        config_.min_segment_arc_length_for_degradation < 0.0 ||
        config_.collision_validation_tolerance < 0.0 ||
        config_.lbfgs_max_iterations <= 0 || config_.lbfgs_epsilon <= 0.0 ||
        config_.lbfgs_epsilon_rel < 0.0 || config_.lbfgs_m <= 0 ||
        config_.lbfgs_max_linesearch <= 0 ||
        config_.lbfgs_linesearch_algo < 1 ||
        config_.lbfgs_linesearch_algo > 3 || config_.lbfgs_ftol <= 0.0 ||
        config_.lbfgs_ftol >= 0.5 ||
        config_.lbfgs_wolfe <= config_.lbfgs_ftol ||
        config_.lbfgs_wolfe >= 1.0 || config_.arc_length_table_step <= 0.0 ||
        config_.arc_length_table_step > 1.0) {
        throw std::invalid_argument(
            "BSplineSmootherConfig contains invalid values");
    }
    if (vehicle_params_.wheelbase <= 0.0 ||
        vehicle_params_.max_steer_angle <= 0.0) {
        throw std::invalid_argument(
            "VehicleParams invalid for BSplineSmoother");
    }
}

int BSplineSmoother::determineControlPointCount(double arc_length) const {
    const int internal_count =
        std::max(2, static_cast<int>(
                        std::ceil(arc_length / config_.control_point_spacing)));
    return internal_count + 4;
}

BSplineSmoother::AnchorPoints BSplineSmoother::buildAnchorPoints(
    const Maneuver& maneuver, double arc_length, int control_point_count,
    Direction direction) const {
    const auto& pts = maneuver.points;
    const TrajectoryPoint& start = pts.front();
    const TrajectoryPoint& end = pts.back();
    const int N_c = control_point_count - 1;
    const double internal_spacing =
        (N_c > 4) ? (arc_length / static_cast<double>(N_c)) : arc_length;
    const double max_extension = std::max(
        1e-3,
        std::min(config_.anchor_extension_length, 0.5 * internal_spacing));
    AnchorPoints anchors;
    anchors.p0 = Eigen::Vector2d(start.x, start.y);
    anchors.pn = Eigen::Vector2d(end.x, end.y);
    // 锚定方向始终使用车辆航向 → 运动切向。
    // 短段上 P₁ 自由优化后切向由 B 样条自行选择，锚定值仅作参考。
    const double start_tangent =
        HeadingToTangentDirection(start.theta, direction);
    const double end_tangent = HeadingToTangentDirection(end.theta, direction);
    anchors.p1 =
        anchors.p0 + max_extension * Eigen::Vector2d(std::cos(start_tangent),
                                                     std::sin(start_tangent));
    anchors.pn_1 =
        anchors.pn - max_extension * Eigen::Vector2d(std::cos(end_tangent),
                                                     std::sin(end_tangent));
    return anchors;
}

void BSplineSmoother::buildReferenceControlPoints(
    const Maneuver& maneuver, int control_point_count,
    std::vector<Eigen::Vector2d>& ref_points,
    std::vector<Eigen::Vector2d>& ref_normals) const {
    const int internal_count = control_point_count - 4;
    const int N_c = control_point_count - 1;
    ref_points.resize(internal_count);
    ref_normals.resize(internal_count);
    if (internal_count == 0) {
        return;
    }
    const std::size_t n = maneuver.size();
    std::vector<double> cumulative(n, 0.0);
    for (std::size_t i = 1; i < n; ++i) {
        const Eigen::Vector2d p1(maneuver.points[i - 1].x,
                                 maneuver.points[i - 1].y);
        const Eigen::Vector2d p2(maneuver.points[i].x, maneuver.points[i].y);
        cumulative[i] = cumulative[i - 1] + (p2 - p1).norm();
    }
    const double total_length = cumulative.back();
    if (total_length < 1e-12) {
        for (int i = 0; i < internal_count; ++i) {
            ref_points[i] = Eigen::Vector2d(maneuver.points.front().x,
                                            maneuver.points.front().y);
            ref_normals[i] = Eigen::Vector2d(0.0, 1.0);
        }
        return;
    }
    for (int j = 0; j < internal_count; ++j) {
        const int i = j + 2;
        const double fraction =
            static_cast<double>(i) / static_cast<double>(N_c);
        const double target_s = fraction * total_length;
        // cumulative 单调递增，target_s 也随 j 单调递增，使用 lower_bound
        // 将单次查找从 O(n) 降到 O(log n)，整体从 O(internal_count × n) 降到
        // O(internal_count × log n)。
        auto it =
            std::lower_bound(cumulative.begin(), cumulative.end(), target_s);
        std::size_t idx = static_cast<std::size_t>(it - cumulative.begin());
        idx = std::clamp(idx, std::size_t{1}, n - 1);
        const std::size_t prev = idx - 1;
        const double seg_length = cumulative[idx] - cumulative[prev];
        const double alpha = (seg_length > 1e-12)
                                 ? (target_s - cumulative[prev]) / seg_length
                                 : 0.0;
        const Eigen::Vector2d p_prev(maneuver.points[prev].x,
                                     maneuver.points[prev].y);
        const Eigen::Vector2d p_curr(maneuver.points[idx].x,
                                     maneuver.points[idx].y);
        ref_points[j] = p_prev + alpha * (p_curr - p_prev);
    }
    for (int j = 0; j < internal_count; ++j) {
        Eigen::Vector2d tangent;
        if (internal_count == 1) {
            tangent = Eigen::Vector2d(std::cos(maneuver.points.front().theta),
                                      std::sin(maneuver.points.front().theta));
        } else if (j == 0) {
            tangent = ref_points[j + 1] - ref_points[j];
        } else if (j == internal_count - 1) {
            tangent = ref_points[j] - ref_points[j - 1];
        } else {
            tangent = ref_points[j + 1] - ref_points[j - 1];
        }
        const double len = tangent.norm();
        if (len < 1e-12) {
            ref_normals[j] = Eigen::Vector2d(0.0, 1.0);
        } else {
            tangent /= len;
            ref_normals[j] = Eigen::Vector2d(-tangent.y(), tangent.x());
        }
    }
}

std::vector<double> BSplineSmoother::buildKnotVector(
    int control_point_count) const {
    const int N_c = control_point_count - 1;
    constexpr int p = 4;
    const int num_knots = N_c + p + 2;
    std::vector<double> knots(num_knots, 0.0);
    for (int i = 0; i < num_knots; ++i) {
        if (i < p + 1) {
            knots[i] = 0.0;
        } else if (i > N_c) {
            knots[i] = 1.0;
        } else {
            knots[i] =
                static_cast<double>(i - p) / static_cast<double>(N_c - p + 1);
        }
    }
    return knots;
}

void BSplineSmoother::computeBasisAtU(double u,
                                      const std::vector<double>& knot_vector,
                                      int control_point_count,
                                      BasisPack& bp) const {
    const int N_c = control_point_count - 1;
    constexpr int p = 4;
    bp.u = u;
    bp.span = p;
    while (bp.span < static_cast<int>(knot_vector.size()) - p - 2 &&
           knot_vector[bp.span + 1] <= u) {
        ++bp.span;
    }
    if (u >= 1.0) {
        bp.span = N_c;
    }
    bp.indices.clear();
    bp.values.clear();
    bp.d1.clear();
    bp.d2.clear();
    bp.d3.clear();

    std::vector<std::vector<double>> ders;
    computeBasisDerivatives(u, bp.span, p, 3, knot_vector, ders);
    const int start_idx = bp.span - p;
    for (int offset = 0; offset <= p; ++offset) {
        const int idx = start_idx + offset;
        if (idx < 0 || idx > N_c) {
            continue;
        }
        // 端点处基函数值可能为0但导数非0（如u=0时N_{1,4}=0而N'_{1,4}=4/L），
        // 因此不能按函数值过滤，必须保留全部5个局部非零项。
        bp.indices.push_back(idx);
        bp.values.push_back(ders[0][offset]);
        bp.d1.push_back(ders[1][offset]);
        bp.d2.push_back(ders[2][offset]);
        bp.d3.push_back(ders[3][offset]);
    }
}

void BSplineSmoother::precomputeBasisPacks(
    const std::vector<double>& knot_vector, int control_point_count,
    std::vector<BasisPack>& basis_packs) const {
    const std::size_t num_steps = static_cast<std::size_t>(std::ceil(
                                      1.0 / config_.arc_length_table_step)) +
                                  1;
    basis_packs.clear();
    basis_packs.reserve(num_steps);
    for (std::size_t i = 0; i < num_steps; ++i) {
        const double u = std::min(
            static_cast<double>(i) * config_.arc_length_table_step, 1.0);
        BasisPack bp;
        computeBasisAtU(u, knot_vector, control_point_count, bp);
        basis_packs.push_back(std::move(bp));
    }
}

void BSplineSmoother::evaluateFineGrid(
    const std::vector<Eigen::Vector2d>& control_points,
    const std::vector<BasisPack>& basis_packs,
    std::vector<Eigen::Vector2d>& positions,
    std::vector<Eigen::Vector2d>& velocities) const {
    positions.resize(basis_packs.size());
    velocities.resize(basis_packs.size());
    for (std::size_t j = 0; j < basis_packs.size(); ++j) {
        const auto& bp = basis_packs[j];
        Eigen::Vector2d pos = Eigen::Vector2d::Zero();
        Eigen::Vector2d vel = Eigen::Vector2d::Zero();
        for (std::size_t k = 0; k < bp.indices.size(); ++k) {
            const int idx = bp.indices[k];
            pos += control_points[idx] * bp.values[k];
            vel += control_points[idx] * bp.d1[k];
        }
        positions[j] = pos;
        velocities[j] = vel;
    }
}

std::vector<std::pair<double, double>> BSplineSmoother::buildArcLengthTable(
    const std::vector<Eigen::Vector2d>& positions) const {
    std::vector<std::pair<double, double>> table;
    table.reserve(positions.size());
    double s = 0.0;
    table.emplace_back(0.0, 0.0);
    for (std::size_t i = 1; i < positions.size(); ++i) {
        s += (positions[i] - positions[i - 1]).norm();
        const double u = static_cast<double>(i) * config_.arc_length_table_step;
        table.emplace_back(std::min(u, 1.0), s);
    }
    return table;
}

std::vector<BSplineSmoother::EvalPoint> BSplineSmoother::buildDenseEvalPoints(
    const std::vector<double>& knot_vector, int control_point_count,
    const std::vector<std::pair<double, double>>& arc_length_table) const {
    std::vector<EvalPoint> dense_points;
    const double total_length = arc_length_table.back().second;
    if (total_length < 1e-12) {
        EvalPoint ep;
        ep.u = 0.0;
        ep.s = 0.0;
        BasisPack bp;
        computeBasisAtU(0.0, knot_vector, control_point_count, bp);
        ep.indices = std::move(bp.indices);
        ep.values = std::move(bp.values);
        ep.d1 = std::move(bp.d1);
        ep.d2 = std::move(bp.d2);
        dense_points.push_back(std::move(ep));
        return dense_points;
    }
    const std::size_t num_dense = std::max(
        std::size_t(2), static_cast<std::size_t>(
                            std::ceil(total_length / config_.dense_step_dist)) +
                            1);
    dense_points.reserve(num_dense);
    for (std::size_t i = 0; i < num_dense; ++i) {
        const double s_target = static_cast<double>(i) /
                                static_cast<double>(num_dense - 1) *
                                total_length;
        const auto it = std::lower_bound(
            arc_length_table.begin(), arc_length_table.end(), s_target,
            [](const std::pair<double, double>& item, double value) {
                return item.second < value;
            });
        double u_target = 0.0;
        double s_actual = 0.0;
        if (it == arc_length_table.begin()) {
            u_target = it->first;
            s_actual = it->second;
        } else if (it == arc_length_table.end()) {
            u_target = arc_length_table.back().first;
            s_actual = arc_length_table.back().second;
        } else {
            const auto prev = std::prev(it);
            const double denom = it->second - prev->second;
            const double alpha =
                (denom > 1e-12) ? (s_target - prev->second) / denom : 0.0;
            u_target = prev->first + alpha * (it->first - prev->first);
            s_actual = s_target;
        }
        EvalPoint ep;
        ep.u = u_target;
        ep.s = s_actual;
        BasisPack bp;
        computeBasisAtU(u_target, knot_vector, control_point_count, bp);
        ep.indices = std::move(bp.indices);
        ep.values = std::move(bp.values);
        ep.d1 = std::move(bp.d1);
        ep.d2 = std::move(bp.d2);
        dense_points.push_back(std::move(ep));
    }
    return dense_points;
}

BSplineSmoother::Result BSplineSmoother::buildDegenerateResult(
    const TrajectoryPoint& start, const TrajectoryPoint& end,
    Direction direction) const {
    Result result;
    result.success = true;
    result.degenerate = true;
    const Eigen::Vector2d start_pos(start.x, start.y);
    const Eigen::Vector2d end_pos(end.x, end.y);
    const Eigen::Vector2d delta = end_pos - start_pos;
    // 退化路径用5个控制点表示一条直线段，保证后续阶段始终拿到合法的p=4 B样条。
    result.control_points.resize(5);
    for (int i = 0; i < 5; ++i) {
        result.control_points[i] =
            start_pos + delta * (static_cast<double>(i) / 4.0);
    }
    const std::size_t num_steps = static_cast<std::size_t>(std::ceil(
                                      1.0 / config_.arc_length_table_step)) +
                                  1;
    result.arc_length_table.reserve(num_steps);
    const double total_length = delta.norm();
    for (std::size_t i = 0; i < num_steps; ++i) {
        const double u = std::min(
            static_cast<double>(i) * config_.arc_length_table_step, 1.0);
        const double s = u * total_length;
        result.arc_length_table.emplace_back(u, s);
    }
    const std::size_t num_dense = std::max(
        std::size_t(2), static_cast<std::size_t>(
                            std::ceil(total_length / config_.dense_step_dist)) +
                            1);
    result.dense_points.reserve(num_dense);
    // 退化线段切向为 start->end；车辆航向需按机动段方向转换。
    const double theta =
        TangentToVehicleHeading(std::atan2(delta.y(), delta.x()), direction);
    const double cos_th = std::cos(theta);
    const double sin_th = std::sin(theta);
    for (std::size_t i = 0; i < num_dense; ++i) {
        const double u =
            static_cast<double>(i) / static_cast<double>(num_dense - 1);
        const double s = u * total_length;
        const Eigen::Vector2d pos = start_pos + delta * u;
        DensePointData dpd;
        dpd.u = u;
        dpd.s = s;
        dpd.position = pos;
        dpd.theta = theta;
        dpd.circles.reserve(outer_circle_num_);
        for (std::size_t k = 0; k < outer_circle_num_; ++k) {
            const Eigen::Vector2d& local = outer_circle_local_centers_[k];
            const Eigen::Vector2d center =
                pos + Eigen::Vector2d(cos_th * local.x() - sin_th * local.y(),
                                      sin_th * local.x() + cos_th * local.y());
            const auto [dist, grad] =
                esdf_map_.getDistAndGrad(center.x(), center.y());
            CircleEsdfData ced;
            ced.local_offset = local;
            ced.center = center;
            ced.dist = dist;
            ced.grad = grad;
            dpd.circles.push_back(ced);
        }
        result.dense_points.push_back(std::move(dpd));
    }
    double max_intrusion = 0.0;
    for (const auto& dpd : result.dense_points) {
        for (const auto& c : dpd.circles) {
            const double intrusion = std::max(
                0.0, outer_circle_radius_ + config_.collision_margin - c.dist);
            max_intrusion = std::max(max_intrusion, intrusion);
        }
    }
    result.max_intrusion_depth = max_intrusion;
    return result;
}

bool BSplineSmoother::validateCollisionFree(
    const std::vector<Eigen::Vector2d>& control_points,
    const std::vector<EvalPoint>& dense_eval_points,
    std::vector<DensePointData>& dense_points, double& max_intrusion_depth,
    Direction direction) const {
    dense_points.clear();
    dense_points.reserve(dense_eval_points.size());
    max_intrusion_depth = 0.0;
    for (const auto& ep : dense_eval_points) {
        Eigen::Vector2d pos = Eigen::Vector2d::Zero();
        Eigen::Vector2d vel = Eigen::Vector2d::Zero();
        for (std::size_t k = 0; k < ep.indices.size(); ++k) {
            const int idx = ep.indices[k];
            pos += control_points[idx] * ep.values[k];
            vel += control_points[idx] * ep.d1[k];
        }
        // dense_points.theta 语义为车辆航向，需按机动段方向从切向转换。
        const double theta =
            TangentToVehicleHeading(std::atan2(vel.y(), vel.x()), direction);
        const double cos_th = std::cos(theta);
        const double sin_th = std::sin(theta);
        DensePointData dpd;
        dpd.u = ep.u;
        dpd.s = ep.s;
        dpd.position = pos;
        dpd.theta = theta;
        dpd.circles.reserve(outer_circle_num_);
        for (std::size_t k = 0; k < outer_circle_num_; ++k) {
            const Eigen::Vector2d& local = outer_circle_local_centers_[k];
            const Eigen::Vector2d center =
                pos + Eigen::Vector2d(cos_th * local.x() - sin_th * local.y(),
                                      sin_th * local.x() + cos_th * local.y());
            const auto [dist, grad] =
                esdf_map_.getDistAndGrad(center.x(), center.y());
            const double intrusion = std::max(
                0.0, outer_circle_radius_ + config_.collision_margin - dist);
            max_intrusion_depth = std::max(max_intrusion_depth, intrusion);
            CircleEsdfData ced;
            ced.local_offset = local;
            ced.center = center;
            ced.dist = dist;
            ced.grad = grad;
            dpd.circles.push_back(ced);
        }
        dense_points.push_back(std::move(dpd));
    }
    return max_intrusion_depth <= config_.collision_validation_tolerance;
}

BSplineSmoother::Result BSplineSmoother::smooth(
    const Maneuver& maneuver) const {
    Result result;
    if (maneuver.size() < 2) {
        if (maneuver.size() == 1) {
            return buildDegenerateResult(maneuver.points.front(),
                                         maneuver.points.front(),
                                         maneuver.direction);
        }
        result.success = true;
        result.degenerate = true;
        return result;
    }
    const TrajectoryPoint& start = maneuver.points.front();
    const TrajectoryPoint& end = maneuver.points.back();
    const double arc_length = maneuver.length();
    if (arc_length < config_.min_segment_arc_length_for_degradation) {
        return buildDegenerateResult(start, end, maneuver.direction);
    }
    const int control_point_count = determineControlPointCount(arc_length);
    if (control_point_count < 6) {
        return buildDegenerateResult(start, end, maneuver.direction);
    }
    const AnchorPoints anchors = buildAnchorPoints(
        maneuver, arc_length, control_point_count, maneuver.direction);
    std::vector<Eigen::Vector2d> ref_points;
    std::vector<Eigen::Vector2d> ref_normals;
    buildReferenceControlPoints(maneuver, control_point_count, ref_points,
                                ref_normals);

    // 短段（< 2.0m）放开 P₁/P_{N-1} 硬锚定：只保留 P₀/P_N 位置固定，
    // 起始与终止切向由优化器根据 F_data + F_smooth 自由选择。
    const bool free_p1 = (arc_length < 0.72);
    const bool free_pn1 = (arc_length < 0.72);
    // 若自由，扩展参考点以覆盖 P₁ 和/或 P_{N-1}
    if (free_p1) {
        // P₁ 的参考位置：锚定延伸点；法向垂直于 P₁_ref - P₀
        const Eigen::Vector2d p1_ref_dir = anchors.p1 - anchors.p0;
        const double p1_ref_len = p1_ref_dir.norm();
        Eigen::Vector2d p1_normal = Eigen::Vector2d::Zero();
        if (p1_ref_len > EPSILON) {
            p1_normal =
                Eigen::Vector2d(-p1_ref_dir.y(), p1_ref_dir.x()).normalized();
        }
        ref_points.insert(ref_points.begin(), anchors.p1);
        ref_normals.insert(ref_normals.begin(), p1_normal);
    }
    if (free_pn1) {
        const Eigen::Vector2d pn_ref_dir = anchors.pn - anchors.pn_1;
        const double pn_ref_len = pn_ref_dir.norm();
        Eigen::Vector2d pn_normal = Eigen::Vector2d::Zero();
        if (pn_ref_len > EPSILON) {
            pn_normal =
                Eigen::Vector2d(-pn_ref_dir.y(), pn_ref_dir.x()).normalized();
        }
        ref_points.push_back(anchors.pn_1);
        ref_normals.push_back(pn_normal);
    }

    // 构建自由控制点索引映射
    const int N = control_point_count - 1;
    std::vector<int> free_cp_indices;
    if (free_p1) free_cp_indices.push_back(1);
    for (int i = 2; i <= N - 2; ++i) free_cp_indices.push_back(i);
    if (free_pn1) free_cp_indices.push_back(N - 1);
    const int n_free = static_cast<int>(free_cp_indices.size());
    const std::vector<double> knot_vector =
        buildKnotVector(control_point_count);
    std::vector<BasisPack> basis_packs;
    precomputeBasisPacks(knot_vector, control_point_count, basis_packs);
    std::vector<Eigen::Vector2d> control_points(control_point_count);
    control_points[0] = anchors.p0;
    control_points[N] = anchors.pn;
    if (!free_p1) control_points[1] = anchors.p1;
    if (!free_pn1) control_points[N - 1] = anchors.pn_1;
    // 自由控制点从参考点初始化
    for (int j = 0; j < n_free; ++j) {
        control_points[free_cp_indices[j]] = ref_points[j];
    }
    // 若 P₁ 自由但之前已设置 ref_points[0]=anchors.p1，则 control_points[1]
    // 已正确初始化
    if (free_p1) control_points[1] = ref_points[0];
    if (free_pn1) control_points[N - 1] = ref_points.back();
    std::vector<Eigen::Vector2d> positions;
    std::vector<Eigen::Vector2d> velocities;
    evaluateFineGrid(control_points, basis_packs, positions, velocities);
    const auto initial_arc_length_table = buildArcLengthTable(positions);
    std::vector<EvalPoint> dense_eval_points = buildDenseEvalPoints(
        knot_vector, control_point_count, initial_arc_length_table);
    double total_chord = 0.0;
    for (int i = 0; i < control_point_count - 1; ++i) {
        total_chord += (control_points[i + 1] - control_points[i]).norm();
    }
    const double avg_chord_length =
        total_chord / static_cast<double>(control_point_count - 1);
    Eigen::VectorXd x(2 * n_free);
    for (int j = 0; j < n_free; ++j) {
        x[2 * j] = control_points[free_cp_indices[j]].x();
        x[2 * j + 1] = control_points[free_cp_indices[j]].y();
    }
    const Eigen::VectorXd initial_x = x;

    // 诊断：记录初始控制点的最大侵入深度，用于判断初始解质量。
    double init_intrusion = 0.0;
    {
        std::vector<Eigen::Vector2d> init_ctrl = control_points;
        for (int j = 0; j < n_free; ++j) {
            init_ctrl[free_cp_indices[j]] =
                Eigen::Vector2d(initial_x[2 * j], initial_x[2 * j + 1]);
        }
        std::vector<EvalPoint> init_dense = buildDenseEvalPoints(
            knot_vector, control_point_count, initial_arc_length_table);
        for (const auto& ep : init_dense) {
            Eigen::Vector2d pos = Eigen::Vector2d::Zero();
            Eigen::Vector2d vel = Eigen::Vector2d::Zero();
            for (std::size_t k = 0; k < ep.indices.size(); ++k) {
                pos += init_ctrl[ep.indices[k]] * ep.values[k];
                vel += init_ctrl[ep.indices[k]] * ep.d1[k];
            }
            const double theta = TangentToVehicleHeading(
                std::atan2(vel.y(), vel.x()), maneuver.direction);
            const double cth = std::cos(theta);
            const double sth = std::sin(theta);
            for (const auto& local : outer_circle_local_centers_) {
                const Eigen::Vector2d center =
                    pos + Eigen::Vector2d(cth * local.x() - sth * local.y(),
                                          sth * local.x() + cth * local.y());
                const double d = esdf_map_.getDist(center.x(), center.y());
                init_intrusion =
                    std::max(init_intrusion,
                             std::max(0.0, outer_circle_radius_ +
                                               config_.collision_margin - d));
            }
        }
        if (init_intrusion > config_.collision_validation_tolerance) {
            LOG_FMT_INFO(
                "BSplineSmoother maneuver initial max_intrusion_depth={:.4f}m",
                init_intrusion);
        }
    }

    // 预优化：仅用碰撞代价做少量梯度下降，快速将曲线推出障碍物。
    // 这对于初始路径已深陷障碍物的场景（如 Hybrid A* 粗分辨率导致的穿透）
    // 至关重要——先"逃生"再"塑形"。
    {
        // 构造仅含碰撞项的配置：清零所有非碰撞权重。
        auto collision_cfg = config_;
        collision_cfg.weight_data = 0.0;
        collision_cfg.weight_smooth_d1 = 0.0;
        collision_cfg.weight_smooth_d2 = 0.0;
        collision_cfg.weight_smooth_d3 = 0.0;
        collision_cfg.weight_reg = 0.0;
        BSplineObjective collision_only_fun{collision_cfg,
                                            esdf_map_,
                                            anchors,
                                            ref_points,
                                            ref_normals,
                                            basis_packs,
                                            dense_eval_points,
                                            outer_circle_local_centers_,
                                            outer_circle_radius_,
                                            control_point_count,
                                            avg_chord_length,
                                            maneuver.direction,
                                            free_p1,
                                            free_pn1,
                                            free_cp_indices};

        // 方向导数验证（仅首次迭代）：用有限差分确认解析梯度方向正确。
        // 只在比值显著偏离 -1.0 时告警，避免正常场景下的日志噪音。
        {
            Eigen::VectorXd grad_check;
            const double fx0 = collision_only_fun(x, grad_check);
            const double eps_fd = 1e-6;
            double dir_deriv_fd = 0.0;
            double dir_deriv_an = 0.0;
            for (int d = 0; d < x.size(); ++d) {
                const double orig = x[d];
                x[d] = orig + eps_fd;
                Eigen::VectorXd grad_dummy;
                const double fx_plus = collision_only_fun(x, grad_dummy);
                x[d] = orig;
                dir_deriv_fd += (fx_plus - fx0) / eps_fd * (-grad_check[d]);
                dir_deriv_an += (-grad_check[d]) * (-grad_check[d]);
            }
            const double ratio =
                (dir_deriv_an > 1e-12) ? (dir_deriv_fd / dir_deriv_an) : -1.0;
            if (std::abs(ratio + 1.0) > 0.5) {
                LOG_FMT_WARN(
                    "BSplineSmoother gradient check deviation: "
                    "ratio={:.4f} (expected -1.0), "
                    "fd={:.2f}, an={:.2f}",
                    ratio, dir_deriv_fd, dir_deriv_an);
            }
        }

        constexpr int pre_push_iters = 50;
        // 初始步长与侵入深度同量级，保证控制点有足够的位移空间。
        const double init_pre_step = std::max(0.1, init_intrusion * 2.0);
        double pre_step = init_pre_step;
        for (int iter = 0; iter < pre_push_iters; ++iter) {
            Eigen::VectorXd grad;
            // 用当前 control_points 更新 x 并求碰撞梯度
            for (int j = 0; j < n_free; ++j) {
                const int ci = free_cp_indices[j];
                x[2 * j] = control_points[ci].x();
                x[2 * j + 1] = control_points[ci].y();
            }
            const double fx = collision_only_fun(x, grad);
            // 梯度缩放：限制最大位移为 pre_step
            const double gnorm = grad.norm();
            if (gnorm < 1e-12) {
                // 仅在经过若干迭代后梯度归零才告警（表示收敛），
                // 初始即零表示路径已无碰撞，属正常情况。
                if (iter > 0) {
                    LOG_FMT_INFO(
                        "BSplineSmoother pre-push: grad norm ≈ 0 at iter {}, "
                        "stopping",
                        iter);
                }
                break;
            }
            const double alpha = std::min(pre_step / gnorm, 1.0);
            // 沿负梯度方向更新（碰撞代价下降 = 远离障碍物）
            x -= alpha * grad;
            // 回写到 control_points
            for (int j = 0; j < n_free; ++j) {
                control_points[free_cp_indices[j]] =
                    Eigen::Vector2d(x[2 * j], x[2 * j + 1]);
            }
            // 逐步减小步长，避免 overshoot
            pre_step *= 0.9;
        }
        if (pre_push_iters > 0 && pre_step < init_pre_step * 0.99) {
            LOG_FMT_INFO(
                "BSplineSmoother collision pre-push: {} iters, "
                "step {:.4f}m→{:.4f}m",
                pre_push_iters, init_pre_step, pre_step);
        }
    }
    // 碰撞预推后的检查点：L-BFGS 精修失败时回退到这里
    const Eigen::VectorXd prepushed_x = x;

    auto solveOptimization = [&](const BSplineSmootherConfig& cfg,
                                 int& out_iter) -> bool {
        BSplineObjective fun{cfg,
                             esdf_map_,
                             anchors,
                             ref_points,
                             ref_normals,
                             basis_packs,
                             dense_eval_points,
                             outer_circle_local_centers_,
                             outer_circle_radius_,
                             control_point_count,
                             avg_chord_length,
                             maneuver.direction,
                             free_p1,
                             free_pn1,
                             free_cp_indices};
        LBFGSpp::LBFGSParam<double> param;
        param.epsilon = cfg.lbfgs_epsilon;
        param.epsilon_rel = cfg.lbfgs_epsilon_rel;
        param.max_iterations = cfg.lbfgs_max_iterations;
        param.m = cfg.lbfgs_m;
        param.max_linesearch = cfg.lbfgs_max_linesearch;
        param.linesearch = cfg.lbfgs_linesearch_algo;
        param.ftol = cfg.lbfgs_ftol;
        param.wolfe = cfg.lbfgs_wolfe;

        double fx = 0.0;
        try {
            // 按线搜索算法分支：Armijo(1) / Wolfe(2) / Strong Wolfe(3) 使用
            // LineSearchBacktracking；默认 LineSearchNocedalWright 对应原始的
            // More-Thuente 线搜索（鲁棒性通常最好，作为兜底）。
            if (cfg.lbfgs_linesearch_algo ==
                    LBFGSpp::LBFGS_LINESEARCH_BACKTRACKING_ARMIJO ||
                cfg.lbfgs_linesearch_algo ==
                    LBFGSpp::LBFGS_LINESEARCH_BACKTRACKING_WOLFE ||
                cfg.lbfgs_linesearch_algo ==
                    LBFGSpp::LBFGS_LINESEARCH_BACKTRACKING_STRONG_WOLFE) {
                LBFGSpp::LBFGSSolver<double, LBFGSpp::LineSearchBacktracking>
                    solver(param);
                out_iter = solver.minimize(fun, x, fx);
            } else {
                LBFGSpp::LBFGSSolver<double> solver(param);
                out_iter = solver.minimize(fun, x, fx);
            }
            return true;
        } catch (const std::exception& e) {
            LOG_FMT_WARN("BSplineSmoother L-BFGS failed: {}", e.what());
            return false;
        }
    };
    int niter = 0;
    bool converged = solveOptimization(config_, niter);
    result.optimizer_converged = converged;
    if (!converged) {
        x = prepushed_x;
    }
    for (int j = 0; j < n_free; ++j) {
        control_points[free_cp_indices[j]] =
            Eigen::Vector2d(x[2 * j], x[2 * j + 1]);
    }
    evaluateFineGrid(control_points, basis_packs, positions, velocities);
    result.arc_length_table = buildArcLengthTable(positions);
    result.control_points = control_points;
    result.lbfgs_iterations = niter;
    std::vector<EvalPoint> final_dense_eval_points = buildDenseEvalPoints(
        knot_vector, control_point_count, result.arc_length_table);
    bool collision_ok = validateCollisionFree(
        control_points, final_dense_eval_points, result.dense_points,
        result.max_intrusion_depth, maneuver.direction);
    // 首次优化后侵入深度超阈值，尝试将碰撞权重翻倍后重新优化
    // 注意：从 initial_x 而非 prepushed_x 重新出发，后者叠加双倍权重易发散
    if (!collision_ok && config_.weight_collision > 0.0) {
        x = initial_x;
        BSplineSmootherConfig retry_config = config_;
        retry_config.weight_collision *= 2.0;
        int retry_iter = 0;
        if (solveOptimization(retry_config, retry_iter)) {
            result.optimizer_converged = true;
            for (int j = 0; j < n_free; ++j) {
                control_points[free_cp_indices[j]] =
                    Eigen::Vector2d(x[2 * j], x[2 * j + 1]);
            }
            evaluateFineGrid(control_points, basis_packs, positions,
                             velocities);
            result.arc_length_table = buildArcLengthTable(positions);
            result.control_points = control_points;
            result.lbfgs_iterations = niter + retry_iter;
            final_dense_eval_points = buildDenseEvalPoints(
                knot_vector, control_point_count, result.arc_length_table);
            collision_ok = validateCollisionFree(
                control_points, final_dense_eval_points, result.dense_points,
                result.max_intrusion_depth, maneuver.direction);
        }
    }
    result.success = collision_ok;
    return result;
}
}  // namespace apa_post_processor
