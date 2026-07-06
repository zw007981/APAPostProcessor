#include "bspline_smoother.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include "../util/logger.h"

namespace apa_post_processor {
namespace {
// NURBS Book 算法 A2.3：一次性计算 span 内全部非零 B 样条基函数
// N_{span-p+r, p}(u)（r=0..p）的 0 到 n 阶导数。
//
// 输出 ders[k][r] 表示上述第 r 个非零基函数的 k 阶导数；k=0 时为函数值。
// 算法核心是用 ndu 表（三角递推表）代替直接递归，原因有二：
//   1. 避免递归求导在 clamped 端点（重复节点）处出现 0/0 的极限问题；
//   2. 通过保存中间商 ndu[r][j] 一次性得到所有阶导数，时间复杂度 O(p^2 + p*n)。
void computeBasisDerivatives(double u, int span, int p, int n,
                             const std::vector<double>& knots,
                             std::vector<std::vector<double>>& ders) {
    ders.assign(n + 1, std::vector<double>(p + 1, 0.0));
    // ndu[j][r] = 节点差分商，对应 NURBS Book 中三角表的第 j 行第 r 列。
    // 它同时承担基函数值（对角线 ndu[j][j]）与导数递推分母的双重角色。
    std::vector<std::vector<double>> ndu(p + 1,
                                         std::vector<double>(p + 1, 0.0));
    // left[j] / right[j] 记录 u 到 span 左右两侧第 j 个相关节点的距离，
    // 用于 De Boor-Cox 递推中的线性组合权重。
    std::vector<double> left(p + 1, 0.0);
    std::vector<double> right(p + 1, 0.0);

    // ---- 第一阶段：构造 ndu 三角表 ----
    // 第 0 行初始化为 1，表示 0 阶基函数在 u 处的“种子”。
    ndu[0][0] = 1.0;
    for (int j = 1; j <= p; ++j) {
        left[j] = u - knots[span + 1 - j];
        right[j] = knots[span + j] - u;
        double saved = 0.0;
        // 利用 De Boor-Cox 公式，由 j-1 阶递推到 j 阶。
        // 当节点重复时（如 clamped 端点 u=0 或 u=1），left/right 中会出现 0，
        // 但 ndu[j][r] = right[r+1] + left[j-r] 作为分母仍保持正确极限值，
        // 因此不会像直接公式那样出现 0/0。
        for (int r = 0; r < j; ++r) {
            ndu[j][r] = right[r + 1] + left[j - r];
            const double temp = ndu[r][j - 1] / ndu[j][r];
            ndu[r][j] = saved + right[r + 1] * temp;
            saved = left[j - r] * temp;
        }
        ndu[j][j] = saved;
    }

    // ---- 第二阶段：提取 0 阶基函数值 ----
    // ndu 表对角线 ndu[j][p] 正好是 p 阶基函数 N_{span-p+j, p}(u) 的值。
    for (int j = 0; j <= p; ++j) {
        ders[0][j] = ndu[j][p];
    }

    // ---- 第三阶段：递推计算 1..n 阶导数 ----
    // a[s][k] 是导数递推中的辅助系数数组，s1/s2 两个下标交替复用两行，
    // 避免每次 k 循环都重新分配内存（滚动数组技巧）。
    std::vector<std::vector<double>> a(2, std::vector<double>(p + 1, 0.0));
    for (int r = 0; r <= p; ++r) {
        int s1 = 0;
        int s2 = 1;
        a[0][0] = 1.0;
        for (int k = 1; k <= n; ++k) {
            double d = 0.0;
            const int rk = r - k;
            const int pk = p - k;
            // 左侧边界项：仅当 r >= k 时存在。
            if (r >= k) {
                a[s2][0] = a[s1][0] / ndu[pk + 1][rk];
                d = a[s2][0] * ndu[rk][pk];
            }
            // 中间项：利用相邻系数差分递推。
            const int j1 = (r >= k - 1) ? 1 : k - r;
            const int j2 = (r <= pk) ? k - 1 : p - r;
            for (int j = j1; j <= j2; ++j) {
                a[s2][j] = (a[s1][j] - a[s1][j - 1]) / ndu[pk + 1][rk + j];
                d += a[s2][j] * ndu[rk + j][pk];
            }
            // 右侧边界项：仅当 r <= p-k 时存在。
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

// L-BFGS目标函数：在内部控制点空间上构建并解析求导F_data+F_smooth+F_collision+F_reg。
// 采用Frozen-theta策略：前向求值使用当前theta，反向梯度将theta视为常量。
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

    Scalar operator()(const Vector& x, Vector& grad) {
        const int internal_count = control_point_count - 4;
        const int N = control_point_count - 1;
        std::vector<Eigen::Vector2d> control_points(control_point_count);
        control_points[0] = anchors.p0;
        control_points[1] = anchors.p1;
        control_points[N - 1] = anchors.pn_1;
        control_points[N] = anchors.pn;
        for (int i = 2; i <= N - 2; ++i) {
            control_points[i] =
                Eigen::Vector2d(x[2 * (i - 2)], x[2 * (i - 2) + 1]);
        }
        // F_data：仅惩罚内部控制点相对参考点的法向漂移。
        double f_data = 0.0;
        std::vector<Eigen::Vector2d> data_grad(control_point_count,
                                               Eigen::Vector2d::Zero());
        for (int i = 2; i <= N - 2; ++i) {
            const Eigen::Vector2d delta = control_points[i] - ref_points[i - 2];
            const double proj = delta.dot(ref_normals[i - 2]);
            f_data += proj * proj;
            data_grad[i] = 2.0 * proj * ref_normals[i - 2];
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
        double f_collision = 0.0;
        std::vector<Eigen::Vector2d> collision_grad(control_point_count,
                                                    Eigen::Vector2d::Zero());
        for (const auto& ep : dense_eval_points) {
            Eigen::Vector2d pos = Eigen::Vector2d::Zero();
            Eigen::Vector2d vel = Eigen::Vector2d::Zero();
            for (std::size_t k = 0; k < ep.indices.size(); ++k) {
                const int idx = ep.indices[k];
                pos += control_points[idx] * ep.values[k];
                vel += control_points[idx] * ep.d1[k];
            }
            const double theta = std::atan2(vel.y(), vel.x());
            const double cos_th = std::cos(theta);
            const double sin_th = std::sin(theta);
            for (std::size_t k = 0; k < outer_circle_local_centers.size();
                 ++k) {
                const Eigen::Vector2d& local = outer_circle_local_centers[k];
                const Eigen::Vector2d center =
                    pos +
                    Eigen::Vector2d(cos_th * local.x() - sin_th * local.y(),
                                    sin_th * local.x() + cos_th * local.y());
                const auto [dist, grad_esdf] =
                    esdf_map.getDistAndGrad(center.x(), center.y());
                const double intrusion = std::max(
                    0.0, outer_circle_radius + config.collision_margin - dist);
                if (intrusion <= 0.0) {
                    continue;
                }
                f_collision += intrusion * intrusion * intrusion;
                const double coeff = -3.0 * intrusion * intrusion;
                for (std::size_t kk = 0; kk < ep.indices.size(); ++kk) {
                    collision_grad[ep.indices[kk]] +=
                        coeff * grad_esdf * ep.values[kk];
                }
            }
        }
        const double total = config.weight_data * f_data + f_smooth +
                             config.weight_collision * f_collision +
                             config.weight_reg * f_reg;
        grad.resize(2 * internal_count);
        grad.setZero();
        for (int i = 2; i <= N - 2; ++i) {
            const Eigen::Vector2d g =
                config.weight_data * data_grad[i] + smooth_grad[i] +
                config.weight_collision * collision_grad[i] +
                config.weight_reg * reg_grad[i];
            grad[2 * (i - 2)] = g.x();
            grad[2 * (i - 2) + 1] = g.y();
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
        config_.weight_reg < 0.0 || config_.collision_margin < 0.0 ||
        config_.anchor_extension_length < 0.0 ||
        config_.min_segment_arc_length_for_degradation < 0.0 ||
        config_.collision_validation_tolerance < 0.0 ||
        config_.lbfgs_max_iterations <= 0 || config_.lbfgs_epsilon <= 0.0 ||
        config_.arc_length_table_step <= 0.0 ||
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
    const int internal_count = std::max(
        2, static_cast<int>(std::ceil(arc_length / config_.dense_step_dist)));
    return internal_count + 4;
}

BSplineSmoother::AnchorPoints BSplineSmoother::buildAnchorPoints(
    const PathPoint& start, const PathPoint& end, double arc_length,
    int control_point_count) const {
    // 为避免锚定点越过相邻内部控制点导致控制多边形折叠，
    // 将实际延伸步长限制在相邻内部控制点间距的一半以下。
    const int N_c = control_point_count - 1;
    const double internal_spacing =
        (N_c > 4) ? (arc_length / static_cast<double>(N_c)) : arc_length;
    const double max_extension = std::max(
        1e-3,
        std::min(config_.anchor_extension_length, 0.5 * internal_spacing));
    AnchorPoints anchors;
    anchors.p0 = Eigen::Vector2d(start.x, start.y);
    anchors.p1 =
        anchors.p0 + max_extension * Eigen::Vector2d(std::cos(start.theta),
                                                     std::sin(start.theta));
    anchors.pn = Eigen::Vector2d(end.x, end.y);
    anchors.pn_1 =
        anchors.pn - max_extension * Eigen::Vector2d(std::cos(end.theta),
                                                     std::sin(end.theta));
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
        dense_points.push_back(std::move(ep));
    }
    return dense_points;
}

BSplineSmoother::Result BSplineSmoother::buildDegenerateResult(
    const PathPoint& start, const PathPoint& end) const {
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
    const double theta = std::atan2(delta.y(), delta.x());
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
    std::vector<DensePointData>& dense_points,
    double& max_intrusion_depth) const {
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
        const double theta = std::atan2(vel.y(), vel.x());
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
                                         maneuver.points.front());
        }
        result.success = true;
        result.degenerate = true;
        return result;
    }
    const PathPoint& start = maneuver.points.front();
    const PathPoint& end = maneuver.points.back();
    const double arc_length = maneuver.length();
    if (arc_length < config_.min_segment_arc_length_for_degradation) {
        return buildDegenerateResult(start, end);
    }
    const int control_point_count = determineControlPointCount(arc_length);
    if (control_point_count < 6) {
        return buildDegenerateResult(start, end);
    }
    const AnchorPoints anchors =
        buildAnchorPoints(start, end, arc_length, control_point_count);
    std::vector<Eigen::Vector2d> ref_points;
    std::vector<Eigen::Vector2d> ref_normals;
    buildReferenceControlPoints(maneuver, control_point_count, ref_points,
                                ref_normals);
    const std::vector<double> knot_vector =
        buildKnotVector(control_point_count);
    std::vector<BasisPack> basis_packs;
    precomputeBasisPacks(knot_vector, control_point_count, basis_packs);
    std::vector<Eigen::Vector2d> control_points(control_point_count);
    control_points[0] = anchors.p0;
    control_points[1] = anchors.p1;
    control_points[control_point_count - 2] = anchors.pn_1;
    control_points[control_point_count - 1] = anchors.pn;
    for (int i = 2; i <= control_point_count - 3; ++i) {
        control_points[i] = ref_points[i - 2];
    }
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
    const int internal_count = control_point_count - 4;
    Eigen::VectorXd x(2 * internal_count);
    for (int i = 2; i <= control_point_count - 3; ++i) {
        x[2 * (i - 2)] = control_points[i].x();
        x[2 * (i - 2) + 1] = control_points[i].y();
    }
    const Eigen::VectorXd initial_x = x;
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
                             avg_chord_length};
        LBFGSpp::LBFGSParam<double> param;
        param.epsilon = cfg.lbfgs_epsilon;
        param.max_iterations = cfg.lbfgs_max_iterations;
        LBFGSpp::LBFGSSolver<double> solver(param);
        double fx = 0.0;
        try {
            out_iter = solver.minimize(fun, x, fx);
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
        x = initial_x;
    }
    for (int i = 2; i <= control_point_count - 3; ++i) {
        control_points[i] = Eigen::Vector2d(x[2 * (i - 2)], x[2 * (i - 2) + 1]);
    }
    evaluateFineGrid(control_points, basis_packs, positions, velocities);
    result.arc_length_table = buildArcLengthTable(positions);
    result.control_points = control_points;
    result.lbfgs_iterations = niter;
    std::vector<EvalPoint> final_dense_eval_points = buildDenseEvalPoints(
        knot_vector, control_point_count, result.arc_length_table);
    bool collision_ok =
        validateCollisionFree(control_points, final_dense_eval_points,
                              result.dense_points, result.max_intrusion_depth);
    // 若首次优化后侵入深度超阈值，尝试将碰撞权重翻倍后重新优化一次。
    if (!collision_ok && config_.weight_collision > 0.0) {
        x = initial_x;
        BSplineSmootherConfig retry_config = config_;
        retry_config.weight_collision *= 2.0;
        int retry_iter = 0;
        if (solveOptimization(retry_config, retry_iter)) {
            result.optimizer_converged = true;
            for (int i = 2; i <= control_point_count - 3; ++i) {
                control_points[i] =
                    Eigen::Vector2d(x[2 * (i - 2)], x[2 * (i - 2) + 1]);
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
                result.max_intrusion_depth);
        }
    }
    result.success = collision_ok;
    return result;
}
}  // namespace apa_post_processor
