#include "speed_profile_planner.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

#include "../core/osqp_solver.h"
#include "../util/constants.h"
#include "../util/logger.h"

namespace apa_post_processor {
namespace {
// 将 (row, col, value) 三元组追加到 CSC 矩阵的当前列
void appendCscEntry(std::vector<double>& values, std::vector<int>& row_indices,
                    int row, double value) {
    values.push_back(value);
    row_indices.push_back(row);
}
}  // namespace

SpeedProfilePlanner::SpeedProfilePlanner(
    const SpeedProfilePlannerConfig& config)
    : config_(config) {
    if (config_.max_v_forward <= 0.0 || config_.max_v_reverse <= 0.0 ||
        config_.weight_v_ref < 0.0 || config_.weight_a_sq < 0.0 ||
        config_.weight_jerk_sq < 0.0 ||
        config_.time_reintegration_epsilon <= 0.0 ||
        config_.max_lateral_accel <= 0.0 || config_.esdf_danger_margin < 0.0) {
        throw std::invalid_argument(
            "SpeedProfilePlannerConfig contains invalid values");
    }
}

void SpeedProfilePlanner::validateInputs(
    const SpeedProfileInput& input, const VehicleParams& vehicle_params,
    const std::vector<int>& direction_signs,
    const std::vector<std::size_t>& cusp_indices) const {
    const auto n_points = input.s.size();
    if (n_points < 2) {
        throw std::invalid_argument(
            "SpeedProfilePlanner requires at least 2 points");
    }
    if (input.kappa.size() != n_points ||
        input.min_esdf_dist.size() != n_points ||
        direction_signs.size() != n_points) {
        throw std::invalid_argument(
            "SpeedProfilePlanner input vectors must have the same size");
    }
    for (std::size_t i = 0; i < n_points; ++i) {
        if (direction_signs[i] != 1 && direction_signs[i] != -1) {
            throw std::invalid_argument(
                "SpeedProfilePlanner direction signs must be +1 or -1");
        }
    }
    if (vehicle_params.max_accel <= 0.0 || vehicle_params.max_decel >= 0.0) {
        throw std::invalid_argument(
            "VehicleParams max_accel must be positive and max_decel must be "
            "negative for SpeedProfilePlanner");
    }
    for (std::size_t i = 1; i < n_points; ++i) {
        if (input.s[i] < input.s[i - 1] - EPSILON) {
            throw std::invalid_argument(
                "SpeedProfilePlanner input arc length s must be "
                "non-decreasing");
        }
    }
    // 尖点必须严格在内部
    for (const auto idx : cusp_indices) {
        if (idx == 0 || idx >= n_points - 1) {
            throw std::invalid_argument(
                "SpeedProfilePlanner cusp index must be strictly interior "
                "(0 < idx < n_points - 1)");
        }
    }
}

std::vector<double> SpeedProfilePlanner::computeSpeedLimitSquared(
    const SpeedProfileInput& input,
    const std::vector<int>& direction_signs) const {
    const auto n_points = input.s.size();
    std::vector<double> v_limit_sq;
    v_limit_sq.reserve(n_points);

    for (std::size_t i = 0; i < n_points; ++i) {
        const double v_ref = (direction_signs[i] > 0) ? config_.max_v_forward
                                                      : config_.max_v_reverse;
        double v_ref_sq = v_ref * v_ref;

        // 曲率侧向加速度约束
        double v_kappa_sq = std::numeric_limits<double>::infinity();
        const double kappa_abs = std::abs(input.kappa[i]);
        if (kappa_abs > EPSILON) {
            v_kappa_sq = config_.max_lateral_accel / kappa_abs;
        }

        // ESDF 危险度约束
        double v_esdf_sq = v_ref_sq;
        const double dist = input.min_esdf_dist[i];
        if (dist < config_.esdf_danger_margin) {
            const double ratio =
                std::max(0.0, dist / config_.esdf_danger_margin);
            v_esdf_sq = v_ref_sq * ratio;
        }

        double limit_sq = std::min({v_ref_sq, v_kappa_sq, v_esdf_sq});
        limit_sq = std::max(limit_sq, 0.0);
        v_limit_sq.push_back(limit_sq);
    }

    return v_limit_sq;
}

SpeedProfileResult SpeedProfilePlanner::solveQp(
    const SpeedProfileInput& input, const VehicleParams& vehicle_params,
    const std::vector<int>& direction_signs,
    const std::vector<std::size_t>& cusp_indices,
    double initial_velocity) const {
    const int n_points = static_cast<int>(input.s.size());
    const int n = 2 * n_points;  // [b_0..b_N, a_0..a_N]
    const int n_intervals = n_points - 1;

    // 变量索引辅助函数
    const auto b_index = [n_points](int i) { return i; };
    const auto a_index = [n_points](int i) { return n_points + i; };

    // 目标函数 P (CSC 上三角) 与线性项 q：两遍法保证 CSC 列指针正确
    std::vector<double> P_x;
    std::vector<int> P_i;
    std::vector<int> P_p(n + 1, 0);
    std::vector<double> q(n, 0.0);

    // 先计算每列的非零元素数量，构造 P_p
    std::vector<int> col_nnz(n, 0);
    for (int i = 0; i < n_points; ++i) {
        // b_i 对角
        ++col_nnz[b_index(i)];
        // a_i 对角
        ++col_nnz[a_index(i)];
        // a_i 与 a_{i+1} 的交叉项落在 a_{i+1} 列
        if (i < n_intervals) {
            ++col_nnz[a_index(i + 1)];
        }
    }
    P_p[0] = 0;
    for (int col = 0; col < n; ++col) {
        P_p[col + 1] = P_p[col] + col_nnz[col];
    }
    P_x.reserve(P_p[n]);
    P_i.reserve(P_p[n]);

    // 按列填充 P：必须严格按列号递增顺序（先 b 列，再 a 列）
    for (int i = 0; i < n_points; ++i) {
        const double v_ref = (direction_signs[i] > 0) ? config_.max_v_forward
                                                      : config_.max_v_reverse;
        const double v_ref_sq = v_ref * v_ref;

        // b_i 列：对角项
        const int b_col = b_index(i);
        appendCscEntry(P_x, P_i, b_col, 2.0 * config_.weight_v_ref);
        q[b_col] = -2.0 * config_.weight_v_ref * v_ref_sq;
    }

    for (int i = 0; i < n_points; ++i) {
        // a_i 列：与 a_{i-1} 的交叉项（若存在），然后对角项
        const int a_col = a_index(i);
        if (i > 0) {
            appendCscEntry(P_x, P_i, a_index(i - 1),
                           -2.0 * config_.weight_jerk_sq);
        }
        double a_diag = 2.0 * config_.weight_a_sq;
        if (i > 0) {
            a_diag += 2.0 * config_.weight_jerk_sq;
        }
        if (i < n_intervals) {
            a_diag += 2.0 * config_.weight_jerk_sq;
        }
        appendCscEntry(P_x, P_i, a_col, a_diag);
    }

    // 约束矩阵 A (CSC) 与边界 l/u：按列收集后排序输出 CSC
    int row = 0;
    const int row_b0 = row++;
    const int row_bN = row++;
    const int row_a0 = row++;
    const int row_aN = row++;
    const int row_dynamics_start = row;
    row += n_intervals;
    const int row_cusp_start = row;
    const int cusp_count = static_cast<int>(cusp_indices.size());
    row += cusp_count;
    const int row_b_bounds_start = row;
    row += n_points;
    const int row_a_bounds_start = row;
    row += n_points;
    const bool has_jerk_constraint = config_.max_jerk_proxy > 0.0;
    const int row_jerk_start = row;
    if (has_jerk_constraint) {
        row += n_intervals;
    }
    const int m = row;

    std::vector<double> l(m, 0.0);
    std::vector<double> u(m, 0.0);

    // 边界条件（等式约束 l = u）
    const double b0_init = initial_velocity * initial_velocity;
    l[row_b0] = b0_init;
    u[row_b0] = b0_init;
    l[row_bN] = 0.0;
    u[row_bN] = 0.0;
    l[row_a0] = 0.0;
    u[row_a0] = 0.0;
    l[row_aN] = 0.0;
    u[row_aN] = 0.0;

    // b 上下界
    const auto v_limit_sq = computeSpeedLimitSquared(input, direction_signs);
    for (int i = 0; i < n_points; ++i) {
        const int r = row_b_bounds_start + i;
        l[r] = 0.0;
        u[r] = v_limit_sq[i];
    }

    // a 上下界
    for (int i = 0; i < n_points; ++i) {
        const int r = row_a_bounds_start + i;
        l[r] = vehicle_params.max_decel;
        u[r] = vehicle_params.max_accel;
    }

    // jerk 代理上下界
    if (has_jerk_constraint) {
        for (int i = 0; i < n_intervals; ++i) {
            const double ds = input.s[i + 1] - input.s[i];
            const double bound = config_.max_jerk_proxy * ds;
            const int r = row_jerk_start + i;
            l[r] = -bound;
            u[r] = bound;
        }
    }

    // ---- 约束矩阵 A：按列收集条目，再整体排序输出 CSC ----
    std::vector<std::vector<std::pair<int, double>>> a_entries(n);
    for (int col = 0; col < n; ++col) {
        // b 列最多 5 个条目（bound + 两段动力学 + 端点/尖点），
        // a 列最多 7 个条目（bound + 两段动力学 + 两段 jerk + 端点）。
        a_entries[col].reserve(col < n_points ? 5 : 7);
    }

    // b0
    a_entries[b_index(0)].push_back({row_b0, 1.0});
    // bN
    a_entries[b_index(n_points - 1)].push_back({row_bN, 1.0});
    // a0
    a_entries[a_index(0)].push_back({row_a0, 1.0});
    // aN
    a_entries[a_index(n_points - 1)].push_back({row_aN, 1.0});

    // dynamics: b_{i+1} - b_i - 2*ds_i*a_i = 0
    for (int i = 0; i < n_intervals; ++i) {
        const double ds = input.s[i + 1] - input.s[i];
        const int r = row_dynamics_start + i;
        a_entries[b_index(i)].push_back({r, -1.0});       // b_i
        a_entries[b_index(i + 1)].push_back({r, 1.0});    // b_{i+1}
        a_entries[a_index(i)].push_back({r, -2.0 * ds});  // a_i
    }

    // cusp: b_k = 0
    for (int ci = 0; ci < cusp_count; ++ci) {
        const int idx = static_cast<int>(cusp_indices[ci]);
        const int r = row_cusp_start + ci;
        a_entries[b_index(idx)].push_back({r, 1.0});
    }

    // b bounds
    for (int i = 0; i < n_points; ++i) {
        const int r = row_b_bounds_start + i;
        a_entries[b_index(i)].push_back({r, 1.0});
    }

    // a bounds
    for (int i = 0; i < n_points; ++i) {
        const int r = row_a_bounds_start + i;
        a_entries[a_index(i)].push_back({r, 1.0});
    }

    // jerk: a_{i+1} - a_i
    if (has_jerk_constraint) {
        for (int i = 0; i < n_intervals; ++i) {
            const int r = row_jerk_start + i;
            a_entries[a_index(i)].push_back({r, -1.0});     // a_i
            a_entries[a_index(i + 1)].push_back({r, 1.0});  // a_{i+1}
        }
    }

    // 将按列收集的条目排序并输出 CSC
    std::vector<double> A_x;
    std::vector<int> A_i;
    A_x.reserve(7 * n);
    A_i.reserve(7 * n);
    std::vector<int> A_p(n + 1, 0);
    for (int col = 0; col < n; ++col) {
        auto& entries = a_entries[col];
        std::sort(entries.begin(), entries.end(),
                  [](const std::pair<int, double>& lhs,
                     const std::pair<int, double>& rhs) {
                      return lhs.first < rhs.first;
                  });
        A_p[col] = static_cast<int>(A_x.size());
        for (const auto& [row_idx, val] : entries) {
            appendCscEntry(A_x, A_i, row_idx, val);
        }
    }
    A_p[n] = static_cast<int>(A_x.size());

    // 求解
    OsqpSolverConfig osqp_config;
    osqp_config.verbose = false;
    osqp_config.max_iter = 4000;
    osqp_config.eps_abs = 1e-4;
    osqp_config.eps_rel = 1e-4;

    OsqpSolver solver;
    SpeedProfileResult result;
    if (!solver.setup(n, m, P_x, P_i, P_p, q, A_x, A_i, A_p, l, u,
                      osqp_config)) {
        result.status_msg = "OSQP setup failed";
        return result;
    }

    const auto osqp_result = solver.solve();
    result.status_msg = osqp_result.status_msg;
    result.solver_iterations = osqp_result.iter;

    if (osqp_result.status != OsqpStatus::Solved &&
        osqp_result.status != OsqpStatus::SolvedInaccurate) {
        return result;
    }

    result.success = true;
    result.b.assign(osqp_result.x.begin(), osqp_result.x.begin() + n_points);
    result.a.assign(osqp_result.x.begin() + n_points, osqp_result.x.end());

    return result;
}

void SpeedProfilePlanner::recoverVelocityAndTime(
    const std::vector<double>& b, const std::vector<int>& direction_signs,
    const std::vector<double>& s, SpeedProfileResult& result) const {
    const auto n_points = b.size();
    result.v.resize(n_points);
    result.t.resize(n_points);

    for (std::size_t i = 0; i < n_points; ++i) {
        const double speed = std::sqrt(std::max(b[i], 0.0));
        result.v[i] = static_cast<double>(direction_signs[i]) * speed;
        // 加速度的符号与运动方向一致
        result.a[i] *= static_cast<double>(direction_signs[i]);
    }

    result.t[0] = 0.0;
    for (std::size_t i = 0; i + 1 < n_points; ++i) {
        const double ds = s[i + 1] - s[i];
        const double denom =
            std::max(std::abs(result.v[i]) + std::abs(result.v[i + 1]),
                     config_.time_reintegration_epsilon);
        result.t[i + 1] = result.t[i] + 2.0 * ds / denom;
    }
}

SpeedProfileResult SpeedProfilePlanner::plan(
    const SpeedProfileInput& input, const VehicleParams& vehicle_params,
    const std::vector<int>& direction_signs,
    const std::vector<std::size_t>& cusp_indices,
    double initial_velocity) const {
    validateInputs(input, vehicle_params, direction_signs, cusp_indices);

    auto result = solveQp(input, vehicle_params, direction_signs, cusp_indices,
                          initial_velocity);
    if (!result.success) {
        return result;
    }

    recoverVelocityAndTime(result.b, direction_signs, input.s, result);
    return result;
}
}  // namespace apa_post_processor
