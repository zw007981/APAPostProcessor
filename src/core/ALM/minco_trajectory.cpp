#include "minco_trajectory.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace apa_post_processor {
namespace {
// 求值时刻允许的端点越界容差：吸收上游按总时长采样时产生的浮点尾差
constexpr double kTimeOvershootTolerance = 1e-9;
}  // namespace

void MincoTrajectory::setTrajectory(
    const MincoBoundaryCondition2d& start, const MincoBoundaryCondition2d& end,
    const std::vector<Eigen::Vector2d>& waypoints,
    const std::vector<double>& durations) {
    const int num_segments = static_cast<int>(durations.size());
    if (num_segments < 1) {
        throw std::invalid_argument("MincoTrajectory 至少需要一个多项式段");
    }
    if (static_cast<int>(waypoints.size()) != num_segments - 1) {
        throw std::invalid_argument("内部航点数量必须等于段数减 1");
    }
    for (const double duration : durations) {
        if (!std::isfinite(duration) || duration <= 0.0) {
            throw std::invalid_argument("段时长必须全部为正有限值");
        }
    }
    std::vector<BlockTridiagonalSolver::Block> lower;
    std::vector<BlockTridiagonalSolver::Block> diagonal;
    std::vector<BlockTridiagonalSolver::Block> upper;
    AssembleK(durations, lower, diagonal, upper);
    solver_.factorize(lower, diagonal, upper);
    // θ 与 s 两维共享同一 K(T) 分解，仅右端项不同
    coeffs_theta_ =
        solver_.solve(AssembleRhs(start.theta, end.theta, waypoints, 0));
    coeffs_s_ = solver_.solve(AssembleRhs(start.s, end.s, waypoints, 1));
    durations_ = durations;
    cumulative_durations_.assign(num_segments + 1, 0.0);
    for (int i = 0; i < num_segments; ++i) {
        cumulative_durations_[i + 1] = cumulative_durations_[i] + durations[i];
    }
}

Eigen::Vector2d MincoTrajectory::evaluate(double t, int order) const {
    checkEvaluable(order);
    const int segment_index = locateSegment(t);
    const double local_time =
        std::min(std::max(t - cumulative_durations_[segment_index], 0.0),
                 durations_[segment_index]);
    return evaluateSegment(segment_index, local_time, order);
}

Eigen::Vector2d MincoTrajectory::evaluateSegment(int segment_index,
                                                 double local_time,
                                                 int order) const {
    checkEvaluable(order);
    if (segment_index < 0 || segment_index >= numSegments()) {
        throw std::out_of_range("段索引越界");
    }
    const double segment_duration = durations_[segment_index];
    const double clamped_time =
        std::min(std::max(local_time, 0.0), segment_duration);
    const Eigen::Matrix<double, 1, COEFFS_PER_SEG> basis = DerivativeBasisRow(
        clamped_time / segment_duration, order, segment_duration);
    return {basis.dot(coeffs_theta_.col(segment_index)),
            basis.dot(coeffs_s_.col(segment_index))};
}

double MincoTrajectory::duration(int segment_index) const {
    return durations_.at(segment_index);
}

double MincoTrajectory::totalDuration() const {
    return cumulative_durations_.empty() ? 0.0 : cumulative_durations_.back();
}

double MincoTrajectory::finalArcLengthAdjointGradient(
    const CoeffMatrix& dJ_over_dcs) const {
    if (!solver_.isFactorized()) {
        throw std::logic_error("MincoTrajectory 尚未构建轨迹");
    }
    if (dJ_over_dcs.rows() != COEFFS_PER_SEG ||
        dJ_over_dcs.cols() != numSegments()) {
        throw std::invalid_argument("∂J/∂c_s 形状必须与系数矩阵一致");
    }
    // s_f 在右端项 b 中固定位于第 6M-3（0 基）个位置，即末块行第 3 行
    const CoeffMatrix adjoint = solver_.solveTranspose(dJ_over_dcs);
    return adjoint(3, numSegments() - 1);
}

MincoTrajectory::CoeffMatrix MincoTrajectory::solveAdjoint(
    const CoeffMatrix& rhs) const {
    if (!solver_.isFactorized()) {
        throw std::logic_error("MincoTrajectory 尚未构建轨迹");
    }
    return solver_.solveTranspose(rhs);
}

double MincoTrajectory::TauToDuration(double tau) {
    // 分段光滑双射：τ>0 对应 T>1，τ<=0 对应 T<=1，τ=0 处一阶连续可导；相比
    // 单纯指数映射，该形式在 T 较大时增长更缓、数值上更不容易溢出
    if (tau > 0.0) {
        const double shifted = tau + 1.0;
        return 0.5 * (shifted * shifted + 1.0);
    }
    const double shifted = 1.0 - tau;
    return 2.0 / (1.0 + shifted * shifted);
}

double MincoTrajectory::DurationToTau(double duration) {
    if (!std::isfinite(duration) || duration <= 0.0) {
        throw std::invalid_argument("段时长必须为正有限值");
    }
    if (duration > 1.0) {
        return std::sqrt(2.0 * duration - 1.0) - 1.0;
    }
    return 1.0 - std::sqrt(2.0 / duration - 1.0);
}

double MincoTrajectory::TauToDurationDerivative(double tau) {
    if (tau > 0.0) {
        return tau + 1.0;
    }
    const double shifted = 1.0 - tau;
    const double denom = 1.0 + shifted * shifted;
    return 4.0 * shifted / (denom * denom);
}

double MincoTrajectory::DurationToTauDerivative(double duration) {
    if (!std::isfinite(duration) || duration <= 0.0) {
        throw std::invalid_argument("段时长必须为正有限值");
    }
    if (duration > 1.0) {
        return 1.0 / std::sqrt(2.0 * duration - 1.0);
    }
    return 1.0 / (duration * duration * std::sqrt(2.0 / duration - 1.0));
}

void MincoTrajectory::AssembleK(
    const std::vector<double>& durations,
    std::vector<BlockTridiagonalSolver::Block>& lower,
    std::vector<BlockTridiagonalSolver::Block>& diagonal,
    std::vector<BlockTridiagonalSolver::Block>& upper) {
    const int num_segments = static_cast<int>(durations.size());
    lower.clear();
    diagonal.clear();
    upper.clear();
    diagonal.reserve(num_segments);
    if (num_segments > 1) {
        lower.reserve(num_segments - 1);
        upper.reserve(num_segments - 1);
    }
    for (int i = 0; i < num_segments; ++i) {
        const double segment_duration = durations[i];
        BlockTridiagonalSolver::Block diag =
            BlockTridiagonalSolver::Block::Zero();
        if (i == 0) {
            // 首段：起点 PVA + 本段末端位置/1/2 阶导数（M=1 时即终点 PVA）
            diag.row(0) = DerivativeBasisRow(0.0, 0, segment_duration);
            diag.row(1) = DerivativeBasisRow(0.0, 1, segment_duration);
            diag.row(2) = DerivativeBasisRow(0.0, 2, segment_duration);
            diag.row(3) = DerivativeBasisRow(1.0, 0, segment_duration);
            diag.row(4) = DerivativeBasisRow(1.0, 1, segment_duration);
            diag.row(5) = DerivativeBasisRow(1.0, 2, segment_duration);
        } else {
            // 其余段：与上一段的 3/4 阶导数连续性（负半部分）+ 两端位置 +
            // 末端 1/2 阶导数；该排列保证全部主对角块非奇异
            diag.row(0) = -DerivativeBasisRow(0.0, 3, segment_duration);
            diag.row(1) = -DerivativeBasisRow(0.0, 4, segment_duration);
            diag.row(2) = DerivativeBasisRow(0.0, 0, segment_duration);
            diag.row(3) = DerivativeBasisRow(1.0, 0, segment_duration);
            diag.row(4) = DerivativeBasisRow(1.0, 1, segment_duration);
            diag.row(5) = DerivativeBasisRow(1.0, 2, segment_duration);
        }
        diagonal.push_back(diag);
    }
    for (int i = 0; i + 1 < num_segments; ++i) {
        // 下对角块：上一段末端 3/4 阶导数（连续性的正半部分）
        BlockTridiagonalSolver::Block low =
            BlockTridiagonalSolver::Block::Zero();
        low.row(0) = DerivativeBasisRow(1.0, 3, durations[i]);
        low.row(1) = DerivativeBasisRow(1.0, 4, durations[i]);
        lower.push_back(low);
        // 上对角块：下一段起点 1/2 阶导数（连续性的负半部分）
        BlockTridiagonalSolver::Block up =
            BlockTridiagonalSolver::Block::Zero();
        up.row(4) = -DerivativeBasisRow(0.0, 1, durations[i + 1]);
        up.row(5) = -DerivativeBasisRow(0.0, 2, durations[i + 1]);
        upper.push_back(up);
    }
}

MincoTrajectory::CoeffMatrix MincoTrajectory::AssembleRhs(
    const MincoBoundaryCondition& start, const MincoBoundaryCondition& end,
    const std::vector<Eigen::Vector2d>& waypoints, int dim) {
    const int num_segments = static_cast<int>(waypoints.size()) + 1;
    CoeffMatrix rhs = CoeffMatrix::Zero(COEFFS_PER_SEG, num_segments);
    rhs(0, 0) = start.pos;
    rhs(1, 0) = start.vel;
    rhs(2, 0) = start.acc;
    if (num_segments == 1) {
        rhs(3, 0) = end.pos;
        rhs(4, 0) = end.vel;
        rhs(5, 0) = end.acc;
        return rhs;
    }
    // 首段末端位置 = 第一个内部航点
    rhs(3, 0) = waypoints[0](dim);
    // 中间段：起点位置 = 上一航点，末端位置 = 下一航点
    for (int i = 1; i + 1 < num_segments; ++i) {
        rhs(2, i) = waypoints[i - 1](dim);
        rhs(3, i) = waypoints[i](dim);
    }
    // 末段：起点位置 = 最后一个航点，末端为终点 PVA
    rhs(2, num_segments - 1) = waypoints[num_segments - 2](dim);
    rhs(3, num_segments - 1) = end.pos;
    rhs(4, num_segments - 1) = end.vel;
    rhs(5, num_segments - 1) = end.acc;
    return rhs;
}

Eigen::Matrix<double, 1, MincoTrajectory::COEFFS_PER_SEG>
MincoTrajectory::DerivativeBasisRow(double tau_norm, int order,
                                    double duration) {
    Eigen::Matrix<double, 1, COEFFS_PER_SEG> row =
        Eigen::Matrix<double, 1, COEFFS_PER_SEG>::Zero();
    // 实时间 order 阶导数基行：k >= order 时取
    // k!/(k-order)!·tau^(k-order)/T^order
    double falling = 1.0;
    for (int k = 1; k <= order; ++k) {
        falling *= k;
    }
    double tau_pow = 1.0;  // tau^(k-order)，k=order 时为 1
    const double duration_pow = std::pow(duration, order);
    for (int k = order; k < COEFFS_PER_SEG; ++k) {
        row[k] = falling * tau_pow / duration_pow;
        // 递推 k!/(k-order)! → (k+1)!/(k+1-order)!：乘以 (k+1)/(k+1-order)
        falling *=
            static_cast<double>(k + 1) / static_cast<double>(k + 1 - order);
        tau_pow *= tau_norm;
    }
    return row;
}

int MincoTrajectory::locateSegment(double t) const {
    const double total = totalDuration();
    if (t < -kTimeOvershootTolerance || t > total + kTimeOvershootTolerance) {
        throw std::out_of_range("求值时刻超出轨迹时间范围");
    }
    const double clamped = std::min(std::max(t, 0.0), total);
    // upper_bound 找到首个累计时刻严格大于 t 的位置，其前一段即所属段；
    // t 恰好落在段交接点时取后一段（连续性保证 0~4 阶导数取值一致）
    const auto it = std::upper_bound(cumulative_durations_.begin(),
                                     cumulative_durations_.end(), clamped);
    const int index =
        static_cast<int>(std::distance(cumulative_durations_.begin(), it)) - 1;
    return std::min(index, numSegments() - 1);
}

void MincoTrajectory::checkEvaluable(int order) const {
    if (!solver_.isFactorized()) {
        throw std::logic_error("MincoTrajectory 尚未构建轨迹");
    }
    if (order < 0 || order >= COEFFS_PER_SEG) {
        throw std::invalid_argument("求导阶数必须在 [0, 5] 内");
    }
}
}  // namespace apa_post_processor
