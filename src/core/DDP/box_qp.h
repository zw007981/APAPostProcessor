#pragma once

#include <Eigen/Core>
#include <array>
#include <cassert>
#include <cmath>
#include <stdexcept>

#include "ddp_reference_builder.h"

namespace apa_post_processor {
// 盒约束 QP（投影牛顿法）：min ½xᵀHx+qᵀx s.t. lb≤x≤ub，H
// 必须已含正则化，Cholesky 仅活动集变化时重做
template <int TDim = DDP_CONTROL_DIM>
class BoxQpSolver {
   public:
    using Vec = Eigen::Matrix<double, TDim, 1>;
    using Mat = Eigen::Matrix<double, TDim, TDim>;
    enum class Status {
        CONVERGED,
        MAX_ITERATIONS,
        NOT_POSITIVE_DEFINITE,
        LINE_SEARCH_FAILED
    };
    struct Problem {
        // 已含正则化的 H
        Mat hessian;
        // 梯度
        Vec gradient;
        // 盒下界
        Vec lower;
        // 盒上界
        Vec upper;
        // 内部自动投影进盒
        Vec initial;
        // 活动集热启动（相邻步 QP 复用）
        bool warm_start{false};
        // 热启动传入的初始钳制状态
        std::array<bool, TDim> initial_clamped{};
    };
    struct Result {
        // 求解状态
        Status status{Status::MAX_ITERATIONS};
        Vec x{Vec::Zero()};
        // Cholesky L，供 K_f = −H_ff⁻¹·Q_ux
        Mat free_factor{Mat::Zero()};
        std::array<int, TDim> free_indices{};
        // 自由维数
        int free_dim{0};
        // 最终活动集，供热启动
        std::array<bool, TDim> clamped{};
        // 迭代数
        int iterations{0};
        // 分解次数
        int factorizations{0};
        // 目标值
        double cost{0.0};
    };
    struct Options {
        // 迭代上限
        int max_iterations{100};
    };
    explicit BoxQpSolver(Options options = Options{}) : options_(options) {}
    Result solve(const Problem& problem) const {
        Validate(problem);
        Result result;
        Vec x = Clamp(problem.initial, problem.lower, problem.upper);
        std::array<bool, TDim> clamped =
            problem.warm_start ? problem.initial_clamped
                               : ClampingSet(x, Gradient(problem, x),
                                             problem.lower, problem.upper);
        bool factorize = true;
        Mat factor = Mat::Zero();
        std::array<int, TDim> free_indices{};
        int free_dim = 0;
        for (int iter = 1; iter <= options_.max_iterations; ++iter) {
            result.iterations = iter;
            free_dim = GatherFree(clamped, &free_indices);
            if (free_dim == 0) {
                // 全钳制退化：仅按梯度重判活动集稳定性，不做矩阵分解
                result.clamped = clamped;
                const std::array<bool, TDim> recomputed = ClampingSet(
                    x, Gradient(problem, x), problem.lower, problem.upper);
                if (recomputed == clamped) {
                    result.status = Status::CONVERGED;
                    break;
                }
                clamped = recomputed;
                factorize = true;
                continue;
            }
            if (factorize) {
                // 分解只在活动集变化（或本轮首次求解）时重做，否则复用旧因子
                if (!Cholesky(
                        GatherHessian(problem.hessian, free_indices, free_dim),
                        free_dim, &factor)) {
                    result.status = Status::NOT_POSITIVE_DEFINITE;
                    result.x = x;
                    result.clamped = clamped;
                    result.free_dim = 0;
                    result.free_factor.setZero();
                    result.cost = Objective(problem, x);
                    return result;
                }
                ++result.factorizations;
                factorize = false;
            }
            const Vec gradient = Gradient(problem, x);
            const Vec delta =
                NewtonStep(problem, x, factor, free_indices, free_dim, clamped);
            Vec candidate = x;
            bool full_step = true;
            if (delta.norm() > STEP_TOLERANCE * (1.0 + x.norm())) {
                // 牛顿步长不可忽略时才进线搜索；可忽略步长视为已在自由子空间
                // 最优点，跳过线搜索直接进入活动集重判（避免零分子零分母退化）
                if (!ArmijoSearch(problem, x, gradient, delta, &candidate,
                                  &full_step)) {
                    result.status = Status::LINE_SEARCH_FAILED;
                    result.x = x;
                    result.clamped = clamped;
                    result.free_dim = 0;
                    result.free_factor.setZero();
                    result.cost = Objective(problem, x);
                    return result;
                }
            }
            const std::array<bool, TDim> previous = clamped;
            if (full_step) {
                // 全步接受：按新点梯度符号重判钳制集——误钳的维（梯度指向盒内）
                // 在此被释放，这是错误活动集热启动仍收敛的关键
                clamped = ClampingSet(candidate, Gradient(problem, candidate),
                                      problem.lower, problem.upper);
            } else {
                // 部分步接受：投影必越过了某自由维边界，把本步被投影到边界的
                // 自由维并入钳制集后重解（Tassa 活动集更新规则）
                AddProjectedClamped(x, candidate, problem.lower, problem.upper,
                                    &clamped);
            }
            x = candidate;
            result.clamped = clamped;
            if (full_step && clamped == previous) {
                // 全牛顿步且活动集稳定：自由子空间已达最优且全部钳制维梯度
                // 指向盒外，KKT 条件满足
                result.status = Status::CONVERGED;
                break;
            }
            factorize = (clamped != previous);
        }
        result.x = x;
        result.free_factor = factor;
        result.free_indices = free_indices;
        result.free_dim = free_dim;
        result.cost = Objective(problem, x);
        return result;
    }
    // 以自由维分解求解致密右端并散布回满维：rhs 的前 free_dim 行按
    template <int Cols>
    static Eigen::Matrix<double, TDim, Cols> SolveFreeExpanded(
        const Result& result, const Eigen::Matrix<double, TDim, Cols>& rhs) {
        assert(result.status == Status::CONVERGED);
        Eigen::Matrix<double, TDim, Cols> expanded =
            Eigen::Matrix<double, TDim, Cols>::Zero();
        for (int c = 0; c < Cols; ++c) {
            Vec column = rhs.col(c);
            SolveCompact(result.free_factor, result.free_dim, &column);
            for (int i = 0; i < result.free_dim; ++i) {
                expanded(result.free_indices[i], c) = column(i);
            }
        }
        return expanded;
    }

   protected:
    // Armijo 充分下降系数 γ=0.1、折半回溯衰减与回溯次数上限（α 下限
    // 2⁻⁴⁰≈1e-12）
    static constexpr double ARMJO_GAMMA = 0.1;
    static constexpr double BACKTRACK_BETA = 0.5;
    static constexpr int MAX_BACKTRACKS = 40;
    // 牛顿步长收敛容差（相对解尺度）：步长低于该阈值视为已在自由子空间最优点
    static constexpr double STEP_TOLERANCE = 1e-12;
    // 输入契约校验：盒边界必须有限且逐维满足 lower ≤ upper
    static void Validate(const Problem& problem) {
        for (int j = 0; j < TDim; ++j) {
            if (!std::isfinite(problem.lower(j)) ||
                !std::isfinite(problem.upper(j)) ||
                problem.lower(j) > problem.upper(j)) {
                throw std::invalid_argument(
                    "BoxQpSolver: 盒边界必须有限且逐维满足 lower <= upper");
            }
        }
    }
    static double Objective(const Problem& problem, const Vec& x) {
        return 0.5 * x.dot(problem.hessian * x) + problem.gradient.dot(x);
    }
    static Vec Gradient(const Problem& problem, const Vec& x) {
        return problem.hessian * x + problem.gradient;
    }
    static Vec Clamp(const Vec& x, const Vec& lower, const Vec& upper) {
        return x.cwiseMax(lower).cwiseMin(upper);
    }
    // 钳制集判定：下界处梯度指向下（继续下降需越下界）或上界处梯度指向上；
    static std::array<bool, TDim> ClampingSet(const Vec& x, const Vec& gradient,
                                              const Vec& lower,
                                              const Vec& upper) {
        std::array<bool, TDim> clamped{};
        for (int j = 0; j < TDim; ++j) {
            const bool at_lower = x(j) == lower(j) && gradient(j) > 0.0;
            const bool at_upper = x(j) == upper(j) && gradient(j) < 0.0;
            clamped[j] = at_lower || at_upper || lower(j) >= upper(j);
        }
        return clamped;
    }
    // 收集自由维索引（保持升序），返回自由维数
    static int GatherFree(const std::array<bool, TDim>& clamped,
                          std::array<int, TDim>* free_indices) {
        int free_dim = 0;
        for (int j = 0; j < TDim; ++j) {
            if (!clamped[j]) {
                (*free_indices)[free_dim] = j;
                ++free_dim;
            }
        }
        return free_dim;
    }
    // 按自由维索引收集 Hessian 主子阵：紧凑约定——返回矩阵仅前
    static Mat GatherHessian(const Mat& hessian,
                             const std::array<int, TDim>& free_indices,
                             int free_dim) {
        Mat h_ff = Mat::Zero();
        for (int i = 0; i < free_dim; ++i) {
            for (int j = 0; j < free_dim; ++j) {
                h_ff(i, j) = hessian(free_indices[i], free_indices[j]);
            }
        }
        return h_ff;
    }
    // 紧凑 Cholesky 分解：对前 free_dim × free_dim 块求下三角因子 L
    // （Hff = L·Lᵀ）；对角残余非正即判定非正定返回 false
    static bool Cholesky(const Mat& h_ff, int free_dim, Mat* factor) {
        factor->setZero();
        for (int i = 0; i < free_dim; ++i) {
            for (int j = 0; j <= i; ++j) {
                double sum = h_ff(i, j);
                for (int k = 0; k < j; ++k) {
                    sum -= (*factor)(i, k) * (*factor)(j, k);
                }
                if (i != j) {
                    (*factor)(i, j) = sum / (*factor)(j, j);
                    continue;
                }
                if (sum <= 0.0) {
                    return false;
                }
                (*factor)(i, i) = std::sqrt(sum);
            }
        }
        return true;
    }
    // 紧凑因子求解 Hff·x = rhs：前代 L·y = rhs、回代 Lᵀ·x = y，
    // 仅前 free_dim 个元素参与运算（其余元素原地不动）
    static void SolveCompact(const Mat& factor, int free_dim, Vec* rhs) {
        for (int i = 0; i < free_dim; ++i) {
            double sum = (*rhs)(i);
            for (int k = 0; k < i; ++k) {
                sum -= factor(i, k) * (*rhs)(k);
            }
            (*rhs)(i) = sum / factor(i, i);
        }
        for (int i = free_dim - 1; i >= 0; --i) {
            double sum = (*rhs)(i);
            for (int k = i + 1; k < free_dim; ++k) {
                sum -= factor(k, i) * (*rhs)(k);
            }
            (*rhs)(i) = sum / factor(i, i);
        }
    }
    // 自由子空间牛顿方向：解 Hff·x_f⁺ = −(q_f + H_fc·x_c)，
    // 返回满维增量（自由维 Δx_f = x_f⁺ − x_f，钳制维恒为零）
    static Vec NewtonStep(const Problem& problem, const Vec& x,
                          const Mat& factor,
                          const std::array<int, TDim>& free_indices,
                          int free_dim, const std::array<bool, TDim>& clamped) {
        Vec rhs = Vec::Zero();
        for (int i = 0; i < free_dim; ++i) {
            const int fi = free_indices[i];
            double sum = problem.gradient(fi);
            for (int j = 0; j < TDim; ++j) {
                if (clamped[j]) {
                    sum += problem.hessian(fi, j) * x(j);
                }
            }
            rhs(i) = sum;
        }
        SolveCompact(factor, free_dim, &rhs);
        Vec delta = Vec::Zero();
        for (int i = 0; i < free_dim; ++i) {
            delta(free_indices[i]) = -rhs(i) - x(free_indices[i]);
        }
        return delta;
    }
    // Armijo 回溯线搜索：α 自 1 起折半，候选点逐元素投影进盒，直到满足充分
    bool ArmijoSearch(const Problem& problem, const Vec& x, const Vec& gradient,
                      const Vec& delta, Vec* candidate, bool* full_step) const {
        const double f0 = Objective(problem, x);
        double alpha = 1.0;
        for (int bt = 0; bt < MAX_BACKTRACKS; ++bt) {
            *candidate = Clamp(x + alpha * delta, problem.lower, problem.upper);
            if ((*candidate - x).isZero(0.0)) {
                *full_step = true;
                return true;
            }
            const double numerator = f0 - Objective(problem, *candidate);
            const double denominator = gradient.dot(x - *candidate);
            if (denominator > 0.0 && numerator > ARMJO_GAMMA * denominator) {
                *full_step = (bt == 0);
                return true;
            }
            alpha *= BACKTRACK_BETA;
        }
        return false;
    }
    // 部分步后的活动集更新：把本步被投影到盒边界的维并入钳制集
    static void AddProjectedClamped(const Vec& before, const Vec& after,
                                    const Vec& lower, const Vec& upper,
                                    std::array<bool, TDim>* clamped) {
        for (int j = 0; j < TDim; ++j) {
            const bool moved = before(j) != after(j);
            const bool at_bound = after(j) == lower(j) || after(j) == upper(j);
            if (moved && at_bound) {
                (*clamped)[j] = true;
            }
        }
    }

   protected:
    Options options_;
};
}  // namespace apa_post_processor
