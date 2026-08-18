#include <gtest/gtest.h>

#include <Eigen/Cholesky>
#include <Eigen/LU>
#include <cmath>
#include <limits>
#include <random>
#include <vector>

#include "core/iLQR/box_qp.h"

namespace apa_post_processor {
namespace {

using QpSolver = BoxQpSolver<>;
using Vec2 = QpSolver::Vec;
using Mat2 = QpSolver::Mat;

// 暴力枚举参照求解器：对每维枚举 {自由, 钳制在下界, 钳制在上界} 共 3^m 种活动集
// 组合，对每种组合解自由子空间线性方程 Hff·x_f = −(q_f +
// H_fc·x_c)，仅保留自由维 解仍落在盒内的可行候选，取代价最小者——严格凸 QP
// 的全局最优必为其中之一
template <int TDim>
double BruteForceBoxQp(const typename BoxQpSolver<TDim>::Mat& hessian,
                       const typename BoxQpSolver<TDim>::Vec& gradient,
                       const typename BoxQpSolver<TDim>::Vec& lower,
                       const typename BoxQpSolver<TDim>::Vec& upper,
                       typename BoxQpSolver<TDim>::Vec* optimum) {
    double best_cost = std::numeric_limits<double>::infinity();
    const int combo_count = static_cast<int>(std::pow(3.0, TDim));
    for (int combo = 0; combo < combo_count; ++combo) {
        int code = combo;
        std::vector<int> free_idx;
        std::vector<int> clamp_idx;
        std::vector<double> clamp_val;
        free_idx.reserve(TDim);
        clamp_idx.reserve(TDim);
        clamp_val.reserve(TDim);
        for (int j = 0; j < TDim; ++j) {
            const int mode = code % 3;
            code /= 3;
            if (mode == 0) {
                free_idx.push_back(j);
            } else {
                clamp_idx.push_back(j);
                clamp_val.push_back(mode == 1 ? lower(j) : upper(j));
            }
        }
        const int nf = static_cast<int>(free_idx.size());
        typename BoxQpSolver<TDim>::Vec x = BoxQpSolver<TDim>::Vec::Zero();
        Eigen::VectorXd x_free;
        if (nf > 0) {
            Eigen::MatrixXd h_ff(nf, nf);
            Eigen::VectorXd rhs(nf);
            for (int i = 0; i < nf; ++i) {
                rhs(i) = gradient(free_idx[i]);
                for (std::size_t c = 0; c < clamp_idx.size(); ++c) {
                    rhs(i) += hessian(free_idx[i], clamp_idx[c]) * clamp_val[c];
                }
                for (int k = 0; k < nf; ++k) {
                    h_ff(i, k) = hessian(free_idx[i], free_idx[k]);
                }
            }
            x_free = h_ff.ldlt().solve(-rhs);
            bool feasible = true;
            for (int i = 0; i < nf; ++i) {
                if (x_free(i) < lower(free_idx[i]) - 1e-12 ||
                    x_free(i) > upper(free_idx[i]) + 1e-12) {
                    feasible = false;
                }
            }
            if (!feasible) {
                continue;
            }
            for (int i = 0; i < nf; ++i) {
                x(free_idx[i]) = x_free(i);
            }
        }
        for (std::size_t c = 0; c < clamp_idx.size(); ++c) {
            x(clamp_idx[c]) = clamp_val[c];
        }
        const double cost = 0.5 * x.dot(hessian * x) + gradient.dot(x);
        if (cost < best_cost) {
            best_cost = cost;
            *optimum = x;
        }
    }
    return best_cost;
}

// 随机良态 QP 生成器（固定种子可复现）：H = AᵀA + I 保证良态对称正定，
// 线性项量级大于 H 使无约束最优经常出盒（覆盖钳制分支），初始点盒内随机
template <int TDim>
struct RandomQp {
    typename BoxQpSolver<TDim>::Mat hessian;
    typename BoxQpSolver<TDim>::Vec gradient;
    typename BoxQpSolver<TDim>::Vec lower;
    typename BoxQpSolver<TDim>::Vec upper;
    typename BoxQpSolver<TDim>::Vec initial;
};

template <int TDim>
RandomQp<TDim> MakeRandomQp(std::mt19937* rng) {
    std::uniform_real_distribution<double> unit_dist(-1.0, 1.0);
    std::uniform_real_distribution<double> grad_dist(-6.0, 6.0);
    std::uniform_real_distribution<double> bound_dist(0.2, 3.0);
    RandomQp<TDim> qp;
    typename BoxQpSolver<TDim>::Mat a;
    for (int i = 0; i < TDim; ++i) {
        for (int j = 0; j < TDim; ++j) {
            a(i, j) = unit_dist(*rng);
        }
    }
    qp.hessian = a.transpose() * a + BoxQpSolver<TDim>::Mat::Identity();
    for (int j = 0; j < TDim; ++j) {
        qp.gradient(j) = grad_dist(*rng);
        qp.lower(j) = -bound_dist(*rng);
        qp.upper(j) = bound_dist(*rng);
    }
    for (int j = 0; j < TDim; ++j) {
        std::uniform_real_distribution<double> init_dist(qp.lower(j),
                                                         qp.upper(j));
        qp.initial(j) = init_dist(*rng);
    }
    return qp;
}

// 无约束激活场景（退化一致性）：宽大盒边界下约束不起作用，
// 投影牛顿解必须退化为无约束解析解 −H⁻¹q，且一次分解、一次迭代收敛
TEST(BoxQpSolverTest, UnconstrainedSolutionMatchesAnalytic) {
    std::mt19937 rng(42);
    const RandomQp<ILQR_CONTROL_DIM> qp = MakeRandomQp<ILQR_CONTROL_DIM>(&rng);
    QpSolver::Problem problem;
    problem.hessian = qp.hessian;
    problem.gradient = qp.gradient;
    problem.lower = Vec2::Constant(-1e3);
    problem.upper = Vec2::Constant(1e3);
    problem.initial = Vec2::Zero();
    const QpSolver solver;
    const QpSolver::Result result = solver.solve(problem);
    ASSERT_EQ(result.status, QpSolver::Status::CONVERGED);
    EXPECT_EQ(result.free_dim, ILQR_CONTROL_DIM);
    EXPECT_EQ(result.factorizations, 1);
    EXPECT_EQ(result.iterations, 1);
    const Vec2 analytic = -qp.hessian.inverse() * qp.gradient;
    EXPECT_LT((result.x - analytic).norm(), 1e-12);
}

// 单边约束激活场景：无约束最优的第 0 维越出下界，最优解应为
// 「第 0 维钳制在下界、第 1 维在自由子空间取条件最优」，与暴力枚举参照一致
TEST(BoxQpSolverTest, SingleSidedClampingMatchesBruteForce) {
    Mat2 h;
    h << 2.0, 0.5, 0.5, 1.0;
    QpSolver::Problem problem;
    problem.hessian = h;
    problem.gradient = (Vec2() << 1.0, 1.0).finished();
    problem.lower = (Vec2() << -0.1, -5.0).finished();
    problem.upper = (Vec2() << 5.0, 5.0).finished();
    problem.initial = Vec2::Zero();
    const QpSolver solver;
    const QpSolver::Result result = solver.solve(problem);
    ASSERT_EQ(result.status, QpSolver::Status::CONVERGED);
    // 无约束最优 ≈ (−0.286, −0.857)，第 0 维越出 −0.1 下界
    EXPECT_EQ(result.free_dim, 1);
    EXPECT_EQ(result.free_indices[0], 1);
    EXPECT_TRUE(result.clamped[0]);
    EXPECT_FALSE(result.clamped[1]);
    Vec2 brute = Vec2::Zero();
    BruteForceBoxQp<ILQR_CONTROL_DIM>(h, problem.gradient, problem.lower,
                                     problem.upper, &brute);
    EXPECT_LT((result.x - brute).norm(), 1e-10);
    EXPECT_DOUBLE_EQ(result.x(0), -0.1);
}

// 双边约束激活场景：梯度在两维都指向盒外，最优解为角点（全维钳制），
// 自由子空间为空，验证全钳制角点解与暴力枚举参照一致
TEST(BoxQpSolverTest, DoubleSidedClampingMatchesBruteForce) {
    Mat2 h;
    h << 2.0, 0.5, 0.5, 1.0;
    QpSolver::Problem problem;
    problem.hessian = h;
    problem.gradient = (Vec2() << 5.0, 5.0).finished();
    problem.lower = Vec2::Constant(-1.0);
    problem.upper = Vec2::Constant(1.0);
    problem.initial = Vec2::Zero();
    const QpSolver solver;
    const QpSolver::Result result = solver.solve(problem);
    ASSERT_EQ(result.status, QpSolver::Status::CONVERGED);
    EXPECT_EQ(result.free_dim, 0);
    Vec2 brute = Vec2::Zero();
    BruteForceBoxQp<ILQR_CONTROL_DIM>(h, problem.gradient, problem.lower,
                                     problem.upper, &brute);
    EXPECT_LT((result.x - brute).norm(), 1e-10);
    EXPECT_LT((result.x - problem.lower).norm(), 1e-15);
}

// 批量随机良态 QP 与暴力枚举参照全量对照（验收标准：误差 < 1e-10）：
// 覆盖无钳制/单边/双边钳制的全部混合情形，初始点含盒内随机点
TEST(BoxQpSolverTest, RandomProblemsMatchBruteForceEnumeration) {
    std::mt19937 rng(20260728);
    const QpSolver solver;
    for (int trial = 0; trial < 60; ++trial) {
        const RandomQp<ILQR_CONTROL_DIM> qp =
            MakeRandomQp<ILQR_CONTROL_DIM>(&rng);
        QpSolver::Problem problem;
        problem.hessian = qp.hessian;
        problem.gradient = qp.gradient;
        problem.lower = qp.lower;
        problem.upper = qp.upper;
        problem.initial = qp.initial;
        const QpSolver::Result result = solver.solve(problem);
        ASSERT_EQ(result.status, QpSolver::Status::CONVERGED)
            << "trial " << trial << " 未收敛";
        Vec2 brute = Vec2::Zero();
        BruteForceBoxQp<ILQR_CONTROL_DIM>(qp.hessian, qp.gradient, qp.lower,
                                         qp.upper, &brute);
        EXPECT_LT((result.x - brute).norm(), 1e-10) << "trial " << trial;
    }
}

// 热启动正确性：以上一次求解的最优活动集热启动同一 QP（初始点取盒内另一点，
// 钳制维贴到对应边界），必须一次牛顿步内收敛——恰好一次迭代、一次分解，
// 且解与冷启动完全一致（Tassa 引理：活动集相同则一步牛顿收敛）
TEST(BoxQpSolverTest, WarmStartWithOptimalActiveSetConvergesInOneStep) {
    std::mt19937 rng(7);
    const QpSolver solver;
    for (int trial = 0; trial < 20; ++trial) {
        const RandomQp<ILQR_CONTROL_DIM> qp =
            MakeRandomQp<ILQR_CONTROL_DIM>(&rng);
        QpSolver::Problem cold;
        cold.hessian = qp.hessian;
        cold.gradient = qp.gradient;
        cold.lower = qp.lower;
        cold.upper = qp.upper;
        cold.initial = qp.initial;
        const QpSolver::Result cold_result = solver.solve(cold);
        ASSERT_EQ(cold_result.status, QpSolver::Status::CONVERGED);
        QpSolver::Problem warm = cold;
        warm.warm_start = true;
        warm.initial_clamped = cold_result.clamped;
        // 初始点取盒中点，钳制维贴到最优解所在的边界值
        warm.initial = 0.5 * (qp.lower + qp.upper);
        for (int j = 0; j < ILQR_CONTROL_DIM; ++j) {
            if (cold_result.clamped[j]) {
                warm.initial(j) = cold_result.x(j);
            }
        }
        const QpSolver::Result warm_result = solver.solve(warm);
        ASSERT_EQ(warm_result.status, QpSolver::Status::CONVERGED);
        EXPECT_EQ(warm_result.iterations, 1) << "trial " << trial;
        // 最优活动集热启动至多一次分解即收敛；全钳制最优时经全钳制退化分支
        // 零分解收敛，两种情形都满足「一次牛顿步内收敛」
        EXPECT_LE(warm_result.factorizations, 1) << "trial " << trial;
        EXPECT_LT((warm_result.x - cold_result.x).norm(), 1e-12)
            << "trial " << trial;
    }
}

// 错误活动集热启动：以最优活动集的逐位取反作为初始钳制集，
// 首步后梯度重判必须纠正错误集合，仍收敛到与冷启动相同的解
TEST(BoxQpSolverTest, WarmStartWithWrongActiveSetStillConverges) {
    std::mt19937 rng(11);
    const QpSolver solver;
    for (int trial = 0; trial < 20; ++trial) {
        const RandomQp<ILQR_CONTROL_DIM> qp =
            MakeRandomQp<ILQR_CONTROL_DIM>(&rng);
        QpSolver::Problem cold;
        cold.hessian = qp.hessian;
        cold.gradient = qp.gradient;
        cold.lower = qp.lower;
        cold.upper = qp.upper;
        cold.initial = qp.initial;
        const QpSolver::Result cold_result = solver.solve(cold);
        ASSERT_EQ(cold_result.status, QpSolver::Status::CONVERGED);
        QpSolver::Problem warm = cold;
        warm.warm_start = true;
        warm.initial = 0.5 * (qp.lower + qp.upper);
        for (int j = 0; j < ILQR_CONTROL_DIM; ++j) {
            warm.initial_clamped[j] = !cold_result.clamped[j];
            if (warm.initial_clamped[j]) {
                warm.initial(j) = qp.upper(j);
            }
        }
        const QpSolver::Result warm_result = solver.solve(warm);
        ASSERT_EQ(warm_result.status, QpSolver::Status::CONVERGED);
        EXPECT_LT((warm_result.x - cold_result.x).norm(), 1e-10)
            << "trial " << trial;
    }
}

// 返回的自由维 Hessian 分解一致性：紧凑 Cholesky 因子重构 L·Lᵀ 必须等于按
// free_indices 收集的 Hff 主子阵；自由维数与最优活动集一致；SolveFreeExpanded
// 的自由行等价于 Hff⁻¹·rhs、钳制行恒为零（M005 增益计算的消费约定）
TEST(BoxQpSolverTest, FreeHessianFactorMatchesFreeSubspace) {
    std::mt19937 rng(99);
    const QpSolver solver;
    for (int trial = 0; trial < 30; ++trial) {
        const RandomQp<ILQR_CONTROL_DIM> qp =
            MakeRandomQp<ILQR_CONTROL_DIM>(&rng);
        QpSolver::Problem problem;
        problem.hessian = qp.hessian;
        problem.gradient = qp.gradient;
        problem.lower = qp.lower;
        problem.upper = qp.upper;
        problem.initial = qp.initial;
        const QpSolver::Result result = solver.solve(problem);
        ASSERT_EQ(result.status, QpSolver::Status::CONVERGED);
        const int nf = result.free_dim;
        // 重构分解：L·Lᵀ ≈ Hff
        Eigen::MatrixXd h_ff(nf, nf);
        for (int i = 0; i < nf; ++i) {
            for (int j = 0; j < nf; ++j) {
                h_ff(i, j) =
                    qp.hessian(result.free_indices[i], result.free_indices[j]);
            }
        }
        const Eigen::MatrixXd l = result.free_factor.topLeftCorner(nf, nf);
        const double scale = h_ff.norm();
        // 全钳制样本（nf=0）时两侧同为零矩阵，故取小于等于
        EXPECT_LE((l * l.transpose() - h_ff).norm(), 1e-12 * scale)
            << "trial " << trial;
        // 满维散布求解：自由行 = Hff⁻¹·rhs，钳制行恒为零
        constexpr int kCols = 3;
        Eigen::Matrix<double, ILQR_CONTROL_DIM, kCols> rhs =
            Eigen::Matrix<double, ILQR_CONTROL_DIM, kCols>::Zero();
        Eigen::MatrixXd rhs_compact(nf, kCols);
        for (int i = 0; i < nf; ++i) {
            for (int c = 0; c < kCols; ++c) {
                rhs_compact(i, c) = 0.3 * (i + 1) + 0.7 * (c + 1);
            }
        }
        for (int i = 0; i < nf; ++i) {
            rhs.row(i) = rhs_compact.row(i);
        }
        const auto expanded = QpSolver::SolveFreeExpanded(result, rhs);
        const Eigen::MatrixXd expect_free = h_ff.inverse() * rhs_compact;
        for (int i = 0; i < nf; ++i) {
            EXPECT_LT(
                (expanded.row(result.free_indices[i]) - expect_free.row(i))
                    .norm(),
                1e-12)
                << "trial " << trial;
        }
        for (int j = 0; j < ILQR_CONTROL_DIM; ++j) {
            if (result.clamped[j]) {
                EXPECT_TRUE(expanded.row(j).isZero(0.0)) << "trial " << trial;
            }
        }
    }
}

// 全维钳制退化行为（无自由维）：初始点即在梯度全部指向盒外的角点上，
// 必须良定义收敛——自由维数为 0、不发生任何矩阵分解、解保持角点、无 NaN
TEST(BoxQpSolverTest, FullyClampedDegenerateCaseIsWellDefined) {
    QpSolver::Problem problem;
    problem.hessian = Mat2::Identity();
    problem.gradient = (Vec2() << 5.0, -5.0).finished();
    problem.lower = Vec2::Constant(-1.0);
    problem.upper = Vec2::Constant(1.0);
    // 角点 (−1, 1)：g = q + x = (4, −4)，第 0 维下界处梯度向下、
    // 第 1 维上界处梯度向上，两维均满足钳制条件
    problem.initial = (Vec2() << -1.0, 1.0).finished();
    const QpSolver solver;
    const QpSolver::Result result = solver.solve(problem);
    ASSERT_EQ(result.status, QpSolver::Status::CONVERGED);
    EXPECT_EQ(result.free_dim, 0);
    EXPECT_EQ(result.factorizations, 0);
    EXPECT_TRUE(result.x.allFinite());
    EXPECT_LT((result.x - problem.initial).norm(), 1e-15);
    // 退化分解下的满维散布求解返回零矩阵（无任何自由行）
    const Eigen::Matrix<double, ILQR_CONTROL_DIM, 2> ones =
        Eigen::Matrix<double, ILQR_CONTROL_DIM, 2>::Ones();
    const auto expanded = QpSolver::SolveFreeExpanded(result, ones);
    EXPECT_TRUE(expanded.isZero(0.0));
}

// 病态输入防御：H 含 1e-12 量级小特征值（叠加正则化 ρ=1e-9 后仍正定，
// 条件数 ~1e9），求解必须收敛且全程有限（无 NaN/Inf），代价不劣于参照解
TEST(BoxQpSolverTest, NearSingularHessianStaysFinite) {
    const double angle = 0.7;
    Mat2 rot;
    rot << std::cos(angle), -std::sin(angle), std::sin(angle), std::cos(angle);
    Mat2 diag = Mat2::Zero();
    diag(0, 0) = 1.0;
    diag(1, 1) = 1e-12;
    const Mat2 h = rot * diag * rot.transpose() + 1e-9 * Mat2::Identity();
    QpSolver::Problem problem;
    problem.hessian = h;
    problem.gradient = (Vec2() << 0.5, -0.3).finished();
    problem.lower = Vec2::Constant(-1e4);
    problem.upper = Vec2::Constant(1e4);
    problem.initial = Vec2::Zero();
    const QpSolver solver;
    const QpSolver::Result result = solver.solve(problem);
    ASSERT_EQ(result.status, QpSolver::Status::CONVERGED);
    EXPECT_TRUE(result.x.allFinite());
    Vec2 brute = Vec2::Zero();
    const double brute_cost = BruteForceBoxQp<ILQR_CONTROL_DIM>(
        h, problem.gradient, problem.lower, problem.upper, &brute);
    EXPECT_LE(result.cost, brute_cost + 1e-6 * (1.0 + std::abs(brute_cost)));
}

// 非正定输入的失败语义：H 含负特征值时 Cholesky 分解失败，
// 必须以 NOT_POSITIVE_DEFINITE 状态码上报（不抛异常、不产生 NaN），
// 供调用方增大正则化后重解
TEST(BoxQpSolverTest, NonPositiveDefiniteInputReportsFailure) {
    QpSolver::Problem problem;
    problem.hessian = Mat2::Identity();
    problem.hessian(1, 1) = -0.5;
    problem.gradient = (Vec2() << 1.0, 1.0).finished();
    problem.lower = Vec2::Constant(-1.0);
    problem.upper = Vec2::Constant(1.0);
    problem.initial = Vec2::Zero();
    const QpSolver solver;
    const QpSolver::Result result = solver.solve(problem);
    EXPECT_EQ(result.status, QpSolver::Status::NOT_POSITIVE_DEFINITE);
    EXPECT_TRUE(result.x.allFinite());
}

// 迭代超限的失败语义：该 QP 冷启动需要 2 次迭代（首次全牛顿步越界投影后
// 活动集变化，需重分解再收敛），把迭代上限压到 1 时必须上报 MAX_ITERATIONS
// 且返回的最后迭代点有限；同一 QP 在默认上限下正常收敛作为对照
TEST(BoxQpSolverTest, MaxIterationsIsReported) {
    Mat2 h;
    h << 2.0, 0.5, 0.5, 1.0;
    QpSolver::Problem problem;
    problem.hessian = h;
    problem.gradient = (Vec2() << 1.0, 1.0).finished();
    problem.lower = (Vec2() << -0.1, -5.0).finished();
    problem.upper = (Vec2() << 5.0, 5.0).finished();
    problem.initial = Vec2::Zero();
    QpSolver::Options options;
    options.max_iterations = 1;
    const QpSolver limited_solver(options);
    const QpSolver::Result result = limited_solver.solve(problem);
    EXPECT_EQ(result.status, QpSolver::Status::MAX_ITERATIONS);
    EXPECT_EQ(result.iterations, 1);
    EXPECT_TRUE(result.x.allFinite());
    const QpSolver default_solver;
    EXPECT_EQ(default_solver.solve(problem).status,
              QpSolver::Status::CONVERGED);
}

// 输入契约校验：盒下界大于上界属于契约违例，抛 std::invalid_argument
TEST(BoxQpSolverTest, InvalidBoundsThrow) {
    QpSolver::Problem problem;
    problem.hessian = Mat2::Identity();
    problem.gradient = Vec2::Zero();
    problem.lower = (Vec2() << 1.0, -1.0).finished();
    problem.upper = (Vec2() << -1.0, 1.0).finished();
    problem.initial = Vec2::Zero();
    const QpSolver solver;
    EXPECT_THROW(solver.solve(problem), std::invalid_argument);
}

// 初始点不可行（越出盒外）：求解器内部逐元素投影进盒，
// 解必须与暴力枚举参照一致（调用方无需保证初始可行）
TEST(BoxQpSolverTest, InfeasibleInitialPointIsProjected) {
    std::mt19937 rng(5);
    const QpSolver solver;
    for (int trial = 0; trial < 20; ++trial) {
        const RandomQp<ILQR_CONTROL_DIM> qp =
            MakeRandomQp<ILQR_CONTROL_DIM>(&rng);
        QpSolver::Problem problem;
        problem.hessian = qp.hessian;
        problem.gradient = qp.gradient;
        problem.lower = qp.lower;
        problem.upper = qp.upper;
        problem.initial = 10.0 * qp.upper;  // 故意越界
        const QpSolver::Result result = solver.solve(problem);
        ASSERT_EQ(result.status, QpSolver::Status::CONVERGED);
        Vec2 brute = Vec2::Zero();
        BruteForceBoxQp<ILQR_CONTROL_DIM>(qp.hessian, qp.gradient, qp.lower,
                                         qp.upper, &brute);
        EXPECT_LT((result.x - brute).norm(), 1e-10) << "trial " << trial;
    }
}

// 维度不硬编码：同一实现实例化为 3 维控制，批量随机 QP
// 仍与暴力枚举参照（27 种活动集组合）对照误差 < 1e-10
TEST(BoxQpSolverTest, HigherDimensionTemplateMatchesBruteForce) {
    std::mt19937 rng(31);
    const BoxQpSolver<3> solver;
    for (int trial = 0; trial < 30; ++trial) {
        const RandomQp<3> qp = MakeRandomQp<3>(&rng);
        BoxQpSolver<3>::Problem problem;
        problem.hessian = qp.hessian;
        problem.gradient = qp.gradient;
        problem.lower = qp.lower;
        problem.upper = qp.upper;
        problem.initial = qp.initial;
        const auto result = solver.solve(problem);
        ASSERT_EQ(result.status, BoxQpSolver<3>::Status::CONVERGED);
        BoxQpSolver<3>::Vec brute = BoxQpSolver<3>::Vec::Zero();
        BruteForceBoxQp<3>(qp.hessian, qp.gradient, qp.lower, qp.upper, &brute);
        EXPECT_LT((result.x - brute).norm(), 1e-10) << "trial " << trial;
    }
}

}  // namespace
}  // namespace apa_post_processor
