#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/LU>
#include <random>
#include <stdexcept>
#include <vector>

#include "core/ALM/block_tridiagonal_solver.h"

namespace apa_post_processor {
namespace {

using Block = BlockTridiagonalSolver::Block;
using BlockMatrix = BlockTridiagonalSolver::BlockMatrix;
constexpr int BLOCK_SIZE = BlockTridiagonalSolver::BLOCK_SIZE;

// 把三条块对角线装配成稠密矩阵，供稠密参考解对拍
Eigen::MatrixXd AssembleDense(const std::vector<Block>& lower,
                              const std::vector<Block>& diagonal,
                              const std::vector<Block>& upper) {
    const int num_blocks = static_cast<int>(diagonal.size());
    Eigen::MatrixXd dense =
        Eigen::MatrixXd::Zero(BLOCK_SIZE * num_blocks, BLOCK_SIZE * num_blocks);
    for (int i = 0; i < num_blocks; ++i) {
        dense.block<BLOCK_SIZE, BLOCK_SIZE>(BLOCK_SIZE * i, BLOCK_SIZE * i) =
            diagonal[i];
    }
    for (int i = 0; i + 1 < num_blocks; ++i) {
        dense.block<BLOCK_SIZE, BLOCK_SIZE>(BLOCK_SIZE * (i + 1),
                                            BLOCK_SIZE * i) = lower[i];
        dense.block<BLOCK_SIZE, BLOCK_SIZE>(BLOCK_SIZE * i,
                                            BLOCK_SIZE * (i + 1)) = upper[i];
    }
    return dense;
}

// 生成主对角块叠加 6I 的随机块三对角系统：对角占优保证块消元数值稳定
void BuildRandomSystem(int num_blocks, std::mt19937* rng,
                       std::vector<Block>* lower, std::vector<Block>* diagonal,
                       std::vector<Block>* upper) {
    std::uniform_real_distribution<double> dist(-1.0, 1.0);
    const auto random_block = [&dist, rng]() {
        Block block;
        for (int row = 0; row < BLOCK_SIZE; ++row) {
            for (int col = 0; col < BLOCK_SIZE; ++col) {
                block(row, col) = dist(*rng);
            }
        }
        return block;
    };
    lower->clear();
    diagonal->clear();
    upper->clear();
    lower->reserve(num_blocks - 1);
    diagonal->reserve(num_blocks);
    upper->reserve(num_blocks - 1);
    for (int i = 0; i < num_blocks; ++i) {
        diagonal->push_back(random_block() + 6.0 * Block::Identity());
    }
    for (int i = 0; i + 1 < num_blocks; ++i) {
        lower->push_back(random_block());
        upper->push_back(random_block());
    }
}

// 生成随机右端项块矩阵（6 x num_blocks）
BlockMatrix BuildRandomRhs(int num_blocks, std::mt19937* rng) {
    std::uniform_real_distribution<double> dist(-1.0, 1.0);
    BlockMatrix rhs(BLOCK_SIZE, num_blocks);
    for (int i = 0; i < rhs.size(); ++i) {
        rhs(i) = dist(*rng);
    }
    return rhs;
}

// 测试单块退化场景（块三对角退化为单个 6x6 块）。
// 因为 M=1 的 MINCO 系统就是单块结构，所以该场景必须精确成立。
TEST(BlockTridiagonalSolverTest, SingleBlockMatchesDenseSolve) {
    std::mt19937 rng(42);
    std::vector<Block> lower, diagonal, upper;
    BuildRandomSystem(1, &rng, &lower, &diagonal, &upper);
    const BlockMatrix rhs = BuildRandomRhs(1, &rng);
    BlockTridiagonalSolver solver;
    solver.factorize(lower, diagonal, upper);

    const BlockMatrix x = solver.solve(rhs);
    const Eigen::MatrixXd dense = AssembleDense(lower, diagonal, upper);
    const Eigen::VectorXd rhs_vec =
        Eigen::Map<const Eigen::VectorXd>(rhs.data(), BLOCK_SIZE);
    const Eigen::VectorXd x_ref = dense.partialPivLu().solve(rhs_vec);

    EXPECT_TRUE(solver.isFactorized());
    EXPECT_EQ(solver.numBlocks(), 1);
    EXPECT_LE((Eigen::Map<const Eigen::VectorXd>(x.data(), BLOCK_SIZE) - x_ref)
                  .cwiseAbs()
                  .maxCoeff(),
              1e-12);
}

// 测试多块系统正向求解与稠密 PartialPivLU 参考解对拍。
// 因为块 Thomas 的数学本质是不选主元的块 LU，所以结果必须与稠密解一致。
TEST(BlockTridiagonalSolverTest, MultiBlockMatchesDenseSolve) {
    std::mt19937 rng(7);
    std::vector<Block> lower, diagonal, upper;
    BuildRandomSystem(5, &rng, &lower, &diagonal, &upper);
    const BlockMatrix rhs = BuildRandomRhs(5, &rng);
    BlockTridiagonalSolver solver;
    solver.factorize(lower, diagonal, upper);

    const BlockMatrix x = solver.solve(rhs);
    const Eigen::MatrixXd dense = AssembleDense(lower, diagonal, upper);
    const Eigen::VectorXd rhs_vec =
        Eigen::Map<const Eigen::VectorXd>(rhs.data(), dense.rows());
    const Eigen::VectorXd x_ref = dense.partialPivLu().solve(rhs_vec);

    EXPECT_LE(
        (Eigen::Map<const Eigen::VectorXd>(x.data(), dense.rows()) - x_ref)
            .cwiseAbs()
            .maxCoeff(),
        1e-9);
}

// 测试转置（伴随）求解与稠密参考解对拍。
// 因为 s_f 伴随梯度依赖 K(T)^{-T}，所以转置求解必须与正向共享同一分解。
TEST(BlockTridiagonalSolverTest, TransposeSolveMatchesDenseSolve) {
    std::mt19937 rng(11);
    std::vector<Block> lower, diagonal, upper;
    BuildRandomSystem(4, &rng, &lower, &diagonal, &upper);
    const BlockMatrix rhs = BuildRandomRhs(4, &rng);
    BlockTridiagonalSolver solver;
    solver.factorize(lower, diagonal, upper);

    const BlockMatrix x = solver.solveTranspose(rhs);
    const Eigen::MatrixXd dense = AssembleDense(lower, diagonal, upper);
    const Eigen::VectorXd rhs_vec =
        Eigen::Map<const Eigen::VectorXd>(rhs.data(), dense.rows());
    const Eigen::VectorXd x_ref =
        dense.transpose().partialPivLu().solve(rhs_vec);

    EXPECT_LE(
        (Eigen::Map<const Eigen::VectorXd>(x.data(), dense.rows()) - x_ref)
            .cwiseAbs()
            .maxCoeff(),
        1e-9);
}

// 测试较大块数（N=50）下的正确性。
// 因为 O(M) 算法的价值体现在大段数场景，所以必须在大块数下保持精度。
TEST(BlockTridiagonalSolverTest, LargeSystemMatchesDenseSolve) {
    std::mt19937 rng(23);
    std::vector<Block> lower, diagonal, upper;
    BuildRandomSystem(50, &rng, &lower, &diagonal, &upper);
    const BlockMatrix rhs = BuildRandomRhs(50, &rng);
    BlockTridiagonalSolver solver;
    solver.factorize(lower, diagonal, upper);

    const BlockMatrix x = solver.solve(rhs);
    const Eigen::MatrixXd dense = AssembleDense(lower, diagonal, upper);
    const Eigen::VectorXd rhs_vec =
        Eigen::Map<const Eigen::VectorXd>(rhs.data(), dense.rows());
    const Eigen::VectorXd x_ref = dense.partialPivLu().solve(rhs_vec);

    EXPECT_LE(
        (Eigen::Map<const Eigen::VectorXd>(x.data(), dense.rows()) - x_ref)
            .cwiseAbs()
            .maxCoeff(),
        1e-9);
}

// 测试一次分解服务多个右端项的场景。
// 因为 θ 与 s 两维共享同一 K(T)，所以分解复用必须各自给出正确解。
TEST(BlockTridiagonalSolverTest, FactorizationReuseServesMultipleRhs) {
    std::mt19937 rng(31);
    std::vector<Block> lower, diagonal, upper;
    BuildRandomSystem(3, &rng, &lower, &diagonal, &upper);
    const BlockMatrix rhs_a = BuildRandomRhs(3, &rng);
    const BlockMatrix rhs_b = BuildRandomRhs(3, &rng);
    BlockTridiagonalSolver solver;
    solver.factorize(lower, diagonal, upper);
    const Eigen::MatrixXd dense = AssembleDense(lower, diagonal, upper);

    const BlockMatrix x_a = solver.solve(rhs_a);
    const BlockMatrix x_b = solver.solve(rhs_b);
    const Eigen::VectorXd rhs_vec_a =
        Eigen::Map<const Eigen::VectorXd>(rhs_a.data(), dense.rows());
    const Eigen::VectorXd rhs_vec_b =
        Eigen::Map<const Eigen::VectorXd>(rhs_b.data(), dense.rows());
    const Eigen::VectorXd x_ref_a = dense.partialPivLu().solve(rhs_vec_a);
    const Eigen::VectorXd x_ref_b = dense.partialPivLu().solve(rhs_vec_b);

    EXPECT_LE(
        (Eigen::Map<const Eigen::VectorXd>(x_a.data(), dense.rows()) - x_ref_a)
            .cwiseAbs()
            .maxCoeff(),
        1e-9);
    EXPECT_LE(
        (Eigen::Map<const Eigen::VectorXd>(x_b.data(), dense.rows()) - x_ref_b)
            .cwiseAbs()
            .maxCoeff(),
        1e-9);
}

// 测试输入尺寸不合法的拒绝行为。
// 因为装配错误属于调用方逻辑错误，所以必须抛出异常而非静默纠错。
TEST(BlockTridiagonalSolverTest, RejectsMismatchedSizes) {
    std::mt19937 rng(37);
    std::vector<Block> lower, diagonal, upper;
    BuildRandomSystem(3, &rng, &lower, &diagonal, &upper);
    BlockTridiagonalSolver solver;

    std::vector<Block> empty;
    EXPECT_THROW(solver.factorize(lower, empty, upper), std::invalid_argument);
    std::vector<Block> bad_lower(lower.begin() + 1, lower.end());
    EXPECT_THROW(solver.factorize(bad_lower, diagonal, upper),
                 std::invalid_argument);
    std::vector<Block> bad_upper(upper.begin() + 1, upper.end());
    EXPECT_THROW(solver.factorize(lower, diagonal, bad_upper),
                 std::invalid_argument);

    solver.factorize(lower, diagonal, upper);
    const BlockMatrix bad_rhs = BlockMatrix::Zero(BLOCK_SIZE, 2);
    EXPECT_THROW(solver.solve(bad_rhs), std::invalid_argument);
    EXPECT_THROW(solver.solveTranspose(bad_rhs), std::invalid_argument);
}

// 测试未分解先求解与奇异对角块两类异常。
// 因为非预期状态必须显式失败，所以分别抛 std::logic_error 与
// std::runtime_error。
TEST(BlockTridiagonalSolverTest, ThrowsOnUnfactoredOrSingular) {
    BlockTridiagonalSolver solver;
    const BlockMatrix rhs = BlockMatrix::Zero(BLOCK_SIZE, 1);
    EXPECT_THROW(solver.solve(rhs), std::logic_error);

    std::vector<Block> lower, diagonal, upper;
    diagonal.push_back(Block::Zero());
    EXPECT_THROW(solver.factorize(lower, diagonal, upper), std::runtime_error);
}

}  // namespace
}  // namespace apa_post_processor
