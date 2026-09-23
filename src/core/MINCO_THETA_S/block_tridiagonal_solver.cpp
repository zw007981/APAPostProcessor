#include "block_tridiagonal_solver.h"

#include <stdexcept>

namespace apa_post_processor {
void BlockTridiagonalSolver::factorize(const std::vector<Block>& lower,
                                       const std::vector<Block>& diagonal,
                                       const std::vector<Block>& upper) {
    const int num_blocks = static_cast<int>(diagonal.size());
    if (num_blocks < 1) {
        throw std::invalid_argument("块三对角矩阵至少需要一个主对角块");
    }
    if (static_cast<int>(lower.size()) != num_blocks - 1 ||
        static_cast<int>(upper.size()) != num_blocks - 1) {
        throw std::invalid_argument("下/上对角块数量必须比主对角块数量少 1");
    }
    lower_ = lower;
    upper_ = upper;
    lu_diag_.clear();
    lu_diag_.reserve(num_blocks);
    // 块 Thomas 前向消元：P_i = D_i - L_i P_{i-1}^{-1} U_{i-1}
    appendFactorization(diagonal[0]);
    for (int i = 1; i < num_blocks; ++i) {
        // 用上一块的 LU 直接求解 P_{i-1}^{-1} U_{i-1}，避免显式求逆
        const Block schur =
            diagonal[i] - lower_[i - 1] * lu_diag_[i - 1].solve(upper_[i - 1]);
        appendFactorization(schur);
    }
    factorized_ = true;
}

BlockTridiagonalSolver::BlockMatrix BlockTridiagonalSolver::solve(
    const BlockMatrix& rhs) const {
    checkSolvable(rhs);
    const int num_blocks = numBlocks();
    // 前向替换：y_i = b_i - L_i (P_{i-1}^{-1} y_{i-1})。多组右端项按连续
    // numBlocks 列分组，组间无耦合，逐组独立替换
    BlockMatrix y = rhs;
    for (int g = 0; g < y.cols(); g += num_blocks) {
        for (int i = 1; i < num_blocks; ++i) {
            y.col(g + i) -=
                lower_[i - 1] * lu_diag_[i - 1].solve(y.col(g + i - 1)).eval();
        }
    }
    // 回代：x_i = P_i^{-1} (y_i - U_i x_{i+1})
    BlockMatrix x(BLOCK_SIZE, rhs.cols());
    for (int g = 0; g < y.cols(); g += num_blocks) {
        x.col(g + num_blocks - 1) =
            lu_diag_[num_blocks - 1].solve(y.col(g + num_blocks - 1));
        for (int i = num_blocks - 2; i >= 0; --i) {
            x.col(g + i) =
                lu_diag_[i].solve(y.col(g + i) - upper_[i] * x.col(g + i + 1));
        }
    }
    return x;
}

BlockTridiagonalSolver::BlockMatrix BlockTridiagonalSolver::solveTranspose(
    const BlockMatrix& rhs) const {
    checkSolvable(rhs);
    const int num_blocks = numBlocks();
    // A^T = U^T L^T。先解 U^T z = rhs（块下三角，对角块 P_i^T、下对角块
    // U_i^T）：z_i = P_i^{-T} (b_i - U_{i-1}^T z_{i-1})；P_i^{-T} 经
    // transpose().solve() 复用同一份 LU，不再单独分解转置块
    BlockMatrix z(BLOCK_SIZE, rhs.cols());
    for (int g = 0; g < z.cols(); g += num_blocks) {
        z.col(g) = lu_diag_[0].transpose().solve(rhs.col(g));
        for (int i = 1; i < num_blocks; ++i) {
            z.col(g + i) = lu_diag_[i].transpose().solve(
                rhs.col(g + i) - upper_[i - 1].transpose() * z.col(g + i - 1));
        }
    }
    // 再解 L^T x = z（块上三角，单位对角块、上对角块 (L_{i+1} P_i^{-1})^T）：
    // x_i = z_i - P_i^{-T} L_{i+1}^T x_{i+1}
    BlockMatrix x(BLOCK_SIZE, rhs.cols());
    for (int g = 0; g < x.cols(); g += num_blocks) {
        x.col(g + num_blocks - 1) = z.col(g + num_blocks - 1);
        for (int i = num_blocks - 2; i >= 0; --i) {
            // 转置求解的 Solve 表达式必须直接赋给具体对象：Eigen 的
            // Assignment 特化只覆盖 dst = dec.transpose().solve(rhs)
            // 形态，嵌套在二元表达式中会落到无 _solve_impl 的泛型求值路径
            const Eigen::Matrix<double, BLOCK_SIZE, 1> back_subst =
                lu_diag_[i].transpose().solve(lower_[i].transpose() *
                                              x.col(g + i + 1));
            x.col(g + i) = z.col(g + i) - back_subst;
        }
    }
    return x;
}

void BlockTridiagonalSolver::appendFactorization(const Block& p) {
    lu_diag_.emplace_back(p);
    // PartialPivLU 无秩判定接口：奇异块使 LU 对角线产生零元（或 0/0 产生
    // NaN），显式检查而非静默继续产生垃圾解
    const auto& lu = lu_diag_.back().matrixLU();
    if (!lu.allFinite() || lu.diagonal().cwiseAbs().minCoeff() == 0.0) {
        throw std::runtime_error("块三对角矩阵消元后出现奇异对角块");
    }
}

void BlockTridiagonalSolver::checkSolvable(const BlockMatrix& rhs) const {
    if (!factorized_) {
        throw std::logic_error("块三对角求解器尚未完成分解");
    }
    if (rhs.cols() < 1 || rhs.cols() % numBlocks() != 0) {
        throw std::invalid_argument("右端项列数必须为分解块数的正整数倍");
    }
}
}  // namespace apa_post_processor
