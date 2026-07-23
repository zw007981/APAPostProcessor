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
    lu_diag_transpose_.clear();
    lu_diag_.reserve(num_blocks);
    lu_diag_transpose_.reserve(num_blocks);
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
    // 前向替换：y_i = b_i - L_i (P_{i-1}^{-1} y_{i-1})
    BlockMatrix y = rhs;
    for (int i = 1; i < num_blocks; ++i) {
        y.col(i) -= lower_[i - 1] * lu_diag_[i - 1].solve(y.col(i - 1)).eval();
    }
    // 回代：x_i = P_i^{-1} (y_i - U_i x_{i+1})
    BlockMatrix x(BLOCK_SIZE, num_blocks);
    x.col(num_blocks - 1) =
        lu_diag_[num_blocks - 1].solve(y.col(num_blocks - 1));
    for (int i = num_blocks - 2; i >= 0; --i) {
        x.col(i) = lu_diag_[i].solve(y.col(i) - upper_[i] * x.col(i + 1));
    }
    return x;
}

BlockTridiagonalSolver::BlockMatrix BlockTridiagonalSolver::solveTranspose(
    const BlockMatrix& rhs) const {
    checkSolvable(rhs);
    const int num_blocks = numBlocks();
    // A^T = U^T L^T。先解 U^T z = rhs（块下三角，对角块 P_i^T、下对角块
    // U_i^T）：z_i = P_i^{-T} (b_i - U_{i-1}^T z_{i-1})
    BlockMatrix z(BLOCK_SIZE, num_blocks);
    z.col(0) = lu_diag_transpose_[0].solve(rhs.col(0));
    for (int i = 1; i < num_blocks; ++i) {
        z.col(i) = lu_diag_transpose_[i].solve(
            rhs.col(i) - upper_[i - 1].transpose() * z.col(i - 1));
    }
    // 再解 L^T x = z（块上三角，单位对角块、上对角块 (L_{i+1} P_i^{-1})^T）：
    // x_i = z_i - P_i^{-T} L_{i+1}^T x_{i+1}
    BlockMatrix x(BLOCK_SIZE, num_blocks);
    x.col(num_blocks - 1) = z.col(num_blocks - 1);
    for (int i = num_blocks - 2; i >= 0; --i) {
        x.col(i) = z.col(i) - lu_diag_transpose_[i].solve(
                                  lower_[i].transpose() * x.col(i + 1));
    }
    return x;
}

void BlockTridiagonalSolver::appendFactorization(const Block& p) {
    lu_diag_.emplace_back(p);
    if (!lu_diag_.back().isInvertible()) {
        throw std::runtime_error("块三对角矩阵消元后出现奇异对角块");
    }
    lu_diag_transpose_.emplace_back(p.transpose());
}

void BlockTridiagonalSolver::checkSolvable(const BlockMatrix& rhs) const {
    if (!factorized_) {
        throw std::logic_error("块三对角求解器尚未完成分解");
    }
    if (rhs.cols() != numBlocks()) {
        throw std::invalid_argument("右端项块数与分解块数不一致");
    }
}
}  // namespace apa_post_processor
