#pragma once

#include <Eigen/Core>
#include <Eigen/LU>
#include <vector>

namespace apa_post_processor {
// 固定 6x6 块的块三对角矩阵专用求解器。对应 h=3 时每个多项式段 6 个系数的
// MINCO 线性系统 K(T)c=b：装配后的 K(T) 为块三对角结构，此处采用手写块
// Thomas 算法（块 LU 前向消元 + 回代），复杂度 O(M)（M 为块数），全程使用
// 固定尺寸 Eigen 矩阵，避免 Eigen::SparseMatrix/SparseLU 的堆分配与索引
// 开销。一次 factorize 同时服务正向求解 solve 与转置（伴随）求解
// solveTranspose：消元阶段对每个对角块同时保留其自身与转置的完全主元 LU，
// 两种求解共享同一份消元结果，不需要重新分解矩阵。
class BlockTridiagonalSolver {
   public:
    // 固定块尺寸：5 阶多项式每段 6 个系数
    static constexpr int BLOCK_SIZE = 6;
    // 单个 6x6 系数块
    using Block = Eigen::Matrix<double, BLOCK_SIZE, BLOCK_SIZE>;
    // 块向量序列：固定 6 行，第 i 列对应第 i 个块
    using BlockMatrix = Eigen::Matrix<double, BLOCK_SIZE, Eigen::Dynamic>;
    BlockTridiagonalSolver() = default;
    // 对块三对角矩阵做块 LU 分解（前向消元）。lower/diagonal/upper 分别为
    // 下/主/上三条块对角线：diagonal 含 N 个块（N >= 1），lower 与 upper
    // 各含 N-1 个块，其中 lower[i] 是块行 i+1 的左邻块、upper[i] 是块行 i
    // 的右邻块。尺寸不匹配抛 std::invalid_argument；消元中对角块奇异抛
    // std::runtime_error。
    void factorize(const std::vector<Block>& lower,
                   const std::vector<Block>& diagonal,
                   const std::vector<Block>& upper);
    // 求解 A x = rhs。rhs 与返回值均为 6xN 块矩阵，列数必须与分解块数一致
    BlockMatrix solve(const BlockMatrix& rhs) const;
    // 求解 A^T x = rhs（伴随系统），复用 factorize 的消元结果，不重新分解
    BlockMatrix solveTranspose(const BlockMatrix& rhs) const;
    // 已分解的块数；未调用 factorize 时为 0
    int numBlocks() const { return static_cast<int>(lu_diag_.size()); }
    // 是否已完成分解
    bool isFactorized() const { return factorized_; }

   protected:
    // 追加一个消元后对角块 P 的正向/转置 LU 分解，P 奇异时抛
    // std::runtime_error
    void appendFactorization(const Block& p);
    // 校验已完成分解且 rhs 列数与块数一致，否则抛标准异常
    void checkSolvable(const BlockMatrix& rhs) const;

   protected:
    // N-1 个下对角块
    std::vector<Block> lower_;
    // N-1 个上对角块
    std::vector<Block> upper_;
    // 消元后对角块 P_i 的完全主元 LU（正向求解用）；完全主元在块病态时
    // 仍能保持秩判定与求解的稳健性，6x6 固定尺寸下开销可忽略
    std::vector<Eigen::FullPivLU<Block>> lu_diag_;
    // 消元后对角块转置 P_i^T 的完全主元 LU（转置伴随求解用）
    std::vector<Eigen::FullPivLU<Block>> lu_diag_transpose_;
    bool factorized_{false};
};
}  // namespace apa_post_processor
