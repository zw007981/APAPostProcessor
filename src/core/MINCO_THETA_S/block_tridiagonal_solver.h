#pragma once

#include <Eigen/Core>
#include <Eigen/LU>
#include <vector>

namespace apa_post_processor {
// 固定 6x6 块的块三对角矩阵专用求解器。对应 h=3 时每个多项式段 6 个系数的
// MINCO 线性系统 K(T)c=b：装配后的 K(T) 为块三对角结构，此处采用手写块
// Thomas 算法（块 LU 前向消元 + 回代），复杂度 O(M)（M 为块数），全程使用
// 固定尺寸 Eigen 矩阵，避免 Eigen::SparseMatrix/SparseLU 的堆分配开销。
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
    // 下/主/上三条块对角线
    void factorize(const std::vector<Block>& lower,
                   const std::vector<Block>& diagonal,
                   const std::vector<Block>& upper);
    // 求解 A x = rhs。rhs 与返回值均为 6xN' 块矩阵
    BlockMatrix solve(const BlockMatrix& rhs) const;
    // 求解 A^T x = rhs（伴随系统），复用 factorize 的消元结果
    BlockMatrix solveTranspose(const BlockMatrix& rhs) const;
    // 已分解的块数；未调用 factorize 时为 0
    int numBlocks() const { return static_cast<int>(lu_diag_.size()); }
    // 是否已完成分解
    bool isFactorized() const { return factorized_; }

   protected:
    // 追加一个消元后对角块P的部分主元LU分解，P奇异时抛异常
    void appendFactorization(const Block& p);
    // 校验已完成分解且rhs列数为分解块数的正整数倍，否则抛标准异常
    void checkSolvable(const BlockMatrix& rhs) const;

   protected:
    // N-1 个下对角块
    std::vector<Block> lower_;
    // N-1 个上对角块
    std::vector<Block> upper_;
    // 消元后对角块 P_i 的部分主元 LU（正向求解与转置伴随求解共用一份）。
    // 部分主元在病态块上仍能保证求解稳健性（极端段时长用例有测试钉住），
    // 相比完全主元省掉列主元搜索与第二份转置分解
    std::vector<Eigen::PartialPivLU<Block>> lu_diag_;
    // 是否已经完成分解
    bool factorized_{false};
};
}  // namespace apa_post_processor
