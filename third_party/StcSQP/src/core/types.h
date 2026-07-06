#pragma once

#include <Eigen/Dense>
#include <Eigen/StdVector>

namespace stc_SQP {
// Eigen向量
using Vector = Eigen::VectorXd;
// Eigen矩阵
using Matrix = Eigen::MatrixXd;
// 使用 Eigen 对齐分配器的 vector 别名。
// Eigen 开启向量化（AVX2/AVX512）时，SIMD 加载要求元素按 32/64 字节对齐；
// 该分配器保证容器起始地址满足 Eigen 对齐要求，是 QPData 内存池安全的基础。
template <typename T>
using AlignedVector = std::vector<T, Eigen::aligned_allocator<T>>;
// 下标
using Index = int;
// 将 double 元素个数向上取整到 4 的倍数，保证每块内存起始地址 32 字节对齐（AVX2）。
// QPData 内存池在计算总容量与偏移量时必须全程调用此函数，禁止直接 offset += nx*nx。
inline size_t AlignSize(size_t num_doubles)
{
    return (num_doubles + 3) & ~3;
}
} // namespace stc_SQP
