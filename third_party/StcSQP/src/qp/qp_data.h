#pragma once

#include <memory>
#include <vector>

#include "../core/types.h"

namespace stc_SQP {
// 软约束配置：HPIPM 原生支持对约束引入松弛变量，以 L2/L1 惩罚替代硬约束
// 目前 idxs 解释为“每一步中需要软化的普通约束（general constraint）在 C/d 中的行号”
// 下界 slack（Zl/zl）接口已预留，待业务需要时扩展。
struct SoftConstraintConfig {
    // 每一步软约束维度
    int ns = 0;
    // 松弛变量下界对应的L2权重
    Vector Zl;
    // 松弛变量上界对应的L2权重
    Vector Zu;
    // 松弛变量下界对应的L1权重（精确惩罚）
    Vector zl;
    // 松弛变量上界对应的L1权重
    Vector zu;
    // 被软化的普通约束索引（在 [0, ng) 范围内）
    std::vector<int> idxs;
};

// QP 数据容器：为 OCP QP 提供一块连续、32 字节对齐的内存池
// 所有矩阵/向量均通过 Eigen::Map 视图映射到内存池，禁止拷贝/移动以避免自引用失效
class QPData {
public:
    // 默认构造，用于延迟初始化
    QPData() = default;
    // 按 horizon 长度 N、状态/控制维度 nx/nu、普通约束上限 ng_max 构造内存池
    QPData(int N, int nx, int nu, int ng_max);
    // 彻底禁止拷贝和移动：QPData 内部 Map 指向 memory_pool_，值语义会导致悬垂指针
    QPData(const QPData&) = delete;
    QPData& operator=(const QPData&) = delete;
    QPData(QPData&&) = delete;
    QPData& operator=(QPData&&) = delete;
    // 将内存池全部置零，不触发重新分配
    void reset();
    // 校验各阶段容器的尺寸是否与 N / N+1 匹配
    // 注意：本函数只检查容器数量，不校验 N/nx/nu/ng_max 本身的合法性
    bool hasValidContainerSizes() const;
    // 原始指针访问（供 HPIPM C API 使用）
    double* rawA(int k) { return A[k].data(); }
    const double* rawA(int k) const { return A[k].data(); }
    double* rawB(int k) { return B[k].data(); }
    const double* rawB(int k) const { return B[k].data(); }
    double* rawQ(int k) { return Q[k].data(); }
    const double* rawQ(int k) const { return Q[k].data(); }
    double* rawR(int k) { return R[k].data(); }
    const double* rawR(int k) const { return R[k].data(); }
    double* rawS(int k) { return S[k].data(); }
    const double* rawS(int k) const { return S[k].data(); }
    double* rawb(int k) { return b[k].data(); }
    const double* rawb(int k) const { return b[k].data(); }
    double* rawq(int k) { return q[k].data(); }
    const double* rawq(int k) const { return q[k].data(); }
    double* rawr(int k) { return r[k].data(); }
    const double* rawr(int k) const { return r[k].data(); }
    double* rawC(int k) { return C[k].data(); }
    const double* rawC(int k) const { return C[k].data(); }
    double* rawD(int k) { return D[k].data(); }
    const double* rawD(int k) const { return D[k].data(); }
    double* rawd(int k) { return d[k].data(); }
    const double* rawd(int k) const { return d[k].data(); }
    double* rawLbx(int k) { return lbx[k].data(); }
    const double* rawLbx(int k) const { return lbx[k].data(); }
    double* rawUbx(int k) { return ubx[k].data(); }
    const double* rawUbx(int k) const { return ubx[k].data(); }
    double* rawLbu(int k) { return lbu[k].data(); }
    const double* rawLbu(int k) const { return lbu[k].data(); }
    double* rawUbu(int k) { return ubu[k].data(); }
    const double* rawUbu(int k) const { return ubu[k].data(); }

public:
    // horizon 长度
    int N = 0;
    // 状态维度
    int nx = 0;
    // 控制维度
    int nu = 0;
    // 普通约束维度上限
    int ng_max = 0;
    // 对齐内存池：使用 Eigen 对齐分配器保证起始地址满足 SIMD 要求
    AlignedVector<double> memory_pool_;
    // Eigen::Map 视图（不拥有内存）
    std::vector<Eigen::Map<Matrix>, Eigen::aligned_allocator<Eigen::Map<Matrix>>> A;
    std::vector<Eigen::Map<Matrix>, Eigen::aligned_allocator<Eigen::Map<Matrix>>> B;
    std::vector<Eigen::Map<Matrix>, Eigen::aligned_allocator<Eigen::Map<Matrix>>> Q;
    std::vector<Eigen::Map<Matrix>, Eigen::aligned_allocator<Eigen::Map<Matrix>>> R;
    std::vector<Eigen::Map<Matrix>, Eigen::aligned_allocator<Eigen::Map<Matrix>>> S;
    std::vector<Eigen::Map<Vector>, Eigen::aligned_allocator<Eigen::Map<Vector>>> b;
    std::vector<Eigen::Map<Vector>, Eigen::aligned_allocator<Eigen::Map<Vector>>> q;
    std::vector<Eigen::Map<Vector>, Eigen::aligned_allocator<Eigen::Map<Vector>>> r;
    // 普通约束状态 Jacobian：C = dg/dx；当 ng_max=0 时为零行矩阵
    std::vector<Eigen::Map<Matrix>, Eigen::aligned_allocator<Eigen::Map<Matrix>>> C;
    // 普通约束控制 Jacobian：D = dg/du；当 ng_max=0 时为零行矩阵
    std::vector<Eigen::Map<Matrix>, Eigen::aligned_allocator<Eigen::Map<Matrix>>> D;
    // 普通约束右端项 d
    std::vector<Eigen::Map<Vector>, Eigen::aligned_allocator<Eigen::Map<Vector>>> d;
    std::vector<Eigen::Map<Vector>, Eigen::aligned_allocator<Eigen::Map<Vector>>> lbx;
    std::vector<Eigen::Map<Vector>, Eigen::aligned_allocator<Eigen::Map<Vector>>> ubx;
    std::vector<Eigen::Map<Vector>, Eigen::aligned_allocator<Eigen::Map<Vector>>> lbu;
    std::vector<Eigen::Map<Vector>, Eigen::aligned_allocator<Eigen::Map<Vector>>> ubu;
    // 软约束配置（nullptr 表示无软约束）
    std::unique_ptr<SoftConstraintConfig> soft_config;
};
} // namespace stc_SQP
