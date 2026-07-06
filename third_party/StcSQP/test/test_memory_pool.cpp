#include <cstdint>
#include <stdexcept>
#include <tuple>
#include <vector>

#include <gtest/gtest.h>

#include "../src/qp/qp_data.h"

using namespace stc_SQP;

// 测试目的：验证 QPData 内存池中所有 Eigen::Map 视图的起始地址均满足 32 字节对齐
// 流程：构造不同维度的 QPData，通过 raw* 接口取裸指针并做取模检查
// 预期效果：所有指针地址 % 32 == 0，且 reset 后池内数据归零
TEST(MemoryPool, AllMapPointersAre32ByteAligned) {
    const std::vector<std::tuple<int, int, int, int>> configs = {
        { 5, 2, 1, 1 },
        { 10, 4, 2, 4 },
        { 3, 5, 3, 40 },
        { 7, 1, 1, 0 },
    };
    for (const auto& cfg : configs) {
        const int N = std::get<0>(cfg);
        const int nx = std::get<1>(cfg);
        const int nu = std::get<2>(cfg);
        const int ng = std::get<3>(cfg);
        QPData data(N, nx, nu, ng);
        for (int k = 0; k < N; ++k) {
            EXPECT_EQ(reinterpret_cast<uintptr_t>(data.rawA(k)) % 32, 0) << "rawA(" << k << ") not aligned";
            EXPECT_EQ(reinterpret_cast<uintptr_t>(data.rawB(k)) % 32, 0) << "rawB(" << k << ") not aligned";
            EXPECT_EQ(reinterpret_cast<uintptr_t>(data.rawQ(k)) % 32, 0) << "rawQ(" << k << ") not aligned";
            EXPECT_EQ(reinterpret_cast<uintptr_t>(data.rawR(k)) % 32, 0) << "rawR(" << k << ") not aligned";
            EXPECT_EQ(reinterpret_cast<uintptr_t>(data.rawS(k)) % 32, 0) << "rawS(" << k << ") not aligned";
            EXPECT_EQ(reinterpret_cast<uintptr_t>(data.rawb(k)) % 32, 0) << "rawb(" << k << ") not aligned";
            EXPECT_EQ(reinterpret_cast<uintptr_t>(data.rawq(k)) % 32, 0) << "rawq(" << k << ") not aligned";
            EXPECT_EQ(reinterpret_cast<uintptr_t>(data.rawr(k)) % 32, 0) << "rawr(" << k << ") not aligned";
            EXPECT_EQ(reinterpret_cast<uintptr_t>(data.rawC(k)) % 32, 0) << "rawC(" << k << ") not aligned";
            EXPECT_EQ(reinterpret_cast<uintptr_t>(data.rawD(k)) % 32, 0) << "rawD(" << k << ") not aligned";
            EXPECT_EQ(reinterpret_cast<uintptr_t>(data.rawd(k)) % 32, 0) << "rawd(" << k << ") not aligned";
            EXPECT_EQ(reinterpret_cast<uintptr_t>(data.rawLbu(k)) % 32, 0) << "rawLbu(" << k << ") not aligned";
            EXPECT_EQ(reinterpret_cast<uintptr_t>(data.rawUbu(k)) % 32, 0) << "rawUbu(" << k << ") not aligned";
        }
        for (int k = 0; k <= N; ++k) {
            EXPECT_EQ(reinterpret_cast<uintptr_t>(data.rawQ(k)) % 32, 0) << "rawQ terminal not aligned";
            EXPECT_EQ(reinterpret_cast<uintptr_t>(data.rawq(k)) % 32, 0) << "rawq terminal not aligned";
            EXPECT_EQ(reinterpret_cast<uintptr_t>(data.rawLbx(k)) % 32, 0) << "rawLbx(" << k << ") not aligned";
            EXPECT_EQ(reinterpret_cast<uintptr_t>(data.rawUbx(k)) % 32, 0) << "rawUbx(" << k << ") not aligned";
        }
    }
}

// 测试目的：验证 raw* 接口返回的裸指针与对应 Eigen::Map 的数据指针一致
// 流程：写入 Map，检查 raw 指针处值是否同步变化
// 预期效果：两者指向同一段内存
TEST(MemoryPool, RawPointersMatchEigenMapData) {
    QPData data(3, 2, 1, 2);
    data.A[0].setConstant(3.0);
    EXPECT_DOUBLE_EQ(data.rawA(0)[0], 3.0);
    data.rawB(0)[0] = 7.0;
    EXPECT_DOUBLE_EQ(data.B[0](0, 0), 7.0);
    data.D[0].setConstant(5.0);
    EXPECT_DOUBLE_EQ(data.rawD(0)[0], 5.0);
}

// 测试目的：验证 reset() 将内存池归零且不重新分配底层内存
// 流程：先写入非零值，调用 reset，检查所有 Map 元素为 0，且 capacity 不变
// 预期效果：reset 后数值全零，内存池指针/容量稳定
TEST(MemoryPool, ResetZerosMemoryWithoutReallocation) {
    QPData data(4, 3, 2, 5);
    const void* old_ptr = data.memory_pool_.data();
    for (auto& block : data.A) {
        block.setConstant(1.0);
    }
    data.reset();
    for (int k = 0; k < data.N; ++k) {
        EXPECT_TRUE(data.A[k].isZero());
        EXPECT_TRUE(data.B[k].isZero());
        EXPECT_TRUE(data.D[k].isZero());
        EXPECT_TRUE(data.q[k].isZero());
    }
    EXPECT_EQ(data.memory_pool_.data(), old_ptr);
}

// 测试目的：验证 QPData 构造函数对非法维度抛出 invalid_argument
// 流程：分别传入 N <= 0、nx <= 0、nu <= 0、ng_max < 0，期望抛出异常
// 预期效果：所有非法维度组合均触发 std::invalid_argument
TEST(MemoryPool, RejectsInvalidDimensions) {
    EXPECT_THROW(QPData(0, 2, 1, 1), std::invalid_argument);
    EXPECT_THROW(QPData(5, 0, 1, 1), std::invalid_argument);
    EXPECT_THROW(QPData(5, 2, 0, 1), std::invalid_argument);
    EXPECT_THROW(QPData(5, 2, 1, -1), std::invalid_argument);
}
