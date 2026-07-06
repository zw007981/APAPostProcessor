#include <gtest/gtest.h>

#include "../src/qp/qp_solution.h"

using namespace stc_SQP;

// 测试目的：锁定 QPSolution::resize() 的 slack 分配语义
// 流程：调用 resize(N, nx, nu, ns) 并检查 x/u/s 的 stage 数量
// 预期效果：x 为 N+1 段，u 为 N 段，s 为 N 段（终端阶段不使用 slack）
TEST(QPSolution, ResizeAllocatesNStageSlacks) {
    const int N = 5, nx = 2, nu = 1, ns = 3;
    QPSolution sol;
    sol.resize(N, nx, nu, ns);
    EXPECT_EQ(static_cast<int>(sol.x.size()), N + 1);
    EXPECT_EQ(static_cast<int>(sol.u.size()), N);
    EXPECT_EQ(static_cast<int>(sol.s.size()), N);
    for (int k = 0; k <= N; ++k) {
        EXPECT_EQ(sol.x[k].size(), nx);
    }
    for (int k = 0; k < N; ++k) {
        EXPECT_EQ(sol.u[k].size(), nu);
        EXPECT_EQ(sol.s[k].size(), ns);
    }
}

// 测试目的：验证 ns == 0 时 QPSolution 不分配 slack
// 流程：调用 resize(N, nx, nu, 0)
// 预期效果：s 为空 vector
TEST(QPSolution, ResizeClearsSlacksWhenNsZero) {
    QPSolution sol;
    sol.resize(5, 2, 1, 0);
    EXPECT_TRUE(sol.s.empty());
}
