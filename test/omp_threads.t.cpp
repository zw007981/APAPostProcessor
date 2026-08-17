#include <gtest/gtest.h>

#include <omp.h>

#include <algorithm>

#include "util/omp_threads.h"

namespace apa_post_processor {
namespace {
// 测试统一 OMP 线程策略的上下界契约：任何运行环境下全量档结果都
// 必须等于「运行期上限与编译期天花板取小」且落在 [1, 天花板]——
// 该契约是车端部署线程数可控（OMP_NUM_THREADS）与防过 fork
// （天花板）两条路径同时成立的前提。
TEST(OmpThreadsTest, FullThreadsRespectEnvAndCeiling) {
    const int env_limit = omp_get_max_threads();
    const int ceiling = APA_OMP_MAX_THREADS;
    const int full = EffectiveOmpThreads(1, 1);
    EXPECT_EQ(full, std::min(env_limit, ceiling));
    EXPECT_GE(full, 1);
    EXPECT_LE(full, ceiling);
}

// 测试缩放档的单调性与下限钳制：1/4 档不得超过全量档、不低于
// 下限与 1；全量档为 1 的单线程环境下，下限钳制使结果恒为 1，
// 保证策略退化到串行时行为仍然良定义。
TEST(OmpThreadsTest, ScaledThreadsNeverExceedFullAndHonorFloor) {
    const int full = EffectiveOmpThreads(1, 1);
    const int quarter = EffectiveOmpThreads(4, 2);
    EXPECT_LE(quarter, full);
    EXPECT_GE(quarter, 1);
    if (full >= 2) {
        EXPECT_GE(quarter, 2);
    }
    const int single = EffectiveOmpThreads(4, 1);
    EXPECT_GE(single, 1);
    EXPECT_LE(single, full);
}
}  // namespace
}  // namespace apa_post_processor
