#include <gtest/gtest.h>

#include <type_traits>

#include "core/NMPC/nmpc_config.h"
#include "core/post_processor.h"
#include "util/config.h"

namespace apa_post_processor {
namespace {

// 基类自身的拷贝/移动构造与赋值对外不可见（protected），从外部直接拷贝或
// 移动基类对象会被编译期拒绝，杜绝 `Config sliced = derived;` 这类按值切片
static_assert(!std::is_copy_constructible_v<Config>);
static_assert(!std::is_copy_assignable_v<Config>);
static_assert(!std::is_move_constructible_v<Config>);
static_assert(!std::is_move_assignable_v<Config>);
// 派生类的隐式特殊成员可调用 protected 基类版本，按值传递/拷贝不受影响
static_assert(std::is_copy_constructible_v<NMPCConfig>);
static_assert(std::is_copy_assignable_v<NMPCConfig>);
static_assert(std::is_move_constructible_v<NMPCConfig>);
static_assert(std::is_copy_constructible_v<MincoConfig>);
static_assert(std::is_copy_assignable_v<MincoConfig>);
static_assert(std::is_move_constructible_v<MincoConfig>);

// 派生配置对象按值拷贝后字段完整（基类 protected 拷贝不影响派生类使用）
TEST(ConfigTest, DerivedConfigsRemainCopyable) {
    NMPCConfig nmpc_src;
    nmpc_src.max_iter = 7;
    const NMPCConfig nmpc_dst = nmpc_src;
    EXPECT_EQ(nmpc_dst.max_iter, 7);
    MincoConfig minco_src;
    minco_src.max_velocity = 1.5;
    const MincoConfig minco_dst = minco_src;
    EXPECT_DOUBLE_EQ(minco_dst.max_velocity, 1.5);
}

}  // namespace
}  // namespace apa_post_processor
