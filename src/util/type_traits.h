#pragma once

#include <iterator>
#include <type_traits>

namespace apa_post_processor {
namespace util {
// 编译期类型萃取：获取可迭代容器的元素类型（decay 后）。
template <typename T>
using iterable_value_t =
    std::decay_t<decltype(*std::begin(std::declval<const std::decay_t<T>&>()))>;

// 编译期类型萃取：判断类型是否为可迭代容器（支持 std::begin/std::end）。
template <typename T, typename = void>
struct is_iterable : std::false_type {};

template <typename T>
struct is_iterable<T,
                   std::void_t<decltype(std::begin(std::declval<const T&>())),
                               decltype(std::end(std::declval<const T&>()))>>
    : std::true_type {};

// is_iterable 的便捷变量模板。
template <typename T>
inline constexpr bool is_iterable_v = is_iterable<T>::value;
}  // namespace util
}  // namespace apa_post_processor
