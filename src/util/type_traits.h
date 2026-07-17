#pragma once

#include <iterator>
#include <type_traits>

namespace apa_post_processor {
namespace util {
// 获取可迭代容器的元素类型（对非可迭代类型安全回退为 void）
template <typename T, typename = void>
struct iterable_value {
    using type = void;  // 非可迭代类型的回退
};

template <typename T>
struct iterable_value<
    T, std::void_t<decltype(*std::begin(std::declval<const T&>()))>> {
    using type = std::decay_t<decltype(*std::begin(std::declval<const T&>()))>;
};

template <typename T>
using iterable_value_t = typename iterable_value<std::decay_t<T>>::type;

// 判断类型是否为可迭代容器
template <typename T, typename = void>
struct is_iterable : std::false_type {};

template <typename T>
struct is_iterable<T,
                   std::void_t<decltype(std::begin(std::declval<const T&>())),
                               decltype(std::end(std::declval<const T&>()))>>
    : std::true_type {};

// is_iterable 变量模板
template <typename T>
inline constexpr bool is_iterable_v = is_iterable<T>::value;
}  // namespace util
}  // namespace apa_post_processor
