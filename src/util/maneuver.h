#pragma once

#include <cmath>
#include <functional>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "logger.h"
#include "path_point.h"

namespace apa_post_processor {
template <typename T>
struct UnsupportedManeuverInputType : std::false_type {};

// 方向枚举类
enum class Direction {
    // 未知/未推断
    UNKNOWN = 0,
    // 前进
    FORWARD = 1,
    // 后退
    BACKWARD = 2,
    // 钟摆泊车等特殊场景下的原地转向
    PIVOT = 3
};

// 机动段数据结构
struct Maneuver {
    Maneuver() = default;
    // 输入一系列PathPoint或者单个PathPoint构造Maneuver，方向只接收外部显式指定值
    template <typename T>
    explicit Maneuver(T&& input, Direction dir = Direction::UNKNOWN) {
        using DecayedType = std::decay_t<T>;
        if constexpr (std::is_same_v<DecayedType, std::vector<PathPoint>>) {
            points = std::forward<T>(input);
            if (points.empty()) {
                LOG_ERROR(
                    "Maneuver constructor received empty points vector!!!");
                throw std::invalid_argument(
                    "Maneuver constructor received empty points vector");
            }
        } else if constexpr (std::is_same_v<DecayedType, PathPoint>) {
            points.emplace_back(std::forward<T>(input));
        } else {
            static_assert(
                UnsupportedManeuverInputType<DecayedType>::value,
                "Unsupported type for Maneuver constructor, see "
                "UnsupportedManeuverInputType<T> in compiler diagnostic!!!");
        }
        direction = dir;
    }
    // 此机动段包含几个节点
    std::size_t size() const { return points.size(); }
    // 此机动段的长度
    double length() const {
        return (points.size() < 2)
                   ? 0.0
                   : std::transform_reduce(
                         std::next(points.cbegin()), points.cend(),
                         points.cbegin(), 0.0, std::plus<>(),
                         [](const auto& p1, const auto& p2) {
                             return std::hypot(p1.x - p2.x, p1.y - p2.y);
                         });
    }
    // 迭代器接口
    std::vector<PathPoint>::iterator begin() { return points.begin(); }
    std::vector<PathPoint>::iterator end() { return points.end(); }
    std::vector<PathPoint>::const_iterator begin() const { return points.begin(); }
    std::vector<PathPoint>::const_iterator end() const { return points.end(); }
    // 把机动段信息转化为json格式的字符串
    std::string toString() const {
        std::ostringstream oss;
        static constexpr const char* DIR_NAMES[] = {"UNKNOWN", "FORWARD",
                                                    "BACKWARD", "PIVOT"};
        int dir_idx = static_cast<int>(direction);
        const char* dir_str =
            (dir_idx >= 0 && dir_idx <= 3) ? DIR_NAMES[dir_idx] : "UNKNOWN";
        oss << "{\"direction\": \"" << dir_str << "\", \"points\": [";
        const char* delim = "";
        for (const auto& point : points) {
            oss << delim << point.toString();
            delim = ", ";
        }
        oss << "]}";
        return oss.str();
    }

    // 机动段方向
    Direction direction{Direction::UNKNOWN};
    // 机动段包含的路径点序列
    std::vector<PathPoint> points;
};
}  // namespace apa_post_processor
