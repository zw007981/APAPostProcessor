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
#include "trajectory_point.h"

namespace apa_post_processor {
template <typename T>
struct UnsupportedManeuverInputType : std::false_type {};

// 行驶方向
enum class Direction { UNKNOWN = 0, FORWARD = 1, BACKWARD = 2, PIVOT = 3 };

// 机动段
struct Maneuver {
    Maneuver() = default;
    // 显式默认拷贝/移动，防止转发引用模板构造函数劫持
    Maneuver(const Maneuver&) = default;
    Maneuver(Maneuver&&) = default;
    Maneuver& operator=(const Maneuver&) = default;
    Maneuver& operator=(Maneuver&&) = default;
    // 输入PathPoint序列构造，方向仅接收外部指定值
    template <
        typename T,
        std::enable_if_t<!std::is_same_v<std::decay_t<T>, Maneuver>, int> = 0>
    explicit Maneuver(T&& input, Direction dir = Direction::UNKNOWN) {
        using DecayedType = std::decay_t<T>;
        if constexpr (std::is_same_v<DecayedType,
                                     std::vector<TrajectoryPoint>>) {
            points = std::forward<T>(input);
            if (points.empty()) {
                LOG_ERROR(
                    "Maneuver constructor received empty points vector!!!");
                throw std::invalid_argument(
                    "Maneuver constructor received empty points vector");
            }
        } else if constexpr (std::is_same_v<DecayedType, TrajectoryPoint>) {
            points.emplace_back(std::forward<T>(input));
        } else {
            static_assert(
                UnsupportedManeuverInputType<DecayedType>::value,
                "Unsupported type for Maneuver constructor, see "
                "UnsupportedManeuverInputType<T> in compiler diagnostic!!!");
        }
        direction = dir;
    }
    // 此段包含的节点数
    std::size_t size() const { return points.size(); }
    // 此段的弧长
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
    std::vector<TrajectoryPoint>::iterator begin() { return points.begin(); }
    std::vector<TrajectoryPoint>::iterator end() { return points.end(); }
    std::vector<TrajectoryPoint>::const_iterator begin() const {
        return points.begin();
    }
    std::vector<TrajectoryPoint>::const_iterator end() const {
        return points.end();
    }
    // 转化为JSON字符串
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

    // 行驶方向
    Direction direction{Direction::UNKNOWN};
    // 路径点序列
    std::vector<TrajectoryPoint> points;
};
}  // namespace apa_post_processor
