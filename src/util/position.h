#pragma once

#include <iomanip>
#include <iostream>
#include <sstream>

#include "apa_post_process.pb.h"
#include "constants.h"

namespace apa_post_processor {
// 位置数据结构
struct Position {
    Position() = default;
    // 使用坐标构造
    Position(double x_val, double y_val) : x(x_val), y(y_val) {}
    // 基于protobuf消息构造
    static Position FromProto(const ::apa::post_processor::Position& proto) {
        return Position{proto.x(), proto.y()};
    }
    // 转化为JSON字符串
    std::string toString() const {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(PRINT_PRECISION)
            << "{\"x\": " << x << ", \"y\": " << y << "}";
        return oss.str();
    }

    // x (m)
    double x{0.0};
    // y (m)
    double y{0.0};
};
}  // namespace apa_post_processor
