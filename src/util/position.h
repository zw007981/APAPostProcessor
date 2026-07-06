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
    // 使用x、y坐标构造Position结构体
    Position(double x_val, double y_val) : x(x_val), y(y_val) {}
    // 基于protobuf消息构造Position结构体
    static Position FromProto(const ::apa::post_processor::Position& proto) {
        return Position{proto.x(), proto.y()};
    }
    // 把位置信息转化为json格式的字符串
    std::string toString() const {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(PRINT_PRECISION)
            << "{\"x\": " << x << ", \"y\": " << y << "}";
        return oss.str();
    }

    // x坐标（m）
    double x{0.0};
    // y坐标（m）
    double y{0.0};
};
}  // namespace apa_post_processor
