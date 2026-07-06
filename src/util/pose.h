#pragma once

#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>

#include "apa_post_process.pb.h"
#include "constants.h"

namespace apa_post_processor {
// 位姿数据结构
struct Pose {
    Pose() = default;
    // 使用x、y坐标和航向角构造Pose结构体，此处角度会被规范化到[-pi, pi)范围内
    Pose(double x_val, double y_val, double theta_val)
        : x(x_val), y(y_val), theta(std::remainder(theta_val, 2.0 * PI)) {}
    // 基于protobuf消息构造Pose结构体
    static Pose FromProto(const ::apa::post_processor::Pose& proto) {
        return Pose{proto.x(), proto.y(), proto.theta()};
    }
    // 把位姿信息转化为json格式的字符串
    std::string toString() const {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(PRINT_PRECISION)
            << "{\"x\": " << x << ", \"y\": " << y << ", \"theta\": " << theta
            << "}";
        return oss.str();
    }

    // x坐标（m）
    double x{0.0};
    // y坐标（m）
    double y{0.0};
    // 航向角（rad）
    double theta{0.0};
};
}  // namespace apa_post_processor
