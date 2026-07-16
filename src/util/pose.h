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
    // 使用坐标和航向角构造，角度自动规范化到[-pi, pi)
    Pose(double x_val, double y_val, double theta_val)
        : x(x_val), y(y_val), theta(std::remainder(theta_val, 2.0 * PI)) {}
    // 基于protobuf消息构造
    static Pose FromProto(const ::apa::post_processor::Pose& proto) {
        return Pose{proto.x(), proto.y(), proto.theta()};
    }
    // 转化为JSON字符串
    std::string toString() const {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(PRINT_PRECISION)
            << "{\"x\": " << x << ", \"y\": " << y << ", \"theta\": " << theta
            << "}";
        return oss.str();
    }

    // x (m)
    double x{0.0};
    // y (m)
    double y{0.0};
    // 航向角 (rad)
    double theta{0.0};
};
}  // namespace apa_post_processor
