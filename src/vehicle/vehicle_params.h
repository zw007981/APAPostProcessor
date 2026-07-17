#pragma once

#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>

#include "../util/constants.h"
#include "../util/logger.h"
#include "apa_post_process.pb.h"

namespace apa_post_processor {
// 车辆参数
struct VehicleParams {
    VehicleParams() = default;
    // 使用物理参数构造
    VehicleParams(double length_val, double width_val, double wheelbase_val,
                  double max_steer_angle_val, double rear_overhang_val = 0.0,
                  double max_accel_val = 1.5, double max_decel_val = -3.0,
                  double max_steer_rate_val = 0.4)
        : length(length_val),
          width(width_val),
          wheelbase(wheelbase_val),
          max_steer_angle(max_steer_angle_val),
          rear_overhang(rear_overhang_val),
          max_accel(max_accel_val),
          max_decel(max_decel_val),
          max_steer_rate(max_steer_rate_val),
          max_kappa(std::abs(wheelbase_val) > EPSILON
                        ? std::tan(max_steer_angle_val) / wheelbase_val
                        : 0.0) {}
    // 基于protobuf消息构造
    static VehicleParams FromProto(
        const ::apa::post_processor::VehicleParams& proto) {
        auto params = VehicleParams{
            proto.length(),
            proto.width(),
            proto.wheelbase(),
            proto.max_steer_angle(),
            proto.rear_overhang(),
            proto.has_max_accel() ? proto.max_accel() : 1.5,
            proto.has_max_decel() ? proto.max_decel() : -3.0,
            proto.has_max_steer_rate() ? proto.max_steer_rate() : 0.4};
        if (params.length < EPSILON || params.width < EPSILON ||
            params.wheelbase < EPSILON || params.max_steer_angle < EPSILON ||
            params.rear_overhang < 0.0) {
            LOG_FMT_ERROR(
                "VehicleParams constructed from protobuf contains invalid "
                "dimensions, protobuf msg: {}!!!",
                proto.ShortDebugString());
            throw std::invalid_argument(
                "VehicleParams constructed from protobuf contains invalid "
                "dimensions!!!");
        }
        return params;
    }
    // 转化为JSON字符串
    std::string toString() const {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(PRINT_PRECISION)
            << "{\"length\": " << length << ", \"width\": " << width
            << ", \"wheelbase\": " << wheelbase
            << ", \"max_steer_angle\": " << max_steer_angle
            << ", \"rear_overhang\": " << rear_overhang
            << ", \"max_accel\": " << max_accel
            << ", \"max_decel\": " << max_decel
            << ", \"max_steer_rate\": " << max_steer_rate
            << ", \"max_kappa\": " << max_kappa << "}";
        return oss.str();
    }

    // 长度 (m)
    double length{0.0};
    // 宽度 (m)
    double width{0.0};
    // 轴距 (m)
    double wheelbase{0.0};
    // 最大前轮偏角 (rad)
    double max_steer_angle{0.0};
    // 后轴到后保险杠距离 (m)
    double rear_overhang{0.0};
    // 最大纵向加速度 (m/s^2)
    double max_accel{1.5};
    // 最大纵向减速度 (m/s^2)
    double max_decel{-3.0};
    // 最大前轮偏角速度 (rad/s)
    double max_steer_rate{0.4};
    // 最大曲率
    double max_kappa{0.0};
};
}  // namespace apa_post_processor
