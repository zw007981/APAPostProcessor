#pragma once

#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>

#include "constants.h"
#include "pose.h"

namespace apa_post_processor {
// 路径点类，在位姿和基础上增加曲率和运动学模型中要求的额外状态和控制量。
// 这些额外信息默认为NaN，获取前需要保证已经设置，否则会抛出异常。
class PathPoint : public Pose {
   public:
    PathPoint() = default;
    // 使用x、y、theta构造，其余派生量默认未设置(NaN)
    PathPoint(double x_val, double y_val, double theta_val)
        : Pose(x_val, y_val, theta_val) {}
    // 基于纯几何Pose构造，其余派生量默认未设置(NaN)
    explicit PathPoint(const Pose& pose) : Pose(pose) {}
    // 判断曲率是否已设置
    bool hasKappa() const { return !std::isnan(kappa_); }
    // 获取曲率(1/m)，未设置时抛出std::logic_error
    double getKappa() const {
        if (!hasKappa()) {
            throw std::logic_error("PathPoint::getKappa: kappa is not set!!!");
        }
        return kappa_;
    }
    // 设置曲率
    void setKappa(double kappa) { kappa_ = kappa; }
    // 判断纵向速度是否已设置
    bool hasV() const { return !std::isnan(v_); }
    // 获取纵向速度(m/s)，未设置时抛出std::logic_error
    double getV() const {
        if (!hasV()) {
            throw std::logic_error("PathPoint::getV: v is not set!!!");
        }
        return v_;
    }
    // 设置纵向速度
    void setV(double v) { v_ = v; }
    // 判断前轮转角是否已设置
    bool hasDelta() const { return !std::isnan(delta_); }
    // 获取前轮转角(rad)，未设置时抛出std::logic_error
    double getDelta() const {
        if (!hasDelta()) {
            throw std::logic_error("PathPoint::getDelta: delta is not set!!!");
        }
        return delta_;
    }
    // 设置前轮转角
    void setDelta(double delta) { delta_ = delta; }
    // 判断纵向加速度是否已设置
    bool hasA() const { return !std::isnan(a_); }
    // 获取纵向加速度(m/s^2)，未设置时抛出std::logic_error
    double getA() const {
        if (!hasA()) {
            throw std::logic_error("PathPoint::getA: a is not set!!!");
        }
        return a_;
    }
    // 设置纵向加速度
    void setA(double a) { a_ = a; }
    // 判断前轮转角变化率是否已设置
    bool hasDeltaDot() const { return !std::isnan(delta_dot_); }
    // 获取前轮转角变化率(rad/s)，未设置时抛出std::logic_error
    double getDeltaDot() const {
        if (!hasDeltaDot()) {
            throw std::logic_error(
                "PathPoint::getDeltaDot: delta_dot is not set!!!");
        }
        return delta_dot_;
    }
    // 设置前轮转角变化率
    void setDeltaDot(double delta_dot) { delta_dot_ = delta_dot; }
    // 把路径点信息转化为json格式的字符串，未设置的派生量不会出现在输出中
    std::string toString() const {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(PRINT_PRECISION) << "{\"x\": " << x
            << ", \"y\": " << y << ", \"theta\": " << theta;
        if (hasKappa()) {
            oss << ", \"kappa\": " << kappa_;
        }
        if (hasV()) {
            oss << ", \"v\": " << v_;
        }
        if (hasDelta()) {
            oss << ", \"delta\": " << delta_;
        }
        if (hasA()) {
            oss << ", \"a\": " << a_;
        }
        if (hasDeltaDot()) {
            oss << ", \"delta_dot\": " << delta_dot_;
        }
        oss << "}";
        return oss.str();
    }

   protected:
    // 有向曲率(1/m)：仅经过Path曲率估计的路径点才有意义，未设置时为NaN
    double kappa_{std::numeric_limits<double>::quiet_NaN()};
    // 纵向速度(m/s)：仅NMPC优化轨迹的状态点才具备，未设置时为NaN
    double v_{std::numeric_limits<double>::quiet_NaN()};
    // 前轮转角(rad)：仅NMPC优化轨迹的状态点才具备，未设置时为NaN
    double delta_{std::numeric_limits<double>::quiet_NaN()};
    // 纵向加速度(m/s^2)：仅NMPC优化轨迹的控制量对应点才具备，未设置时为NaN
    double a_{std::numeric_limits<double>::quiet_NaN()};
    // 前轮转角变化率(rad/s)：仅NMPC优化轨迹的控制量对应点才具备，未设置时为NaN
    double delta_dot_{std::numeric_limits<double>::quiet_NaN()};
};
}  // namespace apa_post_processor
