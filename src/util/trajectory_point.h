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
// 路径点：在位姿基础上增加曲率及运动学状态/控制量。未设置的量默认为NaN，获取时抛出异常。
class TrajectoryPoint : public Pose {
   public:
    TrajectoryPoint() = default;
    // 使用坐标和航向角构造，派生量默认未设置
    TrajectoryPoint(double x_val, double y_val, double theta_val)
        : Pose(x_val, y_val, theta_val) {}
    // 基于Pose构造，派生量默认未设置
    explicit TrajectoryPoint(const Pose& pose) : Pose(pose) {}
    // 曲率是否已设置
    bool hasKappa() const { return !std::isnan(kappa_); }
    // 获取曲率 (1/m)，未设置时抛出
    double getKappa() const {
        if (!hasKappa()) {
            throw std::logic_error(
                "TrajectoryPoint::getKappa: kappa is not set!!!");
        }
        return kappa_;
    }
    // 设置曲率
    void setKappa(double kappa) { kappa_ = kappa; }
    // 纵向速度是否已设置
    bool hasV() const { return !std::isnan(v_); }
    // 获取纵向速度 (m/s)，未设置时抛出
    double getV() const {
        if (!hasV()) {
            throw std::logic_error("TrajectoryPoint::getV: v is not set!!!");
        }
        return v_;
    }
    // 设置纵向速度
    void setV(double v) { v_ = v; }
    // 前轮转角是否已设置
    bool hasDelta() const { return !std::isnan(delta_); }
    // 获取前轮转角 (rad)，未设置时抛出
    double getDelta() const {
        if (!hasDelta()) {
            throw std::logic_error(
                "TrajectoryPoint::getDelta: delta is not set!!!");
        }
        return delta_;
    }
    // 设置前轮转角
    void setDelta(double delta) { delta_ = delta; }
    // 纵向加速度是否已设置
    bool hasA() const { return !std::isnan(a_); }
    // 获取纵向加速度 (m/s^2)，未设置时抛出
    double getA() const {
        if (!hasA()) {
            throw std::logic_error("TrajectoryPoint::getA: a is not set!!!");
        }
        return a_;
    }
    // 设置纵向加速度
    void setA(double a) { a_ = a; }
    // 前轮转角变化率是否已设置
    bool hasDeltaDot() const { return !std::isnan(delta_dot_); }
    // 获取前轮转角变化率 (rad/s)，未设置时抛出
    double getDeltaDot() const {
        if (!hasDeltaDot()) {
            throw std::logic_error(
                "TrajectoryPoint::getDeltaDot: delta_dot is not set!!!");
        }
        return delta_dot_;
    }
    // 设置前轮转角变化率
    void setDeltaDot(double delta_dot) { delta_dot_ = delta_dot; }
    // 时间戳是否已设置
    bool hasT() const { return !std::isnan(t_); }
    // 获取时间戳 (s)，未设置时抛出
    double getT() const {
        if (!hasT()) {
            throw std::logic_error("TrajectoryPoint::getT: t is not set!!!");
        }
        return t_;
    }
    // 设置时间戳
    void setT(double t) { t_ = t; }
    // 转化为JSON字符串，未设置的量不输出
    std::string toString() const {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(PRINT_PRECISION)
            << "{\"x\": " << x << ", \"y\": " << y << ", \"theta\": " << theta;
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
        if (hasT()) {
            oss << ", \"t\": " << t_;
        }
        oss << "}";
        return oss.str();
    }

   protected:
    // 有向曲率 (1/m)，未设置时为NaN
    double kappa_{std::numeric_limits<double>::quiet_NaN()};
    // 纵向速度 (m/s)，未设置时为NaN
    double v_{std::numeric_limits<double>::quiet_NaN()};
    // 前轮转角 (rad)，未设置时为NaN
    double delta_{std::numeric_limits<double>::quiet_NaN()};
    // 纵向加速度 (m/s^2)，未设置时为NaN
    double a_{std::numeric_limits<double>::quiet_NaN()};
    // 前轮转角变化率 (rad/s)，未设置时为NaN
    double delta_dot_{std::numeric_limits<double>::quiet_NaN()};
    // 时间戳 (s)，未设置时为NaN
    double t_{std::numeric_limits<double>::quiet_NaN()};
};
}  // namespace apa_post_processor
