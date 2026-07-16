#pragma once

#include <string>
#include <vector>

#include "../util/trajectory_point.h"
#include "../vehicle/vehicle_params.h"

namespace apa_post_processor {
// 微分平坦解析补全器配置。
struct DifferentialFlatnessSolverConfig {
    // 曲率分母死区保护下限，避免浮点溢出
    double curvature_denominator_epsilon = 1e-6;
};

// 微分平坦解析补全输入：密集配点的几何导数与速度规划结果。
struct DifferentialFlatnessInput {
    // 世界坐标 x (m)
    std::vector<double> x;
    // 世界坐标 y (m)
    std::vector<double> y;
    // 航向角 theta (rad)
    std::vector<double> theta;
    // B 样条一阶导数 x'(u)
    std::vector<double> x_d1;
    // B 样条二阶导数 x''(u)
    std::vector<double> x_d2;
    // B 样条三阶导数 x'''(u)
    std::vector<double> x_d3;
    // B 样条一阶导数 y'(u)
    std::vector<double> y_d1;
    // B 样条二阶导数 y''(u)
    std::vector<double> y_d2;
    // B 样条三阶导数 y'''(u)
    std::vector<double> y_d3;
    // 带符号纵向速度 v (m/s)
    std::vector<double> v;
    // 带符号纵向加速度 a (m/s^2)
    std::vector<double> a;
    // 物理时间戳 t (s)
    std::vector<double> t;
};

// 微分平坦解析补全结果。
struct DifferentialFlatnessResult {
    bool success = false;
    std::string status_msg;
    // 完整轨迹点序列
    std::vector<TrajectoryPoint> points;
};

// 微分平坦解析补全器：由 B 样条几何导数与速度剖面反推 delta 与 delta_dot。
class DifferentialFlatnessSolver {
   public:
    explicit DifferentialFlatnessSolver(
        const DifferentialFlatnessSolverConfig& config);
    // 对单个连续机动段做微分平坦补全。
    DifferentialFlatnessResult solve(const DifferentialFlatnessInput& input,
                                     const VehicleParams& vehicle_params) const;

   protected:
    // 验证输入合法性
    void validateInputs(const DifferentialFlatnessInput& input,
                        const VehicleParams& vehicle_params) const;
    // 逐点计算 delta 与 delta_dot 并组装序列
    DifferentialFlatnessResult computeFlatOutputs(
        const DifferentialFlatnessInput& input,
        const VehicleParams& vehicle_params) const;
    // 对单个点执行微分平坦公式
    std::pair<double, double> computeSinglePoint(
        double x_d1, double x_d2, double x_d3, double y_d1, double y_d2,
        double y_d3, double v, double wheelbase,
        double denominator_epsilon) const;

   protected:
    DifferentialFlatnessSolverConfig config_;
};
}  // namespace apa_post_processor
