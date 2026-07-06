#pragma once

#include <string>
#include <vector>

#include "../util/path_point.h"
#include "../vehicle/vehicle_params.h"

namespace apa_post_processor {

// 微分平坦解析补全器配置：控制数值死区保护与异常行为。
// 字段默认值来源 docs/default_params.md 与 NMPC.md 3.3 节。
struct DifferentialFlatnessSolverConfig {
    // 曲率弧长导数分母 (x'^2 + y'^2)^3 的死区保护下限，
    // 避免极低速或高频采样处出现浮点溢出。
    double curvature_denominator_epsilon = 1e-6;
};

// 微分平坦解析补全输入：每个密集配点对应的几何导数与速度规划结果。
// 调用方（预处理管线）负责从 BSplineSmoother 的 DensePointData 提取几何导数，
// 并从 SpeedProfilePlanner 的 SpeedProfileResult 提取带符号速度/加速度/时间戳。
struct DifferentialFlatnessInput {
    // 各配点的世界坐标 x (m)
    std::vector<double> x;
    // 各配点的世界坐标 y (m)
    std::vector<double> y;
    // 各配点的航向角 theta (rad)
    std::vector<double> theta;
    // B 样条参数 u 处的一阶导数 x'(u)
    std::vector<double> x_d1;
    // B 样条参数 u 处的二阶导数 x''(u)
    std::vector<double> x_d2;
    // B 样条参数 u 处的三阶导数 x'''(u)
    std::vector<double> x_d3;
    // B 样条参数 u 处的一阶导数 y'(u)
    std::vector<double> y_d1;
    // B 样条参数 u 处的二阶导数 y''(u)
    std::vector<double> y_d2;
    // B 样条参数 u 处的三阶导数 y'''(u)
    std::vector<double> y_d3;
    // 各配点的带符号纵向速度 v (m/s)
    std::vector<double> v;
    // 各配点的带符号纵向加速度 a (m/s^2)
    std::vector<double> a;
    // 各配点的物理时间戳 t (s)，t_0 = 0
    std::vector<double> t;
};

// 微分平坦解析补全结果：完整的状态/控制序列。
struct DifferentialFlatnessResult {
    // 是否成功完成补全
    bool success = false;
    // 状态信息
    std::string status_msg;
    // 完整轨迹点序列，每个 PathPoint 携带 x, y, theta, v, a, delta, delta_dot
    std::vector<PathPoint> points;
};

// 状态与控制量解析补全器：基于微分平坦性质，通过纯代数解析运算
// 由 B 样条几何导数与速度剖面反推前轮偏角 delta 及其变化率 delta_dot，
// 输出 NMPC 所需的完整 Z_ref / U_ref 密集序列。
class DifferentialFlatnessSolver {
   public:
    explicit DifferentialFlatnessSolver(
        const DifferentialFlatnessSolverConfig& config);
    // 对单个连续机动段（或全局拼接的多段）做微分平坦补全。
    // @param input         密集配点的几何导数与速度/加速度/时间戳
    // @param vehicle_params 车辆参数，读取 wheelbase 计算转向角
    // @return 补全结果，包含完整的 PathPoint 序列
    DifferentialFlatnessResult solve(
        const DifferentialFlatnessInput& input,
        const VehicleParams& vehicle_params) const;

   protected:
    // 验证输入维度、参数合法性与车辆参数可用性
    void validateInputs(const DifferentialFlatnessInput& input,
                        const VehicleParams& vehicle_params) const;
    // 逐点计算 delta 与 delta_dot，并组装为 PathPoint 序列
    DifferentialFlatnessResult computeFlatOutputs(
        const DifferentialFlatnessInput& input,
        const VehicleParams& vehicle_params) const;
    // 对单个点执行微分平坦公式，返回 [delta, delta_dot]
    std::pair<double, double> computeSinglePoint(
        double x_d1, double x_d2, double x_d3, double y_d1, double y_d2,
        double y_d3, double v, double wheelbase,
        double denominator_epsilon) const;

   protected:
    DifferentialFlatnessSolverConfig config_;
};

}  // namespace apa_post_processor
