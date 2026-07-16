#pragma once

#include <vector>

#include "../vehicle/vehicle_params.h"

namespace apa_post_processor {
// 速度规划配置：s 域 v^2 QP 的目标权重、速度上限与数值保护参数。
struct SpeedProfilePlannerConfig {
    // 前进极限速度 (m/s)
    double max_v_forward = 1.389;
    // 倒车极限速度 (m/s)
    double max_v_reverse = 1.0;
    // 空间加速度变化率上限
    double max_jerk_proxy = 3.0;
    // 贴合参考速度权重
    double weight_v_ref = 10.0;
    // 绝对加速度惩罚权重
    double weight_a_sq = 2.0;
    // Jerk 惩罚权重
    double weight_jerk_sq = 80.0;
    // 时间戳反积分死区保护 (m/s)
    double time_reintegration_epsilon = 1e-3;
    // 最大允许侧向加速度 (m/s^2)
    double max_lateral_accel = 1.0;
    // ESDF 危险裕度 (m)
    double esdf_danger_margin = 0.5;
};

// 速度规划输入：密集配点的弧长、曲率与最小 ESDF 距离。
struct SpeedProfileInput {
    // 各配点的物理弧长 s_i (m)，必须非递减
    std::vector<double> s;
    // 各配点的曲率 kappa(s_i) (1/m)
    std::vector<double> kappa;
    // 各配点处最小 ESDF 距离 (m)
    std::vector<double> min_esdf_dist;
};

// 速度规划结果
struct SpeedProfileResult {
    bool success = false;
    // 求解器实际迭代次数
    int solver_iterations = 0;
    // 带符号速度 v_i (m/s)
    std::vector<double> v;
    // 带符号加速度 a_i (m/s^2)
    std::vector<double> a;
    // 物理时间戳 t_i (s)
    std::vector<double> t;
    // 原始 QP 变量 b_i = v_i^2
    std::vector<double> b;
    std::string status_msg;
};

// 基于空间域 v^2 凸优化的纵向速度规划器。
class SpeedProfilePlanner {
   public:
    explicit SpeedProfilePlanner(const SpeedProfilePlannerConfig& config);

    // 对单个连续机动段做速度规划。
    SpeedProfileResult plan(const SpeedProfileInput& input,
                            const VehicleParams& vehicle_params,
                            const std::vector<int>& direction_signs,
                            const std::vector<std::size_t>& cusp_indices = {},
                            double initial_velocity = 0.0) const;

   protected:
    // 验证输入合法性
    void validateInputs(const SpeedProfileInput& input,
                        const VehicleParams& vehicle_params,
                        const std::vector<int>& direction_signs,
                        const std::vector<std::size_t>& cusp_indices) const;
    // 计算各点速度上限
    std::vector<double> computeSpeedLimitSquared(
        const SpeedProfileInput& input,
        const std::vector<int>& direction_signs) const;
    // 组装 QP 并求解
    SpeedProfileResult solveQp(const SpeedProfileInput& input,
                               const VehicleParams& vehicle_params,
                               const std::vector<int>& direction_signs,
                               const std::vector<std::size_t>& cusp_indices,
                               double initial_velocity) const;
    // 由 b_i 与方向符号反推 v/a/t
    void recoverVelocityAndTime(const std::vector<double>& b,
                                const std::vector<int>& direction_signs,
                                const std::vector<double>& s,
                                SpeedProfileResult& result) const;

   protected:
    SpeedProfilePlannerConfig config_;
};
}  // namespace apa_post_processor
