#pragma once

#include <vector>

#include "../vehicle/vehicle_params.h"

namespace apa_post_processor {

// 速度规划配置：控制 s 域 v^2 QP 的目标权重、速度上限与数值保护参数。
// 字段默认值来源 docs/default_params.md 与 NMPC.md 3.2 节。
struct SpeedProfilePlannerConfig {
    // 泊车前进极限速度 (m/s)，来源：PreprocessParams.max_v_forward
    double max_v_forward = 1.389;
    // 泊车倒车极限速度 (m/s)，来源：PreprocessParams.max_v_reverse
    double max_v_reverse = 1.0;
    // 空间加速度变化率上限 (|delta_a /
    // delta_s|)，来源：PreprocessParams.max_jerk_proxy
    double max_jerk_proxy = 3.0;
    // 贴合参考速度权重，来源：PreprocessParams.weight_v_ref
    double weight_v_ref = 10.0;
    // 绝对加速度惩罚权重，来源：PreprocessParams.weight_a_sq
    double weight_a_sq = 2.0;
    // 舒适度/Jerk惩罚权重(delta_a 平滑)，来源：PreprocessParams.weight_jerk_sq
    double weight_jerk_sq = 80.0;
    // 时间戳反积分死区保护 (m/s)，NMPC.md 3.2 节建议值
    double time_reintegration_epsilon = 1e-3;
    // 最大允许侧向加速度 (m/s^2)，用于由曲率收紧速度上限
    // 本 Milestone 新增（待定），实现阶段用真实场景调参
    double max_lateral_accel = 1.0;
    // ESDF 危险裕度 (m)，距离小于该值的区域会被线性收紧速度上限
    // 本 Milestone 新增（待定），量级与 Milestone 005 的 collision_margin 相当
    double esdf_danger_margin = 0.5;
};

// 速度规划输入：每个密集配点对应的空间、曲率与最小 ESDF 距离。
// 调用方（预处理管线）负责从 BSplineSmoother 的 DensePointData 提取这些标量。
struct SpeedProfileInput {
    // 各配点的物理弧长 s_i (m)，必须非递减
    std::vector<double> s;
    // 各配点的曲率 kappa(s_i) (1/m)
    std::vector<double> kappa;
    // 各配点处车身全部子圆的最小 ESDF 距离 (m)
    std::vector<double> min_esdf_dist;
};

// 速度规划结果
struct SpeedProfileResult {
    // 是否成功求解
    bool success = false;
    // 求解器实际迭代次数
    int solver_iterations = 0;
    // 带符号速度 v_i (m/s)，符号由 direction_signs 决定
    std::vector<double> v;
    // 带符号加速度 a_i (m/s^2)
    std::vector<double> a;
    // 物理时间戳 t_i (s)，t_0 = 0
    std::vector<double> t;
    // 原始 QP 变量：b_i = v_i^2
    std::vector<double> b;
    // 状态信息
    std::string status_msg;
};

// 基于空间域 v^2 凸优化的纵向速度规划器。
// 对固定弧长网格上的 [b_i, a_i] 构建三对角带状 QP，通过 OsqpSolver 求解，
// 再反推带符号速度、加速度与时间戳。
class SpeedProfilePlanner {
   public:
    explicit SpeedProfilePlanner(const SpeedProfilePlannerConfig& config);

    // 对单个连续机动段（或全局拼接的多段）做速度规划。
    // @param input        密集配点输入（s/kappa/min_esdf_dist）
    // @param vehicle_params 车辆参数，读取 max_accel/max_decel 作为 box bound
    // @param direction_signs 每个配点的方向符号（+1 前进，-1 倒车）
    // @param cusp_indices  需要强制 b_k = 0
    // 的内部换挡尖点索引（如多段拼接的分界点）
    // @param initial_velocity 整条路径起始速度大小 (m/s)，默认 0.0；仅影响 b_0
    // @return 规划结果
    SpeedProfileResult plan(const SpeedProfileInput& input,
                            const VehicleParams& vehicle_params,
                            const std::vector<int>& direction_signs,
                            const std::vector<std::size_t>& cusp_indices = {},
                            double initial_velocity = 0.0) const;

   protected:
    // 验证输入维度、参数合法性与尖点索引范围
    void validateInputs(const SpeedProfileInput& input,
                        const VehicleParams& vehicle_params,
                        const std::vector<int>& direction_signs,
                        const std::vector<std::size_t>& cusp_indices) const;
    // 计算各点速度上限 V_limit^2[i]
    std::vector<double> computeSpeedLimitSquared(
        const SpeedProfileInput& input,
        const std::vector<int>& direction_signs) const;
    // 组装 QP 并调用 OsqpSolver 求解
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
