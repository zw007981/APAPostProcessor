#pragma once

#include <Eigen/Core>
#include <vector>

#include "../../spatial/esdf_map.h"
#include "../../vehicle/vehicle_footprint_model.h"
#include "minco_config.h"

namespace apa_post_processor {
// 单时刻位姿的双重碰撞惩罚评估结果
struct MincoEsdfPoseCost {
    // 混合代价 I_obs = Σ_k [W_safe·Φ(C_safe,k) + W_comf·Φ(C_comf,k)]
    double cost{0.0};
    // I_obs 对车辆位姿 (x, y, θ) 的解析梯度
    Eigen::Vector3d gradient{0.0, 0.0, 0.0};
};
// ESDF 双重安全机制惩罚：复用 ESDFMap/VehicleFootprintModel 的外圆
// （Outer Circles）集合，对单时刻位姿 (x,y,θ) 计算 margin_safe/margin_comf
// 两段松弛边界的碰撞惩罚混合代价与解析梯度。第 k 个外圆圆心
// P_k=(x,y)+R(θ)·p_k^local 随 θ 旋转，梯度包含该几何链式法则：
// ∂I_obs/∂θ 经 dP_k/dθ=dR/dθ·p_k^local 反传。惩罚形态为三次光滑外点罚
// Φ(C)=max(0,C)^3（C² 连续，配合 L-BFGS 使用，与运动学约束惩罚同源）。
// 本类只处理单时刻位姿到代价/梯度的映射；时间轴上的梯形离散积分是上层
// 求解器装配的职责，不在本类范围内。
class MincoEsdfPenalty {
   public:
    // 构造时从 footprint_model 提取外圆集合（车身局部坐标与半径）并校验
    // 配置：0<=margin_safe<margin_comf，两个权重均为非负有限值；非法输入
    // 抛 std::invalid_argument
    MincoEsdfPenalty(const ESDFMap& esdf_map,
                   const VehicleFootprintModel& footprint_model,
                   const MincoConfig& config);
    // 主入口：给定车辆位姿 (x,y,θ)，返回 I_obs 与其对 (x,y,θ) 的解析梯度
    MincoEsdfPoseCost evaluate(double x, double y, double theta) const;
    // 当前配置（只读）
    const MincoConfig& config() const { return config_; }

   protected:
    // 累加单个外圆的惩罚值与梯度贡献。圆心越出地图时 ESDF 按实心障碍
    // 处理（M011 L8 修复后的契约）：返回穿透深度（负值随深度线性下降）
    // 与恒指向图内的恢复梯度——惩罚随穿透深度三次增长、梯度非零
    void accumulateCircle(const Eigen::Vector2d& local_center, double cos_theta,
                          double sin_theta, double x, double y,
                          MincoEsdfPoseCost& result) const;

   protected:
    const ESDFMap& esdf_map_;
    MincoConfig config_;
    // 外圆圆心的车身局部坐标（构造时一次性提取，求值时按 θ 解析旋转，
    // 与 NMPC 侧圆形碰撞约束使用同一套几何约定）
    std::vector<Eigen::Vector2d> circle_local_centers_;
    // 外圆半径 r_outer
    double circle_radius_{0.0};
};
}  // namespace apa_post_processor
