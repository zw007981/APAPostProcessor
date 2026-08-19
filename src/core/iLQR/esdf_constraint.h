#pragma once

#include <Eigen/Core>
#include <cstddef>
#include <vector>

#include "../../spatial/esdf_map.h"
#include "../../vehicle/vehicle_footprint_model.h"
#include "ilqr_config.h"
#include "ilqr_reference_builder.h"

namespace apa_post_processor {
using iLQRStateHessian = Eigen::Matrix<double, ILQR_STATE_DIM, ILQR_STATE_DIM>;
struct iLQREsdfPoseCost {
    // 目标值
    double cost{0.0};
    iLQRState gradient{iLQRState::Zero()};
    iLQRStateHessian hessian{iLQRStateHessian::Zero()};
};
struct iLQREsdfCircleConstraint {
    // 安全边界约束值
    double c_safe{0.0};
    // 舒适边界约束值
    double c_comf{0.0};
    Eigen::Vector3d grad{Eigen::Vector3d::Zero()};
};
// ESDF 惩罚：max(0,C)³ 光滑外点罚 + 解析导数，discretize-then-differentiate
// 保证梯度与代价一致
class iLQREsdfConstraint {
   public:
    iLQREsdfConstraint(const ESDFMap& esdf_map,
                      const VehicleFootprintModel& footprint_model,
                      // 配置
                      const iLQRConfig& config = {});
    // 单时刻位姿求值：对全部外圆累加双 margin 惩罚，并散布到七维状态的
    // (x,y,θ) 行/块，其余分量恒零
    iLQREsdfPoseCost evaluate(double x, double y, double theta) const;
    // A1 带跳过掩码的求值：skip_mask[i]==true 跳过第 i 个外圆的整次 ESDF
    // 查询（该圆贡献恒为 0）。out_dists 可选：全量查询的圆填充距离，跳过
    // 的圆置 -1（供调用方维护守卫缓存）。跳过圆贡献恒 0 ⟹ 与全量求值
    // 逐位一致（跳过仅发生在守卫证明距离仍足够远的圆上）
    iLQREsdfPoseCost evaluateWithSkip(
        double x, double y, double theta, const std::vector<bool>* skip_mask,
        std::vector<double>* out_dists = nullptr) const;
    // A1 守卫支持：非活跃阈值 r_outer+margin_comf (m)
    double inactiveThreshold() const {
        return circle_radius_ + config_.esdf_margin_comf;
    }
    // A1 守卫支持：外圆数量
    std::size_t numCircles() const { return circle_local_centers_.size(); }
    // A1 守卫支持：最大外圆杠杆臂（圆心到车辆中心的距离上界，m）
    double maxCircleLeverArm() const;
    // 时间轴抽样判定：第 k 阶段是否做避障检查（k % stride == 0）
    bool isSampled(std::size_t k) const {
        return k % static_cast<std::size_t>(config_.esdf_stride) == 0;
    }
    // 当前配置（只读）
    const iLQRConfig& config() const { return config_; }
    // ESDF 地图只读访问（L8.3 定义域守卫需要地图边界同源）
    const ESDFMap& esdfMap() const { return esdf_map_; }

   protected:
    // 单外圆求值：圆心世界坐标 P = (x,y) + R(θ)·p_local，经 ESDF 距离/梯度
    iLQREsdfCircleConstraint evaluateCircle(const Eigen::Vector2d& local_center,
                                           double cos_theta, double sin_theta,
                                           double x, double y) const;

   protected:
    // 符号距离场（外部持有，引用不转移所有权）
    const ESDFMap& esdf_map_;
    // 配置
    iLQRConfig config_;
    // 车身坐标系下的外圆圆心集合（构造时一次性提取，避免重复查询）
    std::vector<Eigen::Vector2d> circle_local_centers_;
    // 外圆半径 r_outer (m)
    double circle_radius_{0.0};
};
}  // namespace apa_post_processor
