#pragma once

#include <Eigen/Core>
#include <cstddef>
#include <vector>

#include "../../spatial/esdf_map.h"
#include "../../vehicle/vehicle_footprint_model.h"
#include "ddp_reference_builder.h"

namespace apa_post_processor {
// 状态二阶导数（7×7）定长类型：栈上分配，热路径严禁堆分配
using DdpStateHessian = Eigen::Matrix<double, DDP_STATE_DIM, DDP_STATE_DIM>;
// ESDF 双 margin 惩罚配置（设计文档 DDP.md 2.4 节）：硬安全边界只做纯数值
// 意义的浮点比较裕度（不与 ESDF 分辨率/物理余量挂钩），舒适缓冲在空间
// 宽裕时引导车辆居中；stride 为时间轴抽样间隔（>1 时查询量下降，间隙
// 风险由 margin 吸收）
struct DdpEsdfConstraintConfig {
    // 硬安全边界 margin_safe (m)
    double margin_safe{0.02};
    // 舒适缓冲边界 margin_comf (m)：必须严格大于 margin_safe
    double margin_comf{0.10};
    // 安全红线惩罚权重 W_safe
    double weight_safe{100.0};
    // 舒适缓冲惩罚权重 W_comf
    double weight_comf{1.0};
    // 时间轴抽样间隔：每 stride 个阶段做一次避障检查，必须 >= 1
    int stride{1};
};
// 单时刻位姿的双 margin 惩罚评估结果：混合代价、对七维状态的解析梯度与
// GN 形 Hessian（非 (x,y,θ) 行/列恒零——惩罚只依赖位姿）
struct DdpEsdfPoseCost {
    // 混合代价 I_obs = Σ_k [W_safe·max(0,C_safe,k)³ + W_comf·max(0,C_comf,k)³]
    double cost{0.0};
    // I_obs 对状态的解析梯度（仅 x/y/θ 行非零）
    DdpState gradient{DdpState::Zero()};
    // GN 形 Hessian Σ_active 6·W·C·∇C·∇Cᵀ（丢弃含场曲率的 ∇²C 项）
    DdpStateHessian hessian{DdpStateHessian::Zero()};
};
// 单个外圆的双 margin 约束值与约束雅可比（对位姿 (x,y,θ) 三维）
struct DdpEsdfCircleConstraint {
    // 安全边界约束值 C_safe = r_outer + margin_safe − d
    double c_safe{0.0};
    // 舒适边界约束值 C_comf = r_outer + margin_comf − d
    double c_comf{0.0};
    // 约束雅可比 ∇C = −∇d 经圆心位置的链式反传（两段 margin 共享同一
    // 雅可比，仅边界值不同）；越界/非有限查询时为零（不引入虚假恢复方向）
    Eigen::Vector3d grad{Eigen::Vector3d::Zero()};
};
// ESDF 双 margin 惩罚：复用 ESDFMap/VehicleFootprintModel 的外圆集合，对单
// 时刻位姿 (x,y,θ) 计算安全/舒适两段 max(0,C)³ 光滑外点罚与解析导数。
// 取值与空间梯度必须使用同一组 ESDF 插值节点（discretize-then-
// differentiate），否则数值梯度与代价评估不一致，DDP 后向传递的 Q_x 会
// 出现系统性偏差、破坏下降方向
class DdpEsdfConstraint {
   public:
    // 构造校验：margin_safe 非负有限、margin_comf 严格更大、两个权重非负
    // 有限、stride>=1、外圆集非空且外圆半径为正；非法输入抛
    // std::invalid_argument（配置错误会静默污染全部下游优化目标，必须在
    // 构造期显式拒绝）
    DdpEsdfConstraint(const ESDFMap& esdf_map,
                      const VehicleFootprintModel& footprint_model,
                      DdpEsdfConstraintConfig config = {});
    // 单时刻位姿求值：对全部外圆累加双 margin 惩罚，并散布到七维状态的
    // (x,y,θ) 行/块，其余分量恒零
    DdpEsdfPoseCost evaluate(double x, double y, double theta) const;
    // 时间轴抽样判定：第 k 阶段是否做避障检查（k % stride == 0）
    bool isSampled(std::size_t k) const {
        return k % static_cast<std::size_t>(config_.stride) == 0;
    }
    // 当前配置（只读）
    const DdpEsdfConstraintConfig& config() const { return config_; }
    // ESDF 地图只读访问（L8.3 定义域守卫需要地图边界同源）
    const ESDFMap& esdfMap() const { return esdf_map_; }

   protected:
    // 单外圆求值：圆心世界坐标 P = (x,y) + R(θ)·p_local，经 ESDF 距离/梯度
    // 与旋转链式法则 dP/dθ = dR/dθ·p_local 得到约束值与雅可比；ESDF 查询
    // 返回非有限值时按保守侵入处理（C 取最大值、梯度为零）；protected
    // 供派生测试类对约束雅可比做有限差分对拍
    DdpEsdfCircleConstraint evaluateCircle(const Eigen::Vector2d& local_center,
                                           double cos_theta, double sin_theta,
                                           double x, double y) const;

   protected:
    // 符号距离场（外部持有，引用不转移所有权）
    const ESDFMap& esdf_map_;
    DdpEsdfConstraintConfig config_;
    // 车身坐标系下的外圆圆心集合（构造时一次性提取，避免重复查询）
    std::vector<Eigen::Vector2d> circle_local_centers_;
    // 外圆半径 r_outer (m)
    double circle_radius_{0.0};
};
}  // namespace apa_post_processor
