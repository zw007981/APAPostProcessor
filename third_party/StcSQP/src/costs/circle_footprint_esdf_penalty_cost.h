#pragma once

#include <vector>

#include <Eigen/Core>

#include "../core/types.h"
#include "cost_term.hpp"
#include "map_interface.h"

namespace stc_SQP {
// 圆形分解车身的ESDF距离场“软”碰撞代价：与CircleFootprintEsdfConstraint（硬约束版本）
// 表达同一个物理量（车身圆到最近障碍物的距离），但以平方铰链惩罚（hinge penalty）代价的
// 形式施加，而非硬不等式约束。这样即使车辆当前位姿已经违反安全裕度（真实数据里初始猜测
// 可能已经贴着障碍物走），QP仍然总是可行（box/动力学约束本身构成非空可行域），只是会为
// 违反的部分支付高额代价，从而把“绝对不能碰撞”弱化为“碰撞代价显著大于其它代价、但求解
// 永不因碰撞约束不可行而直接失败”。
// 与CircleFootprintEsdfConstraint的另一个区别：本代价直接持有EsdfMapInterface的引用，
// 在每次SQP迭代对当前(x,y,theta)重新查询距离场（而不是像约束那样依赖求解前一次性注入、
// 在整个solve()期间可能过时的线性化采样点），因此不需要配套的ProblemUpdater预注入步骤，
// 精度上也更贴近真实非线性距离场。
// 架构说明：本类依赖examples/parking下的EsdfMapInterface，这是对“SQP引擎不感知业务语义”
// 原则的一处务实妥协——因为CostTerm接口（不同于Constraint）不接收每步参数p，无法复用
// 已有的p注入机制，直接持有地图引用是达成“碰撞代价始终可行”这一目标的最小代价方案。
class CircleFootprintEsdfPenaltyCost : public CostTerm {
public:
    // circle_local_positions：车身坐标系下各圆心的局部坐标，数量必须非空
    // circle_radius：所有圆共用的半径(m)，必须为正的有限数
    // safety_margin：车辆轮廓到障碍物应保持的最小距离(m)，必须为非负的有限数
    // map：ESDF距离场地图，调用方必须保证其生命周期覆盖本代价对象的整个使用周期
    // penalty_weight：违反安全裕度时的惩罚权重，应显著大于其它代价（如控制效果/终端代价），
    //   使碰撞惩罚在数量级上起主导作用；必须为非负有限数
    CircleFootprintEsdfPenaltyCost(std::vector<Eigen::Vector2d> circle_local_positions,
        double circle_radius, double safety_margin, const EsdfMapInterface& map,
        double penalty_weight);
    // 计算标量代价：对每个违反安全裕度的圆做平方铰链惩罚并求和
    void evaluate(const Vector& x, const Vector& u, double& cost) const override;
    // 计算梯度：q = dcost/dx（仅x, y, theta三维非零），r恒为零
    void gradient(const Vector& x, const Vector& u, Vector& q, Vector& r) const override;
    // 计算Hessian（Gauss-Newton近似，忽略ESDF场自身二阶曲率以保证半正定）：
    // Q = sum(penalty_weight * grad_i ⊗ grad_i)，R、S恒为零
    void hessian(const Vector& x, const Vector& u, Matrix& Q, Matrix& R, Matrix& S) const override;

protected:
    // 对当前状态x逐圆计算违反量（required_clearance - distance，负值截断为0）及其对
    // (x, y, theta)的梯度，供evaluate/gradient/hessian共用，避免重复查询地图
    void computeViolations(const Vector& x, std::vector<double>& violations,
        std::vector<Eigen::Vector3d>& violation_grads) const;
    // 检查运行期入参维度是否合法：x至少包含x, y, theta三维
    void validateInputDimensions(const Vector& x, const Vector& u) const;

protected:
    // 车身坐标系下各圆心的局部坐标
    std::vector<Eigen::Vector2d> circle_local_positions_;
    // 圆半径
    double circle_radius_ = 0.0;
    // 安全裕度
    double safety_margin_ = 0.0;
    // ESDF距离场地图（非拥有引用，生命周期由调用方保证）
    const EsdfMapInterface& map_;
    // 惩罚权重
    double penalty_weight_ = 0.0;
};
} // namespace stc_SQP
