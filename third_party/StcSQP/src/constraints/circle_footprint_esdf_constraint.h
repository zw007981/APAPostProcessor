#pragma once

#include <vector>

#include <Eigen/Core>

#include "../core/types.h"
#include "constraint.hpp"

namespace stc_SQP {
// 圆形分解车身的ESDF距离场约束：把车身近似为一组固定半径的圆，逐圆对最近障碍物距离做一阶线性化。
// 约束形式：g(x, u, p) <= 0，ng() = 圆的数量（构造时给定，非编译期固定值，与EsdfDistanceConstraint的
// 4角点矩形版本不同，本约束不依赖任何编译期车辆几何常量，圆的局部坐标与半径均由构造函数显式传入，
// 从而可以直接复用业务层任意车辆几何模型（如按位姿插值圆心的圆形分解模型）算出的圆心/半径。
// 与EsdfDistanceConstraint一致，本约束不依赖CasADi代码生成：距离场是分段光滑但通常非线性的标量场，
// 只能在采样点附近做一阶泰勒展开，因此p中除距离值与梯度外还需携带采样时的世界坐标query_pos，
// 供SQP在同一次solve()内多次迭代、状态偏离采样点时做一阶修正；本约束对象本身不持有任何地图状态。
class CircleFootprintEsdfConstraint : public Constraint {
public:
    // p中预留的圆参数最大槽位数：与EsdfDistanceConstraint共用p[45:150]区间，
    // 该区间共105个double，按每圆5个槽位计算最多可容纳21个圆，这里预留20个圆并留出5个槽位余量。
    // 注意：本约束与EsdfDistanceConstraint（矩形四角点版本）共享p中的同一起始偏移，
    // 两者语义互斥，不应在同一个MultiStageOCP中对同一步同时注入这两种约束的参数，否则会互相覆盖。
    static constexpr int kMaxCircles = 20;
    // 每个圆在p中占用的槽位数：[distance, grad_x, grad_y, query_x, query_y]
    static constexpr int kCircleStride = 5;
    // p中圆参数区间的起始偏移，复用EsdfDistanceConstraint预留的p[45:150]区间
    static constexpr int kParamStart = 45;
    // p中圆参数区间占用的总维度（按kMaxCircles预留，实际圆数量可小于该值）
    static constexpr int kParamDim = kMaxCircles * kCircleStride;
    // 状态中必须存在的前三维语义：x, y, theta；与具体运动学模型的v/delta或v/kappa无关
    static constexpr int kStateXYThetaDim = 3;

    // circle_local_positions：车身坐标系下（后轴中心为原点、航向为0）各圆心的局部坐标，
    //   数量必须在[1, kMaxCircles]之间，且全部有限。
    // circle_radius：所有圆共用的半径（米），必须为正的有限数。
    // safety_margin：车辆轮廓到障碍物必须保持的最小距离（米），必须为非负的有限数。
    CircleFootprintEsdfConstraint(std::vector<Eigen::Vector2d> circle_local_positions,
        double circle_radius, double safety_margin);
    // 约束维度，等于构造时传入的圆数量
    int ng() const override;
    // 计算约束值 g(x, u, p) <= 0
    void evaluate(const Vector& x, const Vector& u, const Vector& p, Vector& g) const override;
    // 计算Jacobian：Cx = dg/dx（仅x, y, theta三列非零），Cu = dg/du（恒为零）
    void jacobian(const Vector& x, const Vector& u, const Vector& p, Matrix& Cx,
        Matrix& Cu) const override;
    // 创建独立副本；本约束不持有CasADi工作区等可变内部状态，clone只需拷贝构造参数
    std::shared_ptr<Constraint> clone() const override;
    // 返回圆半径
    double circleRadius() const { return circle_radius_; }
    // 返回安全裕度
    double safetyMargin() const { return safety_margin_; }
    // 返回车身坐标系下各圆心的局部坐标
    const std::vector<Eigen::Vector2d>& circleLocalPositions() const
    {
        return circle_local_positions_;
    }

    // 将某个圆的ESDF采样（距离、梯度、采样点）写入p的对应槽位；
    // 供业务层（如CircleFootprintEsdfProblemUpdater）复用，避免各处重复手写偏移量。
    static void packCircleSample(int circle_idx, double distance,
        const Eigen::Vector2d& gradient, const Eigen::Vector2d& query_pos, Vector& p);

protected:
    // 检查运行期入参维度是否合法：x至少包含x, y, theta三维
    void validateInputDimensions(const Vector& x, const Vector& u) const;
    // 检查参数p维度是否为STAGE_PARAM_DIM
    static void validateParameters(const Vector& p);

protected:
    // 车身坐标系下各圆心的局部坐标
    std::vector<Eigen::Vector2d> circle_local_positions_;
    // 圆半径
    double circle_radius_ = 0.0;
    // 安全裕度
    double safety_margin_ = 0.0;
};
} // namespace stc_SQP
