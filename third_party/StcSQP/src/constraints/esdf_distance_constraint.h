#pragma once

#include <array>

#include <Eigen/Core>

#include "../core/types.h"
#include "constraint.hpp"

namespace stc_SQP {
// ESDF 距离场约束：将车辆四个角点与最近障碍物的（一阶线性化）距离表示为不等式约束。
// 约束形式：g(x, u, p) <= 0，ng = kNumCorners（4，每角点一个）。
// 与 ConvexCorridorConstraint 不同，本约束不依赖 CasADi 代码生成：ESDF 距离场本身是
// 分段光滑但通常非线性的标量场，只能在采样点附近做一阶泰勒展开
//   d_approx(pos) = d0 + grad^T * (pos - query_pos)
// 因此 p 中除了距离值与梯度外，还必须携带采样时的世界坐标 query_pos，供 SQP 在同一
// solve() 内多次迭代、状态偏离采样点时做一阶修正。业务层（地图/ESDF 查询）负责在每次
// updateOcp 时重新采样并写入 p，本约束对象本身不持有任何地图状态。
class EsdfDistanceConstraint : public Constraint {
public:
    // 角点数量（与 ConvexCorridorConstraint 保持一致的顺序：FL, FR, RL, RR）
    static constexpr int kNumCorners = 4;
    // 每个角点在 p 中占用的槽位数：[distance, grad_x, grad_y, query_x, query_y]
    static constexpr int kCornerStride = 5;
    // p 中 ESDF 参数区间的起始偏移（复用设计文档预留、凸走廊未使用的 p[45:150] 区间）
    static constexpr int kParamStart = 45;
    // p 中 ESDF 参数区间占用的总维度
    static constexpr int kParamDim = kNumCorners * kCornerStride;
    // 状态中必须存在的前三维语义：x, y, theta；与具体运动学模型的 v/delta 或 v/kappa 无关
    static constexpr int kStateXYThetaDim = 3;

    // safety_margin：车辆轮廓到障碍物必须保持的最小距离（米）
    explicit EsdfDistanceConstraint(double safety_margin);
    // 约束维度，固定为 kNumCorners
    int ng() const override;
    // 计算约束值 g(x, u, p) <= 0
    void evaluate(const Vector& x, const Vector& u, const Vector& p, Vector& g) const override;
    // 计算 Jacobian：Cx = dg/dx（仅 x, y, theta 三列非零），Cu = dg/du（恒为零）
    void jacobian(const Vector& x, const Vector& u, const Vector& p, Matrix& Cx,
        Matrix& Cu) const override;
    // 创建独立副本；本约束不持有 CasADi 工作区等可变内部状态，clone 只需拷贝 safety_margin_
    std::shared_ptr<Constraint> clone() const;
    // 返回安全裕度
    double safetyMargin() const { return safety_margin_; }

    // 将某角点的 ESDF 采样（距离、梯度、采样点）写入 p 的对应槽位；
    // 供业务层（如 EsdfProblemUpdater）复用，避免各处重复手写偏移量。
    static void packCornerSample(int corner_idx, double distance,
        const Eigen::Vector2d& gradient, const Eigen::Vector2d& query_pos, Vector& p);
    // 返回车辆四个角点在车身坐标系下的局部坐标（顺序：FL, FR, RL, RR），
    // 与 autogen/generate_corridor.py 中的 CORNERS_LOCAL 保持一致。
    static std::array<Eigen::Vector2d, kNumCorners> cornerLocalPositions();

protected:
    // 检查运行期入参维度是否合法：x 至少包含 x, y, theta 三维
    void validateInputDimensions(const Vector& x, const Vector& u) const;
    // 检查参数 p 维度是否为 STAGE_PARAM_DIM
    static void validateParameters(const Vector& p);

protected:
    // 安全裕度
    double safety_margin_ = 0.0;
};
} // namespace stc_SQP
