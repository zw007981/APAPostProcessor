#pragma once

#include <vector>

#include "map_interface.h"

namespace stc_SQP {
// 圆形障碍物 ESDF 地图：用一组圆（中心 + 半径）近似障碍物，提供闭式距离与梯度。
// 仅用于演示 EsdfMapInterface 闭环与测试/基准，生产环境应替换为真实的栅格/优化 ESDF。
class CircleObstacleEsdfMap : public EsdfMapInterface {
public:
    // 圆心距离小于该阈值时视为退化情形（点与圆心重合），梯度退化为零向量。
    static constexpr double kMinCenterDistance = 1e-9;

    // 添加一个圆形障碍物：center 为世界坐标系圆心，radius 必须为正数。
    void addObstacle(const Eigen::Vector2d& center, double radius);
    // 实现 EsdfMapInterface：返回到最近圆形障碍物边界的符号距离与梯度。
    // 若未添加任何障碍物，返回一个很大的距离与零梯度（视为无约束）。
    EsdfSample queryDistance(const Eigen::Vector2d& point) const override;

protected:
    struct Circle {
        Eigen::Vector2d center;
        double radius = 0.0;
    };
    std::vector<Circle> obstacles_;
};
} // namespace stc_SQP
