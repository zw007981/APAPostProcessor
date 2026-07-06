#pragma once

#include "core/vehicle_geometry.h"
#include "map_interface.h"

#include <Eigen/Core>

#include <array>
#include <vector>

namespace stc_SQP {
// 简单泊车地图实现：用一组带几何边界线段的半空间（墙）描述停车场。
// 每面墙由有限线段 segment 与对应的无限半空间 half-space 组成：
//   - segment 用于 GJK 轮廓距离计算，表示墙体的实际几何范围；
//   - half-space 是 segment 的局部凸化约束，供 ConvexCorridorConstraint 使用。
// queryHalfSpaces() 按车辆轮廓到 segment 的 GJK 距离筛选，而不是到无限半空间边界线的距离。
// 该实现仅用于演示业务层闭环，生产环境可替换为更复杂的地图/感知接口。
class SimpleParkingMap : public MapInterface {
public:
    // 车辆几何参数（与 autogen/common.py 保持一致，单位 m）
    static constexpr double kVehicleLf = vehicle_geometry::kLf;
    static constexpr double kVehicleLr = vehicle_geometry::kLr;
    static constexpr double kVehicleWidth = vehicle_geometry::kWidth;
    // 2D 半空间法向量维度（与 map_interface.h 共享语义）
    static constexpr int kNormalDim = kHalfSpaceNormalDim;
    // 墙端点必须落在 half-space 边界上的容差
    static constexpr double kBoundaryTolerance = 1e-9;

    // 添加一堵墙：half-space 为约束形式 dot(normal, x) <= intercept，
    // segment 为用于 GJK 距离计算的边界线段（世界坐标系）。
    // 要求 start/end 必须位于 half-space 边界上（|n·x - b| <= kBoundaryTolerance），
    // 否则 GJK 距离与约束语义不一致。
    void addWall(const HalfSpace& half_space, const Eigen::Vector2d& start,
        const Eigen::Vector2d& end);
    // 实现 MapInterface：按 GJK 车辆轮廓距离升序返回最近的 top_k 个半空间，
    // 仅包含距离不超过 selection_radius 的墙。
    std::vector<HalfSpace> queryHalfSpaces(
        const Vector& pose, double selection_radius, int top_k) const override;

protected:
    // 根据后轴中心位姿计算车辆四个角点（世界坐标系，顺序：FL, FR, RL, RR）。
    // 使用 std::array 避免固定 4 点场景下的堆分配。
    std::array<Eigen::Vector2d, 4> computeVehicleCorners(const Vector& pose) const;

protected:
    struct Wall {
        HalfSpace half_space;
        Eigen::Vector2d start;
        Eigen::Vector2d end;
    };
    std::vector<Wall> walls_;
};
} // namespace stc_SQP
