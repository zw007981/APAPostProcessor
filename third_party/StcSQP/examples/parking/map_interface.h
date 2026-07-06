#pragma once

#include <cmath>
#include <vector>

#include <Eigen/Core>

#include "core/types.h"

namespace stc_SQP {
// 地图半空间查询接口：业务层通过该接口从地图/障碍物数据中获取当前位姿附近的凸走廊约束。
// 每个半空间表示为 { x | dot(normal, x) <= intercept }
struct HalfSpace {
    // 2D 法向量（指向可行域外部或内部由地图层约定）
    Vector normal;
    // 截距
    double intercept = 0.0;
};

// 半空间法向量维度（2D）。
inline constexpr int kHalfSpaceNormalDim = 2;
// 半空间法向量最小平方长度，拒绝零/近零法向量。
inline constexpr double kHalfSpaceMinNormalNormSq = 1e-12;

// 校验 HalfSpace 是否具备合法的几何语义：normal 为 2 维、数值有限且非零，截距有限。
// 返回 true 表示可用；调用方仍可自行抛出带上下文的异常信息。
inline bool isValidHalfSpace(const HalfSpace& hs) noexcept {
    return hs.normal.size() == kHalfSpaceNormalDim && hs.normal.allFinite()
        && std::isfinite(hs.intercept)
        && hs.normal.squaredNorm() > kHalfSpaceMinNormalNormSq;
}

// 地图接口抽象：ProblemUpdater 通过该接口获取半空间，避免直接依赖具体地图实现。
// 实现类必须基于车辆轮廓距离（GJK）进行筛选，selection_radius 语义见 UpdaterConfig。
class MapInterface {
public:
    virtual ~MapInterface() = default;
    // 查询当前位姿 pose 附近、车辆轮廓 GJK 距离不超过 selection_radius 的至多 top_k 个半空间约束。
    // pose 至少包含 [x, y, theta] 三维；返回结果应按 GJK 距离升序排列（最近优先）。
    virtual std::vector<HalfSpace> queryHalfSpaces(
        const Vector& pose, double selection_radius, int top_k) const = 0;
};

// ESDF 距离场采样：某一点到最近障碍物的（符号）距离与该处的梯度。
struct EsdfSample {
    // 到最近障碍物的距离（米）；车外为正，穿入障碍物为负（符号距离场语义，具体实现自行约定）。
    double distance = 0.0;
    // 距离场在查询点处的梯度（世界坐标系，2D，指向距离增大的方向）。
    Eigen::Vector2d gradient = Eigen::Vector2d::Zero();
};

// ESDF 地图接口抽象：与 MapInterface（SFC/半空间）并列的另一种障碍物表达方式。
// 业务层（如 EsdfProblemUpdater）通过该接口在车辆角点处采样距离场，再由
// EsdfDistanceConstraint 做一阶线性化。两种接口可以在同一个 MultiStageOCP 中并存
// （分别对应不同的 Constraint 子类），SQP 引擎本身不区分它们。
class EsdfMapInterface {
public:
    virtual ~EsdfMapInterface() = default;
    // 查询世界坐标系下某一点到最近障碍物的距离与梯度。
    virtual EsdfSample queryDistance(const Eigen::Vector2d& point) const = 0;
};
} // namespace stc_SQP
