#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "trajectory_point.h"

namespace apa_post_processor {
// 前向声明，避免头文件重依赖
class ESDFMap;
class VehicleFootprintModel;

// 轨迹合法性验证配置
struct TrajectoryValidationConfig {
    // 最大碰撞深度 (m)
    double max_collision_depth = 0.02;
    // 终点位置误差上限 (m)
    double max_terminal_position_error = 0.05;
    // 终点航向误差上限 (°)
    double max_terminal_heading_error_deg = 3.0;
};

// 轨迹合法性验证结果
struct TrajectoryValidationResult {
    bool collision_safe = false;
    bool terminal_position_ok = false;
    bool terminal_heading_ok = false;
    // 三项全部通过
    bool all_passed = false;
    // 最大碰撞深度 (m)
    double max_intrusion_depth = 0.0;
    // 终点位置误差 (m)
    double terminal_position_error = 0.0;
    // 终点航向误差 (°)
    double terminal_heading_error_deg = 0.0;
    // 各门失败原因（空字符串表示通过）
    std::string collision_detail;
    std::string terminal_position_detail;
    std::string terminal_heading_detail;
};

// 轨迹：带时间戳与完整运动学状态/控制量的 TrajectoryPoint 序列。
// 与 Path 的区别：Path 侧重几何路径与机动段分割（Maneuver），Trajectory
// 侧重时序状态序列。
class Trajectory {
   public:
    Trajectory() = default;
    // 从 TrajectoryPoint 向量构造
    explicit Trajectory(std::vector<TrajectoryPoint> points);
    // 轨迹是否为空
    bool empty() const { return points_.empty(); }
    // 轨迹点数量
    std::size_t size() const { return points_.size(); }
    // 预分配内存
    void reserve(std::size_t n) { points_.reserve(n); }
    // 清空轨迹
    void clear();
    // 轨迹总弧长 (m)
    double length() const;
    // 轨迹总时长 (s)：末点时间戳减首点时间戳，若时间戳未设置则返回 0
    double duration() const;
    // 首个轨迹点
    TrajectoryPoint& front();
    const TrajectoryPoint& front() const;
    // 末个轨迹点
    TrajectoryPoint& back();
    const TrajectoryPoint& back() const;
    // 下标访问
    TrajectoryPoint& operator[](std::size_t i);
    const TrajectoryPoint& operator[](std::size_t i) const;
    // 追加轨迹点
    void push_back(const TrajectoryPoint& pt);
    void push_back(TrajectoryPoint&& pt);
    // 就地构造轨迹点
    template <typename... Args>
    void emplace_back(Args&&... args) {
        length_cache_.reset();
        points_.emplace_back(std::forward<Args>(args)...);
    }
    // 迭代器
    auto begin() { return points_.begin(); }
    auto end() { return points_.end(); }
    auto begin() const { return points_.begin(); }
    auto end() const { return points_.end(); }
    auto cbegin() const { return points_.cbegin(); }
    auto cend() const { return points_.cend(); }
    // 只读访问内部向量（用于与旧接口兼容的过渡期）
    const std::vector<TrajectoryPoint>& points() const { return points_; }
    // 验证轨迹合法性：碰撞安全 + 终点收敛（运动学可行性由 SQP
    // 保证，不重复检查）
    TrajectoryValidationResult validate(
        const TrajectoryPoint& goal, const ESDFMap& esdf_map,
        const VehicleFootprintModel& footprint_model,
        const TrajectoryValidationConfig& config = {}) const;
    // 转化为 JSON 字符串
    std::string toString() const;

   protected:
    std::vector<TrajectoryPoint> points_;
    // 弧长缓存：修改轨迹后失效
    mutable std::optional<double> length_cache_;
};
// 将验证结果格式化为单行可读字符串
std::string FormatValidationResult(const TrajectoryValidationResult& result);
}  // namespace apa_post_processor
