#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "time_profile.h"
#include "trajectory_point.h"

namespace apa_post_processor {
// 前向声明，避免头文件重依赖
class ESDFMap;
class Path;
class VehicleFootprintModel;

// 轨迹合法性验证配置
struct TrajectoryValidationConfig {
    // 最大碰撞深度 (m)
    double max_collision_depth = 0.02;
    // 终点位置误差上限 (m)
    double max_terminal_position_error = 0.05;
    // 终点航向误差上限 (°)
    double max_terminal_heading_error_deg = 3.0;
    // 运动学梯形配点残差阈值（对相邻两点 i,i+1，检验一阶 ODE 关系在梯形
    // 积分意义下的残差，直接配点转录法的标准做法）。四项阈值按三数据集
    // （data1/data3/data7）NMPC 优化结果的实测残差分布标定并留余量，
    // 量化依据见 docs/interfaces.md 变更记录
    // 参与残差评估的最大相邻点时长间隔 (s)：梯形截断误差为 O(Δt³)，
    // Δt 过大的点对（实测为换挡停驻补丁步，Δt 达 1.3~3.7 s）残差由截断
    // 误差主导、不携带可行性判别信号，按此上限跳过
    double max_kinematic_dt = 0.5;
    // 位置残差上限 (m)：|Δp - Δt/2·(v_i·u(θ_i) + v_{i+1}·u(θ_{i+1}))|
    // 标定：三数据集 Δt≤0.5s 点对实测最大 0.0060 m（data3），取 3.3 倍余量
    double max_kinematic_position_residual = 0.02;
    // 航向残差上限 (°)：|Δθ - Δt/2·(θ̇_i + θ̇_{i+1})|，θ̇=v·tanδ/L
    // 标定：实测最大 1.53°（data3，换挡后 δ̈ 较大的点对），取约 2 倍余量
    double max_kinematic_heading_residual_deg = 3.0;
    // 速度残差上限 (m/s)：|Δv - Δt/2·(a_i + a_{i+1})|
    // 标定：实测 ~1e-15（a 为状态量、jerk 分段恒定时梯形精确），取宽松上限
    double max_kinematic_velocity_residual = 0.05;
    // 前轮转角残差上限 (rad)：|Δδ - Δt/2·(δ̇_i + δ̇_{i+1})|
    // 标定：实测 ~1e-15（同理），取宽松上限。MINCO 轨迹族复核（换挡曲率
    // 补救调参）：hinge C_δ 压出的"合法但急转"轨迹（硬件上限 |δ|≤1.04×
    // δ_max、|δ̇|≤0.38 rad/s 已单独验证）在急转向区的梯形配点残差实测
    // 0.023~0.030 rad——物理量合法、残差是配点离散伪影，阈值放宽至 0.05
    // （约 2 倍余量），对真实 δ 跳变（0.3~3.0 rad）仍保有一个量级以上的
    // 检出余量
    double max_kinematic_steer_residual = 0.05;
    // 低速跳过阈值 (m/s)：相邻两点 |v| 均低于该值时跳过前轮转角残差评估。
    // 近零速度下 θ̇=v·tanδ/L≈0 与 δ 取值解耦，δ/δ̇ 不再承载运动可行性
    // 信号；θ-s 参数化下换挡尖点（ṡ→0）附近 δ 可在 atan 值域内跳变，
    // 该残差是奇异特征处的截断伪影而非可行性证据（位置/航向/速度残差
    // 在低速下仍照常评估，"v≡0 但 θ 变化"类矛盾仍由航向残差检出）
    double kinematic_low_speed_epsilon = 0.05;
};

// 轨迹合法性验证结果
struct TrajectoryValidationResult {
    bool collision_safe = false;
    bool terminal_position_ok = false;
    bool terminal_heading_ok = false;
    // 运动学可行性（梯形配点残差全部不超标）。时间戳缺失/无相邻点对时
    // 跳过该门（不证伪，记为 true 并在 kinematic_detail 说明原因）
    bool kinematic_feasible = false;
    // 四项全部通过
    bool all_passed = false;
    // 最大碰撞深度 (m)
    double max_intrusion_depth = 0.0;
    // 终点位置误差 (m)
    double terminal_position_error = 0.0;
    // 终点航向误差 (°)
    double terminal_heading_error_deg = 0.0;
    // 最大位置残差 (m)（仅统计两端点量齐备的点对；无齐备点对时为 0）
    double max_kinematic_position_residual = 0.0;
    // 最大航向残差 (°)
    double max_kinematic_heading_residual_deg = 0.0;
    // 最大速度残差 (m/s)
    double max_kinematic_velocity_residual = 0.0;
    // 最大前轮转角残差 (rad)
    double max_kinematic_steer_residual = 0.0;
    // 各门失败原因（空字符串表示通过）；运动学门被跳过时注明跳过原因
    std::string collision_detail;
    std::string terminal_position_detail;
    std::string terminal_heading_detail;
    std::string kinematic_detail;
};

// 轨迹：带时间戳与完整运动学状态/控制量的 TrajectoryPoint 序列。
// 与 Path 的区别：Path 侧重几何路径与机动段分割（Maneuver），Trajectory
// 侧重时序状态序列。
class Trajectory {
   public:
    Trajectory() = default;
    // 从 TrajectoryPoint 向量构造
    explicit Trajectory(std::vector<TrajectoryPoint> points);
    // 由几何路径与车辆运动学参数构造全量参考轨迹：几何/微分平坦量由构造
    // 侧统一补全——运动方向签名曲率 κ=σ·κ_geom（σ 由 Maneuver 方向决定：
    // BACKWARD 取 -1，FORWARD/UNKNOWN/PIVOT 取 +1；与 tanδ/L 及轨迹对比
    // 视图的曲率约定一致）、δ=atan(L·κ)；纵向时序量（v/a/t）由
    // ComputeTimeProfile 的梯形加减速时间参数化给出（"最快走完"前提，
    // 首末点/换挡点零速，各非末机动段的末个发射点登记为换挡点）；δ̇ 由
    // δ 对 t 的段内差分给出（分母非正置 0）。路径点未设置 κ（Path 未
    // finalize）时按 0 处理；后续机动段的首点为前段末点重复，按
    // Path::forEach 语义跳过。wheelbase 非正/非有限、时间参数化配置或
    // 车辆纵向极限非法时抛 std::invalid_argument
    Trajectory(const Path& path, const VehicleParams& vehicle_params,
               const TimeProfileConfig& time_config = {});
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
    // 物理方向段数（换挡次数）：先按明确符号（|v|>=v_epsilon）切段并累计
    // 各段位移，再丢弃位移不足 min_arc 的抖动段并合并同号邻段。停驻点
    // （|v|<v_epsilon）不改变方向状态、不产生段边界，其位移归入当前段；
    // 低速数值抖动（离散求解器停驻区 ±cm/s 级毛刺）因位移不足被过滤，
    // 与 Path::addPoint 方向推断（ds>=DELTA_DIST 才定方向）语义一致
    int countDirectionRuns(double v_epsilon = 1e-3,
                           double min_arc = 0.05) const;
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
    // 验证轨迹合法性：碰撞安全 + 终点收敛 + 运动学可行性（梯形配点残差）
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
