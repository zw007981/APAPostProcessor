#pragma once

#include <models/dynamical_system.h>
#include <ocp/multi_stage_ocp.h>
#include <util/trajectory.h>

#include <memory>
#include <vector>

#include "../../util/path.h"
#include "../../vehicle/vehicle_params.h"

namespace apa_post_processor {
// Path→MultiStageOCP 转换配置，控制固定步长重采样与代价函数权重的可调参数。
struct PathToOcpConfig {
    // 打靶步长 (s)
    double dt = 0.1;
    // 三次多项式速度剖面的目标峰值速度 (m/s)
    double target_peak_speed = 1.0;
    // 速度 box bound 硬上限 (m/s)
    double max_speed = 2.0;
    // 换挡边界速度 box bound 松弛量 (m/s)
    double boundary_velocity_slack = 0.02;
    // 加速度 box bound (m/s^2)
    double accel_limit = 2.0;
    // 前轮转角变化率 box bound (rad/s)
    double steer_rate_limit = 1.0;
    // 位置/航向状态的box bound幅值，足够大以视为无约束
    double pose_bound = 1.0e4;
    // 纵向加加速度 box bound (m/s^3)，仅 PreprocessingToOcpConverter 使用
    double max_jerk = 10.0;
    // 转向角加速度 box bound (rad/s^2)，仅 PreprocessingToOcpConverter 使用
    double max_steer_angular_accel = 5.0;
    // 控制效果代价权重：加速度a
    double control_effort_accel_weight = 1e-2;
    // 控制效果代价权重：前轮转角变化率delta_dot
    double control_effort_steer_rate_weight = 1e-2;
    // J_smooth 顺滑代价权重：jerk 的 R 代价，仅 PreprocessingToOcpConverter 使用
    double smoothing_jerk_weight = 1e-1;
    // J_smooth 顺滑代价权重：转向角加速度的 R 代价，仅 PreprocessingToOcpConverter 使用
    double smoothing_steer_accel_weight = 1e-1;
    // 内部机动段状态代价权重：速度 v
    double interior_speed_weight = 1e-2;
    // 内部机动段状态代价权重：前轮转角 delta
    double interior_steer_weight = 1e-3;
    // 全程目标牵引代价权重：x、y 位置
    double global_target_position_weight = 1e-3;
    // 全程目标牵引代价权重：航向 theta
    double global_target_heading_weight = 1e-3;
    // 终端机动段状态代价权重：x、y位置。
    double terminal_position_weight = 1e5;
    // 终端机动段状态代价权重：航向theta
    double terminal_heading_weight = 1e5;
    // 终端机动段状态代价权重：速度 v
    double terminal_speed_weight = 5.0;
    // 终端机动段状态代价权重：前轮转角delta
    double terminal_steer_weight = 1.0;
};

// 单段 OCP 的离散化描述：步数、步长、方向符号与是否终端段。
struct SegmentProfile {
    // 该段离散步数 N
    int N = 0;
    // 均匀步长（dt_array为空时生效）
    double dt = 0.0;
    // 每步独立步长，非空时优先于 dt
    std::vector<double> dt_array;
    // 速度方向符号：+1.0前进，-1.0后退
    double v_sign = 1.0;
    // 是否为最后一个机动段
    bool is_terminal = false;
};

// Path→MultiStageOCP 转换器：按 Maneuver 切分 Path 并构造 StageSegment 与初始猜测。
class PathToOcpConverter {
   public:
    // 转换结果：MultiStageOCP结构与其对应的初始猜测轨迹
    struct Result {
        stc_SQP::MultiStageOCP ocp;
        stc_SQP::Trajectory init_guess;
    };

    // 使用车辆参数与转换配置构造转换器
    explicit PathToOcpConverter(
        const VehicleParams& vehicle_params,
        const PathToOcpConfig& config = PathToOcpConfig{});
    // 把Path转换为MultiStageOCP与初始猜测轨迹（保留原有行为）
    Result convert(const Path& path) const;
    // 生成默认的三次多项式初始猜测轨迹。
    stc_SQP::Trajectory generateInitialGuess(
        const Path& path, const std::vector<SegmentProfile>& profiles) const;
    // 仅装配 OCP 结构，与初始猜测生成解耦。
    stc_SQP::MultiStageOCP buildOcp(
        const Path& path, const std::vector<SegmentProfile>& profiles,
        const stc_SQP::Trajectory& ref_trajectory) const;
    // 由Path与固定dt配置计算各段离散化描述（默认均匀步长场景）
    std::vector<SegmentProfile> computeSegmentProfiles(const Path& path) const;

   protected:
    // 三次多项式速度剖面参数。
    struct VelocityProfile {
        int step_num = 0;
        double tf = 0.0;
        double b = 0.0;
        double c = 0.0;
    };
    // 单个机动段重采样后的状态/控制序列
    struct ManeuverSamples {
        // 状态序列：长度 step_num+1，状态 [x,y,theta,v,delta]
        std::vector<stc_SQP::Vector> states;
        // 控制序列：长度 step_num，控制 [a,delta_dot]
        std::vector<stc_SQP::Vector> controls;
    };
    // 计算三次多项式速度剖面：s(0)=0,v(0)=0,s(tf)=arc_length,v(tf)=0
    VelocityProfile buildVelocityProfile(double arc_length) const;
    // 对单个机动段重采样，得到状态/控制序列。
    ManeuverSamples sampleManeuver(
        const Maneuver& maneuver, const SegmentProfile& profile,
        const VelocityProfile& velocity_profile) const;
    // 基于段描述与参考轨迹构造 StageSegment。
    stc_SQP::StageSegment buildSegment(
        const Maneuver& maneuver,
        const std::shared_ptr<stc_SQP::DynamicalSystem>& dynamics,
        const SegmentProfile& profile, const stc_SQP::Vector& terminal_x_ref,
        const stc_SQP::Vector& global_target_x_ref,
        const std::vector<double>& theta_refs,
        const std::vector<double>& x_refs,
        const std::vector<double>& y_refs) const;
    // 构建点序列的累计弧长数组，cumulative[0]恒为0
    static std::vector<double> buildCumulativeArcLength(
        const std::vector<TrajectoryPoint>& points);
    // 按弧长s在点序列中做线性插值，返回插值后的(x,y,theta,kappa)
    static TrajectoryPoint interpolateAtArcLength(
        const std::vector<TrajectoryPoint>& points,
        const std::vector<double>& cumulative, double s);

   protected:
    // 车辆参数，提供wheelbase/max_steer_angle等约束边界
    VehicleParams vehicle_params_;
    // 转换配置
    PathToOcpConfig config_;
};
}  // namespace apa_post_processor
