#pragma once

#include <memory>
#include <vector>

#include <models/dynamical_system.h>
#include <ocp/multi_stage_ocp.h>
#include <util/trajectory.h>

#include "../../util/path.h"
#include "../../vehicle/vehicle_params.h"

namespace apa_post_processor {
// Path→MultiStageOCP转换配置：控制固定步长重采样、三次多项式初始猜测速度剖面
// 以及代价函数权重的可调参数。
// 速度剖面公式（与doc/note1.md引用论文Eq.6-7一致）：s(t)=b*t^3+c*t^2，
// 满足s(0)=0,v(0)=0,s(tf)=arc_length,v(tf)=0，峰值速度出现在t=tf/2处，
// 数值上等于1.5*arc_length/tf；本转换器反过来由target_peak_speed推算tf。
// 代价函数设计：
//   - 内部机动段（非最后一段）：状态代价仅在v、delta两维施加小权重
//     （v^2项是论文J4路径长度代价sum|v|dt在v=0处光滑可导的二次近似，
//     delta^2项抑制不必要的大转向），x/y/theta完全自由，不做逐段位置跟踪，
//     从而允许换挡点位置在求解中自由滑动、为削减冗余机动创造空间。
//   - 最后一个机动段：额外施加对该段末端原始位姿的跟踪权重（近似终端硬约束）。
//     受限于StcSQP当前CostTerm接口尚未区分stage/terminal（框架自身注释里的
//     已知待办），该跟踪权重会施加到最后一段的每一步而非仅最后一步，属已知的
//     可接受近似——因为该权重的量级配置为“较强但非强制”，且最后一段通常本就
//     贴近终点区域。
//   - 控制效果代价（R）对a、delta_dot恒定生效：由于BicycleModelDelta把delta设为
//     状态、把delta_dot设为控制，其本身已是速度/转角的“一阶变化率”，因此控制
//     效果代价已隐式覆盖论文J2+J3（控制量代价+控制变化率代价）的平滑意图，
//     无需再引入单独的控制变化率代价项（避免额外的状态增广/框架改造）。
struct PathToOcpConfig {
    // 打靶步长(s)，机动段重采样后的离散步长
    double dt = 0.1;
    // 三次多项式速度剖面的目标峰值速度(m/s)，用于估算单个机动段的总耗时
    double target_peak_speed = 1.0;
    // 状态速度的box bound硬上限(m/s)，需大于target_peak_speed以覆盖对齐dt网格后的实际峰值
    double max_speed = 2.0;
    // 换挡边界速度box bound的松弛量(m/s)，允许换挡处速度小幅偏离0以方便收敛
    double boundary_velocity_slack = 0.02;
    // 加速度box bound(m/s^2)
    double accel_limit = 2.0;
    // 前轮转角变化率box bound(rad/s)
    double steer_rate_limit = 1.0;
    // 位置/航向状态的box bound幅值，足够大以视为无约束
    double pose_bound = 1.0e4;
    // 控制效果代价权重：加速度a
    double control_effort_accel_weight = 1e-2;
    // 控制效果代价权重：前轮转角变化率delta_dot
    double control_effort_steer_rate_weight = 1e-2;
    // 内部机动段状态代价权重：速度v（路径长度代价sum|v|dt的光滑二次近似）
    double interior_speed_weight = 1e-2;
    // 内部机动段状态代价权重：前轮转角delta（抑制不必要的大转向）
    double interior_steer_weight = 1e-3;
    // 终端机动段状态代价权重：x、y位置（近似终端位姿硬约束，泊车场景对终点精度要求高，
    // 取较大权重使其接近硬约束效果）
    double terminal_position_weight = 200.0;
    // 终端机动段状态代价权重：航向theta
    double terminal_heading_weight = 200.0;
    // 终端机动段状态代价权重：速度v（应回到0）
    double terminal_speed_weight = 5.0;
    // 终端机动段状态代价权重：前轮转角delta
    double terminal_steer_weight = 1.0;
};

// Path→MultiStageOCP转换器：把Path按Maneuver切分，对每段按固定dt重采样为
// BicycleModelDelta的打靶轨迹（状态x=[x,y,theta,v,delta]，控制u=[a,delta_dot]），
// 构造对应的StageSegment（含box bound与代价，见PathToOcpConfig的代价设计说明）与
// 全局对齐的初始猜测轨迹。暂不支持Direction::PIVOT/UNKNOWN机动段，遇到时抛出
// std::invalid_argument（当前数据集不含PIVOT，后续如需支持需补充设计，见仓库记忆）。
class PathToOcpConverter {
public:
    // 转换结果：MultiStageOCP结构与其对应的、按固定dt对齐的初始猜测轨迹
    struct Result {
        stc_SQP::MultiStageOCP ocp;
        stc_SQP::Trajectory init_guess;
    };

    // 使用车辆参数与转换配置构造转换器
    explicit PathToOcpConverter(const VehicleParams& vehicle_params,
                               const PathToOcpConfig& config = PathToOcpConfig{});
    // 把Path转换为MultiStageOCP与初始猜测轨迹
    Result convert(const Path& path) const;

protected:
    // 三次多项式速度剖面：给定机动段弧长，返回对齐到dt网格的总步数、总耗时与多项式系数
    struct VelocityProfile {
        int step_num = 0;
        double tf = 0.0;
        double b = 0.0;
        double c = 0.0;
    };
    // 单个机动段重采样后的状态/控制序列
    struct ManeuverSamples {
        // 长度为step_num+1，索引0..step_num，状态为[x,y,theta,v,delta]
        std::vector<stc_SQP::Vector> states;
        // 长度为step_num，索引0..step_num-1，控制为[a,delta_dot]
        std::vector<stc_SQP::Vector> controls;
    };
    // 计算三次多项式速度剖面：s(0)=0,v(0)=0,s(tf)=arc_length,v(tf)=0
    VelocityProfile buildVelocityProfile(double arc_length) const;
    // 对单个机动段按固定dt重采样，得到状态/控制序列
    ManeuverSamples sampleManeuver(const Maneuver& maneuver) const;
    // 基于重采样结果构造StageSegment（box bound + 代价）；is_terminal_segment指示
    // 该段是否为Path的最后一个机动段，决定使用终端权重还是内部权重构造状态代价
    stc_SQP::StageSegment buildSegment(
        const Maneuver& maneuver,
        const std::shared_ptr<stc_SQP::DynamicalSystem>& dynamics,
        const ManeuverSamples& samples, bool is_terminal_segment) const;
    // 构建点序列的累计弧长数组，cumulative[0]恒为0
    static std::vector<double> buildCumulativeArcLength(
        const std::vector<PathPoint>& points);
    // 按弧长s在点序列中做线性插值，返回插值后的(x,y,theta,kappa)
    static PathPoint interpolateAtArcLength(
        const std::vector<PathPoint>& points,
        const std::vector<double>& cumulative, double s);

protected:
    // 车辆参数，提供wheelbase/max_steer_angle等约束边界
    VehicleParams vehicle_params_;
    // 转换配置
    PathToOcpConfig config_;
};
}  // namespace apa_post_processor
