#pragma once

namespace apa_post_processor {
// 单时刻 θ-s 轨迹采样点：位置与各阶导数（可由 MincoTrajectory::evaluate 逐阶
// 查询组装）
struct ThetaSSample {
    // 朝向角 (rad)
    double theta{0.0};
    // 角速度 dθ/dt (rad/s)
    double theta_dot{0.0};
    // 角加速度 d²θ/dt² (rad/s²)
    double theta_ddot{0.0};
    // 累计弧长 (m)
    double s{0.0};
    // 线速度 ds/dt (m/s)，负值表示倒车
    double s_dot{0.0};
    // 纵向加速度 d²s/dt² (m/s²)
    double s_ddot{0.0};
};
// 阿克曼自行车模型的状态/控制量
struct AckermannState {
    // 线速度 v=ṡ (m/s)
    double v{0.0};
    // 纵向加速度 a=s̈ (m/s²)
    double a{0.0};
    // 前轮转角 δ (rad)
    double delta{0.0};
    // 前轮转角速度 δ̇ (rad/s)
    double delta_dot{0.0};
};
// 某标量量关于采样输入导数 (θ̇, θ̈, ṡ, s̈) 的梯度；约束惩罚不依赖 θ,s 本身，
// 故梯度只有这四个分量
struct SampleGradient {
    double d_theta_dot{0.0};
    double d_theta_ddot{0.0};
    double d_s_dot{0.0};
    double d_s_ddot{0.0};
};
// δ 与 δ̇ 的解析梯度（与正则化值公式保持一致）
struct SteerGradients {
    SampleGradient delta;
    SampleGradient delta_dot;
};
// 单个物理约束的软惩罚信息（未加权，权重由外层优化目标施加）
struct ConstraintPenalty {
    // 原始约束值 C，C<=0 表示可行
    double constraint{0.0};
    // 光滑外点罚 max(0, C)^3（C² 连续可导，配合 L-BFGS 使用）
    double penalty{0.0};
    // penalty 对输入导数的解析梯度
    SampleGradient gradient;
};
// 四个泊车物理约束的软惩罚
struct PhysicalConstraintPenalties {
    // 纵向极值约束 C_v = ṡ² - v_max²
    ConstraintPenalty velocity;
    // 纵向加速度约束 C_a = s̈² - a_max²
    ConstraintPenalty acceleration;
    // 前轮最大转角约束（防奇异平方形态）C_δ = L²θ̇² - ṡ²tan²(δ_max)
    ConstraintPenalty steer_angle;
    // 方向盘打角速度约束（防奇异交叉乘积形态）
    // C_δ̇ = L²(θ̈ṡ-θ̇s̈)² - δ̇_max²(ṡ²+L²θ̇²)²
    ConstraintPenalty steer_rate;
};
// 运动学映射与物理约束配置
struct BicycleKinematicsConfig {
    // 轴距 L_base (m)
    double wheelbase{2.8};
    // 纵向速度上限 v_max (m/s)
    double max_velocity{2.0};
    // 纵向加速度上限 a_max (m/s²)
    double max_acceleration{1.5};
    // 前轮最大转角 δ_max (rad)
    double max_steer_angle{0.65};
    // 方向盘打角速度上限 δ̇_max (rad/s)
    double max_steer_rate{0.4};
    // δ/δ̇ 分母正则化常数 ε_g（m²/s² 量级）；取 0 时严格退化为未正则化公式
    double epsilon_g{1e-8};
    // C_δ 惩罚的光滑 hinge 半宽 ε_h（约束值量纲）：C_δ 从三次光滑外点罚
    // max(0,C)³ 改造为分段 C¹ 形态——C≤0 恒为 0，0<C≤ε_h 取 C²/(2ε_h)，
    // C>ε_h 取 C−ε_h/2（梯度 1·∇C）。三次形态在小违反区梯度 ∝C² 消失，
    // 换挡尖点邻域的转向角违反因此对优化器不可见；hinge 形态在 C>ε_h 时
    // 梯度不消失。取值依据：关心的违反量级 ~1e-3（换挡邻域实测），取
    // 1e-4 使该量级落在线性区
    double steer_hinge_epsilon{1e-4};
};
// θ-s 空间到阿克曼状态的运动学映射与物理约束惩罚提取器。输入为某一时刻
// MincoTrajectory 给出的 θ,θ̇,θ̈,s,ṡ,s̈，输出 v,a,δ,δ̇ 及四个物理约束的
// 软惩罚值与解析梯度。核心难点是换挡点附近 ṡ→0 的除零/病态梯度：δ 与 δ̇
// 的取值公式统一施加 ε_g 分母正则化，且解析梯度把 ε_g 当作分母的一部分一
// 并求导，保证值与梯度严格一致；四个约束则全部采用无除法的二次/交叉乘积
// 形态，本身在 (ṡ,θ̇)=(0,0) 处无奇异。
class BicycleKinematicsExtractor {
   public:
    // 构造并校验配置：wheelbase
    // 与各上限必须为正有限值，max_steer_angle∈(0,π/2)，
    // epsilon_g>=0；非法输入抛 std::invalid_argument
    explicit BicycleKinematicsExtractor(BicycleKinematicsConfig config = {});
    // 提取阿克曼状态量。δ=atan(L·θ̇·ṡ/(ṡ²+ε_g))：ε_g→0 时对 ṡ≠0 严格退化
    // 为 atan(L·θ̇/ṡ)，且 (ṡ,θ̇)=(0,0) 处给出有限值 0 而非 0/0 NaN；
    // δ̇=L(θ̈ṡ-θ̇s̈)/(ṡ²+L²θ̇²+ε_g)
    AckermannState extract(const ThetaSSample& sample) const;
    // δ 与 δ̇ 对输入导数的解析梯度，与 extract 的正则化值公式逐字一致
    SteerGradients steerGradients(const ThetaSSample& sample) const;
    // 四个防奇异二次形态约束的软惩罚与解析梯度（未加权）
    PhysicalConstraintPenalties evaluatePenalties(
        const ThetaSSample& sample) const;
    // 当前配置（只读）
    const BicycleKinematicsConfig& config() const { return config_; }

   protected:
    // 由约束值与其梯度组装惩罚项：penalty=max(0,C)^3，∇penalty=3·max(0,C)^2·∇C
    static ConstraintPenalty MakePenalty(double constraint,
                                         const SampleGradient& grad_c);
    // C_δ 专用的光滑 hinge 惩罚项：C≤0 恒为 0；0<C≤ε_h 取 C²/(2ε_h)、
    // 梯度 (C/ε_h)·∇C；C>ε_h 取 C−ε_h/2、梯度 1·∇C（分段点处值与梯度
    // 均连续，C¹ 光滑）
    static ConstraintPenalty MakeHingePenalty(double constraint,
                                              const SampleGradient& grad_c,
                                              double hinge_epsilon);

   protected:
    BicycleKinematicsConfig config_;
    // tan²(δ_max)，构造时预计算
    double tan_steer_max_sq_{0.0};
};
}  // namespace apa_post_processor
