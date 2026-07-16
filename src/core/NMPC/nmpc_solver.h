#pragma once

#include <ocp/multi_stage_ocp.h>
#include <sqp/sqp_algorithm.h>
#include <util/trajectory.h>

#include <map>
#include <optional>
#include <tuple>

#include "../../spatial/esdf_map.h"
#include "../../vehicle/vehicle_footprint_model.h"
#include "../../vehicle/vehicle_params.h"
#include "path_to_ocp_converter.h"

namespace apa_post_processor {
// NMPC求解器配置：打靶/代价 + ESDF安全裕度 + partial condensing/SQP收敛参数。
struct NmpcSolverConfig {
    // Path→OCP转换配置（重采样步长、代价权重等）
    PathToOcpConfig path_to_ocp_config{};
    // ESDF碰撞数值容差(m)，仅用于浮点误差兜底。
    double esdf_safety_margin = 0.0;
    // ESDF碰撞惩罚软代价权重。默认禁用，仅用迭代走廊。
    double esdf_penalty_weight = 0.0;
    // 静态舒适走廊不等式系数矩阵 C * Z <= d（预处理管线产出）。
    std::optional<Eigen::MatrixXd> static_corridor_C;
    // 静态舒适走廊线性不等式右端项，长度与 static_corridor_C 行数一致。
    std::optional<Eigen::VectorXd> static_corridor_d;
    // 迭代走廊硬边界安全裕度 (m)，吸收线性化截断误差。
    double corridor_hard_margin = 0.05;
    // 迭代走廊 HPIPM 软约束二次项权重（Zu，进 Hessian 对角块）。
    double corridor_soft_quadratic_weight = 1e8;
    // 迭代走廊 HPIPM 软约束一次项权重（zu，只进线性代价向量，不贡献条件数）。
    double corridor_soft_linear_weight = 1e8;
    // Partial Condensing块大小
    int hpipm_block_size = 10;
    // 低于此总步数阈值时不启用 partial condensing/OpenMP 并行
    int short_n_threshold = 50;
    // SQP最大迭代次数
    int max_iter = 150;
    bool use_line_search = false;
    // SQP 全局 Hessian 正则化（Levenberg-Marquardt 风格阻尼），默认关闭。
    double sqp_hessian_regularization = 0.0;
    // HPIPM求解精度
    double hpipm_tol = 1e-4;
    // 质量门：终点位置误差上限(m)。<0 表示不启用。
    double terminal_position_error_threshold = 0.05;
    // 质量门：终点航向误差上限(°)。<0 表示不启用。
    double terminal_heading_error_threshold_deg = 3.0;
    // 航向跟踪软代价死区宽度 (rad)
    double max_theta_deviation_from_ref = 0.06;
    // 航向跟踪软代价权重，0.0 表示关闭。
    double theta_tracking_weight = 0.0;
    // 位置跟踪软代价死区宽度 (m)
    double max_position_deviation_from_ref = 0.01;
    // 位置跟踪软代价权重，0.0 表示关闭。
    double position_tracking_weight = 0.0;
    // 终端段最后一步是否跳过静态舒适走廊约束。
    bool skip_last_step_corridor = true;
    // 是否注入静态舒适走廊（与迭代走廊正交叠加的舒适偏好软约束）。
    bool use_static_corridor_soft_constraint = false;
    // 静态舒适走廊 HPIPM 松弛变量惩罚权重。
    double static_corridor_soft_weight = 10.0;
};

// NMPC求解器：基于StcSQP的SQP引擎，对初始路径做平滑与换挡段优化。
class NmpcSolver {
   public:
    // 求解结果
    struct Result {
        // SQP是否收敛；不收敛时trajectory仍写入最新迭代轨迹。
        bool converged = false;
        // 优化后的轨迹（状态x=[x,y,theta,v,delta]，控制u=[a,delta_dot]）
        stc_SQP::Trajectory trajectory;
        // 每个机动段的步数N，与Path的Maneuver一一对应
        std::vector<int> segment_steps;
        // 每个机动段的方向符号：+1前进，-1后退
        std::vector<double> segment_v_signs;
        // 本次optimize()调用的总耗时(ms)
        double solve_time_ms = 0.0;
    };

    // 使用车辆参数、footprint模型与求解器配置构造
    NmpcSolver(const VehicleParams& vehicle_params,
               const VehicleFootprintModel& footprint_model,
               NmpcSolverConfig config = NmpcSolverConfig{});
    // 对初始路径在给定ESDF地图下做NMPC优化
    Result optimize(const Path& initial_path, const ESDFMap& esdf_map) const;
    // 对已装配好的OCP与初始猜测做NMPC优化（预处理管线入口）
    Result optimize(const stc_SQP::MultiStageOCP& ocp,
                    const stc_SQP::Trajectory& init_guess,
                    const ESDFMap& esdf_map) const;
    // 把优化结果还原为apa_post_processor::Path（含Maneuver方向与PathPoint序列）。
    static Path ToPath(const Result& result);

   protected:
    // 共享的求解执行体：约束注入、HPIPM/SQP构造与求解。
    Result solveOcp(const stc_SQP::MultiStageOCP& ocp,
                    const stc_SQP::Trajectory& init_guess,
                    const ESDFMap& esdf_map) const;

   protected:
    VehicleParams vehicle_params_;
    // 车身坐标系下的外圆局部圆心坐标
    std::vector<Eigen::Vector2d> circle_local_positions_;
    double circle_radius_;
    NmpcSolverConfig config_;
    // Path→OCP转换器（仅用于Path入口）
    PathToOcpConverter converter_;
    // QPData 对象池缓存：以 (N, nx, nu, ng_max) 为 key，跨实例共享。
    static std::map<std::tuple<int, int, int, int>,
                    std::unique_ptr<stc_SQP::QPData>>
        qp_data_cache_;
};
}  // namespace apa_post_processor
