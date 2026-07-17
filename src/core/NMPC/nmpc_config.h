#pragma once

#include <optional>

#include <Eigen/Dense>

#include "../../preprocessing/adaptive_resampler.h"
#include "../../preprocessing/bspline_smoother.h"
#include "../../preprocessing/differential_flatness_solver.h"
#include "../../preprocessing/speed_profile_planner.h"
#include "../../preprocessing/static_corridor_builder.h"
#include "../../util/config.h"
#include "path_to_ocp_converter.h"

namespace apa_post_processor {
// NMPC 求解器配置：继承通用 Config，追加 SQP / HPIPM / 迭代走廊等专有参数。
struct NMPCConfig : public Config {
    // 过渡期：保留 PathToOcpConfig 以兼容现有业务代码（后续统一到 Config 基类后移除）
    PathToOcpConfig path_to_ocp_config{};
    // 静态舒适走廊不等式系数矩阵 C * Z <= d（预处理管线产出，运行时填充）
    std::optional<Eigen::MatrixXd> static_corridor_C;
    // 静态舒适走廊线性不等式右端项，长度与 static_corridor_C 行数一致
    std::optional<Eigen::VectorXd> static_corridor_d;
    // 迭代走廊硬边界安全裕度 (m)，吸收线性化截断误差
    double corridor_hard_margin = 0.05;
    // 迭代走廊 HPIPM 软约束二次项权重（Zu，进 Hessian 对角块）
    double corridor_soft_quadratic_weight = 1e8;
    // 迭代走廊 HPIPM 软约束一次项权重（zu，只进线性代价向量，不贡献条件数）
    double corridor_soft_linear_weight = 1e8;
    // Partial Condensing 块大小
    int hpipm_block_size = 10;
    // 低于此总步数阈值时不启用 partial condensing / OpenMP 并行
    int short_n_threshold = 50;
    // 是否启用 SQP 线搜索
    bool use_line_search = false;
    // SQP 全局 Hessian 正则化（Levenberg-Marquardt 风格阻尼），默认关闭
    double sqp_hessian_regularization = 0.0;
    // HPIPM 求解精度
    double hpipm_tol = 1e-4;
    // 终端段最后一步是否跳过静态舒适走廊约束
    bool skip_last_step_corridor = true;
    // 是否注入静态舒适走廊（与迭代走廊正交叠加的舒适偏好软约束）
    bool use_static_corridor_soft_constraint = false;
    // 静态舒适走廊 HPIPM 松弛变量惩罚权重
    double static_corridor_soft_weight = 10.0;

    // === NMPC 使用的预处理管线配置 ===
    // B 样条平滑器配置
    BSplineSmootherConfig bspline;
    // 速度规划器配置
    SpeedProfilePlannerConfig speed;
    // 微分平坦求解器配置
    DifferentialFlatnessSolverConfig diff_flat;
    // 自适应重采样器配置
    AdaptiveResamplerConfig resampler;
    // 静态走廊构建器配置
    StaticCorridorBuilderConfig corridor;
    // 是否构建静态舒适走廊
    bool use_static_corridor = false;
    // 跨阶段统一碰撞检测数值容差 (m)，自动传播到 bspline.collision_margin
    double collision_safety_margin = 0.0;
};
}  // namespace apa_post_processor
