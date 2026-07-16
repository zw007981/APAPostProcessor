#pragma once

#include <LBFGS.h>

#include <Eigen/Core>
#include <utility>
#include <vector>

#include "../spatial/esdf_map.h"
#include "../util/maneuver.h"
#include "../vehicle/vehicle_footprint_model.h"
#include "../vehicle/vehicle_params.h"

namespace apa_post_processor {
// B样条平滑器配置。
struct BSplineSmootherConfig {
    // 密集配点间距 (m)
    double dense_step_dist = 0.05;
    // 控制点生成间距 (m)
    double control_point_spacing = 0.15;
    // 同伦走廊偏移惩罚权重
    double weight_data = 1.0;
    // 一阶差分惩罚权重
    double weight_smooth_d1 = 5.0;
    // 二阶差分惩罚权重
    double weight_smooth_d2 = 50.0;
    // 三阶差分惩罚权重
    double weight_smooth_d3 = 200.0;
    // 密采碰撞惩罚权重
    double weight_collision = 500.0;
    // 显式曲线曲率惩罚权重（当前未启用）
    double weight_kappa = 0.0;
    // 允许的最大曲线曲率 (m⁻¹)
    double max_kappa = 0.18;
    // 切向均匀化正则项权重
    double weight_reg = 0.1;
    // 碰撞检测数值容差 (m)
    double collision_margin = 0.0;
    // 首尾控制点航向锚定延伸步长 (m)
    double anchor_extension_length = 0.3;
    // 短机动段退化阈值 (m)
    double min_segment_arc_length_for_degradation = 0.1;
    // 侵入深度事后校验容忍度 (m)
    double collision_validation_tolerance = 1e-4;
    // L-BFGS 最大迭代次数
    int lbfgs_max_iterations = 80;
    // L-BFGS 梯度范数收敛阈值（绝对）
    double lbfgs_epsilon = 1e-5;
    // L-BFGS 梯度范数收敛阈值（相对）
    double lbfgs_epsilon_rel = 1e-5;
    // L-BFGS 历史记忆长度
    int lbfgs_m = 10;
    // 线搜索最大试探次数
    int lbfgs_max_linesearch = 20;
    // 线搜索算法：1=Armijo, 2=Wolfe, 3=Strong Wolfe
    int lbfgs_linesearch_algo = 1;
    // Armijo 条件参数
    double lbfgs_ftol = 1e-4;
    // Wolfe 曲率条件参数
    double lbfgs_wolfe = 0.9;
    // 弧长重参数化表参数步长
    double arc_length_table_step = 0.001;
};

// 四次 B 样条平滑器。
class BSplineSmoother {
   public:
    // 车身子圆 ESDF 查询缓存
    struct CircleEsdfData {
        // 车身局部偏移 (m)
        Eigen::Vector2d local_offset{Eigen::Vector2d::Zero()};
        // 世界坐标圆心
        Eigen::Vector2d center{Eigen::Vector2d::Zero()};
        // ESDF 符号距离 (m)
        double dist = 0.0;
        // ESDF 梯度
        Eigen::Vector2d grad{Eigen::Vector2d::Zero()};
    };
    // 密集配点数据
    struct DensePointData {
        // B 样条参数 u
        double u = 0.0;
        // 物理弧长 s (m)
        double s = 0.0;
        // 曲线位置
        Eigen::Vector2d position{Eigen::Vector2d::Zero()};
        // 曲线切向角 (rad)
        double theta = 0.0;
        // 全部子圆 ESDF 数据
        std::vector<CircleEsdfData> circles;
    };
    // 平滑结果
    struct Result {
        bool success = false;
        // 是否触发短机动段退化
        bool degenerate = false;
        // B 样条控制点
        std::vector<Eigen::Vector2d> control_points;
        // 弧长重参数化表
        std::vector<std::pair<double, double>> arc_length_table;
        // 密集配点缓存
        std::vector<DensePointData> dense_points;
        // 最终最大侵入深度 (m)
        double max_intrusion_depth = 0.0;
        // L-BFGS 实际迭代次数
        int lbfgs_iterations = 0;
        // L-BFGS 是否成功收敛
        bool optimizer_converged = false;
    };
    // 锚定控制点
    struct AnchorPoints {
        Eigen::Vector2d p0{Eigen::Vector2d::Zero()};
        Eigen::Vector2d p1{Eigen::Vector2d::Zero()};
        Eigen::Vector2d pn_1{Eigen::Vector2d::Zero()};
        Eigen::Vector2d pn{Eigen::Vector2d::Zero()};
    };
    // 基函数及其前三阶导数
    struct BasisPack {
        double u = 0.0;
        // knot span
        int span = 0;
        // 非零基函数对应的控制点索引
        std::vector<int> indices;
        // 0~3 阶基函数值
        std::vector<double> values;
        std::vector<double> d1;
        std::vector<double> d2;
        std::vector<double> d3;
    };
    // 密集评估点：保存局部基函数供梯度计算复用
    struct EvalPoint {
        double u = 0.0;
        // 物理弧长 s (m)
        double s = 0.0;
        // 非零基函数对应的控制点索引
        std::vector<int> indices;
        // 0 阶基函数值
        std::vector<double> values;
        // 1 阶基函数值
        std::vector<double> d1;
        // 2 阶基函数值
        std::vector<double> d2;
    };
    // 使用配置、车辆参数、footprint 模型与 ESDF 地图构造平滑器。
    BSplineSmoother(const BSplineSmootherConfig& config,
                    const VehicleParams& vehicle_params,
                    const VehicleFootprintModel& footprint_model,
                    const ESDFMap& esdf_map);
    // 对单个 Maneuver 执行 B 样条平滑。
    Result smooth(const Maneuver& maneuver) const;
    // 构建 clamped uniform knot vector。
    std::vector<double> buildKnotVector(int control_point_count) const;
    // 计算单个 u 处的非零基函数及其前三阶导数。
    void computeBasisAtU(double u, const std::vector<double>& knot_vector,
                         int control_point_count, BasisPack& bp) const;

   protected:
    // 验证配置合法性
    void validateInputs() const;
    // 确定控制点总数
    int determineControlPointCount(double arc_length) const;
    // 从 Maneuver 提取参考控制点与法向量
    void buildReferenceControlPoints(
        const Maneuver& maneuver, int control_point_count,
        std::vector<Eigen::Vector2d>& ref_points,
        std::vector<Eigen::Vector2d>& ref_normals) const;
    // 根据起止位姿与机动段构造锚定控制点
    AnchorPoints buildAnchorPoints(const Maneuver& maneuver, double arc_length,
                                   int control_point_count,
                                   Direction direction) const;
    // 在精细参数网格上预计算基函数包
    void precomputeBasisPacks(const std::vector<double>& knot_vector,
                              int control_point_count,
                              std::vector<BasisPack>& basis_packs) const;
    // 在精细网格上求位置与速度
    void evaluateFineGrid(const std::vector<Eigen::Vector2d>& control_points,
                          const std::vector<BasisPack>& basis_packs,
                          std::vector<Eigen::Vector2d>& positions,
                          std::vector<Eigen::Vector2d>& velocities) const;
    // 由精细网格位置构建弧长表
    std::vector<std::pair<double, double>> buildArcLengthTable(
        const std::vector<Eigen::Vector2d>& positions) const;
    // 按物理弧长采样密集评估点
    std::vector<EvalPoint> buildDenseEvalPoints(
        const std::vector<double>& knot_vector, int control_point_count,
        const std::vector<std::pair<double, double>>& arc_length_table) const;
    // 生成退化路径的线性插值结果
    Result buildDegenerateResult(const TrajectoryPoint& start,
                                 const TrajectoryPoint& end,
                                 Direction direction) const;
    // 对最终控制点执行侵入深度校验并填充 dense_points 缓存
    bool validateCollisionFree(
        const std::vector<Eigen::Vector2d>& control_points,
        const std::vector<EvalPoint>& dense_eval_points,
        std::vector<DensePointData>& dense_points, double& max_intrusion_depth,
        Direction direction) const;

   protected:
    BSplineSmootherConfig config_;
    VehicleParams vehicle_params_;
    const ESDFMap& esdf_map_;
    // 车身外圆局部圆心
    std::vector<Eigen::Vector2d> outer_circle_local_centers_;
    // 车身外圆半径
    double outer_circle_radius_ = 0.0;
    // 外圆数量
    std::size_t outer_circle_num_ = 0U;
};
}  // namespace apa_post_processor
