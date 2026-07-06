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
// B样条平滑器配置：控制空间离散度、L-BFGS目标函数权重与收敛行为。
// 字段默认值来源 docs/default_params.md 与 NMPC.md 3.1 节新增参数，
// 标为"待定"的字段为本 Milestone 新增，实现阶段用真实数据调参。
struct BSplineSmootherConfig {
    // 原始A*轨迹重采样与多圆检测基准间距(m)，来源：PreprocessParams.dense_step_dist
    double dense_step_dist = 0.05;
    // 同伦走廊偏移惩罚权重，来源：PreprocessParams.weight_data
    double weight_data = 1.0;
    // 一阶差分惩罚(长度)权重，来源：PreprocessParams.weight_smooth_d1
    double weight_smooth_d1 = 5.0;
    // 二阶差分惩罚(曲率)权重，来源：PreprocessParams.weight_smooth_d2
    double weight_smooth_d2 = 50.0;
    // 三阶差分惩罚(曲率导数)权重，来源：PreprocessParams.weight_smooth_d3
    double weight_smooth_d3 = 200.0;
    // 密采碰撞惩罚(三次项系数)权重，来源：PreprocessParams.weight_collision
    double weight_collision = 500.0;
    // 切向均匀化正则项权重：防止F_data只约束法向时控制多边形自相交，
    // 本 Milestone 新增（待定），取值显著小于weight_data
    double weight_reg = 0.1;
    // 碰撞安全裕度(m)：子圆允许侵入障碍物边界的缓冲，本 Milestone 新增（待定）
    double collision_margin = 0.05;
    // 首尾控制点航向锚定的启发式延伸步长L(m)，本 Milestone 新增（待定）
    double anchor_extension_length = 0.3;
    // 短机动段退化阈值(m)：弧长低于该值时直接线性插值，与Milestone
    // 008同名概念数值一致
    double min_segment_arc_length_for_degradation = 0.1;
    // 侵入深度事后校验允许的最大侵入深度(m)
    double collision_validation_tolerance = 0.02;
    // L-BFGS最大迭代次数，本 Milestone 新增（待定）
    int lbfgs_max_iterations = 100;
    // L-BFGS梯度范数收敛阈值，本 Milestone 新增（待定）
    double lbfgs_epsilon = 1e-5;
    // 弧长重参数化表的参数步长Delta u
    double arc_length_table_step = 0.001;
};

// 四次B样条分段平滑器：对单个Maneuver拟合C^3连续几何曲线，
// 输出控制点、弧长表与密集配点缓存，供预处理管线后续阶段复用。
class BSplineSmoother {
   public:
    // 单个车身子圆在某一配点处的ESDF查询缓存
    struct CircleEsdfData {
        // 车身坐标系下的局部偏移(m)
        Eigen::Vector2d local_offset{Eigen::Vector2d::Zero()};
        // 世界坐标系下的圆心
        Eigen::Vector2d center{Eigen::Vector2d::Zero()};
        // 该圆心处的ESDF符号距离(m)
        double dist = 0.0;
        // 该圆心处的ESDF梯度
        Eigen::Vector2d grad{Eigen::Vector2d::Zero()};
    };

    // 单个密集配点：空间位置、航向与挂载的子圆ESDF缓存
    struct DensePointData {
        // B样条参数u
        double u = 0.0;
        // 物理弧长s(m)
        double s = 0.0;
        // 曲线位置
        Eigen::Vector2d position{Eigen::Vector2d::Zero()};
        // 曲线切向角(rad)
        double theta = 0.0;
        // 该点处挂载的全部子圆ESDF数据
        std::vector<CircleEsdfData> circles;
    };

    // 平滑结果
    struct Result {
        // 是否成功完成平滑（退化路径也视为成功）
        bool success = false;
        // 是否触发短机动段退化
        bool degenerate = false;
        // B样条控制点 P_0 .. P_{N_c}
        std::vector<Eigen::Vector2d> control_points;
        // 弧长重参数化表 (u_i, s_i)，严格单调递增
        std::vector<std::pair<double, double>> arc_length_table;
        // 密集配点缓存，供Milestone 008复用
        std::vector<DensePointData> dense_points;
        // 最终最大侵入深度(m)，若超过collision_validation_tolerance则success=false
        double max_intrusion_depth = 0.0;
        // L-BFGS实际迭代次数（若触发重试，为首次+重试之和）
        int lbfgs_iterations = 0;
        // L-BFGS是否成功收敛（异常/失败时false；重试成功后true）
        bool optimizer_converged = false;
    };

    // 锚定控制点：P_0, P_1, P_{N_c-1}, P_{N_c}
    struct AnchorPoints {
        Eigen::Vector2d p0{Eigen::Vector2d::Zero()};
        Eigen::Vector2d p1{Eigen::Vector2d::Zero()};
        Eigen::Vector2d pn_1{Eigen::Vector2d::Zero()};
        Eigen::Vector2d pn{Eigen::Vector2d::Zero()};
    };

    // 单个u处的基函数及其前三阶导数（仅保存局部非零项）
    struct BasisPack {
        // 参数u
        double u = 0.0;
        // 该u对应的knot span
        int span = 0;
        // 非零基函数对应的控制点索引，长度为p+1=5
        std::vector<int> indices;
        // 0阶、1阶、2阶、3阶基函数值
        std::vector<double> values;
        std::vector<double> d1;
        std::vector<double> d2;
        std::vector<double> d3;
    };

    // 单个密集评估点：直接保存该u处的插值后局部基函数，供Frozen-theta梯度计算复用
    struct EvalPoint {
        // 参数u
        double u = 0.0;
        // 物理弧长s(m)
        double s = 0.0;
        // 非零基函数对应的控制点索引
        std::vector<int> indices;
        // 0阶基函数值（用于位置/圆心）
        std::vector<double> values;
        // 1阶基函数值（用于切向/航向）
        std::vector<double> d1;
    };

    // 使用配置、车辆参数、车辆 footprint 模型与ESDF地图构造平滑器
    BSplineSmoother(const BSplineSmootherConfig& config,
                    const VehicleParams& vehicle_params,
                    const VehicleFootprintModel& footprint_model,
                    const ESDFMap& esdf_map);
    // 对单个Maneuver执行B样条平滑，返回控制点、弧长表与密集配点缓存
    Result smooth(const Maneuver& maneuver) const;

   protected:
    // 验证配置与构造参数的合法性
    void validateInputs() const;
    // 根据机动段长度与配置确定控制点总数（含4个锚定点），保证非退化情况下>=6
    int determineControlPointCount(double arc_length) const;
    // 从Maneuver提取参考控制点与法向量（仅内部控制点P_2..P_{N_c-2}有效）
    void buildReferenceControlPoints(
        const Maneuver& maneuver, int control_point_count,
        std::vector<Eigen::Vector2d>& ref_points,
        std::vector<Eigen::Vector2d>& ref_normals) const;
    // 根据起止位姿构造4个锚定控制点，自动限制延伸步长避免控制多边形折叠
    AnchorPoints buildAnchorPoints(const PathPoint& start, const PathPoint& end,
                                   double arc_length,
                                   int control_point_count) const;
    // 构建clamped uniform knot vector，总控制点数为control_point_count
    std::vector<double> buildKnotVector(int control_point_count) const;
    // 计算单个u处的非零基函数及其前三阶导数
    void computeBasisAtU(double u, const std::vector<double>& knot_vector,
                         int control_point_count, BasisPack& bp) const;
    // 在精细参数网格上预计算基函数包
    void precomputeBasisPacks(const std::vector<double>& knot_vector,
                              int control_point_count,
                              std::vector<BasisPack>& basis_packs) const;
    // 给定控制点与预计算基函数，在精细网格上求位置与速度
    void evaluateFineGrid(const std::vector<Eigen::Vector2d>& control_points,
                          const std::vector<BasisPack>& basis_packs,
                          std::vector<Eigen::Vector2d>& positions,
                          std::vector<Eigen::Vector2d>& velocities) const;
    // 由精细网格位置构建弧长表
    std::vector<std::pair<double, double>> buildArcLengthTable(
        const std::vector<Eigen::Vector2d>& positions) const;
    // 由弧长重参数化表按物理弧长采样密集评估点
    std::vector<EvalPoint> buildDenseEvalPoints(
        const std::vector<double>& knot_vector, int control_point_count,
        const std::vector<std::pair<double, double>>& arc_length_table) const;
    // 生成退化路径的线性插值结果
    Result buildDegenerateResult(const PathPoint& start,
                                 const PathPoint& end) const;
    // 对最终控制点执行侵入深度校验，同时填充dense_points缓存
    bool validateCollisionFree(
        const std::vector<Eigen::Vector2d>& control_points,
        const std::vector<EvalPoint>& dense_eval_points,
        std::vector<DensePointData>& dense_points,
        double& max_intrusion_depth) const;

   protected:
    // 配置
    BSplineSmootherConfig config_;
    // 车辆参数
    VehicleParams vehicle_params_;
    // ESDF地图
    const ESDFMap& esdf_map_;
    // 车身外圆局部圆心（theta=0时）
    std::vector<Eigen::Vector2d> outer_circle_local_centers_;
    // 车身外圆半径
    double outer_circle_radius_ = 0.0;
    // 外圆数量
    std::size_t outer_circle_num_ = 0U;
};
}  // namespace apa_post_processor
