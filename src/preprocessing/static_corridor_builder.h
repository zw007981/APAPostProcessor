#pragma once

#include <Eigen/Core>
#include <string>
#include <vector>

#include "../spatial/esdf_map.h"
#include "../util/trajectory_point.h"
#include "../vehicle/vehicle_footprint_model.h"

namespace apa_post_processor {
// 静态舒适走廊构建器配置：只产出舒适软约束，不提供安全担保。
struct StaticCorridorBuilderConfig {
    // 舒适余量 (m)
    double soft_margin = 0.18;
};

// 单个打靶点-圆的静态舒适走廊约束：c^T * Z <= d。
struct StaticCorridorConstraint {
    // 不等式左端系数向量，长度 5
    Eigen::VectorXd c;
    // 不等式右端项
    double d = 0.0;
    // 所属打靶点索引
    int point_idx = 0;
    // 所属车身外圆索引
    int circle_idx = 0;
};

// 静态舒适走廊构建结果。
struct StaticCorridorBuilderResult {
    bool success = false;
    std::string status_msg;
    // 逐约束展开形式
    std::vector<StaticCorridorConstraint> constraints;
    // 标准矩阵形式 C_matrix * Z <= d_vector
    Eigen::MatrixXd c_matrix;
    Eigen::VectorXd d_vector;
};

// 静态舒适走廊构建器：以 Z_ref 为基准点线性化 ESDF，产出舒适软约束矩阵。
class StaticCorridorBuilder {
   public:
    explicit StaticCorridorBuilder(const StaticCorridorBuilderConfig& config);
    // 对给定 Z_ref 序列逐点、逐外圆构建静态线性走廊。
    StaticCorridorBuilderResult build(
        const std::vector<TrajectoryPoint>& z_ref, const ESDFMap& esdf_map,
        const VehicleFootprintModel& footprint_model) const;

   protected:
    // 校验输入合法性
    void validateInputs(const std::vector<TrajectoryPoint>& z_ref,
                        const ESDFMap& esdf_map,
                        const VehicleFootprintModel& footprint_model) const;
    // 计算单条约束的右端项
    double computeDScalar(double dist_ref, const Eigen::VectorXd& a_row,
                          const Eigen::VectorXd& z_ref, double radius,
                          double margin) const;
    // 将展开约束列表填充为矩阵形式
    void assembleMatrixForm(StaticCorridorBuilderResult& result) const;

   protected:
    StaticCorridorBuilderConfig config_;
};
}  // namespace apa_post_processor
