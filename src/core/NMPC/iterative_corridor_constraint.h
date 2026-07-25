#pragma once

#include <constraints/constraint.hpp>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "../../util/constants.h"

namespace apa_post_processor {
// 前向声明
class ESDFMap;
class VehicleFootprintModel;

// 迭代重新线性化走廊约束：每轮 SQP 迭代从当前状态查询 ESDF 并重建线性化走廊。
class IterativeCorridorConstraint : public stc_SQP::Constraint {
   public:
    // 构造迭代走廊约束。
    IterativeCorridorConstraint(
        const ESDFMap& esdf_map,
        const std::vector<Eigen::Vector2d>& circle_local_positions,
        double circle_radius, int global_start_idx, int constraints_per_step,
        double hard_margin = EPSILON_PRECISE);

    // 每步输出的约束行数
    int ng() const override;

    // 组合求值与雅可比，每轮 SQP 迭代调用一次。
    void evaluateAndJacobian(const stc_SQP::Vector& x, const stc_SQP::Vector& u,
                             const stc_SQP::Vector& p, stc_SQP::Vector& g,
                             stc_SQP::Matrix& Cx,
                             stc_SQP::Matrix& Cu) const override;

    // evaluate 单独求值（用于收敛检查等非热路径）
    void evaluate(const stc_SQP::Vector& x, const stc_SQP::Vector& u,
                  const stc_SQP::Vector& p, stc_SQP::Vector& g) const override;

    // jacobian 单独求雅可比（保留向后兼容）
    void jacobian(const stc_SQP::Vector& x, const stc_SQP::Vector& u,
                  const stc_SQP::Vector& p, stc_SQP::Matrix& Cx,
                  stc_SQP::Matrix& Cu) const override;

    // 深拷贝
    std::shared_ptr<stc_SQP::Constraint> clone() const override;

   protected:
    // 从 StageParameters 提取局部步索引
    int localStepIndex(const stc_SQP::Vector& p) const;

    // 对单个外圆计算约束行，返回 ESDF 梯度是否有效。
    bool computeCircleConstraint(double x_pos, double y_pos, double theta,
                                 const Eigen::Vector2d& local_offset,
                                 double& dist, Eigen::Vector2d& grad,
                                 Eigen::Matrix<double, 1, 5>& a_row,
                                 double& g_val) const;

   protected:
    const ESDFMap& esdf_map_;
    const std::vector<Eigen::Vector2d> circle_local_positions_;
    double circle_radius_;
    double hard_margin_;
    int global_start_idx_;
    int constraints_per_step_;
    static constexpr double kMinValidGradientNorm = 1e-12;
};
}  // namespace apa_post_processor
