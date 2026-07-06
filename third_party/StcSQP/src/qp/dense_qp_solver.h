#pragma once

#include <cmath>
#include <stdexcept>

#include "qp_solver.h"
#include "util/constants.h"

namespace stc_SQP {
// 基于 Eigen LDLT 的稠密 QP 求解器（参考实现）采用原始 active-set 策略，将 OCP QP 展开为单个稠密 QP 后求解
// 注意：本求解器定位为调试 oracle / HPIPM 对照，不建议作为实时主求解器
class DenseQPSolver : public QPSolver {
public:
    DenseQPSolver() = default;
    // 求解 QP 问题，结果写入 qp_sol
    QPSolverStatus solve(const QPData& qp_data, QPSolution& qp_sol) override;
    // 设置求解精度。tol 必须是有限正数，否则抛 std::invalid_argument
    void setTolerance(double tol) override {
        if (!std::isfinite(tol) || tol <= 0.0) {
            throw std::invalid_argument("DenseQPSolver::setTolerance: tol must be a finite positive number");
        }
        tol_ = tol;
    }
    // 设置热启动（当前参考实现不实现热启动）
    void setWarmStart(const QPSolution&) override {}

protected:
    // 稠密变量布局：记录 OCP 展开后的维度与偏移
    struct Layout {
        // 预测时域
        int N = 0;
        // 状态维度
        int nx = 0;
        // 控制维度
        int nu = 0;
        // 普通约束维度
        int ng = 0;
        // 每步软约束维度
        int ns = 0;
        // 稠密变量总维度
        int n_var = 0;
        // 是否启用软约束
        bool has_soft = false;
        // x_k 在稠密变量中的起始偏移
        int xOffset(int k) const { return k * nx; }
        // u_k 在稠密变量中的起始偏移
        int uOffset(int k) const { return (N + 1) * nx + k * nu; }
        // s_k 在稠密变量中的起始偏移
        int sOffset(int k) const { return (N + 1) * nx + N * nu + k * ns; }
    };

    // 稠密 QP 装配结果，用于将代价、等式、不等式约束打包传入 active-set 求解器
    struct Assembly {
        Matrix H;
        Vector g;
        Matrix A_eq;
        Vector b_eq;
        Matrix A_ineq;
        Vector b_ineq;
    };

    // 校验 QPData 维度与软约束配置是否满足 Dense 装配要求
    QPSolverStatus validateInput(const QPData& qp_data) const;
    // 校验 QPData 中各容器尺寸是否与 N / N+1 匹配，且维度合法
    bool hasValidDimensionsAndContainers(const QPData& qp_data) const;
    // 根据 QPData 构造稠密布局
    Layout buildLayout(const QPData& qp_data) const;
    // 判断初始状态是否被 lbx[0] == ubx[0] 固定；要求两侧均有限
    bool isInitialStateFixed(const QPData& qp_data) const;
    // 装配 Hessian 与梯度（仅实现上界 slack Zu/zu）
    void assembleCost(const QPData& qp_data, const Layout& layout,
        Matrix& H, Vector& g) const;
    // 装配等式约束：初始状态固定 + 动力学
    void assembleEqualityConstraints(const QPData& qp_data, const Layout& layout,
        Matrix& A_eq, Vector& b_eq) const;
    // 装配不等式约束：状态/控制盒边界、普通约束、软约束替换、slack 非负
    void assembleInequalityConstraints(const QPData& qp_data, const Layout& layout,
        Matrix& A_ineq, Vector& b_ineq) const;
    // 向不等式约束追加状态盒边界；返回新的行号
    int appendStateBounds(const QPData& qp_data, const Layout& layout, bool x0_fixed,
        Matrix& A_ineq, Vector& b_ineq, int row) const;
    // 向不等式约束追加控制盒边界；返回新的行号
    int appendControlBounds(const QPData& qp_data, const Layout& layout,
        Matrix& A_ineq, Vector& b_ineq, int row) const;
    // 向不等式约束追加普通约束与软约束替换；返回新的行号
    int appendGeneralConstraints(const QPData& qp_data, const Layout& layout,
        const std::vector<int>& soft_local,
        Matrix& A_ineq, Vector& b_ineq, int row) const;
    // 向不等式约束追加 slack 非负约束；返回新的行号
    int appendSlackNonNegativity(const Layout& layout,
        Matrix& A_ineq, Vector& b_ineq, int row) const;
    // active-set 求解稠密 QP，结果写入 z
    QPSolverStatus solveActiveSet(const Assembly& assembly, Vector& z) const;
    // 将稠密解向量 z 写回 QPSolution
    void writeSolution(const Vector& z, const Layout& layout,
        QPSolution& qp_sol) const;

protected:
    // 求解精度与数值阈值；必须由 setTolerance 设置为有限正数
    double tol_ = EPSILON_PRECISE;
};
} // namespace stc_SQP
