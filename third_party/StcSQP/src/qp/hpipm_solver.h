#pragma once

#include "qp_solver.h"

namespace stc_SQP {
// HPIPM OCP QP 求解器包装类
class HPIPMQPSolver : public QPSolver {
public:
    // 使用预测步数，状态维度，控制维度，带边界的状态维度，带边界的控制维度，普通约束维度，每步软约束维度构造求解器实例
    // cond_N 为宏观步数（N / block_size）不是块大小，cond_N = -1 表示不启用 condensing，直接求解原始 OCP QP
    HPIPMQPSolver(int N, int nx, int nu, int nbx, int nbu, int ng, int ns,
        int cond_N = -1);
    ~HPIPMQPSolver();
    HPIPMQPSolver(const HPIPMQPSolver&) = delete;
    HPIPMQPSolver& operator=(const HPIPMQPSolver&) = delete;
    // 求解 QP，结果写入 qp_sol
    QPSolverStatus solve(const QPData& qp_data, QPSolution& qp_sol) override;
    // 设置求解精度
    void setTolerance(double tol) override;
    // 设置热启动
    void setWarmStart(const QPSolution& qp_sol) override;

protected:
    // 将 HPIPM 内部返回的整数状态码映射为本项目统一的 QPSolverStatus
    QPSolverStatus mapHpipmStatus(int status) const;

private:
    // PIMPL：隐藏 HPIPM C 结构体、对齐内存缓冲及可复用临时缓冲
    struct Impl;
    // HPIPM 求解器实现实例（禁止拷贝/移动，与 QPSolver 接口生命周期绑定）
    std::unique_ptr<Impl> pimpl_;
};
} // namespace stc_SQP
