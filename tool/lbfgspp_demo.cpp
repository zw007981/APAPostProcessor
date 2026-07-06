/// @file    tool/lbfgspp_demo.cpp
/// @brief   LBFGSpp 头文件可用性验证 Demo
///
/// 使用 L-BFGS 求解 Rosenbrock 函数的最小值，验证 LBFGSpp 在本项目中
/// 的头文件包含与编译链接是否正常。
///
/// Rosenbrock 函数: f(x,y) = 100*(y - x^2)^2 + (1 - x)^2
/// 全局最小值: f(1,1) = 0

#include <cmath>
#include <cstdio>
#include <Eigen/Core>
#include <LBFGS.h>

using namespace LBFGSpp;

/// @brief Rosenbrock 函数算子：返回函数值并填充梯度
class Rosenbrock {
public:
    using Scalar = double;
    using Vector = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;

    /// @brief 计算 f(x) 并将梯度写入 grad（引用传出）
    Scalar operator()(const Vector& x, Vector& grad) {
        const Scalar x1 = x[0];
        const Scalar x2 = x[1];
        const Scalar t1 = x2 - x1 * x1;
        const Scalar t2 = Scalar(1) - x1;

        // 函数值
        const Scalar fx = Scalar(100) * t1 * t1 + t2 * t2;

        // 梯度: df/dx1 = -400*x1*(x2 - x1^2) - 2*(1 - x1)
        //        df/dx2 =  200*(x2 - x1^2)
        if (grad.size() == 2) {
            grad[0] = Scalar(-400) * x1 * t1 - Scalar(2) * t2;
            grad[1] = Scalar(200) * t1;
        }
        return fx;
    }
};

int main() {
    // 初始猜测：x0 = [-1.2, 1.0]，典型 Rosenbrock 测试起点
    Eigen::VectorXd x(2);
    x << -1.2, 1.0;

    // 配置 L-BFGS 参数
    LBFGSParam<double> param;
    param.epsilon   = 1e-6;
    param.max_iterations = 100;

    // 创建求解器并求解
    LBFGSSolver<double> solver(param);
    Rosenbrock fun;

    double fx_opt = 0.0;
    int niter = solver.minimize(fun, x, fx_opt);

    std::printf("===== LBFGSpp Demo 求解结果 =====\n");
    std::printf("迭代次数: %d\n", niter);
    std::printf("最优解 x:\n");
    std::printf("  x[0] = %.6f\n", x[0]);
    std::printf("  x[1] = %.6f\n", x[1]);
    std::printf("目标函数值: %.10f\n", fx_opt);

    // 验证：理论最优 x* = [1, 1], f* = 0
    const double tol = 1e-4;
    bool ok = true;
    if (std::fabs(x[0] - 1.0) > tol) {
        std::printf("[FAIL] x[0] = %.6f 偏离预期 1.0\n", x[0]);
        ok = false;
    }
    if (std::fabs(x[1] - 1.0) > tol) {
        std::printf("[FAIL] x[1] = %.6f 偏离预期 1.0\n", x[1]);
        ok = false;
    }
    if (std::fabs(fx_opt) > tol) {
        std::printf("[FAIL] f* = %.10f 偏离预期 0.0\n", fx_opt);
        ok = false;
    }

    if (ok) {
        std::printf("\n[PASS] LBFGSpp 集成验证通过！\n");
    }

    return ok ? 0 : 1;
}
