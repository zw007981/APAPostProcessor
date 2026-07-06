#include <atomic>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "../src/generated/dynamics_kappa.h"
#include "../src/generated/dynamics_kappa_jac.h"
#include "../src/math/so2.hpp"
#include "../src/models/casadi_wrapper.h"

using namespace stc_SQP;
namespace {
// 用于异常路径测试的 stub：work 函数返回非 0
int failing_work(long long* /*sz_arg*/, long long* /*sz_res*/, long long* /*sz_iw*/,
    long long* /*sz_w*/)
{
    return 1;
}

// 用于异常路径测试的 stub：func 函数返回非 0
int failing_func(const double** /*arg*/, double** /*res*/, long long* /*iw*/,
    double* /*w*/, int /*mem*/)
{
    return 1;
}

// 返回 0 大小的正常 work 函数 stub
int ok_work(long long* sz_arg, long long* sz_res, long long* sz_iw, long long* sz_w)
{
    if (sz_arg) {
        *sz_arg = 2;
    }
    if (sz_res) {
        *sz_res = 1;
    }
    if (sz_iw) {
        *sz_iw = 0;
    }
    if (sz_w) {
        *sz_w = 0;
    }
    return 0;
}

} // namespace

// CasADiWrapper 测试套件：验证生成代码包装器的内存预分配与线程克隆机制
TEST(CasADiWrapper, PreallocatesWorkspacesFromQuery)
{
    // 测试目的：验证构造 CasADiFunction 时会调用 work 函数查询 iw/w 大小并预分配
    // 流程：用 dynamics_kappa_jac（输出 [f, A, B]，n_out=3）实例化包装器
    // 预期效果：n_in=2, n_out=3，且工作区大小查询结果非负（当前生成代码中为 0）
    casadi::CasADiFunction func(
        reinterpret_cast<casadi::FunctionPointer>(dynamics_kappa_jac),
        reinterpret_cast<casadi::WorkSizeFunction>(dynamics_kappa_jac_work), 2, 3);
    EXPECT_EQ(func.n_in(), 2);
    EXPECT_EQ(func.n_out(), 3);
    EXPECT_GE(func.iw_size(), 0);
    EXPECT_GE(func.w_size(), 0);
}

TEST(CasADiWrapper, EvaluatesKappaDynamicsCorrectly)
{
    // 测试目的：验证包装器能正确调用 dynamics_kappa 生成函数
    // 流程：给定状态与控制，调用 operator()，与手算解析结果比较
    // 预期效果：输出 f = [v*cosθ, v*sinθ, v*κ, a, κ_dot]
    casadi::CasADiFunction func(
        reinterpret_cast<casadi::FunctionPointer>(dynamics_kappa),
        reinterpret_cast<casadi::WorkSizeFunction>(dynamics_kappa_work), 2, 1);
    Vector x(5), u(2);
    x << 1.0, 0.5, 0.3, 2.0, 0.05;
    u << 0.5, 0.02;
    Vector f(5);
    std::vector<const double*> arg = { x.data(), u.data() };
    std::vector<double*> res = { f.data() };
    func(arg, res);
    EXPECT_NEAR(f(0), 2.0 * std::cos(0.3), 1e-12);
    EXPECT_NEAR(f(1), 2.0 * std::sin(0.3), 1e-12);
    EXPECT_NEAR(f(2), 2.0 * 0.05, 1e-12);
    EXPECT_NEAR(f(3), 0.5, 1e-12);
    EXPECT_NEAR(f(4), 0.02, 1e-12);
}

TEST(CasADiWrapper, CloneCreatesIndependentInstance)
{
    // 测试目的：验证 clone() 生成的副本拥有独立工作区，原对象与副本互不影响
    // 流程：clone 后分别用不同输入调用原对象和副本，检查输出不受干扰
    // 预期效果：两次调用结果均正确，说明 iw/w 缓冲区已隔离
    casadi::CasADiFunction func(
        reinterpret_cast<casadi::FunctionPointer>(dynamics_kappa),
        reinterpret_cast<casadi::WorkSizeFunction>(dynamics_kappa_work), 2, 1);
    casadi::CasADiFunction cloned = func.clone();
    Vector x_a(5), u_a(2), x_b(5), u_b(2);
    x_a << 1.0, 0.0, 0.0, 1.0, 0.0;
    u_a << 0.0, 0.0;
    x_b << 0.0, 1.0, PI / 2.0, 1.0, 0.0;
    u_b << 0.0, 0.0;
    Vector f_a(5), f_b(5);
    std::vector<const double*> arg_a = { x_a.data(), u_a.data() };
    std::vector<double*> res_a = { f_a.data() };
    std::vector<const double*> arg_b = { x_b.data(), u_b.data() };
    std::vector<double*> res_b = { f_b.data() };
    func(arg_a, res_a);
    cloned(arg_b, res_b);
    EXPECT_NEAR(f_a(0), 1.0, 1e-12);
    EXPECT_NEAR(f_a(1), 0.0, 1e-12);
    EXPECT_NEAR(f_b(0), 0.0, 1e-12);
    EXPECT_NEAR(f_b(1), 1.0, 1e-12);
}

TEST(CasADiWrapper, CloneSupportsConcurrentEvaluation)
{
    // 测试目的：验证 clone() 后的实例可在多线程中并发调用而不产生数据竞争
    // 流程：主实例与克隆实例分别交给两个线程，各执行 1000 次不同输入的调用
    // 预期效果：所有调用均成功，无段错误，结果与单线程一致
    casadi::CasADiFunction func(
        reinterpret_cast<casadi::FunctionPointer>(dynamics_kappa),
        reinterpret_cast<casadi::WorkSizeFunction>(dynamics_kappa_work), 2, 1);
    casadi::CasADiFunction cloned = func.clone();
    std::atomic<int> success_count(0);
    const int iterations = 1000;
    auto worker = [&](casadi::CasADiFunction& f, double theta_offset) {
        for (int i = 0; i < iterations; ++i) {
            Vector x(5), u(2), f_out(5);
            const double theta = theta_offset + static_cast<double>(i) * 0.001;
            x << 0.0, 0.0, theta, 1.0, 0.0;
            u << 0.0, 0.0;
            std::vector<const double*> arg = { x.data(), u.data() };
            std::vector<double*> res = { f_out.data() };
            f(arg, res);
            const double expected_x = std::cos(theta), expected_y = std::sin(theta);
            if (std::abs(f_out(0) - expected_x) < 1e-10 && std::abs(f_out(1) - expected_y) < 1e-10) {
                success_count.fetch_add(1, std::memory_order_relaxed);
            }
        }
    };
    std::thread t1([&]() { worker(func, 0.0); });
    std::thread t2([&]() { worker(cloned, PI / 2.0); });
    t1.join();
    t2.join();
    EXPECT_EQ(success_count.load(), 2 * iterations)
        << "Concurrent calls produced abnormal results or data race";
}

TEST(CasADiWrapper, RejectsNullFunctionPointers)
{
    // 测试目的：验证构造函数对空 func 或空 work_func 抛出 std::invalid_argument
    // 流程：分别传入 nullptr 作为 func 和 work_func 尝试构造
    // 预期效果：均抛出 std::invalid_argument
    EXPECT_THROW(
        casadi::CasADiFunction(nullptr,
            reinterpret_cast<casadi::WorkSizeFunction>(dynamics_kappa_work),
            2, 1),
        std::invalid_argument);
    EXPECT_THROW(
        casadi::CasADiFunction(reinterpret_cast<casadi::FunctionPointer>(dynamics_kappa),
            nullptr, 2, 1),
        std::invalid_argument);
}

TEST(CasADiWrapper, RejectsMismatchedInputCount)
{
    // 测试目的：验证 operator() 在 arg 数量不等于 n_in 时抛出异常
    // 流程：构造正确包装器，但调用时只传入 1 个输入而非 2 个
    // 预期效果：抛出 std::invalid_argument
    casadi::CasADiFunction func(
        reinterpret_cast<casadi::FunctionPointer>(dynamics_kappa),
        reinterpret_cast<casadi::WorkSizeFunction>(dynamics_kappa_work), 2, 1);
    Vector x(5), f(5);
    std::vector<const double*> arg = { x.data() }; // 缺少 u
    std::vector<double*> res = { f.data() };
    EXPECT_THROW(func(arg, res), std::invalid_argument);
}

TEST(CasADiWrapper, RejectsMismatchedOutputCount)
{
    // 测试目的：验证 operator() 在 res 数量不等于 n_out 时抛出异常
    // 流程：构造正确包装器，但调用时 res 为空
    // 预期效果：抛出 std::invalid_argument
    casadi::CasADiFunction func(
        reinterpret_cast<casadi::FunctionPointer>(dynamics_kappa),
        reinterpret_cast<casadi::WorkSizeFunction>(dynamics_kappa_work), 2, 1);
    Vector x(5), u(2);
    std::vector<const double*> arg = { x.data(), u.data() };
    std::vector<double*> res; // 缺少输出
    EXPECT_THROW(func(arg, res), std::invalid_argument);
}

TEST(CasADiWrapper, ThrowsWhenWorkQueryFails)
{
    // 测试目的：验证 work_func 返回非 0 时构造函数抛出 std::runtime_error
    // 流程：用正常 func 与失败 work stub 构造包装器
    // 预期效果：抛出 std::runtime_error
    EXPECT_THROW(casadi::CasADiFunction(
                     reinterpret_cast<casadi::FunctionPointer>(dynamics_kappa), failing_work, 2,
                     1),
        std::runtime_error);
}

TEST(CasADiWrapper, ThrowsWhenFunctionExecutionFails)
{
    // 测试目的：验证 func_ 返回非 0 时 operator() 抛出 std::runtime_error
    // 流程：用失败 func stub 与正常 work stub 构造包装器并调用
    // 预期效果：抛出 std::runtime_error
    casadi::CasADiFunction func(reinterpret_cast<casadi::FunctionPointer>(failing_func), ok_work,
        2, 1);
    Vector x(5), u(2), f(5);
    std::vector<const double*> arg = { x.data(), u.data() };
    std::vector<double*> res = { f.data() };
    EXPECT_THROW(func(arg, res), std::runtime_error);
}
