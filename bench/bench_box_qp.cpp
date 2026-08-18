#include <benchmark/benchmark.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "core/iLQR/box_qp.h"

namespace apa_post_processor {
namespace {

using QpSolver = BoxQpSolver<>;

// 计时场景：单轮后向传递沿时间轴求解 N=399 个控制盒约束 QP，
// 与 iLQR 标称时域一致；相邻时间步的 QP 高度相似（H/q/盒边界沿步缓变），
// 正是活动集热启动的目标工况
constexpr std::size_t kChainLength = 399;

// 构造一条缓变 QP 链：基准 H₀ = AᵀA + I 良态正定，扰动量级远小于对角余量，
// 保证全链正定性；初始猜测恒为零向量（对应 δu 的冷启动初值）
void MakeQpChain(std::vector<QpSolver::Problem>* chain) {
    chain->clear();
    chain->reserve(kChainLength);
    QpSolver::Mat a;
    a << 0.9, -0.4, 0.3, 1.1;
    const QpSolver::Mat h0 = a.transpose() * a + QpSolver::Mat::Identity();
    for (std::size_t k = 0; k < kChainLength; ++k) {
        const double t = static_cast<double>(k);
        QpSolver::Problem problem;
        QpSolver::Mat perturb = QpSolver::Mat::Zero();
        perturb(0, 0) = 0.02 * std::sin(0.05 * t);
        perturb(0, 1) = 0.01 * std::cos(0.07 * t);
        perturb(1, 0) = perturb(0, 1);
        perturb(1, 1) = 0.02 * std::cos(0.06 * t);
        problem.hessian = h0 + perturb;
        problem.gradient = (QpSolver::Vec() << -1.5 + 0.8 * std::sin(0.04 * t),
                            -2.0 + 1.2 * std::cos(0.03 * t))
                               .finished();
        problem.lower = (QpSolver::Vec() << -1.2 - 0.1 * std::sin(0.05 * t),
                         -0.8 - 0.1 * std::cos(0.04 * t))
                            .finished();
        problem.upper = (QpSolver::Vec() << 1.2 + 0.1 * std::cos(0.05 * t),
                         0.8 + 0.1 * std::sin(0.06 * t))
                            .finished();
        problem.initial = QpSolver::Vec::Zero();
        chain->push_back(problem);
    }
}

// 冷启动链：每个 QP 独立从零梯度钳制集出发，统计总分解数作为对照基线
void BM_BoxQpColdStartChain(benchmark::State& state) {
    std::vector<QpSolver::Problem> chain;
    MakeQpChain(&chain);
    const QpSolver solver;
    for (auto _ : state) {
        std::int64_t factorizations = 0;
        std::int64_t iterations = 0;
        for (const auto& problem : chain) {
            const QpSolver::Result result = solver.solve(problem);
            factorizations += result.factorizations;
            iterations += result.iterations;
        }
        benchmark::DoNotOptimize(factorizations);
        state.counters["factorizations_per_qp"] =
            static_cast<double>(factorizations) /
            static_cast<double>(kChainLength);
        state.counters["iterations_per_qp"] =
            static_cast<double>(iterations) / static_cast<double>(kChainLength);
    }
    state.SetItemsProcessed(static_cast<std::int64_t>(state.iterations()) *
                            static_cast<int64_t>(kChainLength));
}
BENCHMARK(BM_BoxQpColdStartChain);

// 热启动链：沿时间轴顺序求解，每个 QP 以上一步的解为初始点、
// 以上一步的最终活动集热启动——模拟后向传递相邻步 QP 的消费方式，
// 验证 Tassa 实验结论「热启动后平均每次迭代分解数 < 2」
void BM_BoxQpWarmStartChain(benchmark::State& state) {
    std::vector<QpSolver::Problem> chain;
    MakeQpChain(&chain);
    const QpSolver solver;
    for (auto _ : state) {
        std::int64_t factorizations = 0;
        std::int64_t iterations = 0;
        QpSolver::Result previous;
        for (std::size_t k = 0; k < kChainLength; ++k) {
            QpSolver::Problem problem = chain[k];
            if (k > 0) {
                problem.warm_start = true;
                problem.initial_clamped = previous.clamped;
                problem.initial = previous.x;
            }
            previous = solver.solve(problem);
            factorizations += previous.factorizations;
            iterations += previous.iterations;
            benchmark::DoNotOptimize(previous);
        }
        benchmark::DoNotOptimize(factorizations);
        state.counters["factorizations_per_qp"] =
            static_cast<double>(factorizations) /
            static_cast<double>(kChainLength);
        state.counters["iterations_per_qp"] =
            static_cast<double>(iterations) / static_cast<double>(kChainLength);
    }
    state.SetItemsProcessed(static_cast<std::int64_t>(state.iterations()) *
                            static_cast<int64_t>(kChainLength));
}
BENCHMARK(BM_BoxQpWarmStartChain);

}  // namespace
}  // namespace apa_post_processor
