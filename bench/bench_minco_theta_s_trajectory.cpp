#include <benchmark/benchmark.h>

#include <Eigen/Core>
#include <Eigen/LU>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <random>
#include <vector>

#include "core/MINCO_THETA_S/block_tridiagonal_solver.h"
#include "core/MINCO_THETA_S/minco_theta_s_trajectory.h"

namespace apa_post_processor {
namespace {

using Block = BlockTridiagonalSolver::Block;
using BlockMatrix = BlockTridiagonalSolver::BlockMatrix;
constexpr int BLOCK_SIZE = BlockTridiagonalSolver::BLOCK_SIZE;

// 生成固定种子的随机块三对角系统与右端项：主对角块叠加 6I 保证对角占优，
// 数据在 benchmark 循环外一次性生成，不计入计时
void BuildBenchmarkSystem(int num_blocks, std::vector<Block>* lower,
                          std::vector<Block>* diagonal,
                          std::vector<Block>* upper, BlockMatrix* rhs) {
    std::mt19937 rng(42);
    std::uniform_real_distribution<double> dist(-1.0, 1.0);
    const auto random_block = [&dist, &rng]() {
        Block block;
        for (int row = 0; row < BLOCK_SIZE; ++row) {
            for (int col = 0; col < BLOCK_SIZE; ++col) {
                block(row, col) = dist(rng);
            }
        }
        return block;
    };
    lower->reserve(num_blocks - 1);
    diagonal->reserve(num_blocks);
    upper->reserve(num_blocks - 1);
    for (int i = 0; i < num_blocks; ++i) {
        diagonal->push_back(random_block() + 6.0 * Block::Identity());
    }
    for (int i = 0; i + 1 < num_blocks; ++i) {
        lower->push_back(random_block());
        upper->push_back(random_block());
    }
    *rhs = BlockMatrix::NullaryExpr(BLOCK_SIZE, num_blocks,
                                    [&dist, &rng]() { return dist(rng); });
}

// 块 Thomas 分解 + 单次正向求解的端到端耗时，应随块数 M 呈 O(M) 线性增长
void BM_MincoThetaSBlockThomasSolve(benchmark::State& state) {
    const int num_blocks = static_cast<int>(state.range(0));
    std::vector<Block> lower, diagonal, upper;
    BlockMatrix rhs;
    BuildBenchmarkSystem(num_blocks, &lower, &diagonal, &upper, &rhs);
    for (auto _ : state) {
        BlockTridiagonalSolver solver;
        solver.factorize(lower, diagonal, upper);
        // 非 const 左值以匹配 DoNotOptimize(Tp&) 重载（const-ref 重载已废弃）
        BlockMatrix x = solver.solve(rhs);
        benchmark::DoNotOptimize(x);
    }
}

// 稠密 PartialPivLU 参考：O(M^3) 增长，与块 Thomas 对照验证复杂度优势；
// 块数过大会导致单轮耗时过长，因此只覆盖小块数档位
void BM_MincoThetaSDenseReferenceSolve(benchmark::State& state) {
    const int num_blocks = static_cast<int>(state.range(0));
    std::vector<Block> lower, diagonal, upper;
    BlockMatrix rhs;
    BuildBenchmarkSystem(num_blocks, &lower, &diagonal, &upper, &rhs);
    Eigen::MatrixXd dense =
        Eigen::MatrixXd::Zero(BLOCK_SIZE * num_blocks, BLOCK_SIZE * num_blocks);
    for (int i = 0; i < num_blocks; ++i) {
        dense.block<BLOCK_SIZE, BLOCK_SIZE>(BLOCK_SIZE * i, BLOCK_SIZE * i) =
            diagonal[i];
    }
    for (int i = 0; i + 1 < num_blocks; ++i) {
        dense.block<BLOCK_SIZE, BLOCK_SIZE>(BLOCK_SIZE * (i + 1),
                                            BLOCK_SIZE * i) = lower[i];
        dense.block<BLOCK_SIZE, BLOCK_SIZE>(BLOCK_SIZE * i,
                                            BLOCK_SIZE * (i + 1)) = upper[i];
    }
    const Eigen::VectorXd rhs_vec =
        Eigen::Map<const Eigen::VectorXd>(rhs.data(), dense.rows());
    for (auto _ : state) {
        // 非 const 左值以匹配 DoNotOptimize(Tp&) 重载（const-ref 重载已废弃）
        Eigen::VectorXd x = dense.partialPivLu().solve(rhs_vec);
        benchmark::DoNotOptimize(x);
    }
}

// MincoThetaSTrajectory 端到端构建（装配 K(T)/b + 分解 + θ/s 两维求解）耗时
void BM_MincoThetaSTrajectorySetTrajectory(benchmark::State& state) {
    const int num_segments = static_cast<int>(state.range(0));
    std::mt19937 rng(42);
    std::uniform_real_distribution<double> dist(0.5, 2.0);
    std::vector<double> durations;
    durations.reserve(num_segments);
    std::vector<Eigen::Vector2d> waypoints;
    waypoints.reserve(num_segments - 1);
    for (int i = 0; i < num_segments; ++i) {
        durations.push_back(dist(rng));
    }
    for (int i = 0; i + 1 < num_segments; ++i) {
        waypoints.emplace_back(0.1 * i, 0.6 * i);
    }
    const MincoThetaSBoundaryCondition2d start{{0.0, 0.1, 0.0}, {0.0, 0.5, 0.0}};
    const MincoThetaSBoundaryCondition2d end{{1.0, 0.0, 0.0}, {10.0, 0.0, 0.0}};
    for (auto _ : state) {
        MincoThetaSTrajectory trajectory;
        trajectory.setTrajectory(start, end, waypoints, durations);
        benchmark::DoNotOptimize(trajectory);
    }
}

BENCHMARK(BM_MincoThetaSBlockThomasSolve)->Arg(10)->Arg(100)->Arg(1000)->Arg(10000);
BENCHMARK(BM_MincoThetaSDenseReferenceSolve)->Arg(10)->Arg(100);
BENCHMARK(BM_MincoThetaSTrajectorySetTrajectory)->Arg(10)->Arg(100)->Arg(1000);

// 构造固定参数、固定规模的基准轨迹：段时长与航点在计时循环外一次生成，
// 供段内求值类基准共用（求解热路径每轮都会重建轨迹，故这是真实形态）
MincoThetaSTrajectory BuildEvalBenchmarkTrajectory(int num_segments) {
    std::mt19937 rng(42);
    std::uniform_real_distribution<double> dist(0.5, 2.0);
    std::vector<double> durations;
    durations.reserve(num_segments);
    std::vector<Eigen::Vector2d> waypoints;
    waypoints.reserve(num_segments - 1);
    for (int i = 0; i < num_segments; ++i) {
        durations.push_back(dist(rng));
    }
    for (int i = 0; i + 1 < num_segments; ++i) {
        waypoints.emplace_back(0.1 * i, 0.6 * i);
    }
    const MincoThetaSBoundaryCondition2d start{{0.0, 0.1, 0.0}, {0.0, 0.5, 0.0}};
    const MincoThetaSBoundaryCondition2d end{{1.0, 0.0, 0.0}, {10.0, 0.0, 0.0}};
    MincoThetaSTrajectory trajectory;
    trajectory.setTrajectory(start, end, waypoints, durations);
    return trajectory;
}

// 段端点 1~4 阶导数：逐阶调用形态（∂J/∂T 装配改造前的写法，每段 8 次
// evaluateSegment），作为批量接口的对照基线
void BM_MincoThetaSSegmentEndDersPerCall(benchmark::State& state) {
    const int num_segments = static_cast<int>(state.range(0));
    const MincoThetaSTrajectory trajectory =
        BuildEvalBenchmarkTrajectory(num_segments);
    for (auto _ : state) {
        double acc = 0.0;
        for (int i = 0; i < num_segments; ++i) {
            const double duration_i = trajectory.duration(i);
            for (int order = 1; order <= 4; ++order) {
                acc += trajectory.evaluateSegment(i, 0.0, order).x();
                acc += trajectory.evaluateSegment(i, duration_i, order).y();
            }
        }
        benchmark::DoNotOptimize(acc);
    }
    state.SetItemsProcessed(state.iterations() * num_segments * 8);
}

// 段端点 3/4 阶导数批量接口 + 端点 1/2 阶复用节点样本：改造后的实际形态
// （每段一次批量调用替掉 4 次逐阶调用，1/2 阶不再求值）
void BM_MincoThetaSSegmentEndDersBatch(benchmark::State& state) {
    const int num_segments = static_cast<int>(state.range(0));
    const MincoThetaSTrajectory trajectory =
        BuildEvalBenchmarkTrajectory(num_segments);
    std::vector<MincoThetaSSegmentEndOrders34> ders(
        static_cast<std::size_t>(num_segments));
    for (auto _ : state) {
        double acc = 0.0;
        for (int i = 0; i < num_segments; ++i) {
            ders[static_cast<std::size_t>(i)] =
                trajectory.evaluateSegmentEndOrders34(i);
        }
        for (const auto& item : ders) {
            acc += item.start_order3.x() + item.end_order4.y();
        }
        benchmark::DoNotOptimize(acc);
    }
    state.SetItemsProcessed(state.iterations() * num_segments * 4);
}

BENCHMARK(BM_MincoThetaSSegmentEndDersPerCall)->Arg(10)->Arg(30)->Arg(60);
BENCHMARK(BM_MincoThetaSSegmentEndDersBatch)->Arg(10)->Arg(30)->Arg(60);
}  // namespace
}  // namespace apa_post_processor
