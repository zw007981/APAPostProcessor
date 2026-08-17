#include <benchmark/benchmark.h>

#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include "core/DDP/bicycle_dynamics.h"
#include "core/DDP/ddp_cost.h"
#include "core/DDP/ms_ilqr.h"

namespace apa_post_processor {
namespace {

// 标称时域：N=399 步、7 维状态（与泊车后处理标称规模一致）
constexpr std::size_t kNumSteps = 399;
constexpr double kWheelbase = 2.7;
constexpr double kDt = 0.1;

// 白盒访问：派生类公开受保护方法，便于对单次回推/双 rollout 独立计时
class MsIlqrBenchAccess : public MsIlqrSolver {
   public:
    using MsIlqrSolver::backwardPass;
    using MsIlqrSolver::computeJacobians;
    using MsIlqrSolver::evaluateNominal;
    using MsIlqrSolver::linearRollout;
    using MsIlqrSolver::MsIlqrSolver;
    using MsIlqrSolver::nonlinearRollout;
    using MsIlqrSolver::prepareWorkspace;
    using MsIlqrSolver::qp_factorization_count_;
    using MsIlqrSolver::setNominalTrajectory;
    using MsIlqrSolver::setShootingLookup;
};

// 计时场景数据：缓变 S 形参考位姿、一致性名义轨迹与小量级非零控制，
// 打靶节点按 n_s=25 规则布设（含末点），与生产配置同构
struct BenchProblem {
    DdpReference reference;
    DdpAlignedVec<DdpState> states;
    DdpAlignedVec<DdpControl> controls;
    DdpCostMultiplierState multipliers;
    DdpCostInput input;
};

BenchProblem MakeBenchProblem() {
    BenchProblem problem;
    problem.multipliers = DdpCostMultiplierState::MakeZero(kNumSteps);
    problem.input.tracking_weight = 10.0;
    problem.input.anneal_exempt_mask = nullptr;
    problem.reference.ds = 0.05;
    problem.reference.dt = kDt;
    problem.reference.poses.reserve(kNumSteps + 1);
    problem.reference.shooting_nodes.reserve(kNumSteps / 25 + 2);
    const BicycleDynamics dynamics(kWheelbase);
    DdpState state;
    state << 0.0, 0.0, 0.0, 0.5, 0.0, 0.05, 0.0, 0.0;
    problem.states.reserve(kNumSteps + 1);
    problem.controls.reserve(kNumSteps);
    problem.states.push_back(state);
    for (std::size_t k = 0; k < kNumSteps; ++k) {
        DdpControl control;
        control << 0.02 * (k % 3 == 0 ? 1.0 : -0.6),
            0.01 * (k % 4 == 0 ? 1.0 : -0.5);
        problem.controls.push_back(control);
        state = dynamics.step(state, control, kDt);
        problem.states.push_back(state);
        if (k % 25 == 0) {
            problem.reference.shooting_nodes.push_back(k);
        }
    }
    for (const auto& node_state : problem.states) {
        problem.reference.poses.emplace_back(node_state(DDP_IDX_X) + 0.02,
                                             node_state(DDP_IDX_Y) - 0.01,
                                             node_state(DDP_IDX_THETA) + 0.01);
    }
    problem.reference.shooting_nodes.push_back(kNumSteps);
    return problem;
}

void PrepareSolver(MsIlqrBenchAccess* solver, const BenchProblem& problem) {
    solver->prepareWorkspace(kNumSteps);
    solver->setShootingLookup(problem.reference.shooting_nodes);
    solver->setNominalTrajectory(problem.reference, problem.states,
                                 problem.controls);
    solver->evaluateNominal(problem.reference, problem.multipliers,
                            problem.input);
    solver->computeJacobians(problem.reference);
}

// 单轮缺陷感知回推（N=399，含逐步 box-QP，活动集沿回推顺序热启动）
void BM_MsIlqrBackwardPass(benchmark::State& state) {
    const BicycleDynamics dynamics(kWheelbase);
    const DdpCostEvaluator evaluator(DdpCostConfig{}, nullptr);
    const BenchProblem problem = MakeBenchProblem();
    MsIlqrBenchAccess solver(MsIlqrConfig{}, &dynamics, &evaluator);
    PrepareSolver(&solver, problem);
    for (auto _ : state) {
        bool success = solver.backwardPass();
        benchmark::DoNotOptimize(success);
    }
    state.counters["factorizations_per_step"] =
        static_cast<double>(solver.qp_factorization_count_) /
        (static_cast<double>(state.iterations()) *
         static_cast<double>(kNumSteps));
}
BENCHMARK(BM_MsIlqrBackwardPass);

// 单次线性 rollout（方向传播 + EC₁/EC₂ 汇总，每轮迭代恰好一次）
void BM_MsIlqrLinearRollout(benchmark::State& state) {
    const BicycleDynamics dynamics(kWheelbase);
    const DdpCostEvaluator evaluator(DdpCostConfig{}, nullptr);
    const BenchProblem problem = MakeBenchProblem();
    MsIlqrBenchAccess solver(MsIlqrConfig{}, &dynamics, &evaluator);
    PrepareSolver(&solver, problem);
    bool success = solver.backwardPass();
    benchmark::DoNotOptimize(success);
    for (auto _ : state) {
        solver.linearRollout();
    }
}
BENCHMARK(BM_MsIlqrLinearRollout);

// 单次非线性 rollout（闭环跟踪 + 缺陷缩放 + 候选全轨迹代价求值，
// 线搜索逐候选 α 调用）
void BM_MsIlqrNonlinearRollout(benchmark::State& state) {
    const BicycleDynamics dynamics(kWheelbase);
    const DdpCostEvaluator evaluator(DdpCostConfig{}, nullptr);
    const BenchProblem problem = MakeBenchProblem();
    MsIlqrBenchAccess solver(MsIlqrConfig{}, &dynamics, &evaluator);
    PrepareSolver(&solver, problem);
    bool success = solver.backwardPass();
    benchmark::DoNotOptimize(success);
    solver.linearRollout();
    for (auto _ : state) {
        // 第 5 参为 merit 早停阈值：取正无穷禁用 ESDF 早停，压测
        // 完整非线性 rollout 路径（本 bench 无 ESDF 约束，阈值无影响）
        double cost = solver.nonlinearRollout(
            0.8, problem.reference, problem.multipliers, problem.input,
            std::numeric_limits<double>::infinity());
        benchmark::DoNotOptimize(cost);
    }
}
BENCHMARK(BM_MsIlqrNonlinearRollout);

}  // namespace
}  // namespace apa_post_processor
