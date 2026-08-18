#include <benchmark/benchmark.h>

#include <cmath>
#include <cstddef>
#include <vector>

#include "core/iLQR/bicycle_dynamics.h"

namespace apa_post_processor {
namespace {

// 计时场景：N=399 步全轨迹，与 iLQR 标称时域一致；轴距/步长取标称值
constexpr std::size_t kHorizonSteps = 399;
constexpr double kWheelbase = 3.0;
constexpr double kDt = 0.1;

// 构造一条有代表性的名义轨迹：速度/转角均时变（混合转向与一次换挡过零），
// 保证计时覆盖 tan/sec²/三角函数的全部非线性路径
void MakeNominalTrajectory(iLQRAlignedVec<iLQRState>* states,
                           iLQRAlignedVec<iLQRControl>* controls) {
    const BicycleDynamics dynamics(kWheelbase);
    states->clear();
    controls->clear();
    states->reserve(kHorizonSteps + 1);
    controls->reserve(kHorizonSteps);
    iLQRState x = iLQRState::Zero();
    x(ILQR_IDX_V) = 0.02;
    states->push_back(x);
    for (std::size_t k = 0; k < kHorizonSteps; ++k) {
        iLQRControl u;
        // 前 200 步前进、后 199 步倒车，j/η 均按平滑伪机动剖面变化
        const double sign = k < 200 ? 1.0 : -1.0;
        u(ILQR_IDX_JERK) = 0.15 * std::sin(0.05 * static_cast<double>(k)) * sign;
        u(ILQR_IDX_ETA) = 0.25 * std::cos(0.04 * static_cast<double>(k));
        controls->push_back(u);
        x = dynamics.step(x, u, kDt);
        states->push_back(x);
    }
}

// 单轮全轨迹线性化（N+1 次 jacobians 调用）：MS-iLQR 一次迭代的
// 「沿线性化」环节耗时基线
void BM_LinearizeFullTrajectory(benchmark::State& state) {
    const BicycleDynamics dynamics(kWheelbase);
    iLQRAlignedVec<iLQRState> states;
    iLQRAlignedVec<iLQRControl> controls;
    MakeNominalTrajectory(&states, &controls);
    iLQRStateJacobian A;
    iLQRControlJacobian B;
    for (auto _ : state) {
        for (std::size_t k = 0; k < kHorizonSteps; ++k) {
            dynamics.jacobians(states[k], controls[k], kDt, &A, &B);
            benchmark::DoNotOptimize(A);
            benchmark::DoNotOptimize(B);
        }
    }
    state.SetItemsProcessed(static_cast<std::int64_t>(state.iterations()) *
                            static_cast<std::int64_t>(kHorizonSteps));
}
BENCHMARK(BM_LinearizeFullTrajectory);

// 单轮全轨迹 rollout（N 次 step 调用）：前向传递单次线搜索候选的耗时基线
void BM_RolloutFullTrajectory(benchmark::State& state) {
    const BicycleDynamics dynamics(kWheelbase);
    iLQRAlignedVec<iLQRState> states;
    iLQRAlignedVec<iLQRControl> controls;
    MakeNominalTrajectory(&states, &controls);
    iLQRState x = states.front();
    for (auto _ : state) {
        for (std::size_t k = 0; k < kHorizonSteps; ++k) {
            x = dynamics.step(x, controls[k], kDt);
            benchmark::DoNotOptimize(x);
        }
    }
    state.SetItemsProcessed(static_cast<std::int64_t>(state.iterations()) *
                            static_cast<std::int64_t>(kHorizonSteps));
}
BENCHMARK(BM_RolloutFullTrajectory);

}  // namespace
}  // namespace apa_post_processor
