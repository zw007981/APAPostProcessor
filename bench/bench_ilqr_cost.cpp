#include <benchmark/benchmark.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "core/iLQR/bicycle_dynamics.h"
#include "core/iLQR/ilqr_cost.h"
#include "core/iLQR/esdf_constraint.h"
#include "spatial/esdf_map.h"
#include "spatial/grid_map.h"
#include "vehicle/vehicle_footprint_model.h"
#include "vehicle/vehicle_params.h"

namespace apa_post_processor {
namespace {

// 计时场景：N=399 步全轨迹，与标称时域一致；轴距/步长取标称值
constexpr std::size_t kHorizonSteps = 399;
constexpr double kWheelbase = 3.0;
constexpr double kDt = 0.1;
// ESDF 地图参数：一面直墙 + 足够大的覆盖范围（32m×16m），
// 保证全轨迹圆心落在图内、惩罚真实激活（走完整插值+惩罚路径）
constexpr double kResolution = 0.125;
constexpr int kMapCols = 256;
constexpr int kMapRows = 128;

// ESDF 场景包：地图/距离场/外圆模型一次性构造（不计入计时）
struct EsdfBundle {
    GridMap grid_map;
    ESDFMap esdf_map;
    VehicleFootprintModel footprint_model;
};

// 构造一条有代表性的名义轨迹与配套参考位姿：速度/转角均时变
// （混合转向与一次换挡过零），保证计时覆盖跟踪/幅值/ESDF 全部路径
void MakeScenario(iLQRReference* reference, iLQRAlignedVec<iLQRState>* states,
                  iLQRAlignedVec<iLQRControl>* controls) {
    const BicycleDynamics dynamics(kWheelbase);
    states->clear();
    controls->clear();
    states->reserve(kHorizonSteps + 1);
    controls->reserve(kHorizonSteps);
    reference->poses.clear();
    reference->poses.reserve(kHorizonSteps + 1);
    reference->dt = kDt;
    reference->ds = 0.05;
    iLQRState x = iLQRState::Zero();
    x(ILQR_IDX_V) = 0.02;
    states->push_back(x);
    reference->poses.emplace_back(x(ILQR_IDX_X), x(ILQR_IDX_Y), x(ILQR_IDX_THETA));
    for (std::size_t k = 0; k < kHorizonSteps; ++k) {
        iLQRControl u;
        // 前 200 步前进、后 199 步倒车，j/η 均按平滑伪机动剖面变化
        const double sign = k < 200 ? 1.0 : -1.0;
        u(ILQR_IDX_JERK) = 0.15 * std::sin(0.05 * static_cast<double>(k)) * sign;
        u(ILQR_IDX_ETA) = 0.25 * std::cos(0.04 * static_cast<double>(k));
        controls->push_back(u);
        x = dynamics.step(x, u, kDt);
        states->push_back(x);
        reference->poses.emplace_back(x(ILQR_IDX_X), x(ILQR_IDX_Y),
                                      x(ILQR_IDX_THETA));
    }
}

// 构造墙场 ESDF 与车辆外圆模型：第 0 列整列占据，轨迹沿墙行驶一段，
// 部分阶段的圆心落入舒适惩罚区
EsdfBundle MakeEsdfBundle() {
    std::vector<Position> cells;
    cells.reserve(kMapRows);
    for (int row = 0; row < kMapRows; ++row) {
        cells.emplace_back(Position{0.0, row * kResolution});
    }
    const GridMap grid_map(kResolution, kMapCols, kMapRows,
                           Position{-4.0, -8.0}, cells);
    const ESDFMap esdf_map(grid_map);
    const VehicleFootprintModel footprint_model(
        VehicleParams(4.3, 1.8, 2.7, 0.6, 0.8), 233, 2, 4);
    return EsdfBundle{grid_map, esdf_map, footprint_model};
}

// 乘子状态：幅值罚权重取 1（轨迹大部在界内，门控路径也覆盖），
// 终点罚权重取 100，乘子取零（首轮外层迭代形态）
iLQRCostMultiplierState MakeMultipliers() {
    auto multipliers = iLQRCostMultiplierState::MakeZero(kHorizonSteps);
    multipliers.amplitude_mu.setConstant(1.0);
    multipliers.terminal_mu.setConstant(100.0);
    return multipliers;
}

// N=399 全轨迹一轮求值 + GN 导数装配（含 ESDF 双 margin 惩罚，stride=1）：
// 内层求解器「沿线性化」环节的代价层耗时基线
void BM_iLQRCostFullEvaluationWithEsdf(benchmark::State& state) {
    iLQRReference reference;
    iLQRAlignedVec<iLQRState> states;
    iLQRAlignedVec<iLQRControl> controls;
    MakeScenario(&reference, &states, &controls);
    const auto bundle = MakeEsdfBundle();
    const iLQREsdfConstraint esdf_constraint(bundle.esdf_map,
                                            bundle.footprint_model);
    const iLQRCostEvaluator evaluator(iLQRCostConfig{}, &esdf_constraint);
    const auto multipliers = MakeMultipliers();
    iLQRCostInput input;
    input.tracking_weight = 10.0;
    for (auto _ : state) {
        auto result =
            evaluator.evaluate(reference, states, controls, multipliers, input);
        benchmark::DoNotOptimize(result.total_cost);
    }
    state.SetItemsProcessed(static_cast<std::int64_t>(state.iterations()) *
                            static_cast<std::int64_t>(kHorizonSteps + 1));
}
BENCHMARK(BM_iLQRCostFullEvaluationWithEsdf);

// 同场景不含 ESDF 的对照组：分离纯二次项装配成本与 ESDF 查询成本
// （设计文档预期运行时被 ESDF 查询主导）
void BM_iLQRCostFullEvaluationWithoutEsdf(benchmark::State& state) {
    iLQRReference reference;
    iLQRAlignedVec<iLQRState> states;
    iLQRAlignedVec<iLQRControl> controls;
    MakeScenario(&reference, &states, &controls);
    const iLQRCostEvaluator evaluator(iLQRCostConfig{}, nullptr);
    const auto multipliers = MakeMultipliers();
    iLQRCostInput input;
    input.tracking_weight = 10.0;
    for (auto _ : state) {
        auto result =
            evaluator.evaluate(reference, states, controls, multipliers, input);
        benchmark::DoNotOptimize(result.total_cost);
    }
    state.SetItemsProcessed(static_cast<std::int64_t>(state.iterations()) *
                            static_cast<std::int64_t>(kHorizonSteps + 1));
}
BENCHMARK(BM_iLQRCostFullEvaluationWithoutEsdf);

}  // namespace
}  // namespace apa_post_processor
