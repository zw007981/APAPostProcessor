#pragma once

#include "util/constants.h"
#include "map_interface.h"
#include "ocp/multi_stage_ocp.h"
#include "util/trajectory.h"

namespace stc_SQP {
// ProblemUpdater 配置：所有距离/幅值单位均为米，与 corridor.c 中 p 的物理语义一致。
struct UpdaterConfig {
    // 基于 GJK 的车辆轮廓距离进行障碍物筛选的半径（非后轴中心距离）
    double selection_radius = 0.0;
    // 单步最大位移（Proximity Bounds 幅度 Δp_max）
    double max_step_displacement = 0.0;
    // 额外安全裕度
    double safety_margin = 0.0;
    // 每步最大半空间数（静态维度，必须满足 1 <= top_k <= kMaxHalfSpaces）
    int top_k = 10;
};

// 泊车问题更新器：业务层核心，负责将地图/障碍物信息转化为固定 150 维参数向量 p，
// 并按预测步注入到 MultiStageOCP 的 stage_params 中。
// SQP 引擎本身不解释 p 的语义，仅通过 stage_params 将 p 传递给 CasADi 生成函数。
class ProblemUpdater {
public:
    // 通用参数 p 的固定维度，与 STAGE_PARAM_DIM / autogen/common.py::P_DIM 保持一致。
    static constexpr int kParameterDim = STAGE_PARAM_DIM;
    // 半空间参数在 p 中的起始索引，与 generate_corridor.py 中的 p[15:] 保持一致。
    static constexpr int kHalfSpaceStart = 15;
    // 支持的半空间最大数量，与 generate_corridor.py 中的 N_HS = 10 保持一致。
    static constexpr int kMaxHalfSpaces = 10;
    // 半空间法向量维度（2D，与 map_interface.h 共享语义）。
    static constexpr int kNormalDim = kHalfSpaceNormalDim;
    // 半空间截距在 p 中的起始索引（固定 ABI：15 + 2 * 10 = 35）。
    static constexpr int kInterceptStart = kHalfSpaceStart + kNormalDim * kMaxHalfSpaces;

    explicit ProblemUpdater(const UpdaterConfig& config);
    // 根据当前轨迹与地图查询结果，更新 OCP 每步的 stage_params.p。
    // 调用前会执行完整性校验，若不满足则抛出 std::invalid_argument。
    // 使用契约：调用前 stage_params 可以为空；也可以预分配为 N 个空 p（会被视为未设置）。
    //          若 stage_params 已包含非空参数，则每步 p 必须为 kParameterDim 维且全部有限。
    void updateOcp(const Trajectory& current_traj, const MapInterface& map,
        MultiStageOCP& ocp);

protected:
    // 数学完备性断言：筛选半径必须严格大于单步最大位移 + 安全裕度。
    // 以异常方式报告非法配置，便于上层捕获与单元测试。
    void assertCompleteness() const;
    // 校验并提取 HalfSpace，要求 normal 为 2 维且所有数值 finite。
    void validateHalfSpace(const HalfSpace& hs) const;
    // 将 Top-K 半空间写入 p 的凸走廊区间 p[15:45]，p 其余槽位保持不变。
    // 要求 p.size() == kParameterDim；调用前 p 的非凸走廊区间应已被赋值为期望保留的参数。
    void buildParameterVector(const std::vector<HalfSpace>& half_spaces, Vector& p) const;

protected:
    UpdaterConfig config_;
};
} // namespace stc_SQP
