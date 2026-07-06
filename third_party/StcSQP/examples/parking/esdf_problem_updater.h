#pragma once

#include "map_interface.h"
#include "ocp/multi_stage_ocp.h"
#include "util/trajectory.h"

namespace stc_SQP {
// ESDF 参数更新器：与 ProblemUpdater（凸走廊/SFC）并列的另一条业务层链路。
// 职责：对 MultiStageOCP 每一步、每个车辆角点，在当前轨迹给出的位姿处采样 ESDF
// 距离场，并通过 EsdfDistanceConstraint::packCornerSample 写入 stage_params[i].p
// 的 p[45:65] 区间。SQP 引擎与 EsdfDistanceConstraint 本身都不感知地图语义。
// 使用契约与 ProblemUpdater 一致：调用前 stage_params 可以为空，也可以是 N 个空 p；
// 若已包含非空参数，则每步 p 必须为 STAGE_PARAM_DIM 维且全部有限，本更新器只覆盖
// 其中的 ESDF 区间，其余槽位（含 p[15:45] 凸走廊区间）保持不变。
class EsdfProblemUpdater {
public:
    // 对 ocp 的每一段、每一步，按 current_traj 给出的位姿重新采样并写入 ESDF 参数。
    void updateOcp(const Trajectory& current_traj, const EsdfMapInterface& map,
        MultiStageOCP& ocp);
};
} // namespace stc_SQP
