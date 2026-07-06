#pragma once

#include <vector>

#include <Eigen/Core>

#include "map_interface.h"
#include "ocp/multi_stage_ocp.h"
#include "util/trajectory.h"

namespace stc_SQP {
// 圆形分解ESDF参数更新器：与EsdfProblemUpdater（矩形四角点版本）并列，
// 职责：对MultiStageOCP每一步、每个构造时给定的车身局部圆心，在当前轨迹给出的位姿处
// 采样ESDF距离场，并通过CircleFootprintEsdfConstraint::packCircleSample写入
// stage_params[i].p的圆参数区间。SQP引擎与CircleFootprintEsdfConstraint本身都不感知地图语义。
// 使用契约与EsdfProblemUpdater一致：调用前stage_params可以为空，也可以是N个空p；
// 若已包含非空参数，则每步p必须为STAGE_PARAM_DIM维且全部有限，本更新器只覆盖
// 其中的圆参数区间，其余槽位保持不变。
class CircleFootprintEsdfProblemUpdater {
public:
    // circle_local_positions：车身坐标系下各圆心的局部坐标，数量必须与
    // CircleFootprintEsdfConstraint构造时传入的一致，否则采样与约束的圆心对应关系会错位。
    explicit CircleFootprintEsdfProblemUpdater(std::vector<Eigen::Vector2d> circle_local_positions);
    // 对ocp的每一段、每一步，按current_traj给出的位姿重新采样并写入圆参数
    void updateOcp(const Trajectory& current_traj, const EsdfMapInterface& map,
        MultiStageOCP& ocp);

protected:
    // 车身坐标系下各圆心的局部坐标
    std::vector<Eigen::Vector2d> circle_local_positions_;
};
} // namespace stc_SQP
