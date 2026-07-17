#pragma once

#include <ocp/multi_stage_ocp.h>
#include <sqp/sqp_algorithm.h>
#include <util/trajectory.h>

#include <map>
#include <optional>
#include <tuple>

#include "../../spatial/esdf_map.h"
#include "../../util/config.h"
#include "../../vehicle/vehicle_footprint_model.h"
#include "../../vehicle/vehicle_params.h"
#include "nmpc_config.h"
#include "path_to_ocp_converter.h"

namespace apa_post_processor {
// NMPC求解器：基于StcSQP的SQP引擎，对初始路径做平滑与换挡段优化。
class NmpcSolver {
   public:
    // 求解结果
    struct Result {
        // SQP是否收敛；不收敛时trajectory仍写入最新迭代轨迹。
        bool converged = false;
        // 优化后的轨迹（状态x=[x,y,theta,v,delta]，控制u=[a,delta_dot]）
        stc_SQP::Trajectory trajectory;
        // 每个机动段的步数N，与Path的Maneuver一一对应
        std::vector<int> segment_steps;
        // 每个机动段的方向符号：+1前进，-1后退
        std::vector<double> segment_v_signs;
        // 本次optimize()调用的总耗时(ms)
        double solve_time_ms = 0.0;
    };

    // 使用车辆参数、footprint模型与求解器配置构造
    NmpcSolver(const VehicleParams& vehicle_params,
               const VehicleFootprintModel& footprint_model,
               NMPCConfig config = NMPCConfig{});
    // 对初始路径在给定ESDF地图下做NMPC优化
    Result optimize(const Path& initial_path, const ESDFMap& esdf_map) const;
    // 对已装配好的OCP与初始猜测做NMPC优化（预处理管线入口）
    Result optimize(const stc_SQP::MultiStageOCP& ocp,
                    const stc_SQP::Trajectory& init_guess,
                    const ESDFMap& esdf_map) const;
    // 把优化结果还原为apa_post_processor::Path（含Maneuver方向与PathPoint序列）。
    static Path ToPath(const Result& result);

   protected:
    // 共享的求解执行体：约束注入、HPIPM/SQP构造与求解。
    Result solveOcp(const stc_SQP::MultiStageOCP& ocp,
                    const stc_SQP::Trajectory& init_guess,
                    const ESDFMap& esdf_map) const;

   protected:
    VehicleParams vehicle_params_;
    // 车身坐标系下的外圆局部圆心坐标
    std::vector<Eigen::Vector2d> circle_local_positions_;
    double circle_radius_;
    NMPCConfig config_;
    // Path→OCP转换器（仅用于Path入口）
    PathToOcpConverter converter_;
    // QPData 对象池缓存：以 (N, nx, nu, ng_max) 为 key，跨实例共享。
    static std::map<std::tuple<int, int, int, int>,
                    std::unique_ptr<stc_SQP::QPData>>
        qp_data_cache_;
};
}  // namespace apa_post_processor
