#pragma once

#include <ocp/multi_stage_ocp.h>
#include <util/trajectory.h>

#include <Eigen/Core>

#include "../../preprocessing/preprocessing_pipeline.h"
#include "../../util/path.h"
#include "../../vehicle/vehicle_params.h"
#include "path_to_ocp_converter.h"

namespace apa_post_processor {
// 将预处理管线输出转换为 MultiStageOCP 与初始猜测。
class PreprocessingToOcpConverter {
   public:
    // 转换结果：OCP 结构、初始猜测、已截断的静态走廊系数
    struct Result {
        stc_SQP::MultiStageOCP ocp;
        stc_SQP::Trajectory init_guess;
        // 截断后的静态走廊系数矩阵
        Eigen::MatrixXd static_corridor_C;
        Eigen::VectorXd static_corridor_d;
    };
    // 使用车辆参数与 OCP 装配配置构造转换器。
    explicit PreprocessingToOcpConverter(
        const VehicleParams& vehicle_params,
        const PathToOcpConfig& config = PathToOcpConfig{});
    // 将预处理管线输出转换为 OCP 与初始猜测。
    Result convert(const Path& original_path,
                   const PreprocessingPipelineResult& pipe_result) const;

   protected:
    // 单段 OCP 在 z_ref 中的边界描述。
    struct SegmentBoundary {
        // z_ref 起始点索引（含）
        int start_idx = 0;
        // z_ref 结束点索引（含）
        int end_idx = 0;
        // 该段速度方向符号：+1.0 前进，-1.0 后退
        double v_sign = 1.0;
    };
    // 按速度符号推断分段边界，全零速度时返回单一段。
    static std::vector<SegmentBoundary> InferSegmentBoundaries(
        const Path& original_path, const std::vector<TrajectoryPoint>& z_ref);
    // 为单个边界构建 StageSegment，is_terminal 表示是否为最后一段。
    stc_SQP::StageSegment buildSegment(
        const PreprocessingPipelineResult& pipe_result,
        const SegmentBoundary& boundary, bool is_terminal) const;
    // 按 OCP 总步数截断静态走廊系数。
    static void TruncateCorridor(const PreprocessingPipelineResult& pipe_result,
                                 int total_steps, Result& result);
    // 推断 OCP 段的速度方向符号。
    static double InferVSign(const Path& original_path,
                             const std::vector<TrajectoryPoint>& z_ref);

   protected:
    // 车辆参数，提供 wheelbase/max_steer_angle 等约束边界
    VehicleParams vehicle_params_;
    // OCP 装配配置（box bound、代价权重）
    PathToOcpConfig config_;
};
}  // namespace apa_post_processor
