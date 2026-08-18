#pragma once

#include <Eigen/Core>
#include <vector>

#include "../../util/path.h"
#include "minco_config.h"

namespace apa_post_processor {
// 单个微观多项式段的初始估计（各字段均为该段终点处的期望状态）
struct MincoSegmentEstimate {
    // 期望空间终点 p_w0（世界系 x,y），来自 A* 锚点，供预处理逐段终点跟踪惩罚
    Eigen::Vector2d desired_position{0.0, 0.0};
    // 段终点初始朝向角 θ_m (rad，已沿路径解缠绕保证连续)
    double theta{0.0};
    // 段终点初始累积弧长 s_m (m，带符号：后退机动沿负方向累积，原地转向不变)
    double arc_length{0.0};
    // 段时长初值 T_m (s)，留给预处理优化收紧
    double duration{0.0};
};
// 单个 Maneuver（宏观段）的初始估计集合
struct MincoManeuverEstimate {
    // 运动方向（直接来自输入 Maneuver 的方向推断结果）
    Direction direction{Direction::UNKNOWN};
    // 起点朝向角 (rad，已解缠绕)
    double start_theta{0.0};
    // 起点累积弧长 (m，带符号；全路径首段从 0 开始)
    double start_arc_length{0.0};
    // 微观段序列（至少 1 段）
    std::vector<MincoSegmentEstimate> segments;
};
// 前端 Hybrid A* 路径解析器：把粗糙 Path 解析为 MINCO 优化所需的初始 M 段
// 估计。换挡打断（宏观段）直接消费 Path 已完成的基于方向的 Maneuver 切分
// （换挡点即 Maneuver 交界，施加 ṡ=0 硬边界是下游装配的职责）；空间等距
// 降采样（微观段）在每个 Maneuver 内部按 M=ceil(L/d_seg) 分段，以步长
// K_step=floor((N-1)/M) 抽取 A* 锚点（末锚点强制为路径终点），产出每段
// 期望终点 p_w0、初始累积弧长 s_m、初始朝向 θ_m（沿路径解缠绕）与时长
// 初值 T_m。纯几何解析，不包含任何优化过程。
class MincoManeuverSegmenter {
   public:
    // 构造并校验配置：四个数值字段均必须为正有限值，非法抛
    // std::invalid_argument
    explicit MincoManeuverSegmenter(const MincoConfig& config);
    // 把输入 Path 解析为每个 Maneuver 的初始段估计；Path 为空抛
    // std::invalid_argument
    std::vector<MincoManeuverEstimate> segment(const Path& path) const;
    // 当前配置（只读）
    const MincoConfig& config() const { return config_; }

   protected:
    // 解析单个 Maneuver：cumulative_arc 为跨 Maneuver 连续累积的带符号弧长，
    // prev_theta 为跨 Maneuver 连续的解缠绕朝向（均为输入输出参数）
    MincoManeuverEstimate segmentManeuver(const Maneuver& maneuver,
                                        double* cumulative_arc,
                                        double* prev_theta) const;
    // 微段融合：移除内部满足融合判据（方向 FORWARD/BACKWARD、|Δs| 与 |Δθ|
    // 均低于阈值）的机动段估计，弧长平移重锚保持累积弧长连续，同向邻段
    // 合并；首末段绝对保护，PIVOT/UNKNOWN 段永不融合
    std::vector<MincoManeuverEstimate> FuseShortManeuvers(
        std::vector<MincoManeuverEstimate> estimates) const;
    // 方向符号：FORWARD=+1、BACKWARD=-1、PIVOT=0；UNKNOWN 按 +1 处理（仅
    // 影响弧长初值符号，方向未识别时路径通常很短）
    static double DirectionSign(Direction direction);
    // 把 theta 调整到 reference 的 ±π 邻域内（2π 周期解缠绕）
    static double UnwrapAngle(double theta, double reference);

   protected:
    MincoConfig config_;
};
}  // namespace apa_post_processor
