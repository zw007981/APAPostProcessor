#pragma once

#include <optional>

#include <ocp/multi_stage_ocp.h>
#include <sqp/sqp_algorithm.h>
#include <util/trajectory.h>

#include "path_to_ocp_converter.h"
#include "../../spatial/esdf_map.h"
#include "../../vehicle/vehicle_footprint_model.h"
#include "../../vehicle/vehicle_params.h"

namespace apa_post_processor {
// NMPC求解器配置：打靶/代价配置 + ESDF安全裕度 + partial condensing/SQP收敛参数。
// 单独定义为命名空间级结构体（而非NmpcSolver内部嵌套类型），是因为嵌套类型带默认成员
// 初始值时直接作为外层类构造函数的默认参数（如`Config config = Config{}`）会触发
// GCC“默认成员初始值需在外层类结束前完成”的编译错误，与PathToOcpConfig保持同样的组织方式。
struct NmpcSolverConfig {
    // Path→OCP转换配置（重采样步长、代价权重等）
    PathToOcpConfig path_to_ocp_config{};
    // 圆形ESDF碰撞的安全裕度(m)
    double esdf_safety_margin = 0.2;
    // ESDF碰撞惩罚代价权重：违反安全裕度时按平方铰链惩罚（hinge penalty）施加的软代价权重。
    // 之所以用软代价而非硬约束（CircleFootprintEsdfConstraint），是因为真实数据里初始
    // 猜测可能已经贴着障碍物走（甚至瞬时违反安全裕度），硬约束会导致QP在第0次迭代就
    // 因不可行直接求解失败（已在真实数据集上复现）；软代价保证box/动力学约束本身构成的
    // 可行域始终非空，QP永不会因碰撞代价不可行而失败，只是违反部分要支付高额代价。
    // 权重取值应显著大于跟踪代价（terminal_position_weight/terminal_heading_weight=200），
    // 使碰撞惩罚在数量级上起主导作用；但经真实数据集调参验证，取值过大（如2000/1000）
    // 反而会在部分复杂换挡场景（如data7.json）诱发SQP在碰撞惩罚的平方铰链拐点附近来回
    // 震荡而发散（实测轨迹长度不降反升），500为兼顾"显著大于其它代价"与"不引发震荡发散"
    // 的经验取值。
    double esdf_penalty_weight = 500.0;
    // Partial Condensing块大小
    int hpipm_block_size = 10;
    // 低于该总步数阈值时不启用partial condensing/OpenMP并行linearize
    int short_n_threshold = 50;
    // SQP最大迭代次数；真实场景（几十个机动段/数百步）需要更多轮次才能收敛，
    // 调参阶段从50上调至100
    int max_iter = 300;
    // 是否启用Armijo线搜索；长N换挡场景经验证关闭更稳定
    bool use_line_search = false;
    // HPIPM求解精度
    double hpipm_tol = 1e-4;
};

// 机动段裁剪配置：M5后处理——检测求解结果中弧长过短（大概率是Hybrid A*离散化产生的
// 冗余换挡）的机动段并裁剪合并，驱动机动段数削减。单独定义为命名空间级结构体，
// 理由与NmpcSolverConfig一致（避免嵌套类型默认成员初始值触发的GCC编译错误）。
struct PruningConfig {
    // 机动段弧长低于该值(m)视为候选：只有弧长低于此值的机动段才会被尝试裁剪，
    // 是否真正接受裁剪由下面的距离-段数等效权衡与终点偏差共同决定
    double min_segment_arc_length = 2.0;
    // 最多裁剪轮数，避免因阈值设置不当导致死循环
    int max_prune_iterations = 10;
    // 裁剪后终点与原始Path终点的最大允许偏差(m)：部分裁剪会让剩余机动段无法在运动学上
    // 到达原目标（如强行合并为单一方向后转弯半径不够），此时求解器虽仍会产出一条
    // “未收敛但非空”的轨迹，但终点可能严重偏离目标。超出该偏差则判定本轮裁剪导致质量
    // 明显下降，回退到裁剪前的结果并停止，而不是被动接受任何非空轨迹。
    // 泊车场景对终点精度要求高，默认给一个较严格的值。
    double max_terminal_deviation = 0.02;
    // 每减少一个机动段所“等效”允许增加的总路径长度(m)：裁剪掉N个机动段后，只有当总弧长
    // 的增量不超过 N * maneuver_length_equivalent 时才接受本轮裁剪，否则认为“为了少换挡
    // 而绕的路”得不偿失，回退并停止。默认按个人偏好取2.0（每个机动段相当于2米路程）。
    double maneuver_length_equivalent = 2.0;
};

// NMPC求解器：基于third_party/StcSQP的通用SQP引擎，对泊车模块给出的初始路径做平滑与换挡段优化。
// 内部流程：PathToOcpConverter把Path转成MultiStageOCP+初始猜测 -> 给每段的代价包装为
// CompositeCost（原有跟踪代价 + 圆形分解ESDF碰撞惩罚代价，见NmpcSolverConfig::esdf_penalty_weight
// 注释——碰撞用软代价而非硬约束，避免真实数据初始猜测已贴障碍物时QP直接不可行）->
// 按总步数N选择是否启用partial condensing/OpenMP并行linearize（复用
// strategies/strategy_common.hpp的判定逻辑，但不经过AutoAdaptiveStrategy类，因为该类未暴露
// SQPSolverOptions/HPIPM容差的自定义入口，无法配置长N场景已验证的收敛配方，见
// test_long_horizon_real_scenario.cpp）-> 构造HPIPMQPSolver+SQPSolver求解（不再使用一般
// 约束，ng恒为0）。
class NmpcSolver {
   public:
    // 求解结果
    struct Result {
        // SQP是否收敛（RTI模式下表示单步QP成功）；不收敛时trajectory仍写入最新迭代轨迹（若第0次
        // 迭代即失败则trajectory保持默认构造为空，调用方需自行判断trajectory.x.empty()）
        bool converged = false;
        // 优化后的轨迹（状态x=[x,y,theta,v,delta]，控制u=[a,delta_dot]）
        stc_SQP::Trajectory trajectory;
        // 每个机动段的步数N，与original Path的Maneuver一一对应，用于把trajectory按段切回机动段
        std::vector<int> segment_steps;
        // 每个机动段的方向符号：+1表示前进，-1表示后退，与segment_steps一一对应
        std::vector<double> segment_v_signs;
        // 本次optimize()调用的总耗时(ms)，对应proto OptimizeResponse.optimization_time_ms
        double solve_time_ms = 0.0;
        // 经历了多少轮机动段裁剪合并（仅optimizeWithPruning()会设置为>0，直接调用optimize()恒为0）
        int prune_iterations = 0;
    };

    // 使用车辆参数、车辆圆形分解模型（提供碰撞约束用的外圆几何）与求解器配置构造
    NmpcSolver(const VehicleParams& vehicle_params,
              const VehicleFootprintModel& footprint_model,
              NmpcSolverConfig config = NmpcSolverConfig{});
    // 对初始路径在给定ESDF地图下做NMPC优化，返回优化后的轨迹
    Result optimize(const Path& initial_path, const ESDFMap& esdf_map) const;
    // 在optimize()基础上叠加机动段裁剪后处理：先求解一次，再检测弧长低于阈值的
    // 机动段并裁剪合并（同号相邻段拼接成一段），用裁剪后的Path重新求解；若裁剪后求解
    // 失败（未产出可用轨迹）、裁剪后终点偏离原始Path终点超过max_terminal_deviation
    // （说明剩余机动段结构已无法运动学可行地到达目标），或总弧长增量超过
    // maneuvers_removed * maneuver_length_equivalent（说明为了少换挡而绕的路得不偿失），
    // 则回退到裁剪前的结果并停止；重复直至无可裁剪段或达到max_prune_iterations。
    // 返回的Result.segment_steps/segment_v_signs对应裁剪后的最终机动段结构，可直接传给ToPath()。
    Result optimizeWithPruning(const Path& initial_path, const ESDFMap& esdf_map,
                              const PruningConfig& pruning_config = PruningConfig{}) const;
    // 把优化结果按Result::segment_steps/segment_v_signs重新切回机动段，还原为apa_post_processor::Path
    // （含Maneuver方向与PathPoint序列）。PathPoint中除x/y/theta外，还会回填NMPC优化产出的
    // v/delta状态量与a/delta_dot控制量（每段最后一个点无对应控制量，故hasA()/hasDeltaDot()
    // 为false，属于预期行为）；kappa保持未设置，因为这些点未经过Path的曲率估计。
    // Path::toProto()序列化时只读取x/y/theta，其余派生量不进入proto。
    static Path ToPath(const Result& result);

   protected:
    // 在result中寻找弧长最短且低于min_arc_length的机动段并裁剪：若该段为首/末段则直接丢弃，
    // 相邻段顺延成为新的首/末段；否则合并该段两侧的相邻段（Maneuver方向严格交替，
    // 两侧相邻段必然同号，直接拼接点序列即可）。若没有低于阈值的段，或result没有可用轨迹，
    // 或只剩1段无法再裁剪，返回std::nullopt。
    static std::optional<Path> pruneShortestSegment(const Result& result,
                                                    double min_arc_length);
    // 车辆参数
    VehicleParams vehicle_params_;
    // 车身坐标系下的外圆局部圆心坐标，用于构造圆形ESDF碰撞约束
    std::vector<Eigen::Vector2d> circle_local_positions_;
    // 外圆半径
    double circle_radius_;
    // 求解器配置
    NmpcSolverConfig config_;
    // Path→OCP转换器
    PathToOcpConverter converter_;
};
}  // namespace apa_post_processor


