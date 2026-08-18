#include "ilqr_reference_builder.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include "../../util/logger.h"
#include "../NMPC/vehicle_circle_geometry.h"

namespace apa_post_processor {
namespace {
// 把 Path 展平为连续 Pose 序列（forEach 已剔除 maneuver 间共享的重复边界点）
std::vector<Pose> FlattenPath(const Path& path) {
    std::vector<Pose> points;
    points.reserve(path.size());
    path.forEach([&points](const TrajectoryPoint& point) {
        points.emplace_back(point.x, point.y, point.theta);
    });
    return points;
}

// 由方向枚举映射运动符号：前进 +1 / 后退 -1 / 原地转向或未知 0
int DirectionSign(Direction direction) {
    if (direction == Direction::FORWARD) {
        return 1;
    }
    if (direction == Direction::BACKWARD) {
        return -1;
    }
    return 0;
}

// 裁剪到对称盒 [-bound, bound]
double ClampSymmetric(double value, double bound) {
    return std::clamp(value, -bound, bound);
}

// 由位姿序列重建 Path：逐点追加由 addPoint 完成去重/方向推断，
// finalize 统一完成曲率估计（与数据加载同一条构造路径）
Path RebuildPath(const std::vector<Pose>& points) {
    Path result;
    for (const Pose& pose : points) {
        result.addPoint(pose);
    }
    result.finalize();
    return result;
}

// 短接配置的合法性校验：非法值显式抛出（静默降级会让上层误以为
// 短接已生效）
void ValidateRsShortcutConfig(const iLQRRsShortcutConfig& config,
                              double wheelbase, double delta_max) {
    if (!std::isfinite(config.cap_ratio) || config.cap_ratio < 0.0 ||
        config.cap_ratio > 1.0) {
        throw std::invalid_argument(
            "ShortcutShiftPoints: 曲率上限比例必须落在 [0,1]");
    }
    if (!std::isfinite(config.collision_margin) ||
        config.collision_margin < 0.0) {
        throw std::invalid_argument(
            "ShortcutShiftPoints: 碰撞裕度必须为非负有限值");
    }
    if (!std::isfinite(config.max_length_growth) ||
        config.max_length_growth < 0.0) {
        throw std::invalid_argument(
            "ShortcutShiftPoints: 长度增长上限必须为非负有限值");
    }
    if (!(config.sample_dist > 0.0)) {
        throw std::invalid_argument(
            "ShortcutShiftPoints: 采样间距必须为正");
    }
    if (!(wheelbase > 0.0) || !(delta_max > 0.0)) {
        throw std::invalid_argument(
            "ShortcutShiftPoints: 轴距与 δ_max 必须为正");
    }
}
// 构造「采样序列是否安全」的判据闭包：任一覆盖圆越出地图或侵入超过
// 裕度即拒绝——图外是未知区域，恢复场只是数值延拓，据此接受等于
// 凭空捏造可行空间
auto MakeSamplesSafeChecker(const ESDFMap& esdf_map,
                            const std::vector<Eigen::Vector2d>& outer_circles,
                            double outer_radius, double collision_margin) {
    return [&esdf_map, &outer_circles, outer_radius, collision_margin](
               const std::vector<RsSamplePoint>& samples) {
        for (const auto& sample : samples) {
            const double cos_theta = std::cos(sample.pose.theta);
            const double sin_theta = std::sin(sample.pose.theta);
            for (const auto& local : outer_circles) {
                const double wx = sample.pose.x + local.x() * cos_theta -
                                  local.y() * sin_theta;
                const double wy = sample.pose.y + local.x() * sin_theta +
                                  local.y() * cos_theta;
                if (!esdf_map.inMap(wx, wy)) {
                    return false;
                }
                if (outer_radius - esdf_map.getDist(wx, wy) >
                    collision_margin) {
                    return false;
                }
            }
        }
        return true;
    };
}
// RS 求解逐次耗时记录器：累计每一次 RS 计算的耗时（调用数/总耗时/
// 单次最大耗时），析构时经日志输出汇总行；rs_timing_csv 非空时把
// 逐次耗时追加写入 CSV（供实验取证，一次调用一段，# 开头行为段落
// 头）。路径为空时纯汇总模式——不产生文件、不影响任何数值
class RsTimingSink {
   public:
    RsTimingSink(std::string csv_path, std::string tag,
                 const char* method_name)
        : method_name_(method_name), tag_(std::move(tag)) {
        if (csv_path.empty()) {
            return;
        }
        stream_.open(csv_path, std::ios::app);
        active_ = stream_.is_open();
        if (active_) {
            stream_ << "# invoke seq=" << NextInvocationSeq()
                    << " method=" << method_name_ << " tag=" << tag_ << "\n";
        }
    }
    ~RsTimingSink() {
        if (active_) {
            stream_ << "# summary calls=" << call_count_
                    << " total_us=" << total_us_ << " max_us=" << max_us_
                    << "\n";
            stream_.close();
        }
        LOG_FMT_INFO(
            "RS timing [{} tag={}]: calls={} total_us={:.1f} "
            "avg_us={:.2f} max_us={:.1f}",
            method_name_, tag_, call_count_, total_us_,
            call_count_ > 0 ? total_us_ / call_count_ : 0.0, max_us_);
    }
    // 记录一次 RS 求解的耗时（微秒）与是否有效
    void record(double time_us, bool valid) {
        if (active_) {
            stream_ << call_index_ << "," << time_us << ","
                    << (valid ? 1 : 0) << "\n";
            ++call_index_;
        }
        ++call_count_;
        total_us_ += time_us;
        max_us_ = std::max(max_us_, time_us);
    }

   protected:
    // 进程内调用序号：同一次运行中不同输入/数据集的段落以此区分
    static int NextInvocationSeq() {
        static int seq = 0;
        ++seq;
        return seq;
    }
    // 输出流（仅 rs_timing_csv 非空且可打开时有效）
    std::ofstream stream_;
    // 是否激活（文件打开成功才激活，失败静默降级为纯汇总日志）
    bool active_{false};
    // 编排方法名（rs）
    const char* method_name_;
    // 分组标签（如数据集名），供日志与 CSV 段落头引用
    std::string tag_;
    // 已记录调用数（CSV 行号与汇总口径共用）
    int call_index_{0};
    // 汇总调用数
    int call_count_{0};
    // 累计耗时 (us)
    double total_us_{0.0};
    // 单次最大耗时 (us)
    double max_us_{0.0};
};
// 把 RS 采样序列按行驶方向旗标切分为 maneuver 序列：相邻 maneuver
// 共享边界点（与本仓库 Path 的 maneuver 边界约定一致），方向由旗标
// 显式给定，不依赖下游再推断
std::vector<Maneuver> SamplesToManeuvers(
    const std::vector<RsSamplePoint>& samples) {
    std::vector<Maneuver> maneuvers;
    if (samples.size() < 2) {
        return maneuvers;
    }
    const auto push_group = [&maneuvers](const std::vector<RsSamplePoint>& src,
                                         std::size_t begin,
                                         std::size_t end) {
        std::vector<TrajectoryPoint> points;
        points.reserve(end - begin);
        for (std::size_t k = begin; k < end; ++k) {
            points.emplace_back(src[k].pose.x, src[k].pose.y,
                                src[k].pose.theta);
        }
        maneuvers.emplace_back(std::move(points),
                               src[begin].forward ? Direction::FORWARD
                                                  : Direction::BACKWARD);
    };
    std::size_t begin = 0;
    for (std::size_t i = 1; i < samples.size(); ++i) {
        if (samples[i].forward == samples[begin].forward) {
            continue;
        }
        push_group(samples, begin, i + 1);
        begin = i;
    }
    push_group(samples, begin, samples.size());
    return maneuvers;
}
// 单段 maneuver 的短接代价：弧长 + 短段惩罚（低于阈值时按比例加权），
// 沿用外部混合 A* 参考实现的定价口径
double ManeuverShortcutCost(const Maneuver& maneuver,
                            const iLQRRsShortcutConfig& config) {
    const double length = maneuver.length();
    double cost = length;
    if (length <= config.short_segment_length) {
        cost += 0.5 * config.short_segment_weight *
                (1.0 + (config.short_segment_length - length) /
                           config.short_segment_length);
    }
    return cost;
}
// 整条路径的短接代价：固定段价 × 段数 + 各段代价之和
double ShortcutPathCost(const std::vector<Maneuver>& maneuvers,
                        const iLQRRsShortcutConfig& config) {
    double total = config.segment_fixed_cost *
                   static_cast<double>(maneuvers.size());
    for (const auto& maneuver : maneuvers) {
        total += ManeuverShortcutCost(maneuver, config);
    }
    return total;
}
}  // namespace

iLQRReferenceBuilder::iLQRReferenceBuilder(iLQRReferenceBuilderConfig config,
                                         const VehicleParams& vehicle_params)
    : config_(config), wheelbase_(vehicle_params.wheelbase) {
    if (!(config_.sample_dist > 0.0) || !(config_.dt > 0.0) ||
        config_.shooting_interval < 1 || !(config_.v_max > 0.0) ||
        !(config_.a_max > 0.0) || !(config_.delta_max > 0.0) ||
        !(config_.omega_max > 0.0)) {
        throw std::invalid_argument(
            "iLQRReferenceBuilder: sample_dist/dt/shooting_interval/box bounds "
            "must be positive!!!");
    }
    if (!(wheelbase_ > EPSILON)) {
        throw std::invalid_argument(
            "iLQRReferenceBuilder: vehicle wheelbase must be positive!!!");
    }
}

// RS 换挡点短接：以 maneuver 边界为节点做动态规划全局择优。
// 状态转移 dp[j] = min(dp[i] + cost(i,j))：cost(i,j) 在「沿用原路径
// i..j-1 段」与「RS 曲线直连（出车方向受节点 maneuver 方向约束、逐点
// 碰撞校验）」之间取更优者；代价口径为「固定段价 × 段数 + 各段弧长
// 与短段惩罚之和」。RS 求解次数为 O(M²)（M 为 maneuver 数，每次为
// 常数开销的闭式解）；拼接点只能落在 maneuver 边界
Path ShortcutShiftPoints(const Path& path, const ESDFMap& esdf_map,
                         const VehicleFootprintModel& footprint_model,
                         double wheelbase, double delta_max,
                         const iLQRRsShortcutConfig& config) {
    ValidateRsShortcutConfig(config, wheelbase, delta_max);
    if (config.cap_ratio == 0.0 || path.empty()) {
        return path;
    }
    const double turning_radius =
        wheelbase / (config.cap_ratio * std::tan(delta_max));
    RsTimingSink timing(config.rs_timing_csv, config.rs_timing_tag, "rs");
    const auto& src_maneuvers = path.getManeuvers();
    const int num_maneuvers = static_cast<int>(src_maneuvers.size());
    // 短接节点：第 k 段 maneuver 的起点位姿与方向（k = 0..M-1），
    // 节点 M 为全局终点。RS 直连只允许发生在这些节点之间
    struct ShortcutNode {
        // 节点位姿
        Pose pose{};
        // 节点出车方向符号（+1 前进 / -1 倒退 / 0 未知）
        int sign{0};
    };
    std::vector<ShortcutNode> nodes;
    nodes.reserve(static_cast<std::size_t>(num_maneuvers) + 1);
    for (const auto& maneuver : src_maneuvers) {
        const auto& front_point = maneuver.points.front();
        nodes.push_back(ShortcutNode{Pose{front_point.x, front_point.y,
                                          front_point.theta},
                                     DirectionSign(maneuver.direction)});
    }
    const auto& back_point = src_maneuvers.back().points.back();
    nodes.push_back(
        ShortcutNode{Pose{back_point.x, back_point.y, back_point.theta}, 0});
    const auto outer_circles =
        vehicle_circle_geometry::ExtractLocalCircleCenters(footprint_model,
                                                           CircleType::OUTER);
    const double outer_radius = footprint_model.getOuterRadius();
    const auto samples_safe = MakeSamplesSafeChecker(
        esdf_map, outer_circles, outer_radius, config.collision_margin);
    // 原始总代价是「改进是否值得采纳」的唯一判据，原始总长是长度守卫
    // 的固定基准（改进后不得超预算）
    const double original_cost = ShortcutPathCost(src_maneuvers, config);
    const double length_budget =
        path.length() * (1.0 + config.max_length_growth);
    // dp[i]：从全局起点到节点 i 的最优路径；seg 为空表示该连接段沿用
    // 原路径 maneuvers[parent..current)
    struct ShortcutState {
        // 到达节点 i 的最小总代价
        double cost{std::numeric_limits<double>::infinity()};
        // 父节点索引（-1 表示不可达）
        int parent{-1};
        // 父节点到当前节点的最优连接段（空 = 沿用原路径）
        std::vector<Maneuver> seg;
    };
    std::vector<ShortcutState> dp(
        static_cast<std::size_t>(num_maneuvers) + 1);
    dp[0].cost = 0.0;
    for (int i = 0; i < num_maneuvers; ++i) {
        if (std::isinf(dp[i].cost)) {
            continue;
        }
        const ShortcutNode& start_node = nodes[i];
        double accumulated_original_cost = 0.0;
        for (int j = i + 1; j <= num_maneuvers; ++j) {
            // 对称记账：原始子路径与 RS 直连都计「固定段价 + 各段代价」，
            // 使转移比较成为完整路径代价的公平对照（外部参考实现的
            // 记账只给 RS 段计固定段价、原始段免计，等价于给 RS 加税，
            // 在段数少、路径已近最优的数据上会让 DP 永远找不到改进）
            accumulated_original_cost +=
                config.segment_fixed_cost +
                ManeuverShortcutCost(src_maneuvers[j - 1], config);
            double best_segment_cost = accumulated_original_cost;
            std::vector<Maneuver> best_segment;
            // 出车方向约束：节点方向明确时 RS 首段必须同向（换挡节点
            // 不得凭空变向，与外部参考实现 set_current_dir 语义一致）；
            // 方向未知（0）时不约束
            std::optional<bool> start_forward;
            if (start_node.sign != 0) {
                start_forward = start_node.sign > 0;
            }
            const auto rs_t0 = std::chrono::steady_clock::now();
            const auto rs = ComputeShortestReedsShepp(
                start_node.pose, nodes[j].pose, turning_radius,
                start_forward);
            timing.record(std::chrono::duration<double, std::micro>(
                              std::chrono::steady_clock::now() - rs_t0)
                              .count(),
                          rs.valid);
            if (rs.valid) {
                const auto samples = SampleReedsShepp(
                    rs, start_node.pose, turning_radius, config.sample_dist);
                // 退化守卫：起终点重合的零长解只有单个采样点，构不成
                // maneuver，不可作为零代价连接
                if (samples.size() >= 2 && samples_safe(samples)) {
                    auto segment_maneuvers = SamplesToManeuvers(samples);
                    const double segment_cost =
                        ShortcutPathCost(segment_maneuvers, config);
                    if (segment_cost < best_segment_cost - 0.01) {
                        best_segment_cost = segment_cost;
                        best_segment = std::move(segment_maneuvers);
                    }
                }
            }
            const double new_cost = dp[i].cost + best_segment_cost;
            if (new_cost < dp[j].cost - 0.01) {
                dp[j] = ShortcutState{new_cost, i, std::move(best_segment)};
            }
        }
    }
    // 代价未改善则不换：DP 结果必须严格优于原路径才被采纳（-0.01 是
    // 参考实现的采纳判据，吸收浮点噪声）
    if (std::isinf(dp[num_maneuvers].cost) ||
        dp[num_maneuvers].cost >= original_cost - 0.01) {
        return path;
    }
    // 回溯重构：沿父链从终点向起点收集各连接段。插入统一用头部插入
    // （先处理的终链在前、后处理的首链插到最前），连接段内部的
    // maneuver 顺序保持不变——严禁整表 reverse：它会把多段（含尖点）
    // RS 连接段内部的 maneuver 也反转，导致终点漂移到中间尖点处
    std::vector<Maneuver> improved;
    improved.reserve(static_cast<std::size_t>(num_maneuvers) + 1);
    for (int current = num_maneuvers; current > 0;
         current = dp[current].parent) {
        const int parent = dp[current].parent;
        if (parent < 0) {
            return path;
        }
        const auto& seg = dp[current].seg;
        if (seg.empty()) {
            improved.insert(improved.begin(), src_maneuvers.begin() + parent,
                            src_maneuvers.begin() + current);
        } else {
            improved.insert(improved.begin(), seg.begin(), seg.end());
        }
    }
    // 展平重建：首个 maneuver 全部点、后续跳过与上一段共享的首点
    // （与输入 Path 的 maneuver 边界约定一致），方向由重建过程再推断
    std::vector<Pose> flattened;
    flattened.reserve(path.size() + 1);
    for (std::size_t m = 0; m < improved.size(); ++m) {
        const auto& points = improved[m].points;
        for (std::size_t k = 0; k < points.size(); ++k) {
            if (m > 0 && k == 0) {
                continue;
            }
            flattened.emplace_back(points[k].x, points[k].y, points[k].theta);
        }
    }
    Path result = RebuildPath(flattened);
    if (result.empty() || result.length() > length_budget) {
        return path;
    }
    // 终点位姿保持契约：短接只允许移动换挡点，终点位姿必须保持——
    // 下游把短接路径的终点作为目标位姿（goal 取 shared_input.back()），
    // 终点一旦漂移，整条链路的终点收敛检查都会对着错误的目标收敛。
    // 该守卫是「终点收敛检查有效」的前提，不是终点检查本身
    const double pos_delta =
        std::hypot(result.back().x - path.back().x,
                   result.back().y - path.back().y);
    const double head_delta =
        std::abs(WrapAngle(result.back().theta - path.back().theta));
    if (pos_delta > 1e-6 || head_delta > 1e-6) {
        LOG_FMT_WARN(
            "RS shortcut terminal pose drifted by {:.3f} m / {:.3f} rad, "
            "reject shortcut and keep original path",
            pos_delta, head_delta);
        return path;
    }
    return result;
}

iLQRReference iLQRReferenceBuilder::build(const Path& path) const {
    const std::vector<Pose> points = FlattenPath(path);
    if (points.size() < 2) {
        throw std::invalid_argument(
            "iLQRReferenceBuilder: path contains fewer than 2 points!!!");
    }
    // 累积无符号弧长（θ-only 的零位移段产生零增量，由重采样 ratio 守卫吸收）
    std::vector<double> arc_length(points.size(), 0.0);
    for (std::size_t i = 1; i < points.size(); ++i) {
        arc_length[i] =
            arc_length[i - 1] + std::hypot(points[i].x - points[i - 1].x,
                                           points[i].y - points[i - 1].y);
    }
    const double total_length = arc_length.back();
    if (total_length < config_.sample_dist) {
        throw std::invalid_argument(
            "iLQRReferenceBuilder: path total length is shorter than one "
            "sample spacing!!!");
    }
    // 全长归一：段数按标称间距四舍五入，实际间距 = L / N，保证均匀覆盖终点
    const std::size_t num_steps = std::max<std::size_t>(
        1, static_cast<std::size_t>(
               std::lround(total_length / config_.sample_dist)));
    const double ds = total_length / static_cast<double>(num_steps);
    // 等弧长重采样：双指针扫描，θ 经统一 wrap 后线性插值
    iLQRReference reference;
    reference.ds = ds;
    reference.dt = config_.dt;
    reference.step_dt.assign(num_steps, config_.dt);
    reference.poses.reserve(num_steps + 1);
    std::size_t segment = 0;
    for (std::size_t k = 0; k <= num_steps; ++k) {
        const double target =
            std::min(static_cast<double>(k) * ds, total_length);
        while (segment + 1 < points.size() &&
               arc_length[segment + 1] < target) {
            ++segment;
        }
        if (segment + 1 >= points.size()) {
            reference.poses.push_back(points.back());
            continue;
        }
        const double segment_length =
            arc_length[segment + 1] - arc_length[segment];
        const double ratio =
            segment_length > 0.0
                ? (target - arc_length[segment]) / segment_length
                : 0.0;
        const Pose& from = points[segment];
        const Pose& to = points[segment + 1];
        reference.poses.emplace_back(
            from.x + (to.x - from.x) * ratio, from.y + (to.y - from.y) * ratio,
            WrapAngle(from.theta + WrapAngle(to.theta - from.theta) * ratio));
    }
    // maneuver 元数据：边界弧长经四舍五入映射到网格索引（单调不减），
    const auto& src_maneuvers = path.getManeuvers();
    reference.maneuvers.reserve(src_maneuvers.size());
    std::vector<std::size_t> end_indices;
    end_indices.reserve(src_maneuvers.size());
    double boundary_arc = 0.0;
    for (const auto& maneuver : src_maneuvers) {
        boundary_arc += maneuver.length();
        const long grid_index = std::clamp(std::lround(boundary_arc / ds), 0L,
                                           static_cast<long>(num_steps));
        end_indices.push_back(static_cast<std::size_t>(grid_index));
        const auto& m_points = maneuver.points;
        reference.maneuvers.push_back(iLQRReferenceManeuver{
            DirectionSign(maneuver.direction), maneuver.length(),
            WrapAngle(m_points.back().theta - m_points.front().theta), 0, 0});
    }
    for (std::size_t m = 0; m < reference.maneuvers.size(); ++m) {
        reference.maneuvers[m].begin_index = (m == 0) ? 0 : end_indices[m - 1];
        reference.maneuvers[m].end_index = end_indices[m];
    }
    // cusp 检测：仅登记方向反号（符号均非零且相异）的 maneuver 边界；
    // 涉及原地转向/未知方向（符号 0）的边界不构成换挡尖点
    for (std::size_t m = 1; m < reference.maneuvers.size(); ++m) {
        if (reference.maneuvers[m - 1].sign * reference.maneuvers[m].sign ==
            -1) {
            reference.cusp_indices.push_back(end_indices[m - 1]);
        }
    }
    // 逐网格点 maneuver 归属：cusp 点归后一段，末点固定归最后一段
    // （末点 N 不参与下方 [begin, end) 循环赋值，由构造默认值直接给出）
    std::vector<std::size_t> point_maneuver(num_steps + 1,
                                            src_maneuvers.size() - 1);
    for (std::size_t m = 0; m < reference.maneuvers.size(); ++m) {
        for (std::size_t k = reference.maneuvers[m].begin_index;
             k < reference.maneuvers[m].end_index; ++k) {
            point_maneuver[k] = m;
        }
    }
    // 初值提取：v 由名义车速带符号给出，κ 由参考朝向差分得到并反解 δ，
    // a/ω 由 v/δ 经三点中心差分（端点单侧差分）得到，全部裁剪进盒约束
    const double v_nominal = ds / config_.dt;
    std::vector<double> v_init(num_steps + 1, 0.0);
    std::vector<double> delta_init(num_steps + 1, 0.0);
    for (std::size_t k = 0; k <= num_steps; ++k) {
        v_init[k] = ClampSymmetric(
            static_cast<double>(reference.maneuvers[point_maneuver[k]].sign) *
                v_nominal,
            config_.v_max);
    }
    for (std::size_t k = 0; k < num_steps; ++k) {
        const double kappa =
            WrapAngle(reference.poses[k + 1].theta - reference.poses[k].theta) /
            ds;
        delta_init[k] =
            ClampSymmetric(std::atan(wheelbase_ * kappa), config_.delta_max);
    }
    delta_init[num_steps] = delta_init[num_steps - 1];
    auto central_difference = [this](const std::vector<double>& values,
                                     double bound, std::vector<double>* out) {
        const std::size_t last = values.size() - 1;
        out->reserve(values.size());
        for (std::size_t k = 0; k <= last; ++k) {
            const double diff =
                (k == 0) ? (values[1] - values[0]) / config_.dt
                : (k == last)
                    ? (values[last] - values[last - 1]) / config_.dt
                    : (values[k + 1] - values[k - 1]) / (2.0 * config_.dt);
            out->push_back(ClampSymmetric(diff, bound));
        }
    };
    std::vector<double> a_init;
    std::vector<double> omega_init;
    central_difference(v_init, config_.a_max, &a_init);
    central_difference(delta_init, config_.omega_max, &omega_init);
    reference.initial_states.reserve(num_steps + 1);
    for (std::size_t k = 0; k <= num_steps; ++k) {
        iLQRState state;
        state << reference.poses[k].x, reference.poses[k].y,
            reference.poses[k].theta, v_init[k], a_init[k], delta_init[k],
            omega_init[k];
        reference.initial_states.push_back(state);
    }
    reference.initial_controls.resize(num_steps, iLQRControl::Zero());
    // 打靶节点布设：{每 n_s 步} ∪ {cusp} ∪ {末点 N}，排序去重
    reference.shooting_nodes.reserve(
        (num_steps + config_.shooting_interval - 1) /
            config_.shooting_interval +
        reference.cusp_indices.size() + 2);
    for (std::size_t node = 0; node < num_steps;
         node += config_.shooting_interval) {
        reference.shooting_nodes.push_back(node);
    }
    reference.shooting_nodes.insert(reference.shooting_nodes.end(),
                                    reference.cusp_indices.begin(),
                                    reference.cusp_indices.end());
    reference.shooting_nodes.push_back(num_steps);
    std::sort(reference.shooting_nodes.begin(), reference.shooting_nodes.end());
    reference.shooting_nodes.erase(std::unique(reference.shooting_nodes.begin(),
                                               reference.shooting_nodes.end()),
                                   reference.shooting_nodes.end());
    std::sort(reference.cusp_indices.begin(), reference.cusp_indices.end());
    reference.cusp_indices.erase(std::unique(reference.cusp_indices.begin(),
                                             reference.cusp_indices.end()),
                                 reference.cusp_indices.end());
    return reference;
}
}  // namespace apa_post_processor
