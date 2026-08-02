// DDP 四数据集调参驱动工具：对给定的 DdpConfig 变体矩阵，逐变体 × 逐数据集
// 执行 `PostProcessor::optimizeDdp`，输出两层机器可读结果：
//   [TUNE]       每个 变体×数据集 的逐项量测（机动段数/长度/终点误差/碰撞
//                深度/运动学四残差/耗时/降级级别）；
//   [TUNE-RANK]  变体级汇总排序（评价函数固化：合法 → maneuver 数 → 长度比，
//                终点误差与耗时仅作参考列不参与排序；回退按「段数=输入段数、
//                长度比=1.0」计入，天然劣于任何合法输出）；
//   [TUNE-GATE]  对照验收口径的逐项门检（合法性/零回退/ALM 基线/前序
//                收口基线/长度比上限/耗时量级），供采纳/否决判断直接引用。
// 变体矩阵定义在 ddp_tune_common.h（与审计工具共享同一真值来源）。
// 运行示例：
//   ./build/Release/apa_tune_ddp
#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "core/post_processor.h"
#include "core/NMPC/vehicle_circle_geometry.h"
#include "spatial/esdf_map.h"
#include "spatial/grid_map.h"
#include "util/logger.h"
#include "util/trajectory.h"
#include "vehicle/vehicle_footprint_model.h"
#include "vehicle/vehicle_params.h"

#include "ddp_tune_common.h"

using namespace apa_post_processor;
using ddp_tune::DatasetCase;
using ddp_tune::TuneVariant;

namespace {
// 单个 变体×数据集 的评价记录：legal=false 的变体直接出局（不参与排序）；
// 回退（success=0）按输入段数/长度计入，天然劣于任何合法输出
struct EvalRecord {
    std::string variant;
    std::string dataset;
    bool legal{false};
    // 输出级别：2=阶段二精化、1=阶段一降级、0=原始 A* 回退
    int level{0};
    int maneuvers_in{0};
    int maneuvers_out{0};
    double length_in{0.0};
    double length_out{0.0};
    double len_ratio{1.0};
    double term_pos{0.0};
    double term_head{0.0};
    double collision{0.0};
    // 曲率包络量测（行驶点 |v|≥0.05，对齐 tune_alm 的口径）：
    // max|κ|、P95 分位、相对车辆物理上限的比值 max|κ|/κ_max
    double max_kappa{0.0};
    double kappa_p95{0.0};
    double kappa_p50{0.0};
    double kappa_ratio{0.0};
    // 同数据集 A* 输入的 κ 分位（形态门的相对基准：输出不劣于输入）
    double input_kappa_max{0.0};
    double input_kappa_p95{0.0};
    double input_kappa_p50{0.0};
    // 转角速率包络 max|ω|（全轨迹，含驻留段——执行器在驻留中同样工作）
    double max_omega{0.0};
    // L8.4 分项归因：输出轨迹的图外足迹点数与仅图内点的最大侵入深度
    int out_of_map_points{0};
    double inmap_intrusion{0.0};
    double time_ms{0.0};
    std::string msg;
};
// 行驶点的曲率包络统计（|v|≥0.05 才计入——驻留/静止段的 κ 无物理意义，
// 与 tune_alm 的 ComputeMaxDrivingKappa 同一口径）；返回 {max, P95, P50}
// （最近秩分位），供硬门（max≤1.002·κ_max）与形态门（输出分位不劣于
// 同数据集输入分位）消费
struct KappaEnvelope {
    double max{0.0};
    double p95{0.0};
    double p50{0.0};
};
KappaEnvelope ComputeKappaEnvelope(const Trajectory& traj, double wheelbase) {
    std::vector<double> kappas;
    kappas.reserve(traj.size());
    for (const auto& pt : traj) {
        if (!pt.hasV() || !pt.hasDelta() || std::abs(pt.getV()) < 0.05) {
            continue;
        }
        kappas.push_back(std::abs(std::tan(pt.getDelta())) / wheelbase);
    }
    if (kappas.empty()) {
        return KappaEnvelope{};
    }
    std::sort(kappas.begin(), kappas.end());
    const auto quantile = [&kappas](double q) {
        return kappas[std::min(
            kappas.size() - 1,
            static_cast<std::size_t>(
                std::ceil(q * static_cast<double>(kappas.size()))) -
                1)];
    };
    return KappaEnvelope{kappas.back(), quantile(0.95), quantile(0.50)};
}
// 输入 A* 路径的曲率分位（θ 差分/弧长；Round 2 形态门改为「输出分位
// 不劣于同数据集输入分位」的相对口径，此处量取输入基准）
KappaEnvelope ComputeInputKappaEnvelope(const Path& path, double wheelbase) {
    std::vector<double> kappas;
    kappas.reserve(path.size());
    double prev_x = 0.0;
    double prev_y = 0.0;
    double prev_theta = 0.0;
    bool first = true;
    path.forEach([&](const TrajectoryPoint& pt) {
        if (!first) {
            const double ds = std::hypot(pt.x - prev_x, pt.y - prev_y);
            if (ds > 1e-9) {
                kappas.push_back(
                    std::abs(WrapAngle(pt.theta - prev_theta)) / ds);
            }
        }
        prev_x = pt.x;
        prev_y = pt.y;
        prev_theta = pt.theta;
        first = false;
    });
    if (kappas.empty()) {
        return KappaEnvelope{};
    }
    std::sort(kappas.begin(), kappas.end());
    const auto quantile = [&kappas](double q) {
        return kappas[std::min(
            kappas.size() - 1,
            static_cast<std::size_t>(
                std::ceil(q * static_cast<double>(kappas.size()))) -
                1)];
    };
    return KappaEnvelope{kappas.back(), quantile(0.95), quantile(0.50)};
}
// 把消息文本压缩成单行（异步日志行/内嵌换行会破坏机器可读行格式）
std::string Sanitize(std::string text) {
    std::replace(text.begin(), text.end(), '\n', ' ');
    std::replace(text.begin(), text.end(), '\r', ' ');
    std::replace(text.begin(), text.end(), '"', '\'');
    return text;
}
// 从结果消息解析输出级别（分级降级的语义经 message 文本如实携带）
int ParseLevel(bool success, const std::string& message) {
    if (!success) {
        return 0;
    }
    return message.find("stage-one candidate") != std::string::npos ? 1 : 2;
}
// 在单数据集上评价一个变体，填充评价记录并输出 [TUNE] 逐项量测行
EvalRecord RunSingle(const TuneVariant& variant, const DatasetCase& dataset) {
    EvalRecord record;
    record.variant = variant.name;
    record.dataset = dataset.name;
    ::apa::post_processor::OptimizeRequest request;
    if (DataLoader::LoadProtoFromJsonFile(dataset.file, request) !=
        LoadResult::SUCCESS) {
        std::cout << "[TUNE] variant=" << variant.name
                  << " dataset=" << dataset.name << " LOAD_FAILED" << std::endl;
        return record;
    }
    const auto vehicle_params = VehicleParams::FromProto(request.vehicle());
    // 外圆行数与变体配置同源（Config::outer_row_num，由
    // data/ddp_config.json 覆盖），保证调参与生产同一碰撞模型
    const VehicleFootprintModel footprint_model(
        vehicle_params, /*heading_sample_num=*/233, /*inner_row_num=*/2,
        variant.config.outer_row_num);
    const auto grid_map = GridMap::FromProto(request.environment());
    const ESDFMap esdf_map(grid_map);
    const auto init_path = Path::FromProto(request.initial_path());
    const PostProcessor processor(vehicle_params, footprint_model, esdf_map);
    const auto result = processor.optimizeDdp(init_path, variant.config);
    record.maneuvers_in = static_cast<int>(init_path.numManeuvers());
    record.length_in = init_path.length();
    record.level = ParseLevel(result.success, result.message);
    record.time_ms = result.total_time_ms;
    record.msg = Sanitize(result.message);
    if (!result.success || result.ddp_traj.empty()) {
        // 回退分支：段数/长度按输入计（天然劣于任何合法输出），message
        // 携带结构化诊断（失败阶段 + 失败项 + 量化值/阈值），原样透传
        record.maneuvers_out = record.maneuvers_in;
        record.length_out = record.length_in;
        std::cout << "[TUNE] variant=" << record.variant
                  << " dataset=" << record.dataset << " success=0"
                  << " level=0 maneuvers=" << record.maneuvers_in << "->"
                  << record.maneuvers_in << " length=" << record.length_in
                  << "->" << record.length_in << " len_ratio=1"
                  << " time_ms=" << record.time_ms << " msg=\"" << record.msg
                  << "\"" << std::endl;
        return record;
    }
    const auto& goal_pt = init_path.back();
    const TrajectoryPoint goal(goal_pt.x, goal_pt.y, goal_pt.theta);
    const auto validation =
        result.ddp_traj.validate(goal, esdf_map, footprint_model);
    record.maneuvers_out = result.final_maneuvers;
    record.length_out = result.final_length;
    record.len_ratio = result.final_length / record.length_in;
    record.term_pos = validation.terminal_position_error;
    record.term_head = validation.terminal_heading_error_deg;
    record.collision = validation.max_intrusion_depth;
    // 曲率包络：max/P95/P50（行驶点）与相对车辆物理上限的比值；
    // 输入分位为形态门的相对基准（输出不劣于同数据集输入）
    const auto output_env =
        ComputeKappaEnvelope(result.ddp_traj, vehicle_params.wheelbase);
    const auto input_env =
        ComputeInputKappaEnvelope(init_path, vehicle_params.wheelbase);
    record.max_kappa = output_env.max;
    record.kappa_p95 = output_env.p95;
    record.kappa_p50 = output_env.p50;
    record.kappa_ratio = output_env.max / vehicle_params.max_kappa;
    record.input_kappa_max = input_env.max;
    record.input_kappa_p95 = input_env.p95;
    record.input_kappa_p50 = input_env.p50;
    for (const auto& pt : result.ddp_traj) {
        if (pt.hasDeltaDot()) {
            record.max_omega =
                std::max(record.max_omega, std::abs(pt.getDeltaDot()));
        }
    }
    // L8.4 分项归因（输出侧）：图外轨迹点数 + 仅图内点的最大侵入深度——
    // 区分「图内轻微贴障」与「图外穿透」（L8 修复前两者的量测被零梯度
    // 平台污染、不可区分）。collision 聚合口径不变（含图外穿透深度）
    const auto outer_circles =
        vehicle_circle_geometry::ExtractLocalCircleCenters(
            footprint_model, CircleType::OUTER);
    const double outer_radius = footprint_model.getOuterRadius();
    for (const auto& pt : result.ddp_traj) {
        const double cos_theta = std::cos(pt.theta);
        const double sin_theta = std::sin(pt.theta);
        for (const auto& local : outer_circles) {
            const double wx =
                pt.x + local.x() * cos_theta - local.y() * sin_theta;
            const double wy =
                pt.y + local.x() * sin_theta + local.y() * cos_theta;
            if (!esdf_map.inMap(wx, wy)) {
                ++record.out_of_map_points;
                continue;
            }
            record.inmap_intrusion =
                std::max(record.inmap_intrusion,
                         outer_radius - esdf_map.getDist(wx, wy));
        }
    }
    // 曲率/转角速率硬门归入合法性口径（验收标准⑨，Round 2 裁决口径）：
    // 幅值硬限必须取 VehicleParams 真值；1.002 为数值容差带（吸收 AL
    // 平衡残余与浮点噪声），不是工程让步——实测三集 1.00015/0.99655/
    // 0.99295 实质已满足严格 1.0，禁止再放宽
    record.legal = validation.all_passed &&
                   record.kappa_ratio <= 1.002 &&
                   record.max_omega <=
                       1.002 * vehicle_params.max_steer_rate;
    std::cout << "[TUNE] variant=" << record.variant
              << " dataset=" << record.dataset << " success=1"
              << " level=" << record.level
              << " maneuvers=" << record.maneuvers_in << "->"
              << record.maneuvers_out << " length=" << record.length_in << "->"
              << record.length_out << " len_ratio=" << record.len_ratio
              << " term_pos=" << record.term_pos
              << " term_head=" << record.term_head
              << " collision=" << record.collision
              << " kin_pos=" << validation.max_kinematic_position_residual
              << " kin_head=" << validation.max_kinematic_heading_residual_deg
              << " kin_vel=" << validation.max_kinematic_velocity_residual
              << " kin_steer=" << validation.max_kinematic_steer_residual
              << " max_kappa=" << record.max_kappa
              << " kappa_p95=" << record.kappa_p95
              << " kappa_p50=" << record.kappa_p50
              << " kappa_ratio=" << record.kappa_ratio
              << " in_max=" << record.input_kappa_max
              << " in_p95=" << record.input_kappa_p95
              << " in_p50=" << record.input_kappa_p50
              << " max_omega=" << record.max_omega
              << " oom_pts=" << record.out_of_map_points
              << " inmap_coll=" << record.inmap_intrusion
              << " legal=" << (record.legal ? 1 : 0)
              << " time_ms=" << record.time_ms << " msg=\"" << record.msg
              << "\"" << std::endl;
    return record;
}
// 验收对照基线（Round 2 裁决口径）：ALM 基线 maneuver 数（车辆真值幅值
// 下测得：9→7、10→4、6→4、6→4）与 ALM 真值长度（当前权威取值：data3
// 21.60 m、data7 16.14 m；data1/data6 无权威 ALM 长度，不参与该项）。
// 假上限下的 M010 数字已作废（依赖真实车辆做不到的转向能力）
struct DatasetBaseline {
    int alm_maneuvers;
    double alm_length;  // 无权威取值记 <0（不参与 ALM 长度对照）
};
DatasetBaseline LookupBaseline(const std::string& dataset) {
    if (dataset == "data3_mid_park") {
        return DatasetBaseline{7, 21.60};
    }
    if (dataset == "data1_rub_park") {
        return DatasetBaseline{4, -1.0};
    }
    if (dataset == "data7_rub_park") {
        return DatasetBaseline{4, 16.14};
    }
    return DatasetBaseline{4, -1.0};  // data6_long_park
}
// 变体级汇总与验收门检：排序键 = 合法数据集数（降）→ 有效段数总和（升）
// → 最大长度比（升）；门检逐项对照验收口径输出 PASS/FAIL
struct VariantSummary {
    std::string name;
    int legal_count{0};
    int sum_maneuvers{0};
    double max_len_ratio{0.0};
    double sum_time_ms{0.0};
    int alm_strictly_better{0};
    bool alm_all_not_worse{true};
    bool len_ratio_ok{true};
    // 相对 ALM 真值长度（data3 21.60 / data7 16.14）劣化不超过 5%
    bool alm_length_ok{true};
    // 相对「同配置关闭融化的对照解」增长不超过 3%（防用绕路换换挡）
    bool melt_control_length_ok{true};
    // 形态门（Round 2 纠错后口径）：输出 κ 的 P50/P95/max 逐项不劣于
    // 同数据集 A* 输入的对应分位
    bool kappa_shape_ok{true};
    // 曲率包络聚合（四数据集最不利值，参考列）
    double max_kappa_ratio{0.0};
    double max_p95_ratio{0.0};
    std::vector<EvalRecord> cells;
};
VariantSummary Summarize(const std::string& name,
                         std::vector<EvalRecord> cells) {
    VariantSummary summary;
    summary.name = name;
    summary.cells = std::move(cells);
    for (const auto& cell : summary.cells) {
        const DatasetBaseline baseline = LookupBaseline(cell.dataset);
        if (cell.legal) {
            ++summary.legal_count;
            summary.max_kappa_ratio =
                std::max(summary.max_kappa_ratio, cell.kappa_ratio);
            // 形态门：输出 κ 的 P50/P95/max 逐项不劣于输入对应分位
            // （留 0.1% 数值容差吸收浮点噪声）
            if (cell.kappa_p50 > cell.input_kappa_p50 * 1.001 ||
                cell.kappa_p95 > cell.input_kappa_p95 * 1.001 ||
                cell.max_kappa > cell.input_kappa_max * 1.001) {
                summary.kappa_shape_ok = false;
            }
            // 相对 ALM 真值长度劣化不超过 5%（仅 data3/data7 有权威取值）
            if (baseline.alm_length > 0.0 &&
                cell.length_out > baseline.alm_length * 1.05) {
                summary.alm_length_ok = false;
            }
        }
        summary.sum_maneuvers += cell.maneuvers_out;
        summary.max_len_ratio = std::max(summary.max_len_ratio, cell.len_ratio);
        summary.sum_time_ms += cell.time_ms;
        // 回退按输入段数计（cell.maneuvers_out 已在记录层做了该约定）；
        // 与 ALM 基线的逐数据集比较一律用有效段数
        if (cell.maneuvers_out < baseline.alm_maneuvers) {
            ++summary.alm_strictly_better;
        }
        if (cell.maneuvers_out > baseline.alm_maneuvers) {
            summary.alm_all_not_worse = false;
        }
        if (cell.len_ratio > 1.05) {
            summary.len_ratio_ok = false;
        }
    }
    return summary;
}
}  // namespace

int main() {
    // 求解日志写文件（build/log），控制台关闭——异步日志行与 [TUNE] 行
    // 在 stdout 交错会破坏机器可读格式，逐轮诊断从日志文件/审计工具获取
    Logger::SetLogDirectory(std::string(PROJECT_ROOT_DIR) + "/build/log");
    Logger::SetConsoleOutputEnabled(false);
    const auto datasets = ddp_tune::BuildDatasets();
    const auto variants = ddp_tune::BuildVariants();
    std::vector<EvalRecord> records;
    records.reserve(variants.size() * datasets.size());
    for (const auto& variant : variants) {
        for (const auto& dataset : datasets) {
            records.push_back(RunSingle(variant, dataset));
        }
    }
    // 变体级汇总排序：全合法的变体参与排名（段数总和 → 最大长度比）；
    // 存在不合法数据集的变体直接出局（只列出不排名）
    std::vector<VariantSummary> summaries;
    summaries.reserve(variants.size());
    for (const auto& variant : variants) {
        std::vector<EvalRecord> cells;
        cells.reserve(datasets.size());
        for (const auto& record : records) {
            if (record.variant == variant.name) {
                cells.push_back(record);
            }
        }
        summaries.push_back(Summarize(variant.name, std::move(cells)));
    }
    std::sort(summaries.begin(), summaries.end(),
              [](const VariantSummary& lhs, const VariantSummary& rhs) {
                  if (lhs.legal_count != rhs.legal_count) {
                      return lhs.legal_count > rhs.legal_count;
                  }
                  if (lhs.sum_maneuvers != rhs.sum_maneuvers) {
                      return lhs.sum_maneuvers < rhs.sum_maneuvers;
                  }
                  return lhs.max_len_ratio < rhs.max_len_ratio;
              });
    // 「同配置关闭融化机制」的对照解长度（防用绕路换换挡的防线，
    // 与待测解同口径、不受历史基线污染）：取 nomelt_control 变体的
    // 逐数据集合法长度
    std::vector<double> control_length(datasets.size(), -1.0);
    for (const auto& record : records) {
        if (record.variant != "nomelt_control" || !record.legal) {
            continue;
        }
        for (std::size_t d = 0; d < datasets.size(); ++d) {
            if (record.dataset == datasets[d].name) {
                control_length[d] = record.length_out;
            }
        }
    }
    int rank = 0;
    for (auto& summary : summaries) {
        const bool ranked =
            summary.legal_count == static_cast<int>(datasets.size());
        if (ranked) {
            ++rank;
        }
        // 相对对照解增长不超过 3%（对照解本身与含回退数据集的变体跳过）
        if (summary.name != "nomelt_control") {
            for (std::size_t d = 0; d < datasets.size(); ++d) {
                const EvalRecord& cell = summary.cells[d];
                if (cell.legal && control_length[d] > 0.0 &&
                    cell.length_out > control_length[d] * 1.03) {
                    summary.melt_control_length_ok = false;
                }
            }
        }
        std::cout << "[TUNE-RANK] rank=" << (ranked ? std::to_string(rank) : "OUT")
                  << " variant=" << summary.name << " legal="
                  << summary.legal_count << "/" << datasets.size()
                  << " sum_maneuvers=" << summary.sum_maneuvers
                  << " max_len_ratio=" << summary.max_len_ratio
                  << " sum_time_ms=" << summary.sum_time_ms << std::endl;
        // 验收门检（Round 2 裁决口径）：合法性（全 4 合法即零回退）/
        // ALM 基线（maneuver 不劣 + ≥2 严格优于）/长度三线
        // （L/L0≤1.05、ALM 真值长度 ≤+5%、对照解 ≤+3%）/曲率形态门
        const bool all_legal = ranked;
        const bool maneuver_ok = summary.alm_all_not_worse &&
                                 summary.alm_strictly_better >= 2;
        std::cout << "[TUNE-GATE] variant=" << summary.name
                  << " all_legal=" << (all_legal ? 1 : 0)
                  << " alm_not_worse=" << (summary.alm_all_not_worse ? 1 : 0)
                  << " alm_strictly_better=" << summary.alm_strictly_better
                  << "(>=2)"
                  << " len_ratio_ok=" << (summary.len_ratio_ok ? 1 : 0)
                  << " alm_length_ok=" << (summary.alm_length_ok ? 1 : 0)
                  << " melt_ctrl_len_ok="
                  << (summary.melt_control_length_ok ? 1 : 0)
                  << " kappa_shape_ok=" << (summary.kappa_shape_ok ? 1 : 0)
                  << " max_kappa_ratio=" << summary.max_kappa_ratio
                  << " accept="
                  << (all_legal && maneuver_ok && summary.len_ratio_ok &&
                              summary.alm_length_ok &&
                              summary.melt_control_length_ok &&
                              summary.kappa_shape_ok
                          ? "PASS"
                          : "FAIL")
                  << std::endl;
        // 曲率-换挡数帕累托行（验收标准⑨量测口径：不允许只报单点）
        std::cout << "[TUNE-PARETO] variant=" << summary.name
                  << " sum_maneuvers=" << summary.sum_maneuvers
                  << " max_len_ratio=" << summary.max_len_ratio
                  << " max_kappa_ratio=" << summary.max_kappa_ratio
                  << " max_p95_ratio=" << summary.max_p95_ratio << std::endl;
    }
    return 0;
}
