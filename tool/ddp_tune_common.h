#pragma once
// DDP 调参/审计工具的共享定义：数据集清单、基线配置加载与调参变体矩阵。
// tune_ddp.cpp（矩阵化跑批 + 评价排序）与 audit_ddp_post.cpp（四断点诊断
// 转储）共同消费本头文件，保证「评价口径」与「诊断取证」跑的是同一组变体，
// 变体定义只在此维护一份（单一真值来源）。
//
// 变体矩阵的维护约定（对应 Milestone 的方法论红线「一次只动一个机制」）：
// 每个变体相对基线只改一个方案/一个参数组，命名反映所改机制；已证伪的
// 变体从矩阵移除并把结论记入 review-log 与 known-limitations，避免矩阵
// 无限膨胀（历史批次见 docs/DDP.md 实测结果章）。
#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include "core/post_processor.h"
#include "util/config_loader.h"
#include "util/data_loader.hpp"

namespace apa_post_processor {
namespace ddp_tune {
// 数据集描述：显示名 + 文件路径
struct DatasetCase {
    std::string name;
    std::string file;
};
// 调参变体：显示名 + 完整 DdpConfig（以基线为底、按需覆盖）。
// modifier 为空指针表示纯基线；否则在基线副本上应用一次修改
struct TuneVariant {
    std::string name;
    DdpConfig config;
};
// 四份真实数据集（顺序固定，便于跨工具/跨轮次对照）
inline std::vector<DatasetCase> BuildDatasets() {
    return {{"data3_mid_park", "data/mid_park/data3.json"},
            {"data1_rub_park", "data/rub_park/data1.json"},
            {"data7_rub_park", "data/rub_park/data7.json"},
            {"data6_long_park", "data/long_park/data6.json"}};
}
// 从生产配置 data/ddp_config.json 构造基线 DdpConfig：基类字段
// （outer_row_num）与 DDP 专有字段的加载入口与 DDPPlanningScene 完全一致，
// 保证调参/审计与生产环境同一碰撞模型/同一参数来源；加载失败回退内置默认
inline DdpConfig LoadBaselineDdpConfig() {
    DdpConfig config;
    if (!LoadBaseConfigOverrides("data/ddp_config.json", &config)) {
        std::cout << "[TUNE] baseline: data/ddp_config.json base overrides "
                     "load failed, use built-in defaults"
                  << std::endl;
    }
    // 基类覆盖与 DDP 专有字段分两次解析同一 JSON：继承 PlanningScene 场景
    // 层的既有设计模式（known-limitations 已登记该技术债），非疏忽
    nlohmann::json details;
    if (DataLoader::LoadJsonFile("data/ddp_config.json", details) !=
        LoadResult::SUCCESS) {
        std::cout << "[TUNE] baseline: data/ddp_config.json parse failed, "
                     "use built-in defaults"
                  << std::endl;
        return config;
    }
    // 字段类型错误会抛 nlohmann::json::exception——显式捕获并回退默认，
    // 避免调参/审计批次被单个坏配置中断
    try {
        LoadDdpConfigOverrides(details, &config);
    } catch (const nlohmann::json::exception& e) {
        std::cout << "[TUNE] baseline: data/ddp_config.json field override "
                     "failed ("
                  << e.what() << "), use built-in defaults" << std::endl;
        config = DdpConfig{};
    }
    // 变体在基线之上以局部副本修改，修改后必须再同步一次幅值边界
    // （reference/inner 为唯一权威来源，直接字段改写会绕过构造期同步）
    config.synchronizeAmplitudeBounds();
    return config;
}
// 以基线为底构造一个命名变体：modifier 在基线副本上应用修改并负责再同步
// 幅值边界（lambda 内部必须调用 synchronizeAmplitudeBounds）
template <typename TModifier>
TuneVariant MakeVariant(std::string name, TModifier&& modifier) {
    TuneVariant variant{std::move(name), LoadBaselineDdpConfig()};
    modifier(&variant.config);
    variant.config.synchronizeAmplitudeBounds();
    return variant;
}
// 本批次跑批的变体矩阵（baseline 恒为第一个，作为全部对比的基准）。
// 批次沿革（逐批假设/实测/结论见 docs/milestones/milestone-011/review-log.md
// 的实验流水账与 docs/DDP.md 第 3 章实测记录）：本轮攻坚的全部实验变体
// 已完成评测，最终默认参数取「深退火 γ=0.3 + 阶段二跟踪权重地板 0.015 +
// 车辆真值幅值上限」（data/ddp_config.json），矩阵只留基线作回归验证。
// 2026-08-05 消融批次（abl_no_reanchor/abl_no_backoff/abl_no_s2floor/
// abl_no_perelem_s2/abl_no_merit_hook/abl_no_esdf_scale）已评测并撤出：
// 重锚与罚参数回退与基线逐位一致（机制随之删除），其余四项保留
// （s2floor/esdf_scale 实测承重，perelem_s2/merit_hook 变化在刀刃噪声带内）——
// 见 docs/milestones/milestone-013/review-log.md 战术改动 #2
inline std::vector<TuneVariant> BuildVariants() {
    std::vector<TuneVariant> variants;
    variants.emplace_back(TuneVariant{"baseline", LoadBaselineDdpConfig()});
    // 「关闭融化机制」的对照解（验收长度防线：待测解相对它不得增长
    // 超过 3%）：退火率 ≈1 使跟踪权重几乎不衰减，融化机制实际关闭；
    // 显式关闭双候选择优，保证对照是纯净的关融化单遍解（生产默认已
    // 开启双候选，不强制关闭会让对照口径被择优逻辑污染）
    variants.emplace_back(MakeVariant("nomelt_control", [](DdpConfig* config) {
        config->solver.outer.anneal_gamma = 0.999;
        config->dual_candidate_select = false;
    }));
    // L6.4 参考保形曲率投影两档已证伪撤出（cap1.0：data3 合法→回退、
    // data1 κ_ratio 1.0022 超门、data6 收敛成穿墙垃圾；cap0.95：data3
    // 阶段一失收敛、data7 回退。证据 build/log/tune_l64.log，逐条分析
    // 见 review-log Round 2 回应）
    // L6.3a 阶段二 ESDF 独立标定两档已证伪撤出（s2_wsafe300：data3 阶段二
    // δ 幅值门失败回退、data1/data7 门失败依旧；s2_margin0.05：data1 依旧、
    // data7 改为阶段二不收敛、data3 长度 +0.5%。证据 build/log/tune_l63a.log。
    // 结论：阶段二 collision 门失败不是 ESDF 标定问题——加强罚只把失败
    // 搬到别的门项，佐证门控-避障的几何不相容假设）
    // L6.2b 绝对长度比冻结触发已证伪（健康集逐位零副作用，但 data6 在
    // 高 w_ref 位冻结也拉不回跑飞盆地——μ 螺旋先杀内层。证据
    // build/log/tune_l62b2.log）
    // L7.1 时域 T 探针已证伪撤出（dt 0.07/0.05 两档四数据集全灭：data1/
    // data3 碰撞 0.14~0.15、data7/data6 阶段一失收敛——T 收缩与 v_nominal
    // 初值盆地移动锁定耦合，缩短 T 从未被干净测到。证据
    // build/log/tune_l71.log。按 spec 证伪条件本方向作废，L7.5 随之取消）
    // L7.2 双候选择优已采纳进生产默认（dual_candidate_select=true）：
    // data7 选对照解（6→4/15.54/阶段二/碰撞 0），data3/data1 选融化解
    // 逐位不变，④对照解防线由该择优自动满足（证据 tune_l72.log）
    // L8.6 修复后重判批次（dt_0.07/dt_0.05/vp_w0.11/rescue_0.05）已全部
    // 完成评测并撤出矩阵，结论固化：L7.1/L6.1 证伪在封闭可行域上复测
    // 仍成立；rescue_0.05 健康集逐位不变、data6 依旧回退。固定验收的
    // 变体列表保持 {baseline, nomelt_control} 两个（见 tool/accept_ddp.sh）
    // M012 Q1.b/Q1.c 评测批次已完成：ampcap_1e4 零收益不采纳（与 M011
    // L3.5 同结论）、merit_al1e-3 已采纳进生产默认（data/ddp_config.json
    // merit_mu_al_ratio=1e-3/merit_mu_max=1e3）——见
    // docs/milestones/milestone-012/review-log.md 实验流水账
    // N1 评测批次：幅值罚参数逐元素独立门控（仅硬化真正违反的元素，
    // 不广播到早已满足的约束）——阶段二形态已采纳进生产默认
    // （data/ddp_config.json amplitude_mu_per_element_stage_two=true；
    // 阶段一形态已证伪并移除 JSON 入口）
    // N2 评测批次：前馈步信赖域盒（饱和方向上的真实步长阻尼）——
    // 部分有效不采纳（data7 长度越 ALM+5% 门、data6 转为稳定卡死/
    // 合法绕远），机制实现已随证伪清除删除
    // Q5 评测批次（arc_length 1.05/1.10/1.15 三档）：data6 全灭证伪
    // （墙在任何比值都吸附、AL 压墙失稳而非改道；机理与取证见
    // review-log Q5 条目）——且第 8 维 ℓ 状态即使默认关闭也扰动
    // 刀刃案例（data3 段数 5→7、data7 +0.49 m），「关闭零成本」
    // 不成立，已经人工裁决**整体回退到 7 维**（代码与测试移除，
    // 文档留档）
    // Q5b 批次（freeze_1_02/freeze_1_05 退火逃逸冻结重试）：证伪
    // 撤出——1.02 档误伤 data3（瞬态触发冻结锁死 w_ref，阶段二失活），
    // 两档 data6 均死于「贴参考 ∧ 幅值可行」不可兼得（review-log
    // Q5b 条目）；机制实现已随证伪清除删除
    return variants;
}
}  // namespace ddp_tune
}  // namespace apa_post_processor
