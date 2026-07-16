#pragma once

#include <memory>
#include <string>
#include <vector>

#include "../constraints/constraint.hpp"
#include "../costs/cost_term.hpp"
#include "../models/dynamical_system.h"
#include "../util/trajectory.h"
#include "stage_parameters.h"

namespace stc_SQP {
// OCP 单段描述：同一动力学、同一代价、同一速度方向的连续预测区间
//   - stage_params 允许非空，要求长度等于本段 N，且每步 p 为 STAGE_PARAM_DIM 维
//     （当前固定 150）并全部有限。SQP 在约束求值前按 step 注入 p。
//   - v_sign 取 +1.0 表示前进，-1.0 表示后退；模型可选择是否解释该符号。
//     当前自行车模型（kappa/delta）不解释 v_sign，方向由状态 v 的符号表达；
//     v_sign 仅用于 OCP 层的换挡点检测与 RTI 降级。
//     hasGearShift() 检测相邻段速度方向变号并触发 RTI 降级。
struct StageSegment {
    // 该段使用的动力学模型
    std::shared_ptr<DynamicalSystem> dynamics;
    // 该段使用的代价项（包含 stage 与 terminal）。
    // 使用 CompositeCost 树形组合多项代价；整体 cost 为根节点 evaluate/gradient/hessian。
    std::shared_ptr<CostTerm> cost;
    // 该段附加的一般约束（不含 box bound；box bound 通过 x_min/x_max/u_min/u_max 表达）
    std::vector<std::shared_ptr<Constraint>> constraints;
    // 该段离散步数
    int N = 0;
    // 该段均匀步长（秒）；若 dt_array 非空则优先使用 dt_array
    double dt = 0.0;
    // 每步独立步长；为空时表示均匀步长 dt；每个元素必须 > 0
    std::vector<double> dt_array;
    // 速度方向符号：+1.0 前进，-1.0 后退；当前模型可选择是否解释该符号
    double v_sign = 1.0;
    // SQP 在约束求值时按 step 显式传入对应 p，不再通过 Constraint::setParameters 注入。
    std::vector<StageParameters> stage_params;
    // 状态上下界
    Vector x_min;
    Vector x_max;
    // 控制上下界
    Vector u_min;
    Vector u_max;
    // 返回第 i 步实际使用的时间步长
    double stepSize(int i) const;
};

// 多段 OCP：支持泊车等存在换挡点（速度方向改变）的轨迹优化问题
class MultiStageOCP {
public:
    // 默认构造
    MultiStageOCP() = default;
    // 添加一段 OCP 描述
    void addSegment(const StageSegment& segment);
    // 返回所有段
    const std::vector<StageSegment>& segments() const { return segments_; }
    // 返回所有段（允许运行时更新 stage_params 等非 const 成员）
    std::vector<StageSegment>& segments() { return segments_; }
    // 返回总离散步数
    int totalSteps() const;
    // 检查是否存在相邻段速度方向变号（换挡点）
    bool hasGearShift() const;
    // 返回状态维度（要求所有段一致）
    int nx() const;
    // 返回控制维度（要求所有段一致）
    int nu() const;
    // 校验 OCP 配置是否合法；若 reason 非空，写入失败原因
    bool validate(std::string* reason = nullptr) const;
    // 将全局步索引 k 映射到所在段与段内索引
    // 返回值：first=段索引，second=段内步索引；若越界返回 {-1,-1}
    std::pair<int, int> globalStepToSegment(int global_k) const;
    // 构造粗粒度 OCP，用于 Coarse-to-Fine 分层策略。
    // coarse_n 为期望的总步数（<=0 则按 coarse_dt 或默认规则自动决定）；
    // coarse_dt 为期望的粗时间步长（<=0 时忽略）。总时间保持与各段时间和一致。
    // 注意：为保证每段至少 1 步，当段数超过目标总步数时，实际粗化步数以段数为下限。
    MultiStageOCP coarsen(int coarse_n, double coarse_dt) const;

protected:
    std::vector<StageSegment> segments_;
};
} // namespace stc_SQP
