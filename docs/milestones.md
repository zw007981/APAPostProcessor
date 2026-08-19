# 开发计划与状态机

> 本文件是所有 Milestone 的顶层索引与状态总览。每个 Milestone 的详细任务书与评审记录在
> `docs/milestones/milestone-NNN/` 下（`spec.md` + `review-log.md`），编号统一用 3 位数字、
> 全局唯一且永不回收/重命名（即使某个 Milestone 中途废弃，编号也作废不复用）。
>
> 战术级别的局部改动（如优化某个热点函数、修复一个孤立缺陷）不需要新建独立条目：若有战略级影响（改变了
> 某个 Milestone 的范围/验收标准），直接更新对应 Milestone 的 `spec.md`；
> Milestone 的开发过程处理即可，不必强制记录。
>
> 本文件已于 2026-07-28 套用 [.agents/prompts/reset-development-history.md](../.agents/prompts/reset-development-history.md) 清空全部历史 Milestone 记录，重新回到初始骨架状态；此前的记录如需追溯，请查阅 Git 历史。

## 状态定义

| 状态 | 含义 |
|---|---|
| 未开始 | 尚未有 Dev Agent 进场 |
| 开发中 | Dev Agent 正在实现（套用 `.agents/prompts/milestone-implement.md`） |
| 评审中 | 等待 Review Agent 给出本轮意见（套用 `.agents/prompts/milestone-code-review.md`） |
| 待修改 | Review Agent 已给出意见，等待 Dev Agent 处理（套用 `.agents/prompts/apply-review-feedback.md`） |
| 已收敛 | 最新一轮 review-log.md 中 🚨=0 且 ⚠️=0，尚未正式关闭 |
| 已完成 | 已套用 `.agents/prompts/milestone-close.md` 正式关闭 |
| 已归档 | 已套用 `.agents/prompts/milestone-archive.md`，详细记录收纳进 `docs/milestones/phases/` 下的阶段小结 |

## 当前 Milestone（默认编号指针）

**当前默认编号：013**

Dev Agent / Review Agent 在人工没有于对话中显式指定 Milestone 编号时，一律以此处登记的编号作为默认的 `{N}`，不需要人工在每一次开场白里手动写明或来回改写编号。人工只需要在切换到不同 Milestone 时更新这一行数字（这是整个流程里唯一需要手动改编号的地方）；如需临时处理非当前编号的 Milestone（如战术改动、复审旧 Milestone），仍可在对话中显式指定，显式指定优先于此处的默认值。

## Milestone 列表

> 当前列表对应 **iLQR（MS-iLQR + AL）后处理算法路径**的开发计划（设计文档：
> [docs/iLQR.md](iLQR.md)，领域模型：[docs/architecture.md](architecture.md) 3.8 节，
> 接口规划：[docs/interfaces.md](interfaces.md)「待实现：iLQR 模块核心接口」一节）。
> 依赖关系：001/002/003 互相独立可任意顺序开发；004 依赖 002；005 依赖 002/003/004；
> 006 依赖 001/005；007 依赖 006；008 依赖 007；009 依赖 008；010 依赖 009；011 依赖 010。
>
> 001~009 完成的是**求解链路本身**（从参考构建到端到端接入）；010/011 是在此之上的
> **效果攻坚**：M009 四数据集验收为「2 收敛 + 2 回退」，一半场景零收益。010 先审计并
> 重构后处理与通过性校验口径（怀疑数值优化有效、损失发生在后处理/校验环节），
> 011 再在正确的口径上做方案层与参数层的系统优化。**011 必须在 010 收口后开始**——
> 否则效果会被错误的校验口径吃掉，得出全部错误的调参结论。
>
> 012 由一次独立复测触发，是 011 的**前置修正**而非续作：复测发现 011 记录的四数据集
> 基线在当前 HEAD 上不可复现（实测「1 成功 + 3 回退」），因此 011 之上的调参结论
> 都缺少可信对照组。012 先把基线复现与验收口径收口（Q0），再攻 data6 阶段一的
> 打靶缺陷死锁与 data3/data7 的两处链路缺陷。**011 与 012 的关系需人工裁决**
> （见 [milestone-012/spec.md](milestones/milestone-012/spec.md) 第 9 节）。

| 编号 | 名称 | 状态 | 详情 |
|---|---|---|---|
| 001 | iLQR 预处理与参考轨迹构建（重采样/初值/打靶节点） | 已完成 | [milestone-001](milestones/milestone-001/spec.md) |
| 002 | 七维自行车动力学与解析雅可比 | 已完成 | [milestone-002](milestones/milestone-002/spec.md) |
| 003 | Box-QP 投影牛顿求解器 | 已完成 | [milestone-003](milestones/milestone-003/spec.md) |
| 004 | 代价与约束求值层（AL 增广 + ESDF 惩罚） | 已完成 | [milestone-004](milestones/milestone-004/spec.md) |
| 005 | MS-iLQR 内层求解器（缺陷感知回推/rollout/merit 线搜索） | 已完成 | [milestone-005](milestones/milestone-005/spec.md) |
| 006 | AL 外层循环与阶段一全局软化求解 | 已完成 | [milestone-006](milestones/milestone-006/spec.md) |
| 007 | 后处理与阶段二门控精化（修剪/驻留/校验/回退） | 已完成 | [milestone-007](milestones/milestone-007/spec.md) |
| 008 | PostProcessor 与场景层接入（optimizeiLQR/ILQRPlanningScene/配置路由） | 已完成 | [milestone-008](milestones/milestone-008/spec.md) |
| 009 | 四数据集调参与端到端验收 | 已完成 | [milestone-009](milestones/milestone-009/spec.md) |
| 010 | iLQR 后处理链路与通过性校验口径审计与重构 | 已完成 | [milestone-010](milestones/milestone-010/spec.md) |
| 011 | iLQR 换挡数系统性优化（方案层 + 参数层） | 已完成 | [milestone-011](milestones/milestone-011/spec.md) |
| 012 | iLQR 阶段一收敛失效根因攻坚（data6 死锁 + 基线不可复现） | 评审中 | [milestone-012](milestones/milestone-012/spec.md) |
| 013 | 合法前提下 maneuver 段数削减攻坚（自由时间/ESDF AL 组/δ→κ 候选） | 开发中 | [milestone-013](milestones/milestone-013/spec.md) |

## 规模化：可选的阶段归档（大多数项目不需要）

如果本项目生命周期长、Milestone 数量会积累到几十上百个（比如多年持续维护的大型 repo），当这张表读起来开始吃力时（通常是一个自然的阶段边界，如"主力开发转维护"），可以人工触发 [.agents/prompts/milestone-archive.md](../.agents/prompts/milestone-archive.md)：Agent 起草阶段小结（`docs/milestones/phases/phase-N-xxx.md`），人工校对定稿后，把已归档区间在本表收起为一行，未归档的 Milestone 继续按行展示。

**这是可选能力，不是必须遵循的结构**——如果项目规模有限（如一个几十个 Milestone 就基本稳定的工具库），永远用上面的扁平列表即可，不需要引入 `phases/` 目录。归档过程**不会**移动或删除任何 `docs/milestones/milestone-NNN/` 原始目录，只新增摘要文件，避免破坏历史链接。
