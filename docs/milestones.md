# 开发计划与状态机

> 本文件是所有 Milestone 的顶层索引与状态总览。每个 Milestone 的详细任务书与评审记录在
> `docs/milestones/milestone-NNN/` 下（`spec.md` + `review-log.md`），编号统一用 3 位数字、
> 全局唯一且永不回收/重命名（即使某个 Milestone 中途废弃，编号也作废不复用）。
>
> 战术级别的局部改动（如优化某个热点函数、修复一个孤立缺陷）不需要新建独立条目：若有战略级影响（改变了
> 某个 Milestone 的范围/验收标准），直接更新对应 Milestone 的 `spec.md`；
> Milestone 的开发过程处理即可，不必强制记录。
>
> 本文件已于 2026-07-18 套用 [.agents/prompts/reset-development-history.md](../.agents/prompts/reset-development-history.md) 清空全部历史 Milestone 记录，重新回到初始骨架状态；此前的记录如需追溯，请查阅 Git 历史。

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

**当前默认编号：001**（ALM/MINCO/PHR-ALM 模块 Milestone 序列起点，`milestone-001`~`milestone-008`，均处于「未开始」状态，尚无 Dev Agent 进场；系统设计见 [docs/architecture.md](architecture.md) 3.7 节与 [docs/ALM.md](ALM.md)，接口规划见 [docs/interfaces.md](interfaces.md)「待实现：ALM 模块核心接口」一节）

Dev Agent / Review Agent 在人工没有于对话中显式指定 Milestone 编号时，一律以此处登记的编号作为默认的 `{N}`，不需要人工在每一次开场白里手动写明或来回改写编号。人工只需要在切换到不同 Milestone 时更新这一行数字（这是整个流程里唯一需要手动改编号的地方）；如需临时处理非当前编号的 Milestone（如战术改动、复审旧 Milestone），仍可在对话中显式指定，显式指定优先于此处的默认值。

## Milestone 列表

| 编号 | 名称 | 状态 | 详情 |
|---|---|---|---|
| milestone-001 | MINCO 核心：多项式轨迹表示与块三对角求解器 | 未开始 | [spec.md](milestones/milestone-001/spec.md) |
| milestone-002 | θ-s 空间到阿克曼状态的运动学映射与物理约束惩罚 | 未开始 | [spec.md](milestones/milestone-002/spec.md) |
| milestone-003 | 前端 Hybrid A* 解析：换挡打断与初值提取 | 未开始 | [spec.md](milestones/milestone-003/spec.md) |
| milestone-004 | ESDF 双重安全机制惩罚（margin_safe/margin_comf） | 未开始 | [spec.md](milestones/milestone-004/spec.md) |
| milestone-005 | 预处理粗优化（两阶段流程第一阶段 Jpre） | 未开始 | [spec.md](milestones/milestone-005/spec.md) |
| milestone-006 | PHR-ALM 主优化循环（AlmSolver 内外层） | 未开始 | [spec.md](milestones/milestone-006/spec.md) |
| milestone-007 | 机动融化与拓扑修剪 | 未开始 | [spec.md](milestones/milestone-007/spec.md) |
| milestone-008 | 端到端集成与四数据集验收 | 未开始 | [spec.md](milestones/milestone-008/spec.md) |

## 规模化：可选的阶段归档（大多数项目不需要）

如果本项目生命周期长、Milestone 数量会积累到几十上百个（比如多年持续维护的大型 repo），当这张表读起来开始吃力时（通常是一个自然的阶段边界，如"主力开发转维护"），可以人工触发 [.agents/prompts/milestone-archive.md](../.agents/prompts/milestone-archive.md)：Agent 起草阶段小结（`docs/milestones/phases/phase-N-xxx.md`），人工校对定稿后，把已归档区间在本表收起为一行，未归档的 Milestone 继续按行展示。

**这是可选能力，不是必须遵循的结构**——如果项目规模有限（如一个几十个 Milestone 就基本稳定的工具库），永远用上面的扁平列表即可，不需要引入 `phases/` 目录。归档过程**不会**移动或删除任何 `docs/milestones/milestone-NNN/` 原始目录，只新增摘要文件，避免破坏历史链接。
