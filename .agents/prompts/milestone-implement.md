---
mode: agent
description: "按设计文档实现指定 Milestone（TDD），首次开发"
---

## 适用场景

Dev Agent 开始开发某个具体 Milestone（首次实现）时使用。

> 战术级别的局部改动（如优化某个热点函数、把一个大函数拆分成多个小函数）不算新的 Milestone，不要为此另建目录或走一轮完整流程：它一定属于某个战略目标（某个 Milestone）。如果这个改动会影响所在 Milestone 的范围/验收标准（战略级影响），先更新该 Milestone 的 `spec.md` 记录这次变更，再按本模板正常开发；如果不影响对外承诺的范围（纯执行细节），直接在该 Milestone 开发过程中处理即可，不需要为此新建目录或单独走一轮完整流程。

## 前置阅读

- [docs/architecture.md](../../docs/architecture.md) —— 系统设计文档
- [docs/interfaces.md](../../docs/interfaces.md) —— 核心接口契约（本 Milestone 若涉及冻结接口，严禁擅自变更签名）
- `docs/milestones/milestone-{N}/spec.md` —— 本 Milestone 的任务书（范围/交付物/验收标准）
- [.agents/instructions/cpp-style.md](../instructions/cpp-style.md)

> 编号 `{N}` 的确定方式：若人工在本次对话中已明确指定 Milestone 编号，以人工指定为准；否则读取 [docs/milestones.md](../../docs/milestones.md) 中「当前 Milestone（默认编号指针）」一节的默认编号。

## 标准步骤

0. 若 `docs/milestones/milestone-{N}/` 尚不存在，先复制 `docs/milestones/milestone-001/` 目录结构创建它（`spec.md` + `review-log.md`），并在 [docs/milestones.md](../../docs/milestones.md) 登记一行——无需等待人工提前手动准备。`spec.md` 的范围/验收标准应基于人类在启动本次任务时给出的描述来填写，不得自行臆造范围。编号统一用 3 位数字（`milestone-002`、`milestone-003` ...），不要回退到两位数。
1. 逐条对照 `spec.md` 的交付物清单，拆解本 Milestone 的实现任务。
2. 先补齐/编写单元测试（`test/`，命名 `<被测对象>.t.cpp`，如 `pose.t.cpp`），覆盖 Happy Path + 边界/异常分支，确认失败。
3. 实现业务代码（`src/`），严格遵循 cpp-style.md；如 spec.md 要求 benchmark，补充 `bench/bench_*.cpp`。
4. 编译 + 跑测试，确认全绿；如涉及冻结接口变更，先在 [docs/interfaces.md](../../docs/interfaces.md) 登记变更记录。
5. 在 `docs/milestones/milestone-{N}/review-log.md` 追加「Round 0 — Dev Agent 提交」，简要说明实现范围与自测结论。
6. 将 [docs/milestones.md](../../docs/milestones.md) 中本 Milestone 状态更新为「评审中」。

## 完成标准

不需要自行判断"是否达标"——本步骤的完成标准是"已提交给 Review Agent 评审"，实际验收由 Review Agent 在 review-log.md 中给出结论。

## 需要同步的记忆文件

[docs/milestones.md](../../docs/milestones.md)（状态更新，若是新 Milestone 则登记新的一行）、`docs/milestones/milestone-{N}/review-log.md`（新增 Round 0）。
