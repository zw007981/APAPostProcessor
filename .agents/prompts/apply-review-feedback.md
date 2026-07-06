---
mode: agent
description: "消费 Review Agent 产出的评审报告并处理"
---

## 适用场景

Dev Agent 收到 `docs/milestones/milestone-{N}/review-log.md` 的新一轮评审意见后，需要处理其中问题时使用。

## 前置阅读

- `docs/milestones/milestone-{N}/review-log.md` —— **只处理最新一轮**，历史轮次仅作背景参考
- `docs/milestones/milestone-{N}/spec.md`
- [.agents/instructions/cpp-style.md](../instructions/cpp-style.md)

> 编号 `{N}` 的确定方式：若人工在本次对话中已明确指定 Milestone 编号，以人工指定为准；否则读取 [docs/milestones.md](../../docs/milestones.md) 中「当前 Milestone（默认编号指针）」一节的默认编号。

## 处理优先级（强制）

- `🚨 阻断`：必须修复，不可协商。
- `⚠️ 严重`：必须修复；如有不同意见，需在回应中说明理由，不能静默忽略。
- `📝 建议`：可自行判断是否采纳；不采纳需简要说明原因。

## 标准步骤

1. 逐条对照最新一轮意见，标记处理状态（已修复 / 不采纳+理由）。
2. 针对每处修改，先确认或补充对应的单元测试（`test/*.t.cpp`）。
3. 全量回归测试，确认编译通过、测试全绿。
4. 若某条意见反映的是 `docs/architecture.md`/`docs/interfaces.md` 本身的设计缺陷（而非代码实现问题），先更新对应设计文档并登记变更记录，再改代码。
5. 若某条 `📝 建议` 不采纳，且属于值得记录的技术债，补充到 [docs/known-limitations.md](../../docs/known-limitations.md)。
6. 在 `docs/milestones/milestone-{N}/review-log.md` 追加本轮回应（`## Round {N} — Dev Agent 回应`），逐条列出处理结果，并请求下一轮复审。

## 完成标准

回应已追加到 review-log.md，且回归测试全绿；下一步交给 Review Agent 复审，不由 Dev Agent 自行判断是否收敛。
