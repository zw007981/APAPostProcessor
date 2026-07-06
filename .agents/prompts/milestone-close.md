---
mode: agent
description: "确认 Milestone 已收敛并正式关闭"
---

## 适用场景

`docs/milestones/milestone-{N}/review-log.md` 最新一轮结论为 🚨=0 且 ⚠️=0（收敛）后，用于正式关闭该 Milestone。

## 前置阅读

- `docs/milestones/milestone-{N}/review-log.md`（确认最新一轮结论）
- [docs/milestones.md](../../docs/milestones.md)

> 编号 `{N}` 的确定方式：若人工在本次对话中已明确指定 Milestone 编号，以人工指定为准；否则读取 [docs/milestones.md](../../docs/milestones.md) 中「当前 Milestone（默认编号指针）」一节的默认编号。关闭后请同步检查该指针是否需要推进到下一个 Milestone 编号。

## 标准步骤

1. 核对 review-log.md 最新一轮结论确实是 🚨=0 且 ⚠️=0；如不满足，不得执行本模板，应回到 `apply-review-feedback.md` 继续处理。
2. 将 [docs/milestones.md](../../docs/milestones.md) 中本 Milestone 状态更新为「已完成」。
3. 将本轮遗留但未采纳的 `📝 建议` 汇总确认已登记到 [docs/known-limitations.md](../../docs/known-limitations.md)（如尚未登记，在此补充）。
4. 如本 Milestone 涉及 `docs/interfaces.md` 中接口的新增/变更，确认该文件的"已冻结接口清单"与"变更记录"均已同步更新。
5. 如后续 Milestone 依赖本 Milestone 的产出，检查 `docs/milestones.md` 中是否需要解除对应的"前置依赖"阻塞状态。
6. 若本次关闭的正是「当前 Milestone（默认编号指针）」所指向的编号，且下一个待开发的 Milestone 已明确（无前置依赖阻塞），将该指针推进为下一个编号；若下一个 Milestone 尚不明确或存在阻塞，先保持指针不变，等人工确认后再手动推进。

## 完成标准

`docs/milestones.md` 状态已更新为「已完成」，且不存在未登记的遗留问题；「当前 Milestone」指针已按第 6 步确认（推进或保持不变均需明确说明理由）。
