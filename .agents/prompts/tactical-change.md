---
mode: agent
description: "在已有 Milestone 内完成一次战术级改动（不影响范围/验收标准），含 Dev/Review 两阶段"
---

## 适用场景

某个 Milestone 已经存在（开发中或已完成），出现一个**不在 `spec.md` 交付物清单里、且不影响该 Milestone 范围/验收标准**的战术级改动需求（如"优化基于栅格地图构建 ESDF 地图的速度"、"把某个大函数拆分成多个小函数"）。这类改动**不新建 Milestone**，直接在所在 Milestone 内完成。

> 如果这次改动其实会影响所在 Milestone 的范围/验收标准（战略级影响），不要套用本模板——应先由人工更新该 Milestone 的 `spec.md`（必要时同步 `docs/architecture.md`/`docs/interfaces.md`），再按 `milestone-implement.md` 正常处理。

## 前置阅读

- `docs/milestones/milestone-{N}/spec.md` —— 确认这次改动确实不影响已登记的范围/验收标准
- `docs/milestones/milestone-{N}/review-log.md` —— 了解该 Milestone 已有的历史记录，并确认这次战术改动的编号（`战术改动 #k`，取已有最大编号 + 1）
- [.agents/instructions/cpp-style.md](../instructions/cpp-style.md)

> 编号 `{N}` 的确定方式：若人工在本次对话中已明确指定 Milestone 编号，以人工指定为准；否则读取 [docs/milestones.md](../../docs/milestones.md) 中「当前 Milestone（默认编号指针）」一节的默认编号。

---

## 第一阶段：Dev Agent 实现

### 标准步骤

1. 确认这次改动的具体范围和验收标准以人类在启动任务时给出的描述为准（如"基于栅格地图构建 ESDF 地图的耗时从 X ms 降到 Y ms，现有测试保持通过"），不得自行扩大范围；如果发现范围其实会波及 spec.md 的既有承诺，停下来提醒人类，不要继续套用本模板。
2. 先补齐/更新单元测试，覆盖改动前后的行为差异，确认失败（若是纯重构类改动，先确保重构前的行为已被现有测试覆盖，作为回归基线）。
3. 实现改动，严格遵循 cpp-style.md。
4. 编译 + 跑测试，确认全绿。
5. 在 `docs/milestones/milestone-{N}/review-log.md` **追加**一个新的战术改动小节（不要新建文件，不要复用/覆盖已有 `Round` 或已有战术改动编号）：

   ```markdown
   ## 战术改动 #k — {一句话描述改动} — Dev Agent 提交
   （改动动机、实现方式、验证方式简述）
   ```

### 完成标准

不自行判断"是否达标"，提交给 Review Agent 评审。

---

## 第二阶段：Review Agent 评审（只读）

### 标准步骤

沿用 [milestone-code-review.md](milestone-code-review.md) 的五个审查维度，但审查对象只是这次战术改动涉及的代码变更，不是整个 Milestone；其中"需求对齐"维度改为核对"这次改动是否确实没有超出人类给出的范围、且没有影响 spec.md 已登记的验收标准"。

在同一个战术改动小节下追加结论：

```markdown
## 战术改动 #k — Review Agent 评审
（五维度意见，标注 🚨 阻断 / ⚠️ 严重 / 📝 建议）
结论：🚨=x，⚠️=y。
```

**关键区别**：本次结论**不计入**该 Milestone 整体的收敛判断，不影响 `milestone-close.md` 的执行条件——它只针对这一项战术改动本身是否可以收尾。

---

## 第三阶段：若有 🚨/⚠️，Dev Agent 处理

沿用 [apply-review-feedback.md](apply-review-feedback.md) 的处理优先级（🚨 必须修复、⚠️ 必须修复或说明理由、📝 可自行判断），在同一战术改动小节下追加回应，直至该项 🚨=0 且 ⚠️=0。

收敛后**无需执行** `milestone-close.md`——那是给整个 Milestone 用的，这次只是其内部的一项战术改动，Milestone 本身的状态不受影响。
