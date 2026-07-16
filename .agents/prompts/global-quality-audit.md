---
mode: agent
description: "所有/一批 Milestone 完成后，对整个代码库做一次宏观质量校验（含 Review/Dev 两阶段）"
---

## 适用场景

一批 Milestone（通常是 `docs/milestones.md` 中已标记「已完成」的连续区间，也可以是全部 Milestone）已经分别收敛之后，用于对整个代码库做一次跨 Milestone 的宏观复核。与逐个 Milestone 的 Code Review 不同，本流程不对某一个 Milestone 的 diff 负责，而是对**整个代码库当前状态**的架构一致性、接口一致性、跨模块重复度、全局构建/测试健康度、技术债务收敛情况负责。

> 触发时机由人工判断，不需要每个 Milestone 关闭后都执行——建议在一个自然的阶段边界触发（如"某个功能大类的相关 Milestone 全部完成""发布前的最终体检""距离上一次全局校验已经过了较长时间/较多 Milestone"）。

## 前置阅读

- [docs/architecture.md](../../docs/architecture.md)
- [docs/interfaces.md](../../docs/interfaces.md)
- [docs/milestones.md](../../docs/milestones.md) —— 确认本次校验覆盖哪些「已完成」的 Milestone
- [docs/known-limitations.md](../../docs/known-limitations.md)
- [.agents/instructions/cpp-style.md](../instructions/cpp-style.md)
- 本次实际操作的 `docs/quality-audits/audit-{N}/`（编号由下方规则自动确定）下的 `scope.md`/`review-log.md`（如果不是首次发起）—— 通读历史轮次

## 编号确定规则（自动识别，无需人工每次手动指定）

与 Milestone 开发流程中的「当前 Milestone 默认编号指针」类似，本 Prompt 的 `audit-{N}` 编号同样不需要人工在对话中手动写明，Agent 应自行判断：

- **新起一轮全局质量校验**（即将执行第一阶段的"标准步骤 1"）：`{N}` = `docs/quality-audits/` 下已有最大编号 + 1；该目录尚不存在时从 `audit-001` 开始。人工只需说明本次触发原因与覆盖范围，不需要指定编号。
- **续接处理/复核一次已经发起的校验**（第二阶段 Dev Agent 处理意见、或回到第一阶段由 Review Agent 再次复核）：默认操作对象是 `docs/quality-audits/` 下**编号最大**的目录（即最近一次发起、大概率尚未收敛的那次校验），不需要人工指定路径。
- 只有当人工明确要求"重新开始新一轮校验"（即使上一次编号尚未收敛）或"针对更早的历史 audit 目录"操作时，才需要在对话中显式指定编号，显式指定优先于以上自动规则。

---

## 第一阶段：Review Agent 宏观审查（只读）

### 标准步骤

1. 按上方"编号确定规则"确认本次校验的编号 `audit-{N}`（Round 0 首次发起时为新编号；Round {k} 复核时沿用已发起的最大编号目录），以及本次覆盖的 Milestone 范围（默认是全部「已完成」的 Milestone；如人工指定了子集，以人工指定为准）。
2. 若 `docs/quality-audits/audit-{N}/scope.md` 尚不存在（即本次是 Round 0 首次发起），新建该文件，写明：本次覆盖的 Milestone 编号区间、触发原因（人工给出）、校验日期；若已存在（即本次是复核 Round {k}），直接沿用，无需重新创建。
3. 对 `src/`、`test/`、`bench/` 的当前整体状态（而非某一次 diff）按以下七个维度展开审查：
   1. **代码逻辑正确性**：核心算法与控制流是否存在逻辑错误、边界条件遗漏、未定义行为或潜在 Bug？跨 Milestone 的修改是否引入了不一致的假设或状态机冲突？
   2. **计算效率**：关键路径（热循环、频繁调用的函数、数值优化求解器交互等）是否存在不必要的拷贝、重复计算、低效数据结构或冗余内存分配？跨 Milestone 的增量修改是否在无意中劣化了整体性能？
   3. **架构一致性**：当前代码是否仍然符合 [docs/architecture.md](../../docs/architecture.md) 的设计意图？是否存在多个 Milestone 各自演进导致的架构漂移或职责错位？
   4. **接口契约完整性**：[docs/interfaces.md](../../docs/interfaces.md) 中冻结清单的签名/语义是否与代码实际实现一致？是否有不同 Milestone 对同一接口做了不一致的扩展？
   5. **跨模块重复与耦合**：不同 Milestone 引入的代码之间是否存在重复实现、不必要的循环依赖或耦合？是否有可以抽取的公共逻辑？
   6. **全局构建与测试健康度**：全量编译（仓库内全部 CMake target）与全量测试套件（含 `bench/` 若适用）是否通过；核心模块的测试覆盖是否存在明显盲区。
   7. **技术债务收敛情况**：[docs/known-limitations.md](../../docs/known-limitations.md) 中登记的历史坑是否有可以借这次机会一并解决的；是否发现了只有站在跨 Milestone 视角才能看到的新问题，需要补充登记。
4. 问题分级：`🚨 阻断` / `⚠️ 严重` / `📝 建议`，与 Milestone 级评审一致；每条问题必须指出具体文件/函数，并说明是由哪些 Milestone 交叉导致的。
5. 在 `docs/quality-audits/audit-{N}/review-log.md` 追加 `## Round 0 — 日期 — Review Agent`（首次）或 `## Round {k} — 日期 — Review Agent`（复审），结尾写明"结论：🚨=x，⚠️=y，是否收敛"。

### 完成标准

`scope.md` 与 `review-log.md` 均已落盘；不自行判断是否收敛，交给人工决定是否进入第二阶段。

---

## 第二阶段：若有 🚨/⚠️，Dev Agent 跨 Milestone 处理

### 处理优先级（与 [apply-review-feedback.md](apply-review-feedback.md) 一致）

- `🚨 阻断`：必须修复，不可协商。
- `⚠️ 严重`：必须修复；如有不同意见，需在回应中说明理由，不能静默忽略。
- `📝 建议`：可自行判断是否采纳；不采纳需简要说明原因。

### 标准步骤

0. 按上方"编号确定规则"定位本次实际操作的 `docs/quality-audits/audit-{N}/`（默认是编号最大的现有目录），不需要人工指定路径。
1. 逐条对照 `review-log.md` 最新一轮意见，标记处理状态；由于问题可能跨越多个 Milestone 目录，修改时允许一次性跨多个 `src/`、`test/` 文件修复。
2. 每处修改先确认/补充对应单元测试，确保修复有回归保护。
3. 全量回归测试，确认编译通过、测试全绿。
4. 若某条意见反映的是 `docs/architecture.md`/`docs/interfaces.md` 本身的设计缺陷，先更新设计文档并登记变更记录，再改代码；如涉及某个具体 Milestone 的 `spec.md` 范围变化，同步更新对应 `spec.md`。
5. 若某条 `📝 建议` 不采纳，补充理由到 [docs/known-limitations.md](../../docs/known-limitations.md)。
6. 在 `docs/quality-audits/audit-{N}/review-log.md` 追加 `## Round {k} — Dev Agent 回应`，逐条列出处理结果，并请求 Review Agent 再次执行第一阶段复核。

### 完成标准

回归测试全绿，回应已追加；回到第一阶段由 Review Agent 复核，直至最新一轮 🚨=0 且 ⚠️=0。收敛后在 `scope.md` 顶部登记"本次全局质量校验已通过，日期：xxx"，不需要执行 `milestone-close.md`（那是针对单个 Milestone 的收尾动作，与本次跨 Milestone 校验无关）。
