---
mode: agent
description: "可选：为长生命周期/大型项目做阶段性 Milestone 归档小结（只读起草 + 强制人工定稿）"
tools: ["read_file", "grep_search", "file_search", "semantic_search", "list_dir"]
---

## 适用场景

**这是可选能力，不是每个项目都需要**：只有当 `docs/milestones.md` 的扁平列表因为 Milestone 数量积累过多（典型场景：长生命周期、多人团队项目，比如持续维护数年、Milestone 数已到几十上百个）而变得难以阅读时，才由人工在一个自然的阶段边界（如"主力开发转维护"）显式触发本流程。

小型/规模有限的项目（如一个几十个 Milestone 后就基本稳定的工具库）不需要执行本流程，永远使用扁平列表即可。

## 前置阅读

- [docs/milestones.md](../../docs/milestones.md) —— 确认人工指定的归档范围（Milestone 编号区间 + 阶段名称）
- 归档范围内每个 `docs/milestones/milestone-NNN/spec.md` 与 `review-log.md`

## 核心原则（不可违反）

- **只读起草，不做任何破坏性操作**：禁止移动、重命名、删除任何 `docs/milestones/milestone-NNN/` 目录或其中文件。归档只是新增一层摘要视图，原始记录必须保持可独立访问。
- **本流程只产出草案，不自行定稿生效**：草案必须等待人工校对确认后才正式替换 `docs/milestones.md` 中对应内容；在人工确认前，不要覆盖 `docs/milestones.md` 的现有内容。
- 只处理人工明确指定范围内、状态为"已完成"的 Milestone；不得擅自扩大归档范围。

## 标准步骤

1. 确认人工给出的归档范围（Milestone 编号区间）与阶段名称（如"第一年主力开发"），若范围内存在非"已完成"状态的 Milestone，先向人工确认是否要排除它们。
2. 若 `docs/milestones/phases/` 目录不存在，先创建它。
3. 依次读取范围内每个 Milestone 的 `spec.md`（范围/验收标准）与 `review-log.md`（做了什么、关键接口变更、遗留的 📝 建议），为每个 Milestone 起草 3~5 句的浓缩摘要。
4. 将所有摘要汇总写入 `docs/milestones/phases/phase-{N}-{阶段英文短名}.md`，格式建议：

   ```markdown
   # 阶段：{阶段名称}（Milestone {起始编号} ~ {结束编号}）

   > 本文件是该阶段已完成 Milestone 的浓缩摘要，完整记录见各自的 `review-log.md`，本文件不替代原始记录。

   ## milestone-{NNN}：{名称}
   {3~5 句摘要：做了什么、关键接口变更、遗留问题}
   详见 [milestone-{NNN}/](../milestone-{NNN}/spec.md)

   （其余 Milestone 依次列出）
   ```

5. 起草 `docs/milestones.md` 的更新草案（不直接写入，作为本轮输出的一部分展示给人工）：把归档范围内的 Milestone 从列表中收起为一行，如：

   ```markdown
   | 001~050 | {阶段名称} | 已归档 | [docs/milestones/phases/phase-1-xxx.md](milestones/phases/phase-1-xxx.md) |
   ```

6. 将草案（阶段小结文件 + `docs/milestones.md` 更新草案）完整呈现给人工，明确说明"以上为草案，需要你校对确认后我才会正式写入 `docs/milestones.md`"。

## 完成标准

阶段小结草案已产出，且已明确告知人工"需要校对确认"；未经人工确认，不视为归档完成，`docs/milestones.md` 不应被改动。
