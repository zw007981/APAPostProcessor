---
mode: agent
description: "⚠️ 高危/不可逆操作：彻底清空 docs/milestones.md、docs/milestones/、docs/quality-audits/ 中的全部历史记录，为下一轮开发提供绝对干净的空间。仅限人工显式触发并二次确认后执行。"
---

## 适用场景

当项目要开启一轮与此前完全不同、不再需要延续任何历史 Milestone/质量校验记录的全新开发（例如：推翻重来、更换技术路线、面向新的评审/交付方复用本仓库脚手架），需要把 `docs/milestones.md`、`docs/milestones/`、`docs/quality-audits/` 中积累的全部历史信息彻底清空，恢复到"从未开发过任何 Milestone"的初始骨架状态。

**这是本仓库所有 Prompt 模板中破坏性最强、最不可逆的一个**：一旦执行，被删除的 Milestone 任务书、评审记录、全局质量校验记录将从工作区永久消失（除非依赖 Git 历史回滚）。因此本流程的触发与执行门槛，比只读起草的 [milestone-archive.md](milestone-archive.md) 高得多。

## 核心原则（不可违反）

- **绝不自主触发**：Agent 不得因为"看起来项目该收尾了""历史记录太多了"等自行推断的理由主动发起本流程。本流程只能由人工在对话中明确、显式地要求执行。
- **二次确认硬门槛**：即使人工已经要求执行，Agent 仍必须在真正删除任何文件前，让人工回复完全一致的确认短语 **"确认清空全部历史"**。在收到这句原文确认前，禁止执行标准步骤第 3 步之后的任何删除/覆盖操作，只能展示"即将删除的清单"供确认。
- **删除前必须核实可回滚性**：执行删除前必须先检查 `git status --porcelain` 是否干净（无未提交改动）。若不干净，停止并提示人工先提交或自行处理，不得替人工擅自 `git commit`/`git stash`。工作区干净只代表"改动已入库"，不代表"删除后一定能找回"——如果人工尚未推送到远程或没有其它备份，仍建议人工自行决定是否先打一个 Git tag（如 `pre-reset-{YYYYMMDD}`）作为回滚点；Agent 可以在人工同意后代为创建这个 tag（这是可逆操作），但不能替人工做"是否需要备份"这个决策。
- **范围严格限定**：只清空以下三处，不得波及其它任何文件/目录：
  1. `docs/milestones.md` 的正文内容（重置为空骨架，见标准步骤第 6 步）
  2. `docs/milestones/` 下所有 `milestone-*/` 子目录，以及 `docs/milestones/phases/`（如存在）
  3. `docs/quality-audits/` 整个目录（如存在）

  明确禁止删除或修改：`docs/architecture.md`、`docs/interfaces.md`、`docs/known-limitations.md`、`docs/default_params.md`、`docs/glossary.md`、`docs/NMPC.md`、`docs/visualizer_for_preprocessing_pipeline.md` 等其它设计/记忆类文档，以及 `.agents/`、`AGENTS.md`、`CLAUDE.md` 等规则/路由文件本身——这些是项目的设计资产与流程骨架，不属于"Milestone/质量校验历史记录"。
- **悬空引用需主动模糊化改写，不是只报告**：`docs/known-limitations.md`、`docs/interfaces.md`、`docs/architecture.md`等文件中可能存在指向具体 `Milestone-NNN`/`audit-NNN` 的引用（如"来源：Milestone 004 收尾审计"或指向已删除 `docs/milestones/milestone-NNN/spec.md` 的链接）。这些文件本身承载的是独立于编号的技术知识，删除历史记录后这些引用会变成悬空引用：Agent 应在本次流程中**主动梳理改写**（参见标准步骤第 7 步），而非仅列出清单交给人工。
- **模糊化改写的范围边界**：只针对 [AGENTS.md](../../AGENTS.md) 指定的核心治理文档集合（`docs/architecture.md`、`docs/interfaces.md`、`docs/known-limitations.md`，以及因实现细节涉及而需同步改的 `third_party/*/AGENTS.md`/`design_document.md`、`docs/default_params.md` 等）做改写；**不主动展开** AGENTS.md 指定集合之外、体量巨大且章节结构本身就按 Milestone 轮次组织的独立领域设计文档（如 `docs/NMPC.md` 这类）——这类文档不在 AGENTS.md 开发流程的必读清单中，重写代价远超普通引用清理；若梳理中发现此类文档也大量引用了已删除的编号，只在完成报告中说明规模与位置，交由人工决定是否另行处理。
- **编号重置是本流程的唯一例外**：仓库其它规则（见 [AGENTS.md](../../AGENTS.md)）要求 Milestone/审计编号全局唯一、永不回收，但那针对的是"正常开发过程中个别 Milestone 被废弃"的场景。本流程是人工主动发起的整体清空，执行后下一次开发的 Milestone 编号从 `001` 重新开始、`quality-audits` 编号从 `audit-001` 重新开始，这是本流程唯一被允许触发的编号例外。

## 标准步骤

0. 确认人工确实明确要求执行本流程（而非 Agent 自行提议）；如果只是在讨论"要不要清空"，不要进入后续步骤。
1. 执行 `git status --porcelain`，确认工作区干净；若不干净，停止并提示人工先提交/处理未跟踪改动。
2. 列出即将被删除的完整清单（逐一列举 `docs/milestones/` 下每个 `milestone-*/` 目录、`phases/`（如有）、`docs/quality-audits/` 下每个 `audit-*/` 目录），并附上"是否建议先打 Git tag 备份"的提示，将清单完整展示给人工，明确写出："请回复『确认清空全部历史』以继续，回复其它内容将不会执行任何删除。"
3. 收到人工回复的确认短语原文后，方可继续；若人工同意备份，代为创建一个 Git tag（如 `pre-reset-{YYYYMMDD}`），不得跳过人工确认直接打 tag。
4. 删除 `docs/milestones/` 下所有 `milestone-*/` 子目录与 `docs/milestones/phases/`（如存在）。
5. 删除 `docs/quality-audits/` 整个目录（如存在）。
6. 将 `docs/milestones.md` 重写为空骨架，只保留"状态定义"表格、编号规则等结构性说明文字，清空"Milestone 列表"表格为仅剩表头的空表，并将"当前 Milestone（默认编号指针）"重置为：

   ```markdown
   **当前默认编号：尚未开始新一轮开发，下一个可用编号为 001**
   ```

   删除表格下方所有针对具体已删除 Milestone 编号的历史说明段落（如"Milestone 019~022 是……系列"这类叙述性脚注）。
7. 在"模糊化改写的范围边界"约定的文档集合内（`docs/architecture.md`、`docs/interfaces.md`、`docs/known-limitations.md` 及因实现细节涉及而需同步改的 `third_party/*/AGENTS.md`/`design_document.md`、`docs/default_params.md` 等），检索所有引用了本次删除的具体 `Milestone NNN`/`milestone-NNN`/`audit-NNN` 的位置（含正文提及的编号字样、指向已删除路径的链接、按编号命名的章节标题/表格列），逐条模糊化改写：
   - 去掉具体编号/路径本身，但保留该条记录的技术实质（现象、根因、设计取舍、涉及的文件/函数/配置项），做到脱离编号也能独立读懂；风格类似"经过对 XX 的权衡考虑，本 repo 在 YY（具体文件/函数）采用/实现了 ZZ"；
   - 若原文本身引用了 `docs/NMPC.md` 等未被删除文档的章节号（如"6.8 节"），可以保留，因为这些引用依然有效，不属于悬空链接；
   - 若某个表格列（如"涉及 Milestone"）整列都是编号性质的元信息，直接删除该列，不要留空占位；
   - 检索中若发现"模糊化改写的范围边界"里排除的独立领域设计文档（如 `docs/NMPC.md`）也大量引用了已删除编号，不展开重写，只记录规模与位置留给第 8 步的完成报告。
8. 汇总本次实际删除的文件/目录清单、（如有）创建的备份 tag 名称、第 7 步已模糊化改写的文件清单，以及范围之外未处理的大型文档规模提示，作为完成报告呈现给人工。

## 完成标准

`docs/milestones.md` 已重置为空骨架且不再包含任何具体 Milestone 记录；`docs/milestones/` 下不再存在任何 `milestone-*/`/`phases/` 目录；`docs/quality-audits/` 已不存在；"模糊化改写的范围边界"约定的核心治理文档集合内已不再包含悬空的 Milestone/audit 编号引用；已经完整报告本次删除清单、改写清单与范围之外未处理的大型文档提示，且全过程严格发生在人工回复"确认清空全部历史"之后。
