# 全局硬红线（任何任务都不可违反）

> 本文件只做"触发器"，不展开细节。命中以下任一场景时，必须先去读对应的完整规则文件，再动手改代码。

- 🚫 修改 `src/**`、`test/**` 或 `bench/**` 下的 C++ 文件前，必须先完整阅读 [.agents/instructions/cpp-style.md](instructions/cpp-style.md)，逐条遵循，不可自行简化。
- 🚫 修改/编写任何 `**/*.py` 文件前，必须先完整阅读 [.agents/instructions/python-style.md](instructions/python-style.md)，逐条遵循，不可自行简化（含强制 Type Hints、camelCase 方法命名等本仓库特有约定）。
- 🚫 严禁在 `src/**`、`test/**`、`bench/**` 或任意 `**/*.py` 的代码注释中引用“Milestone N”/“里程碑 N”等编号（如 `// Milestone 019 —— ...`）；此类过程性追溯信息只允许写在 `docs/milestones/` 下的文档中，代码注释必须能脱离编号独立读懂。
- 🚫 禁止裸指针 `new`/`delete`，必须使用 RAII / 智能指针管理资源。
- 🚫 数据规模可预知时，禁止使用未提前 `reserve` 的 `std::vector`。
- 🚫 禁止跳过测试直接提交：先写/改测试用例，确认失败，再改实现，确认通过。
- 🚫 禁止在没有阅读 [docs/known-limitations.md](../docs/known-limitations.md) 的情况下对已知问题重复"重新发明修复方式"。
- 🚫 [docs/interfaces.md](../docs/interfaces.md) 中登记的核心接口一经冻结，禁止在未登记变更记录的情况下擅自修改签名。
- 🚫 Review Agent（套用 `milestone-code-review.md` 时）只做只读审查，禁止直接修改任何代码/文档文件，意见只能写入对应 Milestone 的 `review-log.md`。
- 🚫 Dev Agent 不得自行宣布某个 Milestone "已完成"：唯一判定标准是该 Milestone `review-log.md` 最新一轮结论中 🚨 与 ⚠️ 数量均为 0，且已执行 `milestone-close.md`。
- 🚫 战术级别的局部改动（如优化某个热点函数）不得擅自包装成一个新的 Milestone：若有战略级影响（改变范围/验收标准），必须先更新所在 Milestone 的 `spec.md`；若无战略级影响，套用 [.agents/prompts/tactical-change.md](prompts/tactical-change.md) 在所在 Milestone 内处理。
- 🚫 `review-log.md` 中的"战术改动 #k"结论不得与"Round N"混淆：前者只影响该项战术改动本身，不计入 Milestone 整体收敛判断，不能作为执行 `milestone-close.md` 的依据。
- 🚫 套用 [.agents/prompts/milestone-archive.md](prompts/milestone-archive.md) 做阶段归档时，禁止移动/重命名/删除任何 `docs/milestones/milestone-NNN/` 原始目录或文件，只能新增 `docs/milestones/phases/*.md` 摘要；且归档结果必须等人工校对确认后才能写入 `docs/milestones.md`，不得自行定稿生效。
- 🚫 [.agents/prompts/reset-development-history.md](prompts/reset-development-history.md)（彻底清空全部 Milestone/质量校验历史）禁止由 Agent 自行发起；执行前必须确认工作区 git 状态干净，且必须等人工回复原文"确认清空全部历史"后才能删除 `docs/milestones/`、`docs/quality-audits/` 下的任何文件；清空范围仅限该模板明确列出的三处，不得波及 `docs/architecture.md`/`docs/interfaces.md`/`docs/known-limitations.md` 等其它设计/记忆文档。
- 🚫 禁止在没有人类明确批准（回复"Approve proposal"）前修改依赖清单文件（`CMakeLists.txt`/`package.json`/`requirements.txt`/`pyproject.toml`/`Cargo.toml` 等）新增/替换第三方依赖：必须先按 [.agents/instructions/dependency-policy.md](instructions/dependency-policy.md) 输出 Dependency Proposal，等待批准后才能动手改依赖清单。
- 🚫 同一个 Bug/测试用例连续尝试修复达到 3 次仍失败时，必须立即停止，套用 [.agents/prompts/debug-circuit-breaker.md](prompts/debug-circuit-breaker.md) 回退代码并输出尸检报告，禁止继续第 4 次尝试。
- ⚠️ Dev Agent 每处理完一轮评审意见，必须在对应 `review-log.md` 中逐条回应（已修复/不采纳+理由），不能只改代码不留痕迹。
- ⚠️ 每完成一个任务，必须更新 [docs/milestones.md](../docs/milestones.md)（若该文件已建立）。
