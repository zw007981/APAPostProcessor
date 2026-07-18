# AGENTS.md

> 本文件是所有编程 Agent（GitHub Copilot / Claude Code / Cursor / 其他）进入本仓库后的**第一站**，是唯一的索引地图。
> 各 Agent 的专属入口文件（如 `.github/copilot-instructions.md`）只做路由，不重复本文件及下述规则文件的正文内容。

## 这是什么仓库

本仓库是APA模块路径后处理器的C++实现，主要用于实现多种后处理算法并比较它们在不同场景中的表现。

本仓库的核心开发模式是：**系统设计文档 + 开发计划（Milestone 拆分）先行 → 每个 Milestone 由「Dev Agent」与「Review Agent」两个独立 Agent 会话，通过文件（而非人工转述）迭代收敛完成**。

## 开始任何任务前，必须按顺序阅读

1. [.agents/rules.md](.agents/rules.md) —— 全局硬红线（极简，务必先看，违反会被打回）
2. [docs/architecture.md](docs/architecture.md) —— 系统设计文档：问题定义、设计思路、建模、模块划分
3. [docs/interfaces.md](docs/interfaces.md) —— 核心接口契约（冻结清单），修改核心接口前必须核对
4. [docs/milestones.md](docs/milestones.md) —— Milestone 列表与状态机，定位当前工作对应哪个 Milestone
5. [docs/known-limitations.md](docs/known-limitations.md) —— 已知坑与边界条件，避免重复踩雷
6. 根据你要修改的目录，按下表加载对应的局部规则

## 如果你的任务是"开发/评审某个 Milestone"

不要自由发挥，直接套用 `.agents/prompts/` 下对应模板：

| 你的角色 | 套用模板 | 说明 |
|---|---|---|
| Dev Agent，首次实现 | [.agents/prompts/milestone-implement.md](.agents/prompts/milestone-implement.md) | 读设计文档 + 任务书，TDD 实现；若对应 Milestone 目录尚不存在会自动创建并登记状态 |
| Review Agent | [.agents/prompts/milestone-code-review.md](.agents/prompts/milestone-code-review.md) | 只读审查，五维度评审，结果追加到 review-log.md |
| Dev Agent，处理评审意见 | [.agents/prompts/apply-review-feedback.md](.agents/prompts/apply-review-feedback.md) | 读 review-log.md 最新一轮，处理并回应 |
| 收敛后关闭 | [.agents/prompts/milestone-close.md](.agents/prompts/milestone-close.md) | 更新状态机，登记遗留技术债 |

每个 Milestone 的任务书与评审记录在 `docs/milestones/milestone-NNN/`（`spec.md` + `review-log.md`），登记状态到 `docs/milestones.md`；新增 Milestone 时复制 `milestone-001/` 目录结构、按 3 位数字顺序编号（不要回退到两位数，避免将来破百后需要重命名）——这一步由 Dev Agent 在套用 `milestone-implement.md` 时自动完成，无需人工提前准备。

> 各模板中的 Milestone 编号 `{N}` 默认取 [docs/milestones.md](docs/milestones.md) 顶部「当前 Milestone（默认编号指针）」登记的编号，无需每次在开场白里手动写明；只有当你要临时处理非当前编号的 Milestone 时，才需要在对话中显式指定覆盖。切换到下一个 Milestone 时，只需更新该指针这一处即可。

**战术级别的局部改动**（如优化某个热点函数、把一个大函数拆分成多个小函数）不算新的 Milestone，也不需要单独立项：
- 若会影响所在 Milestone 的范围/验收标准（战略级影响），先更新该 Milestone 的 `spec.md` 记录这次变更，再正常开发。
- 若不影响对外承诺的范围（纯战术执行细节），套用 [.agents/prompts/tactical-change.md](.agents/prompts/tactical-change.md)：Dev Agent 实现 → Review Agent 评审 → （如有问题）Dev Agent 处理，记录追加在所在 Milestone 的 `review-log.md` 里（`战术改动 #k` 小节），**不计入**该 Milestone 的整体收敛判断，也不需要执行 `milestone-close.md`。

**规模化（可选，多数项目不需要）**：如果 Milestone 数量随项目多年演进积累到几十上百个、`docs/milestones.md` 变得难以阅读，人工可以在自然的阶段边界（如"主力开发转维护"）显式触发 [.agents/prompts/milestone-archive.md](.agents/prompts/milestone-archive.md)。该流程只读起草阶段小结（`docs/milestones/phases/*.md`），**不移动/删除任何原始 Milestone 目录**，且必须经人工校对确认后才正式生效。规模有限的项目不需要用到这个能力。

**彻底清空历史（罕见，且不可逆，务必谨慎）**：如果需要让仓库回到"从未开发过任何 Milestone"的绝对干净状态（如推翻重来、更换用途），人工可以显式触发 [.agents/prompts/reset-development-history.md](.agents/prompts/reset-development-history.md)。该流程会**永久删除** `docs/milestones/`、`docs/quality-audits/` 下的全部历史记录并把 `docs/milestones.md` 重置为空骨架，只能由人工显式发起，且必须回复原文"确认清空全部历史"后才会真正执行删除；Agent 不得自行判断"该清空了"而主动发起。

## 目录 → 规则映射表

| 修改路径 | 必读规则 | 说明 |
|---|---|---|
| `src/**/*.{h,hpp,cpp}` | [.agents/instructions/cpp-style.md](.agents/instructions/cpp-style.md) | C++ 编码规范，唯一权威源，逐条严格遵循，不可违反 |
| `test/**` | [.agents/instructions/cpp-style.md](.agents/instructions/cpp-style.md) | 同上；单元测试代码同样适用该规范；测试文件命名 `*.t.cpp` |
| `bench/**` | [.agents/instructions/cpp-style.md](.agents/instructions/cpp-style.md) | 性能压测代码，命名 `bench_*.cpp`，入口 `bench/main.bench.cpp` |
| `CMakeLists.txt`、`**/*.cmake` | [.agents/instructions/build-conventions.md](.agents/instructions/build-conventions.md) | CMake 构建脚本约定 |
| `**/*.py` | [.agents/instructions/python-style.md](.agents/instructions/python-style.md) | Python 编码规范，唯一权威源，适用于脚本与纯 Python repo（如数值计算/优化计算类工具），逐条严格遵循 |
| `CMakeLists.txt`、`package.json`、`requirements.txt`、`pyproject.toml`、`Cargo.toml` 等依赖清单 | [.agents/instructions/dependency-policy.md](.agents/instructions/dependency-policy.md) | 新增/替换第三方依赖前必须先输出 Dependency Proposal，等人类明确批准后才能修改 |

> 说明：`.github/instructions/*.instructions.md` 与 `.github/prompts/*.prompt.md` 均为软链接，指向 `.agents/` 下的同名文件，供 GitHub Copilot 按路径/斜杠命令自动加载，**不是独立副本**，编辑请始终针对 `.agents/` 下的本体文件。

## 遇到反复失败的 Bug/测试用例

同一个具体失败点连续尝试修复达到 3 次仍未解决时，必须停止并套用 [.agents/prompts/debug-circuit-breaker.md](.agents/prompts/debug-circuit-breaker.md)：回退代码到干净状态，输出"尸检报告"（三次尝试分别是什么/核心 Error Trace/怀疑的根因），等待人类介入，禁止继续自行尝试第 4 次。这条规则适用于任何任务场景，不局限于 Milestone 开发。

## 其他 Agent 的适配文件

除 GitHub Copilot 外，本仓库还为以下 Dev Agent 预留/提供了适配文件：

| Agent | 适配文件 | 说明 |
|---|---|---|
| Kimi | [.kimirules](.kimirules) | 根级薄路由，工具不支持按路径自动加载局部规则，需主动读取 |
| GLM / CodeGeeX | [.codegeexrules](.codegeexrules) | 同上 |
| Claude（Claude Code 官方 CLI/编辑器） | [CLAUDE.md](CLAUDE.md) | 根级薄路由；该工具原生支持按目录层级合并多个 CLAUDE.md，后续可在子目录按需新增 |
| ChatGPT（OpenAI Codex 官方 CLI/插件） | 复用 [AGENTS.md](AGENTS.md) 本身 | 该工具原生按 `AGENTS.md` 这一事实标准加载，无需额外的专属文件；如后续该工具改变约定，再单独适配 |

后续如需适配更多 Agent，按同样的"薄路由 + 主动读取"模式新增对应的根级配置文件即可，无需改动 `.agents/` 下的规则正文。

## 开发闭环的基本要求（适用于所有任务）

- 先写/更新测试，确认失败，再修改实现，确认通过（TDD）。
- 编译通过、测试全绿、无新增静态检查告警后才算完成。
- 任务结束后更新 `docs/milestones.md`；如发现新的坑或边界条件，补充到 `docs/known-limitations.md`。
