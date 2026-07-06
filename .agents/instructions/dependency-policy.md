---
applyTo: "CMakeLists.txt,**/*.cmake,package.json,requirements.txt,pyproject.toml,Cargo.toml"
description: "第三方依赖变更提议机制（Dependency Proposal SOP），防止 Agent 擅自引入新依赖"
---

# 依赖变更提议机制（Dependency Proposal SOP）

## 背景

Agent 在遇到难以解决的问题（例如一个复杂的数学/矩阵运算 bug）时，容易倾向于"走捷径"——擅自引入新的第三方库（比如突然加上 `scipy`，或在 C++ 里引入一个庞大的 header-only 库）。这会导致项目依赖膨胀、二进制体积失控，甚至引入开源协议（License）风险。本文件规定 Agent 在依赖变更上的行为边界。

## 硬性规定

- **静默禁止**：Agent 绝对禁止在没有人类明确授权的情况下，修改依赖清单文件（`CMakeLists.txt`、`package.json`、`requirements.txt`、`pyproject.toml`、`Cargo.toml` 等）来新增/替换第三方依赖。
- **提议流程**：当 Agent 判断必须引入新库才能继续时，必须**停止编写代码**，输出一份 `Dependency Proposal`（见下方格式），而不是先斩后奏地改完依赖文件再补说明。
- **人类介入**：只有当人类明确回复"Approve proposal"（或等价的明确批准）后，Agent 才可以执行对应的依赖清单修改。

## Dependency Proposal 输出格式

```markdown
## Dependency Proposal：{库名}

- **动机**：为什么现有代码/已有依赖做不到，必须引入这个库？
- **体积与 License**：这个库的大致体积量级；License 类型，是否与本项目现有 License 兼容？
- **替代方案**：至少给出一个"不引入新依赖，用现有依赖/手写实现"的替代思路，并说明为什么权衡后仍建议引入新库（如果确实建议引入）。

请回复 "Approve proposal" 后我才会修改依赖清单文件。
```

## 与其他规则的关系

本文件与 [build-conventions.md](build-conventions.md) 的关系：build-conventions.md 规定"依赖已经确定要用时，CMakeLists.txt 该怎么写"（必需依赖 REQUIRED、可选依赖 QUIET 优雅降级等）；本文件规定的是"要不要引入一个新依赖"这个决策本身必须经过人类批准，两者不冲突，均需遵守。
