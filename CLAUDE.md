# Claude Code 配置（本文件仅做路由，不包含规则正文）

在处理本仓库的任何任务前，请**按顺序**完整阅读：

1. [AGENTS.md](AGENTS.md) —— 仓库唯一索引地图
2. [.agents/rules.md](.agents/rules.md) —— 全局硬红线

Claude Code 支持按目录层级自动合并多个 `CLAUDE.md`（从当前工作目录向上直到仓库根），但本仓库目前只在根目录维护这一份。修改代码前，请根据 [AGENTS.md](AGENTS.md) 中的"目录 → 规则映射表"**主动读取**对应的 `.agents/instructions/*.md`（例如：C++ 对应 `cpp-style.md`，Python 对应 `python-style.md`，CMake 对应 `build-conventions.md`），逐条严格遵循，不要凭经验替代。

如果本次任务是开发/评审某个 Milestone，同样按 AGENTS.md 的说明，主动读取 `.agents/prompts/` 下对应的模板文件并遵循其步骤，不要自由发挥。

> 如后续需要更细粒度的按目录自动加载，可在对应子目录（如 `src/`、`test/`）下新增同名 `CLAUDE.md`，Claude Code 会自动向上合并；新增时请只做路由，不要复制 `.agents/` 下的规则正文。
> 本文件不会随规则内容变化而频繁修改；所有规则的唯一权威来源在 `.agents/` 与 `docs/` 目录下。
