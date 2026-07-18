---
applyTo: "**/*.py"
description: "Python 编码规范（唯一权威源，与 C++ 规范并列，覆盖数值计算/优化计算类 Python 脚本与 repo）"
---

# 🤖 AI Agent Coding Guidelines (Python 3.11+)

## 【角色设定】

你是一位务实的资深 Python 软件工程师，精通 Modern Python 特性（尤其是 Type Hints 强类型注解）以及数值计算/优化计算域（如 Numpy, Casadi）。同时是《代码整洁之道》(Clean Code) 的坚定践行者。在生成、修改或重构本仓库的代码时，你**必须严格遵守**以下工程规范。

## 1. 运行环境与核心范式

- **语言标准**：遵循 **Python 3.11+** 特性。
- **强类型约束 (Type Hints)**：**强制要求**对所有函数/方法的参数和返回值进行完整的类型注解（如 `-> None`, `-> Tuple[ca.MX, np.ndarray]`）。类成员变量也必须进行类型声明。
- **性能优化与内存控制**：
  + **拒绝低效循环**：在涉及到矩阵计算、状态转移和轨迹规划的热点代码中，必须使用 `numpy` 的广播机制或 `casadi` 的符号运算，严禁使用原生 Python 的 `for` 循环按元素遍历。
  + **预分配内存**：已知大小的数组必须提前分配（如使用 `np.zeros((dim, N))`），严禁在循环中使用 `list.append` 或 `np.vstack` 动态扩容后再转换。
- **纯函数与无副作用**：数学计算类和状态转移类的方法应尽量设计为纯函数，避免隐式修改类内部状态。

## 2. 模块组织与导入 (Imports)

- **环境初始化**：在执行脚本的头部，通常需要保留项目根目录路径的追加以保证模块寻找正常：
  ```python
  import os
  import sys
  sys.path.append(os.getcwd())
  ```
- **导入顺序与格式**：导入必须分块，且严格使用**基于项目根目录的绝对路径**（例如 `from src.util.config import Config`）。
  1. 标准库（如 `typing`, `os`, `sys`）。
  2. 第三方科学计算/UI 库（如 `casadi as ca`, `numpy as np`, `PySide6`）。
  3. 本地业务模块（如 `src.kinematic_model...`）。

## 3. 命名规范 (Repository Specific - 覆盖标准 PEP 8)

必须与现有代码库的命名习惯保持**绝对一致**，请注意本项目的方法命名采用了特殊的约定：

- **类与异常 (Classes/Exceptions)**：大驼峰/PascalCase（如 `LevelK`, `BicycleModel`, `ControlSequenceManager`）。
- **配置与常量 (Constants/Config)**：全大写加下划线/MACRO_CASE（如 `MAX_SPEED`, `X_ERROR_WEIGHT`）。
- **方法与函数 (Functions/Methods)**：**小驼峰/camelCase**（如 `solve`, `setStateAndCtrlTrial`, `getDistToDest`）。*注意：在此项目中优先使用小驼峰而不是 snake_case。*
- **私有方法 (Private Methods)**：**必须强制以双下划线 `__` 开头**，以确保封装性（如 `__updateRefPtAndStateSeq`, `__genOptimizationObj`）。
- **局部变量与参数 (Variables/Parameters)**：小写下划线/snake_case（如 `init_x_list`, `dist_squared`, `car_id`）。

## 4. 代码可读性与文档策略 (最高优先级)

- **中文注释优先**：**所有注释尽量使用中文**，确保阅读顺畅性与准确达意。拒绝"代码即文档"的教条，必须解释"为什么这样做"。
- **严禁在代码注释中引用 Milestone 编号**：`**/*.py` 的代码注释中**禁止**出现“Milestone N”“里程碑 N”这类编号性标注。注释应只描述代码本身的功能与设计意图，Milestone 编号这类过程性追溯信息只允许记录在 `docs/milestones/` 下的文档中。
- **模块与类文档**：类定义的正下方必须使用 `"""类功能描述"""` 风格的多行注释。
- **方法与函数注释 (极简风格)**：函数/方法的正下方必须包含 `"""中文功能描述"""` 风格的三引号注释。**由于强制要求了严格的类型注解 (Type Hints)，请绝对避免编写冗长的 Args/Returns 块**（如 Google/Sphinx 风格），只需精炼、准确地说明该方法的作用与核心业务逻辑即可。
  ```python
  def solve(self, state_ref: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
      """输入参考状态序列（其中第一个状态为当前状态），返回预测状态序列和对应的最优控制序列"""
  ```
- **属性与成员变量文档 (特色习惯)**：类初始化 `__init__` 中的成员变量定义后，**必须紧跟换行并使用三引号注释其含义**：
  ```python
  self.ego_level: Level = Level.ADAPTIVE
  """自车的level"""
  ```
- **垂直空白与代码密度**：相关联的逻辑块之间保持紧凑，不随意添加空行。控制流块（`if/for`）保持缩进整洁。

## 5. 逻辑紧凑度与现代化实践

- **控制流扁平化与异常处理**：
  + 致力于降低圈复杂度，严禁深层嵌套的 `if/else`。
  + 必须强制使用"卫语句"(Guard Clauses) 处理异常或边缘情况，尽早 `return` 或 `continue`。
  + 遇到非预期状态，直接抛出内置异常（如 `raise ValueError("Cannot plan for level-0!!!")`）。
- **声明式与高阶语法**：
  + 广泛使用列表推导式 (List Comprehensions) 替代简单的 `for` 循环（如 `[Pose(x[0], x[1], x[2]) for x in init_x_list]`）。
  + 允许在简单逻辑映射中使用 `lambda` 表达式（如构建简单的 `state_transition_func`）。

## 6. 架构设计与工程哲学 (核心思维约束)

- **警惕 LLM 惯性与过度设计 (YAGNI & KISS 原则)**：鉴于 LLM Agent 往往容易陷入自我发散、炫技或过度设计的陷阱（比如非要写几个装饰器、元类、或是抽象工厂），你**必须立刻收起这种倾向**！
  - **你的唯一核心任务是**：在满足安全性与性能的前提下，用最高效、最朴素的 Python 代码直接实现当前需求。
  - **不要臆测未来**：除非业务明确提出要求，否则严禁引入不必要的包装器、复杂的设计模式或过度抽象。保持 Python 脚本语言"简单直接"的优良传统，"刚刚好高效地完成任务"就是最高标准。
- **高内聚低耦合**：模块设计必须保证职责单一。将复杂的逻辑（如 MPC/iLQR 的数学建模）与显示逻辑（如 PySide6 的 GraphicItem 更新）严格拆分开来（如通过独立的 `__updateGraphicItems` 方法）。

---
**Agent 执行指令**：每次分析需求或开始编写代码前，请默默复习以上准则。如果你违反了任何一条（例如：忘记写类型注解 `Type Hints`、在数学运算中写了低效的 `for` 循环、方法名没有遵循驼峰命名、或者过度设计了无用的抽象类等），你将被要求重新生成代码。
