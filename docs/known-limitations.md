# 已知坑与边界条件

> 记录调试过程中踩过的坑、设计上的已知限制、Review 中未采纳的 📝 建议等，避免不同 Agent/不同 Session 重复踩雷。

## 记录格式约定

每条记录建议包含：现象、根因、当前应对方式、是否有后续解决计划、来源（哪次调试/评审）。

## 记录

### 两阶段"信赖域硬约束改软代价"计划完整执行后仍未实现机动段削减（已被六次重构推翻，历史记录保留）

- **现象**：用户设计的两阶段计划——第一阶段把位置信赖域从硬约束改为软代价跟踪（`docs/NMPC.md` 6.8 节），第二阶段把航向信赖域同样改为软代价并默认弃用静态走廊（`docs/NMPC.md` 6.9 节）——均已完整执行，理论分析在两个阶段都被合成场景单元测试证实成立（软约束确实能让 SQP 探索硬约束下不可达的解，如 65.5% 长度压缩），但在四个真实数据集（data1/data3/data6/data7）上，**两轮重构均未实现哪怕一个机动段的削减**。
- **根因（六次重构才定位到）**：问题不在信赖域是硬是软，而在于此前从二次重构起就引入、四/五次重构中只是软化但从未质疑过的 $J_{process}$ —— 该代价项持续跟踪参考轨迹 $Z_{ref}$ 本身，而 $Z_{ref}$ 恰恰就是包含冗余换挡的粗糙路径，等价于持续奖励"保持冗余换挡的原始几何形状"，与"熔化冗余换挡"目标方向相反。参考 Zhang et al.(Sci Rep 2025) 论文的代价函数设计后发现：该论文的粗路径只提供 warm start 初值，优化开始后代价函数里根本没有对粗路径的持续跟踪项。
- **已排查过的错误方向**：权重量级排查（10→1e3→1e4）两轮均无变化；`max_iter` 翻倍（300→600）结果完全不变（排除迭代预算不足）——这些排查在"信赖域硬/软"这个维度上是正确的，但问题根本不在这个维度，而在 $J_{process}$ 这个此前从未被质疑过的代价项本身。
- **解决方案**：默认关闭 $J_{process}$（`position_tracking_weight`/`theta_tracking_weight` 默认改为 0.0），新增全程目标牵引代价 $J_{target}$（对每一步施加向**常量**停车目标的二次牵引，而非向粗参考轨迹），四数据集广泛调参后 `data1`/`data3` 均达成"至少削减 3 个机动段"验收目标，详见 `docs/NMPC.md` 6.10 节。
- **来源**：经过对该问题的多轮重构考虑，本 repo 依次在 `docs/NMPC.md` 6.8 节、6.9 节记录的重构中排查，最终在 6.10 节记录的重构中给出解决方案。

### 位置跟踪软代价上线后，`data7` 终点精度出现回归（已在六次重构中彻底解决）

- **现象**：经过对可行域灵活性的考虑，本 repo 在一轮重构中把位置信赖域从硬约束改为 HPIPM 原生软约束（纯 L2 二次跟踪代价，详见 `docs/NMPC.md` 6.8 节）后，`data7.json` 的终点位置误差从此前 <0.07m 恶化到 0.347m、航向误差恶化到 3.24°（均超出质量门）。后续一轮重构后收窄到 0.118m/0.07°（仅位置误差仍超标）。
- **根因与解决**：六次重构定位到真正根因是 $J_{process}$（持续跟踪包含冗余换挡的粗参考轨迹）本身与新增的终端/全程目标代价产生数值竞争；默认关闭 $J_{process}$（`position_tracking_weight`/`theta_tracking_weight` 改回 0.0）并改用全程目标牵引代价（`global_target_position_weight`=0.001）后，`data7` 终点位置误差回到 0.0000m、航向误差 0.00°，完全达标。

### `data6`/`data7` 在六次重构后仍未达成"至少削减 3 个机动段"目标（新记录，非本轮回归）

- **现象**：经过对全程目标牵引代价的设计考虑，本 repo 在一轮重构中（详见 `docs/NMPC.md` 6.10 节）在 `data1`/`data3` 上均达成"至少削减 3 个机动段"验收目标，但 `data6`（6→6，无变化）与 `data7`（最好 6→5，仅 -1）仍未达标。四数据集共 14 组权重（`global_target_position/heading_weight` 从 0.001 到 100 不等，含 `interior_speed_weight` 交叉扫描）与更高迭代预算（`max_iter=600`）对这两个数据集均无实质改善。
- **根因（已诊断，非全新问题）**：`data6` 从相关重构立项之初就被诊断为"无冗余换挡几何"的不同问题类别——其最后 3 个机动段是贴着终点车位右侧墙体（净空约 1.5m）做的真实航向修正（K 字形三点调头，航向从约 118° 连续修正到 82.5°），删除其中任意一段都会导致车辆以错误航向停在错误位置，不存在可直接裁剪的冗余重复段。`data7` 的揉库串几何诊断（`docs/NMPC.md` 6.8.1 节）早已计算出其所需偏移量远超 `data1`，本轮牵引代价在合理权重范围内不足以提供这个量级的"熔化压力"；继续加大权重（20/50/100）已验证无效，且部分权重（0.1）在 `data1` 上会引入真实的 HPIPM 求解失败风险，不是安全的通用方向。
- **`data6` 精确失败信号（2026-07-16 复测确认）**：实测日志显示 NMPC 在 SQP 第 2 轮外层迭代时，`StcSQP: QP solve failed with status code:1`——对应 `third_party/StcSQP/src/qp/qp_solver.h` 的 `QPSolverStatus::MAX_ITER_REACHED`（HPIPM 内点法未在其硬编码的 `iter_max=1000` 次迭代预算内收敛），而非 `INFEASIBLE`(2) 或历史记录的 `UNKNOWN_ERROR`(5)。`PostProcessor` 的兜底行为正常：`success=true`，安全回退到预处理轨迹，不是崩溃。
- **`data6` QP 病态专项排查结论（2026-07-16，临时插桩诊断，已清理不留代码）**：在 `SQPSolver::solveQP()` 临时插入诊断代码，逐步导出每一步普通约束的 `d`（约束余量，`d<0` 表示当前线性化点已违反）与动力学残差 `b` 的模长，定位到：
  1. **`[C D]` 条件数在单步内恒为无穷大是结构性假象，与病态无关**——碰撞约束（`IterativeCorridorConstraint`）只依赖 x/y/theta 三个状态分量，`D`（控制 Jacobian）恒为 0，限制在真正非零的列上重新计算后，绝大多数步的条件数是合理的个位数到两位数。
  2. **iteration 0（原始 warm start）完全无违反**——B样条平滑+碰撞预推产出的预处理轨迹本身是无碰撞的，问题出在 SQP 优化过程本身。
  3. **iteration 0→1 的第一次全步长牛顿更新，把轨迹大范围推入实质性碰撞违反**（多处 `d_min` 低至 -0.4~-0.7m，涉及步数远多于最终阶段），到 iteration 2 时收敛/回退到一个更集中但依然真实存在的违反区间：步 211~221（共 11 步）连续违反 -0.12~-0.78m，对应的世界坐标从 (-4.06,0.27) 斜向下延伸到 (2.54,-6.48)，直接穿过地图中央一片此前从未纳入分析范围的密集障碍物簇（约 300+ 个占据栅格，`x∈[-3,3], y∈[-7,-3]`）——这片障碍物正是 Hybrid A* 最初绕一个大圈（M0/M1/M2）刻意避开的区域；NMPC 在移除了此前重构引入的"贴合参考轨迹"信赖域/软代价后，中段轨迹完全失去了"沿原路线走"的约束力，只剩 `J_effort`/`J_smooth`/全程目标牵引这些"求直求短"的代价，会自然地把中段往地图中央捷径拉，直接撞上这片障碍物簇。
  4. **已排除的假设**：（a）该违反与后续新增的 `global_target_position_weight` 无关——把该权重临时改为 0（等价于关闭全程目标牵引，只保留 `J_effort`/`J_smooth`）复测，违反区间/数值几乎完全相同（步 211~221，误差在小数点后 3~4 位内一致），说明就算没有目标牵引代价，中段轨迹依然会被"求直/求短"的其它代价拉向该障碍物簇；（b）临时开启 `use_line_search=true` 不能解决，反而在 iteration 0 就以另一种失败模式退出（`QP directional derivative positive, not a descent direction`，与 `nmpc_solver.h` 现有注释"开启后首迭代不可行"一致，属已知的另一权衡，并非本次新发现）。
  5. **初步结论**：这更像是 SQP 全局化（globalization）问题——移除信赖域约束、默认关闭线搜索后，长视野（336 步）、强非凸（大范围回环+终点大角度修正）问题的首次全步长牛顿更新缺乏阻尼，容易把轨迹中段甩出到远离原参考路径的区域，一旦甩到障碍物密集区就形成需要跨多步协同修正的真实碰撞违反，而走廊是硬约束，HPIPM 必须精确满足，此类跨步耦合的大幅修正在 1000 次内点迭代内难以收敛。真正的修复需要重新设计 SQP 的步长阻尼/信赖域机制（例如按步长比例回退、或对长视野问题分段降级为更保守的更新策略），而不是调整现有的软代价权重，工作量与风险都明显超出"调参"范畴，本次专项排查到此为止，留待用户决策是否投入。
- **（2026-07-16）四轴独立实验交叉验证，锁定"非代价权重可修复"结论**：先复核确认 `data6` 真实、可复现的失败模式是**第 2 次迭代 HPIPM QP 求解失败**（连续 5 次重跑 100% 一致；此前一次"iteration 0 not a descent direction"的观察经排查确认是终端 scrollback 缓冲区混杂历史命令输出造成的假象，非真实行为，`options_.use_line_search` 运行时确认为 `false`）。随后代码走读确认 `third_party/StcSQP` 当前**完全没有自适应正则化/信赖域机制**（`SQPSolverOptions::reg_min` 仅在某 stage 完全无代价时才作为结构性兜底生效，`reg_max`/`reg_factor` 声明但从未被引用）。为验证"缺乏步长阻尼"假说，新增 `SQPSolverOptions::hessian_regularization`（默认 0.0，无条件叠加到每个 stage 的 Q/R 对角，`assembleCostImpl()` 实现）并扫描 `1e-4~10`。连同此前已完成的走廊软约束二次项权重（Zu）、静态舒适走廊软约束、ESDF 直接引导代价三组独立实验，**四个独立维度全部对 `data6` 零效果**（`max_intrusion_depth`/机动段数/失败迭代号完全不变），且 Hessian 正则化在多数取值下反而让 `data1` 从 10→7 退化为 10→9（不满足"其它数据不退化"前提）。**结论：`data6` 的失败不是任何代价函数权重或步长阻尼参数能修复的问题**；`sqp_hessian_regularization` 代码保留（默认 0.0，不采纳为默认改动）。当时推测根因"大概率是 `IterativeCorridorConstraint` 梯度查询退化/多圆共线导致约束子块秩亏"——**该推测已在后续实地验证中被证伪，见下**。
- **（2026-07-16）实地验证上述两个候选假设，均被证伪**：在 `IterativeCorridorConstraint::computeCircleConstraint` 临时加环境变量门控诊断（逐圆打印 global_step/dist/grad/g_val/完整 3 维 `a_row`），强制串行 linearize（`short_n_threshold=1000`）+ `max_iter=3` + 禁用重试，只保留 iteration 0/1/2 三轮完整线性化日志，排查后已完整清理（`grep CORRIDOR_DIAG|TEMP DIAG src/` 零匹配，全量测试560/561通过）。结果：
  1. **梯度真退化假设证伪**：真实违反窗口（步 205~225）内，12 个圆在所有步上 `valid=true`，**零退化行**——梯度退化（越界或梯度范数 < 1e-12）根本没有发生在这个关键区域；
  2. **多圆共线假设证伪，且此前"cond(C_active)高达3~8万"的分析方法有缺陷**：只用 2D 梯度算逐圆两两夹角确实测得违反窗口内大量接近 0° 的夹角，但这遗漏了 `a_row` 真正参与 QP 的第三分量——航向 `theta` 偏导（每个圆的力臂不同，该分量并不相同）。补上这一维用完整 3 维 `a_row` 重新做 SVD：**违反窗口内每步 12×3 约束子块秩恒为 3（满秩），条件数只有个位数到约 55**，完全良性，不构成数值病态；
  3. **新发现（未完全定位）**：两个假设都被排除后，说明病态不是任何单一 stage 局部约束块的秩亏问题，更可能是**跨越 211~221 这 11 个连续耦合 stage 的全局/时序层面现象**——这些步的约束方向模式持续相似、且都携带真实违反（`g_val` 0.12~0.67），这种"长时间窗口持续、方向高度相关的真实违反"可能影响 HPIPM 沿时间轴 Riccati 递推的数值稳定性，但已超出约束线性化单步打印能诊断的范围，需要 HPIPM/StcSQP 内部 IPM 迭代级别诊断（如逐次内点迭代对偶间隙/步长/正则化触发情况）才能进一步定位。
  4. **教训**：分析约束矩阵"共线性/秩亏"必须使用完整的、真正参与 QP 的行向量（含所有非零列），只看部分分量会得出误导性结论。
- **当前应对方式**：暂无进一步的 NMPC 内部调参计划（14 组权重扫描 + 插桩排查 + 实地验证已充分排查，继续在代价权重维度盲目调参边际收益存疑，按 debug-circuit-breaker 精神停止）。若要继续推进，方向是 HPIPM/StcSQP 内部 IPM 迭代级别诊断，或跨多 stage 耦合的数值稳定性专项分析（工作量明显更大，需按依赖变更流程评估），而非继续调权重；`iter_max=1000` 是 `third_party/StcSQP` 内部硬编码值，单独调大大概率无效（本身已经很宽裕）。留待用户决策：是否接受 `data6`/`data7` 在当前架构下的能力边界，还是投入上述更深层的 SQP/HPIPM 内部诊断或 warm start/预处理层拓扑合并等方案。
- **来源**：本 repo 在 `docs/NMPC.md` 6.10.4/6.10.5 节记录的一轮重构中排查；`data6` 精确状态码诊断与插桩专项排查均为 2026-07-16 用户复测触发的补充调查；四轴交叉验证、实地验证同为 2026-07-16。

### `IterativeCorridorConstraint`/`StaticCorridorLinearConstraint` 硬编码状态维度为 5，状态增广后触发维度不匹配（已修复）

- **现象**：经过对跨 stage 顺滑代价的实现考虑，本 repo 在一轮重构中把 NMPC 状态从 5 维（`[x,y,theta,v,delta]`）增广为 7 维（新增 `a`、`ddelta` 状态分量，见下一条记录）后，四数据集调参工具与部分单元测试立即报错 `constraint linearization received invalid dimension output`：`IterativeCorridorConstraint::evaluateAndJacobian/jacobian`（ESDF 兜底路径唯一避障约束）与 `StaticCorridorLinearConstraint::jacobian`（生产主链路走廊约束）内部都把 `Cx` 硬编码为 `Matrix::Zero(ng, 5)`，与实际 7 维状态不匹配。
- **根因**：这两处约束类是在状态维度恒为 5 的历史假设下实现的，从未参数化为"按 `x.size()` 动态构造"。`StaticCorridorLinearConstraint` 的构造函数校验此前已改为"至少 5 列"，但 `jacobian()` 内部实际构造 `Cx` 的代码被遗漏未同步修改，是本轮排查中定位到的一处真实遗留 bug。
- **发现方式的教训**：现有单元测试（如 `MapsDtArrayAndInitialGuess`/`TruncatesStaticCorridorToTotalSteps`）只检查 `PreprocessingToOcpConverter` 的转换结果（走廊系数矩阵本身的维度），从未真正驱动 NMPC 完整求解一次，因此从未触发过这两个约束类 `jacobian()` 的实际调用，长期未被覆盖。**验证 OCP 装配正确性的单元测试必须至少覆盖一次端到端求解，仅检查中间产物维度不足以捕获约束求值阶段的 bug。**
- **当前应对方式**：两处均改为 `Cx = Matrix::Zero(ng, x.size())`（或等价动态构造），走廊/ESDF 约束本身只依赖 `x,y,theta`（`IterativeCorridorConstraint` 的 `a_row` 固定 5 列语义不变，只是外层 `Cx` 按 `x.size()` 动态构造后只写入前 5 列），新增的状态分量梯度保持为 0，语义与原 5 维行为完全一致。
- **来源**：本 repo 在 `docs/NMPC.md` 6.7.4 节记录的一轮重构中排查（详见该节）。

### `TruncatesStaticCorridorToTotalSteps` 测试比较维度不匹配崩溃（已修复）

- **现象**：状态增广重构为 `PreprocessingToOcpConverter::TruncateCorridor` 增加了"走廊系数矩阵从 5 列补齐到 7 列"的逻辑（新增 `a`、`ddelta` 两列全零系数），但 `test/preprocessing_to_ocp_converter.t.cpp` 的 `TruncatesStaticCorridorToTotalSteps` 用例仍直接用 `Eigen::isApprox` 比较补齐后的 7 列结果与原始 5 列 `pipe_result.c_matrix`，维度不匹配触发 Eigen 断言崩溃（`aLhs.cols() == aRhs.cols()` 失败）。该回归在状态增广落地时未被察觉，是在后续一轮重构（本文档 6.9 节）跑全量测试时才被发现。
- **根因**：`TruncateCorridor` 的列补齐逻辑与该测试的比较逻辑分别由不同轮次的改动引入，缺少一次覆盖两者交互的完整测试运行来及时暴露。
- **当前应对方式**：测试改为只比较原始 5 列范围（`leftCols(src_cols)`），新增列的补零语义已由 `TruncateCorridor` 自身实现保证，无需重复验证。
- **来源**：本 repo 在 `docs/NMPC.md` 6.9.2 节记录的一轮重构中排查（详见该节）。

### NMPC 状态维度从 5 增广到 7（控制量升阶实现 J_smooth）对 HPIPM `UNKNOWN_ERROR` 的敏感性（新证据，根因仍未定位）

- **现象**：经过对跨 stage 顺滑代价的实现考虑，本 repo 在一轮重构中把 `a`、`delta_dot` 从控制量升级为状态量（`BicycleModelJerk`，7 维状态）以实现真正的跨 stage 顺滑代价 $J_{smooth}$ 后，`data3.json` 在默认配置下从"移除终端硬约束后完全收敛"（6.6 节记录）**回归为**首次 SQP 迭代即报 HPIPM `UNKNOWN_ERROR`（状态码 5）。
- **排查过的假设**：把新增的 `smoothing_jerk_weight`/`smoothing_steer_accel_weight` 从 1e-1 调低到 1e-3 并未修复该回归，说明不是新增代价权重量级的问题，而是状态/控制维度的结构性变化本身改变了 HPIPM 内部数值条件。
- **与既有记录的关系**：这是下方"静态走廊+终端硬约束组合导致 HPIPM UNKNOWN_ERROR，根因未定位"条目的新证据——本轮进一步确认该数值不稳定性问题不仅对终端约束/信赖域/走廊模式敏感，也对状态/控制维度敏感，说明触发条件比此前设想的更广泛。
- **当前应对方式**：按 `.agents/prompts/debug-circuit-breaker.md` 精神，该问题此前已在一轮重构中尝试 3 种独立假设未解决根因、已暂停排查；本轮验证新假设（权重量级）后同样未解决，不再做进一步盲目尝试，留待专项调试 Round（建议下一轮把"状态/控制维度"也列入排查变量矩阵）。
- **来源**：本 repo 在 `docs/NMPC.md` 6.7.5 节记录的一轮重构中排查（详见该节）。

### `BSplineSmoother` L-BFGS 精修失败时错误回退到预推前状态，导致预处理管线在"已经足够安全"的合并几何上无谓拒绝（已修复）

- **现象**：经过对剪枝合并流程的诊断考虑，本 repo 在一轮重构中诊断"剪枝合并短机动段后重新求解为何总是失败"时发现，失败根本没有到达 NMPC/HPIPM 阶段，而是在 `BSplineSmoother::smooth()` 内部：碰撞预推（`pre-push`，50 次纯梯度下降）能把合并后新引入的尖锐局部几何的侵入深度从米级压到毫米级（如 0.69m→7.2mm），但精修阶段的 L-BFGS 经常在这类几何上线搜索失败（抛异常）；此时代码把控制点 `x` 整体回退到**预推之前**的 `initial_x`（0.69m 侵入），而不是**预推之后**的检查点，导致最终碰撞校验永远针对未逃生的原始高侵入深度判定失败，`BSplineSmoother::success=false` 进而拖垮整条预处理管线，NMPC 根本没有机会运行。此前多轮调参一直在 HPIPM/NMPC 侧寻找根因，实际上大部分失败案例的真正瓶颈在这里。
- **根因**：`x = initial_x;` 的回退目标选错——设计意图应该是"精修失败时保留预推的逃生成果，只丢弃精修阶段本身的进度"，但实现上错误地把回退点选在了预推之前。
- **当前应对方式**：新增预推后的检查点 `prepushed_x`，L-BFGS 精修失败时回退到 `prepushed_x` 而非 `initial_x`。"双倍碰撞权重重试"这一步的重新出发点保持为 `initial_x` 不变——修复过程中曾尝试把这一步也改成从 `prepushed_x` 出发，但实测导致 L-BFGS 在更陡峭的罚函数地形上从更靠近边界的起点直接发散（控制点飞出到数千米外，触发大量 `ESDFMap query out of bounds` 告警），因此保留该步骤从更安全、验证充分的 `initial_x` 重新出发，只有"最终失败兜底"这一处改为 `prepushed_x`。
- **实测效果**：修复后全量单元测试从 337/338 变为 338/338（长期记录在案的脆弱测试 `BSplineSmootherTest.RetriesWithDoubledCollisionWeightAndSucceeds` 意外转为通过，但仍属临界状态，非稳定修复，见下一条记录）；四数据集调参工具复测显示 `data1`/`data3` 不再在预处理阶段被无谓拒绝，`data1` 能产出轨迹（此前直接失败），`data3` 变为完全收敛。但四数据集的最终机动段数削减效果本轮仍未达标（详见 `docs/NMPC.md` 6.6.7 节），说明该 bug 修复解决的是"被错误拒绝"这一类失败，而非全部失败原因。
- **后续解决计划**：本轮四数据集具体剪枝候选的碰撞残余在当前预推+精修流程下仍有一部分确实收敛不到位（不是被错误拒绝，是真的没收敛好）。建议后续评估：(1) 剪枝合并逻辑本身是否需要在合并点附近插入过渡采样/局部顺滑；(2) 针对"大幅初始侵入+尖锐局部曲率"场景设计专门的多阶段递增碰撞权重或分段预推策略。
- **来源**：本 repo 在 `docs/NMPC.md` 6.6.3/6.6.4 节记录的一轮重构中排查（详见该节）。

### `BSplineSmootherTest.OmpParallelProducesSameResultAsSerial` 揭示碰撞预推阶段固有的 OMP 浮点非确定性（已改为同线程数确定性检测机制）

- **现象**：修复上述预推回退 bug 后，该测试从长期稳定通过变为失败，串行/4 线程结果差异最大达 0.018m（此前容差为 1e-6）。ESDFMap 切换 double 存储后，跨线程控制点分歧进一步确定性地放大到米级（最大 1.08m），0.05m 容差机制彻底失效。
- **根因**：排查确认不是并行归约的数据竞争 bug（`thread_collision_grads` 按线程独立缓冲区累加、循环外统一合并，符合仓库既有的并行归约范式；double 切换后失败值在多次重跑间逐位一致，排除竞争）。真正原因是浮点加法不满足结合律：`reduction(+ : f_collision)` 与分线程梯度累加在不同线程数下的求和顺序不同，经过 50 次预推迭代 + 完整 L-BFGS 精修的链式放大后收敛到不同局部极小。这个非确定性此前一直存在，只是被"回退到与线程数无关的 `initial_x`"bug 意外掩盖——回退发生时结果退化为同一个确定性值，测试因此"巧合地"一直通过；float 存储的 ESDF 量化噪声（~1e-7）进一步充当了"快照网格"：小于量化步长的串并行差异不改变查询到的代价/梯度值，两条轨迹被吸附在同一路径上，double 切换后该掩盖机制消失，混沌敏感性完全暴露。
- **当前应对方式**：✅ 已由人工决策改为新机制（测试更名为 `BSplineSmootherTest.OmpParallelDeterministicAcrossRuns`）：相同线程数（4 线程）重复运行结果必须逐位一致（数据竞争/索引错误的直接探测器）；跨线程（1 vs 4）只比较 `success` 与 `max_intrusion_depth`（0.05m），不再逐点比较控制点几何——优化轨迹对归约舍入顺序的混沌敏感性与正确性无关，两条轨迹均为满足安全门的可行解。
- **后续解决计划**：暂无。如果未来需要严格的跨线程数确定性（如需要逐字节复现调试），需要考虑改用固定顺序的串行归约或 Kahan 求和等方案，但会牺牲当前的并行加速收益，本次不作为默认方案。
- **来源**：本 repo 在 `docs/NMPC.md` 6.6.5 节记录的一轮重构中排查（详见该节）；ESDF double 切换后的机制改为 2026-07-20 人工决策。

### NMPC 优化后轨迹在内部机动段呈现折线/直线（缺少中间打靶点的位置跟踪），已通过信赖域解耦修复

- **现象**：用户报告可视化对比图中，红色（预处理管线产出的 Z_ref）路径平滑，蓝色（NMPC 优化后）路径的每个 maneuver 却近似折线甚至直线，明显丢失了曲线形状。
- **根因**：`PreprocessingToOcpConverter::buildSegment`（生产入口）与 `PathToOcpConverter::buildSegment`（直接 Path 入口）的代价函数设计中，非终端（内部）机动段的状态代价矩阵 `Q` 仅在 v/delta 两维施加极小权重，`x/y/theta` 三维权重恒为 0（这是早期阶段的既有设计，注释写明"避免多段同时争夺端点导致收敛困难"）。这意味着中间打靶点完全不受任何代价函数约束，SQP 在控制效果代价（仅惩罚 `a`/`delta_dot`）驱动下自然收敛到"用最少的转向/加速度连接首尾点"的解，即分段直线。唯一能够把中间点"拉回"参考曲线附近的机制——`ThetaTrustRegionConstraint`/`PositionTrustRegionConstraint`——此前被硬编码为仅在 `use_static_corridor=true` 时才注入（`use_theta_trust_region = use_static_corridor && ...`），而 `max_position_deviation_from_ref` 默认值又是 0.0（关闭）；当静态走廊因下方"HPIPM UNKNOWN_ERROR"问题回退到迭代走廊（动态 ESDF）模式时，这两个信赖域约束完全不生效，中间点毫无约束，直线化现象最明显。
- **当前应对方式**（经过对该问题的考虑，本 repo 采用了以下实现）：
  1. 将 `ThetaTrustRegionConstraint`/`PositionTrustRegionConstraint` 的注入条件从 `use_static_corridor && ...` 改为仅取决于配置值本身（`config_.max_theta_deviation_from_ref > 0.0` / `config_.max_position_deviation_from_ref > 0.0`），任意走廊模式下都生效。
  2. `max_position_deviation_from_ref` 默认值从 0.0 改为 0.15（历史调参验证值），与已有的 `max_theta_deviation_from_ref=0.06` 配合，使中间打靶点被强制约束在参考轨迹附近的一个紧邻带状区域内，视觉上能贴合 Z_ref 曲线。
  3. 修复 `PathToOcpConverter::buildSegment`（直接 Path 入口）此前从未填充 `stage_params.p(3)/p(4)`（x_ref/y_ref）的缺口——`buildSegment` 签名扩展为额外接受 `x_refs`/`y_refs`，与已有 `theta_refs` 参数并列传入，否则该入口下启用位置信赖域会错误地把轨迹约束到坐标原点附近。
  4. 已知代价：该修复放大了下方"HPIPM UNKNOWN_ERROR"问题的影响面——之前迭代走廊（动态 ESDF）回退路径不受信赖域约束时，虽然耗时长（约 24~30s）但通常能收敛出一条机动段数略有削减的轨迹（如 data7 6→5）；解耦后该路径新增了信赖域约束行，四数据集回归显示 `data1`/`data6`/`data7` 从"慢但基本可用"变为"HPIPM 在第 0 次迭代即失败，快速回退到预处理轨迹"（约 200~300ms），机动段数不再被 NMPC 削减（但也不会被拓扑清洗 bug 放大，参见下方 fallback 拓扑清洗记录）；`data3` 则从"未完全收敛但机动段数 9→8"变为"完全收敛（`converged=true`）但机动段数保持 9"。净效果：折线视觉缺陷在四个数据集上均已消除（要么 NMPC 收敛且贴合参考曲线，要么直接回退到本就平滑的预处理曲线），但代价是暂时牺牲了部分数据集的机动段削减效果，这与下方 UNKNOWN_ERROR 问题是同一个根因链条上的两个表现。
- **后续解决计划**：核心仍然是定位并修复下方记录的 HPIPM `UNKNOWN_ERROR` 数值稳定性问题；一旦解决，预计信赖域约束能与走廊/剪枝机制正常协同，同时获得平滑轨迹与机动段削减。若需要在此之前进一步权衡，可考虑：让信赖域约束的容差随重试轮次自适应放宽（类似 `AdaptiveRetryConfig` 的思路），或为迭代走廊回退路径单独设置更宽松的信赖域默认值。
- **来源**：一轮实测截图反馈诊断。

### 静态走廊 + 终端硬约束组合在 HPIPM 上出现非确定性 `UNKNOWN_ERROR`（状态码 5），跨数据集复现，根因未定位

- **现象**：经过诊断 `data6.json` 静态走廊路径首次 SQP 迭代即失败（`QPSolverStatus::UNKNOWN_ERROR`，状态码 5）时发现，该失败模式并非 `data6.json` 独有——`data1.json`/`data3.json` 在相同默认配置（`use_static_corridor=true` + `TerminalPoseBoxConstraint` + `TerminalFinalStateConstraint` + `ThetaTrustRegionConstraint` + `PositionTrustRegionConstraint` 同时启用）下同样会在多数重试尝试中于 SQP 迭代 0 复现同一状态码；但并非每次都失败——例如 `data3.json` 在 `dense_step_dist` 加倍重试、`total_steps` 从 174 变为 240 时曾一次性收敛成功（`converged=true`），随后紧邻的下一次剪枝重求解（`total_steps=300`，其余配置不变）又立刻复现失败。这种"仅因总步数/离散粒度发生微小变化就在失败/成功之间切换"的表现，指向 HPIPM 内部 IPM 迭代在当前约束组合下处于数值条件边缘（很可能是 `mapHpipmStatus()` 中的 `MIN_STEP` 分支，即 IPM 步长退化到数值下限），而非真正意义上的约束不可行。
- **已排除的假设**（逐条验证，均未解决）：
  1. **静态走廊在参考点自身违反安全边界处产生不可行约束** —— 已实现 `StaticCorridorBuilder::computeDScalar` 自洽性修正（补偿违反深度使约束在 `Z_ref` 处取等号），复测后 `data6.json` 仍复现同一状态码，说明这不是（至少不是唯一）根因。
  2. **`ng_max` 手工计算与实际装配的约束行数不一致** —— 已确认存在真实 bug（终端位姿盒约束关闭时手工公式仍无条件加 6 行，导致 `INVALID_ARGUMENT`），已修复为直接调用 `stc_SQP::strategy_internal::computeOcpNgMax()` 动态求和；修复后 `INVALID_ARGUMENT` 消失，但默认配置（终端约束启用）下的 `UNKNOWN_ERROR` 依然存在，说明 ng_max 不是这里的根因。
  3. **`ThetaTrustRegionConstraint`/`PositionTrustRegionConstraint` 信赖域约束与走廊/终端约束冲突** —— 关闭两者后单独在 `data6.json` 上复测仍复现同一 `UNKNOWN_ERROR`，排除信赖域为根因。
- **根因**：尚未定位。合理猜测方向（未验证）：`terminal_position_weight`/`terminal_heading_weight`（1e5）与走廊硬约束、终端硬 box 约束叠加后使 QP 的 KKT 系统条件数在特定问题规模下急剧恶化；或 `TerminalPoseBoxConstraint` 与 `TerminalFinalStateConstraint` 两者同时作用在最后一段导致的约束冗余/近似线性相关。
- **当前应对方式**：`PostProcessor`/`AdaptiveRetryConfig` 现有的"关闭静态走廊重试"兜底机制可以绕过这个问题（回退到迭代走廊 + 动态 ESDF 代价，虽然更慢但数值上更稳健），因此当前不会导致业务层整体失败，只是会牺牲静态走廊本应提供的“哪怕未收敛也绝对安全”的强保证与更快的求解速度。
- **后续解决计划**：按 `.agents/prompts/debug-circuit-breaker.md` 的精神，本轮已对该 bug 尝试 3 种独立假设均未解决根因，暂停继续排查，留待专项调试 Round：建议后续（1）尝试降低 `terminal_position_weight`/`terminal_heading_weight` 数量级并观察 `UNKNOWN_ERROR` 出现频率是否下降；（2）尝试仅保留 `TerminalPoseBoxConstraint` 或仅保留 `TerminalFinalStateConstraint`（不同时启用两者）；（3）在 HPIPM 侧开启更详细的 IPM 诊断日志（若 `third_party/StcSQP`/HPIPM 提供），确认具体是哪一步 IPM 迭代触发 `MIN_STEP`。
- **来源**：一轮实测诊断（`data1.json`/`data3.json`/`data6.json` 均复现）。

### `static_corridor_builder.h` 中 `soft_margin > hard_margin` 约束无代码级强制校验

- **现象**：`StaticCorridorBuilder` 构造函数（`static_corridor_builder.cpp:27-28`）只校验了 `hard_margin >= 0`、`soft_margin >= 0` 且均为有限值，但未检查 `soft_margin > hard_margin`。当前默认值（0.18 > 0.0）自然满足，但若未来有人将 `hard_margin` 设为大于 `soft_margin` 的值，系统不会在构造期报错，而是静默产生语义错误的走廊约束。
- **根因**：这是早期设计 `StaticCorridorBuilderConfig` 时遗漏的约束校验。
- **当前应对方式**：默认值下自然满足（0.18 > 0.0），`PreprocessingPipeline` 构造函数中增加了对 `collision_safety_margin` 的非负有限校验。
- **后续解决计划**：建议在后续战术改动中为 `StaticCorridorBuilder` 构造函数增加 `soft_margin > hard_margin` 的显式校验。
- **来源**：一次复审（⚠️ 严重，既存问题，非本次改动引入，不阻塞本次收敛）。

### 三次方碰撞惩罚在 `collision_margin = 0` 时存在近零梯度消失

- **现象**：碰撞检测物理安全裕度移除后（`collision_margin = 0`），三次方碰撞惩罚 `max(0, R - d)^3 * weight` 在侵入深度接近 0 时梯度趋于 0（梯度 ∝ `intrusion²`），低碰撞权重下优化器几乎"看不见"轻微侵入。`collision_validation_tolerance` 已从 2cm 调整为 1e-4（0.1mm），但低权重场景（如 weight < 10）下优化器仍可能无法将侵入压到该阈值以下。
- **根因**：三次方惩罚的数学特性——对近零侵入的梯度消失是设计使然（防止 L-BFGS 在已足够安全的区域浪费迭代），而非缺陷。
- **当前应对方式**：默认碰撞权重（500）足够推动优化器将侵入压到 1e-4 以下；`BSplineSmootherTest.RetriesWithDoubledCollisionWeightAndSucceeds` 已更新为显式设置 `collision_validation_tolerance = 0.05` 以隔离默认容忍度变更。生产路径不受影响。
- **后续解决计划**：暂无。如果未来需要支持极低碰撞权重的场景，可考虑引入 L1/L2 混合惩罚或 hinge loss 替代纯三次方形式。
- **来源**：2026-07-10 碰撞检测重大更新（移除物理安全裕度，改为纯数值容差）。

### ~~`main.cpp` 生产入口尚未接入 `PreprocessingPipeline`，预处理管线产出仍局限于测试/benchmark~~（已解决）

- **现象**：~~`src/main.cpp` 直接调用 `NmpcSolver::optimizeWithPruning(init_path, esdf_map)`，全程未构造或调用 `PreprocessingPipeline`；预处理管线产出的固定维数 Z_ref/U_ref、静态走廊系数 `C_matrix`/`d_vector` 目前只在 `test/preprocessing_pipeline.t.cpp`（含 `PipelineOutputFeedsNmpcSolverEndToEnd` 端到端验证测试）与 `bench/bench_preprocessing_pipeline.cpp` 中被消费，生产可执行路径仍使用 `PathToOcpConverter` 的简化初始猜测生成逻辑。~~
- **根因**：~~`third_party/StcSQP` 现有 SQP 框架尚未扩展到能直接消费 `PreprocessingPipeline` 的输出结构——`NmpcSolver::optimize()` 内部的 `MultiStageOCP` 构建/热启动机制仍按 `PathToOcpConverter` 的简化插值假设组织；`NmpcSolverConfig::static_corridor_C/d` 字段虽已登记，但 `optimize()` 内部尚未消费。~~
- **当前应对方式**：✅ 已解决。`src/main.cpp` 已切换为 `Path → PreprocessingPipeline → PreprocessingToOcpConverter → NmpcSolver` 完整链路，直接消费 `PreprocessingPipelineResult` 的 `z_ref`/`delta_t`/`c_matrix`/`d_vector`；新增 `PreprocessingToOcpConverter` 把非均匀 `delta_t` 通过 `StageSegment::dt_array` 接入，并同步截断静态走廊系数到 OCP 总步数。
- **后续解决计划**：无。该条目已关闭，保留记录供追溯。
- **来源**：一次全局质量校验（原 ⚠️ 严重，已解决）。

### PathPoint 派生量"不一定存在"，下游必须先 `hasXxx()` 再 `getXxx()`

- **现象**：`PathPoint::getKappa()` / `getV()` / `getDelta()` / `getA()` / `getDeltaDot()` 在对应派生量未设置时会抛出 `std::logic_error`，导致下游代码在忘记检查的情况下直接崩溃。
- **根因**：`PathPoint` 设计为 `Pose` + 一组"不一定存在"的派生量，用 `std::numeric_limits<double>::quiet_NaN()` 作为"未提供"哨兵，并通过 `has/get/set` 三件套强制调用方显式确认可用性。不同来源的 `PathPoint` 只会携带其来源天然具备的派生量：
  - `kappa`：唯一权威来源是 `Path` 内部的外接圆法曲率估计，只有经过 `Path::finalize()` 的初始路径点才会设置；
  - `v` / `delta`：来自 `NmpcSolver::ToPath()` / `pruneShortestSegment()` 对优化状态的回填；
  - `a` / `delta_dot`：来自上述 NMPC 方法对优化控制的回填，且每段最后一个点通常没有对应控制量，因此保持未设置；
  - 直接从 proto 反序列化或由 `Pose` 构造的 `PathPoint` 则五个派生量全部未设置。
- **当前应对方式**：
  - 所有下游读取前必须先调用 `hasXxx()` 判断，例如 `PathToOcpConverter::interpolateAtArcLength` 在读取 `kappa` 时会对端点分别调用 `hasKappa()`，缺失时回退为 `0.0`；`Visualizer` 在绘制曲率相关细节时同样先 `hasKappa()` 再 `getKappa()`。
  - 如果下游确实需要给没有 `kappa` 的 `PathPoint` 补曲率（例如 NMPC 输出），可以将其重新组织为 `Path` 并调用 `Path::finalize()`，由 `Path` 统一批量估计曲率。
- **后续解决计划**：暂无。这是 `has/get/set` 设计本身的预期行为，不是缺陷；后续新增消费 `PathPoint` 派生量的代码时，应继续遵守"先 has 后 get"的契约。
- **来源**：早期 Pose/PathPoint 重构收尾审计。

### 项目级编译配置未启用 `-Wall`/`-Wextra` 等额外警告标志

- **现象**：`CMakeLists.txt` 中未配置任何 `-Wall`/`-Wextra`/`-Wpedantic` 或等价编译器警告选项；此前各阶段评审中"无新增编译告警"的结论均基于默认警告级别，可能遗漏一些本可被更严格警告捕获的问题（如隐式类型转换、未使用变量等）。
- **根因**：这是项目早期遗留的基线构建配置，不在早期 Pose/PathPoint 重构范围内；直接开启更严格警告可能会一次性暴露大量历史遗留告警，需要单独规划清理工作。
- **当前应对方式**：本次全局质量校验中不处理该基线配置变更；Dev Agent 在新增/修改代码时仍按现有规范保持逻辑清晰、避免明显可Warning的写法。
- **后续解决计划**：建议后续单独立项评估是否开启 `-Wall`/`-Wextra` 并将历史告警清零，避免与本次重构混为一谈。
- **来源**：一次全局质量校验（📝 建议，未采纳在该次审计中处理）。

### `ESDFMap::getDistAndGrad` 越界查询返回 `(0.0, Zero)`，会被碰撞校验误判为刚好在障碍物边界

- **现象**：在 `BSplineSmoother` 的 `SmoothsCurvedManeuverWithContinuousHeading` 测试中，车身外圆局部偏移后的圆心 `y≈7.01`，超出 `MakeEmptyEsdfMap()` 原地图 `max_y=7.0`，`ESDFMap::getDistAndGrad` 越界返回距离 `0.0`，被碰撞校验计算为 `intrusion = outer_radius + margin - 0.0 ≈ 0.67 m`，导致 `success=false`、L-BFGS 未启动。
- **根因**：`ESDFMap` 对越界坐标统一返回 `(0.0, Eigen::Vector2d::Zero())`，该哨兵值恰好与"距离障碍物边界 0 m"无法区分；车身 footprint 模型叠加转弯时的横向偏移后，圆心很容易落在测试地图边缘外。
- **当前应对方式**：
  - 单元测试与 benchmark 中构造的 ESDF/GridMap 必须完整覆盖被测车身包络，并留有一定余量；本次将 `MakeEmptyEsdfMap()` 的高度从 `100` 扩大到 `150`，使 `max_y` 从 `7.0 m` 提升到 `12.0 m`。
  - 真实数据 benchmark 直接使用 `OptimizeRequest.environment()` 构建 `GridMap`/`ESDFMap`，地图范围由上游保证覆盖车辆可行区域。
  - 碰撞校验在 `dist=0` 时仍按侵入处理，这是保守行为；若未来需要区分"未知/越界"与"真实零距离"，应在 `ESDFMap` 层引入独立哨兵（如 `NaN` 或可选返回值），但属于冻结接口变更，需单独评估。
- **后续解决计划**：暂无。当前在测试/使用侧保证地图覆盖范围即可；如后续在真实场景边界频繁触发误报，再讨论是否修改 `ESDFMap` 越界语义。
- **来源**：一次调试 `SmoothsCurvedManeuverWithContinuousHeading` 失败。

### OSQP CSC 矩阵必须严格按列号递增填充，否则 `osqp_setup` 报数据校验错误

- **现象**：`SpeedProfilePlanner` 首个测试 `PlansForwardStraightSegment` 中 `OsqpSolver::setup()` 返回 `exitflag=1`（`OSQP_DATA_VALIDATION_ERROR`），调试发现 Hessian `P` 的 `P_i` 中出现 `row > col`。
- **根因**：CSC 格式要求非零元按列顺序存放。实现 `P` 时曾因把同一循环内先填 `b_i` 列再填 `a_i` 列误当成列递增，但实际上 `a_i` 的列号（`n_points + i`）远大于后续 `b_{i+1}` 列号（`i+1`），导致 `P_p` 与 `P_i` 错位。
- **当前应对方式**：`SpeedProfilePlanner::solveQp` 中把 `P` 的填充拆为“先全部 `b` 列、再全部 `a` 列”两个循环；`A` 矩阵仍用 `std::vector<std::vector<std::pair<int, double>>>` 按列收集后统一排序输出，保证每列内行号递增。
- **后续解决计划**：暂无。新增通过 `OsqpSolver` 调用 OSQP 的代码时，应沿用“列优先、同列行号递增”的 CSC 构造方式，并在单元测试中覆盖 setup 成功路径。
- **来源**：一次调试 `SpeedProfilePlannerTest.PlansForwardStraightSegment` 失败。

### `RetriesWithDoubledCollisionWeightAndSucceeds` 测试含合谋参数，默认值变更时可能失效

- **现象**：该测试依赖 `weight_collision=10`（远低于默认 500）与障碍物位置 `(2.5, 1.1)` 恰好使首次优化碰撞校验失败、翻倍到 20 后成功。若 `VehicleFootprintModel` 参数、`collision_margin` 或 `outer_circle_radius_` 默认值发生变化，这个"恰好"条件可能被打破，导致测试误失败（首次直接成功）或误通过（重试后仍失败）。
- **参数收紧影响**：一轮调参将 `lbfgs_max_iterations` 默认值从 100 经多数据集验证后最终确定为 80，`lbfgs_max_linesearch` 确定为 20；断言已修复为 `EXPECT_GT(result.lbfgs_iterations, config.lbfgs_max_iterations)`，使断言随参数变化自动适应。当前语义为首次+重试两轮总迭代次数大于 80。
- **根因**：重试逻辑的正确性验证需要一个可控的"先失败后成功"场景，当前通过精心调谐的轻障碍物 + 低碰撞权重实现，参数间存在隐式耦合。
- **当前应对方式**：测试命名为 `RetriesWithDoubledCollisionWeightAndSucceeds` 并在注释中说明了场景构造意图；后续修改 `VehicleFootprintModel` 或 `BSplineSmootherConfig` 默认值时，需同步检查该测试是否仍按预期通过。
- **后续解决计划**：暂无。这是测试脆弱性（test brittleness）而非业务逻辑缺陷；若将来频繁因无关变更导致该测试失败，可考虑重构为 mock L-BFGS 求解器直接控制收敛/失败行为的白盒测试。
- **来源**：一次复审（📝 建议）。

### `AdaptiveResampler` 起始对齐补丁在 `n_pad=1` 时静默丢弃

- **现象**：当转向过渡极快（`t_steer ≤ delta_t_min`，对应角度差 ≤ 0.016 rad ≈ 0.9°）导致 `n_pad = 1` 时，`buildSteerPaddingSegment` 产生 0 个内部点、1 条 `delta_t`。`assembleFinalTrajectory` 因 `start_padding.points.empty()` 跳过整个起始补丁块，该条 `delta_t` 也随之丢弃。此时从 `initial_steer_angle` 到第一个打靶点 `delta` 的过渡时间未被记录在输出序列中。
- **根因**：`n_pad=1` 意味着整个转向过渡可在单步内完成（≤ 0.05s），补丁段没有内部桥接点的存在价值。但 `assembleFinalTrajectory` 的"空 points → 跳过整个起始补丁块"逻辑没有区分"不需要补丁"（角度差 ≤ epsilon）和"需要补丁但 n_pad=1"（角度差很小但非零）这两种情况。
- **当前应对方式**：实际影响极低。`delta_t_min` 有 `kSteerPaddingDeltaTMin = 0.05s` 下限约束，`n_pad=1` 仅发生在角度差 ≤ 0.016 rad 的场景，此量级下 NMPC 首步求解即可自然覆盖。当前代码依赖此事实，不做额外处理。
- **后续解决计划**：暂无。若未来下游 NMPC 求解器对首步时间精度要求提高，可考虑在 `assembleFinalTrajectory` 中为 `n_pad=1` 场景单独插一条兜底 `delta_t`。
- **来源**：一次复审（📝 建议）。

### Benchmark 管线组装代码在 `bench_adaptive_resampler.cpp`、`bench_differential_flatness_solver.cpp` 与 `bench_static_corridor_builder.cpp` 之间大量重复

- **现象**：`bench_adaptive_resampler.cpp`、`bench_differential_flatness_solver.cpp`、`bench_static_corridor_builder.cpp` 三个文件各自包含 ~150 行几乎相同的预处理管线各阶段组装代码（`DirectionToSign`、`ComputeMinEsdfDistAtPoint`、`BuildSpeedProfileInput`、`BuildDifferentialFlatnessInput`、`BSplineSmootherBenchmarkAccessor`；`BuildAdaptiveResamplerSegmentInput` 在两个文件中重复）。`bench_bspline_smoother.cpp` 因测试的是最上游阶段本身、无需构造上游管线，不在此列；`bench_preprocessing_pipeline.cpp` 已正确复用 `PreprocessingPipeline` 一体化调用，同样不存在此问题。
- **根因**：每个预处理阶段的 benchmark 都需要单独跑通前置管线以获取本阶段的输入数据，当前各 benchmark 独立实现了管线的上游部分，未抽取公共工具函数。
- **当前应对方式**：各 benchmark 独立维护自己的管线组装代码；基准测试均能正常运行。一次全局质量校验中已将上述公共工具函数提取到 `bench/bench_preprocessing_utils.h`，供 `bench_adaptive_resampler.cpp`、`bench_differential_flatness_solver.cpp`、`bench_static_corridor_builder.cpp` 三者复用。
- **后续解决计划**：已解决（一次全局质量校验）。
- **来源**：一次复审（📝 建议，未采纳，记录为后续技术债）；一次全局质量校验复审（📝 建议，发现遗漏 `bench_differential_flatness_solver.cpp` 并实际解决）。

### `PreprocessingPipeline::buildSpeedProfileInput` 与 `buildDifferentialFlatnessInput` 各自独立计算 B 样条基函数（重复计算）

- **现象**：`PreprocessingPipeline` 的两个 build 方法（`preprocessing_pipeline.cpp`）在对同一组 `dense_points` 构造 `SpeedProfileInput` 和 `DifferentialFlatnessInput` 时，各自独立调用 `buildKnotVector` 并逐点 `computeBasisAtU`，对同一组密集配点遍历了两遍、各做了一次完整的基函数求值。两者需要的导数阶数不同（前者需要 d1/d2，后者需要 d0/d1/d2/d3），但 knot vector 和 basis indices 完全相同。
- **根因**：当前设计以代码清晰度优先，两个 build 方法职责独立、各自完成自己的输入构造。理想情况下 `BSplineSmoother` 可在 `Result` 中提供预计算的 `BasisPack` 缓存，或管线在两个 build 方法间共享 knot vector + basis 中间结果。
- **当前应对方式**：维持现状，不做优化。离线后处理场景下两次遍历的额外开销可忽略。
- **后续解决计划**：若未来需要在线部署或性能敏感场景，可考虑在 `BSplineSmoother::Result`（已冻结接口）中新增预计算 `BasisPack` 缓存字段，或由 `PreprocessingPipeline` 在两个 build 方法间共享中间结果。该变更涉及冻结接口修改，需单独评估。
- **来源**：一次复审（📝 建议，不采纳，登记到 known-limitations）。

### `PreprocessingPipeline::run()` 中 `per_maneuver_outputs` 持有全部中间结果拷贝导致内存峰值

- **现象**：`PreprocessingPipeline::run()`（`preprocessing_pipeline.cpp`）将所有 maneuver 的 `smooth_result`（含 `dense_points` 全部子圆数据）、`speed_result`、`diff_flat_result` 均保存在 `per_maneuver_outputs` vector 中直到 `run()` 结束才释放。对于真实数据（数十个 maneuver，每个数百 dense_points），内存峰值可能达到数十 MB。
- **根因**：当前设计以代码清晰度优先——先收集全部中间结果再做统一的自适应重采样处理，逻辑简单直观。若改为逐 maneuver 立即构造 `AdaptiveResamplerSegmentInput` 并释放中间结果，可降低内存峰值但会降低代码可读性。
- **当前应对方式**：维持现状，不做优化。离线后处理场景下数十 MB 的内存峰值可接受。
- **后续解决计划**：若未来需要在线部署或内存受限环境，可考虑在处理完每个 maneuver 后立即构造 `AdaptiveResamplerSegmentInput` 并释放中间结果，而非全部攒到循环结束后再统一处理。
- **来源**：一次复审（📝 建议，不采纳，登记到 known-limitations）。

### `SQPSolver::assembleCostImpl` 产出的标量代价 `cost_value` 被丢弃，`computeMerit()` 仍独立调用 `CostTerm::evaluate()`

- **现象**：`CircleFootprintEsdfPenaltyCost::evaluateGradientAndHessian()` 在装配 QP 时一次性产出了 cost/gradient/hessian，但其中标量 `cost` 在 `assembleCostImpl` 中计算后即被丢弃；后续 `SQPSolver::lineSearch()` → `computeMerit()` 路径仍会逐步调用 `segment.cost->evaluate()`。对 `CircleFootprintEsdfPenaltyCost` 而言，这意味着每步 SQP 迭代中 `computeViolations()` 仍会被调用 2 次（装配 1 次 + merit 1 次），而非理论上的 1 次。
- **根因**：`QPData` 只存储梯度与 Hessian，没有保留 per-step 标量代价字段；`Trajectory` 也没有 per-step cost 缓存，`computeMerit()` 必须重新计算。
- **当前应对方式**：维持现状。此前一轮优化的范围是"补齐组合求值接口 + 消除装配阶段的重复查询"，merit 路径的重复查询属于超出范围的进一步优化机会；当前已从旧模型的每步 3~4 次 `computeViolations()` 降为 2 次。
- **后续解决计划**：若未来需要彻底消除 merit 路径的重复查询，可考虑扩展 `Trajectory` 增加 per-step cost 缓存字段，并让 `assembleCostImpl` 写入、`computeMerit()` 读取。该变更涉及 `Trajectory` 数据结构扩展与 `computeMerit` 重构，需单独评估，可能作为后续性能优化项落地。
- **来源**：一次复审（📝 建议，当时未处理，登记为后续优化机会）。

### `HPIPMQPSolver` 热启动求解失败后的缓存清空路径缺少单元测试覆盖

- **现象**：`HPIPMQPSolver::solve()` 在 `status != SUCCESS` 时执行 `clearWarmStartCache()`，保证脏解不会污染下一次求解。但当前没有测试用例构造一个必然失败的 QP（如不可行的硬约束组合）并验证后续正常 QP 的 `solve()` 仍能成功（证明缓存已被正确清空）。
- **根因**：此前新增的 4 个测试覆盖了 Happy Path（相同 QP 复用、维度不匹配退化、Partial Condensing 路径、SQP 集成），但未覆盖失败清空缓存的 Sad Path。
- **当前应对方式**：代码逻辑本身正确（失败 → `clearWarmStartCache()` → 返回错误码），人工审查已确认该路径不会被绕过；缺少的测试用例不影响功能正确性。
- **后续解决计划**：后续战术改动中可补充 `HpipmWarmStartClearsCacheOnFailedSolve` 测试用例。
- **来源**：一次复审（📝 建议，不阻塞收敛，登记到 known-limitations）。

### `HPIPMQPSolver::totalIterations()` 跨多次 solve 累计语义缺少单元测试覆盖

- **现象**：`totalIterations()` 是后续新增的公开接口，仅在 benchmark 中被间接使用（读取差值计算单次 solve 的 IPM 迭代数），没有单元测试验证其跨多次 `solve()` 调用的累计行为是否正确。
- **根因**：`totalIterations()` 的实现极简（成功时 `total_iter_ += last_iter_`），Dev Agent 判断单独为其写测试的投入产出比不高。
- **当前应对方式**：benchmark 中已通过 `totalIterations() - total_iter_before` 间接验证了单次 solve 的迭代计数；跨多次 solve 的累计语义依赖代码审查确认正确。
- **后续解决计划**：若未来 `totalIterations()` 的实现逻辑变复杂（如引入重置/溢出处理），应补充单元测试。
- **来源**：一次复审（📝 建议，不阻塞收敛，登记到 known-limitations）。

### `Constraint`/`CostTerm` 去虚拟化（`std::variant`）已评估不采纳

- **现象**：曾尝试将 `third_party/StcSQP` 的 `Constraint`/`CostTerm` 从虚函数多态迁移到 closed-set `std::variant`，并改造 `StageSegment`/`SQPSolver` 全部分派点使用 `std::visit`。实现已编译、测试通过，但 `third_party/StcSQP/bench/bench_performance_profiling.cpp` 的 Release 端到端基准（`BM_Data3RealScenario_MultiSegmentBicycleCorridor`，data3.json 全段，N=493）显示：variant 实现（mean ~802 ms）与 HEAD 原始虚函数实现（mean ~726 ms）处于同一数量级，variant 没有可解释的性能提升。
- **根因**：真实泊车场景的总耗时主要由 HPIPM IPM 求解、CasADi 凸走廊 Jacobian 计算、RK4 动力学线性化占据，约束/代价接口的分派开销在端到端耗时中占比过低；`-march=native` 与 OpenMP 并行化已使虚函数调用本身的延迟被进一步摊薄。
- **当前应对方式**：经 Dev Agent 实现 + Review Agent 评审（确认 🚨=0 ⚠️=0），最终结论为**不采纳**。在一次全局质量校验中，人工确认选择路径 A：已将 `third_party/StcSQP` 的 `Constraint`/`CostTerm`/`StageSegment`/`SQPSolver` 相关代码真正回退到之前版本完成时的虚函数 + `clone()` 池版本，从工作树中移除了 `constraint_variant.h`/`cost_variant.h`，并同步恢复了主仓库 `src/core/NMPC/nmpc_solver.cpp`、`path_to_ocp_converter.cpp` 及相关测试/benchmark 的调用方式。当前代码库 100% 运行在虚函数机制上，与文档描述一致。
- **后续解决计划**：本次评估结论与回退动作已登记在 `docs/interfaces.md`、`docs/known-limitations.md` 与 `third_party/StcSQP/design_document.md` 中，避免未来重复调研同一个问题。若未来硬件架构或求解器构成发生重大变化（如引入 GPU 加速使分派开销占比上升），可重新评估。
- **来源**：一次评估（已关闭，不采纳）；一次全局质量校验的回退动作。

### 四数据集 NMPC 均未在 300 次迭代内完全收敛

- **现象**：在四个目标真实数据集（`20260525202558684/data3.json`、`20260605220906669/data1.json`、`20260605220906669/data7.json`、`20260608184048757/data6.json`）上，使用默认 `NmpcSolverConfig`（`max_iter=300`、`use_line_search=false`、静态走廊默认开启）运行完整后处理链路，NMPC 均未触发 `converged` 标志，最终返回末次迭代轨迹。
- **根因**：
  - 真实泊车场景 maneuver 较多、OCP 总步数大，SQP 在 300 次迭代内无法将 KKT 残差压到收敛阈值；
  - 尝试启用 `max_theta_deviation_from_ref` θ 信赖域、调整 `esdf_penalty_weight`、切换 `use_line_search`、提升 HPIPM 软约束 L2 权重等参数均未带来收敛突破，部分参数反而触发首迭代失败或导致终端精度恶化；
  - `data6.json` 的静态走廊 hard 约束与 Warm Start 存在不兼容，导致首迭代 QP 不可行；`data3.json` 的静态走廊过紧导致优化后长度异常增加且存在侵入。
- **当前应对方式**：
  - 接受“未完全收敛但返回可用末次迭代”作为当时的可收尾结论；
  - 在 `NmpcSolver::solveOcp()` 中实现混合求解策略：先尝试 hard-only 静态走廊，再尝试 HPIPM 原生软约束（`ns > 0`，`L2` 权重 `1e5`），然后 dynamic-ESDF，三者均不满足统一质量门时返回空轨迹，由 `PostProcessor` 关闭静态走廊重试或最终回退到预处理轨迹；
  - 制定量化准出判据：优化后 maneuver 数不增加、总路径长度相对增幅不超过 +5%、最大侵入深度不超过 0.02m、终端误差（若启用）不超过 0.1m/1°、曲率无明显异常跳变；四数据集均满足；
  - `data3.json` 通过关闭静态走廊重试，路径长度从 24.582m 降至 23.514m（-4.34%），最大侵入深度 0m，终端误差 0.0152m/0.11°；
  - `data1.json` 采用 soft-corridor 模式，路径长度从 12.988m 降至 12.476m（-3.94%），最大侵入深度 0m，终端误差 0.0492m/0.05°；
  - `data7.json` 因初始猜测已超出 hard 走廊边界，直接采用 ESDF 模式，路径长度从 18.744m 降至 18.708m（-0.20%），最大侵入深度 0m，终端误差 0.0254m/0.45°；
  - `data6.json` 在 hard/soft/ESDF 三种模式下均无法满足质量门，最终回退到经 BSplineSmoother 碰撞预推的预处理轨迹，长度 36.859m（-0.01%），最大侵入深度 0m，终端误差 0m/0°。
- **后续解决计划**：后续收尾阶段可评估是否将“提升 NMPC 收敛率”作为遗留技术债继续优化；候选方向包括：更大 `max_iter`、分段级联求解、针对未收敛场景继续细化代价权重/信赖域调参，或探索更鲁棒的终端约束机制。HPIPM 软约束禁用 partial condensing 导致的耗时问题（`data6.json` 约 20s）也列为后续性能优化项。
- **来源**：一轮四数据集调参实测、补充迭代（HPIPM 软约束 + 混合求解策略）、续补充迭代（ESDF 质量门 + 预处理兜底）。

### 主仓库业务日志宏与 `third_party/StcSQP` 内部日志宏重定义

- **现象**：同时包含 `src/util/logger.h`（业务层日志宏）与 `third_party/StcSQP/src/core/logger.h` 的翻译单元（典型如 `test/preprocessing_pipeline.t.cpp`，通过 `nmpc_solver.h` 间接引入两套头文件），编译时出现 `LOG_TRACE`/`LOG_DEBUG`/`LOG_INFO`/`LOG_WARN`/`LOG_ERROR` 宏重定义警告。
- **根因**：两套日志系统各自独立定义同名宏导致的长期结构性问题，非近期改动引入。
- **当前应对方式**：这是一个已知的非阻塞性编译警告，不影响功能正确性；已在此前一次全局质量校验中登记，避免未来被误判为新增回归。
- **后续解决计划**：未来可通过统一日志抽象层、命名空间化宏前缀或调整头文件包含顺序来解决；属于跨仓库集成层面的长期技术债务，不在本次全局校验范围内。
- **来源**：一次全局质量校验评审（📝 建议）。

### HPIPM 原生软约束（`ns > 0`）禁用 partial condensing，导致 soft-corridor 场景求解耗时显著增加

- **现象**：`NmpcSolver::solveOcp()` 在 `data6.json` 上启用 HPIPM 原生软约束（`ns > 0`）后，300 次 SQP 迭代耗时约 37s；而无软约束的 ESDF 模式（`ns == 0`）在同样数据集上仅需约 10s。耗时差异主要源于 HPIPM 在 `ns > 0` 时无法使用 partial condensing。
- **根因**：
  - `third_party/StcSQP/src/qp/hpipm_solver.cpp` 原已实现 `use_partial_condensing = (cond_N > 0 && cond_N < N && ns == 0)`，明确在 `ns > 0` 时回退到无凝聚路径；
  - 补充迭代中曾尝试移除 `&& ns == 0` 条件以启用 partial condensing + 软约束，但 `third_party/StcSQP/test/test_qp_solvers.cpp` 中的 `QPSolvers.HpipmPartialCondensingMatchesDenseLDLTWithSoftConstraints` 测试失败，确认当前 HPIPM/StcSQP 版本不支持该组合。
- **当前应对方式**：
  - 保持 `ns > 0` 时禁用 partial condensing；
  - 在 `NmpcSolver::solveOcp()` 中为 soft 模式设置 `max_iter = min(config_.max_iter, 150)`，将 `data6.json` 耗时从约 37s 降至约 20s，同时保持优化质量（31.331m，最大侵入深度 0m）。
- **后续解决计划**：若未来升级 HPIPM 或 StcSQP 后支持 partial condensing + 软约束，可重新评估是否开启；或改用自定义二次惩罚代价替代 HPIPM 原生软约束，以保留 partial condensing 加速。
- **来源**：一轮调参补充迭代调试。

### 终端位姿硬约束在 StcSQP/HPIPM 当前接口下难以实现

- **现象**：曾尝试通过 `TerminalPoseConstraint` 在最后一段最后一步施加 `|x_N - x_goal| <= 0.1m`、`|theta_N - theta_goal| <= 1°` 的 box 约束；在真实数据集及合成多段测试（如 `PreprocessingToOcpConverterTest.SplitsCorridorAcrossSegmentsWithoutFirstIterationFailure`、`EndToEndSwitchbackManeuver`）上均触发 HPIPM 首迭代失败/UNKNOWN_ERROR。
- **根因**：StcSQP 的 `HPIPMQPSolver` 按单一 `ng_max` 构造所有阶段的普通约束维度，而终端约束仅作用于最后一段；对非最后一段无论填充零行还是引入冗余约束，都会破坏 HPIPM 内部数值条件，导致 QP 求解失败。
- **当前应对方式**：回退硬约束实现，改为通过高终端代价权重（`terminal_position_weight = terminal_heading_weight = 1e4`）将终点误差压到目标范围；`apa_tune_post_processor` baseline 变体已采用该权重，`data1`/`data7`/`data3(ESDF)` 的终端位置误差均 < 0.05m、航向误差均 < 0.5°，满足 `< 0.1m / 1°` 验收指标。
- **后续解决计划**：若未来 StcSQP/HPIPM 支持每阶段独立 `ng` 或更友好的阶段约束接口，可重新引入硬终端约束；当前通过代价权重方案已满足数据验收。
- **来源**：一轮调参延续（终端精度调参）。

### `data6.json` 通过预处理轨迹兜底满足质量门

- **现象**：在 `apa_tune_post_processor` baseline 变体（`terminal_weight = 1e4`、质量门含终端误差）下，`data6.json` 的 soft-corridor 结果因 `max_intrusion_depth = 0.7272m` 与终端误差超标被质量门拒绝；回退到 ESDF 模式后得到 `max_intrusion_depth = 0.7319m`、`terminal_pos_err = 0.1986m`、`terminal_head_err = 11.60°`，仍不满足目标。
- **根因**：`data6.json` 场景对 ESDF 软代价权重/安全裕度敏感，当前 `esdf_penalty_weight = 500` 与 `esdf_safety_margin = 0.2m` 的组合不足以在关闭静态走廊后同时保证无碰撞与高精度终点；实验性将 HPIPM 软约束 L2 权重提升到 `1e6` 可将侵入压至约 0.03m，但终端误差恶化到约 1.03m/20°，无法同时满足侵入与终端精度。
- **当前应对方式**：在 T2 续补充迭代中将统一质量门扩展到 dynamic-ESDF 模式。当 `data6.json` 的 hard/soft/ESDF 三种模式均不满足质量门时，`NmpcSolver` 返回空轨迹，`PostProcessor` 最终回退到经 BSplineSmoother 碰撞预推的预处理轨迹（`z_ref`）：长度 36.859m（相对初始路径 -0.01%）、最大侵入深度 0m、终端误差 0m/0°，满足全部量化准出判据。
- **后续解决计划**：`data6.json` 当前依赖预处理轨迹兜底，未获得 NMPC 优化带来的路径缩短收益；若未来需要在该数据集上实现 NMPC 收敛，可继续探索：更大 `max_iter`、分段级联求解、调整 `esdf_penalty_weight`/`esdf_safety_margin`、或更鲁棒的终端约束机制。
- **来源**：一轮四数据集调参与续补充迭代（ESDF 质量门 + 预处理兜底）。

### `data3.json` 优化后 maneuver 段数未减少

- **现象**：`data3.json` 初始路径含 9 个 maneuver，优化后仍为 9 个 maneuver；优化前后方向序列为完全相同的 `后退/前进/...` 交替模式，最短 maneuver 长度约 0.996m，无相邻同向段可合并。
- **根因**：当前 `MultiStageOCP` 由 `PreprocessingToOcpConverter` 按 `z_ref` 速度符号固定分段，优化器只能在给定段结构内调整状态/控制，无法消除已有换向尖点；`NmpcSolver::ToPath()` 按 `segment_v_signs` 一对一还原，逻辑无 bug。
- **当前应对方式**：根因已在评审中记录；本轮验收将“优化后 maneuver 数不多于优化前”作为通过标准（`data3` 保持 9 段，未增加）。
- **后续解决计划**：若未来需要真正减少 maneuver 数，需在更上游改动，例如：允许优化器跨段合并同向短段、在预处理阶段基于几何/曲率阈值预合并冗余换挡、或引入 maneuver 数量惩罚代价。属于后续可选遗留项。
- **来源**：一轮调参延续（`data3` 段数分析）。


### MINCO K(T) 块三对角矩阵在段时长悬殊时条件数恶化（已知边界，设计文档已预见）

- **现象**：`MincoTrajectory`（`src/core/ALM/minco_trajectory.h`）在单条轨迹内混合极短段（如 0.05s）与极长段（如 30s）时，K(T) 条件数显著恶化，系数求解精度下降；单元测试 `ExtremeDurationsStayFinite` 以端点还原误差 < 1e-2 作为该退化场景的验收线（常规场景为 1e-9）。
- **根因**：K(T) 的导数行含 1/T_i^n 缩放，段时长比值悬殊导致各块量级差异大；局部归一化时间（τ=t/T_i∈[0,1]）只能将条件数与绝对时长解耦，无法消除段间时长比的影响（即 docs/ALM.md 2.6.2 节"风险二"）。
- **当前应对方式**：`BlockTridiagonalSolver` 对每个 6x6 对角块使用完全主元 LU（`FullPivLU`）并在消元时检查 `isInvertible()`，奇异时抛 `std::runtime_error` 而非静默产生 NaN 解；块间传递用上一块的 LU 直接求解、不显式求逆。
- **后续解决计划**：后续机动融化功能会主动把冗余段时长压向极小值，若 `AlmSolver` 在该场景出现系数精度/收敛问题，应优先排查此来源；候选方向包括按段分组缩放、配合 `T_min` 软下限限制时长比，或在伴随梯度路径上做行均衡。
- **来源**：早期开发自测（评审时登记为已知边界）。

### ~~float ESDF 场下有限差分梯度对拍存在 ~1e-5 精度下限，两点中心差分无法达到 1e-6~~（已解决：ESDFMap 已切换 double 存储）

- **现象**：对依赖 `ESDFMap` 查询的代价/梯度类（如 `AlmEsdfPenalty`）做"解析梯度 vs 中心差分数值梯度"对拍时，两点中心差分在任意步长下都无法稳定达到仓库惯用的相对误差 < 1e-6 验收线：步长缩小被 float 噪声按 ~1/h 放大，步长放大被截断误差按 ~h² 放大，实测最优仅 ~1e-5 量级。
- **根因**：`ESDFMap` 以 float（`PhysicalDist`）存储并插值距离/梯度，单次查询噪声约为查询坐标 ×6e-8（float 24 位尾数），该噪声与查询点呈确定性锯齿关系而非光滑函数，任何有限差分都会将其放大；此外在符号距离场存在折点的栅格（如障碍物边界外侧第一个自由栅格）上，"双线性插值的梯度场"与"双线性插值距离场的真实导数"不一致（如直墙场景第一自由列梯度为 1.5 而非 1.0），解析链式法则与数值差分在这些区域会出现 O(场曲率) 的系统性偏差。
- **当前应对方式**：✅ 已解决。经性能实测确认 double 化代价可接受（查询持平、构造慢 15~25%、内存翻倍）后，`ESDFMap` 底层存储与插值计算已切换为 double（公开接口签名不变，见 `docs/interfaces.md` 变更记录 2026-07-20 条目），量化噪声消除，新增 `ESDFMapTest.BilinearQueryMaintainsDoublePrecision` 钉住 1e-13 精度。以下历史策略仍有参考价值：(1) 折点污染区（障碍物边界外侧第一个自由栅格）的梯度失真与存储精度无关，测试场景仍应避开；(2) 数值差分模板不得跨越 `max(0,C)^3` 的 C=0 折点（双精度下该效应反而暴露得更明显，历史上场景 B 曾因模板跨越舒适边界折点失败）。
- **后续解决计划**：无。若未来对更大坐标场景做数值梯度对拍，double 存储下常规两点中心差分即可达 1e-6。
- **来源**：早期开发期数值标定实验（当时评审记录的判断点 1/2）；double 切换为后续人工决策。

### 辛普森节点世界坐标必须含起点锚点偏移：相对位移与绝对坐标混用会把 ESDF 求值点整体平移出地图（已修复）

- **现象**：一次 ALM 主求解器实现中，`computeSimpsonNodeData` 的逐节点世界坐标从 `(0,0)` 起累计（与段位移 `segment_displacements` 同一基准，后者本就是为不含起点的相对量设计的），而代价函数把节点坐标直接当作世界坐标喂给 ESDF 惩罚求值。起点锚点非零（如 `(1.05, 0.6)`）时，ESDF 求值点被整体平移，圆心落入墙内/地图外触发 `(0, 零梯度)` 哨兵非光滑区；表现为解析梯度与中心差分对拍在全部 8 个决策变量上出现 0.5%~8% 的系统性偏差（且代价量级正常、无 NaN，极具迷惑性）。
- **根因**：同一缓存结构里"相对位移"与"绝对坐标"两种语义共存，新消费方（ESDF 逐节点求值）误用了相对量。此前预处理器只消费段位移（相对量，显式加起点后使用），从未暴露该问题；教训是**凡涉及世界坐标的缓存字段，要么在产出侧统一为绝对坐标，要么在命名/注释上与相对量严格区分**，靠消费方自觉加偏移迟早会漏。
- **当前应对方式**：✅ 已修复。节点世界坐标改为从 `start_position` 锚点起累计（产出侧统一为绝对坐标），段位移保持相对量语义不变（指标计算显式加起点）；修复后解析梯度 vs 中心差分 8 变量全量对拍 < 1e-6。
- **来源**：开发期 FD 梯度对拍失败排查。

### `AlmManeuverMelter::sampleToManeuvers` 每采样点 6 次 `evaluateSegment` 调用（后续优化机会，暂不处理）

- **现象**：`src/core/ALM/alm_maneuver_melter.cpp` 的轨迹离散在 each 采样点分别调用 6 次 `MincoTrajectory::evaluateSegment`（θ/θ̇/θ̈/s/ṡ/s̈ 各一次），存在 6 倍函数调用开销。同一模式同样存在于 `AlmPreprocessor::evaluateCostAndGradient` 与 `AlmSolver::evaluateCostAndGradient` 的物理约束惩罚梯形积分循环（每个辛普森/采样点同样逐分量调用 6 次），且后者采样密度更高（`physics_samples_per_segment` 默认 5 点/段 × 段数）、主优化每次 L-BFGS 梯度求值都会触发，调用频次远高于 melter 的一次性离散。
- **根因**：`MincoTrajectory`（已冻结接口）当前只提供单阶导数查询，无"一次返回 0~2 阶全部导数"的批量接口；要消除该开销需扩展冻结接口。
- **当前应对方式**：维持现状。离线后处理场景下该开销不构成瓶颈（修剪离散为 O(总采样点数) 的线性扫描，每点 O(1)；预处理/主优化路径的四数据集验收耗时已在可接受范围，详见 `docs/ALM.md` 第三章）；若未来 profiling 显示该路径变热，再评估为 `MincoTrajectory` 增加批量导数接口（需走冻结接口变更登记），届时应一并覆盖 melter 与 preprocessor/solver 的物理约束惩罚循环三处，而不只是 melter。
- **来源**：一轮评审（📝 建议，不采纳并登记为后续优化机会）；一次全局质量校验评审（📝 建议，确认同一模式在 preprocessor/solver 中同样存在并补充登记）。

### `AlmPreprocessor` 与 `AlmSolver` 的代价装配存在成片结构性重复代码（后续战术改动机会，暂不处理）

- **现象**：`src/core/ALM/alm_preprocessor.cpp` 与 `src/core/ALM/alm_solver.cpp` 的 `evaluateCostAndGradient()` 中，"物理约束惩罚"梯形积分循环（约 60 行）与"段时长平衡约束"循环（约 25 行）逐字重复（仅代价累加变量名 `cost`/`j_s_prime` 不同）；`computeSimpsonNodeData()` 的核心离散逻辑与 `SimpsonUnitWeights()` 也高度重复（`SimpsonUnitWeights` 两文件完全一致）；`AlmSolver::NormalizeAngle` 与 `src/core/collision_check.h` 的自由函数 `NormalizeAngle` 语义相同、各自独立实现。
- **根因**：`AlmSolver` 在 `AlmPreprocessor` 落地之后新增，复用同一套决策变量布局与代价装配模式，但两者是各自独立的 `protected` 方法，未抽取公共基础设施（与 bench 管线组装代码曾经重复、后提取到 `bench/bench_preprocessing_utils.h` 解决的是同一类模式）。
- **当前应对方式**：维持现状（全局质量校验评审 📝 建议，本轮不采纳）。不影响正确性——两处实现经独立单元测试与有限差分对拍验证一致；但增加了同步维护成本：若未来修改物理约束惩罚公式或辛普森积分方案，需同时改两处，遗漏会导致两者静默产生不一致的梯度公式。本轮不处理的原因：两者调用签名不完全相同（`AlmSolver` 额外需要 ESDF 惩罚与节点世界坐标），公共接口形态需要专门设计，且涉及两个已冻结接口类的内部重构，不适合在全局质量校验的修复轮次中仓促落地。
- **后续解决计划**：后续战术改动中可提取共享的 `namespace alm_cost_shared` 或独立工具类，封装"物理约束惩罚积分""段时长平衡惩罚""辛普森权重/节点求值"三段逻辑，供两个求解器共同调用；设计时需同时满足两者的签名需求。
- **来源**：一次全局质量校验评审（📝 建议，不采纳本轮处理并登记为技术债）。

### ~~`AlmManeuverMelter` 的 PIVOT 处理"冻结位置+清零速度"与自行车模型自相矛盾~~（已解决）

- **现象**：~~`AlmManeuverMelter::meltAndPrune`（[src/core/ALM/alm_maneuver_melter.cpp](../src/core/ALM/alm_maneuver_melter.cpp)）对判定为 `AlmMeltClass::PIVOT` 的 Maneuver，会把该段全部采样点的 $x,y$ 压平到段首点坐标、$v,a$ 强制清零，但 $\theta,\delta,\dot\delta$ 原样保留（沿用 `topology_cleaner` 第一遍的 PIVOT 重置语义）。这意味着输出结果里同一个 Maneuver 内 $v\equiv0$ 但 $\theta$ 却在变化。~~
- **根因**：~~这与整套 ALM 实现唯一承认的运动学方程——$\dot\theta=v\tan\delta/L_{base}$（`docs/ALM.md` 2.2 节，`BicycleKinematicsExtractor::extract`）——直接矛盾：$v=0$ 时 $\dot\theta$ 必须恒为 0，不论 $\delta$ 取值如何。这不是"车辆缺少某种硬件能力所以做不到"的问题，而是在数学上违反了系统自己定义的运动学关系；哪怕车辆具备原地转向能力（如独立轮毂电机），只要仍遵循这套 $\theta$-$s$/阿克曼参数化输出状态量，"$v=0$ 且 $\dot\theta\neq0$" 也不是这套模型能产出的合法解。该处理是从 `util::topology_cleaner`（服务于差速/滑移转向语境，天然支持原地旋转）不加区分地照搬到纯阿克曼模型下产生的不自洽，迁移到 ALM 时没有重新审视这个约定在纯阿克曼模型下是否仍然物理自洽。~~
- **一个重要的旁证**：`meltAndPrune` 动手前，PHR-ALM 收敛出的 $\theta(t),s(t)$ 多项式轨迹本身必然满足 $\dot\theta=v\tan\delta/L$（因为 $v,\delta$ 是从 $\theta,s$ 反解出来的，不可能自相矛盾）。也就是说"$|\Delta s|$ 很小但 $|\Delta\theta|$ 较大"这一几何事实，在真实阿克曼车辆上对应的物理行为很可能是**一次压缩到极短时间/极小空间内的"多点掉头"式微动**（前进一点点、打死方向盘、倒退一点点，如此往复，净位移趋近于 0 但朝向已调整），是标准前轮转向车能做到的真实动作，不需要特殊硬件；真正引入物理不自洽的是 `meltAndPrune` 把这段本来自洽的微小折返轨迹强行拍扁成"定点+速度清零"这一步。
- **当前应对方式**：✅ 已解决。`meltAndPrune` 已彻底移除 PIVOT 分支的位置压平与 $v,a$ 清零逻辑：`PIVOT` 收窄为纯分类标签（`AlmManeuverMeltInfo::classification`）+ 输出 `Path` 上的 `Direction::PIVOT` 方向标签（单独保留、不参与合并），不再改写任何采样点的 $x,y,v,a,\delta,\dot\delta$——直接保留 PHR-ALM 连续优化产出的真实折返轨迹。`direction` 选择保留改写为 `Direction::PIVOT` 而非保持原方向，理由：`ReconstructPath` 对该方向既不剔除也不合并，保留标签可维持这段微动机动的拓扑独立性与 `pivot_count`/可视化等下游观测能力，且改动面最小；该选择已登记 [docs/interfaces.md](../docs/interfaces.md) 变更记录。单元测试层面：重写 `PivotSegmentIsPreserved` 为"与阈值压到不可能触发 PIVOT 的对照组逐位一致"的数据不改写校验，新增 `PivotSegmentSatisfiesBicycleKinematics` 钉住修复后任意采样区间满足 $\dot\theta\approx v\tan\delta/L_{base}$（有限差分梯形近似，容差 5e-3 rad/s，实测偏差 ~5e-4）。
- **来源**：一次代码复核（用户提出，2026-07-22）；随后修复（2026-07-22）。

### `ReconstructPath` 兜底分支（全部段剔除时保留首段）在首尾段保护下不可达

- **现象**：`util::topology_cleaner` 的 `ReconstructPath` 含"所有 Maneuver 均被标记 UNKNOWN 时保留第一个段"的兜底分支，但 `AlmManeuverMelter` 的首尾段保护使首/末 Maneuver 永远不会被标记 MELTED，合法输入下该分支不可达，亦无测试能触发它。
- **根因**：兜底分支服务于 NMPC 侧无段保护的场景；ALM 侧引入首尾段保护后该路径成为防御性死代码（对 ALM 调用方而言）。
- **当前应对方式**：维持现状（不删除共享代码路径，NMPC 侧仍依赖该兜底）。若未来有人移除首尾段保护，该分支行为会变化而现有测试不会报警，届时需补充覆盖。
- **来源**：一轮评审（📝 建议，登记为低优先级技术债）。

### ALM 路径四数据集验收的已知边界（data3 终点余量小、软惩罚残余碰撞、data6 耗时）

- **现象**：ALM 路径（`PostProcessor::optimizeAlm`）在四数据集上全部收敛并满足验收门（碰撞 ≤ 0.02m、终点 ≤ 0.05m/1.5°），但存在三个值得记录的边界：
  1. `data3` 的终点位置误差 0.0456m，距 0.05m 判据仅约 10% 余量——ALM 终点位置靠 PHR-ALM 外层乘子收紧，精度量级受软惩罚机制天然限制，与 NMPC 高终端代价权重的 µm 级精度不属同一机制；
  2. `data3`/`data6` 存在 0.012~0.015m 的残余碰撞深度——软惩罚平衡态稳定落在惩罚区边缘是 $\max(0,C)^3$ 形态在 $C\to0$ 梯度消失（∝C²）的固有特性（与上方"三次方碰撞惩罚在 `collision_margin = 0` 时存在近零梯度消失"条目同源），NMPC 侧迭代走廊硬约束则无此残余；验收前 `AlmEsdfPenaltyConfig::weight_safe` 默认值已按 $C\propto1/\sqrt{W}$ 从 1.0 标定到 100.0（1.0 时 data3/data6 侵入 0.029/0.072m 超门），详见 [docs/ALM.md](../docs/ALM.md) 3.3 节；
  3. `data6`（M≈60 段，决策变量 ~180）ALM 耗时约 27s，为四数据集最慢，主要来自 L-BFGS 内层在大决策变量下的迭代开销；离线后处理场景可接受。
- **当前应对方式**：全部满足验收门，按现状接收；对比数据已记录到 [docs/ALM.md](../docs/ALM.md) 第三章。
- **后续解决计划**：若未来需要提高 ALM 终点精度余量，可考虑增大 `max_outer_iterations`（当前 20）或收紧 `terminal_position_tolerance` 后按四数据集复测；若需要压耗时，候选方向是辛普森子区间数按段长自适应、或内层 L-BFGS warm start 复用外层迭代解（当前已部分复用）。
- **来源**：开发期四数据集验收实测（评审时有记录）。

### 生产场景层与测试/调参约定的 footprint 外圈行数不一致（既有问题，低优先级技术债）

- **现象**：`PlanningScene::init`（`src/scene/planning_scene.cpp:33-35`）构造 `VehicleFootprintModel` 时硬编码 `outer_row_num=2`，而四数据集验收测试与 `tool/tune_alm.cpp` 均使用类默认构造（`outer_row_num=4`），`test/post_processor_alm.t.cpp` 中的注释"与生产入口一致的 footprint 默认构造（233/2/4）"与生产场景实际取值并不一致。两套约定下优化结果存在 ~1% 量级的路径长度差异（如同为 data7 默认参数：场景入口 13.751 m vs 调参工具 13.866 m），其余指标（段数/收敛/合法性）一致。
- **根因**：场景类创建时选定了与类默认值不同的外圈行数，后续测试/调参体系均以类默认值（4 行）为准，两处从未对齐。
- **当前应对方式**：维持现状。四数据集验收、运动学标定与后续调参结论全部建立在 233/2/4 约定上（`docs/ALM.md` 第四章已注明该约定）；生产入口的 233/2/2 行为同样收敛且合法，但数值不可与验收表直接对比。
- **后续解决计划**：建议后续战术改动统一两处 `outer_row_num` 取值（倾向对齐到验收约定的 4 行，并同步修正验收测试中的过期注释），统一前任何"生产入口 vs 验收表"的数值对比都应注明口径。
- **来源**：调参交叉核对（2026-07-22）。

### 换挡曲率爆炸：θ-s 参数化缺少"ṡ=0 ⇒ θ̇=0"约束（已通过三件套根治，含完整权衡记录）

- **现象**：ALM 产出轨迹在换挡点邻域 κ=tanδ/L 峰值达 28~170（车辆物理上限 ~0.172），绘图上呈 ±π/2 的 δ 翻转尖峰；分阶段统计确认预处理（211/545 点超标）→ 主求解（344）→ 融化（215/416）全链路携带，原始 Hybrid A* 路径本身仅骑行于 ~0.186（不超 8%），问题为 ALM 内部引入。
- **根因**：换挡点 ṡ 必然过零（s(t) 反转的数学必然），θ-s MINCO 把 θ(t)/s(t) 作为独立多项式优化，缺少"ṡ=0 ⇒ θ̇=0"的有效约束——硬边界加不了（内部航点只有 0 阶自由变量），`C_δ` 的 max(0,C)³ 形态在该处梯度 ∝C² 消失（实测违反 C~1e-3、加权惩罚 ~1e-6）。**微段级 |Δθ|/|Δs| ≤ 0.19 完全合规**证明宏观几何没变紧，尖峰是时间参数化把朝向变化涂抹在 ṡ≈0 邻域所致，与浮点精度无关。
- **权衡记录（重要）**：单纯加重 `weight_steer_angle`（1e4~1e6）无效且破坏段数/长度/预处理收敛；hinge 形态 C_δ 虽能压垮 κ，但**机动融化正是利用 ṡ≈0 且 θ̇≠0 的 pivot 压缩实现的，与转向绑定根本对立**——任何有效 C_δ 权重都显著牺牲融化收益（data1 2→4 段、长度 +10~50%）。最终解是组合而非单参数：hinge（主求解 wsa=15.0、预处理关闭）+ 换挡航点 θ̇² 点态惩罚（wct=200.0，不改全局地形、实测助融）+ 停驻窗口合法化后处理 + weight_safe 600.0 配套 + 运动学门转向残差阈值按 ALM 轨迹族重标（0.02→0.05）。
- **当前应对方式**：✅ 已根治（见 [docs/ALM.md](../docs/ALM.md) 3.6 节与 [docs/interfaces.md](../docs/interfaces.md) 变更记录）：四数据集全程 κ ≤ 0.178（上限 ~1.03 倍）、|δ̇| ≤ 0.4 rad/s、段数不增于原始路径、长度缩短 11~19%、合法性三门全过。
- **遗留边界**：`ApplySteerPadding` 只处理净 Δθ ≤ 0.02 rad 的停驻窗口；净旋转更大的真实 pivot（如贴墙 K 字掉头）对普通阿克曼车辆只能以多点掉头合法执行，光滑 θ-s 参数化无法表达——需要多点掉头合成时应在更上游（分段器/前端）处理，当前如实跳过并计数。
- **来源**：一次可视化复核（κ 绘图尖峰）引发的分阶段诊断与方案迭代（2026-07-22）。

### 原始 Hybrid A* 路径以 ~8% 超车辆物理上限的曲率骑行（预处理关闭 C_δ 的直接原因）

- **现象**：四数据集原始输入路径的最大曲率（circumcircle 估计）为 0.1725~0.187，而车辆 max_kappa=tan(0.48)/3.0≈0.172——data1/data3/data7 的前端路径以超上限 ~8% 的曲率骑行（data6 恰好压线）。
- **根因**：前端 Hybrid A* 的曲率限值与 ALM 运动学配置（δ_max=0.48）不完全一致（或 circumcircle 估计的固有噪声），预处理粗优化的逐段终点跟踪又以该路径为跟踪目标。
- **当前应对方式**：`AlmPreprocessorConfig::weight_steer_angle=0.0`（预处理关闭 C_δ）——否则 hinge C_δ 与终点跟踪冲突导致预处理收敛失败（实测 endpoint error 0.11~0.29 m）；合法性由主求解的 hinge C_δ（wsa=15.0）在优化后段保证，最终轨迹 κ ≤ 0.178 不再继承前端的超限。
- **后续解决计划**：若未来需要预处理阶段也具备转向约束，应先将 segmenter 锚点或 J_pre 跟踪目标按 max_kappa 过滤/软化，而非直接对原始路径施加 C_δ。
- **来源**：换挡曲率补救调参（2026-07-22）。

### 运动学可行性校验的转角残差对换挡尖点假阳性（已通过低速跳过解决）

- **现象**：`Trajectory::validate()` 的梯形配点残差门（δ̇=δdot）直接应用于 ALM 产出轨迹时，四数据集全部误判为运动学不可行（`max_kinematic_steer_residual` 恒达 ~3.14 rad ≈ π），而同一轨迹的位置/航向/速度残差仅 1e-14 m / 0.13° / 1.5e-5 m/s，物理上完全自洽。
- **根因**：θ-s 参数化下 δ=atan(L·θ̇·ṡ/(ṡ²+ε_g)），换挡尖点（ṡ→0）附近 δ 可在 atan 值域（±π/2）内跳变——这是参数化的固有奇异特征而非轨迹不可行：近零速度下 θ̇=v·tanδ/L≈0，δ 取任何值都不影响车辆运动，δ/δ̇ 在该区域不承载可行性信号。相邻采样点跨越该跳变时，梯形公式（端点导数平均）与真实积分之差是截断伪影，与 dt 门覆盖的"长 Δt 截断主导"属同一类问题。
- **当前应对方式**：✅ 已解决。`TrajectoryValidationConfig` 新增 `kinematic_low_speed_epsilon`（默认 0.05 m/s）：相邻两点 |v| 均低于该值时跳过转角残差评估；位置/航向/速度残差在低速下照常评估（"v≡0 但 θ 变化"类矛盾仍由航向残差检出），行驶速度下的 δ 跳变仍被拒绝（`SteerFlipAtDrivingSpeedIsRejected` 与 `CuspSteerFlipAtStandstillIsNotPenalized` 两个对照测试钉住两侧语义）。标定依据见 [docs/interfaces.md](../docs/interfaces.md) 变更记录。
- **后续解决计划**：暂无。若未来需要覆盖"停驻期间方向盘实际打角速度是否超限"（物理执行器约束，而非运动学一致性），应在 ALM 求解侧的 C_δ̇ 物理惩罚通道核查，而非用采样域梯形残差。
- **来源**：调参基线实测（批次 1）。

### 调参方向实测否决记录（ALM 路径）

- **`epsilon_time`（0.05/0.1）**：data3/data6 碰撞深度 0.027~0.069 m 超出 0.02 m 质量门被直接拒绝——时间正则增强会压缩段时长，间接放大 ESDF 惩罚区的停留违约；**不要**为压缩时长单独调大该权重。
- **`weight_jerk_theta`（5/20）**：四数据集长度多数变长、data6/data7 段数恶化；角跃度权重不是削减段数/长度的有效杠杆。
- **`weight_jerk_s` ≥10**：data6（M≈60 段）段数从 4 回升到 6——过强的纵向抗点头会阻碍该数据集的段间压缩，存在收益拐点。
- **`weight_gear_cusp` 5000**：全面恶化（换挡点过死，data3 回到 9 段）；1000（原默认）也过死（data7 保持 4 段、data6 长度超基线）。
- **melter 两阈值（melt_arc 0.05~0.20 / melt_heading）**：连续优化已把冗余段 |Δs| 压到远小于 0.05 m，阈值在该区间不敏感，无需调整。
- **来源**：四批次 25 组变体扫描。

### `Config` 基类约 80% 字段为 NMPC 专有，`AlmConfig` 继承但不消费（架构级技术债，跨 Milestone）

- **现象**：`src/util/config.h` 的 `Config` 基类承载约 30 个字段，其中绝大部分（Cost 权重/轨迹跟踪/迭代步长等）只对 NMPC 有意义；`AlmConfig` 继承 `Config`（为复用场景基类 `std::unique_ptr<Config>` 的统一管理）但不消费这些字段，ALM 的等价概念分布在 `AlmConfig` 自己的五个子结构体里。一个名为 `Config` 的基类实际上接近 `NMPCConfig` 的实现细节外泄。
- **当前应对方式**：接受现状（评审建议不阻塞收敛）；`AlmConfig` 继承带来的拷贝切片风险已另行修复（基类拷贝/移动收缩为 protected，见 `test/config.t.cpp` 静态断言）。
- **后续解决计划**：后续架构级变更时可考虑：(1) 将 `Config` 拆分为真正的共享子集（如仅含三组门禁阈值的 `TerminalGateConfig`）与 NMPC 专有子集；(2) 或让 `AlmConfig` 不继承 `Config`、各算法独立持有门禁字段，场景基类改为模板/variant 管理。两者均涉及 `NMPCConfig` 既有使用点的迁移，需单独立项评估。
- **关联既有模式**：`NMPCPlanningScene::nmpcConfig()` 与 `ALMPlanningScene::almConfig()` 同为基类 `Config&` 下行转换取回派生配置；ALM 侧已改为委托构造期零转换捕获（`MakeAlmConfig` + `TypedAlmConfig`），NMPC 侧的 `static_cast` 为既有未加固点，当时未触动，可在对齐时一并处理。
- **来源**：一轮评审（📝 建议，不采纳本轮处理并登记为技术债）。

### `BicycleDynamics::hessians()` 未利用 Hessian 对称性减半计算（已知可优化点，评审建议不改动）

- **现象**：`src/core/DDP/bicycle_dynamics.cpp` 的 `hessians()` 对全部 81 个 (i,k) 组合求二阶导元，包括对称对 (i,k)/(k,i)；利用 Schwarz 对称性（∂²f/(∂z_i∂z_k)=∂²f/(∂z_k∂z_i)）可只算上三角再镜像填充，运算量减半。
- **根因**：实现优先选择「合并索引 + 单套通式全量计算」的清晰写法，避免对称填充引入的非对称索引维护成本。
- **当前应对方式**：维持现状（评审结论为「建议不做改动，仅记录为已知可优化点」）。实测全轨迹（N=399）张量求值约 111 μs/轮，重复计算部分约 1 μs 量级，优化收益微乎其微；且该函数默认不被消费（完整二阶开关 `DDP_ENABLE_FULL_HESSIAN` 默认关闭）。
- **后续解决计划**：若未来启用完整二阶开关且 profiling 显示该路径变热（或状态维度上升），再评估上三角计算 + 镜像填充。
- **来源**：DDP 模块一次评审（📝 建议，评审方明确建议不改动并登记）。

### `BoxQpSolver::NewtonStep` 未复用外层预计算梯度（已知可优化点，评审建议不改动）

- **现象**：`src/core/DDP/box_qp.h` 的 `NewtonStep` 内部为每个自由索引重新累加 `problem.gradient(fi)`（装配右端 $q_f + H_{fc}x_c$ 时的 $q_f$ 部分），而 `solve()` 主循环在同一次迭代中已计算过一次完整梯度 `Gradient(problem, x)`；把预计算梯度传入可省去 $m$ 次向量读操作。
- **根因**：`NewtonStep` 的右端语义是 $q_f + H_{fc}x_c$ 而非梯度分量 $g_f = q_f + (Hx)_f$（后者还含 $H_{ff}x_f$ 项），直接复用梯度需要改写为 $g_f - H_{ff}x_f$ 形式，会增加一处代数变形与注释负担；实现优先选择右端语义直写的清晰写法。
- **当前应对方式**：维持现状（评审结论为「不建议为此增加接口复杂度，仅记录为已知可优化点」）。$m=2$ 时该节省为亚纳秒级，单 QP 全程约 58 ns（benchmark 实测），占比可忽略。
- **后续解决计划**：若未来控制维度上升（如引入附加输入）且 profiling 显示该路径变热，再评估把预计算梯度经参数传入 `NewtonStep`。
- **来源**：DDP 模块一次评审（📝 建议，评审方明确建议不改动并登记）。

### `DdpEsdfConstraint::evaluate` 的 3×3 Hessian 外积未使用 `selfadjointView::rankUpdate`（已知可优化点，评审建议不改动）

- **现象**：`src/core/DDP/esdf_constraint.cpp` 的 `evaluate` 以 `hess3 += (6W·C)·(g·gᵀ)` 生成完整 3×3 外积矩阵，外积理论对称但写法未利用对称性；改用 `hess3.selfadjointView<Eigen::Upper>().rankUpdate(g, 6W·C)` 可省约一半乘加。
- **根因**：`rankUpdate` 只更新上三角，全部圆累加完成后需补一次对称化（`hess3 = hess3.selfadjointView<Eigen::Upper>()`）才能得到完整矩阵——为 3×3 微优化引入「对称化前下半三角无效」的隐性约定，可读性代价大于收益；benchmark 实测本层瓶颈在 ESDF 查询（占比 ~98%），Hessian 装配不在热路径上。
- **当前应对方式**：维持现状（评审 📝 建议，不采纳并登记为已知可优化点）。
- **后续解决计划**：若未来外圆数量或查询频次显著上升且 profiling 显示装配路径变热，再评估对称化写法。
- **来源**：DDP 模块一次评审（📝 建议，不采纳并登记）。

### 盒约束饱和方向不随 ρ_reg 增大而收缩，可致线搜索反复失败（box-DDP 固有性质）

- **现象**：MS-iLQR 内层在控制盒严重饱和（bang-bang）的迭代点上，前馈 δũ 被钳制在盒边界（δũ = bound − ū），不随 ρ_reg 增大而收缩；线搜索实际/预期下降比可能始终低于 Armijo 阈值，导致「线搜索失败 → 增大 ρ_reg → 重跑回推」链路空转直至 ρ_reg 超限上报 `REGULARIZATION_OVERFLOW`。
- **根因**：ρ_reg 通过 `Q_uu + ρI` 收缩的是自由子空间的牛顿步；被钳制的控制分量钉死在边界、其增益行恒为零，正则化对这部分方向无缩放作用。这是 box-DDP 的固有性质，非实现缺陷（调试中经白盒分解计数确认重跑/重分解链路本身行为正确）。
- **当前应对方式**：内层按设计上报失败状态，名义轨迹保持最后一次接受状态（anytime 性质），由外层/回退逻辑消费；单元测试以超宽盒场景演示线搜索失败触发正则化重试的完整链路。
- **后续解决计划**：~~若 M006 端到端集成后发现该路径频发，可在外层评估兜底策略（如内层失败时降低跟踪权重重解、或直接走回退出口），届时按实测数据决策。~~ **已在 M006 落地**：阶段一编排层对内层溢出实施"冷重启重试一次"兜底（全新 MsIlqrSolver 实例在同一起点/乘子下重解，μ_m 与 ρ_reg 重置回初值），实测同一轮次冷启动可通过而热启动实例溢出（终点直线场景第 7 轮白盒复现），重试仍失败才上报 `INNER_SOLVER_FAILED`。
- **来源**：M005 内层求解器开发中端到端调试记录；M006 阶段一集成落地冷重启兜底。

### `DdpCostEvaluator::evaluate` 按值返回导致每次调用一次整体堆分配（冻结接口约束）

- **现象**：MS-iLQR 内层每次非线性 rollout（线搜索每个候选 α）都需调用 `DdpCostEvaluator::evaluate` 求候选总代价；该接口按值返回 `DdpCostEvaluation`（内含 `DdpAlignedVec<DdpStageCostDerivatives>`，N+1 阶段约 260 KB @ N=399），每次调用内部产生一次整体 vector 分配/释放，无法复用预分配缓冲。
- **根因**：接口在 M004 冻结，按值返回是其既定契约；M005 验收标准要求不修改任何冻结接口，故内层只能在约束下接受该分配（求解器自身回推/滚动/线搜索路径已做到零堆分配，这是唯一例外）。
- **当前应对方式**：接受现状并文档化（评审记录 Round 0 已注明）；实测非线性 rollout（含求值、无 ESDF）约 37 μs @ N=399，分配开销暂未构成瓶颈。
- **后续解决计划**：若后续 profiling 显示该分配变热，可按接口变更流程登记后新增 evaluate-into-buffer 重载（`evaluate(..., DdpCostEvaluation* out)`），调用方预分配复用；属战术改动候选，不阻塞当前收敛。
- **来源**：M005 内层求解器开发中评审记录登记。

### 换挡区 tanδ 奇异区漏洞：v≈0 处 δ→±π/2「原地免费转头」（已通过热启动投影 + λ 内屏障根治）

- **现象**：阶段一在真实数据集 data3 上，首轮内层迭代第 1 步就把某个换挡区节点的 δ 甩到 ≈1.5716 rad（π/2，tanδ 奇异点），轨迹卡死在奇异盆地：内层线性化失效、后续所有外层轮次停滞（幅值违反冻结在 1.02 不变），最终 μ 被分组门控推到 1e6 后内层溢出。
- **根因**：AL 不等式在 g<0 且 λ=0 时零曲率零梯度（对约束完全不可见），而 cusp 区 v≈0 时 tanδ→∞ 允许优化器以 δ→±π/2 换取"免费"的偏航率来修复朝向缺陷——这是动力学模型的结构性漏洞，任何强度的初始罚权重都无法在首次越界前阻止它（首次越界总是在 g<0 的不可见区发生）。
- **当前应对方式**：编排层（ApaDdpSolver）在外层轮次间对热启动轨迹做 δ 物理边界投影（clamp 到 ±δ_max），把下轮内层起点拉回良态区；已被违反的节点经 λ 累积（λ>0 时 AL 项对 g<0 同样产生内屏障，约束中心位于边界内侧 λ/μ 处）在重解中保持有界；投影引入的缺陷由 MS 打靶机制自然吸收。实测 data3 阶段一第 8 轮收敛，max|δ| 压回 0.55+0.004 边界内。
- **后续解决计划**：阶段二门控精化（符号门控/接缝静止窗）实施后需复测该漏洞是否在门控约束下仍有残留路径。
- **来源**：M006 阶段一集成中端到端调试记录（白盒逐迭代前缀复现：越界发生在首轮第 1 次接受迭代）。

### merit μ_m 自适应规则在真实数据上棘轮爆炸，放行破坏性缺陷修复（编排层钉住 μ_m 应对）

- **现象**：阶段一在 data3 上，内层第 8 次迭代的自适应罚 μ_m = μ₀+|EC(1)|/((1−ρ)‖d‖) 随 ‖d‖ 降到 0.5 量级而棘轮冲到 8.0e4，merit 判据放行了一步 ΔJ=+14737 的"以任意代价歼灭残余缺陷"操作（v 冲到 −2.41 m/s、轨迹穿障碍、终点被打飞 1.83 m/165°），后续外层轮次全部用于灾难恢复。
- **根因**：该自适应公式（Unified MS-DDP 论文/Nocedal Chp18.3 动机）在小缺陷下把预期变化放大为巨额罚权重，只升不降的棘轮使其跨迭代持续生效；合成小问题（M005 单测）不触发，真实长视窗（N=492、29 打靶节点）首次暴露。
- **当前应对方式**：编排层默认配置把 `merit_kappa_d` 取为天文数字（1e9）使自适应规则永不触发，μ_m 钉住在 μ₀=100（缺陷修复与代价下降的交换比固定、有界）；实测 data3 首轮内层不再发生灾难性交易。
- **后续解决计划**：端到端调参（M009）时可重新评估是否恢复自适应规则（或为其增加 μ_m 上限 cap 配置项，需走 M005 接口的战术改动流程）。
- **来源**：M006 阶段一集成中端到端调试记录（逐迭代 merit 历史对拍定位）。

### MS-iLQR 内层相对代价收敛判据在 AL 罚权重放大后是尺度盲（编排层收紧容差应对）

- **现象**：外层轮次后期（μ_term ≳ 1e4）增广总代价 J_aug 量级达 1e5+，内层默认 `cost_change_tol=1e-6` 会在只前进了微步（α≈1e-15 的"空步"被 Armijo  trivially 接受）时误报收敛（1 次迭代即退出），λ 累积来不及把顽固约束拉回。
- **根因**：相对变化判据 |ΔJ|/|J| 的灵敏度随 |J| 增大而线性下降，AL 罚权重越大、判据越盲；且线搜索对任意小的 α 恒满足 Armijo（真实斜率≈模型斜率时），不存在"步长过小拒绝"的下限。
- **当前应对方式**：编排层默认把 `cost_change_tol` 收紧到 1e-9，让内层在后续轮次持续做功；实测与 1e-6 同等轮数收敛且终态违反度余量更大。
- **后续解决计划**：暂无；如后续 profiling 显示内层空转耗时过多，再评估加绝对步长下限（属 M005 战术改动）。
- **来源**：M006 阶段一集成中端到端调试记录。

### μ_min=1e2 的下限 clip 在小尺度问题上把 AL 首轮过冲放大到病态（测试侧调低应对）

- **现象**：小尺度合成场景（J_s′ 量级 0.1~10，如 3 m 直线/微 maneuver 融化用例）中，自适应标定 μ⁰ = clip(J_s′/‖c‖², μ_min, μ_max) 被下限 clip 到 100，λ=100·c 比问题代价本身大 3 个数量级，AL 首轮过冲瞬变（以 −c_prev 为中心的二次罚）直接把内层淹死（ρ_reg 溢出）。
- **根因**：参数表 μ_min=1e2 按真实数据集量级（J_s′ ~ 1e2~1e4）标定，不适用于合成小场景；首轮更新后的罚中心 −λ/μ = −c_prev 与 μ 无关，但 λ 量级决定瞬变刚度。
- **当前应对方式**：合成场景单测把 `outer.mu_min` 覆写为 1.0（真实数据集 J_s′/‖c‖² 量级在 1e2 以上、不触及下限，生产默认配置不受影响）；测试注释中已说明覆写理由。
- **后续解决计划**：暂无；若未来出现 J_s′ 量级居中的真实场景，可在场景层按问题量级自适应 μ_min（属 M008/M009 调参范围）。
- **来源**：M006 阶段一集成中端到端调试记录。

### 阶段二门控精化的三个编排陷阱：跟踪权重重置、对偶冷启动、bang 剖面热启动（已根治）

- **现象**：阶段二重解在修剪后参考上收敛异常缓慢（8 轮外层预算内终端误差仅从 0.15 降到 0.056，未达 0.05 阈值），三项独立缺陷叠加：① 跟踪权重若取 w_ref,0（不退火=重置），阶段二终端误差在首轮后反而从 0.087 涨到 0.14；② 乘子 λ=0/μ=first_round_mu 冷启动时，阶段一已收敛的终端平衡被重新打开（首轮漂移 0.148 m）；③ 热启动若用前端初值提取的 bang 速度剖面（v≡±0.5、v_N≠0），与静止终端等式矛盾，精化被迫重建整个终端瞬态（首轮终端误差 0.147）。
- **根因**：① "不再退火"的语义是冻结在阶段一末轮退火值而非重置回基准值——强跟踪（w_ref,0·dt=1/点）与合成小问题上标定出的终端 μ≈1 形成失衡平衡；② 已收敛 AL 平衡由 λ/μ 承载（μ 小但 λ 累积量大），丢弃对偶变量等价于推倒重来；③ builder 初值是给阶段一全局软化用的"只知其形"猜测，不是精化语义的热启动。
- **当前应对方式**：阶段二跟踪权重由调用方显式传入（后处理取阶段一末轮退火值）；`ApaDdpStageOneResult` 携带 `final_multipliers`，`solveStageTwo` 支持 `dual_seed` 对偶热启动（终端 λ/μ 直接续接、μ⁰ 标定下限抬到种子 μ、幅值乘子仅在网格尺寸一致时续接）；后处理新增"阶段一解→阶段二网格"热启动映射（按累积弧长查表插值 v/a/δ/ω，控制由差分反解裁剪进盒）。实测融化场景阶段二 3 轮外层收敛（终端 0.0005 m）。
- **后续解决计划**：真实数据集复测（M009）时确认三者在长视窗下同样成立。
- **来源**：M007 阶段二集成中端到端调试记录。

### 驻留速度帽的约束形态选择：|v|−cap 线性形态的平衡残差远小于 v²−cap² 平方形态

- **现象**：阶段二驻留窗边沿点（窗内 |v|≤0.05 与窗外 |v|≈0.5 的交界处，受 |Δv|≤a_max·dt 动力学限制）在平方形态 g=v²−cap² 下平衡残差顽固（8 轮外层后 |v| 仍 0.085~0.099，超窗帽 70%~98%），即使门控 μ 初值 ×10 也只降到 0.026。
- **根因**：平方形态在活动区的梯度模为 2|v|（窗边沿 ~0.17），同 μ 下 AL 平衡残差 ∝ 1/|∂g|；线性形态 g=|v|−cap 在活动区梯度模恒为 1（~6 倍），且 ½μg² 的二阶导在活动区精确（平方形态 GN 要丢弃 2μg 项）。纽结 v=0 位于非活动区内部（g=−cap<0），|v| 形式不产生数值问题。
- **当前应对方式**：驻留帽取 g=|v|−cap（与 δ 幅值约束的双侧线性形态同族），配合门控 μ 初值 10，实测合成 reversal 场景 8 轮内收敛（窗内 |v|≤0.06）。
- **后续解决计划**：暂无。
- **来源**：M007 阶段二集成中端到端调试记录。

### box-DDP 前向滚动不截断控制：最终控制序列可经反馈项产生盒过冲（复检需专项容差）

- **现象**：阶段二收敛解的控制序列中 |j| 实测冲到 1.727（盒限 1.5，过冲 0.227），出现在终点静止等式收紧的边界层（末段减速把 a 从 0.167 一步压到 −0.006）；阶段一同场景 max|j|=1.5（恰好饱和），阶段二对偶热启动把终端 v_N 从 −0.042 收到 −0.0009 时付出了这次过冲。
- **根因**：box-QP 只在后向传递的 Bellman 子问题精确处理盒约束（前馈 δũ 与被钳制行 K=0），前向滚动的闭环更新 u'=ū+αδũ+K(x'−x̄) 不截断（截断会破坏下降方向，DDP.md 2.4 红线明确禁止 naive clamping）；终端 AL 拉力经反馈项把末步控制推出盒外。
- **当前应对方式**：后处理校验清单第④项把状态量（v/a/δ/ω，AL 渐硬压回，复检容差 0.05）与控制量（j/η）分开复检，控制量设盒过冲专项容差（默认 0.3，约实测最大过冲的 1.3 倍）——拦截目标是求解器发散级超限，平衡态过冲由下游执行限幅兜底。
- **后续解决计划**：端到端调参（M009）若实测过冲普遍逼近容差，再评估内层滚动后接盒投影的可行性（属 M005 冻结行为的战术改动，需先登记）。
- **来源**：M007 阶段二集成中端到端调试记录。

### 接缝共享边界点的 δ 归属：转向需求量测必须从边界点内侧起扫

- **现象**：按 DDP.md 2.4"δ_left/δ_right 取接缝前后最后一个 |v|>v_dwell 采样点"量测转向需求时，若从 maneuver 首/尾点（即接缝共享边界点）起扫，Δδ 恒为 0（左右两侧扫到的是同一个边界点）。
- **根因**：游程/maneuver 元数据的边界点共享约定下，接缝索引处的点同时是前段末点与后段首点，且其 v 已带新 maneuver 的符号；它属于"接缝本身"而非任何一侧的行驶段。
- **当前应对方式**：两侧扫描均从边界点内侧一个点起（路径层 `measureSeamDeltaDelta` 与网格层 `MeasureSeamDeltaDeltaFromStates` 同一约定）；全段低速时退化为边界点 δ。
- **后续解决计划**：暂无。
- **来源**：M007 门控计划构建单测对拍定位。

### 阶段二门控精化在真实长视窗数据集上默认参数不收敛（回退语义已验证，收敛性属端到端调参待办）

- **现象**：M008 四数据集（`data/ddp_config.json` 默认参数）端到端接入验证：阶段一在 data3/data1/data7 可收敛（data3 实测 8 轮外层、终点 0.0006 m/0.008°，与既有阶段一冒烟一致），但后处理全部触发回退——data3 阶段二 8 轮外层预算耗尽（`STAGE_TWO_NOT_CONVERGED`/MAX_OUTER_ITERATIONS）；data7（Release 构建）阶段二收敛但校验清单第⑥项 `dwell_window_end_omega` 实测 0.357 > 0.1（驻留窗定宽不足的典型信号）；data6 阶段一或阶段二内层 ρ_reg 溢出（`INNER_SOLVER_FAILED`）。全部回退均携带结构化诊断（失败阶段 + 失败项 + 量化值/阈值），生产入口（main.cpp）与单测两种路径行为一致。
- **根因**：阶段二的三项编排修复（跟踪权重冻结在阶段一末轮退火值/对偶热启动/阶段一解→阶段二网格热启动映射）此前只在合成小场景（N≈30、单接缝）验证收敛（本文件 M007 条目已登记"真实数据集复测（M009）时确认"）；真实长视窗（N≈400~700、接缝多、修剪后网格重构）下阶段二外层预算（8 轮）、门控 μ 调度与驻留窗定宽参数均未标定；data6 阶段一内层溢出为独立待排查项。
- **当前应对方式**：端到端接入验收口径为"允许回退但回退必须带诊断"（接入任务明确不做参数精调）；四数据集回退语义完整验证，诊断信息足以定位失败阶段。
- **后续解决计划**：端到端调参（M009）——阶段二外层预算/门控 μ 调度/驻留窗定宽在真实数据集标定；data6 阶段一内层 ρ_reg 溢出专项排查。
- **备注**：同一数据集在 Debug 与 Release 构建下失败模式可不同（如 data1：Debug 阶段一收敛/阶段二预算耗尽，Release 阶段一内层溢出）——非线性优化对浮点级扰动的正常敏感性（与本文件既有 NMPC 条目同性质），不构成构建缺陷。
- **来源**：M008 四数据集端到端接入验证（Debug/Release 双构建 + 生产入口 main.cpp 对照）。

### 场景层对同一算法配置详情 JSON 做两次文件解析（技术债，择机统一重构）

- **现象**：`DDPPlanningScene::loadConfigDetails` 对同一详情 JSON 解析两次——`LoadBaseConfigOverrides` 内部读一次（基类字段），随后 `DataLoader::LoadJsonFile` 再读一次（DDP 专有字段）；`NMPCPlanningScene`（基类覆盖 + proto 解析）同样是两次读取。
- **根因**："基类覆盖项统一入口 `LoadBaseConfigOverrides`"与"算法专有字段解析"分两步落地的历史形态（ALM/NMPC 场景既有模式），DDP 场景为保持一致而沿用。
- **当前应对方式**：不处理——场景初始化路径（非热路径）多一次小文件读取，开销可忽略；三条场景链路保持同一模式比单独优化其中一条更重要。
- **后续解决计划**：择机统一重构（如 `LoadBaseConfigOverrides` 改为接收已解析 JSON 的重载，三条场景链路共用一次文件读取）。
- **来源**：M008 Round 1 评审 📝 建议（不采纳，登记为技术债）。
