# 基于NMPC的APA模块后处理方法

<!-- TOC tocDepth:2..3 chapterDepth:2..6 -->

- [基于NMPC的APA模块后处理方法](#基于nmpc的apa模块后处理方法)
  - [1. 序言](#1-序言)
  - [2. 预处理管线](#2-预处理管线)
    - [2.1. B样条分段平滑](#21-b样条分段平滑)
    - [2.2. 纵向速度规划：基于空间域的 $v^2$ 凸优化](#22-纵向速度规划基于空间域的-v2-凸优化)
    - [2.3. 状态与控制量解析补全](#23-状态与控制量解析补全)
    - [2.4. 轨迹自适应重采样](#24-轨迹自适应重采样)
    - [2.5. 碰撞安全](#25-碰撞安全)
  - [3. NMPC 优化问题](#3-nmpc-优化问题)
    - [3.1. 优化变量](#31-优化变量)
    - [3.2. 优化目标](#32-优化目标)
    - [3.3. 约束](#33-约束)
    - [3.4. 后处理](#34-后处理)

<!-- /TOC -->

## 1. 序言

为什么混合 A* 会产生多余的换挡 (Maneuver)？车辆的有效运动轨迹被严格限制在状态空间（$\mathcal{C}$-space）的一个低维非完整约束子流形上。混合 A* 本质上是在用离散的控制量和固定的空间步长，去强行采样这个连续流形。这种离散采样会产生稀疏的晶格骨架。当最优目标点（或狭窄避障通道）恰好落在晶格的拓扑缝隙中时，混合 A* 只能通过引入换挡点来产生局部的切向位移。这里的多余换挡点往往不是物理环境逼出来的，而是采样算法分辨率不足导致的拓扑畸变。

当我们忽略算力限制，将包含多次换挡的整段初始轨迹送入连续的最优控制（NLP/NMPC）求解器时，求解器在连续的 $\mathbb{R}^n$ 空间中彻底粉碎了离散晶格。在代价函数（极度惩罚 $a^2$ 和时间浪费）的梯度力牵引下，那些为了弥补采样缝隙而产生的换挡点，理论上可以在避障超平面允许的范围内被平滑地拉直、融化，最终塌缩回那个纯粹的最优连续流形。

---

## 2. 预处理管线

### 2.1. B样条分段平滑

**核心目标：** 剥离时间，将离散网格点转化为高置信度无碰撞且 $C^3$ 高阶连续的纯几何曲线（注：碰撞规避在本节以软罚函数 $F_{collision}$ 实现，“绝对安全”在数学上无法严格保证）。采用四次样条（p=4）是为了保证最终喂给 NMPC 的 $\dot{\delta}$ 是连续曲线。以 A* 轨迹给出的换挡点为绝对物理界限，将轨迹进行严格切分，在每段内部，利用四次 B 样条（$p=4$）的凸包性，构建无奇点、且物理起步/终止边界高置信度无碰撞的几何平滑管线。

**数学建模：** 对于任意一段切分后的 Maneuver 轨迹点列，我们构建一组 B 样条控制点 $\mathbf{P} = \{\mathbf{P}_0, \mathbf{P}_1, \dots, \mathbf{P}_{N_c}\}$。为了保证平滑后的轨迹能够与相邻段完美拼接，且起步/刹停时刻的横摆角速度 $\dot{\theta}$ 为零，我们对首尾控制点施加硬约束：

- **位置与航向锚定：**
设起步真实位姿为 $pose_{start}$，终止真实位姿为 $pose_{end}$，启发式延伸步长为 $L$：
$\mathbf{P}_0 = [pose_{start}.x, pose_{start}.y]^T$
$\mathbf{P}_1 = \mathbf{P}_0 + L \cdot [\cos(pose_{start}.\theta), \sin(pose_{start}.\theta)]^T$
$\mathbf{P}_{N_c} = [pose_{end}.x, pose_{end}.y]^T$
$\mathbf{P}_{N_c-1} = \mathbf{P}_{N_c} - L \cdot [\cos(pose_{end}.\theta), \sin(pose_{end}.\theta)]^T$

因此真正送入优化求解器的变量仅为内部控制点：$\mathbf{X} = \{\mathbf{P}_2, \mathbf{P}_3, \dots, \mathbf{P}_{N_c-2}\}$。

**目标函数构建：** 构建无约束非线性优化问题：

$$\min_{\mathbf{X}} F(\mathbf{X}) = w_{data} F_{data} + w_{smooth} F_{smooth} + w_{collision} F_{collision} + w_{reg} F_{reg}$$

- **拓扑同伦走廊 ($F_{data}$)：** 在原始 A* 轨迹处提取参考控制点 $\mathbf{P}^{ref}_i$ 及其局部法向量 $\mathbf{n}_i$。仅惩罚控制点在法向上的漂移，允许切向自由滑动：

$$F_{data} = \sum_{i=2}^{N_c-2} \left( (\mathbf{P}_i - \mathbf{P}^{ref}_i) \cdot \mathbf{n}_i \right)^2$$

- **切向均匀化正则项 ($F_{reg}$)：** $F_{data}$ 只约束法向、放开切向自由度，理论上存在相邻控制点在切向上聚集甚至顺序颠倒的风险。引入一个极轻量的相邻控制点弦长正则项：

$$F_{reg} = \sum_{i} \left( \lVert \mathbf{P}_{i+1} - \mathbf{P}_i \rVert - \Delta s_{avg} \right)^2$$

其中 $\Delta s_{avg}$ 为该段控制多边形的平均弦长；$w_{reg}$ 取值应显著小于 $w_{data}$，仅起正则化兜底的作用。

- **高阶运动学平滑 ($F_{smooth}$)：** 利用 B 样条导数由控制点差分决定的特性，直接对控制多边形进行有限差分惩罚，确保曲线曲率及其导数的高阶平滑：

$$F_{smooth} = w_{1} \sum_{i} ||\Delta \mathbf{P}_i||^2 + w_{2} \sum_{i} ||\Delta^2 \mathbf{P}_i||^2 + w_{3} \sum_{i} ||\Delta^3 \mathbf{P}_i||^2$$

- **避障 ($F_{collision}$)：** 为解决非凸环境下的穿模问题，在实际生成的 B 样条曲线 $C(u)$ 上提取密集配点（如 5cm 间隔，参数 $u_m$），并采用多圆覆盖模型计算 ESDF。对于曲线上第 $m$ 个配点的第 $k$ 个子圆（偏移量 $l_k$，半径 $R_k$）的圆心绝对坐标为：$$\mathbf{C}_{m,k} = C(u_m) + l_k \begin{bmatrix} \cos(\theta_m) \\ \sin(\theta_m) \end{bmatrix}$$ 惩罚函数定义为：$$F_{collision} = \sum_{m} \sum_{k} \max \left(0, R_k - d_{esdf}(\mathbf{C}_{m,k}) + \varepsilon \right)^3$$ 其中 $\varepsilon \approx 10^{-6}$ 为纯数值容差`EPSILON_PRECISE`，仅用于防止 ESDF 双线性插值与圆心坐标变换中的浮点舍入误差引发虚假碰撞告警，此处不额外叠加安全裕度，因为外圆半径 $R_k$ 本身已超出车辆矩形轮廓边界。

**短机动段退化：** 若某个 Maneuver 段扣除 4 个锚定控制点（$\mathbf{P}_0, \mathbf{P}_1, \mathbf{P}_{N_c-1}, \mathbf{P}_{N_c}$）后可优化的内部控制点数 $N_c - 4 < 1$（即 $N_c < 5$），或该段物理弧长小于极小阈值（如 0.1m），直接旁路掉 L-BFGS 优化器：控制点退化为起止锚定点之间的线性插值（直线段），不进行样条平滑与避障优化。这类超短段本身信息量不足以支撑四次样条的高阶导数估计，强行优化反而容易在病态的稀疏控制多边形上产生数值噪声。

**侵入深度事后校验：** $F_{collision}$ 是软罚函数，收敛结果不保证零穿透，尤其在狭窄车位场景。L-BFGS 收敛后必须执行 `ValidateCollisionFree()`：重新遍历全部密集配点，计算每个子圆的最大侵入深度 $\max_m \max_k E_{m,k}$；若超过数值容忍度视为本次平滑未达到可接受安全水平，触发降级策略（提高 $w_{collision}$ 重新优化一次，或向上层报告失败、退回未平滑的原始路径分段）。

**B样条平滑的不可能三角：** 在短揉库段（$L < 2.0\text{m}$）上，以下三个目标有时无法同时满足：

- C³ 连续：B 样条段内自然满足，但硬边界条件要求首尾曲率和曲率导数也匹配，放大了曲率峰值
- $\kappa \leq \kappa_{max}$：车辆物理极限（$\kappa_{max} = \tan(\delta_{max}) / L \approx 0.18\text{ m}^{-1}$）
- 紧密跟踪初始路径：$F_{data}$ 将控制点拉向 A\* 初始路径

设某 Maneuver 段弧长为 $L$，首尾锚定切向夹角为 $\Delta\theta$。对于 $C^1$ 圆弧连接，最小曲率为 $\kappa_{min}^{C^1} = \Delta\theta / L$。对于 clamped $C^3$ 四次 B 样条，因须同时匹配首尾的 $\kappa$ 和 $d\kappa/du$，实际最小曲率 $\kappa_{min}^{C^3} > \kappa_{min}^{C^1}$。当 $L$ 很小时，即使 $\Delta\theta$ 不大，$\Delta\theta/L$ 也可能远超 $\kappa_{max}$。

### 2.2. 纵向速度规划：基于空间域的 $v^2$ 凸优化

**核心目标：** 上一步已将路径在空间上固定，这一步在一维 S 轴上生成平滑的速度剖面。如果不做这步只给初始轨迹强行赋一个恒定速度，数学上底层的NMPC确实有能力去把速度曲线算出来，但是在工程上会带来收敛速度慢的问题，在某些需要反复微调的极限泊车场景中如果初始速度给的不合理可能会导致Jacobian矩阵计算出现严重偏差进而让求解器陷入局部极小值。

**数学建模：** 不优化 $v(t)$，优化动能的代理变量：$b(s) = v^2(s)$。定义 $b_i \triangleq v_i^2$。根据物理学运动学方程 $v_f^2 - v_i^2 = 2 a \Delta s$，加速度 $a_i$ 与 $b_i$ 呈线性关系。现在固定空间步长 $\Delta s$ 上的状态量：$$\mathbf{X} = [b_0, b_1, \dots, b_N, a_0, a_1, \dots, a_N]^T$$

**优化目标：** 惩罚速度偏差、绝对加速度，并通过惩罚相邻点加速度的差值 ($\Delta a$) 来实现Jerk平滑

$$\min_{\mathbf{X}} \sum_{i=0}^{N} \left( w_v(b_i - v_{ref}^2)^2 + w_a a_i^2 + w_{\Delta a}(a_{i+1} - a_i)^2 \right)$$

*(注：此处 $(a_{i+1}-a_i)^2$ 惩罚的是空间加速度差分 $da/ds$ 的代理量，并非严格时间域 Jerk $da/dt$——真实 jerk 还需除以随 $v$ 变化的 $\Delta t$，此项只用于提供平滑先验不追求 jerk 数值本身的物理精确性。)*

**限制条件：**

- 空间域运动学等式约束 (严格线性)：$$b_{i+1} = b_i + 2 a_i \Delta s$$

- 边界条件硬约束：起步与刹停的物理法则不可违背。$$b_0 = v_{start}^2, \quad b_N = 0$$ $$a_0 = a_{start}, \quad a_N = 0$$

- 换挡点约束：若整把泊车路径由前进-倒车-前进等多段 Maneuver 拼接而成，除了全局首尾边界，所有内部换挡点对应的空间索引 $k_{cusp}$ 同样必须显式强制为静止状态：$$b_{k_{cusp}} = 0$$

- 物理与环境不等式约束 (彻底凸化)：由于当前是空间域，$s_i$ 已知。我们可以提前查表得到每个点处的曲率 $\kappa(s_i)$ 和 ESDF 危险度，生成一个静态的常数上限数组 $V_{limit}^2[i]$。$$0 \le b_i \le V_{limit}^2[i]$$ 加减速极限：$$a_{min} \le a_i \le a_{max}$$

**求解：** 由于去除了非线性的空间查找和时间耦合，Hessian 矩阵变为极致紧凑的三对角带状稀疏矩阵，直接给OSQP求解器求解即可。

**时间时间戳复原：** QP 极速求解完成后，我们得到了一条平滑的速度大小曲线 $|v_i| = \sqrt{\max(b_i, 0.0)}$。利用速度大小做后处理积分反推每个空间点对应的精确时间 $t_i$：

$$t_{i+1} = t_i + \frac{2 \Delta s}{\max(|v_i| + |v_{i+1}|,\ 10^{-3})}$$

**方向符号复原：** $b(s)=v^2$ 的 QP 本身没有方向，真正写入状态向量的带符号速度需要在此单独复原，直接复用该 Maneuver 段既有的方向符号 $\text{sign} \in \{+1,-1\}$（与现有 `NmpcSolver::Result::segment_v_signs` 同源、由 Path 的方向切分逻辑给出）：

$$v_i = \text{sign} \cdot \sqrt{\max(b_i, 0.0)}$$

### 2.3. 状态与控制量解析补全

**核心目标：** 利用前轮转角自行车模型的微分平坦特性通过纯代数解析运算为NMPC补全参考控制量 $U_{ref} = [a, \dot{\delta}]^T$ 和状态量 $Z_{ref} = [x, y, \theta, v, \delta]^T$。

**数学建模：** 直接利用四次B样条的解析导数来反推控制量。已知车辆轴距为 $L$，由阿克曼转向几何可得前轮偏角参考值：

$$\delta(t_i) = \arctan(\kappa(u_i) \cdot L)$$

前轮偏角变化率（方向盘转角速度代理）的链式推导为 $\dot{\delta} = \frac{d\delta}{d\kappa} \cdot \frac{d\kappa}{ds} \cdot \frac{ds}{dt}$。
结合速度 $v = \frac{ds}{dt}$，其原始公式为：

$$\dot{\delta}(t_i) = \frac{L \cdot v(t_i)}{1 + (\kappa(u_i) \cdot L)^2} \cdot \left( \frac{\frac{d\kappa}{du}}{\sqrt{x'^2 + y'^2}} \right)$$

为避免计算 $\frac{d\kappa}{du}$ 时的数值震荡，我们利用 B 样条基函数的解析导数提取出曲线在一维参数 $u_i$ 处的一阶、二阶、三阶导数（$x', x'', x''', y', y'', y'''$），并将其彻底代数展开。上述公式中极其复杂的曲率弧长导数项 $\frac{d\kappa}{ds}$ 可直接化简为以下公式：

$$\frac{\frac{d\kappa}{du}}{\sqrt{x'^2 + y'^2}} = \frac{ (x'y''' - y'x''')(x'^2 + y'^2) - 3(x'y'' - y'x'')(x'x'' + y'y'') }{ (x'^2 + y'^2)^3 }$$

在低速或高频采样处，速度向量模长 $x'^2 + y'^2$ 可能极小，其 3 次方会导致严重的浮点数精度截断或无穷大溢出。在 C++ 实现时将分母替换为 $\max((x'^2 + y'^2)^3, 10^{-6})$。

上述数学解析公式在 $v=0$ 时会给出 $\dot{\delta} = 0$，但在真实的自动泊车换挡点，上一段 Maneuver 的终止偏角 $\delta_{end}$ 往往不等于下一段 Maneuver 的起始偏角 $\delta_{start}$。若直接拼接，系统将要求方向盘在 $\Delta t = 0$ 的时间内完成角度突变，这需要物理上无穷大的角速度，会导致NMPC报infeasible，此处补丁的计算与注入被统一后移到下一节，作为完成全部弧长域处理之后的纯时域后处理步骤执行”。

### 2.4. 轨迹自适应重采样

**核心目标：** 将预处理生成的密集点列（约 5cm 间隔，总计数百个点）压缩，实现危险/急弯处密集，空旷直道处稀疏的自适应采样方法。

**全局维数统筹与维度预算分配** 为保证底层 HPIPM/OSQP 求解器单次求解的内存消耗有确定上界，系统为整把泊车轨迹的最终打靶维数设定硬预算 $N_{max\_pool}$（默认 444）。

- 计算全局名义激活点数：设整段泊车路径由 $M$ 段 Maneuver 组成，计算总长 $S_{total} = \sum_{j=1}^{M} S_j$。给定名义期望步长 $\Delta s_{nom}$（默认 0.15m），计算基础点数并按维度预算截断（下限 $N_{min}=4M$，为每段 4 点硬下限的全局折合；上限默认 $444-40=404$）：$$N_{base\_total} = \min\left( \max\left( \text{round}\left( \frac{S_{total}}{\Delta s_{nom}} \right), N_{min} \right), N_{max\_pool} - \text{Margin} \right)$$ 预留 Margin（默认 40）是为了容纳后续加入的原地打轮补丁点。
- 配额分发 (Quota Distribution)：按比例将算力配额下发给各个 Maneuver 段，保证长段点多，短段点少：$$N_{active\_j} = \text{round} \left( N_{base\_total} \cdot \frac{S_j}{S_{total}} \right)$$
- **配额下限与短段退化：** 四次 B 样条求三阶导数至少需要几个有效样本点，强制 $N_{active\_j} \ge 4$；若某段物理弧长小于极小阈值（如 0.1m），不再走本节的信息密度重采样，直接触发与节一致的“短机动段退化”策略（起止状态线性插值，固定给 2 个点）。
- **配额对齐：** 各段按比例四舍五入后 $\sum_j N_{active\_j}$ 与 $N_{base\_total}$ 一般不严格相等，将差值（可正可负）补给（或扣除自）物理长度最长的一段。

**段内信息密度函数与 CDF 积分** 针对第 $j$ 段 Maneuver 内部的原始采样点（索引为 $i=0, 1, \dots, I_{dense}$）：

- 连续密度函数 $\rho(s)$ 的数学定义：定义点 $s_i$ 处的采样密度由三大物理要素叠加：

$$
\begin{aligned}
&\rho(s_i) = w_{base} + w_{\kappa} |\kappa(s_i)| + w_{obs} \sum_{k} \max(0, \varepsilon - d_{esdf}(\mathbf{C}_{i,k}))^2\\
\text{where: }& w_{base} \longrightarrow \text{基础刚度，保证空旷直道上的最大物理步长不会无限长} \\
& w_{\kappa} \longrightarrow \text{曲率权重，曲率越大的弯道，密度激增，迫使 NMPC 在弯道加密打靶以防切线漂移} \\
& w_{obs} \longrightarrow \text{避障权重（二次方惩罚，对该配点挂载的全部子圆求和），当任一子圆 ESDF 距离趋近 0 时密度极速拉升}
\end{aligned}
$$

  *(注：车辆并非质点，若只用车辆几何中心/后轴中心 $s_i$ 查询 ESDF，转弯时车头即便已贴近障碍物、后轴中心到障碍物仍可能很远，导致该处密度无法被激发、采样过稀。因此这里对在计算 $F_{collision}$ 时已经算好的每个配点的全部子圆圆心 $\mathbf{C}_{i,k}$ 逐一查询 ESDF 并求和，保证车身任何部位受到威胁时该段弧长都会被立刻加密。)*

  其中 $d_{esdf}(\mathbf{C}_{i,k})$ 直接复用上面 $F_{collision}$ 评估同一批密集配点的多圆坐标 $\mathbf{C}_{m,k}$ 时已经查询过的 ESDF 距离缓存，不重复查表。

- 累积分布函数 CDF 的数值积分推导：沿着离散的密集点列，利用梯形积分法累加密度。设 $CDF_0 = 0$，对于 $i > 0$：$$CDF_i = CDF_{i-1} + \frac{\rho(s_i) + \rho(s_{i-1})}{2} \cdot \Delta s_{dense}$$ 至此，该段的总信息量为 $CDF_{end} = CDF_{I_{dense}}$。由于密度函数绝对非负，该 CDF 数组严格单调递增。

接下来计算信息步长：$$\Delta CDF = \frac{CDF_{end}}{N_{active\_j} - 1}$$ 上一步已强制 $N_{active\_j} \ge 4$，此处分母恒 $\ge 3$，不会出现除零。二分查找与线性插值：对于第 $k$ 个目标打靶点（$k = 0, 1, \dots, N_{active\_j}-1$），其目标信息量为 $CDF_{target}^{(k)} = k \cdot \Delta CDF$。由于 CDF 数组严格单调，使用 `std::lower_bound` 即可在 $O(\log I_{dense})$ 极速找到对应的密集点区间 $[i, i+1]$。计算局部插值比例因子 $\alpha$：$$\alpha = \frac{CDF_{target}^{(k)} - CDF_i}{CDF_{i+1} - CDF_i}$$ 利用 $\alpha$ 对 $[x, y, \theta, \kappa, v, a]$ 进行线性（或基于 B 样条的高阶）插值，还原该打靶点在真实物理空间中的状态。

**段内时间戳复原** 每段独立完成上述重采样后，相邻打靶点 $k$ 和 $k+1$ 之间的物理空间步长 $\Delta s_k$ 是平滑的。复用已给出的死区保护公式与方向符号复原公式，反推该段内部每个区间的精确物理耗时：

$$\Delta t_k = \frac{2 \Delta s_k}{\max(|v_k| + |v_{k+1}|,\ 10^{-3})}$$

**维度统计（不含补丁点）：** 完成全部 $M$ 段的独立重采样与段内时间戳复原后，弧长域部分的总点数为 $\sum_{j=1}^{M} N_{active\_j}$。

**换挡点padding注入** 至此，所有 Maneuver 段均已在各自独立的弧长 $s$ 域完成重采样与时间戳复原，产出的都是 $\Delta s > 0$ 的动态点列，不再需要顾及弧长域数值稳定性问题。现在才作为纯粹的时域后处理，在相邻两段的拼接处显式插入原地打轮补丁：

1. **评估转向偏差：** 计算换挡点前后的角度差 $\Delta \delta = |\delta_{start\_next} - \delta_{end\_prev}|$（$\delta_{end\_prev}$/$\delta_{start\_next}$ 来自微分平坦解出的段边界前轮转角）。
2. **计算物理耗时：** 设执行器安全角速度上限为 $\dot{\delta}_{safe}$（例如真实物理极限的 80%），则原地转向所需时间为 $T_{steer} = \frac{\Delta \delta}{\dot{\delta}_{safe}}$。
3. **Padding填充：** 若 $T_{steer}$ 大于单个打靶时间步长 $\Delta t_{min}$，则在此尖点处强行塞入 $N_{pad} = \lceil T_{steer} / \Delta t_{min} \rceil$ 个额外的静态打靶点：

- **位置与航向强锁定：** $x, y, \theta$ 严格保持不变，即 $\Delta s \equiv 0$。

- **纵向静止锁定：** $v = 0, a = 0$。
- **横向控制过渡：** 赋予常数控制量 $\dot{\delta} = \pm \dot{\delta}_{safe}$，并将状态量 $\delta$ 进行线性插值，直至与下一段无缝接合。
- **时间戳：** 每个padding步长直接硬编码为常数：$$\Delta t_{pad} = \frac{T_{steer}}{N_{pad}}$$

通过这样的处理在整个预处理管线完成并送入 NMPC 前，打靶点数被绝对固化为：$$N_{final} = \sum_{j=1}^{M} N_{active\_j} + \sum N_{pad\_j}$$ 若 $N_{final} > N_{max\_pool}$（补丁点数超出预留 Margin 的极端场景，例如接近 $180^\circ$ 的“V 型掉头”在 $\dot{\delta}_{safe}$ 较小时可能一次性需要数十个补丁点），按以下顺序兜底：

1. 压缩常规段：丢弃 Margin 富余，把常规段总配额按弧长比例重新分配并压缩到 $N_{max\_pool} - N_{pad\_total}$（下限仍为每段 4 点），按新配额对各段重新执行信息密度重采样；
2. 若压缩常规段后仍然 $N_{final} > N_{max\_pool}$，保持补丁转向总角度不变，反向增大补丁段的单步物理时长 $\Delta t_{pad}$（等价于放宽原地打轮所需的等效角速度假设），把各补丁段 $N_{pad} = \lceil T_{steer} / \Delta t_{pad} \rceil$ 循环压缩（最多 10 轮）至预算剩余额度允许的范围内。这在物理上意味着允许更激进（但仍在执行器极限内）的原地打轮速度、或忍受轻微的控制超调，以换取内存维度的绝对安全，而非静默丢弃轨迹的一部分；
3. 两级兜底后 $N_{final}$ 仍超限时，本次重采样直接判定失败（报 `final_dimension` 超 $N_{max\_pool}$）。

### 2.5. 碰撞安全

- **迭代走廊（`IterativeCorridorConstraint`）负责安全下界**：每轮 SQP 迭代以当前状态重新查询 ESDF 并重建线性化走廊，经 HPIPM 原生 slack 以 $10^8$ 量级松弛权重（`corridor_soft_quadratic_weight`/`corridor_soft_linear_weight`）实现准硬约束效果；其硬边界 `corridor_hard_margin` 默认 0.05m，用于吸收线性化截断误差；
- **ESDF 直接代价（`esdf_penalty_weight > 0`，默认 3.0 开启）负责收敛方向引导**：每次迭代基于当前实际状态经 `ApaEsdfMapAdapter` 重新查询真实（非线性化近似）距离场，提供更准确的梯度方向，帮助优化器绕开线性化走廊过于保守导致的"绕远路"或收敛慢；`esdf_safety_margin` 默认 0.1m，为低于该距离才开始生效的舒适间隙偏好。

---

## 3. NMPC 优化问题

### 3.1. 优化变量

采用状态增广的单车模型并基于多重打靶的方法进行求解：

- **状态变量：** $\tilde{\mathbf{Z}}_k = [x_k, y_k, \theta_k, v_k, \delta_k, a_k, \dot{\delta}_k]^T \in \mathbb{R}^7$
- **控制变量：** $\tilde{\mathbf{u}}_k = [j_k, \ddot{\delta}_k]^T \in \mathbb{R}^2$（纵向 jerk 与转向角加速度）

### 3.2. 优化目标

$$J = J_{target} + J_{terminal} + J_{interior} + J_{effort} + J_{smooth} + J_{esdf} + J_{slack}$$

1.全程目标牵引代价 $J_{target}$：施加对停车目标位姿 $(x_{target}, y_{target}, \theta_{target})$（即 $Z_{ref}$ 末点）的二次牵引：

$$ J_{target} = \sum_{k=0}^{N} \left[ W_{x,target} (x_k - x_{target})^2 + W_{y,target} (y_k - y_{target})^2 + W_{\theta,target} (\theta_k - \theta_{target})^2 \right] $$

实现上为各段 Q 矩阵对角项 Q(0,0)=Q(1,1)=`global_target_position_weight`、Q(2,2)=`global_target_heading_weight`（默认均 $10^{-3}$），参考向量为常量目标位姿；因参考不随 $k$ 变化，直接用标准 `QuadraticTrackingCost` 承载。

2.终端代价 $J_{terminal}$：终端段在全程目标牵引之上叠加极强的终端跟踪权重，作用于终端段的每一个打靶点，保证车辆精准停入目标车位：

$$ J_{terminal} = \sum_{k \in terminal} \left[ W_{xy,N} \lVert \mathbf{p}_k - \mathbf{p}_{target} \rVert^2 + W_{\theta,N} (\theta_k - \theta_{target})^2 + W_{v,N} v_k^2 + W_{\delta,N} (\delta_k - \delta_{ref,N})^2 \right] $$

3.内部机动段状态代价 $J_{interior}$：非终端段只对 $v$、$\delta$ 两个状态分量施加小幅值抑制，x/y/θ 权重恒为 0——避免多段同时争夺端点导致收敛困难，中间打靶点的几何形态交由 $J_{target}$ 牵引：

$$ J_{interior} = \sum_{k \in interior} \left( W_{v,int}\, v_k^2 + W_{\delta,int}\, \delta_k^2 \right) $$

4.控制输出代价 $J_{effort}$：$a$、$\dot{\delta}$ 升为状态分量后，其幅值惩罚落地为对状态分量 5/6 的标准 Q 代价，抑制执行器的无效能量消耗：

$$ J_{effort} = \sum_{k=0}^{N} \left( W_{a} a_k^2 + W_{\dot{\delta}} \dot{\delta}_k^2 \right) $$

5.平顺性代价 $J_{smooth}$：对新控制量 $[j, \ddot{\delta}]$（纵向 jerk 与转向角加速度）施加标准 R 代价（`smoothing_jerk_weight`=`smoothing_steer_accel_weight`=$10^{-1}$，`Config` 基类字段，`loadFromProto` 时同步覆盖 `PathToOcpConfig` 过渡期同名字段）。在 $j = da/dt$、$\ddot{\delta} = d\dot{\delta}/dt$ 且时间步长由静态参数 $\Delta t_k$ 给定的语义下，等价于惩罚跨 stage 的 $(a_{k+1}-a_k)^2$ 类差分——这是 NMPC 求解本身具备消除冗余换挡能力的核心机制：

$$ J_{smooth} = \sum_{k=0}^{N-1} \left( W_{j} j_k^2 + W_{\ddot{\delta}} \ddot{\delta}_k^2 \right) $$

这里不显示引入换挡次数惩罚以避免 SQP 崩溃，但每次多余的换挡都会带来换挡前速度值的大幅减小和换挡后速度值的急速增大，进而带来巨大的 jerk 惩罚。

6.ESDF 直接代价 $J_{esdf}$：每轮迭代基于当前状态经 `ApaEsdfMapAdapter` 查询真实距离场，对每个打靶点的每个外圆施加二次外点罚，负责收敛方向引导：

$$ J_{esdf} = \frac{1}{2} W_{esdf} \sum_{k} \sum_{m} \max\big(0,\; (R + m_{esdf}) - d_{esdf}(\mathbf{C}_{k,m}(\mathbf{Z}_k))\big)^2 $$

7.软约束松弛代价 $J_{slack}$：松弛变量服务于迭代走廊约束。碰撞检测通过把车辆轮廓拆分为 $M$ 个外圆并与 ESDF 距离场比较实现：每轮 SQP 迭代在当前解处重新线性化，得到每个打靶点、每个外圆一条的走廊约束行 $g_{k,m}(\mathbf{Z}) = R + m_{hard} - d_{esdf}(\mathbf{C}_{k,m}(\mathbf{Z})) \le 0$（外圆半径 $R$ 已超出车辆矩形轮廓边界，$m_{hard}$ 仅为吸收线性化截断误差的 0.05m），并以 HPIPM 原生 slack 松弛为 $g_{k,m}(\mathbf{Z}) \le \xi_{k,m}$、$\xi_{k,m} \ge 0$。

为了保证 HPIPM 底层基于 Schur 补的松弛变量算法能够完美运作且不破坏海森矩阵的极小值特性，基于罚函数的思想我们对松弛变量 $\xi$ 采用 L1 + L2 混合惩罚：

$$J_{slack} = \sum_{k=0}^{N} \sum_{m=0}^{M-1} W_{L1}\, \xi_{k,m} + W_{L2}\, \xi_{k,m}^2$$

此处的一次项承担主要的惩罚职责，只要 $W_{L1}$ 足够大，求解器只有在迫不得已时才会让 $\xi > 0$；二次项提供局部的强凸性，当车辆真的被逼入绝境导致 $\xi > 0$ 时，二次项能提供平滑递增的梯度尽量减少数值震荡。当前默认 $W_{L1}$ = `corridor_soft_linear_weight`、$W_{L2}$ = `corridor_soft_quadratic_weight`，均为 $10^8$——松弛代价极高，约束效果近似硬约束。

### 3.3. 约束

NMPC 的完整边界由等式约束（物理连续性法则）与不等式约束（工程极限与安全走廊）共同组成。在单次 SQP 迭代中：

1.非完整运动学约束：相邻打靶点之间的物理演进被转化为连续性等式约束 $$ \mathbf{Z}_{k+1} = \mathbf{F}_{RK4}(\mathbf{Z}_k, \mathbf{u}_k, \Delta t_k) $$

- 这里的 $\Delta t_k$ 并非优化变量，而是预处理管线（2.5 节）生成的静态参数向量。结合四阶龙格-库塔 (RK4)，将原本非凸的 $v \cdot \Delta t$ 强耦合处理为标准且解析求导稳定的常微分方程（`BicycleModelJerk`，7 维状态、2 维控制）。
- 起始点锚定：SQP 子问题按增量 $(\Delta x, \Delta u)$ 装配，首节点增量边界强制 $\Delta x_0 = 0$——首状态恒等于初始猜测首点（预处理 $Z_{ref}$ 的起点，即当前位姿）。

2.执行器与物理极限边界约束：利用 HPIPM 原生的 `lbx/ubx` 和 `lbu/ubu` 接口，直接对状态量和控制量施加截断：

| 变量 | 边界 | 配置字段（默认值） |
|---|---|---|
| $v$ | 按段方向单向压紧：前进 $[-0.02,\ 2.0]$、后退 $[-2.0,\ +0.02]$ m/s | `max_speed`=2.0、`boundary_velocity_slack`=0.02 |
| $\delta$ | $[-\delta_{max},\ \delta_{max}]$（车辆参数，0.48 rad） | `VehicleParams::max_steer_angle` |
| $a$ | $[-2.0,\ 2.0]$ m/s² | `accel_limit`=2.0 |
| $\dot{\delta}$ | $[-1.0,\ 1.0]$ rad/s | `steer_rate_limit`=1.0 |
| $j$ | $[-10,\ 10]$ m/s³ | `max_jerk`=10.0 |
| $\ddot{\delta}$ | $[-5,\ 5]$ rad/s² | `max_steer_angular_accel`=5.0 |
| $x,\ y,\ \theta$ | $\pm 10^4$（视为无约束） | `pose_bound`=1e4 |

3.迭代走廊碰撞约束（见 2.6 节）：每个打靶点、每个外圆一条 $g_{k,m}(\mathbf{Z}) = R + m_{hard} - d_{esdf}(\mathbf{C}_{k,m}(\mathbf{Z})) \le 0$，每轮 SQP 以当前解重新线性化，经 HPIPM slack（$10^8$ 权重）实现准硬约束，无条件始终注入。

4.松弛变量非负约束：$ \xi_{k,m} \ge 0 $（迭代走廊 slack，对应 3.2 节 $J_{slack}$）

### 3.4. 后处理

NMPC 求解的是一个固定拓扑结构的 OCP——`PreprocessingToOcpConverter` 将预处理序列按速度方向符号切分为多个 OCP stage（`v_sign = ±1` 编码前进/后退方向），求解后 `NmpcSolver::ToPath()` 再将每个 stage 1:1 还原为一个 Maneuver。StcSQP/HPIPM 求解的是固定维度的 QP，无法在迭代中改变 stage 数量——这等价于要求求解器改变问题自身的拓扑结构。

因此，入N个Maneuver一定会出N个Maneuver，即使 NMPC 在物理上将某个 stage 压平（速度趋近于零、位移极小、方向盘几乎未动），该段在数据结构上依然存在，因此这里还需要一个后处理环节来剔除几何上无意义的冗余换挡。为此引入拓扑清洗层（`util/topology_cleaner.h`），它位于NMPC求解与最终结果返回之间，不对 OCP 结构做任何修改，仅对NMPC产出的几何轨迹做压缩。配置参数如下：

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `min_arc_length` | 0.05 m | 弧长低于此值视为"极小段" |
| `pivot_delta_threshold` | 0.1 rad | 极小段中 Δδ > 此值 → PIVOT，Δδ ≤ 此值 → 可压缩段 |

首先遍历每个 Maneuver，计算物理弧长 $\Delta s$（`Maneuver::length()`）和首尾前轮转角变化量 $\Delta\delta = |\delta_{end} - \delta_{start}|$：

1. **正常段（$\Delta s \ge L_{min}$）：** 不改动。
2. **可压缩段（$\Delta s < L_{min}$ 且 $\Delta\delta \le \Delta\delta_{th}$）：** `direction` 标记为 `UNKNOWN`，作为第二遍剔除的记号。这是 NMPC 物理上无法消除但几何上无意义的冗余换挡——车辆几乎没动，方向盘也几乎没转。
3. **原地打轮段（$\Delta s < L_{min}$ 且 $\Delta\delta > \Delta\delta_{th}$）：** `direction` 改为 `PIVOT`。将所有点的 $(x, y)$ 锁定为首点坐标（车辆位置不动），$v$ 和 $a$ 强制置零，但 $\theta$ 和 $\delta$ 保留 NMPC 原值——编码旋转过程与方向盘动作轨迹。

标记完成后跳过所有 `direction == UNKNOWN` 的段，构建有效段列表。若所有段均被剔除，保留首个段作为兜底防止空路径。接下来遍历有效段列表，若相邻两段方向相同（如剔除中间的 BWD 废段后，前后两个 FWD 段相邻），拼接点序列并剔除前段尾点。而方向不同的相邻段（如 FORWARD 与 BACKWARD）不合并——它们是真实的换挡意图。此外 PIVOT 段不与任何方向合并，作为独立机动段保留。
