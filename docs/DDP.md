# 基于DDP的APA模块后处理方法

<!-- TOC tocDepth:2..3 chapterDepth:2..6 -->

- [基于DDP的APA模块后处理方法](#基于ddp的apa模块后处理方法)
  - [1. 原论文理论框架](#1-原论文理论框架)
    - [1.1. 1 DDP 与 iLQR 基础](#11-1-ddp-与-ilqr-基础)
    - [1.2. 2 控制受限 DDP（Box-QP）](#12-2-控制受限-ddpbox-qp)
    - [1.3. 3 ALTRO 增广拉格朗日框架](#13-3-altro-增广拉格朗日框架)
    - [1.4. 4 MS-DDP 多重打靶](#14-4-ms-ddp-多重打靶)
  - [2. 自行车模型下的自动泊车 (APA) 轨迹后处理方法](#2-自行车模型下的自动泊车-apa-轨迹后处理方法)
    - [2.1. 1 预处理：混合 A\* 解析、等弧长重采样与打靶节点布设](#21-1-预处理混合-a-解析等弧长重采样与打靶节点布设)
    - [2.2. 2 优化变量：七维状态链、二维控制与打靶状态](#22-2-优化变量七维状态链二维控制与打靶状态)
    - [2.3. 3 优化目标：平滑主项、退火跟踪与可选换挡代理](#23-3-优化目标平滑主项退火跟踪与可选换挡代理)
    - [2.4. 4 限制条件：Box-QP 精确控制约束、AL 状态约束与符号门控](#24-4-限制条件box-qp-精确控制约束al-状态约束与符号门控)
    - [2.5. 5 求解方法：外层 AL 循环与内层 MS-iLQR](#25-5-求解方法外层-al-循环与内层-ms-ilqr)
    - [2.6. 6 后处理：符号游程分析、拓扑修剪、门控精化与分级输出](#26-6-后处理符号游程分析拓扑修剪门控精化与分级输出)

<!-- /TOC -->

## 1. 原论文理论框架

本章所基于的 DDP（微分动态规划）类方法与姊妹方案 ALM.md 所基于的直接法（MINCO 参数化 + L-BFGS 直接转录）在方法论定位上有本质差异：直接法把状态与控制一并参数化为决策变量、以惩罚项近似动力学与物理约束，再由拟牛顿法一次性求解整个非线性规划；而 DDP 属于**间接法/打靶法（shooting method）**——只以控制序列 $U$ 为优化变量，状态由动力学前向积分隐式获得，因此任一中间迭代点的轨迹都严格动力学可行（anytime 性质，算法任意时刻中断都可输出一条可执行轨迹），且天然利用最优控制问题的时间链式结构，把一个大问题分解为沿时间轴递推的一串小问题。三篇原论文各自补上了经典 DDP 的一块缺口：Tassa et al. 解决**控制盒约束**（执行器极限）的精确处理；Howell et al. 用增广拉格朗日（AL）外层解决**一般状态约束与终点等式**，并给出**不可行状态轨迹初始化**机制；Li et al. 用多重打靶（MS）解决单打靶 DDP 对**初值敏感**的鲁棒性问题。本章依次推导三者，第二章将把它们拼装成面向 APA 后处理的完整求解器。

> Yuval Tassa, Nicolas Mansard, Emo Todorov. *Control-Limited Differential Dynamic Programming*. IEEE International Conference on Robotics and Automation (ICRA), 2014.

### 1.1. 1 DDP 与 iLQR 基础

DDP 的出发点是 Bellman 方程的局部递推。记离散动力学 $x_{k+1}=f(x_k,u_k)$、运行代价 $\ell(x,u)$，则在每一步求解

$$V(x) = \min_u \left[ \ell(x,u) + V'(f(x,u)) \right]$$

其中 $V'$ 为下一时刻的价值函数。把右端中括号内的**作用值函数（action-value）** $Q(\delta x,\delta u)$ 在 nominal 轨迹 $(\bar x,\bar u)$ 处作二阶展开（nominal 量以上横线标记），其各阶导数为

$$Q_x = \ell_x + f_x^T V'_x$$

$$Q_u = \ell_u + f_u^T V'_x$$

$$Q_{xx} = \ell_{xx} + f_x^T V'_{xx} f_x + V'_x \cdot f_{xx}$$

$$Q_{ux} = \ell_{ux} + f_u^T V'_{xx} f_x + V'_x \cdot f_{ux}$$

$$Q_{uu} = \ell_{uu} + f_u^T V'_{xx} f_u + V'_x \cdot f_{uu}$$

其中 $f_x,f_u$ 为动力学雅可比（第二章记 $A_k,B_k$），末项 $V'_x\cdot f_{xx}$ 等是**向量-张量积**（动力学二阶导数张量与价值梯度沿第一维收缩）。**是否保留这组张量项正是 DDP 与 iLQR 的唯一定义性区别**（原论文："Classic DDP requires second order derivatives of the dynamics… If these are ignored one obtains a Gauss-Newton approximation known as iterative-LQG/iLQR"）：完整 DDP 保留张量项、具有局部二次收敛率；iLQR 丢弃之、换取每轮迭代的大幅提速（收敛阶降为线性，但常数优势往往反超）。本文档的求解器默认采用 iLQR/Gauss-Newton 变体（见 1.4 节变体选择），但保留完整二阶项作为可选开关——与本章三篇论文的开源实现核查结论一致，见 1.4 节末的实现对照表。

对展开的二次型关于 $\delta u$ 求极值，得最优修正

$$\delta u^*(\delta x) = k + K\delta x,\qquad k = -Q_{uu}^{-1}Q_u,\qquad K = -Q_{uu}^{-1}Q_{ux}$$

其中 $k$ 是前馈（feedforward）项，$K$ 是免费附带的局部反馈增益（对应 ALTRO 记号的 $d_k,K_k$；本文档 $d$ 专指打靶缺陷，故前馈项在 1.4 节与第二章统一记 $\delta\tilde u$）。代回展开式即得价值的局部更新（由 $Q$ 各阶量组合而成，无需额外求导）：

$$\Delta V = -\tfrac{1}{2}k^T Q_{uu}k,\qquad V_x = Q_x - K^T Q_{uu}k,\qquad V_{xx} = Q_{xx} - K^T Q_{uu}K$$

**后向传递（backward pass, BP）**从终点 $V_x=\ell_{f,x}(x_N)$、$V_{xx}=\ell_{f,xx}(x_N)$ 出发，沿时间轴反向递推全部 $k_k,K_k$；**前向传递（forward pass, FP）**则以非线性动力学做一次真正的 rollout，并带回溯线搜索：

$$\hat u_i = u_i + \alpha k_i + K_i(\hat x_i - x_i),\qquad \hat x_{i+1}=f(\hat x_i,\hat u_i)$$

步长 $\alpha$ 从 1 回溯衰减，直到实际代价下降被接受。注意反馈项中的 $\hat x_i-x_i$ 是**实际 rollout 状态**与 nominal 状态之差，这使控制更新天然是一个沿新轨迹闭合的反馈律，而非开环修正。

**复杂度论证是 DDP 相对直接法的结构性优势**：时间链式结构把一个 $Nm$ 维的联合优化拆成 $N$ 个独立的 $m$ 维局部问题，每步只需对 $m\times m$ 的 $Q_{uu}$ 做一次分解，单轮 BP/FP 复杂度 $O(Nm^3)$（含状态矩阵运算则为 $O(N(n^3+nm^2))$），对时域长度 $N$ **线性**；而若把整个问题当作单个 $Nm$ 维 QP，稠密求解是 $O(N^3m^3)$。这也是它能承担 $N\approx400$ 步泊车轨迹的根本原因。

**正则化与工程注记。** $Q_{uu}$ 在 iLQR 下只保证半正定，必须加 Tikhonov 正则化：以 $Q_{uu}+\rho_{reg} I$ 替换 $Q_{uu}$（等价于给 $V'_{xx}$ 加 $\rho_{reg} I$ 再传播），$\rho_{reg}$ 按 **Levenberg-Marquardt 启发式**自适应：本次迭代被接受则减小 $\rho_{reg}$（向 Gauss-Newton 极限靠拢），被拒绝则增大（向梯度下降靠拢）。需要强调 $\rho_{reg}$ 与线搜索的**交互陷阱**：线搜索失败（$\alpha$ 回溯到下界仍不接受）不能简单放弃本轮迭代，而应先增大 $\rho_{reg}$ 并**重跑整个后向传递**——因为 $k,K$ 依赖正则化后的 $Q_{uu}^{-1}$，仅重跑前向传递是在同一个被拒方向上继续缩小步长，无法脱困；只有 $\rho_{reg}$ 超过上界后才判定本轮收敛失败并重启/退出。

### 1.2. 2 控制受限 DDP（Box-QP）

实际控制量总有盒约束 $b \le u \le \bar b$：对泊车而言这是执行器与运动学的硬极限（纵向跃度、方向盘角加加速度的幅值，上游又来自转角与加速度极限）。Tassa et al. 的实验结论明确指出两种直觉做法均低效，**本文档严禁使用**：

- **naive clamping（前向截断）**：BP 忽略约束、FP 把越限控制截断到边界。截断后的控制不再对应 BP 解出的下降方向，代价可能不降反升，线搜索大面积拒绝，收敛严重退化；
- **squashing（挤压函数）**：用 sigmoid 型光滑函数把无约束控制压进盒内。它引入**人工非线性**——当控制落在 sigmoid 平台区时梯度趋于零（梯度消失），求解器丢失向边界回退的信号，收敛同样退化。

正确做法是把约束移入 Bellman 局部问题：每一步 BP 解的不是无约束二次型，而是一个**盒约束 QP（box-QP）**

$$k = \arg\min_{\delta u}\ \tfrac{1}{2}\delta u^T Q_{uu}\,\delta u + Q_u^T \delta u,\qquad \text{s.t.}\ b-\bar u \le \delta u \le \bar b-\bar u$$

由于 Bellman 结构把问题拆成 $N$ 个小 QP 且相邻时间步的 QP 高度相似，原论文采用**投影牛顿法（projected Newton，active-set 子类）**求解，可热启动、开销可忽略。对一般盒约束 QP $\min \frac12 x^THx+q^Tx$，记梯度 $g = q + Hx$。定义**钳制指标集**（带梯度符号条件——只有梯度指向盒外的边界分量才被钳制）：

$$c(x) = \{\, j : (x_j = b_j \wedge g_j > 0)\ \vee\ (x_j = \bar b_j \wedge g_j < 0)\,\}$$

自由集 $f$ 为其补集。在自由子空间取牛顿步、钳制分量不动（$\Delta x_c = 0$）：

$$\Delta x_f = -H_{ff}^{-1}\left(q_f + H_{fc}\, x_c\right) - x_f$$

候选点由逐元素投影（clamp 到盒内）生成：

$$\hat x(\alpha) = \llbracket x + \alpha\,\Delta x \rrbracket_{b}$$

$\alpha$ 从 1 回溯，直至满足 **Armijo 充分下降条件**

$$\frac{f(x) - f(\hat x(\alpha))}{g^T\,(x - \hat x(\alpha))} > \gamma,\qquad \gamma = 0.1$$

若投影步越过了某自由分量的边界，则将其移入钳制集后重解。原论文给出一个关键引理：**若初始点与最优解的活动集相同，则一次牛顿步即收敛**；实测每次外层迭代的平均 Hessian 分解次数 $<2$——这正是复杂度注记的核心：$H_{ff}$ 的 Cholesky 分解**只在活动集发生变化时才重做**，否则复用上一次分解。

对 DDP 而言，box-QP 除了给出前馈 $k$，还必须**返回自由维度 Hessian 的分解** $Q_{uu,f}$，用于构造反馈增益

$$K_f = -Q_{uu,f}^{-1}\, Q_{ux}$$

即只有自由控制分量参与反馈；**被钳制的控制分量对应的 $K$ 行恒为零**——这些控制量已钉死在边界上，对状态扰动不应再作反馈响应。工程注记：相邻时间步的 QP 相似，因此第 $k$ 步的活动集应**以第 $k+1$ 步（BP 顺序的上一步）的活动集热启动**，绝大多数步一次分解即收敛。实现位置提示：`src/core/DDP/box_qp.h`。

### 1.3. 3 ALTRO 增广拉格朗日框架

> Taylor A. Howell, Brian E. Jackson, Zachary Manchester. *ALTRO: A Fast Solver for Constrained Trajectory Optimization*. IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 2019.

ALTRO 处理的标准约束轨迹优化问题为

$$\min_{X,U}\ \ell_f(x_N) + \sum_{k=0}^{N-1}\ell_k(x_k,u_k),\qquad \text{s.t.}\ x_{k+1}=f(x_k,u_k),\quad g_k(x_k,u_k)\le 0,\quad h_k(x_k,u_k)=0$$

控制盒约束仍按 1.2 节的 box-QP 精确处理；**一般不等式 $g_k\le0$、等式 $h_k=0$（含状态约束与终点对齐）则无法纳入逐步 Bellman 子问题**，ALTRO 的方案是把它们交给外层增广拉格朗日（AL）循环：把所有一般约束记为 $c(\cdot)$，构造 AL 代价

$$\mathcal{L}_A = f(X,U) + \lambda^T c + \tfrac{1}{2} c^T I_\mu\, c$$

其中 $I_\mu$ 是**对角门控罚权重阵**：对等式约束恒取 $\mu_i$；对不等式约束按

$$I_{\mu,ii} = \begin{cases} 0 & c_i < 0 \ \wedge\ \lambda_i = 0 \\ \mu_i & \text{otherwise} \end{cases}$$

门控——约束未被激活且乘子为零时完全退出代价，一旦越界或被乘子"记住"过则以二次罚压回。外层更新规则为经典 Hestenes-Powell 形式：

$$\lambda_i \leftarrow \begin{cases} \lambda_i + \mu_i c_i & \text{（等式）} \\ \max(0,\ \lambda_i + \mu_i c_i) & \text{（不等式）} \end{cases},\qquad \mu \leftarrow \phi\,\mu,\ \phi>1$$

**内层子问题**（固定 $\lambda,\mu$）是无约束轨迹优化，由 iLQR 求解：只需把运行/终端代价替换为 AL 增广后的 $\tilde\ell = \ell + \lambda^T c + \frac12 c^T I_\mu c$，其一阶导数为 $\tilde\ell_x = \ell_x + c_x^T(\lambda + I_\mu c)$（$\tilde\ell_u$ 同理），二阶导数按 Gauss-Newton 取 $\tilde\ell_{xx} \approx \ell_{xx} + c_x^T I_\mu c_x$（忽略 $c$ 自身的二阶项，与 1.1 节丢弃动力学张量项的近似一致），随后 $Q$ 各阶展开、$k,K$ 修正、线搜索与 1.1 节完全相同。工程上 $\mu$ 的初值与增长必须门控（见第二章自适应 $\mu$ 初始化与充分下降门控），否则大 $\mu$ 会使 $Q_{uu}$ 病态、内层收敛急剧变慢。

**不可行状态轨迹初始化**是 ALTRO 对本文档最关键的机制：前端（混合 A*）只给出状态轨迹 $\tilde X$、**给不出合法的控制初值**，而单打靶 DDP 恰恰要求控制初值。ALTRO 的解法是增广一组**虚拟控制** $w_k$，把动力学改写为

$$x_{k+1} = f(x_k,u_k) + w_k,\qquad w_k = \tilde x_{k+1} - f(x_k,u_k)$$

即 $w_k$ 由期望状态轨迹**反解**得到，使初始" rollout"恰好复现前端状态轨迹；同时在代价中加入 $\frac12\sum_k w_k^T R_{inf}\, w_k$（$R_{inf}$ 为大权重），并以 AL 等式约束 $w_k = 0$ 驱动虚拟控制归零。**收敛时 $w_k\to0$，问题严格退化回原始问题**——这正对应「前端只给状态轨迹、给不出控制初值」的初始化场景，本文档将其列为打靶初始化的备选机制（默认用 1.4 节的 MS 打靶节点直接注入状态初值）。

**可选增强注记（各一小段，均标注为可选、本期默认不启用）：**

- *平方根回推（square-root backward pass，可选）*：不直接传播 $V_{xx}$ 本身，而以 QR 分解传播其 Cholesky 因子，$V_{xx}$ 恒以平方因子形式出现，数值上保证对称半正定，缓解大 $\mu$ 下 AL 内层的病态；代价是每步一次 $O((n+m)^3)$ 级 QR。
- *投影牛顿抛光（可选）*：iLQR 收敛到容差附近后，再对全部决策变量做 1–2 步带 active-set 投影与乘子投影的牛顿步，把解从 iLQR 的一阶精度抛光到机器精度；对实时模块通常不必要。
- *最短时间扩展 $\tau=\sqrt{dt}$（可选）*：把每步步长平方根 $\tau_k$ 作为附加输入，动力学按 $\tau_k^2$ 缩放，代价加 $\sum R\tau_k^2$，约束等步长 $\tau_k=\tau_{k+1}$；本文档时间网格固定（见第二章），仅作未来扩展备案。

**与泊车场景的相关性证据**：原论文的平行泊车算例（Reeds-Shepp 车，$N=51$、$t_f=3\,\text{s}$）显示，直接配点法 DIRCOL 在 $v\to0$ 的换挡拐角处控制量剧烈振荡，而 ALTRO 给出平滑的 bang-bang 型控制——换挡拐角正是泊车轨迹最敏感的区段，这一对比是本文档选择 iLQR+AL 而非直接配点的重要实证依据。实现位置提示：`src/core/DDP/al_outer_loop.h`。

### 1.4. 4 MS-DDP 多重打靶

> He Li, Wenhao Yu, Tingnan Zhang, Patrick M. Wensing. *A Unified Perspective on Multiple Shooting in Differential Dynamic Programming*. IEEE Robotics and Automation Letters (RA-L), arXiv:2309.07872.

单打靶（SS）DDP 中状态恒被动力学覆写、状态初值无法直接注入，对控制初值敏感。多重打靶把状态变量分为两类：**打靶状态（shooting state）**是独立决策变量，可独立初始化与更新；**滚动状态（roll-out state）**恒被前一步动力学覆写。打靶节点集合记为 $I$，在打靶节点处动力学连续性不再自动成立，定义**缺陷（defect）**

$$d_{k+1} = f(x_k,u_k) - x_{k+1}$$

缺陷在收敛前不必为零——由此，前端给出的整条状态轨迹猜测可以**直接注入**打靶节点，初始缺陷非零正是"吃初值"能力的来源。

**缺陷感知回推**的关键修改只有一处：把 SS-DDP 展开中的 $\delta x_{k+1} = f(\bar x+\delta x,\bar u+\delta u) - f(\bar x,\bar u)$ 改为

$$\delta x_{k+1} = f(\bar x+\delta x,\bar u+\delta u) - \bar x_{k+1}$$

即扣除的是**当前打靶状态**而非 nominal 积分结果，缺陷信息由此进入价值传播。记代价一阶/二阶导数为 $q,r,Q,R,P$，动力学雅可比 $A,B$，下游价值梯度/Hessian 为 $s',S'$，得修改后的 $Q$ 各阶量（对照 1.1 节五式，多出的缺陷修正项以 $S'\bar d'$ 形式出现）：

$$Q_x = q + A^T\left(s' + S'\bar d'\right)$$

$$Q_u = r + B^T\left(s' + S'\bar d'\right)$$

$$Q_{xx} = Q + A^T S' A + s'\cdot f_{xx}$$

$$Q_{uu} = R + B^T S' B + s'\cdot f_{uu}$$

$$Q_{ux} = P + B^T S' A + s'\cdot f_{ux}$$

价值（记 $S,s,s_0$，与 ALTRO 记号的 $P_k,p_k,V$ 等价）更新为

$$S = Q_{xx} - Q_{ux}^T Q_{uu}^{-1} Q_{ux},\qquad s = Q_x - Q_{ux}^T Q_{uu}^{-1} Q_u$$

$$s_0 = s'_0 - \tfrac{1}{2}Q_u^T Q_{uu}^{-1}Q_u + s'^T\bar d' + \tfrac{1}{2}\bar d'^T S'\bar d'$$

原论文指出，上述公式中**红/蓝两类项的取舍统一了四种变体**：MS-DDP、SS-DDP、MS-iLQR、SS-iLQR（例如 SS 变体令 $\bar d'=0$ 即退化回 1.1 节经典式）。**本文档选择 MS-iLQR（Gauss-Newton）**：保留打靶状态与缺陷修正、丢弃动力学二阶张量项——动力学 Hessian 的计算代价远大于其收敛阶收益，缺陷修正项才是初值鲁棒性的来源。非线性滚动下 MS 变体仍保持局部二次收敛。

**前向滚动（rollout）有三种方案**：*线性滚动*用回推得到的线性化模型推进打靶状态，段间可并行但有预测误差；*非线性滚动*用真实动力学逐步积分，精确但严格串行；*混合滚动*让打靶状态先按线性模型推进、段内再非线性滚动，线搜索可段间并行。控制更新在三种方案下恒为 $u' = \bar u + \alpha\,\delta\tilde u + K(x'-\bar x)$；注意在打靶节点处**状态更新并非纯动力学积分**，而是带缺陷缩放项的修正（非线性滚动情形）：

$$x'_{k+1} = \bar x_{k+1} + \left[f(x'_k, u'_k) - f(\bar x_k, \bar u_k)\right] + \alpha\,\bar d_{k+1}$$

$\alpha=1$ 时上式即「从上一步新状态做真实积分、缺陷原样保留」；$\alpha<1$ 时缺陷随步长线性收缩，这是线搜索能平滑调节「缺陷修复进度」的关键。**本文档默认非线性滚动**（保证缺陷度量与线搜索接受的精确性，$n=7,m=2$ 时串行开销可忽略），混合滚动作为段间并行的可选开关。

**鲁棒化采用自适应 merit function 而非纯代价线搜索**——缺陷非零时单纯代价下降会拒绝"先增大缺陷、后修复"的有益步骤。定义

$$M(X,U) = J + \mu_m\,\|d\|$$

其中 merit 罚权重记 $\mu_m$，与 1.3 节 AL 外层的罚权重 $\mu$ 是**两个独立参数**，实现中不得共用一个变量。$\mu_m$ 按**自适应规则**更新（原论文取 $\rho=0.5,\ \mu_0=10$）：

$$\mu_m = \frac{EC(\alpha)}{(1-\rho)\,\|d\|_p} + \mu_0,\qquad \text{仅当 } \|d\|_p > \kappa_d \text{ 时更新}$$

即仅在缺陷总量超过阈值 $\kappa_d$ 时才按当前步的预期变化与缺陷范数之比重新标定权重（$(1-\rho)$ 在分母，提供安全余量），否则保持原值——避免缺陷已接近归零时权重被无意义地刷新。其中 $EC(\alpha)$ 为预期变化（expected change），采用**含缺陷效应的精确模型**

$$EC(\alpha) = \alpha\, EC_1 + \frac{1}{2}\alpha^2 EC_2$$

（二阶项带 $\tfrac{1}{2}$ 系数，与 1.1 节 $\Delta V$ 的 $\tfrac{1}{2}\delta u^TQ_{uu}\delta u$ 自洽；$EC_1,EC_2$ 由名义轨迹各阶代价导数与线性滚动得到的 $\delta x^l,\delta u^l$ 汇总，见原论文式 (19)。）

接受判据为 Armijo 型：

$$M' < \bar M + \gamma\left(EC(\alpha) - \alpha\mu_m\|d\|\right)$$

即要求 merit 的实际下降超过"预期变化扣除缺陷惩罚项"的一定比例；当缺陷较大时右端显著放松，允许暂时穿越不可行区域。

**段间惩罚法（inter-segment penalty，可选增强、强非线性问题建议启用）**：在打靶节点处加终端型二次惩罚 $\|x^-_{k+1}-\bar x_{k+1}\|^2_{Q_d}$（$x^-_{k+1}=f(\bar x_k,\bar u_k)$ 为左侧段积分终点），促使每个打靶段从**左侧**也向缺陷靠拢（而非只靠右侧段右向修正），等价于回推终点的简单修改

$$s' \leftarrow s' - Q_d\,\bar d,\qquad S' \leftarrow S' + Q_d$$

**下标对齐警告（C++ 实现极易踩坑）**：原论文式 (20) 印刷为 $s_{k+1}\leftarrow s_{k+1}-Q_d\bar d_k$，其中 $\bar d_k$ 按左端索引编号；其物理含义是**节点 $k+1$ 处的连续性缺陷**，即左邻段积分结果 $f(\bar x_k,\bar u_k)$ 与该节点打靶状态 $\bar x_{k+1}$ 之差。本文档统一采用右端索引约定：缺陷数组 $d[i] \equiv f(x_{i-1},u_{i-1}) - x_i$，则惩罚修改在同一索引上对齐为 $s[i] \mathrel{-}= Q_d\,d[i]$、$S[i] \mathrel{+}= Q_d$。SoA 内存布局下若把缺陷按左索引存、按右索引用，会造成 off-by-one 错位——建议单元测试中专设一条「单缺陷节点 + 全零其余缺陷」用例，核对惩罚恰好只作用在该节点的 $s,S$ 上。

原论文实测在强非线性问题（四足跳跃等）上显著减少迭代次数；对泊车场景，换挡 cusp 处恰是打靶节点布设位置（见第二章），该惩罚可改善换挡点两侧的缺陷收敛。

---

**三篇论文所用变体与开源实现核查（2026-07 核查，供选型背书）。** 「DDP 还是 iLQR」不能只看论文标题，必须核对其实验与官方代码：

| 论文 | 实际变体 | 证据 |
|---|---|---|
| Tassa 2014（Box-DDP） | box-QP 叠加在 **iLQR/Gauss-Newton** 上；完整二阶仅为可选开关 | 论文 Algorithm I 直接命名 `ILQG`；HRP-2 实验明确声明用 Gauss-Newton（36 自由度下二阶导"very expensive"）；官方代码 MATLAB Central #52069 `iLQG.m` 中张量项 `if ~isempty(fxx)` 才参与回推，泊车 demo `demo_car.m` 默认 `full_DDP = false`（代码注释：打开则单轮更贵、终收敛 quadratic） |
| Howell 2019（ALTRO） | 内层严格 **iLQR** | 论文式 (11)-(15) 不含 $f_{xx}$ 张量项；官方 Altro.jl README 明确 "ALTRO uses iterative LQR (iLQR)"；C++ 移植版 altro（bjack205/Optimus Ride）同为 iLQR。注意其投影牛顿抛光是直接法全牛顿步，不属于 DDP 二阶项 |
| Li 2023（MS-DDP） | 框架**同时统一两者**：红项（二阶张量项）取舍即 MS-DDP 与 MS-iLQR 之分 | 论文式 (8) 含 $s'\cdot f_{xx}$；Fig 3 实测：一阶恒为线性收敛，二阶+非线性滚动恒达局部二次收敛；官方代码 github.com/heli-sudoo/Multiple-Shooting-DDP（MATLAB，借 CasADi 计算二阶导） |

两点结论：(i) 三篇论文的**工程部署形态全部默认 iLQR**，其共同顾虑是"高维刚体系统的动力学 Hessian 太贵"；(ii) 该顾虑在本文档场景**不成立**——状态仅 7 维、动力学为解析三角/多项式链，$s'\cdot f_{xx}$ 张量收缩每步约 $n^2(n+m)\approx 441$ 次乘加，与 $A^TS'A$ 同量级，故第二章把完整二阶项留作"买得起的二次收敛"可选开关（见 2.5 节第 1 步末尾注记）。

三篇论文的机制在第二章拼装为一个整体求解器：**内层用 MS-iLQR（Gauss-Newton，1.4 节）做缺陷感知回推与非线性滚动**，消化混合 A* 注入的状态初值；**每个 BP 步用 box-QP（1.2 节）精确处理控制盒约束**，活动集沿 BP 顺序热启动；**一般状态约束、ESDF 避障与终点对齐交给 ALTRO 式 AL 外层（1.3 节）**，其不可行初始化机制作为打靶初始化的备选；正则化 $\rho_{reg}$、线搜索与 merit 接受判据按 1.1/1.4 节的工程注记交互实现。由此得到的求解器天然满足「轨迹恒动力学可行、初值直接可注入、控制硬限不软化」三项 APA 后处理的核心诉求。

## 2. 自行车模型下的自动泊车 (APA) 轨迹后处理方法

本章将求解内核从 ALM 方案的「多项式参数化 + L-BFGS」彻底替换为「多重打靶 DDP + 增广拉格朗日外层」，底盘模型同样采用阿克曼自行车模型，输入同样是带档位标志的混合 A* 轨迹。方法的整体路线为三阶段：**阶段一**（全局软化 DDP）以软跟踪初值 + 跃度主导代价 + ESDF/物理约束 + 终点对齐求解一整条 39.9s 的连续轨迹，允许无效 maneuver（如无意义的「停-倒-停」微动）在连续优化内部被「融化」——速度全程不变号地穿过原换挡点；**后处理**检测并修剪残余微 maneuver；**阶段二**（门控精化 DDP）按修剪后的 maneuver 序列施加符号门控并热启动重解，输出最终轨迹。后处理的输出采用分级结构：阶段二不可用（未收敛/未过合法性门）时，降级输出已收敛的阶段一解（经修剪 + 驻留插入）；两个候选共用同一套合法性门，均不过才回退原始混合 A* 路径。本章理论基础来自三篇论文：

> Y. Tassa, N. Mansard, E. Todorov. *Control-Limited Differential Dynamic Programming*. ICRA 2014.

> T. A. Howell, B. E. Jackson, Z. Manchester. *ALTRO: A Fast Solver for Constrained Trajectory Optimization*. IROS 2019.

> H. Li, W. Yu, T. Zhang, P. M. Wensing. *A Unified Perspective on Multiple Shooting in Differential Dynamic Programming*. arXiv:2309.07872 / IEEE RA-L.

### 2.1. 1 预处理：混合 A* 解析、等弧长重采样与打靶节点布设

**输入规格。** 混合 A* 输出约 400 个离散点，标称间距 0.05m、总长度 20m 量级，每点携带档位（前进/倒退）标志。与 ALM.md 2.1 相同，直接使用原始点列既引入采样噪声又无法对接固定步长动力学，必须先做规范化剥离。

**等弧长重采样与固定步长。** 对原始点列做等弧长重采样到**恰好 0.05m**（必要时全长归一），得到 $N_p=400$ 个参考位姿 $(x_{ref,k}, y_{ref,k}, \theta_{ref,k})$，$k=0,\dots,N_p-1$。步长按名义泊车车速 $v_{nom}\approx 0.5\text{m/s}$ 选取：

$$dt = \frac{0.05\text{m}}{v_{nom}} = 0.1\text{s}, \qquad N = N_p - 1 = 399 \text{ 步}, \qquad T_{total} \approx 39.9\text{s}$$

**固定 $dt$ 的含义必须说清**：与 ALM 方案不同，本方案的段时间不再是优化变量，因此代价中没有显式时间项（$\epsilon_T T_s$ 无对应物），融化动力全部来自 2.3 节的跃度-跟踪权衡；时间效率交由下游速度重规划模块兜底，并非本模块目标。ALTRO §III-C 的 $\tau=\sqrt{dt}$ 最短时间扩展（每步附加输入、动力学按 $\tau^2$ 缩放）留作未来可选，本期不实现。

**maneuver 分段与 cusp 检测。** 按档位符号扫描参考点列，方向反号处记为换挡尖点，构成 cusp 集 $\mathcal{C}$；相邻 cusp 之间为一个 maneuver，记录其方向 $s_m\in\{+1,-1\}$、位移 $\Delta s_m$、朝向变化 $\Delta\theta_m$。**此处与 ALM.md 2.1 存在结构性差异，必须显式标注**：ALM 中换挡点是多项式段的结构性连接点，被强制施加 $\dot s_k=0$ 硬边界，因此连续优化最多把无效 maneuver 压平成「停-微动-停」，无法真正消除；本文档中 **cusp 处不施加 $v=0$ 硬边界**，cusp 仅用于三件事——初值提取、打靶节点布设、跟踪权重排布（见 2.3 的退火调度）——速度 $v$ 允许全程不变号地连续穿过原 cusp。这才是「在连续优化内消除换挡」的结构性前提，与 ALM 中「所有 maneuver 共享同一全局 $K(T)$ 系统、梯度可跨换挡点传递」的设计动机同源但更进一步：我们连过零点的结构钉死也一并拆除。

**初值提取。** 由参考位姿差分提取名义状态与控制初值：

$$v_k^{(0)} = s_{m(k)}\cdot\frac{\Delta s_k}{dt}, \qquad \kappa_k = \frac{\mathrm{wrap}(\theta_{ref,k+1}-\theta_{ref,k})}{\Delta s_k}, \qquad \delta_k^{(0)} = \mathrm{atan}(L_{base}\,\kappa_k)$$

$a_k^{(0)}$、$\omega_k^{(0)}$ 由 $v^{(0)}$、$\delta^{(0)}$ 经平滑差分（如三点中心差分）得到；$j_k^{(0)}=\eta_k^{(0)}=0$。全部初值裁剪进 2.4 节的盒约束内，避免第一轮迭代就触发剧烈的约束修正。

**MS 打靶节点布设。** 打靶节点集 $\mathcal{I}$ = {每 $n_s=25$ 步一个节点} $\cup$ {全部 cusp 索引} $\cup$ {末点 $N$}，共约 $399/25 + |\mathcal{C}| + 1$ 个。打靶节点上的状态是独立决策变量，以 A* 位姿（加上述初值）初始化，**允许初始缺陷** $d_k \neq 0$——这正是多重打靶相对单打靶的核心优势：混合 A* 只给状态、给不出严格动力学一致的轨迹，打靶状态可以直接吸收这份「只知其形」的初值。实现备注：ALTRO 式不可行初始化（增广虚拟控制 $w_k$，$x_{k+1}=f(x_k,u_k)+w_k$ 并加代价 $\frac{1}{2}\sum w_k^T R_{inf} w_k$ 与约束 $w_k=0$）在数学本质上与 MS 打靶缺陷**同构**——$w_k$ 扮演的正是缺陷 $d_k$ 的角色。既然本方案已实现带精确 EC 与 merit function 的 MS-DDP 框架，$w_k$ 增广**明确不实施**（工程修订，2026-07-28）：再引入它只会白白增加 $n\times N\approx 2.8\times10^3$ 个决策变量与一套冗余 AL 约束，不带来任何额外的初值吸收能力。

**ESDF 与空心边界外圆集准备。** 完全复用 ALM.md 2.4 的构造：仅在 OBB 最外层网格构建空心边界外圆集 $\{C_1,\dots,C_{N_c}\}$（半径 $r_{outer}$），双层裕度 $margin_{safe}=0.02\text{m}$（纯数值浮点比较裕度，不与 ESDF 分辨率挂钩）、$margin_{comf}=0.10\text{m}$；`outer_row_num` 是独立于 NMPC 生产配置的可调超参（$2\to12$ 圆 / $4\to26$ 圆），按「求解耗时 vs 贴库精度」实测定夺。继承同一条警告：ESDF 取值与梯度必须使用**同一组插值节点**（discretize-then-differentiate），否则数值梯度与代价评估不一致，DDP 后向传递中的 $Q_x$ 会出现系统性偏差，破坏下降方向。约束形态与 AL 处理方式见 2.4 节。

### 2.2. 2 优化变量：七维状态链、二维控制与打靶状态

**状态与控制。** 取状态 $x_k = [x,\ y,\ \theta,\ v,\ a,\ \delta,\ \omega]^T \in \mathbb{R}^7$、控制 $u_k = [j,\ \eta]^T \in \mathbb{R}^2$（$j$ 为纵向跃度，$\eta$ 为前轮转角加加速度）。连续动力学为七式：

$$\dot{x} = v\cos\theta, \quad \dot{y} = v\sin\theta, \quad \dot{\theta} = \frac{v\tan\delta}{L_{base}}, \quad \dot{v} = a, \quad \dot{a} = j, \quad \dot{\delta} = \omega, \quad \dot{\omega} = \eta$$

离散化采用半隐式 Euler（先更新积分链顶端，再逐级代入）：先 $a^+ = a + j\,dt$、$\omega^+ = \omega + \eta\,dt$；再 $v^+ = v + a^+ dt$、$\delta^+ = \delta + \omega^+ dt$；再 $\Delta\theta = v^+\tan(\delta^+)\,dt/L_{base}$、$\theta^+ = \theta + \Delta\theta$；最后 $x^+ = x + v^+\cos\theta_{mid}\,dt$、$y^+ = y + v^+\sin\theta_{mid}\,dt$，其中 $\theta_{mid} = \theta + \Delta\theta/2$。**位移更新必须用中点朝向角 $\theta_{mid}$ 而非上时刻 $\theta$**（工程修订，2026-07-28）：若用旧 $\theta$，曲线运动下每步产生系统性截断偏置（轨迹向外侧螺旋漂移），在 $dt=0.1\text{s}$、约 40s 的长视窗上误差是带符号累积的；中点格式使旋转达到二阶精度、偏置归零，代价仅是 $x^+,y^+$ 行雅可比增密（经 $\theta_{mid}$ 依赖链条上游的 $v,a,\delta,\omega,j,\eta$，见 2.5 节），单步仅增加十余次乘加。退而求其次可用 $\theta^+$（半隐式一致、实现略简），但中点格式的无偏性最好，本文档取中点格式为最终约定。完整的解析雅可比 $A_k, B_k$ 在 2.5 节给出。

**选择理由四条（对照设计决策记录 B2）：**
(i) $v(t)$、$\delta(t)$ 均为 $C^2$ 光滑（跃度/角加加速度有界），抗点头、抗方向盘异响，与 ALM 方案的 min-jerk 目标在物理意图上完全对齐；
(ii) $\delta$ 为**显式状态**，彻底避免 ALM.md 2.2 的 0/0 奇异除法与 $\epsilon_g$ 分母正则化——ALM 中 $\delta$ 必须由 $\arctan(L_{base}\dot\theta/\dot s)$ 反解、$\dot\delta$ 需链式求导压分母，而本方案中 $\delta$ 由 $\eta$ 积分而来，换挡点 $v\to0$ 处动力学依然光滑有界，无任何分支判断与正则化常数需要标定；
(iii) $\dot\theta$ **不列为状态**：它由 $(v,\delta)$ 代数决定，列为状态只会引入冗余与不一致；
(iv) **档位不是决策变量**：采用 Reeds-Shepp 有符号速度观点，$v$ 变号即换挡，后处理（2.6 节）由 $\mathrm{sign}(v)$ 恢复档位序列——这避免了混合整数规划，也是融化机制能成立的前提。

**其余决策变量。** 除各步 $x_k, u_k$ 外，决策变量还包括：MS 打靶节点上的打靶状态（见 2.1，可独立初始化、收敛前允许缺陷）；AL 外层的对偶变量 $\lambda, \mu$（见 2.4/2.5）。**可选第 8 状态 $\ell$**（累计行驶长度，工程变体，默认关闭）：$\ell_{k+1} = \ell_k + \sqrt{v_k^2 + \varepsilon_\ell}\cdot dt$，配终端不等式 $\ell_N \le (1+\rho_{len})L_0$ 作长度守卫；$\sqrt{\cdot+\varepsilon_\ell}$ 保证 $v=0$ 处可微。仅在实测出现长度蠕变超标时启用。

**维度与规模注记。** $n=7$，$m=2$，$N=399$。状态维数虽比 ALM 的多项式系数空间小得多，但 DDP 的复杂度对 $n$ 是三次的（见 2.5 复杂度评估），选择 7 维而非更高维积分链（如再积一档 snap）正是在光滑性与单次扫描代价之间的工程折中：$n=8$ 会使每步回推矩阵运算量增约 40%，而收益（snap 有界）对 APA 低速场景并不显著。

### 2.3. 3 优化目标：平滑主项、退火跟踪与可选换挡代理

总代价取标准轨迹优化形式：

$$J = \ell_f(x_N) + \sum_{k=0}^{N-1} \ell(x_k, u_k)$$

其中 $\ell$ 由下文平滑项、跟踪项、可选换挡代理项与 2.4 节 AL 增广约束项共同组成；终端 $\ell_f$ 见本节末。

**平滑主项（融化动力来源）。**

$$\ell_{smooth,k} = \frac{1}{2}\left( w_j\, j_k^2 + w_\eta\, \eta_k^2 \right) dt$$

**融化动力的离散论证。** 考察一个「停-倒-停」无效 maneuver：其两端 $v, a$ 归零，需在 $T = m\cdot dt$ 内完成净位移 $\Delta s$。对最小跃度解（边界 $v=a=0$ 的五次曲线族），离散跃度代价的量级为

$$J_{min} = \sum_k \frac{1}{2} w_j j_k^2\, dt \;\sim\; w_j\,\frac{\Delta s^2}{T^5}$$

即代价随 $T^{-5}$ 陡峭增长——与 ALM.md 2.6.1 的连续时间结论是同一物理直觉的离散版本。对本模块而言关键推论是：固定 $dt$ 网格下 $T$ 不能收缩，无效 maneuver 唯一的降压方向是 $\Delta s \to 0$，即位移本身被压没——v 全程不变号穿过原 cusp，maneuver 在连续优化内部「融化」。注意这里 $T^{-5}$ 是有理函数级增长而非指数增长，但已足够驱动梯度方向的压缩。

**同伦保持跟踪项。**

$$\ell_{track,k} = \frac{1}{2} w_{ref,k}\, \|p_k - p_{ref,k}\|^2\, dt + \frac{1}{2} w_\theta\, \mathrm{wrap}(\theta_k - \theta_{ref,k})^2\, dt$$

其中 $p_k=[x_k,y_k]^T$。$w_{ref,k}$ 采用**退火调度**：外层迭代第 0 轮取较高初值 $w_{ref,0}$（保证首轮解紧贴 A* 同伦类、不穿越障碍区），之后 maneuver 内部点按 $w_{ref,0}\cdot\gamma_{anneal}^{r}$ 逐轮衰减（$r$ 为外层轮次，$\gamma_{anneal}<1$），而**首/末 maneuver 与保留锚点不衰减**——首末段承载起点状态与终点语义，锚点承载必须保持的同伦特征。**融化是否发生的量级平衡式**：保留 maneuver 的平滑代价 $\sim w_j\Delta s^2/T^5$ vs 融化后偏离参考的跟踪代价 $\sim w_{ref}\Delta s^2\, n_{pts}\, dt$。两侧同阶正比于 $\Delta s^2$，故融化决策在一阶近似上与 maneuver 大小无关，完全由权重与可用时长 $T$ 的对比决定——退火正是通过逐步压低右端系数，把「是否值得保留」的裁决权渐进地交还给平滑项。**有用 maneuver 不会被误融**，原因有二：其位移是绕障或终点对齐所必需，一旦压缩，ESDF 避障与终点对齐（2.4 节）作为 AL 渐硬约束会产生不可承受的惩罚；而跟踪项只是软项，系数再退火也压不过 AL 约束的渐硬增长。

**可选换挡代理代价 $\ell_{shift}$（工程变体，非默认配置）。** 若退火完成后仍残留顽固的虚警 maneuver，可启用显式换挡代理项。定义平滑符号门 $\sigma_\beta(z) = \frac{1}{2}(1+\tanh(z/\beta))$，则

$$\ell_{shift,k} = w_g\left[ \sigma_\beta(v_k)\,\sigma_\beta(-v_k^+) + \sigma_\beta(-v_k)\,\sigma_\beta(v_k^+) \right], \qquad v_k^+ = v_k + a_k\, dt$$

它是**纯状态代价、处处可微**，每次 $v$ 由正转负或由负转正穿过零点附近时产生量级 $w_g$ 的惩罚。注意此处 $v_k^+ = v_k + a_k\,dt$ 是**显式一步预测**（不含控制 $j$ 的 $dt^2$ 修正），并非 2.2/2.5 节动力学链中的 $v^+$——该写法刻意让 $\ell_{shift}$ 不依赖控制量，求导时按纯状态函数处理即可，不得代入动力学链的 $v^+$ 表达式。该项非凸，故 $\beta$ 需连续退火：$0.3 \to 0.05\ \text{m/s}$，由宽门逐渐收窄到速度滞回阈值附近。默认关闭，仅作为退火兜底手段启用，启用时应在参数表中显式记录。**实测结论（2026-07-30 效果攻坚）**：本项连同 β 退火调度已完整实现并在四数据集上逐档实测，两种曲率形态（精确/PSD 投影）× 七档权重全部出局（cusp |v| 不响应且普遍失稳），方向证伪——详见 3.7 节；机制保留在代码中（默认关闭），不建议后续再试。

**终端与起点。** 终端约束（AL 等式，硬指标 $0.05\text{m}/1.5^\circ$，具体机制见 2.4/2.5）：位置 $x_N, y_N$、朝向 $\mathrm{wrap}(\theta_N - \theta_{target})$、以及 $v_N = a_N = 0$（泊车必须静止收尾）；$\delta_N, \omega_N$ 自由（停稳后前轮转角无物理要求，留给下游回正逻辑）。起点 $x_0$ = 车辆当前状态**硬固定**——DDP 的前向滚动天然钉死初态，无需任何约束机制，这是打靶法相对配置法的免费红利。

### 2.4. 4 限制条件：Box-QP 精确控制约束、AL 状态约束与符号门控

**控制盒约束（Box-QP 精确处理）。** $|j_k| \le j_{max}$、$|\eta_k| \le \eta_{max}$ 在每步后向传递中由盒约束 QP 精确处理（算法见 2.5）。**严禁使用 clamping（前向传递截断）或 squashing（sigmoid 压缩）**：Tassa 等的实验结论表明，clamping 会破坏下降方向、K 行置零的直觉做法低效，squashing 引入人工非线性导致收敛退化；本方案沿其结论直接使用 box-QP，被钳制控制对应的反馈增益行恒为零，由 QP 返回的自由集 Hessian 分解自然处理。

**状态幅值约束（AL 不等式，光滑平方形态）。**

$$v_k^2 - v_{max}^2 \le 0, \qquad a_k^2 - a_{max}^2 \le 0, \qquad \omega_k^2 - \omega_{max}^2 \le 0, \qquad \delta_k - \delta_{max} \le 0,\quad -\delta_k - \delta_{max} \le 0$$

**必须强调 $v$ 的幅值（平方）形式**：若写成盒约束 $v \ge 0$ 或 $-v_{max}\le v \le v_{max}$ 并交给 box-QP，$v\ge0$ 会把倒车直接禁掉、$v$ 的双侧盒约束则会破坏「$v$ 过零自由」这一融化前提；平方形态在 $v=0$ 处光滑且对符号中性，是唯一与 Reeds-Shepp 观点相容的写法。其代价是平方形态在边界处梯度只含单侧信息，故交由 AL 外层渐硬处理而非内层精确钉死——APA 车速余量充足，渐硬足够。

**ESDF 避障（AL 不等式，双层 margin 双权重）。** 对每个外圆 $i$（圆心世界坐标 $P_i(x_k)$）：

$$c_{safe,i,k} = r_{outer} + margin_{safe} - \mathrm{ESDF}(P_i(x_k)) \le 0, \qquad c_{comf,i,k} = r_{outer} + margin_{comf} - \mathrm{ESDF}(P_i(x_k)) \le 0$$

计入 $\mathcal{I}_{obs,k} = W_{safe}\Phi(c_{safe,\cdot,k}) + W_{comf}\Phi(c_{comf,\cdot,k})$，$\Phi(\cdot)$ 复用 ALM.md 2.4 的光滑外点罚形态（$\max(0,\cdot)$ 的光滑平方化），取值与空间梯度共用同一组 ESDF 插值节点；时间轴上可按 stride=1~2 抽样检查（stride=2 时查询量减半，需以 $margin$ 吸收采样间隙风险）。**为何走 AL 而非硬约束**：与 ALM.md 2.4 的论证一致——碰撞代价是逐圆软惩罚标量和，外圆数量增加只带来线性代价增长，不会像 NMPC 式逐点逐圆硬不等式那样引发约束规模爆炸；且 ESDF 惩罚在不可行区域外的梯度信号天然引导轨迹离障，AL 外层负责把残余违例渐硬压回。

**终点对齐（AL 等式 + 自适应初始化 + 门控增长）。** 终点位置/朝向/静止条件以 AL 等式收紧：$\ell_{AL} \supset \sum_i \lambda_i c_i + \frac{\mu_i}{2} c_i^2$。初始罚权重不设任意常数，复用 ALM.md 2.5 的自适应标定：

$$\mu^0 = \mathrm{clip}\!\left( \frac{J_s'}{\max(\|C\|^2,\ \varepsilon_\rho)},\ \mu_{min},\ \mu_{max} \right)$$

其中 $J_s'$ 为首轮内层收敛后的基础代价、$C$ 为终点约束残差向量、$\varepsilon_\rho$ 防分母趋零。$\mu$ 的增长采用**充分下降门控**（工程加固，沿用 ALM.md 1.4 节声明的非原论文内容）：仅当 $\|C\|$ 相对上一轮未充分减小（$>\kappa\|C^{prev}\|$，$\kappa<1$）时才 $\mu \leftarrow \min(\varphi\mu, \mu_{max})$，否则只更新乘子 $\lambda \leftarrow \lambda + \mu c$，避免 $\mu$ 过快增长导致内层 Riccati 病态。

**阶段二符号门控（仅精化重解时启用，见 2.6）。** 按修剪后的 maneuver 序列：段内 $-s_m v_k \le 0$（AL 不等式）；相邻 maneuver 接缝点 $v_k = 0$（AL 等式）；接缝前后短窗 $|v_k| \le v_{dwell}$（AL 不等式）。注意这些约束**严禁在阶段一出现**——它们本质上是在局部把 ALM 的结构性换挡边界请回来，提前施加会立即摧毁融化机制。

**接缝窗口宽度不是固定值，而由该接缝的转向需求逐接缝计算。** 接缝两侧 maneuver 的曲率方向若发生翻转（典型如 S 型换挡），前轮转角需在车辆近似静止的窗口内从 $\delta_{left}$ 摆到 $\delta_{right}$，而 $|\omega|\le\omega_{max}$、$|\eta|\le\eta_{max}$ 有界——窗口分配的时间若不够，优化器被迫让转向摆动溢出到 $|v|$ 不可忽略的区域（边挪边打、蹭胎且偏离参考）。从阶段一输出测量第 $j$ 个接缝的转向需求 $\Delta\delta_j = |\delta_{right}-\delta_{left}|$（$\delta_{left}$/$\delta_{right}$ 取接缝前后最后一个 $|v|>v_{dwell}$ 采样点处的 $\delta$），则原地转向所需的最短时间为双积分器 bang-bang 剖面：

$$T_{resteer}(\Delta\delta) = \begin{cases} 2\sqrt{\Delta\delta/\eta_{max}} & \Delta\delta \le \omega_{max}^2/\eta_{max} \quad\text{（三角剖面，$\omega$ 不饱和）}\\ \Delta\delta/\omega_{max} + \omega_{max}/\eta_{max} & \text{否则（梯形剖面）}\end{cases}$$

接缝窗口半宽取 $m_j = \lceil \max(T_{resteer}(\Delta\delta_j),\ T_{shift}) / (2\,dt) \rceil$（$T_{shift}$ 为换挡执行器延迟下限，约 0.3~0.5s），窗口即 $[k_j - m_j,\ k_j + m_j]$。由此每个接缝的静止窗口时长 $\ge T_{resteer}$，优化器在窗内自行排出满足 $\omega,\eta$ 边界的 $\delta(t)$ 摆动剖面——这是后处理驻留插入（2.6 第 4 步）能够只做时间拉伸、不改空间剖面的前提。

**MS 缺陷约束。** 打靶节点处缺陷 $d_{k+1} = f(x_k, u_k) - x_{k+1} = 0$ 不单独建 AL 项，而由 2.5 节的缺陷感知回推 + merit function 内建处理：缺陷在回推公式中作为显式项传播，在线搜索的 merit function 中以 $\mu\|d\|$ 计入接受判据，收敛时自然归零。

**ESDF 定义域之外的语义（L8 修复后补写，此前本节从未定义）。** ESDF 距离场的定义域是有限地图矩形；定义域之外不是「自由空间」也不是「零距离平台」，而是**按实心障碍处理**（人工已确认的建模意图：地图边界之外默认就是障碍物）。具体构造分两层：

- **图内一侧（L8.2）**：EDT 之前把地图最外圈栅格标记为占据，图内距离场在接近边界时自然衰减到 ~0——边界在轨迹**跨出之前**就产生斥力；可用区域每边缩小一个栅格（四数据集路径离边界 ≥7 m，零实际影响；data7/data3 存在贴边障碍，至多被加厚一格，已逐集核实无路径经过）。
- **图外一侧（L8.1）**：对图外查询点 $q$，令 $p=\mathrm{clamp}(q,\text{地图矩形})$、$s=\|q-p\|$，则 $d(q)=d_{map}(p)-s$、$\nabla d(q)=(p-q)/s$（恒指向图内）。于是 $c_{safe}=r_{outer}+margin_{safe}-d$ 随穿透深度**线性增长**、罚 $\max(0,c)^3$ 随深度**三次增长**、恢复力方向恒指向图内。$s\approx0$（恰在边界）时按连续延拓取图内侧场值与梯度。两层叠加后全场在边界上**构造性连续**（不是靠外插缝合）。
- **前向定义域守卫（L8.3，独立防线）**：AL 幅值约束只覆盖 $v/a/\delta/\omega$、Box-QP 只约束控制，位置 $(x,y)$ 不受任何约束；内层 MS-iLQR 的前向 rollout 若产生越出「地图 ⊕ `domain_guard_margin`（默认 2 m）」的候选状态，该试探步**直接判失败回溯**（约束优化的基本要求：试探步不得离开问题定义域），而非拿去评价 merit。L8.1 提供恢复力、L8.3 保证恢复力被压过时也不会发散到 $10^9$ 量级，两者互不替代；margin 外扩若干米是为了不误杀 L8.1 主导的合法小幅越界恢复。

**修复前的缺陷形态（留档）**：图外查询曾一律返回 $d=0,\nabla d=0$——距离场在边界上是 ~19 m 断崖（图内贴边界处最近障碍可能很远），图外是违反度恒定、梯度为零的平台；出界后 AL 违反度原理上降不下来，$\mu$ 指数抬升完全无效（$\mu\times0=0$），只制造 Riccati 病态直至 $\rho_{reg}$ 溢出。data6 单次运行曾产生 7.9 万次越界查询、最深 $1.04\times10^9$ m——修复后同一运行越界查询 1895 次、守卫拒绝 94 次、阶段一形式收敛。告警从逐条打印（单次运行 8 万行，掩盖缺陷三个 Milestone 的直接原因）改为原子计数 + 编排层单次汇总（L8.5）。

### 2.5. 5 求解方法：外层 AL 循环与内层 MS-iLQR

**分层结构总览。** 外层为 AL 循环：固定 $\lambda, \mu$ 调内层 MS-iLQR 求解增广问题，收敛后更新乘子与罚权重（$\mu$ 门控增长），终止判据为终点双指标（$0.05\text{m}/1.5^\circ$）达标**且** $\|d\| \le tol$（打靶缺陷归零），或达到外层迭代上限。内层为 MS-iLQR（Gauss-Newton，丢弃动力学二阶项），一次迭代四步：

**第 1 步：沿线性化。** 在当前名义轨迹 $(\bar X, \bar U)$ 上解析求取 $A_k = \partial f/\partial x$（$7\times7$）、$B_k = \partial f/\partial u$（$7\times2$）。按 2.2 节半隐式 Euler 的链式更新顺序（先 $a^+,\omega^+$，再 $v^+,\delta^+$，再 $\Delta\theta,\theta^+,\theta_{mid}$，最后 $x^+,y^+$）逐层代入求导。记 $v^+ = v + (a + j\,dt)dt$、$\delta^+ = \delta + (\omega + \eta\,dt)dt$（均取线性化点处的值），并引入两个辅助符号——中点角对链上变量的标量偏导：

$$g_1 \equiv \frac{\partial\theta_{mid}}{\partial v} = \frac{\tan\delta^+}{2L_{base}}\,dt, \qquad g_2 \equiv \frac{\partial\theta_{mid}}{\partial\delta} = \frac{v^+\sec^2\delta^+}{2L_{base}}\,dt$$

（链阶规律与 $\theta^+$ 行相同：$\partial\theta_{mid}/\partial a = g_1\,dt$、$\partial\theta_{mid}/\partial j = g_1\,dt^2$、$\partial\theta_{mid}/\partial\omega = g_2\,dt$、$\partial\theta_{mid}/\partial\eta = g_2\,dt^2$。）再记 $c_m = \cos\theta_{mid}$、$s_m = \sin\theta_{mid}$，全部非零元为：

$$A_k = \begin{bmatrix}
1 & 0 & -v^+ s_m\, dt & c_m dt - v^+ s_m g_1 dt & c_m dt^2 - v^+ s_m g_1 dt^2 & -v^+ s_m g_2\, dt & -v^+ s_m g_2\, dt^2 \\
0 & 1 & v^+ c_m\, dt & s_m dt + v^+ c_m g_1 dt & s_m dt^2 + v^+ c_m g_1 dt^2 & v^+ c_m g_2\, dt & v^+ c_m g_2\, dt^2 \\
0 & 0 & 1 & \frac{\tan\delta^+}{L_{base}} dt & \frac{\tan\delta^+}{L_{base}} dt^2 & \frac{v^+\sec^2\delta^+}{L_{base}} dt & \frac{v^+\sec^2\delta^+}{L_{base}} dt^2 \\
0 & 0 & 0 & 1 & dt & 0 & 0 \\
0 & 0 & 0 & 0 & 1 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 1 & dt \\
0 & 0 & 0 & 0 & 0 & 0 & 1
\end{bmatrix}$$

$$B_k = \begin{bmatrix}
c_m dt^3 - v^+ s_m g_1 dt^3 & -v^+ s_m g_2\, dt^3 \\
s_m dt^3 + v^+ c_m g_1 dt^3 & v^+ c_m g_2\, dt^3 \\
\frac{\tan\delta^+}{L_{base}} dt^3 & \frac{v^+\sec^2\delta^+}{L_{base}} dt^3 \\
dt^2 & 0 \\
dt & 0 \\
0 & dt^2 \\
0 & dt
\end{bmatrix}$$

注意求导顺序必须与动力学更新顺序严格一致：$\theta^+$ 行中 $\partial\theta^+/\partial a = \frac{\tan\delta^+}{L}dt^2$ 来自 $a \to a^+ \to v^+ \to \theta^+$ 的二级链，$\partial\theta^+/\partial \eta$ 来自 $\eta \to \omega^+ \to \delta^+ \to \theta^+$ 的三级链；$x^+,y^+$ 行在采用中点角后额外经 $\theta_{mid}$ 获得对 $\delta,\omega$（状态）与 $\eta$（控制）的依赖——例如 $\partial x^+/\partial\delta = -v^+s_m\,dt\cdot g_2$，这是中点格式相对旧 $\theta$ 格式新增的非零元，漏写它们与写错链阶 $dt$ 幂次同为最常见的实现 bug，必须由有限差分单测把关（对 $A_k,B_k$ 全元素逐点对照，而非只抽查对角块）。代价导数取 Gauss-Newton 形（$\ell_x, \ell_u, \ell_{xx}, \ell_{uu}, \ell_{ux}$，全部代价项二次或二次化）；AL 增广项的梯度/Hessian 一并并入 $\tilde\ell$ 的导数（等式项 $\lambda c + \frac{\mu}{2}c^2$ 与不等式门控项分别展开）。

**可选完整二阶动力学项（true MS-DDP，编译期开关，默认关闭）。** 1.4 节末的实现核查表明：三篇论文默认 iLQR 的共同理由是"高维刚体 Hessian 太贵"，而本问题 $n=7$、动力学解析，该理由不成立。若启用完整二阶项，$Q_{xx}, Q_{uu}, Q_{ux}$ 按 1.4 节式补回 $s'\cdot f_{xx}$ 类张量收缩，每步代价约 $n^2(n+m)\approx 441$ 次乘加（与 $A^TS'A$ 同量级），换取 MS-DDP 论文 Fig 3 实测的**局部二次收敛**（默认 iLQR 为线性收敛）。张量结构高度稀疏：$v^+,a^+,\delta^+,\omega^+$ 四行对 $(x,u)$ 皆为线性（张量切片全零），非零元只出现在 $x^+,y^+,\theta^+$ 三行。基本二阶导元为

$$\frac{\partial^2 x^+}{\partial\theta^2} = -v^+ c_m\,dt, \quad \frac{\partial^2 x^+}{\partial v\,\partial\theta} = -s_m\,dt, \quad \frac{\partial^2 y^+}{\partial\theta^2} = -v^+ s_m\,dt, \quad \frac{\partial^2 y^+}{\partial v\,\partial\theta} = c_m\,dt$$

$$\frac{\partial^2 \theta^+}{\partial\delta^2} = \frac{2v^+\sec^2\delta^+\tan\delta^+}{L_{base}}\,dt, \qquad \frac{\partial^2 \theta^+}{\partial v\,\partial\delta} = \frac{\sec^2\delta^+}{L_{base}}\,dt$$

注意采用中点角后，$x^+,y^+$ 行的张量切片同步增密：对 $(v,a,\delta,\omega,j,\eta)$ 的混合二阶偏导（如 $\partial^2 x^+/\partial\delta^2$、$\partial^2 x^+/(\partial v\partial\delta)$）均非零，由基本元经 $g_1,g_2$ 链阶推导（$a\leftrightarrow g_1 dt$、$j\leftrightarrow g_1 dt^2$、$\omega\leftrightarrow g_2 dt$、$\eta\leftrightarrow g_2 dt^2$ 的代换规律，另需对 $g_1,g_2$ 自身再求导，产生 $\sec^2\delta^+\tan\delta^+$ 项），实现时**必须**让有限差分单测覆盖全部三个张量的全部切片，而不仅是 $A_k,B_k$。两个工程副作用需提前知晓：(i) 引入 $s'\cdot f_{uu}$ 后 $Q_{uu}$ 更频繁地失去正定性，$\rho_{reg}$ 触发率上升、后向传递重启次数增加（legged 机器人文献已报告同类现象），实测若重启占比过高应回退 iLQR；(ii) 二阶项与 box-QP 的 $Q_{uu,f}$ 分解、缺陷感知回推公式完全兼容，无需改动其他环节。

**第 2 步：后向传递（缺陷感知 Riccati + 每步 box-QP）。** 沿用 Unified MS-DDP 的缺陷感知回推，记 $S', s', s_0'$ 为下一步已回传的价值 Hessian/梯度/标量（与 ALTRO 记号的 $V_{xx}, V_x, V$ 等价），$\bar d' = f(\bar x_k, \bar u_k) - \bar x_{k+1}$ 为本步缺陷：

$$Q_x = \tilde\ell_x + A^T(s' + S'\bar d'), \qquad Q_u = \tilde\ell_u + B^T(s' + S'\bar d')$$

$$Q_{xx} = \tilde\ell_{xx} + A^T S' A, \qquad Q_{uu} = \tilde\ell_{uu} + B^T S' B, \qquad Q_{ux} = \tilde\ell_{ux} + B^T S' A$$

（对照原论文的红/蓝项分类：红项为 $s'\cdot f_{xx}$ 类动力学二阶张量项，iLQR 将其丢弃；蓝项 $S'\bar d'$ 即缺陷修正，是 MS 相对 SS 回推的全部差异。）随后对控制修正解盒约束 QP：

$$\min_{\delta u}\ \frac{1}{2}\delta u^T (Q_{uu} + \rho_{reg} I)\,\delta u + Q_u^T \delta u \qquad \text{s.t.}\quad \underline{u} - \bar u \le \delta u \le \overline{u} - \bar u$$

采用投影牛顿法（active-set 子类）：钳制集 $c(x) = \{j: x_j=\underline{b}_j \wedge g_j>0,\ \text{或}\ x_j=\overline{b}_j \wedge g_j<0\}$，自由子空间牛顿步 + 逐元素投影候选点 $\hat x(\alpha) = \llbracket x + \alpha\Delta x\rrbracket_b$，Armijo 判据 $(f(x)-f(\hat x))/(g^T(x-\hat x)) > \gamma$（$\gamma=0.1$）；相邻步 QP 高度相似，活动集热启动后平均每次迭代分解数 $<2$（Tassa 实验结论）。QP 须返回自由维 Hessian 分解 $Q_{uu,f}$，据此算 $K_f = -Q_{uu,f}^{-1} Q_{ux,f\cdot}$、前馈 $\delta\tilde u$，**被钳制控制对应的 $K$ 行恒为零**。随后回传：

$$S = Q_{xx} + K^T Q_{uu} K + K^T Q_{ux} + Q_{ux}^T K, \qquad s = Q_x + K^T Q_{uu}\,\delta\tilde u + K^T Q_u + Q_{ux}^T\,\delta\tilde u$$

$$s_0 = s_0' + \delta\tilde u^T Q_u + \tfrac{1}{2}\delta\tilde u^T Q_{uu}\,\delta\tilde u + s'^T \bar d' + \tfrac{1}{2}\bar d'^T S'\bar d', \qquad \Delta V \mathrel{+}= \delta\tilde u^T Q_u + \tfrac{1}{2}\delta\tilde u^T Q_{uu}\,\delta\tilde u$$

（$S, s, s_0$ 一律用 $K, \delta\tilde u$ 形式而非 $Q_{uu}^{-1}$ 形式书写：box-QP 存在钳制行时 $Q_{uu}^{-1}$ 形式与真实 QP 解不再等价，而 $\delta\tilde u$ 形式在任意活动集下均良定义；无钳制时两者退化一致。）**正则化**：$\rho_{reg}$ 按 Levenberg-Marquardt 启发式调度——前向传递失败则 $\rho_{reg}\times$ 增大并重跑本轮后向传递，成功则 $\times$ 减小；$\rho_{reg}$ 超上限 $\rho_{reg,max}$ 判定本轮内层失败，向外层/回退逻辑上报。

**正则化与 Box-QP 交互的实现红线。** QP 内部的 Cholesky 分解必须作用在**已含正则化**的 $(Q_{uu}+\rho_{reg}I)_{ff}$ 上，QP 返回给增益计算的 $Q_{uu,f}$ 分解也必须包含 $\rho_{reg}$（如此算出的 $K_f,\delta\tilde u$ 才与 ALTRO 式 (16) 的正则化修正一致）。由此推出两条硬规则：(i) $\rho_{reg}$ 一旦增大，**必须用新 Hessian 重建并重解全部 QP**——不得只复用上轮活动集而保留旧分解，否则下降方向与正则化假设不自洽（活动集本身可作为热启动输入，但分解必须重做）；(ii) 线搜索拒绝而 $\rho_{reg}$ 不变的重试中，QP 结果与分解可直接复用，无需重解。

**第 3 步：前向传递（非线性滚动 + 精确 EC 线搜索）。** 默认非线性滚动（串行、精确、局部二次收敛）；可选混合滚动（打靶状态先线性推进、段内非线性滚动，线搜索可段间并行）留作并行化接口。控制更新恒为

$$u_k' = \bar u_k + \alpha\,\delta\tilde u_k + K_k (x_k' - \bar x_k)$$

打靶节点处的状态更新带缺陷缩放项（见 1.4 节）：$x_{k+1}' = \bar x_{k+1} + [f(x_k',u_k') - f(\bar x_k,\bar u_k)] + \alpha\bar d_{k+1}$，$\alpha<1$ 时缺陷随步长线性收缩。$\alpha$ 自 1 起回溯。

**双重 Rollout 开销的实现陷阱（必须在类设计时规避）。** 精确 $EC(\alpha)$ 的系数 $EC_1, EC_2$ 依赖**线性化模型推进**得到的搜索方向 $\delta x^l, \delta u^l$（原论文式 (19) 的输入），而实际状态更新用的是**非线性滚动**——两者是不同的轨迹传播。因此每轮内层迭代包含两次滚动：(i) **线性 Rollout 每轮只做一次**（沿 $A_k,B_k$ 传播 $\delta x^l$，含打靶节点的缺陷缩放），目的是获得 $\delta x^l,\delta u^l$ 后把 $EC_1,EC_2$ 缓存为标量系数，此后 $EC(\alpha)=\alpha EC_1+\frac12\alpha^2EC_2$ 在整条 Armijo 回溯链上都是闭式求值；(ii) **非线性 Rollout 只在线搜索循环内**按候选 $\alpha$ 逐次执行。严禁把线性 Rollout 放进 `while` 回溯循环里重复计算，也严禁用非线性滚动的产物反推 $EC$——那会让预期模型与接受判据失配。C++ 侧应将 `linearRollout()` 与 `nonlinearRollout(alpha)` 设计为两个独立接口，$EC_1,EC_2$ 作为本轮迭代的成员缓存。混合滚动选项天然覆盖此结构（其打靶节点线性推进部分即为接口 (i)）。接受判据用 $L_p$ merit function $M = J + \mu_m\|d\|$（缺陷纳入一阶代价；$\mu_m$ 与 AL 罚权重 $\mu$ 是两个独立参数），自适应罚

$$\mu_m = \frac{EC(\alpha)}{(1-\rho)\,\|d\|_p} + \mu_0,\qquad \text{仅当 } \|d\|_p > \kappa_d \text{ 时更新} \quad (\rho=0.5,\ \mu_0=10)$$

Armijo 条件

$$M' < \bar M + \gamma\big( EC(\alpha) - \alpha\mu_m\|d\| \big), \qquad EC(\alpha) = \alpha\, EC_1 + \frac{1}{2}\alpha^2 EC_2$$

其中 $EC(\alpha)$ 为含缺陷效应的精确预期变化模型（$EC_1, EC_2$ 由 $\Delta V$ 各级系数与缺陷耦合项汇总，见 Unified MS-DDP 原文），而非朴素的 $\alpha\Delta V$ 外推——这是打靶法线搜索与单打靶的本质区别，直接用后者会在大缺陷初期频繁误拒。

**第 4 步：收敛判据。** $|\Delta J|/|J| < 10^{-6}$，或梯度范数低于阈值，或达内层迭代上限（建议 50 次，超限按未收敛上报外层）。

**复杂度评估。** 每轮 BP+FP 为 $O\big(N(n^3 + n^2m + nm^2 + m^3)\big)$：主导项为每步的 $A^T S' A$（约 $2n^3\approx 686$ 次标量乘加），$B^T S' A$、$B^T S' B$ 各约 $O(n^2 m + n m^2)$，box-QP 的 $Q_{uu,f}$ 分解为 $O(m^3)$ 且活动集不变时复用；代入 $n=7, m=2, N=399$，每轮全程约 $6\times10^5$ 次标量乘加、折算标量运算 $10^6$ 量级，纯求解部分为微秒~毫秒级。**运行时被 ESDF 查询主导**：$N\times N_c/\text{stride}$ 次取值+梯度查询（如 12 圆、stride=1 时约 $4.8\times10^3$ 次/轮），ESDF 插值的缓存友好性是第一优化对象。与 NMPC/直接法相比，DDP 把单个 $O((Nm)^3)$ 大问题拆成 $N$ 个 $O(m^3)$ 小问题，常数优势在 $N=399$ 的长视窗下是数量级的，这正是选择打靶法而非直接配置法的核心理由。

**C++ 实现规范。** 矩阵一律使用栈上定长类型 `Eigen::Matrix<double,7,7>`、`Eigen::Matrix<double,7,2>`，**严禁热循环内堆分配**；轨迹采用 SoA（Structure-of-Arrays）存储以保证回推扫描缓存友好；雅可比全部手写解析式（禁 autodiff 运行时开销）；角度 wrap 在跟踪代价与终端约束中必须一致处理（同一 `wrap()` 实现）；单元测试三件套：梯度/雅可比有限差分对照单测、box-QP 单测（对照 Tassa 附录算例）、单 maneuver 融化场景回归测试。建议文件结构：`src/core/DDP/{apa_ddp_solver.h, bicycle_dynamics.h, box_qp.h, al_outer_loop.h, esdf_constraint.h}`。

**内存对齐与矢量化（SoA 容器的配套要求）。** 每步的 $A_k,B_k,S,K,\delta\tilde u$ 等按 SoA 存为 `std::vector<Eigen::Matrix<double,7,7>>` 时，**必须附加 Eigen 对齐分配器**，否则容器元素的首地址不保证 16/32 字节对齐，Eigen 的 packet 路径会退化为标量访存（在部分平台/编译配置下甚至触发对齐异常）：

```cpp
using Mat7  = Eigen::Matrix<double, 7, 7>;
using Mat72 = Eigen::Matrix<double, 7, 2>;
template <typename T>
using AlignedVec = std::vector<T, Eigen::aligned_allocator<T>>;

struct Workspace {
    AlignedVec<Mat7>  A_buf, S_buf;      // N / N+1
    AlignedVec<Mat72> B_buf, K_buf;
    // 复用缓冲，禁止在 BP/FP 循环内 resize/push_back
};
```

三点版本与平台注记：(i) **C++17 起** `operator new` 对 over-aligned 类型自动保证对齐，`aligned_allocator` 在语法上成为保险而非必需，但跨编译单元传递容器时仍建议显式保留（ABI 边界上是零成本的防御）；(ii) 自定义结构体若**按值**持有定长 Eigen 成员且会被 `new`/`std::vector` 承载，C++17 之前需加 `EIGEN_MAKE_ALIGNED_OPERATOR_NEW` 宏；(iii) 热循环内的临时矩阵（$A^TS'A$ 的中间产物等）是栈对象，编译器自动对齐，无需任何额外处理——所以该规范只约束**容器层**。编译配置：`-O3 -march=native`（或交叉编译时显式 `-mavx2 -mfma`）并确认未定义 `EIGEN_DONT_VECTORIZE`；对 $7\times7$ 双精度乘法，AVX2（4 宽 packet）下 $A^TS'A$、$B^TS'A$ 这类主导项的实测常数开销可再降近一倍——以 bench 单测（固定 $N=399$ 的一轮 BP 计时）作为回归基线，防止后续改动悄悄丢失矢量化。ARM 平台（部分 APA 域控为 ARM SoC）对应 NEON 2 宽 packet，对齐要求同样适用，加速比略低但结论不变。

**APA-DDP 主算法伪代码：**

```text
APA-DDP(X_ref, x0, target):
  # ---- 阶段一：全局软化 ----
  (X̄,Ū) ← 初值提取(X_ref)              # 2.1：v/κ/δ/a/ω 差分 + 盒约束裁剪
  I ← 打靶节点(每 n_s 步 ∪ cusp ∪ {N}) # 打靶状态以 A* 位姿初始化
  λ ← 0;  μ ← μ_init(自适应 clip)      # 2.4 公式
  w_ref ← w_ref,0
  for r = 0 .. R_max-1:                # 外层 AL 循环
      repeat:                          # 内层 MS-iLQR
          沿线性化: A_k, B_k, ℓ̃ 导数    # 解析, Gauss-Newton
          后向传递: 缺陷感知 Riccati    # 每步 box-QP(活动集热启动)
                    → δũ_k, K_k, ΔV    # ρ_reg LM 调度, 超限则失败上报
          前向传递: α 回溯             # u'=ū+αδũ+K(x'-x̄), 非线性滚动
                    接受 iff Armijo(M, EC(α)) 成立
      until |ΔJ|/|J|<1e-6 or 内层上限
      if 终点双指标达标 and ‖d‖≤tol: break
      λ ← λ + μc (等式) / max(0, λ+μc) (不等式)
      if ‖C‖ 未充分下降: μ ← min(φμ, μ_max)   # 充分下降门控
      w_ref ← w_ref · γ_anneal ( maneuver 内部点 )
  # ---- 后处理与阶段二见 2.6 ----
  return (X̄,Ū) 或回退原始 A* 路径
```

**参数表（建议初值与标定方法）：**

| 参数 | 建议值 | 说明/标定方法 |
|---|---|---|
| $w_j$ | 1.0（基准） | 平滑主项基准权重，其余权重相对它标定 |
| $w_\eta$ | 1.0~5.0 | 按 $j$ 与 $\eta$ 量纲比 $(j_{max}/\eta_{max})^2$ 归一后微调 |
| $w_{ref,0}$ | 10~50 | 首轮使跟踪项与 ESDF 罚同量级；由平衡式反推 |
| $\gamma_{anneal}$ | 0.5/轮 | 衰减过快易换同伦类，过慢收敛轮数增加 |
| $w_\theta$ | 5~20 | 与位置项按 $(0.05\text{m}/1.5^\circ)^2$ 精度比配平 |
| $w_g$（可选） | 1.0 | 默认关闭；启用时与 $\ell_{smooth}$ 同量级起步 |
| $\beta$（可选） | 0.3→0.05 m/s | 随外层轮次线性/几何退火 |
| $v_{max}$ | 1.5 m/s | 按 APA 车速规范，平方幅值形式 |
| $a_{max}$ | 1.0 m/s² | 抗点头舒适度限值 |
| $j_{max}$ | 1.5 m/s³ | 与 ALM min-jerk 目标同源标定 |
| $\delta_{max}$ | ≈0.55 rad | 由最小转弯半径 $L_{base}/\tan\delta_{max}$ 反推 |
| $\omega_{max}$ | 0.5 rad/s | 转向执行器速率限 |
| $\eta_{max}$ | 1.0 rad/s² | 转向执行器加加速度限，防异响 |
| $margin_{safe}/margin_{comf}$ | 0.02 / 0.10 m | 复用 ALM.md 2.4，含义不变 |
| stride | 1~2 | ESDF 时间轴抽样；2 时以 margin 吸收间隙风险 |
| $n_s$ | 25 步 | 打靶段长 2.5s；段多则初值鲁棒、回推常数增大 |
| $\varepsilon_v$ | 0.02 m/s | 符号游程滞回阈值 |
| $\varepsilon_\ell$（可选） | $10^{-6}$ m²/s² | 长度守卫平滑化 |
| $\rho_{len}$ | 0.02~0.05 | 长度比上限 $L/L_0\le1+\rho_{len}$ |
| $v_{dwell}$ | 0.05 m/s | 接缝短窗速度帽 |
| $T_{shift}$ | 0.3~0.5 s | 换挡执行器延迟，按实测标定；驻留与窗口宽度的下限 |
| $\kappa_{pad}$ | 1.2 | 驻留时长安全余量系数，$T_{dwell,j}=\kappa_{pad}\max(T_{resteer},T_{shift})$ |
| $v_{roll}$（可选） | 0.1 m/s | 滚动转向放宽阈值，默认关闭（静止转向） |
| $\mu_{min}/\mu_{max}/\varphi$ | $10^2$ / $10^6$ / 10 | 初值用 clip 公式自适应 |
| tol（终点/缺陷） | 0.05m, 1.5° / $10^{-3}$ | 外层终止双指标 + $\|d\|$ |
| 内层/外层上限 | 50 / 20 | 实时系统兜底退出 |

**M011 机制参数补遗（2026-07-30 效果攻坚落地，全部默认关闭或行为中性；
逐档实测结论见 3.7 节）**：换挡代理调度 `shift_beta_initial/final/gamma`
（0.3/0.05/0.8）、候选段掩码 `melt_crit_threshold`（5000）与
`candidate_anneal_gamma`（默认=全局 γ，机制等价关闭）、种子 μ 上限
`seed_mu_cap_ratio`（0=关闭）、幅值组 μ 上限 `amplitude_mu_max`
（默认=μ_max）、merit 上限 `merit_mu_max`（默认 1e9 不封顶）、δ 奇异区
护栏 `weight_delta_guard`/`delta_guard`（0/0.7）、退火保持轮数
`anneal_hold_rounds`（0）、阶段二跟踪权重地板
`stage_two_min_tracking_weight`（0）、cusp 几何预剪枝 `cusp_prune.*`
（`max_prune_arc` 0=关闭）、margin 延续救援 `rescue_margin_safe`
（0=关闭）、完整二阶编译开关 `APA_DDP_FULL_HESSIAN`（OFF）。

### 2.6. 6 后处理：符号游程分析、拓扑修剪、门控精化与分级输出

阶段一输出的轨迹已动力学一致，但可能残留未被完全融化的微 maneuver，且换挡点处缺少真实执行器所需的时间余量。后处理按以下步骤执行：符号游程分析 → 拓扑修剪 → 候选一（阶段二门控精化 + 驻留插入）；候选一不可用（未收敛/未过合法性门/参考退化）时降级为候选二（阶段一解 + 修剪 + 驻留插入）；两个候选共用同一套合法性门，输出第一个通过门检的候选，全部失败才汇入原始 A* 回退出口。

1. **符号游程分析。** 对 $v_k$ 做带滞回的符号判定：$|v| < \varepsilon_v$（$\approx0.02\text{m/s}$）的样本不计入任何游程，据此把全轨迹切分为正/负速度游程，恢复 maneuver 列表 $\{(s_m, \Delta s_m, \Delta\theta_m)\}$ 与档位序列（Reeds-Shepp 观点的最终兑现：档位由 $\mathrm{sign}(v)$ 恢复，而非优化变量）。滞回是必要的：融化残留的速度涟漪若不过滤，会产生大量亚厘米级虚警游程。

2. **拓扑修剪（$\Delta\theta$ 判据，红线同 ALM.md 2.6.3）。** 用两遍分类算法处理 maneuver 列表：若 $|\Delta s|$ 低于弧长阈值（$0.05\text{m}$，一个重采样间距，与物理方向段数统计的位移过滤口径一致）且 $|\Delta\theta|$ 低于朝向阈值（$0.1\text{rad}$），判定为压平废段，整体剔除并将前后**同向** maneuver 直接拼接；若 $|\Delta s|$ 很小但 $|\Delta\theta|$ 超过阈值（原地掉头式微动），判定为 PIVOT。**判据量取朝向变化 $\Delta\theta$ 而非前轮转角变化 $\Delta\delta$**（口径审计结论，对照 ALM.md 2.6.3 的物理语义）：$\Delta\delta$ 大只说明方向盘在换挡点附近摆动（四数据集实测正常 maneuver 的首末 $\Delta\delta$ 达 0.65~0.79 rad），与原地掉头无关，且 DDP 的 $\delta$ 是显式状态、在换挡点两侧必然大幅摆动；而动力学一致解在微弧长游程内的 $|\Delta\theta|$ 受 $\dot\theta=v\tan\delta/L_{base}$ 上界约束（爬行速度下 $\approx\tan\delta_{max}/L_{base}\cdot\Delta s$，远小于 $0.1\text{rad}$），不可能逼近阈值——故 $\Delta\theta$ 超阈的 PIVOT 判定在物理上只可能由未愈合的打靶缺陷或失实的收敛声明产生，检出即按求解失败处理（默认车辆无钟摆泊车能力，除非特别说明）。**红线**：绝不合并方向相反的相邻段；首/末 maneuver 无论判据量如何均不参与剔除与 PIVOT 重分类，仅允许同向合并；分类只打方向标签（剔除标记 / PIVOT 标记），**绝不改写采样点数据**——压平位置/清零速度会产生 $v\equiv0$ 但 $\theta$ 变化的状态，与 $\dot\theta=v\tan\delta/L_{base}$ 直接矛盾（ALM 侧已废弃的做法）。修剪后在接缝处重采样重排网格，保持 0.05m 间距与 $dt=0.1\text{s}$ 不变。**有效性注记（四数据集实测）**：收敛阶段一解的非保护游程弧长均 $\ge0.91\text{m}$（$\varepsilon_v$ 滞回已在游程分析层吸收融化残余），修剪环节实测零剔除、零 PIVOT——融化的有效性由阶段一连续优化承载，修剪是残余微段的安全网而非主力。

3. **阶段二门控精化 DDP（必须重解，严禁直接拼接输出）。** 以修剪后轨迹热启动，施加 2.4 节的符号门控（段内 $-s_m v_k\le0$、接缝 $v=0$、接缝前后短窗 $|v|\le v_{dwell}$），用同一求解器重跑少量内外层迭代。必须重解而非直接拼接修剪结果的理由有二：修剪删除了网格点，接缝处的 $a, \delta, \omega$ 存在数值毛刺，动力学一致性已被破坏；且拼接后的轨迹未必仍满足全部 AL 约束的渐松紧度。热启动下精化通常数轮外层即收敛，耗时远小于阶段一。

4. **驻留插入（逐接缝计算，非固定值）。** 对每个保留的换挡点 $j$，插入的驻留时长为

$$T_{dwell,j} = \kappa_{pad}\cdot\max\big(T_{resteer}(\Delta\delta_j),\ T_{shift}\big)$$

其中 $T_{resteer}$ 为 2.4 节的双积分器原地转向最短时间（$\Delta\delta_j$ 以阶段二最终轨迹重测），$T_{shift}$ 覆盖换挡执行器延迟（液压/电机换向），$\kappa_{pad}\approx1.2$ 为安全余量。$\Delta\delta_j\approx0$（接缝处无曲率翻转）时退化为纯执行器延迟 padding。**实现上不是简单复制 $v=0$ 帧**：若 $T_{dwell,j}$ 超过阶段二窗口时长 $W_j=2m_j\,dt$，对窗口内容做**线性时间拉伸**——$v\equiv0$ 保持不变，窗内 $\delta(t)$ 摆动剖面按 $T_{dwell,j}/W_j$ 比例放慢重定时，$\omega,\eta$ 只会同比减小、可行性严格保持；$a,j$ 在窗内恒零不受影响。此操作只拉伸时间轴、不改空间路径，总时长增加 $\sum_j T_{dwell,j}$。注意两项前置条件均由阶段二保证：窗内转向摆动已被优化器完整排入（窗口按 $T_{resteer}$ 定宽）、窗口两端 $\omega\approx0$（剖面端点条件），若校验发现摆动溢出窗口或端点 $|\omega|$ 超阈，说明窗口宽度不足，应加大 $m_j$ 重跑阶段二而非强行 padding。**可选变体（默认关闭）**：若底盘规范允许低速滚动转向，窗口约束可放宽为 $|v|\le v_{roll}$（如 $0.1\text{m/s}$），部分摆动在滚动中完成以缩短静止时间——静止转向对轮胎的磨损与回正力矩需求是该开关的主要权衡。

5. **校验口径：合法性门与质量指标两层。** 两个输出候选共用同一套校验；两层均逐项全量量测（不短路返回首个失败项），量测结果进入结构化诊断与日志。
   - **合法性门（Gate，不过即不可输出，不得放宽）**：① 碰撞复检（侵入深度 $\le0.02\text{m}$）；② 终点双指标（$0.05\text{m}/1.5^\circ$）；③ 运动学梯形配点四残差（位置 $0.02\text{m}$/朝向 $3^\circ$/速度 $0.05$/转角 $0.05$）；④ 状态幅值 $v/a/\delta/\omega$ 复检（AL 平衡容差 $0.05$，限值与求解配置同源）。归类理由：这四类即「路径合法」的定义——无碰撞、收敛于终点、符合运动学约束；且 $v/a/\delta/\omega$ 直接进入输出轨迹契约（下游执行消费该剖面），$|\delta|$ 超限即物理不可达的曲率。
   - **质量指标（Metric，记录但不否决）**：⑤ 控制盒过冲 $j/\eta$（参考阈值 $0.3$，求解器发散级探针——$j/\eta$ 不进入输出轨迹契约，前向滚动按设计不截断控制，已收敛解的过冲实测集中出现在终端静止等式收紧的边界层末步，物理可执行性由 ④ 独立保证）；⑥ 接缝与驻留完整性子项（接缝零速 $0.02$/实际驻留时长 $\ge T_{dwell}$/窗内速度帽 $0.07$/窗端 $|\omega|\le0.55$——阶段二收敛解由门控结构保证，阶段一降级候选未施加门控、换挡在蠕动速度内完成属预期形态，度量换挡质量而非路径合法性）；⑦ maneuver 数不增与长度比 $\le1.05$（效果指标，记录供方案比较；maneuver 数曾作为合法性门存在逻辑倒挂——触发时回退目标的 maneuver 数只会更多，该判据的每次生效都必然让最终结果更差）。旧版校验清单中的 $T_{dwell}\ge T_{resteer}$ 子项为死判据（构造上 $T_{dwell}=\kappa_{pad}\max(T_{resteer},T_{shift})$ 且 $\kappa_{pad}\ge1$，恒成立），已移除。

6. **分级输出与失败回退。** 按优先级依次尝试并输出**第一个通过合法性门**的候选：**候选一**（阶段二门控精化 + 驻留插入，完全成功）；**候选二**（阶段一解 + 修剪 + 驻留插入，降级输出——阶段二未收敛/未过门/参考退化时，已收敛的阶段一解本身是合法优化成果，收益虽不及候选一但一定优于整体回退；接缝取修剪后保留游程在阶段一网格上的共享边界，驻留窗口定宽公式与门控计划同源 $m_j=\lceil\max(T_{resteer},T_{shift})/(2dt)\rceil$）；**候选三**（原始混合 A* 路径，最终兜底，语义不变）。降级输出必须在状态码、诊断结构与日志中如实反映实际走的是哪一级，**不得把降级伪装成完全成功**。输出格式为时间序列 $[t, x, y, \theta, v, a, \delta, gear]$（$gear$ 由 $\mathrm{sign}(v)$ 结合驻留段恢复，$v=0$ 窗口内保持前一非零档位或按下游协议填空档）+ maneuver 元数据（每段 $s_m, \Delta s_m, \Delta\theta_m$、起止时间、PIVOT 标志）。两个候选均不过门、阶段一未收敛、或检出 PIVOT 时，**回退原始混合 A* 路径**并记录完整诊断（失败阶段、失败项量化值/阈值、降级原因、两层全量量测），保证模块在任何输入下都有安全输出——这与 NMPC 生产模块的兜底语义一致。

## 3. 实测结果（四数据集端到端验收，2026-07-30）

> 本章为端到端调参与验收的实测记录：最终默认参数、四数据集逐组指标、
> 两处回退的结构化诊断与根因分析、扫描后否决的参数方向（避免重复踩雷）。
> 调参工具：`tool/tune_ddp.cpp`（构建目标 `apa_tune_ddp`），评价口径与
> `Trajectory::validate()` 生产质量门一致。

### 3.1. 最终默认参数（相对 2.5 节参数表的标定增量）

2.5 节参数表的全部取值经四数据集系统扫描后保持默认，仅两项运行期
调度参数按实测标定（已同步写入 `data/ddp_config.json` 与 C++ 结构体
默认值，变更记录见 docs/interfaces.md）：

| 参数 | 参数表初值 | 标定值 | 依据 |
|---|---|---|---|
| `stage_two_max_outer_iterations` | 8 | **16** | 真实长视窗（N≈400~700、接缝 4~8 个）下 8 轮外层预算不足：data3/data7 阶段二分别在 10/11 轮收敛（门控 AL 乘子需要足够的累积轮次）；合成小场景行为不变 |
| `dwell_omega_tol` | 0.1 rad/s | **0.55 rad/s**（= `omega_max` + `amplitude_check_tol`） | 2.6 节校验清单⑥"窗口端点 ω≈0"的设计前提（转向摆动完整排入静止窗）在真实弯曲参考几何上不成立，详见 3.4 节 |

### 3.2. 四数据集验收总表

| 数据集 | 结果 | maneuver 变化 | 长度变化 (m) | 终点误差 | 碰撞深度 | 耗时（阶段一/阶段二+后处理） |
|---|---|---|---|---|---|---|
| `data/mid_park/data3.json` | ✅ 收敛 | 9→7 | 24.58→14.94（−39.2%） | 0.0002 m / 0.001° | 0.0155 m ✅ | 389 ms（≈188/≈236） |
| `data/rub_park/data1.json` | ⚠️ 回退（阶段二未收敛） | — | — | — | — | 167 ms（≈84/≈94） |
| `data/rub_park/data7.json` | ✅ 收敛 | 6→2 | 18.74→14.47（−22.8%） | 0.0025 m / 0.015° | 0.0000 m ✅ | 274 ms（≈95/≈200） |
| `data/long_park/data6.json` | ⚠️ 回退（阶段一内层溢出） | — | — | — | — | 370 ms（≈384/—） |

对照既有基线（ALM 调参后实测）：data3 maneuver 9→7 与 ALM 持平、
长度 14.94 m 显著短于 ALM 21.60 m；data7 maneuver 6→2 优于 ALM
6→4、长度 14.47 m 短于 ALM 16.14 m。两个收敛数据集均满足验收口径：
质量三门全过（碰撞 ≤0.02 m、终点 ≤0.05 m/1.5°、运动学梯形配点残差
达标）、maneuver 数不超原始 A*（9/6）、长度不超原始 +5%。两处回退
均携带结构化诊断（失败阶段 + 失败项 + 量化值/阈值，逐轮外层历史与
接缝级报告随日志转储），回退次数 2/4，超出验收标准④"回退数 ≤1"
的目标，根因与后续计划见 3.3 节。

运动学/控制量实测包络（供下游执行限幅参考）：收敛解 |κ| 始终在车辆
物理上限内，控制量盒过冲（前向滚动反馈项产生，按设计不截断）远低
于 0.3 的专项容差。

### 3.3. 回退根因分析与后续计划

**data1（阶段二未收敛，INNER_SOLVER_FAILED/外层耗尽振荡）**：阶段一
干净收敛（9 轮外层，终点 0.0002 m/0.025°），但终态罚权重一路爬到
μ_max=1e6（终点违反度下降缓慢触发门控增长）。对偶热启动把阶段二的
首轮罚权重与 μ⁰ 标定下限一并抬到种子水平（2.5 节设计：保住已收敛
的终端平衡），种子 μ=1e6 使阶段二内层从第 0 轮即处于强病态：实测
前 5 轮内层 1~2 次迭代即"收敛"（merit 线搜索拒绝一切移动，打靶缺陷
恒为 0.58 降不动），门控 μ 按 ×10/轮追到 1e6 后与终端 μ 同量级对拉，
终点误差反弹至 0.07 m/1.8° 并振荡耗尽预算。已证伪的修复方向：
μ_max 全局截断 1e5（data1 阶段一本身需要 1e6 才能收敛，截断后阶段一
饿死）、门控 μ 截断 1e4/1e5、门控容差放松 0.02、merit μ₀ 降档
（50/20/10 均破坏 data3）、移除种子 μ 抬升（data3 阶段二随之不收敛，
抬升是其必要条件）。根因是**种子罚权重无上限**：抬升机制在种子
μ=μ_max 时把阶段二冻死，而阶段一对该种子的需求是真实的——需要在
抬升路径上加与量级自适应的上限（如 min(seed_μ, κ·μ_calibrated)），
属求解器调度层的后续改进项，超出参数调参空间。

**data6（阶段一内层 ρ_reg 溢出）**：重采样参考在第 447/738 节点处存在
急弯（实测该点阶段一解 δ=−1.21 rad、v=−1.53 m/s，曲率需求 ≈0.88/m，
为车辆物理上限 tan(0.477)/3.0≈0.17/m 的 5 倍——混合 A* 换挡拐角的
V 形折点在 0.05 m 重采样网格上的曲率伪影）。幅值 AL（δ 双侧线性
形态）渐硬压回途中，终点 μ 因航向误差卡在 1.379° 附近不下降而被
门控触发指数增长（10→1362→13621→136211），内层 Riccati 病态化、
线搜索全面拒绝（inner_iter=1 冻结），最终 ρ_reg 溢出。已证实
margin_safe=0.05 可让阶段一收敛（ESDF 罚边界外移给了急弯腾挪空间，
7 轮干净收敛），但该值会让 data3 阶段一首轮内层直接溢出（A* 初值
贴障，罚梯度过刚），且 data6 自身阶段二随后也溢出——单一全局
margin 无法兼顾；已证伪方向：μ 增长 ×2/×3（data6 改善但 data1 早期
即发散）、内层迭代 100、段间惩罚 w_d=10、w_θ=1、打靶加密 n_s=15、
merit μ₀ 降档。根因是**参考构建的曲率伪影与 AL 门控的量纲耦合**：
可行方向是参考构建期对 V 形折点做曲率上限平滑（前端语义层面消除
不可达急弯，而非靠求解器硬扛），属预处理/参考构建层的后续改进项。

### 3.4. 驻留窗端点 ω 校验的标定说明（设计前提在真实几何上的偏差）

2.6 节校验清单⑥的"窗口端点 ω≈0"建立在"优化器会把转向摆动完整排入
静止窗（窗口按 T_resteer 定宽）"的假设上；合成小场景（窗口占轨迹大
半）验证该假设成立，但真实数据集的参考几何在接缝两侧连续弯曲，
阶段二解以**低速滚动出窗**完成残余摆动——实测四数据集大 Δδ 接缝
（0.5~0.9 rad）窗端 |ω| 普遍 0.34~0.50（≈ω_max），且对窗口加宽
（κ_pad 定宽实验，1.2/1.5/1.8 三档）不敏感：平滑代价 ∫η²dt∝Δδ²/T³
决定了优化器总偏好把摆动摊出窗口。物理可执行性由校验④的
|ω|≤ω_max、|η|≤η_max 硬限独立保证（残余摆角 ~ω_e²/(2η_max)≤0.13 rad
在 |v|≤0.05→蠕行的出窗段内完成，与 2.6 节 v_roll 可选变体同一物理
图像）。据此把 `dwell_omega_tol` 标定为
`omega_max + amplitude_check_tol`（0.55）：与校验④共用同一 AL 平衡
包络，窗端探针退化为求解器发散级检测（拦截 ω 严重超限的异常解），
不再承担"摆动必须完整排入静止窗"这一在弯曲参考上不可达的语义。
若未来底盘规范要求严格静止转向，正确做法是实现 2.6 节的 v_roll 变体
（窗口速度帽放宽为 |v|≤v_roll）或在窗端施加 ω=0 的 AL 等式门控，
而非收紧本容差。

### 3.5. 扫描后否决的方向（调参记录，避免重复踩雷）

约 30 组变体（11 批）在四数据集上的实测结论：

- **weight_safe ≥ 200 或 margin_safe ≥ 0.03**：data3 阶段一首轮内层
  直接 ρ_reg 溢出（A* 初值贴障，ESDF 罚梯度过刚，首轮 λ=0/弱终端下
  无解空间）。weight_safe=100/margin_safe=0.02 是阶段一可存活的上限；
  ALM 侧 weight_safe=600 的标定不可移植（求解器结构不同）。
- **κ_pad 参与驻留窗定宽**（窗口半宽 ⌈κ_pad·max(T_resteer,T_shift)/(2dt)⌉，
  1.2/1.5/1.8 三档）：对窗端 ω 无改善（平滑代价决定摆动摊出窗口），
  反而加宽驻留帽覆盖区、降低阶段二收敛鲁棒性（data3 由收敛转为内层
  溢出），并引入运动学残差超标。已干净回退。
- **μ 增长倍率放缓（×2/×3）与门控阈值 κ=0.5**：data6 阶段一改善但
  data1 阶段一早期即发散（终点 0.45 m/129°、幅值违反 28.6）；data1
  需要 μ 快速渐硬，data6 需要慢——单一全局调度无法兼顾。
- **μ_max 全局截断 1e5**：data1 阶段一在违反度 0.0102（距 0.01 阈值
  一步之遥）处饿死。
- **anneal_gamma=0.7**：阶段一末轮跟踪权重升高使阶段二紧贴参考，
  data3 碰撞由 0.016 恶化到 0.18。
- **weight_steer_accel 0.5/0.2**：窗端 ω 无改善（0.34→0.33），阶段一
  在 data3 上失稳。
- **weight_comf 3/10、v_dwell=0.15、inter_segment_weight=10、
  n_s=15、gating_mu_max 截断、merit μ₀ 50/20/10、weight_theta=1**：
  各自在至少一个数据集上引入阶段一/阶段二失稳或碰撞回归，无一净收益。
- **移除对偶热启动的种子 μ 抬升**：data3 阶段二随即不收敛——抬升
  机制在种子量级正常时是终端平衡的必要条件（仅种子=μ_max 时有害）。

### 3.6. 后处理链路与校验口径审计（2026-07-30，M010）

> 本节为后处理链路审计与口径重构的实测记录：四断点诊断证据、五处
> 嫌疑点的逐条裁决、重构后的四数据集验收对照。诊断工具：
> `tool/audit_ddp_post.cpp`（构建目标 `apa_audit_ddp_post`），复刻
> `optimizeDdp` 装配并逐步手动编排，对阶段一解/修剪环节/阶段二解/
> 驻留与最终输出四个断点转储机器可读量测（12 项校验判据不短路）。

**五处嫌疑点的逐条裁决（实测证据 → 判断 → 处置）**：

| # | 嫌疑点 | 关键实测证据 | 判断 | 处置 |
|---|---|---|---|---|
| 1 | PIVOT 判据 Δδ/Δθ 量纲错配 | 正常 maneuver 首末 Δδ 实测 0.65~0.79 rad（换挡点两侧的方向盘摆动），四数据集 PIVOT 零触发；微弧长游程的 |Δθ| 受 θ̇=v·tanδ/L 约束不可能逼近 0.1 rad | **成立**（语义错误，当前未激活但处置致命：误判即整体回退） | 判据改 Δθ（对齐 ALM 语义），DDP 自有判据配置 `DdpPruneConfig`，禁用压平改写 |
| 2 | 剔除弧长阈值与网格量纲不匹配 | 收敛阶段一解非保护游程 Δs 全部 ≥0.91 m，修剪环节四数据集零剔除：融化残余被 ε_v 滞回在游程分析层吸收，不以游程形式存在 | **成立**（判据从不触发），但实测分布不支持任何更大取值 | 阈值保持 0.05 m（与 countDirectionRuns 位移过滤同口径）；结论固化：融化有效性由阶段一承载，修剪为安全网 |
| 3 | 校验清单两条路径不对称、过严 | `dwell_resteer` 全部接缝实测余量 0.09~0.39 s 恒正（死判据）；`maneuver_count` 四数据集均不触发且逻辑倒挂；阶段二收敛解的接缝/驻留子项全过，降级候选不过属预期（阶段一无门控） | **部分成立** | 校验分两层：合法性门（碰撞/终点/运动学/状态幅值，不放宽）+ 质量指标（控制过冲/接缝驻留子项/段数/长度比，记录不否决）；死判据移除 |
| 4 | 二元回退丢弃已收敛阶段一解 | data1 阶段一干净收敛（终点 0.0002 m/0.025°），其降级候选过全部合法性门（碰撞 0、运动学四残差达标、状态幅值 0.003）；唯一「超标」是控制过冲 0.609——实测定位于最后一个控制步（终端静止等式边界层尖峰，j/η 不进入输出契约） | **成立**（最大收益浪费点） | 分级降级出口：阶段二 → 阶段一降级 → 原始 A*，降级如实上报不伪装 |
| 5 | 缺少停驻窗合法化对应物 | 阶段二候选运动学四残差全部达标（航向残差 0.03~0.06°），无「v≈0 但 θ 在变」伪影；但发现驻留插入窗口左边界存在重定时缩放断点（窗外原始 v/a、窗内首点 v/r、a/r²），窗速未受帽约束的降级候选上可产生 0.06~0.07 速度残差（data3/data7 假设性候选实测） | **不成立**（原假设），相关伪影已登记 known-limitations | 不引入 steer padding；重定时边界伪影记录留待后续（生产路径窗速受帽 ≤0.05 不受影响） |

**修剪环节修改前后对照（阶段一解游程数 → 修剪后 maneuver 数）**：

| 数据集 | 修改前（Δδ 判据） | 修改后（Δθ 判据） | 说明 |
|---|---|---|---|
| data3 | 9 → 9（零剔除） | 9 → 9（零剔除） | 最小游程弧长 0.91 m |
| data1 | 4 → 4（零剔除） | 4 → 4（零剔除） | 最小游程弧长 0.93 m |
| data7 | 5 → 5（零剔除） | 5 → 5（零剔除） | 末段 0 m 为保护段 |
| data6 | 阶段一未收敛，无修剪 | 同左 | — |

四数据集修剪环节均一段剔不掉：**是阶段一解本身就没有可剔的段**
（融化在阶段一连续优化内已完成），而非判据问题。

**重构后四数据集端到端验收（默认参数不变，对照 3.2 节基线）**：

| 数据集 | M009 基线 | 重构后 | maneuver 变化 | 长度变化 (m) |
|---|---|---|---|---|
| data3 | ✅ 9→7 / 14.94 | ✅ 阶段二输出（无回归） | 9→7 | 24.58→14.94 |
| data1 | ⚠️ 回退（阶段二未收敛） | ✅ **阶段一降级输出** | 10→4 | 12.99→11.29 |
| data7 | ✅ 6→2 / 14.47 | ✅ 阶段二输出（无回归） | 6→2 | 18.74→14.47 |
| data6 | ⚠️ 回退（阶段一内层溢出） | ⚠️ 回退（不变，根因属 M011 范围） | — | — |

完全回退数由 2 降为 1（仅剩 data6，其阶段一未收敛不存在可降级
的解，根因是参考构建曲率伪影与种子 μ 调度，均归 M011）。data1
降级输出的合法性由同一质量门实测背书：碰撞 0、终点 0.0002 m /
0.025°、运动学四残差达标、状态幅值违反 0.003（门限 0.05）；其
质量指标如实记录（接缝 |v| 0.042、窗内 |v| 0.21、控制过冲 0.609
——均为阶段一无门控与终端边界层的预期形态），日志不伪装为完全
成功。

### 3.7. 换挡数系统性优化（2026-07-30，M011 效果攻坚）

> 本节为「方案层 + 参数层系统优化」的完整实测记录：L0 基线与工具、
> L1~L4 四层机制的逐变体假设/实测/结论、最终默认参数决策。全部
> 20+ 变体的 [TUNE]/[AUDIT] 留档在 `build/log/tune_m011_*.txt` 与
> `build/log/audit_m011_baseline.txt`；逐条实验的三段论详见
> [docs/milestones/milestone-011/review-log.md](milestones/milestone-011/review-log.md)
> Round 0 实验流水账。

**L0 基线与评价口径**：以 M010 收口代码 + `data/ddp_config.json` 默认
值冻结基线；`tool/tune_ddp.cpp` 重写为「变体矩阵 × 四数据集」一次
跑批（[TUNE-RANK] 按 合法→maneuver 数→长度比 排序、[TUNE-GATE] 逐项
门检），`tool/audit_ddp_post.cpp` 变体化并新增断点 0（参考 maneuver
临界比 [AUDIT-REFMAN]、曲率伪影 Top-5 [AUDIT-REFKAPPA]、首轮内层
探针 [AUDIT-R0]）。

**关键定量结论（先于一切调参的「先算后调」产物）**：

- 融化平衡式（2.3 节）在四数据集上定量成立，实际融化阈值约为
  2×crit（crit=T⁵·n_pts·dt）：γ=0.3 探针在 data3 收敛轮处
  w_j/w_ref≈457≈2.3×crit(199) 时 m1/m5/m7 中两段精确融化，γ=0.4
  （381≈1.9×crit）一段不融。基线收敛轮处的权重比（25~51）全部
  远低于残余内部段的临界比——阶段一融化在基线调度下已达上限。
- 残余内部 maneuver 的临界比分布：下一档可融带 crit≈90~800
  （data1 m2=90、data3 m1/m5/m7=199/m6=753、data6 m3=199），真实
  maneuver ≥8.5e3；几何重叠判据进一步证明这些段全部承载真实位移
  （折返终点落在未探索新区域），不是可剪枝的几何冗余。
- M009 登记的「参考曲率伪影 0.88/m」在当前参考构建器输出上不
  存在（θ 流峰值 0.173~0.187/m < 上限 0.204/m）——该数值是
  阶段一解在失稳点的隐含曲率，不是参考的固有伪影；data6 的真正
  特性是「首轮内层 δ 奇异区逃逸（data3 同样逃逸但可愈合）在长
  视窗下不愈合」。

**采纳/否决的方案清单**：

| 机制 | 层级 | 结论 | 一句话依据 |
|---|---|---|---|
| 驻留窗边缘斜坡重定时 | 修复 | ✅ **采纳（默认生效）** | M010 登记伪影修复；基线运动学速度残差全面改善（0.045→0.018 等），长度/段数/合法性逐位保持 |
| 换挡代理 ℓ_shift + β 退火 | L1.1 | ❌ 证伪 | 精确/PSD 两种曲率 × 七档权重全部出局：cusp |v| 不响应、普遍失稳（与 v≈0 奇异区相互作用） |
| 全局权重比调整（γ=0.4 / w_j=8） | L1.2 | ❌ 证伪 | 全局抓手同时松开该融与不该动的段（data1 发散/data3、data6 首轮溢出） |
| 逐 maneuver 差异化退火（候选掩码） | L1.3 | ❌ 证伪 | 局部释放造成锚点冲突——被释放段被拉拽而非压缩；分段常数退火同样证伪（收敛-退火赛跑无解） |
| 完整二阶 DDP（编译开关） | L3.3 | ❌ 证伪 | 全线劣化（Q_uu 频繁失去正定性）；接线保留默认关闭 |
| 种子 μ 量级自适应上限 | L3.1 | ❌ 证伪 | 无有效 κ 分离区间（病态比值 9.5 < 健康 13.3） |
| 幅值组独立 μ 上限 | L3.5 | ⚠️ 零回归但不采纳 | 健康数据集逐位不变；data6 破螺旋但缺陷不愈，无合法输出收益 |
| merit μ_m 自适应 + 上限 | L3.4' | ❌ 证伪 | κ_d=1.0 误伤健康数据集（求解瞬态同样越门限） |
| δ 奇异区光滑铰链护栏 | 探针 | ❌ 证伪 | 健康求解的中间迭代同样越界 0.7 rad |
| 打靶稀疏化 n_s=40/60、重采样加粗 0.10 m | 探针 | ❌ 证伪 | 四数据集全灭——0.05 m/25 步是负载结构 |
| margin 系列（0.05/×weight 50/延续救援） | 探针 | ❌ 证伪 | data6 唯一收敛路径（margin 0.05）的解穿障 0.63 m，不可能合法 |
| cusp 几何预剪枝 | L2.2 | ⚠️ 落地不采纳（默认关闭） | 判据证明残余 maneuver 均非几何冗余（data1 输入 10→7 但输出不变） |
| 深退火+阶段二地板（γ=0.3+floor 0.015） | L1.3' | ❌ 不可采纳 | data3 9→**5** 合法但长度 +7.5% 超 +3% 帽；data1 发散、data7 变劣 |
| L3.2 平方根回推 / L4.1 多候选 / L4.2 阶段二跳过 / L4.3 迭代融化 / L2.3 打靶布设 | L2~L4 | ⚪ 评估后不实施 | 逐条理由见 review-log（目标场景瓶颈已被其他实测排除） |

**最终四数据集验收总表（最终默认参数 = M010 标定值）**：

| 数据集 | 结果 | maneuver 变化 | 长度变化 (m) | 终点误差 | 碰撞深度 | 耗时 |
|---|---|---|---|---|---|---|
| `data/mid_park/data3.json` | ✅ 收敛（阶段二） | 9→7 | 24.58→14.94（−39.2%） | 0.0002 m/0.001° | 0.0155 m ✅ | ≈357 ms |
| `data/rub_park/data1.json` | ✅ 阶段一降级输出 | 10→4 | 12.99→11.29（−13.1%） | 0.0002 m/0.025° | 0 ✅ | ≈151 ms |
| `data/rub_park/data7.json` | ✅ 收敛（阶段二） | 6→2 | 18.74→14.47（−22.8%） | 0.0025 m/0.015° | 0 ✅ | ≈254 ms |
| `data/long_park/data6.json` | ⚠️ 回退（阶段一不收敛） | — | — | — | — | ≈341 ms |

**与验收标准的差距（诚实记录）**：标准 1/2（全部合法/零回退）因
data6 未达成——其阶段一收敛性缺口经 10+ 机制变体实测不可弥合，
判定为本求解器配置在该长视窗数据集（N=738，近 data3 两倍）上的
结构性能力缺口而非标定不足；标准 3 的「≥2 严格优于 ALM」达成 1
个（data7 6→2），data3 的 9→5 前沿点因超 +3% 长度帽不可采纳。
data1/data3/data7 三集与 M010 基线逐位持平且运动学残差全面改善。

### 3.8. 转角/曲率口径修正与曲率正则（2026-07-30，M011 L5 层）

> 本节为 L5 层（转角/曲率口径修正 + 曲率正则）的实测记录，承接
> 3.7 节的结论评审。完整实验三段论见
> [docs/milestones/milestone-011/review-log.md](milestones/milestone-011/review-log.md)
> Round 1 回应。

**L5.0 口径修正（已落地）**：DDP 链路的 δ/ω 幅值上限曾长期以 JSON
硬编码值运行（0.55/0.5），比车辆物理参数（`max_steer_angle`=0.47728、
`max_steer_rate`=0.4）放大 15%/25%，对应曲率上限放大 18.5%
（0.2044 vs 0.1724 /m）——此前全部「合法」输出实际贴着被放大的
假边界运行（κ≈0.2 = 113% 真值上限），且三道校验自检自全部漏检。
修正：`DdpConfig::clampToVehicleParams`（`optimizeDdp` 单一收口点，
JSON 值只准收紧到车辆真值、不准放宽），结构体默认值与
`data/ddp_config.json` 改为真值；调参/审计工具接入 κ 量测
（`max_kappa`/`kappa_p95`/`kappa_ratio`/`max_omega` 列与 [TUNE-PARETO]）。

**输入侧的固有形态（决定一切下游结论的取证）**：四数据集原始 A\*
路径的 κ 分布——data3 P50=0.1723/P95=0.1739、data1 P50=0.1789/
P95=0.1854、data7 P50=0.1847/P95=0.1863、data6 P50=0.1724/P95=0.1726。
**前端路径本身就在大部分弧长上贴着/超过物理上限**（data1/data7 的
前端以超上限 ~8% 的曲率骑行，known-limitations 早有记录）。由此：
(i) 曲率形态门（P95 ≤ 0.8·κ_max）在这些数据集上物理不可达；
(ii) 严格 max κ ≤ κ_max 与 AL 平衡残余（实测输出落在上限之上
0.5~1.8%）不兼容，内缩 δ 边界则环境不可行——本仓既有实践按
ALM 的 ~4% 包络验收，tune 门检折入阈值同此口径、严格值仍全量上报。

**L5.1 曲率正则（已实现、任何档位不可采纳）**：ℓ_κ=w(tanδ/L)²
（天然 PSD、三阶极点内屏障）按 spec 三档 {22,67,225} 实测全部碰撞
出局（对拉参照错配——把 ESDF 罚在 margin 处的梯度压过一个数量级，
优化器宁穿障碍不转弯）；细化档 {1,2,5} 不移动 max κ、P95 仅微降而
长度爆炸（+1.4%~+14%），且对超上限参考明确有害（data7 碰撞随权重
单调恶化）。连续贴顶是参考的固有形态，惩罚 κ 就是惩罚环境可达性
本身——机制保留默认关闭。

**重锚机制（L4.3 结构变体，探针驱动，默认阈值 0.01 m）**：阶段一解
侵入超阈时把解重建为新参考并热启动重解一轮——解除超上限参考的
跟踪拉力后 ESDF 把解推出侵入（data7 阶段一侵入 0.031→0.0098 一轮
落门、data1 0.0116→0；多轮迭代可发散，故限一轮）。

**真值上限下的最终四数据集验收总表**（最终默认参数：深退火 γ=0.3 +
阶段二跟踪权重地板 0.015 + 重锚阈值 0.01 + 车辆真值幅值上限 +
曲率正则关闭）：

| 数据集 | 结果 | maneuver 变化 | 长度变化 (m) | 终点误差 | 碰撞深度 | max κ / P95（/0.1724） | 耗时 |
|---|---|---|---|---|---|---|---|
| `data/mid_park/data3.json` | ✅ 收敛（阶段二） | 9→7 | 24.58→17.56（−28.6%） | 0.0002 m/0.0001° | 0.0035 m ✅ | 1.0002/1.0000 | ≈237 ms |
| `data/rub_park/data1.json` | ✅ 阶段一降级输出 | 10→4 | 12.99→12.32（−5.1%） | 0.0049 m/0.324° | 0 ✅ | 0.9966/0.9915 | ≈145 ms |
| `data/rub_park/data7.json` | ✅ 阶段一降级输出 | 6→4 | 18.74→16.74（−10.7%） | 0.0045 m/0.021° | 0.0190 m ✅ | 0.9930/0.9893 | ≈205 ms |
| `data/long_park/data6.json` | ⚠️ 回退（阶段一无可输出解） | — | — | — | — | — | ≈473 ms |

**与假上限口径的历史数字的关系**：3.2/3.6 节的四数据集表（data3
14.94 m、data7 14.47 m、data1 11.29 m）是在 δ 上限被放大 18.5%
的配置下测得的，依赖 κ≈0.204 的非物理转向能力，**不是车辆可执行
解**；真值口径下同场景的诚实数字为 17.56/16.74/12.32 m——仍然全部
显著短于 ALM 真值基线（21.60/16.14/—）。data6 在真值口径下阶段一
可形式收敛但解不可用（10 游程/58.8 m 的 bang-bang 形态），其阻塞
定量归结为「单一全局退火调度下，data3 融化需要末轮 w_ref≲0.002 而
data6 可行性需要 w_ref≳0.3，150 倍冲突不可兼得」。

### 3.9. L6 层机制评测（2026-07-30，M011 Round 2）

> 本节为 L6 层（data6 跑飞对策 + 阶段二对拉 + 参考治理 + 结构性分解）
> 的实测记录，承接 3.8 节。逐项实现细节/假设/证据链见
> [docs/milestones/milestone-011/review-log.md](milestones/milestone-011/review-log.md)
> Round 2 回应；否决项的勿重试清单见
> [docs/known-limitations.md](known-limitations.md) L6 条目。**本节机制
> 全部默认关闭、代码与单测保留**；最终默认参数保持 3.8 节标定不变。

| 机制 | 形态 | 四数据集实测要点 | 结论 |
|---|---|---|---|
| L6.1 弧长惩罚 ℓ_v=w_v·v² | `weight_velocity`（PSD） | w∈{0.11,0.45,1.8} 全灭：罚 v² 与固定位移需求对拉，代价转嫁安全门（穿墙 0.13~0.73 m）或 δ/ω 幅值门 | 证伪 |
| L6.2 逃逸冻结（环比触发） | `anneal_freeze_*` 三阈值 + 滞回 | data6 收敛成 63 m 级垃圾（触发太迟）；data3 长度 +3.4%、data1 降级解 δ 超门 | 对策证伪，机制保留 |
| L6.2b 逃逸冻结（绝对长度比 1.2） | `anneal_freeze_ref_length_ratio` | 健康三集**逐位零副作用**；data6 仍 ~59 m 跑飞——冻结在高 w_ref 位也拉不回（μ 螺旋先杀内层） | 对策证伪，触发器合格保留 |
| L6.3a 阶段二 ESDF 独立标定 | `esdf_stage_two` 节 + 专用求解器注入 | weight_safe×3 → data3 δ 门失败回退；margin_safe 0.05 → data7 变阶段二不收敛。**collision 门失败是几何不相容，不是标定问题** | 证伪 |
| L6.4 参考曲率投影 | `ProjectReferenceCurvature`（θ 钳制 + 航向守恒摊派） | cap=1.0/0.95 两档：data3 回退或失收敛、data1 κ 超门或引入碰撞、data7 回退——θ 摊派制造 θ-位置矛盾，跟踪项对拉瓦解平衡 | 证伪 |
| L6.5 时域分解 | （探针，未实现） | data6 逐 maneuver 子问题原样复现跑飞（m2 长度 1.81×、v 越上限）；时域裕量比 T/(L/v)=1/v_nominal=2 在分解下**不变**——视窗长度不是跑飞决定因素 | 探针级证伪 |

**L6.4 的实现级教训（对后续参考治理类方案通用）**：均匀超限弧段在
保持两端位姿的前提下不存在 κ≤κ_cap 的连接曲线（min-max κ 下界
=|Δθ|/L=原 κ）；单弧替换的端点过冲接缝折角可达 ~160°。θ 口径钳制
是唯一自恰形态（与参考构建器 δ=atan(L·κ) 反解同口径），但 θ-位置
不一致性本身对跟踪代价有毒。

**data6 根因的最终结算**：Round 2 的「固定时域无项反对跑飞」代价
缺项判断成立，但该缺项**无法通过调度/惩罚/分解在现有代价族内补上**
（L6.1/L6.2/L6.2b/L6.5 四条独立路径均封闭）。剩余候选方向（超出
M011 spec 范围，供后续立项）：显式弧长状态增广、终端时间自由化
（dt 进优化变量）、或以前端可行性治理替代后处理修复。

**附带发现**：关融化对照解（γ≈1）在 data7 上为 6→4/15.54 m/阶段二
全合法，优于生产默认融化解（16.74 m/降级）——data7 的现行融化定价
为负收益（+7.7% 长度换同一 maneuver 数）；同一对照在 data3/data1
碰撞回退，不构成全局翻转依据，已报评审。

### 3.10. L7 层：时域探针、双候选择优与人工裁决项（2026-07-31，M011 Round 3）

> 逐项证据链见
> [docs/milestones/milestone-011/review-log.md](milestones/milestone-011/review-log.md)
> Round 3 回应。**本轮起最终默认参数新增「双候选择优」**
> （`dual_candidate_select=true`，见下）。

**L7.2 双候选择优（采纳，唯一净收益项）**：同一输入跑「融化开」与
「退火率 0.999（融化关）」两遍完整链路，按「成功 → maneuver 数少 →
长度短」择优。实测（生产默认口径）：data7 **16.74→15.54 m（−7.2%）、
阶段一降级升阶段二、碰撞 0.019→0**；data3（9→7/17.56/阶段二）与
data1（10→4/12.32/降级）逐位不变；data6 双候选均败保持回退。耗时
约 +100%（四集 0.47~0.80 s，标准⑦内）。验收标准④的对照解防线由
择优自动满足。评审的事前判断（两配置严格互补、data7 融化净负收益）
全部兑现。

**L7.1 时域 T 探针（作废）**：单改 `reference.dt`∈{0.07,0.05}（裕量
3×→2.1×/1.5×）两档四数据集全灭（碰撞 0.05~0.15 或阶段一失收敛）。
实现层面更正：T=L_ref/v_nominal 与初值 bang 剖面速率
v_nominal=sample_dist/dt 锁定耦合，「干净缩 T」在当前构建器结构里
不存在；L7.5（缩 T 后重测初值）按依赖条款随之取消。

**L7.3 projected-Newton polishing（探针证伪，未实现）**：data1/data7
的阶段二求解本已收敛（gating_ok=1，门控残余 ≤2.5e-4 远在容差内）——
**无可清残余**；唯一失败的 collision 门是 ESDF 惩罚的代价平衡（不在
AL 约束集内），polishing 不触碰代价平衡。「失败模式搬家」的机理由此
更正：不是 AL 残余预算共享，而是 ESDF 代价与门控几何的不相容。

**L7.4 异构回退 ALM（人工裁决不批准）**：保持纯 DDP 语义，失败回退
原始 A*。**L7.6 验收标准③口径复议（人工悬置）**：维持「≥2 严格优于
ALM」原口径。

**Round 3 末态验收结算**：①❌（data6）、②❌（data6 回退；data7
本轮升阶段二）、③❌（7/4/4/6 vs ALM 7/4/4/4，严格优于 0 个）、
**④✅（本轮转绿）**、⑤✅。剩余缺口（data6 合法输出、③的严格优于
条款）均超出算法机制范畴，待人类决策方向。
