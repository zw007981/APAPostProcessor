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
    - [2.6. 6 后处理：符号游程分析、拓扑修剪与门控精化](#26-6-后处理符号游程分析拓扑修剪与门控精化)

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

本章将求解内核从 ALM 方案的「多项式参数化 + L-BFGS」彻底替换为「多重打靶 DDP + 增广拉格朗日外层」，底盘模型同样采用阿克曼自行车模型，输入同样是带档位标志的混合 A* 轨迹。方法的整体路线为三阶段：**阶段一**（全局软化 DDP）以软跟踪初值 + 跃度主导代价 + ESDF/物理约束 + 终点对齐求解一整条 39.9s 的连续轨迹，允许无效 maneuver（如无意义的「停-倒-停」微动）在连续优化内部被「融化」——速度全程不变号地穿过原换挡点；**后处理**检测并修剪残余微 maneuver；**阶段二**（门控精化 DDP）按修剪后的 maneuver 序列施加符号门控并热启动重解，输出最终轨迹。任一硬校验不过则回退原始混合 A* 路径。本章理论基础来自三篇论文：

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

它是**纯状态代价、处处可微**，每次 $v$ 由正转负或由负转正穿过零点附近时产生量级 $w_g$ 的惩罚。注意此处 $v_k^+ = v_k + a_k\,dt$ 是**显式一步预测**（不含控制 $j$ 的 $dt^2$ 修正），并非 2.2/2.5 节动力学链中的 $v^+$——该写法刻意让 $\ell_{shift}$ 不依赖控制量，求导时按纯状态函数处理即可，不得代入动力学链的 $v^+$ 表达式。该项非凸，故 $\beta$ 需连续退火：$0.3 \to 0.05\ \text{m/s}$，由宽门逐渐收窄到速度滞回阈值附近。默认关闭，仅作为退火兜底手段启用，启用时应在参数表中显式记录。

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

### 2.6. 6 后处理：符号游程分析、拓扑修剪与门控精化

阶段一输出的轨迹已动力学一致，但可能残留未被完全融化的微 maneuver，且换挡点处缺少真实执行器所需的时间余量。后处理按以下六步执行，全部失败路径最终汇入第 6 步的回退出口。

1. **符号游程分析。** 对 $v_k$ 做带滞回的符号判定：$|v| < \varepsilon_v$（$\approx0.02\text{m/s}$）的样本不计入任何游程，据此把全轨迹切分为正/负速度游程，恢复 maneuver 列表 $\{(s_m, \Delta s_m, \Delta\theta_m)\}$ 与档位序列（Reeds-Shepp 观点的最终兑现：档位由 $\mathrm{sign}(v)$ 恢复，而非优化变量）。滞回是必要的：融化残留的速度涟漪若不过滤，会产生大量亚厘米级虚警游程。

2. **拓扑修剪（复用 TopologyCleaner，红线同 ALM.md 2.6.3）。** 用已验证的两遍分类算法处理 maneuver 列表：若 $|\Delta s| < 0.05\text{m}$ 且 $|\Delta\theta|$ 低于阈值，判定为压平废段，整体剔除并将前后**同向** maneuver 直接拼接；若 $|\Delta s|$ 很小但 $|\Delta\theta|$ 超过阈值（原地掉头式微动），判定为 PIVOT 单独保留、不参与合并，考虑到默认情况车辆没有装三电机四电机无法执行钟摆泊车等动作，因此除非特别说明一旦出现PIVOT可视为求解失败。**红线**：绝不合并方向相反的相邻段；首/末 maneuver 无论判据量如何均不参与剔除与 PIVOT 重分类（首段承载车辆当前位姿、末段承载终点语义），仅允许同向合并。修剪后在接缝处重采样重排网格，保持 0.05m 间距与 $dt=0.1\text{s}$ 不变。复用 NMPC 侧 `TopologyCleaner` 的判据结构，仅按本模块输出重新标定量纲/阈值，不另起炉灶。

3. **阶段二门控精化 DDP（必须重解，严禁直接拼接输出）。** 以修剪后轨迹热启动，施加 2.4 节的符号门控（段内 $-s_m v_k\le0$、接缝 $v=0$、接缝前后短窗 $|v|\le v_{dwell}$），用同一求解器重跑少量内外层迭代。必须重解而非直接拼接修剪结果的理由有二：修剪删除了网格点，接缝处的 $a, \delta, \omega$ 存在数值毛刺，动力学一致性已被破坏；且拼接后的轨迹未必仍满足全部 AL 约束的渐松紧度。热启动下精化通常数轮外层即收敛，耗时远小于阶段一。

4. **驻留插入（逐接缝计算，非固定值）。** 对每个保留的换挡点 $j$，插入的驻留时长为

$$T_{dwell,j} = \kappa_{pad}\cdot\max\big(T_{resteer}(\Delta\delta_j),\ T_{shift}\big)$$

其中 $T_{resteer}$ 为 2.4 节的双积分器原地转向最短时间（$\Delta\delta_j$ 以阶段二最终轨迹重测），$T_{shift}$ 覆盖换挡执行器延迟（液压/电机换向），$\kappa_{pad}\approx1.2$ 为安全余量。$\Delta\delta_j\approx0$（接缝处无曲率翻转）时退化为纯执行器延迟 padding。**实现上不是简单复制 $v=0$ 帧**：若 $T_{dwell,j}$ 超过阶段二窗口时长 $W_j=2m_j\,dt$，对窗口内容做**线性时间拉伸**——$v\equiv0$ 保持不变，窗内 $\delta(t)$ 摆动剖面按 $T_{dwell,j}/W_j$ 比例放慢重定时，$\omega,\eta$ 只会同比减小、可行性严格保持；$a,j$ 在窗内恒零不受影响。此操作只拉伸时间轴、不改空间路径，总时长增加 $\sum_j T_{dwell,j}$。注意两项前置条件均由阶段二保证：窗内转向摆动已被优化器完整排入（窗口按 $T_{resteer}$ 定宽）、窗口两端 $\omega\approx0$（剖面端点条件），若校验发现摆动溢出窗口或端点 $|\omega|$ 超阈，说明窗口宽度不足，应加大 $m_j$ 重跑阶段二而非强行 padding。**可选变体（默认关闭）**：若底盘规范允许低速滚动转向，窗口约束可放宽为 $|v|\le v_{roll}$（如 $0.1\text{m/s}$），部分摆动在滚动中完成以缩短静止时间——静止转向对轮胎的磨损与回正力矩需求是该开关的主要权衡。

1. **结果校验清单（全部通过才允许输出）。** ① 碰撞复检；② $\delta/\omega/a/j$ 全时限值复检；③ 终点双指标 $0.05\text{m}/1.5^\circ$ 复检；④ 长度比 $L/L_0 \le 1+\rho_{len}$（超标说明发生了路径蠕变，若启用第 8 状态 $\ell$ 守卫仍超标则判失败）；⑤ 换挡次数对比报告（输入 A* 换挡数 vs 输出换挡数，作为融化/修剪效果的在线指标记录日志）；⑥ **接缝原地转向可行性复检**：逐接缝核对 $T_{dwell,j} \ge T_{resteer}(\Delta\delta_j)$、驻留窗内 $|\omega|,|\eta|$ 留有余量、窗口端点 $\omega\approx0$ 且窗末 $\delta$ 已稳定到下一段 maneuver 所需值——任一不过说明窗口定宽或 padding 不足，按第 4 步的规则放大后重跑阶段二，不得带病输出。

2. **输出与失败回退。** 输出格式为时间序列 $[t, x, y, \theta, v, a, \delta, gear]$（$gear$ 由 $\mathrm{sign}(v)$ 结合驻留段恢复，$v=0$ 窗口内保持前一非零档位或按下游协议填空档）+ maneuver 元数据（每段 $s_m, \Delta s_m, \Delta\theta_m$、起止时间、PIVOT 标志）。任一校验不过或求解失败（$\rho_{reg}$ 超限、外层迭代耗尽、终点双指标不达标）时，**回退原始混合 A* 路径**并记录完整诊断日志（失败阶段、约束残差、$\mu$ 终值、缺陷范数），保证模块在任何输入下都有安全输出——这与 NMPC 生产模块的兜底语义一致。
