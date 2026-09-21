# 基于iLQR的APA模块后处理方法

## 参考文献

### DDP 与 iLQR 基础

DDP 的出发点是 Bellman 方程的局部递推。记离散动力学 $x_{k+1}=f(x_k,u_k)$、运行代价 $\ell(x,u)$，则在每一步求解

$$V(x) = \min_u \left[ \ell(x,u) + V'(f(x,u)) \right]$$

其中 $V'$ 为下一时刻的价值函数。动作价值函数 $Q(\delta x,\delta u)$ 在 nominal 轨迹 $(\bar x,\bar u)$ 处作二阶展开（nominal 量以上横线标记），其各阶导数为

$$
\left\{
\begin{aligned}
Q_x &= \ell_x + f_x^T V'_x \\
Q_u &= \ell_u + f_u^T V'_x \\
Q_{xx} &= \ell_{xx} + f_x^T V'_{xx} f_x + V'_x \cdot f_{xx} \\
Q_{ux} &= \ell_{ux} + f_u^T V'_{xx} f_x + V'_x \cdot f_{ux} \\
Q_{uu} &= \ell_{uu} + f_u^T V'_{xx} f_u + V'_x \cdot f_{uu}
\end{aligned}
\right.
$$

其中 $f_x,f_u$ 为动力学雅可比矩阵（第二章记 $A_k,B_k$），末项 $V'_x\cdot f_{xx}$ 等是向量-张量积（动力学二阶导数张量与价值梯度沿第一维收缩）。**是否保留这组张量项正是 DDP 与 iLQR 的唯一区别**：完整 DDP 保留张量项、具有局部二次收敛率；iLQR 舍弃了这一项以换取每轮迭代的大幅提速。本repo中的求解器默认采用 iLQR/Gauss-Newton 变体。

对展开的二次型关于 $\delta u$ 求极值，得到最优控制率：

$$\delta u^*(\delta x) = k + K\delta x,\qquad k = -Q_{uu}^{-1}Q_u,\qquad K = -Q_{uu}^{-1}Q_{ux}$$

其中 $k$ 是前馈项，$K$ 是局部反馈增益，接下来有：

$$\Delta V = -\tfrac{1}{2}k^T Q_{uu}k,\qquad V_x = Q_x - K^T Q_{uu}k,\qquad V_{xx} = Q_{xx} - K^T Q_{uu}K$$

**后向传递（backward pass, BP）** 从终点 $V_x=\ell_{f,x}(x_N)$、$V_{xx}=\ell_{f,xx}(x_N)$ 出发，沿时间轴反向递推全部 $k_k,K_k$；
**前向传递（forward pass, FP）** 以非线性动力学做一次真正的 rollout，一般带有线搜索：

$$\hat u_i = u_i + \alpha k_i + K_i(\hat x_i - x_i),\qquad \hat x_{i+1}=f(\hat x_i,\hat u_i)$$

步长 $\alpha$ 从 1 回溯衰减，直到实际代价下降被接受。注意反馈项中的 $\hat x_i-x_i$ 是实际 rollout 状态与 nominal 状态之差，这使控制更新天然是一个沿新轨迹闭合的反馈律，而非开环修正。

从这里我们也可以看到相较于直接法DDP在复杂度上的结构性优势：把一个 $Nm$ 维的联合优化问题拆成 $N$ 个独立的 $m$ 维局部问题，每步只需对 $m\times m$ 的 $Q_{uu}$ 做一次分解，单轮 BP/FP 复杂度 $O(Nm^3)$，对时域长度 $N$ 线性；而若把整个问题当作单个 $Nm$ 维的QP问题，时间复杂度是 $O(N^3m^3)$。

### Box-QP

传统的iLQR不太擅长处理约束问题，而实际的控制量几乎总是有盒约束 $b \le u \le \bar b$，这些来自于执行器的硬极限必须满足，如EPS给前轮转角和其转动速率的盒约束，iBCU给减速度的盒约束。

> Yuval Tassa, Nicolas Mansard, Emo Todorov. *Control-Limited Differential Dynamic Programming*. IEEE International Conference on Robotics and Automation (ICRA), 2014.

Tassa et al. 的这篇论文中对比了几种施加盒约束的方法：

- **naive clamping**：BP 忽略约束、FP 把越限控制截断到边界。截断后的控制不再对应 BP 解出的下降方向，代价可能不降反升，线搜索大面积拒绝，收敛严重退化；
- **squashing**：用 sigmoid 型光滑函数把无约束控制压进盒内。此方法的问题是引入了人工非线性，当控制落在 sigmoid 平台区时梯度趋于零，求解器丢失向边界回退的信号，收敛同样可能退化。

最后采用的方法是把约束移入 Bellman 局部问题：每一步 BP 解的不是无约束二次型，而是一个盒约束 QP（box-QP）：

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

即只有自由控制分量参与反馈；**被钳制的控制分量对应的 $K$ 行恒为零**——这些控制量已钉死在边界上，对状态扰动不应再作反馈响应。工程注记：相邻时间步的 QP 相似，因此第 $k$ 步的活动集应**以第 $k+1$ 步（BP 顺序的上一步）的活动集热启动**，绝大多数步一次分解即收敛。实现位置提示：`src/core/iLQR/box_qp.h`。