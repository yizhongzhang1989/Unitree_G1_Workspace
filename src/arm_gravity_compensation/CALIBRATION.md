# 手臂重力标定的数学

面向 `arm_gravity_compensation`。记号：单臂 7 关节，$q\in\mathbb R^{14}$ 为双臂关节角，
$g\in\mathbb R^{3}$ 为躯干系重力向量，$\tau\in\mathbb R^{7}$ 为单臂关节力矩。

---

## 1. 要解的问题

关节重力矩是质量的线性函数。第 $i$ 个连杆质心的雅可比记 $J_{c_i}(q)$，则

$$
G(q,g)\;=\;\sum_i m_i\,J_{c_i}(q)^{\mathsf T} g .
$$

URDF 给出标称质量 $\hat m_i$，但它与实物有出入（贴装、线缆、工具）。引入**每连杆一个缩放**
$m_i=s_i\hat m_i$，再给每个关节一个**常值偏置** $b_j$ 吸收标定不了的部分（减速器预载、
电流环零漂）：

$$
\boxed{\;G(q,g;s,b)\;=\;\underbrace{\sum_i s_i\,\hat m_i J_{c_i}(q)^{\mathsf T} g}_{\text{对 }s\text{ 线性}}\;+\;b\;}
$$

**参数对观测是线性的**，这是整套方法的前提。待估 $\theta=(s,b)$。

$g$ 取自躯干 IMU 而非假设竖直，所以躯干倾斜时标定依然成立。

---

## 2. 观测方程

电机内部闭环，实际输出

$$
\tau_{\text{applied}} \;=\; \tau_{\text{ff}} \;+\; k_p\,(q_{\text{ref}}-q)\;-\;k_d\,\dot q .
$$

三项全部已知或可测，所以**即使手臂没精确停在目标点，观测量依然精确**。静止时
$\dot q=0$，关节上的力矩平衡为

$$
\tau_{\text{applied}} \;+\; G(q,g) \;+\; \tau_f \;=\; 0 ,
\qquad |\tau_f|\le \tau_s .
$$

朴素做法令 $\tau_{\text{applied}}=-G$，**这一步默认了 $\tau_f=0$**。

### 2.1 摩擦是主误差项

$\tau_f$ 在 $[-\tau_s,+\tau_s]$ 内**取值不定**——静力平衡对它欠定，关节停在哪儿，
摩擦就取多少。所以单次静止采样的观测量带着最多一个 $\tau_s$ 的误差。

本机实测 $\tau_s$（14 轴，梯形波反转法）：

| | 肩 pitch | 肘 | 肩 roll | 腕 pitch | 腕 yaw |
|---|---|---|---|---|---|
| $\tau_s$ (N·m) | 0.69–0.77 | 0.38–0.55 | 0.30–0.38 | 0.22–0.31 | 0.05–0.11 |

而 $|G|$ 本身在 0.03–4.7 N·m。**误差占比 5%–50%**。作为佐证：改进前左臂的标定残差
中位 0.081 N·m，恰好落在 $\tau_s$ 的量级内——**残差主要就是摩擦，不是模型误差**。

### 2.2 双向逼近

从两侧各逼近同一位姿，把 $\tau_f$ 钉在已知符号上：

$$
\begin{aligned}
\text{自下而上停：}&\quad \tau^{\uparrow}=-G(q^{\uparrow})+\tau_c\\
\text{自上而下停：}&\quad \tau^{\downarrow}=-G(q^{\downarrow})-\tau_c
\end{aligned}
\;\Longrightarrow\;
\boxed{\;\frac{\tau^{\uparrow}+\tau^{\downarrow}}{2}=-\,\frac{G(q^{\uparrow})+G(q^{\downarrow})}{2}\approx -G(\bar q)\;}
$$

摩擦项**恒等消去，与 $\tau_c$ 多大无关**。残留只有方向不对称性
$\tfrac12(\tau_c^{+}-\tau_c^{-})$，二阶量。$q^{\uparrow},q^{\downarrow}$ 相距一个死区，
$G$ 在其上的变化是二阶，取平均位形即可。

实测该平均量的组内相对离散度 **3.3%**，而同一批数据里 $\tau_s$（两方向之**差**）
是 30.3%——**和比差稳 9 倍**：突破点判定偏早时两方向误差反号，在和里抵消、在差里翻倍。

**唯一的实现陷阱**：退让幅度必须超过死区

$$
\Delta_{\text{retreat}} \;>\; \frac{2\tau_s}{k_p}\quad(\text{实测最大 }0.107\ \text{rad}) .
$$

否则关节根本没挪到另一侧，两次停在同一点、同一摩擦符号，平均下来**等于没做**。
故 `approach_offset_rad` 默认 0.12。

---

## 3. 可辨识性：连杆分组

焊接在同一运动体上的连杆，只通过**聚合质量与一阶矩**进入关节力矩——单独估计是奇异的。
设指示矩阵 $A\in\{0,1\}^{L\times 7}$，$A_{ij}=1$ 表示连杆 $i$ 归属关节 $j$ 之后的刚体，
则每组一个缩放 $\sigma\in\mathbb R^{7}$：

$$
s = A\,\sigma .
$$

设计矩阵按列聚合：$\Phi_{\text{group}} = \Phi_{\text{link}}A$。求解后组缩放写回该组每个连杆。

---

## 4. 设计矩阵

第 $i$ 列由"只有连杆 $i$ 带标称质量、其余置零"的模型算出：

$$
\Phi(q,g)\;=\;\bigl[\,G_1(q,g)\;\cdots\;G_L(q,g)\;\big|\;I_7\,\bigr]\in\mathbb R^{7\times(L+7)},
\qquad G_i \;=\; \hat m_i J_{c_i}^{\mathsf T}g .
$$

各列用 Pinocchio 的 `computeGeneralizedGravity` 在单连杆基模型上求得；右侧 $I_7$ 是偏置列。
$N$ 个位姿纵向堆叠：

$$
y \;=\; \Phi\,\theta \;+\; \varepsilon,\qquad
\Phi\in\mathbb R^{7N\times 14},\quad \theta=(\sigma,b) .
$$

只选中部分关节标定时，未选中的列用当前值折进 $y$，只对选中列求解。

---

## 5. 加权

每行按该关节的假定静态力矩噪声归一，起点为额定力矩的 1%：

$$
\sigma_j^{(0)} = \rho\,\tau_j^{\text{rated}},\qquad \rho = 0.01,
\qquad \tilde\Phi = W^{-1}\Phi,\;\; \tilde y = W^{-1}y,\;\; W=\operatorname{diag}(\sigma_j).
$$

高额定力矩的关节噪声按比例更大，因此不会仅凭绝对力矩大就主导拟合。

---

## 6. MAP 估计

$\sigma$ 与 $b$ 都有物理先验（质量不该偏离 URDF 太多、偏置应该小），故解最大后验而非最小二乘：

$$
\hat\theta \;=\; \arg\min_{\theta\in[\ell,u]}
\;\bigl\lVert \tilde\Phi\theta-\tilde y\bigr\rVert^2
\;+\;\sum_j \lambda_j\,(\theta_j-\theta_j^{\text{prior}})^2
$$

$$
\lambda_{\sigma} = \frac{\eta^2}{0.3^{2}},\qquad
\lambda_{b,j} = \frac{\eta^2}{(0.05\,\tau_j^{\text{rated}})^{2}},
$$

盒约束 $\sigma\in[0.2,3.0]$、$b\in[-8,8]$ N·m 由 `lsq_linear` 的 TRF 直接处理。
实现上把先验写成增广行（对 $\delta=\theta-\theta^{\text{prior}}$ 求解），
与数据行一起做一次有界最小二乘。

### 噪声水平自洽

先验只应按**真实**噪声的比例压过数据，而 $\eta$ 未知，故迭代（至多 5 次）：

$$
\eta \;\leftarrow\; \hat\sigma_{\text{resid}}\sqrt{\frac{n_{\text{eff}}}{n_{\text{eff}}-p}} ,
$$

$n_{\text{eff}}=\sum p_k$ 为有效样本数，$p$ 为参数个数。**自由度修正不可省**：位姿集勉强
定出参数时，残差会假性变小，不修正就会让先验被错误地压过去。

---

## 7. 鲁棒：双高斯混合 EM

采样中会混入被碰到、没停稳、或卡在异常摩擦态的位姿。用两个零均值高斯建模残差，
宽的那个吸收扰动并强制至少宽 $\kappa=10$ 倍：

$$
r_k \;\sim\; \pi\,\mathcal N(0,\eta^2)\;+\;(1-\pi)\,\mathcal N(0,\eta_{\text{out}}^2),
\qquad \eta_{\text{out}}\ge\kappa\eta .
$$

**按位姿整块判定**：一次扰动同时污染该位姿的 7 行，所以对数似然比按 block $\mathcal B$ 求和。

**E 步**

$$
\ell_{\mathcal B}=\sum_{k\in\mathcal B}\left[\log\frac{\eta}{\eta_{\text{out}}}
+\frac{r_k^2}{2\eta^2}-\frac{r_k^2}{2\eta_{\text{out}}^2}\right]
+\log\frac{1-\pi}{\pi},
\qquad
p_{\mathcal B}=\frac{1}{1+e^{\ell_{\mathcal B}}} .
$$

**M 步**　等效精度归一到 $\eta^2$ 单位，得权重

$$
w_k \;=\; p_k+(1-p_k)\frac{\eta^{2}}{\eta_{\text{out}}^{2}} ,
$$

以 $w$ 重解第 6 节的有界 ridge，再更新

$$
\eta^2=\frac{\sum_k p_k r_k^2}{\sum_k p_k},\qquad
\eta_{\text{out}}^2=\max\!\left(\frac{\sum_k (1-p_k) r_k^2}{\sum_k (1-p_k)},\;\kappa^2\eta^2\right),\qquad
\pi=\overline{p_{\mathcal B}} .
$$

离群位姿被整块降权而不是逐行裁剪——逐行裁会把一个倾斜位姿里"碰巧对得上"的那几个
关节留下，反而把偏差焊进解里。

---

## 8. 诊断量

**秩**　在噪声单位下，一个方向被数据分辨的条件是奇异值超过噪声：

$$
\operatorname{rank}=\#\{\varsigma_i(\tilde\Phi)>\eta\},\qquad
\text{nullity}=p-\operatorname{rank}.
$$

**条件数**　先按列范数归一再取奇异值比，衡量的是姿态集的形状而非量纲。

**可观测性**　后验方差相对先验的收缩，逐参数给出

$$
o_j \;=\; 1-\Bigl[\bigl(\tilde\Phi^{\mathsf T}\tilde\Phi/\eta^2+\Lambda\bigr)^{-1}\Bigr]_{jj}\lambda_j
\;\in[0,1] ,
$$

$o_j\to0$ 表示该参数完全靠先验（姿态集没约束住它），$o_j\to1$ 表示完全由数据定出。

---

## 9. 标定结果如何执行

电机是位置环，`tau` 通道写死为 0，所以补偿量**折算成位置偏移**下发：

$$
q_{\text{cmd}} = q_{\text{target}} + \frac{\alpha\,G(q_{\text{target}},g)}{k_p}
\;\Longrightarrow\;
\tau = k_p(q_{\text{cmd}}-q) - k_d\dot q = \alpha G + k_p(q_{\text{target}}-q) - k_d\dot q .
$$

$\alpha$ 为 `compensation_scale`（激活时 2 s 内从 0 线性升起）。$g$ 每拍取自躯干 IMU
的低通姿态，所以躯干一动补偿就跟着转。

注意 $G$ 按 $q_{\text{target}}$ 而非 $q$ 求值：前馈不应引入反馈噪声。代价是它与真实的
$G(q)$ 差一个死区上的增量。

---

## 10. 与摩擦补偿的关系

两者共用同一个静力平衡式，只是取和取差：

$$
\underbrace{\tfrac12(\tau^{\uparrow}+\tau^{\downarrow})=-G}_{\text{重力标定}}
\qquad
\underbrace{\tfrac12(\tau^{\uparrow}-\tau^{\downarrow})=\tau_f}_{\text{摩擦辨识}}
$$

所以**一次双向标定同时给出两者**。差值记在 `StaticSample.friction`，随每轮迭代写进
`parameters.json`，并在每侧标定结束时打印（跨位姿取中位数与 p90）。**它不进回归**——
拟合只看重力，摩擦是白拿的副产物。

### 摩擦正比于关节载荷

2026-08-19 实测（11 位姿 × 7 关节 × 两侧）：$\tau_f$ 与 $|\tau_g|$ 的相关系数左臂
0.71~0.96、右臂 0.17~0.90，拟合

$$\tau_f = \mu\,|\tau_g| + \tau_0,\qquad \mu \in [0.13,\ 0.22]$$

$\mu$ 在各关节与两侧高度一致，是减速器载荷相关损耗的典型特征。线性模型的残差比常数
模型小 2~4 倍（左 `shoulder_roll` 0.105 vs 0.453 N·m）。

**这一条决定了补偿能不能安全做**：同一关节的 $\tau_f$ 在不同位姿间能差 11 倍
（左 `shoulder_roll` 0.106~1.205 N·m）。按中位数做常数补偿，在轻载位姿就是过补 10 倍，
等于给关节接了负阻尼。载荷模型则天然安全，而 $\tau_g$ 恰好是重力补偿已经在算的量。

早先把这 30% 的跨位姿离散当成测量噪声是误读——它是载荷在变，不是量不准。

### 顺序：重力标定必须在前

摩擦补偿一旦生效，原先被摩擦顶住的重力残差 $\Delta G$ 会立刻表现为 $\Delta G/k_p$ 的
位置漂移。改进前右臂 $|\Delta G|$ 最大 0.949 N·m，$k_p=14.3$ 即 **0.066 rad = 3.8°**。

---

## 附：符号表

| 符号 | 含义 | 代码 |
|---|---|---|
| $q,\;g$ | 关节角、躯干系重力 | `StaticSample.q`, `.gravity` |
| $\tau_{\text{applied}}$ | 电机总力矩（观测量） | `TorqueStep.applied` |
| $\tau_f,\;\mu$ | 摩擦力矩、载荷系数 | `StaticSample.friction` |
| $s,\;\sigma$ | 连杆 / 分组质量缩放 | `mass_scales`, `group_scales` |
| $b$ | 关节力矩偏置 | `torque_bias` |
| $\Phi$ | 设计矩阵 | `design_matrix()` |
| $A$ | 分组指示矩阵 | `group_aggregation()` |
| $\eta,\;\eta_{\text{out}}$ | 内/外点噪声 | `EMResult.noise_std` |
| $p_{\mathcal B}$ | 内点后验概率 | `inlier_probability` |
| $o_j$ | 可观测性 | `scale_observability` |
| $\alpha$ | 补偿倍率 | `compensation_scale` |
