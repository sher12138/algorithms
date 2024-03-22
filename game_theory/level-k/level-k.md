
# Game Theoretic Modeling of Vehicle Interactions at Unsignalized Intersections and Application to Autonomous Vehicle Control

## 算法设计：
**控制模型**：
$$
\begin{equation}
\begin{align*}
x(t+1) &= x(t) + v(t) cos(\theta (t)) \Delta t \\
y(t+1) &= y(t) + v(t) sin(\theta (t)) \Delta t \\
v(t+1) &= v(t) + a(t) \Delta t \\
\theta (t+1) &= \theta (t) + \omega (t) \Delta t \\
\end{align*}
\end{equation}
$$

**Action set**:
每一个动作时一个二元组$\gamma = (a(t), \omega (t))$，其中$a(t)$是车辆的加速度，$\omega (t)$是车辆的角速度。动作集合包括以下几种动作：
$$
\begin{equation}
\begin{align*}
"Maintain" &\rightarrow a(t) = 0, \omega (t) = 0 \\
"Accelerate" &\rightarrow a(t) = 2.5, \omega (t) = 0 \\
"Decelerate" &\rightarrow a(t) = -5, \omega (t) = 0 \\
"Turn Left" &\rightarrow a(t) = 0, \omega (t) = \pi / 4 \\
"Turn Right" &\rightarrow a(t) = 0, \omega (t) = \pi / 4 \\
\end{align*}
\end{equation}
$$

**Action selection**:The approach to selecting actions is based on receding horizon optimization

*累加奖励：*
$$
\R(\gamma) = \sum_{i=0}^{n-1} \gamma^i R_i{(\gamma)} \\
R_i{(\gamma)} = R( \gamma_i | s_i) \\ 
式中 s_i  表示环境状态。
$$

每个时间 $t$ 选择一系列的动作 $\gamma = (\gamma_0, \gamma_1, \cdots, \gamma_{n-1})$，其中 $\gamma_i = (a_i, \omega_i)$。每个动作 $\gamma_i$ 有一个奖励 $R_i(\gamma_i)$。累加奖励 $\R(\gamma)$ 是所有动作的奖励的和。 目的是在 "a horizon of length n"最大化累加奖励。
假设在每个步骤顺序应用动作

**找一个奖励最高的动作序列，用该序列的第一个动作作为当前动作。**

**奖励函数**：

$$
\begin{align*}
R =& \omega_1 \hat{c} + \omega_2 \hat{s} + \omega_3 \hat{o} + \omega_4 \hat{l} + \omega_5 \hat{d} \\ 
\hat{c}:& 定义每个车辆的几何碰撞区，如何区相交，\hat{c} = 1，否则 \hat{c} = 0 \\
\hat{s}:& 定义包括c区的安全区，如果区相交，\hat{s} =1 \\
\hat{o}:& 如果c区合道路边界外相交，\hat{o} = 1 \\
\hat{l}:& 必须遵守交通规则，\hat{l} = 0, else \hat{l} = -1 \\
\hat{d}:&  -\hat{d} = \lvert{x-x_r} \rvert + \lvert{y-y_r} \rvert

\end{align*}
$$

**交通状态预测**： --> game_theory 

level-k 博弈策略，动作选择方法：
$$
\begin{equation}
\begin{align*}
\R_k(\gamma) &= \R(\gamma | \hat{\gamma}_{k-1}^{other}) = \sum_{i=0}^{n-1} R_i(\gamma | \hat{\gamma}_{k-1}^{other})  \\ 
\hat{\gamma}_k^{ego}  &\in \argmax_{\gamma_i \in \Tau} \R_k(\gamma), 
\end{align*}
\end{equation}
$$

给定其它车辆的动作序列 $\hat{\gamma}_{k-1}^{other}$，ego车辆选择一个动作序列 $\hat{\gamma}_k^{ego}$，使得累加奖励最大。

假设其它车辆的动作序列为 level-(k-1) 

## 示例

假如自车是 l2, 首先规划一条 l0 的动作序列，然后根据 l0 的动作序列规划它车 l1 的动作序列，基于它车序列规划自车 l2 的动作序列。 如果它车应用 l1的动作序列，那么自车应用 l2的动作序列。 

**简化**：假设它车永远比自车低 1 级。

## 控制器设计：

假设：通过交互不断修正从而能够解决冲突。

引入 belief 表征其它代码能够以 level-k 规划自身行为的置信度。

人类通常是： 0， 1， 2 级相互，很少是 3 级。

## 博弈控制器 --> 
自车对每个它车的模型持有一个 belief ， belief 是一个概率分布，表示它车的 level-k 的概率。
    通过分析每个代码真实动作来在每个更新步修正belief 

**2-agent level-k controller**:

ego: P(k) 其它车辆被建模为 level-k 的概率分布。
p = [p0,p1,p2]  --> 3 个概率值， 

一条动作序列的期望累加奖励 --> 

$$
\R_P(\gamma) = \sum_{k=0}^2 P(k) \R(\gamma | \hat{\gamma}_k^{other})
$$
找到后，自车应用该动作序列的第一个动作。

其它代理依据自己的真实动作更新自己的 belief。
$$
k^\prime \in \argmin_{k \in \{0,1,2\}} || \gamma^{other} - \hat{\gamma}_{k,0}^{other} || \\ 
P(k^\prime)  \leftarrow P(k^\prime) + \Delta P,\\
P(k) \leftarrow P(k) / ( \sum^2_{k=0} P(k)) 
$$