# MDP

状态价值
$$
\begin{equation}
\begin{align}
v(s,\pi) &= E(G_t | S_t = s)\\
&=E(r_{t+1} + \gamma G_{t+1} | S_t = s)  \\ 
&=E(r_{t+1} | S_t = s) + \gamma E(G_{t+1} | S_t = s)\\
\end{align}   
\end{equation}
$$

两部分单独计算：
$$
\begin{equation}
\begin{align}
E(r_{t+1} | S_t = s) &= \sum_{a} \pi(a|s) \sum_{s',r} p(s',r|s,a) r\\
E(G_{t+1} | S_t = s) &= \sum_{a} \pi(a|s) \sum_{s',r} p(s',r|s,a) v(s',\pi)\\
\end{align}
\end{equation}
$$

动作价值
$$
q(s,a,\pi) = E(G_t | S_t = s, A_t = a)
$$

$$
q(s,a,\pi) = \sum_r p(r | s,a) r + \gamma \sum_{s`n} p(s^{next} | s, a) v(s^{next}, \pi)
$$


## MDP模型建立

状态空间定义：$S = \{s_1, s_2, s_3, s_4, s_5\}$
动作空间定义：$A = \{a_1, a_2\}$
状态转移概率：$P_{ss^{'}}^a = P(S_{t+1} = s^{'} | S_t = s, A_t = a)$
奖励函数：$R_s^a = E(R_{t+1} | S_t = s, A_t = a)$

本文所研究的问题是如何用尽可能少的历史状态来尽可能准确的预测出它车的行为轨迹。

考虑本文实际要解决的是如何识别出前车是否为异动车辆的问题。

