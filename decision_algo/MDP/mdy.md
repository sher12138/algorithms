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