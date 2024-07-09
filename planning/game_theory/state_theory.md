# 静态博弈
定义各自的收益：
$$
\begin{matrix}
& \text{Player 2} \\
\text{Player 1} & \begin{matrix}
(3, 3) & (0, 5) \\
(5, 0) & (1, 1)
\end{matrix}    
\end{matrix}
$$

静态博弈求解要求各自的收益最大化。

## 一个典型的多对象多策略博弈问题构建

$$
\begin{matrix}
\begin{align*}
定义每个博弈对象 \quad i \quad 的策略集合 \quad :& \\
 S_i &= \{s_{i1}, s_{i2}, ..., s_{im}\} \\
定义博弈对象 \quad i \quad 的收益函数 \quad :\\
&u_i(s_1, s_2, ..., s_n) \\
定义博弈对象 \quad i \quad 的策略组合 \quad : \\
&s_i = (s_{i1}, s_{i2}, ..., s_{in}) \\
定义博弈对象 \quad i \quad 的策略组合空间 \quad :\\
&S = S_1 \times S_2 \times ... \times S_n \\
定义最优反应函数 \quad :\\
&BR_i(s_{-i}) = argmax_{s_i \in S_i} E_{s_i}(u_i(s_i, s_{-i})) \\
定义纳什均衡 \quad :\\
&s^* = (s_1^*, s_2^*, ..., s_n^*) \quad s.t. \quad s_i^* = BR_i(s_{-i}^*)
最优反应函数，求导 == 0，可以求得一阶导为0的点，同时，如果二阶导为负，那么这个点就是局部最优解。
\end{align*}
\end{matrix}
$$

# 静态博弈通用混合策略纳什均衡解法

1. 定义一个博弈问题 $G(S,u)$, 其中 $S$ 是策略空间，$u$ 是效用函数。
2. 定义一个混合策略 $\sigma_i$ ，其中 $\sigma_i$ 是一个概率分布，表示玩家 $i$ 选择策略的概率。
3. 定义一个混合策略纳什均衡 $(\sigma_1^*,...  , \sigma_n^*)$ ，其中 $\sigma_1^*$ 和 $\sigma_n^*$ 是玩家 1 和玩家 2 的最优策略。

求解最优策略：  *最优反应函数*

1.1 二人有限博弈纳什均衡 or 混合策略纳什均衡

```python 

a11 = 
a12 =
a21 =
a22 =
b11 =
b12 =
b21 =
b22 =


payoff = [
    [[a11, b11],[a12, b12]],
    [[a21, b21],[a22, b22]]
]

# 纯策略纳什均衡 --> 最优反应函数

# 混合策略纳什均衡 --> 策略分布函数

```

