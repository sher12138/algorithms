

$$
A^TP + PA - PBR^{-1}B^TP + Q = 0 \\ 
A^T P A - P - A^T P B (R + B^T P B)^{-1} B^T P A + Q = 0 \\ 
P = A^T P A - A^T P B (R + B^T P B)^{-1} B^T P A + Q
$$ 



 代数黎卡提方程推导 

有限时域LQR问题：

$$
U^{\star} = \argmax_U \sum_{t=0}^{T-1} x_t^T Q x_t + u_t^T R u_t + x_T^T Q_T x_T \\
s.t. x_{t+1} = A x_t + B u_t \\
x_0 = x_{init}
$$

Q, Q_T, R 为正定矩阵
终端时刻反向递归 

$$
V_{t} = min_{u_t} x_t^T Q x_t + u_t^T R u_t + V_{t+1} \\  
V_N(x_N) = x_N^T Q_T x_N  \\
\\
Q_t(x_t,u_t) = x_t^T Q x_t + u_t^T R u_t + V_{t+1}(x_{t+1}) \\ 
$$

t时刻的价值函数只和t时刻的状态有关，和之前的状态无关 
$$
V_t(x_t) = x_t^T P_t x_t \\ 
P_N = Q_T 
$$ 


状态函数的递推关系 基于 P_t 的递推关系
$$
Q_t(x_t,u_t) = x_t^T Q x_t + u_t^T R u_t + (Ax + Bu)P_{t+1}(Ax^T + Bu^t)
$$

求每个状态的最优控制 
$$
min_{u}Q_t(x_t,u_t) = x_t^T Q x_t + u_t^T R u_t + (Ax + Bu)^T P_{t+1}(Ax + Bu)  \\
2R^T u_t  + 2B^T P_{t+1}(Ax_t + Bu_t)  = 0\\
u_t = -(R^T + B^T P_{t+1} B)^{-1} B^T P_{t+1} A x_t
\\ 
k_T = -(R + B^T P_{t+1} B)^{-1} B^T P_{t+1} A 
\\ 
代入 Q 
\\
Q_t(x_t,u_t) = V_t(x_t) \\ 
P_t = Q + A^T P_{t+1} A - A^T P_{t+1} B (R + B^T P_{t+1} B)^{-1} B^T P_{t+1} A \\ 离散迭代黎卡提方程
$$

