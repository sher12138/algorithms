<!--
 * @Author: chengxing 1208109938@qq.com
 * @Date: 2024-07-01 10:15:41
 * @LastEditors: chengxing 1208109938@qq.com
 * @LastEditTime: 2024-07-02 10:29:10
 * @FilePath: /av_algorithm_valid_platform/home/cxx/Documents/algorithms/planning/opt_algo/cilqr.md
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%A
-->


## 迭代求解基本思路 

$$
Q(k) = q(k) + V(k+1) \\ 
V(k) = \min_{u(k)} Q(k) \\
$$

其中 
$Q(k)$ 为 $k$ 时刻的代价函数，任意动作。
$V(k)$ 为 $k$ 时刻的值函数，最优动作。
$q(k)$ 为 $k$ 时刻的状态转移代价函数。k --> k+1 
$u(k)$ 为 $k$ 时刻的控制量。

优化模型完整描述：

$$
\begin{align}
min_{u_1,...,u_n} &= \sum_{k=1}^{n} C(k) \\
s.t. x_{k+1} &= f(x_k,u_k) \\
C(k)&=\frac{1}{2}\begin{bmatrix}X_k\\U_k\end{bmatrix}^T\begin{bmatrix}C_{xx}&&C_{xu}\\C_{ux}&&C_{uu}\end{bmatrix}\begin{bmatrix}X_k\\U_k\end{bmatrix}+\begin{bmatrix}X_k\\U_k\end{bmatrix}^T\begin{bmatrix}D_x\\D_u\end{bmatrix}
\end{align}
$$

$C(k)$ 指k时间步的代价函数，也是代价船体式子中，Q(k)的一部分。

于是问题就是求出最优的$u_1,...,u_n$，使得$\sum C(k)$最小。

### 2.1 迭代求解控制率

**动规倒推**

边界：
最后一步，只有状态转移代价，尾巴代价为0
$$
V(end) = 0 
q(end-1) = ?  
$$


状态转移方程：
$$
Q(k) = q(k) + V(k+1) \\ 
V(k) = \min_{u(k)} Q(k) \\
$$

### 2.2 问题描述

已知期望轨迹 goal_traj，求解最优控制量 u* 使得获得满足约束条件下的最优轨迹。

假设代价可以写成形式：
$$
Q(k)=\frac{1}{2}\begin{bmatrix}X_k\\U_k\end{bmatrix}^T\begin{bmatrix}C_{xx}&&C_{xu}\\C_{ux}&&C_{uu}\end{bmatrix}\begin{bmatrix}X_k\\U_k\end{bmatrix}+\begin{bmatrix}X_k\\U_k\end{bmatrix}^T\begin{bmatrix}D_x\\D_u\end{bmatrix}
$$
最优控制率可以通过对控制量求导获得，无约束二次规划问题。 
$$
\frac{\partial Q(99)}{\partial U_{99}}=0
$$
求导：
$$
u^* = K(k)X_k +k(k) \\
K(k) = -C_{uu}^{-1}C_{ux} \\
k(k) = -C_{uu}^{-1}D_u
$$

最优控制率代入Q(k) -->  V(k) 
$$
\begin{align}
V(k) &= \min_{u(k)} Q(k) \\
&= \frac{1}{2} X_k^T V_{k} X_k + X_k^T v_k + V(k+1) \\ 
\end{align}
$$
式中：
$$
\begin{align} 
V_{k} &= C_{xx} + K_{k} C_{ux} + C_{xu} K_{k} + K_{k} C_{uu} K_{k} \\ 
v_{k} &= D_x + K_{k} D_u + C_{xu} k_{k} + K_{k} C_{uu} K_{k} 
\end{align}
$$
于是，V的更新只需要知道 X_k, V(k+1)即可。X_k 通过下一帧的状态量和控制量反推出来即可。
动规倒推：




```python
V_ans = [0] * end 
for k in range(n-1,-1,-1):
    # 计算Q(k) 
    Q = q_k(state_k) + V_ans[k+1] 
    # 计算V(k)
    V[k] = min(Q)
# 每一步的代价即为 V_ans 

# min(Q) 怎么求，给定一个控制量的函数 u，如何求一个u^* 使的Q最小，

def q_k(k):  # 从状态k --> k+1 的代价函数
    pass 


def min_Q(Q): # 最小Q，求解最优控制量
    pass 

```



### 2.3 ILQR 
如果cost 函数、系统状态方程非线性，线性化 
$$
X_{k+1} = f(X_k,U_k) \approx f(X_k,U_k) + \frac{\partial f}{\partial X} \Delta X + \frac{\partial f}{\partial U} \Delta U \\ 
C_{k+1} = C(X_{k+1},U_{k+1}) \approx C(X_{k+1},U_{k+1}) + \frac{\partial C}{\partial X} \Delta X + \frac{\partial C}{\partial U} \Delta U + \frac{1}{2} \Delta X^T \frac{\partial^2 C}{\partial X^2} \Delta X + \frac{1}{2} \Delta U^T \frac{\partial^2 C}{\partial U^2} \Delta U + \Delta X^T \frac{\partial^2 C}{\partial X \partial U} \Delta U
$$

代价增量：
$$
\Delta C = \frac{\partial C}{\partial X} \Delta X + \frac{\partial C}{\partial U} \Delta U + \frac{1}{2} \Delta X^T \frac{\partial^2 C}{\partial X^2} \Delta X + \frac{1}{2} \Delta U^T \frac{\partial^2 C}{\partial U^2} \Delta U + \Delta X^T \frac{\partial^2 C}{\partial X \partial U} \Delta U
$$

代价的自定义，代价增量则可以表示为二次型。
定义好代价函数，然后求增量。所以，如果代价定义为各项的和，则增量代价梯度表示为 lx,lu,lxx,luu,lxu,lux这些值组成了增量代价函数中的二次型部分，是求差分式获得的。
$$
\delta Q(k)= \frac{1}{2}\begin{bmatrix}\delta X_k\\\delta U_k\end{bmatrix}^T\begin{bmatrix}l_{xx}&&l_{xu}\\l_{ux}&&l_{uu}\end{bmatrix}\begin{bmatrix}\delta X_k\\\delta U_k\end{bmatrix}+\begin{bmatrix}\delta X_k\\\delta U_k\end{bmatrix}^T\begin{bmatrix}l_x\\l_u\end{bmatrix}+ \delta V(k+1) \\ 
\delta X(k+1)= \begin{bmatrix}f_x&&f_u\end{bmatrix}\begin{bmatrix}\delta X_k\\\delta U_k\end{bmatrix}
$$

增量的形式问题在于，非常值，都是增量，需要求的有两个值，一个是增量的控制量，一个是增量的k+1时刻后的最优代价。 

并不知道怎么求增量形式的k+1时刻的最优代价。

1. 已知代价的形式必然是二次型。
$$
\delta V(k+1) = 
$$


代入X(k+1) 
$$
\begin{aligned}
&\delta Q(k)= \frac{1}{2}\begin{bmatrix}\delta X_{k}\\\delta U_{k}\end{bmatrix}^{T}\begin{bmatrix}l_{xx}&&l_{xu}\\l_{ux}&&l_{uu}\end{bmatrix}\begin{bmatrix}\delta X_{k}\\\delta U_{k}\end{bmatrix}+\begin{bmatrix}\delta X_{k}\\\delta U_{k}\end{bmatrix}^{T}\begin{bmatrix}l_{x}\\l_{u}\end{bmatrix}+ \\
&\frac12\quad(\begin{bmatrix}f_x&f_u\end{bmatrix}\begin{bmatrix}\delta X_k\\\delta U_k\end{bmatrix})^TV_{k+1}\quad(\begin{bmatrix}f_x&f_u\end{bmatrix}\begin{bmatrix}\delta X_k\\\delta U_k\end{bmatrix})\quad+\quad(\begin{bmatrix}f_x&f_u\end{bmatrix}\begin{bmatrix}\delta X_k\\\delta U_k\end{bmatrix})^Tv_{k+1}
\end{aligned}\\
\delta Q(k)= \frac{1}{2}\begin{bmatrix}1\\\delta X_k\\\delta U_k\end{bmatrix}^T\begin{bmatrix}0&Q_x^T&Q_u^T\\Q_x&Q_{xx}&Q_{xu}\\Q_u&Q_{ux}&Q_{uu}\end{bmatrix}\begin{bmatrix}1\\\delta X_k\\\delta U_k\end{bmatrix}
$$ 
二次型各参数的值
$$
Q_x = l_x + f_x^T v_{k+1} \\
Q_u = l_u + f_u^T v_{k+1} \\
Q_{xx} = l_{xx} + f_x^T V_{k+1} f_x \\
Q_{xu} = l_{xu} + f_x^T V_{k+1} f_u \\
Q_{ux} = l_{ux} + f_u^T V_{k+1} f_x \\
Q_{uu} = l_{uu} + f_u^T V_{k+1} f_u \\
$$

对控制量求导，求解最优控制量
$$
\delta U_k^* = -Q_{uu}^{-1}Q_{ux} \delta X_k \\ 
K_k = -Q_{uu}^{-1}Q_{ux} \\
k_k = -Q_{uu}^{-1}Q_{u}
$$

代入Q(k) --> V(k)
$$
\delta V(k) = -0.5 Q_u Q_{uu}^{-1} Q_u\\
V_k = Q_x - Q_u Q_{uu}^{-1} Q_{ux} \\ 
v_k = Q_{xx} - Q_{xu}Q_{uu}^{-1}Q_{ux}
$$












