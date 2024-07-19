## ILQR 

backword:从最后一步递推求解，获取前馈控制率k与反馈控制率K，直至第一步：
$$
u_{k+1} = K_t x_t + k_t
$$ 
forward:根据已知的初始状态$x_0$，状态转移方程$x_{t+1}=f(x_t,u_t)$, 以及backword过程中所得到的控制率，依次递推得到状态序列$X=\{x_0,x_1,...,x_T\}$

- 某个节点到终端节点的路径的最小累计cost即为该节点的状态价值函数，即$V(k)$
- 从k节点给定一个控制量，其从当前k状态根据系统运动方程转移到k+1状态产生的cost即为，**状态转移代价**$q(k+1 | k,a)$
- 假如从k步给定一个任意的控制量，车辆转移到k+1，而从k+1到终端状态，确保一直选择最优控制量，即为,状态k采取动作a的**动作价值**：$Q(k+1 | k,a) = q(k+1 | k,a) + V(k+1)$
- 寻找最优控制序列问题变成从嵌套的从终端状态寻找最优控制量的子问题。
- 目标函数可以写成X,U二次型，运动学约束可以写成X,U的线性组合，标准LQR优化问题。


### 算法过程
1. 反馈推导最优控制量和反馈控制量：
- $Q(e|e-1,a) = q(e|e-1,a) + V(e)$
- $q(e|e-1,a) = \frac{1}{2}\begin{bmatrix}X_{e-1}\\U_{e-1}\end{bmatrix}^T\begin{bmatrix}C_{xx}&&C_{xu}\\C_{ux}&&C_{uu}\end{bmatrix}\begin{bmatrix}X_{e-1}\\U_{e-1}\end{bmatrix}+\begin{bmatrix}X_{e-1}\\U_{e-1}\end{bmatrix}^T\begin{bmatrix}D_x\\D_u\end{bmatrix}$
- $V(e)$ 终端代价已知为 0，
- 于是$Q(e|e-1,a)$ 可以看做 a，即u的二次函数，此时子问题为一个二次型优化问题，求最好的a时Q的状态代价最小，即为最优控制量。
- 上市是一个单变量的二次优化问题，求导数即可获得最优解。
- $\frac{\delta Q(e-1)}{\delta U_{e-1}}=0 \rightarrow U^*_{e-1} = K_{e-1}X_{e-1} + k_{e-1}$
- 将最优控制量代入Q:
$$
\begin{align*}
Q_{e-1} &= \frac{1}{2}X_{e-1}^T V_{e-1} X_{e-1} + X_{e-1}^T v_{e-1} + const \\ 
 V_{e-1} &= C_{xx} +K_{e-1}^TC_{ux} + C_{xu}K_{e-1} + K_{e-1}^TC_{uu}K_{e-1} \\
 v_{e-1} &= C_{xu}k_{e-1} + k_{e-1}^TC_{uu}k_{e-1} \\
 const &= k_{e-1}D_u+k_{e-1}C_{uu}k_{e-1} + V_{e}
\end{align*}
$$
上述式子，可以解决的问题是：已知$X_{e-1}$的时候，可以计算出最优控制量了，那么，怎么拿到$X_{e-1}$，也就是怎么引入等式约束，起点状态已知，如果按照这个式子递推，，
- 同时满足：$V(e-1) = Q(e|e-1,a^*)$，即为最优状态代价。






## 补充公式求解过程

最优控制量的求解过程：
$$
\begin{align*}
\frac{\delta Q(99)}{\delta U_{99}}&=0 \\
C_{ux} + C_{uu}U_{99} + D_u &= 0 \\
U_{99} &= -C_{uu}^{-1}C_{ux} - C_{uu}^{-1}D_u \\
K_{99} &= -C_{uu}^{-1}C_{ux} \\
k_{99} &= -C_{uu}^{-1}D_u \\ 
\end{align*}
$$
最优状态代价的求解过程：
$$
\begin{align*}
V(e-1) &=  
\frac{1}{2}
\begin{bmatrix}
    X_{e-1}\\
    U_{e-1}
\end{bmatrix}^T
\begin{bmatrix}
    C_{xx} && C_{xu}\\
    C_{ux} && C_{uu}
\end{bmatrix}
\begin{bmatrix}
    X_{e-1}\\
    U_{e-1}
\end{bmatrix}
+
\begin{bmatrix}
    X_{e-1}\\
    U_{e-1}
\end{bmatrix}^T
\begin{bmatrix}
    D_x\\   
    D_u
\end{bmatrix} \\
&=
\frac{1}{2}
\begin{bmatrix}
    X_{e-1}\\
    K_{e-1}X_{e-1} + k_{e-1}
\end{bmatrix}^T
\begin{bmatrix}
    C_{xx} && C_{xu}\\
    C_{ux} && C_{uu}
\end{bmatrix}
\begin{bmatrix}
    X_{e-1}\\
    K_{e-1}X_{e-1} + k_{e-1}
\end{bmatrix}
+
\begin{bmatrix}
    X_{e-1}\\
    K_{e-1}X_{e-1} + k_{e-1}
\end{bmatrix}^T
\begin{bmatrix}
    D_x\\   
    D_u
\end{bmatrix} \\
&=
\frac{1}{2}
(X_{e-1}^T
\begin{bmatrix}
    1 \\
    K_{e-1}
\end{bmatrix}^T
+
\begin{bmatrix}
    0 \\
    k_{e-1}
\end{bmatrix}^T)
\begin{bmatrix}
    C_{xx} && C_{xu}\\
    C_{ux} && C_{uu}
\end{bmatrix}
(
\begin{bmatrix}
    1 \\
    K_{e-1}
\end{bmatrix}
X_{e-1}
+
\begin{bmatrix}
    0 \\
    k_{e-1}
\end{bmatrix})
+
(
    X_{e-1}
\begin{bmatrix}
    1 \\
    K_{e-1}
\end{bmatrix}^T
+
\begin{bmatrix}
    0 \\
    k_{e-1}
\end{bmatrix}^T)
\begin{bmatrix}
    D_x\\   
    D_u
\end{bmatrix} \\ 
&=
\frac{1}{2}
X_{e-1}^T
\begin{bmatrix}
    1 \\
    K_{e-1}
\end{bmatrix}^T
\begin{bmatrix}
    C_{xx} && C_{xu}\\
    C_{ux} && C_{uu}
\end{bmatrix}
\begin{bmatrix}
    1 \\
    K_{e-1}
\end{bmatrix}
X_{e-1}
+
    X_{e-1}
\begin{bmatrix}
    1 \\
    K_{e-1}
\end{bmatrix}^T
\begin{bmatrix}
    D_x\\   
    D_u
\end{bmatrix}
+k_{e-1}D_u
+k_{e-1}C_{uu}k_{e-1} + (k_{e-1}C_{ux} + k_{e-1}K_{e-1}C_{uu})X_{e-1} + X_{e-1}^T(C_{xu}k_{e-1} + k_{e-1}C_{uu}k_{e-1}) \\ 
&=
\begin{align*} 
&\frac{1}{2}
X_{e-1}^T
(C_{xx} +K_{e-1}^TC_{ux} + C_{xu}K_{e-1} + K_{e-1}^TC_{uu}K_{e-1})X_{e-1} \\
&+ X_{e-1}^T(C_{xu}k_{e-1} + k_{e-1}^TC_{uu}k_{e-1})\\
&+k_{e-1}D_u+k_{e-1}C_{uu}k_{e-1} + V_{e}
\end{align*}
\end{align*}  , \mathrm{结果是单值，所以全部可以按照实对称处理，于是X = X^T}\\
$$
###
