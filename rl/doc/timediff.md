# 时序差分

状态值函数的更新公式为：

$$

V_{\pi}(s_t) = \Epsilon_{\pi} (R(s_{t+1}) + \gamma V(s_{t+1} | S = s_t))   \\
V(s) \leftarrow V(s) + \alpha \left( R + \gamma V(s') - V(s) \right)
$$