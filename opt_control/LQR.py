# 最优控制问题 LQR 



# Q:构建一个状态序列，求最优控制 
# 已知状态方程 x_{t+1} = A x_t + B u_t + w_t

from dataclasses import dataclass

@dataclass
class State:
    x: float =0.0 
    y: float = 0.0
    speed: float = 0.0
    angle: float = 0.0
    omega: float = 0.0


# 运动学模型 状态量 s = [x, y, speed, angle], 控制量 u = [acceleration, omega]
# x_{t+1} = A x_t + B u_t + w_t
# 观测模型 y = C x + v_t
# 1. 状态方程
# x = s[0] + speed * cos(angle) * dt
# y = s[1] + speed * sin(angle) * dt
# speed = s[2] + u[0] * dt
# angle = s[3] + speed * tan(u[1]) / L * dt

# 2. 观测方程

class StateModel():
    def __init__(self) -> None:
        self.A = [
            []
        ]
        self.B = 


# method 1: 代数Riccati 方程 
"""
J = \sum  x^T Q x + u^T R u  dt
存在最优控制量 u_t = -k x_t 
J = \sum  x^T Q x + x^T K^T R K x  dt
J = \sum x^T (Q + K^T R K) x  dt
假设存在一个正定矩阵P：  d(x^T P x)/dt = x^T (Q + K^T R K) x
J = \int_0^T d(x^T P x)/dt  dt
J =x^T P x|_{t=\inf} -  x^T P x|_{t=0} 
如果x 构造为误差形式，假设t -> \inf , 系统趋于稳定， 那么 J = x^T P x|_{t=\inf} = 0
J = -x^T P x|_{t=0}  
# 也就是，如果系统稳定，那么最优控制量是 u_t = -k x_t，正定矩阵P满足 d(x^T P x)/dt = x^T (Q + K^T R K) x 

A^T P A - P - A^T P B (R + B^T P B)^{-1} B^T P A + Q = 0



"""


# method 2:  动态规划


