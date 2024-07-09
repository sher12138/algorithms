"""
LQR实现及测试

线性模型：
x_{t+1} = A * x_t + B * u_t + c
y_t = C * x_t + D * u_t + c

输入：
goal: 目标状态
A, B, C, D, c: 线性模型参数
Q, R: 状态和控制成本权重
T: 时间步数

输出：
u: 控制输入

测试：


纵向模型:

A = np.array([[1, 0, -T * v_r * sin \phi_r], [0, 1, T * v_r * cos \phi_r], [0, 0, 1]])

B = np.array([[T * cos \phi_r, 0], [T * sin \phi_r, 0], [T*tan(phi_r) / l, T * v_r / (l*cos^2(phi_r))]])

x(k+1) = A * x(k) + B * u(k)


曲线坐标系下追踪:

s = 


"""

class IDM:
    class Param:
        kminmumGap = 2.0  #最小车距 
        kDesiredHeadwayTime = 1.5 # 期望车头时距
        kAcceleration = 1.0 # 加速度 
        kComfortableBrakingDeceleration = 1.5 # 舒适制动减速度 
        kVehicleLength = 5.0 # 车宽 
        kDesiredSpeed = 30.0 # 期望车速 
        kDelta = 4.0

    def __init__(self) -> None:
        pass 

    def get_desired_acc(self,cur_state):
        pass 


    def generate_trajectory(self):
        pass 


import numpy as np

class State:
    def __init__(self, x, y, theta, v):
        self.x = x
        self.y = y
        self.theta = theta
        self.v = v


def lqr(A, B, Q, R):
    pass 








## TEST 
## 匀加速直线运动，

# 生成一条匀加速转向轨迹 
init_state = State(0, 0, 0, 0)
generate(init_state,)

