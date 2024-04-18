import matplotlib.pyplot as plt
import numpy as np


# 两个车的轨迹为同向毕竟一个点 

# 场景最大限速 



""" 斥力场定义：
U(q) = 1/2 * k * (1/d(q, q0) - 1 / d_0)^2

k : 斥力尺度因子 

d(q,q0) : q0为障碍物的位置，q为车辆的位置  自车与障碍物之间的距离 

d0   影响半径 
"""
import random
from mpl_toolkits.mplot3d import Axes3D
DELTAT = 0.1
TIMEALL = 3

def repulsive_force(q, q0, k, d0):
    d = np.sqrt((q[0] - q0[0])**2 + (q[1] - q0[1])**2)
    print(d)
    if d < d0:
        force = k * (1 / d - 1 / d0) * 1 / d ** 2
        fx =force * np.abs(q[0] - q0[0]) / d
        fy =force * np.abs(q[1] - q0[1]) / d
        return fx,fy
    else:
        return 0,0
    
def model(state, action):
    # state: [x, y, theta, v]
    # action: [w, a] 
    
    x = state[0] + DELTAT * (state[3] + DELTAT * action[1]) * np.cos(state[2])
    y = state[1] + DELTAT * (state[3] + DELTAT * action[2]) * np.sin(state[2])
    theta = state[2] + DELTAT * action[0]
    v = state[3] + DELTAT * action[1]
    return [x, y, theta, v]

def action_seq_q():
    # 仅生成加速度为
    ans = []
    n = int(TIMEALL / DELTAT)
    for i in range(n):
        ans.append([- 2 / 180 * np.pi,0.5 + random.uniform(-0.3, 0.3),0.5 + random.uniform(-0.3, 0.3)])
    return ans

def main():
    # 车辆初始位置
    q = np.array([-1, 3, -30/180 * np.pi,3])
    # 障碍物位置
    q0 = np.array([0, 0, 30 / 180 * np.pi,3])
    q_act = action_seq_q()
    # 斥力尺度因子
    k = 1
    # 影响半径
    d0 = 10
    # 斥力场
    ans_q0 = []
    ans_q = []  
    F_list = [] 
    for w,a,a in q_act:
        Fx,Fy = repulsive_force(q, q0, k, d0)
        # print(Fx,Fy)
        q0 = model(q0,[0,Fx,Fy])
        q = model(q,[w,a,a]) # 
        F_list.append([Fx,Fy])
        # break
        ans_q0.append(list(q0))
        ans_q.append(list(q))
    print(ans_q)
    print(ans_q0)
    print(F_list)
    fig = plt.figure()
    ax = fig.add_subplot(111,projection='3d')
    # 绘制场景
    ax.scatter([i*0.1 for i in range(len(ans_q0))],[i[0] for i in ans_q0],[i[1] for i in ans_q0],label='q0')
    ax.scatter([i*0.1 for i in range(len(ans_q))],[i[0] for i in ans_q],[i[1] for i in ans_q],label='q')
    ax.axis('equal')
    plt.legend()
    plt.show()

main()