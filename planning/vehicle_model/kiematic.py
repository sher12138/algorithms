
"""
运动学模型：

模型离散、差分推导流程：
拿到非线性形式后，需要模型线性化， 

\dot{(x)} = f(x,u)  --- 1

在参考点附近做泰勒展开，设参考点 x_ref,u_ref 

\dot{x - x_ref} = f(x_ref,u_ref) + df/dx *(x- x_ref) + df/du * (u - u_ref)  --- 2

由1式可以得到 
\dot{x} - \dot{x_ref} = df/dx * (x - x_ref) + df/du * (u - u_ref)  --- 3

于是可以构建误差形式模型
\dot{x_e} = A * x_e + B * u_e --- 4 
误差形式可以修改为离散形式 
(x(k+1) - x(k)) / T = A * x(k) + B * u(k)  --- 5

x(k+1) = (TA + E) * x(k) + TB * u(k)  --- 6



1. 二自由度小车模型 
vx = v * cos(theta)
vy = v * sin(theta)
theta_dot = v * tan(delta) / L

状态量： x,y,phi
控制量： v, delta

A = [
    [0,0,-vsin(theta)],
    [0,0,vcos(theta)],
    [0,0, 0]
]

B = [
    [cos(theta),0],
    [sin(theta),0],
    [tan(delta)/L, v/(L*cos(delta)**2)]
]

TA + E = [
    [1,0,-T vsin(theta)],
    [0,1,T vcos(theta)],
    [0,0, 1]
]

TB = [
    [T cos(theta),0],
    [T sin(theta),0],
    [T tan(delta)/L, T v/(L*cos(delta)**2)]
]

"""

import numpy as np

class KinematicBicycle:
    def __init__(self, L=2.9, dt=0.1) -> None:
        self.L = L 
        self.dt = dt 

    def update(self, x, u):
        x[0] += u[0] * np.cos(x[2]) * self.dt
        x[1] += u[0] * np.sin(x[2]) * self.dt
        x[2] += u[0] * np.tan(u[1]) / self.L * self.dt
        return x 
    # 输入参考，获取矩阵
    def jacobian(self, x, u):
        A = np.array([
            [1, 0, -u[0] * np.sin(x[2]) * self.dt],
            [0, 1, u[0] * np.cos(x[2]) * self.dt],
            [0, 0, 1]
        ])

        B = np.array([
            [np.cos(x[2]) * self.dt, 0],
            [np.sin(x[2]) * self.dt, 0],
            [0, u[0] * self.dt / (self.L * np.cos(u[1])**2)]
        ])

        return A, B 


"""
状态量： x,y,theta,delta
控制量： v, \dot{delta}

\dot{X} = f(x,u)= [
vcos(theta),
vsin(theta),
v/L * tan(delta),
\dot{delta}
]

沿参考点展开 
\dot{x} = f(x_ref,u_ref) + [[0,0,-vsin(theta),0],
                            [0,0,vcos(theta),0],
                            [0,0,0,0],
                            [0,0,0,0]] * (x - x_ref) + [cos(theta),sin(theta),1/Ltan(delta),0],
                                                        [0,0,0,1] * (u - u_ref)

\dot{x_e} = A (x_e) + B(u_e)

x(k+1) = (TA + E) * x(k) + TB * u(k)
"""

class KinematicBicycle2:
    def __init__(self) -> None:
        self.L = 2.4 
        self.dt = 0.1 

    def jacabi(self,x_ref,u_ref):
        A = [[0,1,-self.dt * u_ref[0] * np.sin(x_ref[2]),0],
                            [0,1,self.dt* u_ref[0] * np.cos(x_ref[2]),0],
                            [0,0,1,0],
                            [0,0,0,1]]
        
        B = [
            [np.cos(x_ref[2]),np.sin(x_ref[2]),1/self.L* np.tan(x_ref[3]),0],
            [0,0,0,1]
        ]

        return A, B




"""
曲线坐标系下运动学模型：

定义沿轨迹的长度为s， 令 \theta_p(s) 为全局坐标系下路径s点处的切线与x轴的夹角， 
车辆朝向的航向角误差定义为：
\theta_e = \theta = \theta_p(s) 

轨迹点的曲率定义为：
k(s) = \frac{d\theta_p(s)}{ds}   曲线偏离直线的程度。

定义e_{ra} 为车辆后轮中心到参考轨迹的横向误差
\dot{s} = vcos(theta_e) + \dot{theta_p} * e_{ra} 
\dot{e_{ra}} = vsin(theta_e)


状态量： s, e_{ra}, theta_e, delta 
控制量： v, \dot{delta}

\dot{x} = [
cos(theta_e) / (1- e_{ra} k(s)),
sin(theta_e), 
(tan(delta) / L - (k(s) cos(theta_e)) / (1- e_{ra} k(s))), 
0
] v +
[
0,
0,
0,
1
] \dot{delta}

"""



"""

\dot{s} = v 
\dot{v} = a
\dot{a} = j 

参考点附近做泰勒展开 
\dot{s} 

"""
class KinematicFrenetLongtiduinal:
    def __init__(self) -> None:
        pass

    def update(self, s, v, a, j, dt):
        s += v * dt 
        v += a * dt 
        a += j * dt
        return s, v, a
    
    def jacobian(self, s, v, a, j, dt):
        A = np.array([
            [1, dt, 0.5 * dt**2],
            [0, 1, dt],
            [0, 0, 1]
        ])

        B = np.array([1/6 * dt**3 , 1/2*dt**2,dt]).reshape(-1, 1)
        return A, B
    

