import numpy as np 
import matplotlib.pyplot as plt
import math 
import random 

"""
            B
        y
A x|a11,b11| |a12,b12|
   |a21,b21| |a22,b22|

a 自车收益
b 它车收益
"""



k_param_a =1.
k_param_c = 1.0



def a11(t1_in,t1_out,t2_in,t2_out):
    return  -1.0 / t1_in - k_param_a * t1_out  # 
def a12(t1_in,t1_out,t2_in,t2_out):
    return 1.0 / t1_in + k_param_a * t1_out   # 
def a21(t1_in,t1_out,t2_in,t2_out):
    return 1. / t1_in - k_param_a * t1_out   # 
def a22(t1_in,t1_out,t2_in,t2_out):
    return 1.0 / t1_in - k_param_a * t1_out   #
def b11(t1_in,t1_out,t2_in,t2_out):
    return -1.0 / t2_in - k_param_a * k_param_c * t2_out  # 
def b12(t1_in,t1_out,t2_in,t2_out):
    return 1.0 / t2_in - k_param_a * t2_out  #
def b21(t1_in,t1_out,t2_in,t2_out):
    return 1.0 / t2_in + k_param_a * t2_out # 
def b22(t1_in,t1_out,t2_in,t2_out):
    return 1.0 / t2_in - k_param_a * t2_out

# 自车期望收益
def mean_f(x,y, t1_in,t1_out,t2_in,t2_out):
    return x * (y * a11(t1_in,t1_out,t2_in,t2_out) \
                + (1-y)* a12(t1_in,t1_out,t2_in,t2_out))  \
        + (1-x) * (y * a21(t1_in,t1_out,t2_in,t2_out) \
                   + (1-y)* a22(t1_in,t1_out,t2_in,t2_out))

def mean_g(x,y,t1_in,t1_out,t2_in,t2_out):
    return y * (x * b11(t1_in,t1_out,t2_in,t2_out) \
                + (1-x) * b21(t1_in,t1_out,t2_in,t2_out)) \
        + (1-y) * (x * b12(t1_in,t1_out,t2_in,t2_out) \
                   + (1-x) * b22(t1_in,t1_out,t2_in,t2_out))

# 动态反应方程
# x ( f1 - mean_f)
def f(x,y, t1_in,t1_out,t2_in,t2_out):
    return x *(1-x) * ( y*(a11(t1_in,t1_out,t2_in,t2_out) - a21(t1_in,t1_out,t2_in,t2_out)) \
                        +(1-y)*(a12(t1_in,t1_out,t2_in,t2_out) - a22(t1_in,t1_out,t2_in,t2_out)))

# 动态反应方程求解 
def g(x,y, t1_in,t1_out,t2_in,t2_out):
    return y *(1-y) * ( x*(b11(t1_in,t1_out,t2_in,t2_out) - b12(t1_in,t1_out,t2_in,t2_out)) \
                        +(1-x)*(b21(t1_in,t1_out,t2_in,t2_out) - b22(t1_in,t1_out,t2_in,t2_out)))

def solver_point(t1_in,t1_out,t2_in,t2_out):
    point = [[0,0],
             [0,1],
             [1,0],
             [1,1]
    ]

    s_point = [(b22(t1_in,t1_out,t2_in,t2_out)-b21(t1_in,t1_out,t2_in,t2_out)) / (b11(t1_in,t1_out,t2_in,t2_out) - b12(t1_in,t1_out,t2_in,t2_out) - b21(t1_in,t1_out,t2_in,t2_out) + b22(t1_in,t1_out,t2_in,t2_out)),
            (a22(t1_in,t1_out,t2_in,t2_out) - a12(t1_in,t1_out,t2_in,t2_out))/ (a11(t1_in,t1_out,t2_in,t2_out) - a21(t1_in,t1_out,t2_in,t2_out) - a12(t1_in,t1_out,t2_in,t2_out) + a22(t1_in,t1_out,t2_in,t2_out))
    ]

    return point, s_point

def stability_verification(t1_in,t1_out,t2_in,t2_out):
    point, s_point = solver_point(t1_in,t1_out,t2_in,t2_out)
    ans = []
    for x,y in point:
        fxx  = (1-2*x) * (y * a11(t1_in,t1_out,t2_in,t2_out) - y * a21(t1_in,t1_out,t2_in,t2_out) + (1-y)*(a12(t1_in,t1_out,t2_in,t2_out) - a22(t1_in,t1_out,t2_in,t2_out)))
        fxy = x*(1-x)*((a11(t1_in,t1_out,t2_in,t2_out) - a21(t1_in,t1_out,t2_in,t2_out) - a12(t1_in,t1_out,t2_in,t2_out) + a22(t1_in,t1_out,t2_in,t2_out)))

        fyx = y*(1-y)*((b11(t1_in,t1_out,t2_in,t2_out) - b12(t1_in,t1_out,t2_in,t2_out) - b21(t1_in,t1_out,t2_in,t2_out) + b22(t1_in,t1_out,t2_in,t2_out)))
        fyy = (1-2*y) *(x * b11(t1_in,t1_out,t2_in,t2_out) - x * b12(t1_in,t1_out,t2_in,t2_out) + (1-x) * (b21(t1_in,t1_out,t2_in,t2_out) - b22(t1_in,t1_out,t2_in,t2_out)))
        if(fxx * fyy - fxy * fyx <= 0):
            continue
        if (fxx + fyy >= 0):
            continue
        print(f"point ({x},{y}) is stability point!")
        ans.append([x,y])
    if 0 <= s_point[0] <= 1 and 0 <= s_point[1] <= 1:
        print(f"point ({s_point[0]},{s_point[1]}) is stability point!")
        return ans, s_point
    return ans, None

# 演化积分
def calculateValue(initX, initY, dt, epoch, t1_in,t1_out,t2_in,t2_out):
    x = []
    y = []

    x.append(initX)
    y.append(initY)
    for index in range(epoch):
        tempx = x[-1] + (f(x[-1], y[-1], t1_in,t1_out,t2_in,t2_out)) * dt
        tempy = y[-1] + (g(x[-1], y[-1], t1_in,t1_out,t2_in,t2_out)) * dt

        x.append(tempx)
        y.append(tempy)
    return (x, y)


# 

def plot(t1_in,t1_out,t2_in,t2_out):
    plot_data =[] 
    point,center_point = stability_verification(t1_in,t1_out,t2_in,t2_out)
    if center_point == None:
        return 
    for index in range(200):
        random_a=random.random()
        random_b=random.random()
        line_points =calculateValue(random_a,random_b,0.001,1000,t1_in,t1_out,t2_in,t2_out)
        plot_data.append(line_points)
    for x,y in plot_data:
        plt.plot(x,y)
    
    plt.scatter(center_point[0],center_point[1],c='r')
    plt.legend()
    plt.show()



def main():
    
    
    s1_in = 2
    v1 = 4
    s1_out = s1_in + 2 

    s2_in = 20
    s2_out = s2_in + 2
    v2 = 0.5

    t1_in = s1_in / v1 
    t1_out = s1_out / v1 

    t2_in = s2_in / v2 
    t2_out = s2_out / v2 
    print(t1_in,t1_out,t2_in,t2_out)
    print(f"a11:{a11(t1_in,t1_out,t2_in,t2_out)},b11:{b11(t1_in,t1_out,t2_in,t2_out)} | a12:{a12(t1_in,t1_out,t2_in,t2_out)},b12:{b12(t1_in,t1_out,t2_in,t2_out)} \n a21:{a21(t1_in,t1_out,t2_in,t2_out)},b21:{b21(t1_in,t1_out,t2_in,t2_out)} | a22:{a22(t1_in,t1_out,t2_in,t2_out)},b22:{b22(t1_in,t1_out,t2_in,t2_out)}")
    point,center_point = stability_verification(t1_in,t1_out,t2_in,t2_out)
    # plot(t1_in,t1_out,t2_in,t2_out)

main()

