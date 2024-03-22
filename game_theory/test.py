

alpha = 1 
beta = 1 
theta = 1   

def payoff(t1,t2,v1,v2):

    a11 = alpha * t1 - beta * 1/((t2-t1 + 1e-8)**2)  # 时间：  安全 通过收益 + 碰撞风险
    
    # RF
    a12 = alpha * t1 + beta * 1/t1     # 自车收益应该适当降低，优先让行，如何表征？  通过收益 + 快速通过收益
    # FR
    a21 = - beta * 1/t1 +  theta * 1/(t1 - t2 + 1e-8)   # 通过收益 + 快速通过损失 + 安全收益 
    # RR 
    a22 = -alpha * t1   # 双让的效率损失如何衡量？
    # 它车 
    # RR
    b11 = alpha * t2 - beta * ((t1-t2 + 1e-8)**2 )   # 时间：  安全
    # RF
    b12 = -alpha * t2 + beta * 1/t2   # 通过损失，时间越大，快速通过损失越小。 
    # FR
    b21 =  alpha * t2 + beta * 1/t2 + theta * (t1-t2) # 时间：  安全
    # RR 
    b22 = -alpha * t2
    print(a11,a12,a21,a22,b11,b12,b21,b22)
    return a11,a12,a21,a22,b11,b12,b21,b22

def solver(data):
    a11,a12,a21,a22,b11,b12,b21,b22 = data

    x = a22 / (a11 - a12 + a22)
    y = b22 / (b11 - b21 + b22)

    # 验证 

    xy = [[0,1],[1,0],[1,1],[0,0], [x,y]]

    for _x,_y in xy:
        _a11 = (1 - 2 * _x) *(_y * a11 - _y * a12 - a22 + _y * a22)
        _a12 = _x * (1-_x) * ( a11 -a12 + a22)
        _a21 = _y * (1- _y) * (b11 - b21 + b22)
        _a22 = (1 - 2 * _y) * (_x * b11 - _x * b21 - b22 + _x * b22)

        det_j  = _a11 * _a22 - _a12 * _a21
        tr_j = _a11 + _a22 
        print(det_j, tr_j)
        
        if det_j > 0 and tr_j < 0:
            print(f"({_x},{_y}) is a saddle point")
        else:
            print(f"({_x},{_y}) is not a saddle point")


def cal(s1,s2,v1,v2):
    t1 = s1 / v1
    t2 = s2 / v2
    # if abs(t1-t2) > 2:
    #     return False
    solver(payoff(t1,t2,v1,v2))

import numpy as np 

# t1 =5
# t2 = 5
# t3 = 2
# t4 = 2

# for s1 in np.linspace(3,20,t1):
#     for s2 in np.linspace(3,20,t2):
#         for v1 in np.linspace(1,5,t3):
#             for v2 in np.linspace(1,5,t4):


s1 = 5 
v1 = 2
s2 = 7 
v2 = 1 
best_tra = cal(s1,s2,v1,v2)