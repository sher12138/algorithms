import matplotlib.pyplot as plt
from pylab import *
import math
import random
plt.rcParams['axes.unicode_minus'] = False
mpl.rcParams['font.sans-serif'] = ['SimHei']

k = 1
SIGMA = 1E-6

k_param_a =1.
k_param_c = 1.0

def a11(t1_in,t1_out,t2_in,t2_out):
    return  -1.0 * t1_in - k_param_a / t1_out  # 时间：  安全 通过收益 + 碰撞风险
def a12(t1_in,t1_out,t2_in,t2_out):
    return 1.0 * t1_in + k_param_a / t1_out   # 自车收益应该适当降低，优先让行，如何表征？  通过收益 + 快速通过收益
def a21(t1_in,t1_out,t2_in,t2_out):
    return 1. / t1_in - k_param_a * t1_out   # 通过收益 + 快速通过损失 + 安全收益
def a22(t1_in,t1_out,t2_in,t2_out):
    return 1.0 / t1_in - k_param_a * t1_out   # 双让的效率损失如何衡量？
def b11(t1_in,t1_out,t2_in,t2_out):
    return -1.0 * t2_in - k_param_a * k_param_c / t2_out  # 时间：  安全
def b12(t1_in,t1_out,t2_in,t2_out):
    return 1.0 * t2_in + k_param_a / t2_out  # 通过损失，时间越大，快速通过损失越小。
def b21(t1_in,t1_out,t2_in,t2_out):
    return  1.0 / t2_in - k_param_a * t2_out # 时间：  安全
def b22(t1_in,t1_out,t2_in,t2_out):
    return 1.0 / t2_in - k_param_a * t2_out


def GeneralTrans(s_e_s, s_e_e, s_a_s, s_a_e, v_e, v_a):
    a = 1.0
    b = 1.0
    t1 = s_e_e / v_e
    t2 = s_a_e / v_a
    k1 = v_e / s_e_s
    k2 = v_a / s_a_s
    return a, b, t1, t2, k1, k2


def HeuristicTrans(s_e_s, s_e_e, s_a_s, s_a_e, v_e, v_a, h_a):
    a = 1.0
    b = 1.0
    t2 = s_a_e / v_a
    k2 = v_a / s_a_s
    if h_a < 0.0:
        if math.pow(v_e, 2) / (2.0 * h_a) < s_e_s:
            return a, b, 1e+8, t2, 1e+8, k2
        if math.pow(v_e, 2) / (2.0 * h_a) < s_e_e:
            ttc = CalEquation(0.5 * h_a, v_e, -s_e_s)
            k1 = 1.0 / ttc
            return a, b, 1e+8, t2, k1, k2
    ttc = CalEquation(0.5 * h_a, v_e, -s_e_s)
    t1 = CalEquation(0.5 * h_a, v_e, -s_e_e)
    k1 = 1.0 / ttc
    return a, b, t1, t2, k1, k2


def CalEquation(a, b, c):
    return (-b + math.sqrt(math.pow(b, 2) - 4.0 * a * c)) / (2.0 * a)


# def g(x, y, v0, t0, s0, ve, te, se):
#     return y * (x*t0 - x*k/((t0-te + SIGMA)**2) + t0 - ve*ve / 2 / se - x*t0 + x * ve*ve / se / 2 - x*v0 *v0 / s0 / 2 + y * ve*ve /2 / se + \
#                 + x*y*v0 * v0 / s0 /2 - x*y*ve * ve / se/2 + x*y*k /((t0 - te + SIGMA)**2) - 2 * y * t0 + t0 
#                                   )


# def f(x, y, v0, t0, s0, ve, te, se):
#     return x * (-y*k / (t0-te + SIGMA)**2 - v0*v0 / s0 /2 + y * v0*v0 /s0 /2 - 2*x*te - x*y*v0*v0 /s0 /2 + x*y*ve*ve / se /2 + x* v0*v0 / s0 /2 +\
#                  -y * ve * ve / se /2 + x*y*k / (t0 - te + SIGMA)**2   )
def f(x, y, v0, t0, s0, ve, te, se):
    return x * (1 -x) * (y * (a11(t0,te)-a12(t0,te)+a22(t0,te)) - a22(t0,te) )

def g(x, y, v0, t0, s0, ve, te, se):
    return y * (1 - y) * (x * (b11(t0,te)-b21(t0,te)+b22(t0,te)) - b22(t0,te))


def calculateValue(initX, initY, dt, epoch, v0,t0,s0,ve,te,se):
    x = []
    y = []

    x.append(initX)
    y.append(initY)
    for index in range(epoch):
        tempx = x[-1] + (f(x[-1], y[-1], v0,t0,s0,ve,te,se)) * dt
        tempy = y[-1] + (g(x[-1], y[-1], v0,t0,s0,ve,te,se)) * dt

        x.append(tempx)
        y.append(tempy)
    return (x, y)


def PlotEvolution(prob_set, v0, t0, s0, ve, te, se):
    D = []
    for param in prob_set:
        x = param[0]
        y = param[1]
        d = calculateValue(x, y, 0.01, 1000, v0, t0, s0, ve, te, se)
        D.append(d)
    return D


if __name__ == '__main__':
    # s_ego_s = 6.0
    # s_ego_e = 7.0
    # s_agent_s = 5.0
    # s_agent_e = 5.5
    # v_ego = 3.0
    # v_agent = 2.0

    v0 = 3
    t0 = 2.10
    s0 = v0 * t0
    ve= 3
    te = 2.11
    se = ve * te
    # rr1 = te - k / (t0 - te + SIGMA)**2
    # rr2 = t0 - k / (t0 - te + SIGMA)**2
    # rh1 = te - v0 / s0 / 2
    # rh2 = -t0 + v0 / s0 / 2

    # hr1 = -te + ve*ve / 2 / se
    # hr2 = t0 - ve*ve / 2 / se
    # hh1 = - te 
    # hh2 = - t0
    # print(rr1,rr2,rh1,rh2,hr1,hr2,hh1,hh2)
    # a, c, t1, t2, k1, k2 = GeneralTrans(s_ego_s, s_ego_e, s_agent_s, s_agent_e, v_ego, v_agent
    #                                     )
    # prob_set = []
    # for index in range(200):
    #     random_a=random.random()
    #     random_b=random.random()
    #     d=calculateValue(random_a,random_b,0.001,1000,v0, t0, s0, ve, te, se)
    #     prob_set.append(d)

    prob_set = [[0.5, 0.5],[0.01,0.95]]
    x = a22(t0,te) / (a11(t0,te) - a12(t0,te) + a22(t0,te))
    y = b22(t0,te) / (b11(t0,te) - b21(t0,te) + b22(t0,te))
    D = PlotEvolution(prob_set, v0, t0, s0, ve, te, se)
    for n,m in D:
        plt.plot(n,m)     

    # plt.scatter(x,y)  
    plt.scatter(y,x)
    plt.ylabel("$y$", fontsize=18)
    plt.xlabel("$x$", fontsize=18)
    plt.xticks([0, 0.2, 0.4, 0.6, 0.8, 1])
    plt.legend()

    # prob_set = [[0.5, 0.5]]
    # h_a_set = [0.5, 1.0, 1.5]
    # D = []
    # D_H = []
    # for h_a in h_a_set:
    #     a, c, t1, t2, k1, k2 = HeuristicTrans(v0, t0, s0, ve, te, se, h_a
    #                                           )
    #     D = PlotEvolution(prob_set, a, c, t1, t2, k1, k2)
    #     D_H.append(D[0])
    # # print(D_H)
    # plt.figure(2)
    # t_set = [x * 0.01 for x in range(500)]
    # plt.plot(t_set, (D_H[0][0])[:500], label='heuristics a=0.5')
    # plt.plot(t_set, (D_H[1][0])[:500], label='heuristics a=1.0')
    # plt.plot(t_set, (D_H[2][0])[:500], label='heuristics a=1.5')
    # # plt.plot(t_set, (D_H[3][0])[:500], label='heuristics a=-0.5')
    # # plt.plot(t_set, (D_H[4][0])[:500], label='heuristics a=-1.0')
    # # plt.plot(t_set, (D_H[5][0])[:500], label='heuristics a=-1.5')

    # plt.ylabel("$x$", fontsize=18)
    # plt.xlabel("$t$", fontsize=18)
    # plt.legend()

    # plt.figure(3)
    # t_set = [x * 0.01 for x in range(500)]
    # plt.plot(t_set, (D_H[0][1])[:500], label='heuristics a=0.5')
    # plt.plot(t_set, (D_H[1][1])[:500], label='heuristics a=1.0')
    # plt.plot(t_set, (D_H[2][1])[:500], label='heuristics a=1.5')
    # # plt.plot(t_set, (D_H[3][1])[:500], label='heuristics a=-0.5')
    # # plt.plot(t_set, (D_H[4][1])[:500], label='heuristics a=-1.0')
    # # plt.plot(t_set, (D_H[5][1])[:500], label='heuristics a=-1.5')

    # plt.ylabel("$y$", fontsize=18)
    # plt.xlabel("$t$", fontsize=18)
    # plt.legend()

    plt.show()



# 17


# 23  a multi-point turn decision making framework is designed
# based on the combination of the real human driving data and
# the vehicle dynamics for human-like autonomous driving
    

#     [26] A game theoretic lane-change model is built in 

# In [27], the interactions between the
# host vehicle and surrounding ones are captured in a formulated
# game, which is supposed to make an optimal lane-change
# decision to overtake, merge and avoid collisions.