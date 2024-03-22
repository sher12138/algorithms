"""
MDP:
    - Markov Decision Process
    - A tuple (S, A, P, R, γ)
        - S: set of states
        - A: set of actions
        - P: state transition probability matrix
        - R: reward function
        - γ: discount factor
    - A discrete-time stochastic control process


v(s,pi)  策略下的状态价值函数 
q(s,a,pi) 策略下的动作价值函数
v(s) = max(q(s,a))  最优状态价值函数
q(s,a) = R(s,a) + γ * Σ P(s'|s,a) * v(s')  贝尔曼方程
q(s,a) = R(s,a) + γ * Σ P(s'|s,a) * max(q(s',a'))  贝尔曼最优方程
R(s,a) = E[r|s,a]  期望奖励
P(s'|s,a) = P(s'|s,a)  状态转移概率
γ: 折扣因子

# 完整版 

v(s) = max(q(s,a))  最优状态价值函数
q(s,a) = R(s,a) + γ * Σ P(s'|s,a) * v(s')  贝尔曼方程
q(s,a) = R(s,a) + γ * Σ P(s'|s,a) * max(q(s',a'))  贝尔曼最优方程
R(s,a) = E[r|s,a] = \sum_r p(r | s,a) r  期望奖励
P(s'|s,a) = P(s'|s,a)  状态转移概率



    
"""
import numpy as np
import random 
import math


# 1. 构建一个 问题


mdp_table = [
    [0,0,10],
    [0,-10,0],
    [0,0,0]
]


# 求最优策略 

state_action_values = np.zeros((9,4))

for i in range(100):
    for s in range(9):
        # 四个动作下的期望
        # ans = 0 
        for a in range(4):
            if a == 0:
                next_state = (s + 3) if s+ 3 < 9 else s
            elif a == 1:
                next_state = (s - 3) if s-3 >= 0 else s
            elif a == 2:
                next_state = (s + 1) if (s+1) % 3 != 0 else s
            else:
                next_state = (s - 1) if s % 3 != 0 else s
            if next_state < 0 or next_state > 8:
                continue
            if next_state == s:
                continue
            # 贝尔曼最优策略 
            # state_action_values[s,a] = mdp_table[s//3][s%3] + 0.9 * state_action_values[next_state].max()
            state_action_values[s,a] = mdp_table[s//3][s%3] + 0.9 * sum(state_action_values[next_state]) / 4 
            # 贝尔曼公式？
            # ans += 0.25 * (mdp_table[s//3][s%3] + 0.9 * state_action_values[next_state][])
            
    print(state_action_values)
    print("----")
"""
MDP / POMDP 求解框架设计

1. 状态空间  s
2. 动作空间  a
3. 状态转移概率  p(s'|s,a)
4. 奖励函数 r(s,a)
5. 策略空间 π 
6. 策略评估 v(s,π)  q(s,a,π)

v(s,π) = \sum_a π(a|s) * (r(s,a) + γ * \sum_s' p(s'|s,a) * v(s',π))
q(s,a,π) = \sum_r p(r |s,a)r  + γ * \sum_s' p(s'|s,a) * v(s',π)


7. 观测空间 o(z | s`,a) 
"""