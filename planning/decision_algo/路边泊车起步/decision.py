# MDP 建模不确定性，判断对方意图。

"""
1. MDP建模 确定状态转移模型S()，奖励函数R，状态空间，动作空间，策略空间\pi
2. 车辆的状态转移，应该满足基本的运动学模型要求。s,a --> s' 
3. 策略空间： 离散空间，策略决定动作的选择。设定不同场景的运行模式。值迭代。遍历每一种策略，返回奖励最大的策略。
4. 奖励函数： ？？ 
"""
import numpy as np
import math
from enum import Enum, unique

# 状态空间：
class State:
    def __init__(self,x,y,theta,v,a):
        self.x = x 
        self.y = y 
        self.theta = theta 
        self.v = v 
        self.a = a

    def __repr__(self) -> str:
        return "s_front: %s, s: %s, v: %s, v_load: %s" % (self.s_front, self.s, self.v, self.v_load)

# 动作定义
class Action:
    def __init__(self,a,delta):
        self.a = a 
        self.delta = delta

    def __repr__(self) -> str:
        return "a: %s, delta: %s" % (self.a, self.delta)


"""
可能行为定义：
1. 安全停止状态   停止 
2. 安全运动状态   运动但是不会对自车产生风险 
3. 风险停止状态   速度为0，但是位置的变化存在运动可能
4. 风险运动状态   低速，但是超车风险较大
"""
@unique
class PolicyBase(Enum):
    kSafeStop = 0 
    kSafeRun = 1 
    kRiskStop = 2
    kRiskRun = 3

class Policy():
    def __init__(self,state) -> None:
        self.generate_action(state)

    def generate_action(self,state:State):
        pass

# 策略空间
class PolicySpace:


def kinimatics_model(state:State, action:Action, delta_t):
    """
    state: x,y,theta,v,a
    action: a, delta
    """
    x = state.x 
    y = state.y 
    theta = state.theta 
    v = state.v 
    a = state.a

    a = action.a 
    delta = action.delta

    x = x + v * math.cos(theta) * delta_t
    y = y + v * math.sin(theta) * delta_t
    theta = theta + v / 2.0 * math.tan(delta) * delta_t
    v = v + a * delta_t
    return x,y,theta,v,a

def state_transfor(state:State,action:Action):
    x,y,theta,v,a = kinimatics_model(state,action,0.1)
    return State(x,y,theta,v,a)

def reward(state:State):
    # 奖励设置
    pass 






    
