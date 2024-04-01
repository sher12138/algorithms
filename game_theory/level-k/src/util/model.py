"""离散运动学模型

x(t+1) &= x(t) + v(t) cos(\theta (t)) \Delta t \\
y(t+1) &= y(t) + v(t) sin(\theta (t)) \Delta t \\
v(t+1) &= v(t) + a(t) \Delta t \\
\theta (t+1) &= \theta (t) + \omega (t) \Delta t \\

"""
import math 


class Model:
    """给定一个状态和动作，返回下一个状态
        state = [x,y,v,\theta]
        action = [a,w]
    """
    @staticmethod
    def step(self,state,action,dt):
        x = state[0] + state[2] * math.cos(state[3]) * dt
        y = state[1] + state[2] * math.sin(state[3]) * dt
        v = state[2] + action[0] * dt
        theta = state[3] + action[1] * dt
        return [x,y,v,theta]
    

    """
    action seq
    """
    @staticmethod
    def step(self,state,action_seq,dt):
        ans_state = []
        for action in action_seq:
            ans_state.append(self.step(state,action,dt))
        return ans_state 
    @staticmethod
    def fack_generate(self,start_state,end_state,t, agent):
        """基于开始状态，生成动作序列和中间状态,匀速圆周运动到达位置"""
        x0,y0,v0,theta0 = start_state
        x1,y1,v1,theta1 = end_state
        r = math.sqrt((x1-x0)**2 + (y1-y0)**2)
        theta = math.atan2(y1-y0,x1-x0)
        omega = theta / t
        a = (v1 - v0) / t
        action_seq = []
        for i in range(t):
            action_seq.append([a,omega])
        agent.add_act_seq(action_seq)

