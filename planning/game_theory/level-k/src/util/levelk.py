
"""
Level-k 模型 --> 两对象交互版 

Action :定义一个完整的动作，定义动作空间，定义随机获取一个动作，
State: 一个动作序列对应的状态序列中的每个动作对应的状态 

ActionSeqence: 一个动作序列，对应一个状态序列，对应一个value值

Agent: 一个agent对应一个交互体，会存储对应level- 下的最优动作序列，以及代理的真实动作序列 

EgoAgent: 不存 P？ 存储不同level下的最优动作序列。 

InterGame: 交互游戏，存储代理 --> 

Algorithm:
1. 构建一个交互游戏 
2. 分别计算非自车代理在各个level下的最优动作序列及对应的奖励 
3. 计算\sum_k=0^{n} R_k(\gamma)  --> 计算自车的采样轨迹的奖励，筛选出最大的一条。



Algorithm:  计算代理在某个level下的最优动作序列及对应的奖励
1. 初始化代理的信念状态
2. 生成对应level的最优动作序列:如果代理level a, 假定自车level a-1,逐级回馈并计算代价，反推出代理的最优动作序列 


"""

import random 
import model 
import matplotlib.pyplot as plt 
from matplotlib.animation import FuncAnimation
from convex_opt import solver


time_gap = 0.1

time_count = 100
delta_k = 0.1
LEVEL_SPACE = [0,1,2]
ActionSpace = [[-2,-1,0,1,2],
                   [-20,-10,0,10,20]]
class Action:

    def __init__(self, a = 0,w = 0) -> None:
        self.a = self.validate_a(a)
        self.w = self.validata_w(w)
    
    
    @staticmethod
    def validate_a(a):
        if a in ActionSpace[0]:
            return a 
        else:
            raise ValueError("a value is out of range")


    @staticmethod
    def validata_w(w):
        if w in ActionSpace[1]:
            return w 
        else:
            raise ValueError("w value is out of range") 

    
def random_act(self):
    a_sample = random.randrange(0,len(self.ActionSpace[0]))
    w_sample = random.randrange(0,len(self.ActionSpace[0]))
    return Action(a = a_sample,w = w_sample)


class State:
    def __init__(self,x = 0 ,y = 0, a= 0, w = 0 ) -> None:
        self.x = x 
        self.y = y 
        self.a = a 
        self.w = w


"""
一条动作序列

"""
class ActionSeqence:
    def __init__(self) -> None:
        self.act_seq = []
        self.value = 0

    def add_act(self,act:Action):
        self.act_seq.append(act)

class StateSeqence(ActionSeqence):
    def __init__(self,init_state) -> None:
        super().__init__()
        self.init_state = init_state


class Trajectory:
    def __init__(self) -> None:
        self.state_seq = StateSeqence()

""""""
class Agent:

    def __init__(self) -> None:
        self.level_p = [0.33,0.33,0.34]
        self.level_opt_tra = [None,None,None]
        self.real_act_seq = []
        self.predict_act_seq = [] 
        self.current_state = State()
        
    
    def get_level_opt_tra(self,level):
        if level - 1 >= 0:
            return self.level_opt_tra[level-1]
        else:
            return [self.current_state] * time_count

    def create_traj_for_level():
        """创建指定level的对应轨迹"""
        pass 

    def add_real_act_seq(self,act_seq:list[Action]):
        self.real_act_seq.append(act_seq)

    def reset_real_act_seq(self):
        self.real_act_seq.clear()

    def generate_opt_tra(self,tra,level):
        # 基于给出的 对象的 tra ，更新level下的结果 
        self.real_act_seq[level] = LevelK.trajectory_generate(ego,tra)  # 轨迹生成部分 


class EgoAgent(Agent):
    def __init__(self,aim) -> None:
        super().__init__()
        self.aim_pos = aim

class Math:
    @staticmethod
    def man_hat_ton(act_orins:Action,act_real:Action):
        ans = []
        for act_orin in act_orins:
            ans.append(abs(act_orin[0].a - act_real.a) + abs(act_orin[0].w - act_real.w))
        return ans 

class Util:
    @staticmethod
    def min_index(arr):
        return arr.index(min(arr))
    @staticmethod
    def standard(level_k):
        # 基于给的概率，归一化为概率和为1 
        sum_p = sum(level_k)
        for i in range(len(level_k)):
            level_k[i] /= sum_p
        return level_k

class LevelK:
    @staticmethod
    def belief_update(agent:Agent,agent_act:Action):
        # 信念更新
        update_k = Util.min_index(Math.man_hat_ton(agent.level_opt_tra,agent_act))
        agent.level_p[update_k] += delta_k
        agent.level_p = Util.standard(agent.level_p)
        print("belief update:", agent.level_p)

    # 为代理 ego_update 生成 等级为 level 的最优轨迹
    @staticmethod 
    def opt_act_generate(ego:EgoAgent,agent:Agent,ego_update = True,level = 0):
        if ego_update:
            update_obj = ego 
            game_obj = agent
        else:
            update_obj = agent 
            game_obj = ego    

        # 获取对应level的轨迹 
        game_obj_tra = game_obj.get_level_opt_tra(level-1)
        # 基于 博弈对象的优化轨迹，生成一条指定level的轨迹，并获取采样动作 
        update_obj.generate_opt_tra(game_obj_tra,level) 


    @staticmethod
    def trajectory_generate(ego:Agent,tra):
        # 轨迹生成，  A* 
        # 优化 --> 
        solver(ego,tra)





    def arg_max_R(ego:EgoAgent,agent:Agent):
        pass 



class InterGame:
    def __init__(self,ego:EgoAgent,agent:Agent) -> None:
        self.ego = ego
        self.agent = agent
        
    def step(self,agent_act):
        # 分别生成各自 各level下的动作序列
        LevelK.belief_update(self.agent,agent_act) # 更新 P 矩阵
        # 生成代理和自车对应level的最优动作序列
        for i in LEVEL_SPACE:
            LevelK.opt_act_generate(ego,agent,True,i)  # 更新自车指定Level的opt轨迹 
            LevelK.opt_act_generate(ego,agent,False,i)  # 更新博弈对象指定level的opt轨迹
        # 寻找最合理轨迹 
        ego.predict_act_seq = LevelK.arg_max_R(ego,agent)  

        self.ego.update_state()  # 应用博弈结果的第一个 
        self.agent.update_state()  # 应用 传入状态 

        
    def run(self,agent_seq):
        # 迭代过程中，信念也在不断更新，换言之，信念最大的轨迹也在不断变化，因此，需要每次基于信念对结果做修正来反推自车安全轨迹。
        for act in agent_seq:
            self.step(act)
        

    def animate_draw(self,agent_seq):
        fig,ax = plt.subplots()
        line, =ax.plot([],[],lw =2 )
        ax.set_ylim(0,10)
        ax.set_xlim(0,100)
        xdata,ydata = [],[]
        def init():
            line.set_data([],[])
            return line 
        ani = FuncAnimation(fig,self.step,frames = agent_seq,init_func = init,blit=True)
        plt.show()





class Utils:
    pass 

if __name__ == "__main__":
    """
    两代理测试 
    """
    ego = EgoAgent()  # 未知
    agent = Agent()  # 已知
    # 预处理为动作序列 
    model.Model.fack_generate(start = State(5,5,1,0),end = State(1,1,2,0), agent = agent)
    # 构建游戏 
    game =  InterGame(ego,agent)

    game.run(agent.real_act_seq)
    