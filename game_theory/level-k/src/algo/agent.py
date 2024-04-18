"""
Agent 设计 


- P[]  每个概率映射到一个轨迹中
- 轨迹
    - 状态序列 
        - p,v,a,\theta
- cost_cal  每条轨迹都要能够保存自己的代价  

- 


"""
import copy
import utils.utils as utils 
from model.model import Model


LEVEL_SPACE = [0,1,2]
LEVEL = ['level_0','level_1','level_2']
LEN = len(LEVEL)

INIT_TRAJ_COST = 0
INIT_LEVEL_PROB = 0.3


class Action:
    def __init__(self) -> None:
        pass


class State:
    def __init__(self,x,y,heading,v,a,omega) -> None:
        self.x = x 
        self.y = y 
        self.heading = heading
        self.v = v 
        self.a = a 
        self.omega = omega

class StateSeq:
    def __init__(self) -> None:
        self.states = []

    # 追加一个状态 
    def add_state(self,state:State):
        self.states.append(copy.deepcopy(state))

    # 基于动作更新状态
    def add_state_base_act(self,act):
        self.states.append(Model.step(self.states[-1],act))


class Trajectory(StateSeq):

    def __init__(self,level_level='Real') -> None:
        super().__init__()
        self.level_level = level_level
        self.cost = INIT_TRAJ_COST

class Agent:
    
    def __init__(self,init_state) -> None:
        # level  tra
        self.level_tra_map = dict()
        self.tra_cost_map = dict()
        self.level_p =  [INIT_LEVEL_PROB] * LEN

        for level in LEVEL:
            self.level_tra_map[level] = Trajectory(level)
            self.tra_cost_map[level] = INIT_TRAJ_COST  

        
        # real info 
        self.current_state = init_state 
        self.real_tra_seq = Trajectory() 
        
    # 获取对应level的轨迹信息
    def get_level_info(self,level):
        return self.level_tra_map[level]
    def get_tra_cost(self,level):
        return self.tra_cost_map[level]
    
    def update_p(self):
        self.level_p = utils.Math.standard(self.level_p)

    # 添加真实动作序列  
    def add_real_act_seq(self,act_seq:list[Action]):
        for act in act_seq:
            self.real_tra_seq.add_state_base_act(act)
    

class EgoAgent(Agent):
    def __init__(self, init_state,aim_pos) -> None:
        super().__init__(init_state)
        self.aim_pos = aim_pos


class OtherAgent(Agent):
    def __init__(self, init_state,pred_tra) -> None:
        super().__init__(init_state)
        # self.pred_tra = pred_tra  # 一阶段不考虑预测轨迹，纯博弈确定轨迹，并通过信念更新确认未来障碍物轨迹

