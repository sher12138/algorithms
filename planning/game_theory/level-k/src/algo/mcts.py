from agent import EgoAgent,OtherAgent
import random
import copy
import math


AVAILABLE_CHOICES = [0, 1, -2]  # action: 加速度
AVAILABLE_CHOICE_NUMBER = len(AVAILABLE_CHOICES)
MAX_ROUND_NUMBER = 50  # 视窗: 5s
PLAYER_NUMBER = 2
CONST_C = math.sqrt(2)
CONST_DELTA_T = 0.1

ITER_NUMBER = 1000

Refficiency = 0.5
Rcomfort = -0.5
Rsafe = -100.0
Rgoal = 10.0




class MCTSState():
    def __init__(self,ego_agent:EgoAgent,other_agent:OtherAgent) -> None:
        # 状态基本信息，当前的玩家，对应的节点。
        self.player = [ego_agent,other_agent]
        # 生成信息 
        self.reward = 0

        self.is_collision = False
        self.round_count = 0 

    def is_terminal(self):
        # 判断是否是终止状态
        if self.is_collision:
            return True
        if(self.round_count >= MAX_ROUND_NUMBER):
            return True
        return False
    
    def is_collisioned(self):
        return self.is_collision
        
    # def update_player_info(self,player_index,action):
    #     self.player[player_index].update(action,player_index)
    def get_reward(self):
        return self.reward

    def set_reward(self,reward):
        self.reward = reward

    def __repr__(self) -> str:
        return f"Player:{self.player}"


class MCTSTreeNode():
    def __init__(self,state:MCTSState,parent,player_index) -> None:
        # 原始信息
        self.state = state
        self.parent = parent
        # self.index = index
        self.player_index = player_index
        self.is_root = False
        # self.current_player = state.player[player_index] # 表示轮到谁的回合。即下一个动作由谁产生。

        # 生成信息
        self.children:list[MCTSTreeNode] = []
        self.not_expanded_action_table = [i for i in range(AVAILABLE_CHOICE_NUMBER)]
        self.visits = 0
        self.winner_count = 0
        self.reward = 0.0
        self.is_update = False
        # self.winner_index = -1 

    def is_fully_expanded(self):
        if len(self.children) == AVAILABLE_CHOICE_NUMBER:
            return True

    def expand(self):
        # 节点扩展策略,选择一个没有用过的动作进行扩展。
        if(self.is_fully_expanded()):
            return 
        # 玩家确认 
        action_index = random.choice(self.not_expanded_action_table)
        self.not_expanded_action_table.remove(action_index)
        # 基于该动作构建新节点 
        new_state = self.get_next_state(action_index)
        new_node = MCTSTreeNode(new_state,self,next(self.player_index))
        self.children.append(new_node)
        print(f"add expand node --> {new_node.player_index}")

    def best_child(self):
        # 选择一个最好的子节点，即选择一个奖励最大的子节点。
        max_score = -1e9
        best_child = None
        for child in self.children:
            score = child.reward / child.visits
            if score > max_score:
                max_score = score
                best_child = child
        return best_child

    def is_leaf(self):
        return len(self.children) == 0
    
    def is_updated(self):
        return self.is_update

    def rollout(self): # 模拟操作，不生成新节点。
        # 从当前节点开始rollout
        # draw_data = []
        current_node = copy.deepcopy(self)  # 非子节点。
        current_node.parent = None
        action = 0
        while not current_node.state.is_terminal():
            # draw_data.append(copy.deepcopy(current_node)) # 
            action = TreeUtil.action_choice() # 模拟，动作选择随机。
            TreeUtil().step(current_node,action) # 基于动作规划下一步,只规划自车的，其它玩家动作内部随机生成。? 一轮更新，回到自车节点。
            # print_child(self)
        reward = RewardUtil.cal_tran_reward(current_node,action)
        # print("rollout 长度：", len(draw_data))
        # draw_tra(draw_data)
        
        return reward

    # 自举下一个状态。
    def next_state(self,action):
        # TrajectoryUtil.update_state(self,action)  # 自车先动作，玩家0， 其它车跟随，即自车时间为它车时间
        # self.player_index = next(self.player_index)
        # 所有其它玩家也都要做出各自的举动,生成各自的新动作。
        for i in range(PLAYER_NUMBER):
            TrajectoryUtil.update_state(self,action)
            action = TrajectoryUtil.other_action_get()
            self.player_index  = next(self.player_index) # 
    
    # 扩展节点时，获取下一个节点状态。
    def get_next_state(self,action_index):
        new_state = copy.deepcopy(self.state)
        action = AVAILABLE_CHOICES[action_index]
        TrajectoryUtil.update_state_base_state(new_state,action,self.player_index)
        return new_state


    def __repr__(self) -> str:
        return f"Current Player:{self.player_index},Visits:{self.visits},Reward:{self.reward},Is Updated:{self.is_update}"
    
