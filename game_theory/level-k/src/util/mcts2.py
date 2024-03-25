"""
MCTS:

    game: s(s,a)
"""

import random 
from dataclasses import dataclass
import math 
import queue

import matplotlib
import matplotlib.pyplot as plt 

AVAILABLE_CHOICES = [0, 1, -1, 2, -2]  # action: 加速度
AVAILABLE_CHOICE_NUMBER = len(AVAILABLE_CHOICES)
MAX_ROUND_NUMBER = 200  # 视窗: 5s
PLAYER_NUMBER = 2
CONST_C = math.sqrt(2)
CONST_DELTA_T = 0.1  # 可能某些地方用的还是 0.1 

Refficiency = 0.5
Rcomfort = -0.5
Rsafe = -100.0
Rgoal = 10.0


"""
假设自车运动路径固定，障碍物运动轨迹固定，求解自车的最优运动轨迹。
实体状态定义:
    id: 车辆id
    t: 时间
    start_position: 起始位置    
    s_end: 终止位置
    v: 速度
    a: 加速度


"""
@dataclass
class Posiiton:
    x = 0.0
    y = 0.0

@dataclass
class Vehicle:
    id = 0
    t = 0.0
    v = 0.0
    a = 0.0
    heading = 0.0
    x = 0.0
    y = 0.0
    radius = 3


    def __eq__(self, other):
        return self.t == other.t and self.v == other.v and self.a == other.a and self.id == self.id and self.position == other.position

    def __repr__(self) -> str:
        return f"id:{self.id} \t t:{self.t} \t v:{self.v} \t a:{self.a} \t heading:{self.heading} \t x:{self.x} \t y:{self.y} \t radius:{self.radius}"
""" 
存玩家的当前状态，

状态定义 --> 




"""
class Player():
    def __init__(self,init_info:Vehicle) -> None:
        # 一个节点的基本信息，节点定义为游戏的某个步骤
        # current state 
        self.info = init_info
    
    # def update(self,action):
    #     TrajectoryUtil().update_state(self.info,action)
        
    def __repr__(self) -> str:
        return f"Player:{self.info}"

# 即通过一系列动作后，当前的游戏状态。
class State():
    def __init__(self,players:list[Player]) -> None:
        # 状态基本信息，当前的玩家，对应的节点。
        self.player = players
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

"""
往上 --> 环境定义 
往下 --> MCTS求解 
树上不留其它节点操作后的信息，只存储自车节点操作后的信息。
"""
# 树上每个节点的所有信息 
class TreeNode():
    def __init__(self,state:State,parent,player_index) -> None:
        # 原始信息
        self.state = state
        self.parent = parent
        # self.index = index
        self.player_index = player_index
        # self.current_player = state.player[player_index] # 表示轮到谁的回合。即下一个动作由谁产生。

        # 生成信息
        self.children:list[TreeNode] = []
        self.not_expanded_action_table = [i for i in range(AVAILABLE_CHOICE_NUMBER)]
        self.visits = 0
        # self.winner_count = 0
        self.reward = 0
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
        new_state = self.state.next_state(action_index)

        new_node = TreeNode(new_state,self, (self.player_index + 1) % PLAYER_NUMBER)
        self.children.append(new_node)

    def best_child(self):
        # 选择一个最好的子节点，即选择一个奖励最大的子节点。
        max_score = -1
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

    # 
    def rollout(self): # 模拟操作，不生成新节点。
        # 从当前节点开始rollout
        current_node = TreeNode(self.state,self.parent,self.player_index)  # 不改变原始节点。
        action = 0
        while not current_node.state.is_terminal():
            action = TreeUtil.action_choice() # 模拟，动作选择随机。
            current_node = TreeUtil().step(current_node,action) # 基于动作规划下一步,只规划自车的，其它玩家动作内部随机生成。? 一轮更新，回到自车节点。
        reward = RewardUtil.cal_tran_reward(current_node,action)
        # self.is_updated = True
        return reward

    def next_state(self,action):
        new_state = TrajectoryUtil.update_state(self,action)  # 自车先动作，玩家0， 其它车跟随，即自车时间为它车时间
        # 所有其它玩家也都要做出各自的举动,生成各自的新动作。
        for i in range(PLAYER_NUMBER):
            if i != self.player_index:
                # 获取其它玩家的动作
                action = TrajectoryUtil.other_action_get(i,new_state.t)
                new_state = TrajectoryUtil.update_state(self,action)
        
        return new_state
    
    def __repr__(self) -> str:
        return f"State:{self.state},Current Player:{self.player_index},Visits:{self.visits},Reward:{self.reward},Is Updated:{self.is_update}"
        



class TreeUtil():
    @staticmethod
    def choice(root:TreeNode): #选择节点的策略，满了，以一定比例选择最好的和随机的
        # 选择一个没有child的节点，且不为root节点。
        leaf_node = TreeUtil.find_leaf(root) # 一定是个叶子 
        if leaf_node.is_updated():
            if leaf_node.state.is_terminal():
                print("叶节点处于结束状态，")
                return None # 已经是结束状态。
            else:
                leaf_node.expand()
                return leaf_node.children[-1]
        else:
            # 没更新过，直接返回。
            return leaf_node

    @staticmethod
    def action_choice():
        return random.choice(AVAILABLE_CHOICES)
    
    @staticmethod
    def UCB(node:TreeNode):
        # 找到
        ans = 0
        return_ans = None
        for child in node.children:
            if child.visits == 0:
                return child
            _tmp = child.reward / child.visits + CONST_C * math.sqrt(math.log2(node.visits) / child.visits)
            if _tmp > ans:
                return_ans = child 
        return return_ans

    @staticmethod
    def find_highest_leaf_node(node):
        pass


    @staticmethod
    def find_leaf_updated(node:TreeNode):
        if node.is_leaf() and not node.is_updated():
            return node
        child_table:queue.Queue[TreeNode] = queue.Queue()
        child_table.put(node)
        
        while len(child_table)!=0:
            nd = child_table.pop()
            for child in nd.children:
                if child.is_leaf() and not child.is_updated():
                    return TreeUtil.UCB(child.parent)
                else:
                    child_table.put(child)
        return None
    
    # 广搜找叶子节点
    @staticmethod
    def find_leaf(node:TreeNode)->TreeNode:
        if node.is_leaf():
            # print("node info:",node)
            return node
        child_table:queue.Queue[TreeNode] = queue.Queue()
        child_table.put(node)
        
        while len(child_table)!=0:
            nd = child_table.pop()
            for child in nd.children:
                if child.is_leaf():
                    return TreeUtil.UCB(child.parent)
                else:
                    child_table.put(child)
    
    @staticmethod
    def step(node:TreeNode,action):  # 每一步都是从自车开始，顺序遍历一遍所有其它车辆动作 
        # 给一个节点，给一个动作，产生一个更新后的节点信息。
        new_state:State = node.next_state(action)  # 一轮循环后的自车新的状态。
        new_state.round_count += 1
        new_node = TreeNode(new_state,node,node.player_index)  # 完成一轮循环后，自车到达的新节点。
        new_node.state.is_collision = CollisionCheck.check(new_node)
        print(f"完成一轮循环， 新节点信息：{new_node.player_index}")
        return new_node

    @staticmethod
    def back_propagation(node:TreeNode,reward):
        # 输入的节点是 胜利节点，从这个节点开始，逐步向上更新。
        current_node = node
        # winner_player_index= current_node.parent.player_index
        while current_node != None:
            current_node.visits += 1
            # if current_node.player_index == winner_player_index:
            current_node.reward += reward
            current_node = current_node.parent
        
class RewardUtil:
    @staticmethod
    def cal_tran_reward(end_node:TreeNode,action):
        # 获取当前状态的奖励
        reward = 0.0 
        step_s = end_node.state.player[0].info.v * 0.1 + 0.5 * action * 0.01 if end_node.state.player[0].info.v + action * \
            0.1 >= 0.0 else - pow(end_node.state.player[0].info.v, 2) / (2.0 * action)
        is_collision = end_node.state.is_collision()

        reward += Refficiency * step_s 
        reward += Rcomfort * abs(action)
        reward += Rsafe * (1.0 if is_collision else 0.0)
        # reward += Rgoal *(1.0 if ego.)
        return reward
                           
class TrajectoryUtil:
    @staticmethod
    def update_state(node:TreeNode,action):
        # 基于简单的运动学模型，更新状态。
        vehicle:Vehicle = node.state.player[next(node.player_index)].info  # 
        new_v = vehicle.v + action * CONST_DELTA_T if vehicle.v >= -action * CONST_DELTA_T else 0.0
        new_s = (new_v ** 2 - vehicle.v ** 2) / (2 * action) if action != 0 else new_v * CONST_DELTA_T
        new_x = vehicle.x + new_s * math.cos(vehicle.heading)
        new_y = vehicle.y + new_s * math.sin(vehicle.heading)
        new_t = vehicle.t + CONST_DELTA_T
        vehicle.t = new_t
        vehicle.a = action
        vehicle.v = new_v
        vehicle.x = new_x
        vehicle.y = new_y

        # node.state.is_collision  = CollisionCheck.check(node.state.player)

    @staticmethod
    def other_action_get(player_index,t):
        # 获取其它玩家的动作
        action = random.choice(AVAILABLE_CHOICES)
        return action

class CollisionCheck:
    @staticmethod
    def check(players):
        # 检查是否有碰撞
        for i in range(len(players)):
            for j in range(i + 1, len(players)):
                # 碰撞检查 
                if CollisionCheck().is_collision(players[i].info,players[j].info):
                    return True
        return False
    
    @staticmethod
    def is_collision(player1_info:Vehicle,player2_info:Vehicle):
        distance = math.sqrt((player1_info.x - player2_info.x) ** 2 + (player1_info.y - player2_info.y) ** 2)
        return distance <= player1_info.radius + player2_info.radius
  

        

class MCTS():

    def __init__(self,root:TreeNode):
        self.root = root    

    def search(self):
        count = 0
        while count < 1000:
            # 1.选择一个没有child的节点，且不为root节点。
            # 逐层选择合适的节点，
            print(f"第 {count} 次 搜索........")
            leaf_node = TreeUtil.choice(self.root)  # 光搜找一个UCB节点，如果已经终止，则return 
            if leaf_node == None:
                return  # 如果已经终止，则return  可以用的叶节点已经是终止态了，表示已经遍历了所有可能，不需要继续遍历了。
            # 2.从这个节点开始rollout
            reward = leaf_node.rollout()
            TreeUtil.back_propagation(leaf_node,reward)
            count += 1

    def opt_tra(self):
        # 从根节点逐步寻找 胜率最高的节点
        ans = []
        current_node = self.root
        while not current_node.state.is_terminal():
            current_node = current_node.best_child()
            ans.append(current_node)
        return ans

def next(n):
    return (n+1) % AVAILABLE_CHOICE_NUMBER 


def main():
    # 初始化玩家信息

    v1 = Vehicle()
    v1.id = 0
    v1.t = 0.0
    v1.v = 0.0
    v1.a = 0.0
    v1.heading = 0.0
    v1.x = 0.0
    v1.y = 0.0
    v1.radius = 3

    v2 = Vehicle()
    v2.id1= 1
    v2.t1= 0.0
    v2.v1= 1.0
    v2.a1= 0.0
    v2.heading1= -math.pi/2
    v2.x1= 10.
    v2.y1= 0.0
    v2.radius1= 3


    v3 = Vehicle()
    v3.id2= 0
    v3.t2= 0.0
    v3.v2= 1.0
    v3.a2= 0.0
    v3.heading2= math.pi/2
    v3.x2= 10.
    v3.y2= 0.0
    v3.radius2= 3

    player1 = Player(v1)
    player2 = Player(v2)
    player3 = Player(v3)

    players = [player1,player2,player3]
    state = State(players)
    root = TreeNode(state,None,0)
    mcts = MCTS(root)
    mcts.search()
    ans = mcts.opt_tra()

    # 画图
    fig, ax = plt.subplots()
    ax.set_xlim(-10, 20)
    ax.set_ylim(-10, 20)
    # 3个车两个车随机，自车的最优轨迹结果。
    for i in range(len(ans)):
        ax.add_patch(plt.Circle((ans[i].state.player[0].x, ans[i].state.player[0].y), 3, color='r'))
        ax.add_patch(plt.Circle((ans[i].state.player[1].x, ans[i].state.player[1].y), 3, color='b'))
        ax.add_patch(plt.Circle((ans[i].state.player[2].x, ans[i].state.player[2].y), 3, color='g'))
    plt.show()



main()