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
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import pprint
import copy

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
        self.is_root = False
        # self.current_player = state.player[player_index] # 表示轮到谁的回合。即下一个动作由谁产生。

        # 生成信息
        self.children:list[TreeNode] = []
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
        new_node = TreeNode(new_state,self,next(self.player_index))
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
    

class TreeUtil():
    @staticmethod
    def choice(root:TreeNode): #选择节点的策略，满了，以一定比例选择最好的和随机的
        # 选择一个没有child的节点，且不为root节点。
        return TreeUtil.find_leaf(root) # 一定是个叶子 
        # 先找到一个叶子节点，如果该节点不可用，则判断其父节点是否还可以新建节点，如果可以，则新建，然后选择；如果不可以。
        

    @staticmethod
    def action_choice():
        return random.choice(AVAILABLE_CHOICES)
    
    # 每层选点的策略。
    @staticmethod
    def UCB(node:TreeNode):
        # 找到
        ans = -1e9
        return_ans = None
        for child in node.children:
            if child.visits == 0:
                return child
            _tmp = child.reward / child.visits + CONST_C * math.sqrt(math.log2(node.visits) / child.visits)
            if _tmp >= ans:
                return_ans = child 
                ans = _tmp
        # print("UCB:",return_ans.reward)
        return return_ans

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
    
    # 给根节点，广搜找叶子节点，UCB找一排叶子节点中最大的。不是叶节点，则选择一个UCB最大的。
    @staticmethod
    def find_leaf(root:TreeNode)->TreeNode:
        child_table:queue.Queue[TreeNode] = queue.Queue()

        node = root 
        child_table.put(node)
        print(f"root state: updated :{node.is_fully_expanded()}, fully expand:{node.is_updated()}")
        # x_node = copy.deepcopy(node)
        while node.is_fully_expanded() and node.is_updated():  # 基于UCB规则逐步向下，直到找到一个非完全扩展的节点
            # print("ucb down --> ")
            min_ucb_x_node = TreeUtil.UCB(node)
            if min_ucb_x_node == None:
                node.expand()  # 已经找到叶子节点上了，扩展一个节点.
                node = node.children[-1]
                break
            node = min_ucb_x_node

        print(f"root state: updated :{node.is_fully_expanded()}, fully expand:{node.is_updated()}")
        if node.is_fully_expanded():
            # 没更新，直接返回 
            print(f"找到未更新叶子节点: {node.player_index}")
            return node 
        if node.is_updated():
            print("need to expand children ")
            node.expand() # 返回扩展节点
            print(f"扩展新叶子节点: {node.player_index}")
            return node.children[-1]  # 一定是一个叶子节点。
        return node 
    
    
    @staticmethod
    def step(node:TreeNode,action):  # 每一步都是从自车开始，顺序遍历一遍所有其它车辆动作 
        # 给一个节点，给一个动作，产生一个更新后的节点信息。
        node.next_state(action)  # 一轮循环后的自车新的状态。
        node.state.round_count += 1
        # new_node = TreeNode(node.state,node,node.player_index)  # 完成一轮循环后，自车到达的新节点。
        node.state.is_collision = CollisionCheck.check(node)
        # print(f"完成一轮循环， 新节点信息：{node},\t 节点状态:{node.state}")

    @staticmethod
    def back_propagation(node:TreeNode,reward):
        # 输入的节点是 胜利节点，从这个节点开始，逐步向上更新。
        node.is_update = True
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
        is_collision = end_node.state.is_collisioned()

        reward += Refficiency * step_s 
        reward += Rcomfort * abs(action)
        reward += Rsafe * (1.0 if is_collision else 0.0)
        # reward += Rgoal *(1.0 if ego.)
        return reward
                           
class TrajectoryUtil:

    @staticmethod
    def update_state_base_state(state:State,action,player_index):
        vehicle:Vehicle = state.player[player_index].info  # 更新的是上一个玩家的状态
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


    @staticmethod
    def update_state(node:TreeNode,action):
        # 基于简单的运动学模型，更新状态。
        # print(f"节点原始状态:{node.state}")
        vehicle:Vehicle = node.state.player[node.player_index].info  # 
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

        # print(f"节点更新后状态:{node.state}")

        # node.state.is_collision  = CollisionCheck.check(node.state.player)

    @staticmethod
    def other_action_get():
        # 获取其它玩家的动作
        action = random.choice(AVAILABLE_CHOICES)
        return action

class CollisionCheck:
    @staticmethod
    def check(node:TreeNode):
        # 检查是否有碰撞
        players = node.state.player
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
  

def draw_tra(ans):
    fig, ax = plt.subplots()
    ax.set_xlim(-20, 20)
    ax.set_ylim(-20, 20)
    # 3个车两个车随机，自车的最优轨迹结果。
    for i in range(len(ans)):
        ax.add_patch(plt.Circle((ans[i].state.player[0].info.x, ans[i].state.player[0].info.y), 3, color='r'))
        ax.add_patch(plt.Circle((ans[i].state.player[1].info.x, ans[i].state.player[1].info.y), 3, color='b'))
        ax.add_patch(plt.Circle((ans[i].state.player[2].info.x, ans[i].state.player[2].info.y), 3, color='g'))
        # print(f"Node Player info :{ans[i].state.player[0]},{ans[i].state.player[1]},{ans[i].state.player[2]}")
    plt.show()
 
def print_child(node):
    print("-------",[ch.player_index for ch in node.children])

def print_tree(child:TreeNode, level=0):
    if level > 100:return 
    if child is None:
        return
    # 打印当前节点
    print(' ' * level + '->', child.player_index , "(" , child.state.player[next(child.player_index - 2)].info.x , "," , child.state.player[next(child.player_index-2)].info.y,"reward:", child.reward ,")")
    for ch in child.children:
        print_tree(ch, level + 2)

class MCTS():

    def __init__(self,root:TreeNode):
        self.root = root    

    def search(self):
        count = 0
        while count < ITER_NUMBER:
            # 1.选择一个没有child的节点，且不为root节点。
            # 逐层选择合适的节点，
            print(f"第 {count} 次 搜索........")
            leaf_node = TreeUtil.choice(self.root)  # 广搜找一个UCB节点，如果已经终止，则return 
            print(f"选择节点:{leaf_node.player_index}")
            if leaf_node == None:
                print("已经终止")
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
            if current_node == None:
                break
            ans.append(current_node)
        return ans

def next(n):
    return (n+1) %  PLAYER_NUMBER


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
    v2.id= 1
    v2.t= 0.0
    v2.v= 1.0
    v2.a= 0.0
    v2.heading= -math.pi/2
    v2.x= 20.
    v2.y= 20.0
    v2.radius= 3


    v3 = Vehicle()
    v3.id= 2
    v3.t =  0.0
    v3.v= 1.0
    v3.a= 0.0
    v3.heading= math.pi/2
    v3.x= 20.
    v3.y= -20.0
    v3.radius= 3

    player1 = Player(v1)
    player2 = Player(v2)
    # player3 = Player(v3)

    players = [player1,player2]
    # print(player1,player2,player3)
    state = State(players)
    root = TreeNode(state,None,0)
    root.is_root = True
    mcts = MCTS(root)
    mcts.search()
    print_tree(root)
    ans:TreeNode = mcts.opt_tra()
    print(len(ans))
    for i in ans:
        for player in i.state.player:
            print(f"player info :{player.info.x},{player.info.y}")

    # 画图
    fig = plt.figure()
    ax = fig.add_subplot(111,projection='3d')
    ax.set_xlim(-20, 20)
    ax.set_ylim(-20, 20)
    ax.set_zlim(-20, 20)
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('t Label')
    ax.set_aspect('equal')
    # 3个车两个车随机，自车的最优轨迹结果。
    for i in range(len(ans)):
        ax.scatter(ans[i].state.player[0].info.x, ans[i].state.player[0].info.y,ans[i].state.player[0].info.t, color='r',s=100)
        ax.scatter(ans[i].state.player[1].info.x, ans[i].state.player[1].info.y,ans[i].state.player[1].info.t, color='b',s=100)
        # ax.scatter(ans[i].state.player[2].info.x, ans[i].state.player[2].info.y,ans[i].state.player[2].info.t, color='g')
        # print(f"Node Player info :{ans[i].state.player[0]},{ans[i].state.player[1]},{ans[i].state.player[2]}")
    plt.show()

main()