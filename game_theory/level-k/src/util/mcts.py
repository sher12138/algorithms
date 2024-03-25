import sys
import math
import random
import numpy as np
from dataclasses import dataclass
import matplotlib.pyplot as plt
import time

# AVAILABLE_CHOICES = [1, -1, 2, -2]
# AVAILABLE_CHOICE_NUMBER = len(AVAILABLE_CHOICES)
# MAX_ROUND_NUMBER = 10

AVAILABLE_CHOICES = [0, 1, -1, 2, -2]  # action: 加速度
AVAILABLE_CHOICE_NUMBER = len(AVAILABLE_CHOICES)
MAX_ROUND_NUMBER = 200  # 视窗: 5s

# reward
Refficiency = 0.5
Rcomfort = -0.5
Rsafe = -100.0
Rgoal = 10.0


@dataclass
class Vehicle:
    id = 0
    t = 0.0
    ss = 0.0
    se = 0.0
    v = 0.0
    a = 0.0
    sso = 0.0
    sse = 0.0

    def __eq__(self, other):
        return self.t == other.t and self.ss == other.ss and self.se == other.se and self.sso == other.sso and self.sse == other.sse and self.v == other.v and self.a == other.a


def StateTrans(state, action, state_e):
    next_state = Vehicle()
    next_state.v = state.v + action * 0.1 if state.v >= -action * 0.1 else 0.0
    step_s = state.v * 0.1 + 0.5 * action * 0.01 if state.v >= - \
        action * 0.1 else pow(state.v, 2) / (2 * action)

    next_state.ss = state.ss - step_s
    next_state.se = state.se - step_s
    if state.id != 0:
        next_state.sso = state_e.ss
        next_state.seo = state_e.se
    return next_state


class Node(object):
    def __init__(self):
        self.parent = None
        self.children = []
        self.visit_times = 0
        self.quality_value = 0.0
        self.state = None
        self.is_terminal = False

    def set_state(self, state):
        self.state = state
        # self.quality_value = self.state.get_reward()

    def get_state(self):
        return self.state

    def set_parent(self, parent):
        self.parent = parent

    def get_parent(self):
        return self.parent

    def set_children(self, children):
        self.children = children

    def get_children(self):
        return self.children

    def get_visit_times(self):
        return self.visit_times

    def set_visit_times(self, times):
        self.visit_times = times

    def visit_times_add_one(self):
        self.visit_times += 1

    def get_quality_value(self):
        return self.quality_value

    def set_quality_value(self, value):
        self.quality_value = value

    def quality_value_add_n(self, n):
        self.quality_value += n

    def is_all_expand(self):
        if len(self.children) == AVAILABLE_CHOICE_NUMBER:
            return True
        else:
            return False

    def add_child(self, sub_node):
        sub_node.set_parent(self)
        self.children.append(sub_node)

    def check_terminal(self):
        self.is_terminal = self.state.is_terminal()
        return self.is_terminal

    # def __repr__(self):
    #     return "Node:{}, t:{}, visit time:{}, ego ss:{}, ego se:{}, ego v:{}, ego a:{}, agent ss:{}, agent se:{}, agent v:{}, agent a:{}".format(hash(self), self.visit_times, self.state.get_current_ego_state().ss, self.state.get_current_ego_state().se, self.state.get_current_ego_state().v, self.state.get_current_ego_state().a, self.state.get_current_agent_state().ss, self.get_current_agent_state().se, self.state.get_current_agent_state().v, self.state.get_current_agent_state().a)


def grab_conflict_zone(all_agents: list):
    all_agents_handle = list()
    ego_minimum_s = -1.0
    ego_maximum_s = -1.0
    ego = Vehicle()
    for agent in all_agents:
        if agent.id == 0:
            if agent.ss > ego_minimum_s:
                ego.ss = agent.ss
                ego_minimum_s = agent.ss
            if agent.se > ego_maximum_s:
                ego.se = agent.se
                ego_maximum_s = agent.se
        else:
            if agent.sse > 0.0:
                all_agents_handle.append(agent)
    ego.id = 0
    all_agents_handle.append(ego)
    return all_agents_handle


class State(object):
    def __init__(self):
        # self.current_value = 0.0
        self.value = -1000
        self.t = 0.0
        self.current_round_index = 0  # 第几轮
        self.cumulative_choices = []  # 选择过程记录
        self.ego = Vehicle()
        self.agents = list()
        self.agent = Vehicle()
        self.action = 0.0
        self.is_collision = False

    def __eq__(self, state):
        self.agents = self.agents_sorting(self.agents)
        state_agents = self.agents_sorting(state.agents)
        set_agents = set(self.agents)
        set_state_agents = set(state_agents)
        return state.t == self.t and state.ego == self.ego and (set_agents == set_state_agents)

    def agents_sorting(self, agents: list):
        agents = sorted(agents, key=lambda x: x.ss)
        return sorted(agents, key=lambda x: x.sso)

    def is_terminal(self):  # 判断交互是否结束
        if self.current_round_index == MAX_ROUND_NUMBER-1:
            return True
        elif self.ego.se <= 0.0:
            return True
        # elif self.agent.se <= 0.0:
        #     return True
        elif self.is_collision:
            return True
        return False

    def compute_collision(self):  # 检查碰撞
        self.is_collision = False
        for agent in self.agents:
            if agent.id == 0:
                continue
            if agent.ss <= 0.0 and agent.se >= 0.0 and agent.sso <= 0.0 and agent.seo >= 0.0:
                self.is_collision = True
                break
        return self.is_collision

    def step(self):
        next_ego = StateTrans(self.ego, self.action, self.ego)
        next_agents = list()
        for agent in self.agents:
            min_value = sys.maxsize
            max_value_a = -sys.maxsize
            next_agent = Vehicle()
            for a in AVAILABLE_CHOICES:
                agent_tmp = StateTrans(agent, a, next_ego)
                value = self.compute_reward(
                    self.ego, next_ego, self.action, self.agents)
                if min_value > value:
                    min_value = value
                    next_agent = agent
                elif min_value == value:
                    value_a = self.compute_reward(
                        agent, agent_tmp, a, self.agents)
                    if max_value_a < value_a:
                        max_value_a = value_a
                        next_agent = agent
        return next_ego, next_agent, min_value

    def compute_reward(self, ego, ego_next, action, agents):  # cost
        reward = 0.0
        step_s = self.ego.v * 0.1 + 0.5 * self.action * 0.01 if self.ego.v + self.action * \
            0.1 >= 0.0 else - pow(self.ego.v, 2) / (2.0 * self.action)
        is_collision = False
        for agent in agents:
            if ego_next.id == 0:
                is_collision = self.compute_collision(ego_next, agents)
                if is_collision:
                    break
                continue
            if agent.id == 0:
                is_collision = self.compute_collision(ego_next, )

        is_collision = self.compute_collision(ego_next, agents)
        reward += Refficiency * step_s
        reward += Rcomfort * abs(self.action)
        reward += Rsafe * (1.0 if is_collision else 0.0)
        reward += Rgoal * (1.0 if ego.se <= 0.5 else 0.0)
        return reward

    def set_current_ego_state(self, state):
        self.ego = state

    def set_current_agent_state(self, state):
        self.agent = state

    def get_current_ego_state(self):
        return self.ego

    def get_current_agent_state(self):
        return self.agent

    def set_current_round_index(self, round):
        self.current_round_index = round

    def set_cumulative_choices(self, choices):
        self.cumulative_choices = choices

    def get_next_state_with_random_choice(self):  # 下个状态：random
        random_choice = random.choice([choice for choice in AVAILABLE_CHOICES])
        self.action = random_choice
        next_state = State()
        next_state.ego, next_state.agent, next_state.value = self.step()
        next_state.t += 0.1
        # next_state.set_current_value(self.current_value+random_choice)
        next_state.set_current_round_index(self.current_round_index+1)
        next_state.set_cumulative_choices(
            self.cumulative_choices+[random_choice])
        return next_state

    def get_reward(self):
        return self.value

    def get_action(self):
        return self.action


def monte_carlo_tree_search(node):  # 蒙特卡洛树搜索
    computation_budget = 1000
    for i in range(computation_budget):
        expend_node = tree_policy(node)  # 找一个子节点，
        reward = default_policy(expend_node)
        backup(expend_node, reward)
    best_next_node = best_child(node, True)
    return best_next_node


def best_child(node, is_exploration):  # 若子节点都扩展完了，求UCB值最大的子节点
    best_score = -sys.maxsize
    best_sub_node = None
    for sub_node in node.get_children():
        if is_exploration:
            C = 1/math.sqrt(2.0)
        else:
            C = 0.0
        left = sub_node.get_quality_value()/sub_node.get_visit_times()
        right = 2.0*math.log(node.get_visit_times())/sub_node.get_visit_times()
        score = left+C*math.sqrt(right)
        if score > best_score:
            best_score = score
            best_sub_node = sub_node
    return best_sub_node


def expand(node):  # 得到未扩展的子节点
    tried_sub_node_states = [sub_node.get_state()
                             for sub_node in node.get_children()]
    new_state = node.get_state().get_next_state_with_random_choice()
    while new_state in tried_sub_node_states:
        new_state = node.get_state().get_next_state_with_random_choice()
    sub_node = Node()
    sub_node.set_state(new_state)
    node.add_child(sub_node)
    return sub_node


def tree_policy(node):  # 选择子节点的策略
    while node.get_state().is_terminal() == False:
        if node.is_all_expand():
            node = best_child(node, True)
        else:
            sub_node = expand(node)
            return sub_node
    return node


def default_policy(node):
    current_state = node.get_state()
    while current_state.is_terminal() == False:
        current_state = current_state.get_next_state_with_random_choice()
    # final_state_reward = current_state.compute_reward()
    final_state_reward = current_state.get_reward()
    return final_state_reward


def backup(node:Node, reward):
    while node != None:
        node.visit_times_add_one()
        node.quality_value_add_n(reward)
        node = node.parent

# plot


def add_set_for_state(state, ego_ss, ego_se, ego_v, agent_ss, agent_se, agent_v):
    ego_ss.append(state.get_current_ego_state().ss)
    ego_se.append(state.get_current_ego_state().se)
    ego_v.append(state.get_current_ego_state().v)
    agent_ss.append(state.get_current_agent_state().ss)
    agent_se.append(state.get_current_agent_state().se)
    agent_v.append(state.get_current_agent_state().v)


def add_set_for_node(node, visit_times, values):
    visit_times.append(node.visit_times)
    values.append(node.quality_value / node.visit_times)


if __name__ == '__main__':
    node = Node()
    state = State()
    # plot
    ego_ss_set = list()
    ego_se_set = list()
    ego_v_set = list()
    agent_ss_set = list()
    agent_se_set = list()
    agent_v_set = list()
    visit_times_set = list()
    values_set = list()
    # init
    ego_state = Vehicle()
    ego_state.t = 0
    ego_state.ss = 10
    ego_state.se = 15
    ego_state.v = 1
    ego_state.a = 0
    agent_state = Vehicle()
    agent_state.t = 0
    agent_state.ss = 10
    agent_state.se = 15
    agent_state.v = 2
    agent_state.a = 0
    state.set_current_ego_state(ego_state)
    state.set_current_agent_state(agent_state)
    node.set_state(state)
    node.set_visit_times(1)
    add_set_for_state(state, ego_ss_set, ego_se_set, ego_v_set,
                      agent_ss_set, agent_se_set, agent_v_set)
    add_set_for_node(node, visit_times_set, values_set)
    # # once
    # s_t = time.time()
    # node = monte_carlo_tree_search(node)
    # e_t = time.time()
    # print("time: ", e_t - s_t)
    # add_set_for_state(node.state, ego_ss_set, ego_se_set, ego_v_set,
    #                   agent_ss_set, agent_se_set, agent_v_set)
    # add_set_for_node(node, visit_times_set, values_set)
    # print("visit time:{}, ego ss:{}, ego se:{}, ego v:{}, ego a:{}, agent ss:{}, agent se:{}, agent v:{}, agent a:{}".format(node.visit_times, node.state.get_current_ego_state().ss, node.state.get_current_ego_state(
    # ).se, node.state.get_current_ego_state().v, node.state.get_current_ego_state().a, node.state.get_current_agent_state().ss, node.state.get_current_agent_state().se, node.state.get_current_agent_state().v, node.state.get_current_agent_state().a))

    while not node.check_terminal():
        s_t = time.time()
        node.set_quality_value(0.0)
        node = monte_carlo_tree_search(node)
        e_t = time.time()
        print("time: ", e_t - s_t)
        add_set_for_state(node.state, ego_ss_set, ego_se_set, ego_v_set,
                          agent_ss_set, agent_se_set, agent_v_set)
        add_set_for_node(node, visit_times_set, values_set)
        print("visit time:{}, node value:{}, ego ss:{}, ego se:{}, ego v:{}, ego a:{}, agent ss:{}, agent se:{}, agent v:{}, agent a:{}".format(node.visit_times, node.quality_value / node.visit_times, node.state.get_current_ego_state().ss, node.state.get_current_ego_state(
        ).se, node.state.get_current_ego_state().v, node.state.get_current_ego_state().a, node.state.get_current_agent_state().ss, node.state.get_current_agent_state().se, node.state.get_current_agent_state().v, node.state.get_current_agent_state().a))
    t_set = [t * 0.1 for t in range(len(ego_se_set))]
    plt.figure(1)
    plt.plot(t_set, ego_ss_set, label='ego ss')
    plt.plot(t_set, ego_se_set, label='ego se')
    plt.plot(t_set, ego_v_set, label='ego v')
    plt.plot(t_set, agent_ss_set, label='agent ss')
    plt.plot(t_set, agent_se_set, label='agent se')
    plt.plot(t_set, agent_v_set, label='agent v')
    plt.xlabel('t (s)')
    plt.ylabel('s (m)')
    plt.legend()
    plt.show()
    plt.figure(2)
    plt.plot(t_set, ego_v_set, label='ego v')
    plt.plot(t_set, agent_v_set, label='agent v')
    plt.xlabel('t (s)')
    plt.ylabel('v (m/s)')
    plt.legend()
    plt.show()
    plt.figure(3)
    plt.plot(t_set, visit_times_set, label='visit times')
    plt.xlabel('t (s)')
    plt.ylabel('number of times (freq)')
    plt.legend()
    plt.show()
    plt.figure(4)
    plt.plot(t_set[:-1], values_set[:-1], label='values')
    plt.xlabel('t (s)')
    plt.ylabel('reward')
    plt.legend()
    plt.show()
