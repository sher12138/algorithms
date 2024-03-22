"""
Q-learning 

环境：FrozenLake-v0

状态：16个

动作：4个

奖励：到达终点+1，掉入洞-1

策略：epsilon-greedy

Q-learning是一种基于值的强化学习算法，它是一种off-policy算法，它的目标是学习一个最优的Q-table，Q-table的每一行对应一个状态，每一列对应一个动作，Q-table的值表示在某个状态下采取某个动作的累积奖励。

"""


import numpy as np
import gym
import random
import time
from IPython.display import clear_output

# 创建环境
env = gym.make("FrozenLake-v1",desc=None,map_name="8x8",is_slippery=False)

# 初始化Q-table
action_size = env.action_space.n
state_size = env.observation_space.n
qtable = np.zeros((state_size, action_size))  # 状态空间*动作空间

# 初始化超参数
total_episodes = 15000        # 总训练次数
total_test_episodes = 10      # 总测试次数
max_steps = 99                # 每次训练的最大步数
learning_rate = 0.7           # 学习率
gamma = 0.618                 # 折扣因子

# Exploration parameters
epsilon = 1.0                 # 探索率
max_epsilon = 1.0             # 探索率的最大值
min_epsilon = 0.01            # 探索率的最小值
decay_rate = 0.001             # 探索率的衰减率

# 训练Q-table
rewards = []
for episode in range(total_episodes): # start of learning

    state,info= env.reset()
    step = 0
    done = False
    total_rewards = 0

    for step in range(max_steps):
        # 采取一个动作
        exp_exp_tradeoff = random.uniform(0, 1)
        if exp_exp_tradeoff > epsilon:
            action = np.argmax(qtable[state, :])
        else:
            action = env.action_space.sample()
        # 执行动作生成新状态
        new_state, reward, done,trunc,info = env.step(action)
        # Q(s,a):= Q(s,a) + lr [R(s,a) + gamma * max Q(s',a') - Q(s,a)]
        qtable[state, action] = qtable[state, action] + learning_rate * (reward + gamma * np.max(qtable[new_state, :]) - qtable[state, action])
        # opt
        if state == new_state:
            qtable[state, action] = -1
        
        # 累积奖励
        total_rewards += reward
        if done == True:
            if reward == 0:
                qtable[state, action] = -1
            break
        state = new_state
    # 减小探索率
    # epsilon = min_epsilon + (max_epsilon - min_epsilon) * np.exp(-decay_rate * episode)
    epsilon =min_epsilon +  (max_epsilon - min_epsilon) * (1 - np.sum(qtable[qtable > 0].__len__()) / state_size / action_size)
    rewards.append(total_rewards)

print("Score over time: " + str(sum(rewards) / total_episodes))
print(qtable)

# 测试Q-table
env = gym.make("FrozenLake-v1",desc=None,map_name="8x8",is_slippery=False,render_mode="human")

rewards = []
for episode in range(total_test_episodes):
    state,info = env.reset()
    step = 0
    done = False
    total_rewards = 0
    print("****************************************************")
    print("EPISODE ", episode)

    for step in range(max_steps):
        env.render()
        # 采取最优动作
        action = np.argmax(qtable[state, :])
        new_state, reward, done,trunc, info = env.step(action)
        total_rewards += reward
        if done:
            rewards.append(total_rewards)
            print ("Score", total_rewards)
            break
        state = new_state
env.close()
print("Score over time: " + str(sum(rewards) / total_test_episodes))
print("Q-table")