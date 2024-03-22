"""
DQN:
    - Deep Q Network

基于价值的算法：不学习策略，而是学习评论员critic 

评论员：评价现在的动作有多好或有多不好。评论员无法凭空评价一个状态的好坏，它所评价的是在给定某一个状态的时候，如果接下来交互的演员的策略是 
π，我们会得到多少奖励

例如： 状态价值函数 $V_{\pi}$
 
- 使用深度神经网络替代原来的Q表
- 使用经验回放机制
- 使用目标网络 + 策略网络

"""

import gym 
import torch
import numpy as np
import torch.nn as nn 
import torch.nn.functional as F
from collections import deque
import random
import torch.optim as optim
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
import math

class MLP(nn.Module):
    def __init__(self, n_states, n_actions, hidden_dim = 128) -> None:
        super(MLP,self).__init__()
        self.fc1 = nn.Linear(n_states, hidden_dim)
        self.fc3 = nn.Linear(hidden_dim, n_actions)

    def forward(self, x):
        x = F.relu(self.fc1(x))
        return self.fc3(x)
    

# 经验回放机制
class ReplayBuffer:
    def __init__(self,capacity) -> None:
        self.capacity = capacity
        self.buffer = deque(maxlen = self.capacity)
    def push(self,transitions):
        self.buffer.append(transitions)

    # 随机选k个样本
    def sample(self, batch_size:int, sequential = False):
        if batch_size > len(self.buffer):
            batch_size = len(self.buffer) # 全采，随机 
        if sequential:
            rand = random.randint(0, len(self.buffer) - batch_size) # 随机选一个起始位置 
            batch =  [self.buffer[i] for i in range(rand,rand + batch_size)] # 顺序采样
            return zip(*batch) # 返回一个迭代器
        else:
            batch = random.sample(self.buffer, batch_size)
            return zip(*batch)  

    def clear(self):
        self.buffer.clear()

    def __len__(self):
        return len(self.buffer)
    

class DQN:
    def __init__(self, model,memeory,cfg):
        self.n_actions = cfg.n_actions
        self.device = cfg.device
        self.gamma = cfg.gamma # 折扣因子
        # \epislon-greedy 参数 
        self.sample_count = 0 # 衰减计算 
        self.epsilon_start = cfg.epsilon_start
        self.epsilon_end = cfg.epsilon_end
        self.epsilon_decay = cfg.epsilon_decay
        self.batch_size = cfg.batch_size
        self.policy_net = model.to(self.device)
        self.target_net = model.to(self.device)
        # 复制参数到目标网络
        for target_param, param in zip(self.target_net.parameters(), self.policy_net.parameters()):
            target_param.data.copy_(param.data)
        self.optimizer = optim.Adam(self.policy_net.parameters(), lr = cfg.lr)
        self.memory = memeory

    def sample_action(self,state):
        self.sample_count += 1
        # \epsilon-greedy
        self.epsilon = self.epsilon_end + (self.epsilon_start - self.epsilon_end) * math.exp(-1. * self.sample_count / self.epsilon_decay)

        if random.random() > self.epsilon:  # 利用 --> 选择最大奖励的动作
            with torch.no_grad(): # 做计算，单不保留梯度
                state = torch.tensor(state, device=self.device, dtype=torch.float32).unsqueeze(dim = 0)
                q_values = self.policy_net(state)
                action = q_values.max(1)[1].item()  
        
        else:
            action = random.randrange(self.n_actions) # 探索 --> 随机选择动作
        return action  

    @torch.no_grad()
    def predict_action(self,state):
        state = torch.tensor(state, device=self.device, dtype=torch.float32).unsqueeze(dim = 0)
        q_values = self.policy_net(state)
        action = q_values.max(1)[1].item()  
        return action
    
    def update(self):  # 用一批次数据来更新 Q
        if len(self.memory) < self.batch_size:
            return 
        # 从经验回放中采样一个batch
        state_batch, action_batch, reward_batch, next_state_batch, done_batch = self.memory.sample(self.batch_size)
        # 
        # print("state_batch",state_batch)
        state_batch = torch.tensor(np.array(state_batch), device=self.device, dtype=torch.float)
        action_batch = torch.tensor(action_batch, device=self.device, dtype=torch.long).unsqueeze(1)
        reward_batch = torch.tensor(reward_batch, device=self.device, dtype=torch.float)
        next_state_batch = torch.tensor(np.array(next_state_batch), device=self.device, dtype=torch.float)
        done_batch = torch.tensor(done_batch, device=self.device, dtype=torch.float)

        # 
        q_values = self.policy_net(state_batch).gather(dim = 1, index = action_batch).squeeze(1)  # squeeze 避免 1 * n 
        next_q_values = self.target_net(next_state_batch).max(1)[0].detach()  # detach 避免梯度传播 

        expected_q_values = reward_batch + self.gamma * next_q_values * (1 - done_batch)

        loss = nn.MSELoss()(q_values.unsqueeze(1), expected_q_values.unsqueeze(1))

        self.optimizer.zero_grad()
        loss.backward()
        for param in self.policy_net.parameters():
            param.grad.data.clamp_(-1, 1)  # 梯度裁剪
        self.optimizer.step()


def train(cfg,env,agent):
    print("Start Training")

    rewards = []
    steps = [] 
    for i_ep in range(cfg.train_eps):
        ep_reward = 0 
        ep_step = 0
        state,_ = env.reset() 
        for _ in range(cfg.ep_max_steps):
            ep_step += 1

            action = agent.sample_action(state)
            next_state, reward, done,trunc, _ = env.step(action)
            agent.memory.push((state, action, reward, next_state, done))
            state = next_state
            agent.update()  # 更新Q网络 
            ep_reward += reward
            if done:
                break 
        if (i_ep + 1) % cfg.target_update == 0: # target_net update 
            agent.target_net.load_state_dict(agent.policy_net.state_dict())
        steps.append(ep_step)
        rewards.append(ep_reward)
        if(i_ep + 1) % 10 == 0:
            print(f"Episode:{i_ep + 1}, Reward:{ep_reward}, Steps:{ep_step}, Epsilon:{agent.epsilon}")
    print("Training Done")
    env.close()
    return {"rewards":rewards}

def test(cfg,env,agent):
    print("Start Testing")
    rewards = []
    steps = []
    state,info = env.reset()
    
    for i_ep in range(cfg.test_eps):
        ep_reward = 0 
        ep_step = 0
        env.render()
        for _ in range(cfg.ep_max_steps):
            ep_step += 1
            action = agent.predict_action(state)
            next_state, reward, done,trunc, _ = env.step(action)
            ep_reward += reward
            state = next_state
            if done:
                break 
        steps.append(ep_step)
        rewards.append(ep_reward)
        if(i_ep + 1) % 10 == 0:
            print(f"Episode:{i_ep + 1}, Reward:{ep_reward}, Steps:{ep_step}")

    return {"rewards":rewards}



import os 

def all_seed(env,seed=1):
    env.seed(seed) # env config
    np.random.seed(seed)
    random.seed(seed)
    torch.manual_seed(seed) # config for CPU
    torch.cuda.manual_seed(seed) # config for GPU
    os.environ['PYTHONHASHSEED'] = str(seed) # config for python scripts
    # config for cudnn
    torch.backends.cudnn.deterministic = True
    torch.backends.cudnn.benchmark = False
    torch.backends.cudnn.enabled = False

def env_agent_config(cfg):
    env = gym.make(cfg.env_name)
    if cfg.seed != 0:
        all_seed(env,cfg.seed)
    n_states = env.observation_space.shape[0]
    n_actions = env.action_space.n
    cfg.n_sattes = n_states
    cfg.n_actions = n_actions
    print("state size:",n_states)
    print("action size:",n_actions)
    model = MLP(n_states, n_actions, cfg.hidden_dim)
    memory = ReplayBuffer(cfg.memory_capacity)
    agent = DQN(model, memory, cfg)
    return env,agent


class Args:
    def __init__(self) -> None:
        self.env_name = "CartPole-v1"
        self.seed = 0
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.hidden_dim = 256
        self.memory_capacity = 100000
        self.train_eps = 200
        self.test_eps = 10
        self.ep_max_steps = 100000
        self.target_update = 4
        self.gamma = 0.95
        self.epsilon = 0.1
        self.epsilon_start = 0.95
        self.epsilon_end = 0.01
        self.epsilon_decay = 500
        self.batch_size = 64
        self.lr = 0.0001
        self.algo_name = "DQN"

def smooth(data,weight = 0.9):
    last = data[0]
    smoothed = []
    for point in data:
        smoothed_val = last * weight + (1 - weight) * point
        smoothed.append(smoothed_val)
        last = smoothed_val
    return smoothed

def plot_rewards(rewards,cfg, tag='train'):
    ''' 画图
    '''
    sns.set_theme()
    plt.figure()  # 创建一个图形实例，方便同时多画几个图
    plt.title(f"{tag}ing curve on {cfg.device} of {cfg.algo_name} for {cfg.env_name}")
    plt.xlabel('epsiodes')
    plt.plot(rewards, label='rewards')
    plt.plot(smooth(rewards), label='smoothed')
    plt.legend()
    plt.show()



##
cfg = Args()
env, agent = env_agent_config(cfg)
res_dic = train(cfg,env,agent)
plot_rewards(res_dic['rewards'],cfg,tag = "train")

env = gym.make(cfg.env_name, render_mode="human")
res_dic = test(cfg,env,agent)
plot_rewards(res_dic['rewards'],cfg,tag = "test")  