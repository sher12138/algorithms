# RL achieve 

## 1. base architecture

sample ***

predict 

update ***

save & load 


```python

# train 

for i_ep in range(train_eps):
    state = env.reset()  

    # 
    while True:
        action = agent.sample(state)
        next_state, reward, done, _ = env.step(action) # 有模型，可以直接得到下一个状态；没有模型，需要采样
        agent.predict(state, action, reward, next_state, done)
        state = next_state
        if done:
            break