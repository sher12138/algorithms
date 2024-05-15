

"""
states = ('Healthy', 'Fever')   初始状态 
observations = ('normal', 'cold', 'dizzy')   观测序列
start_probability = {'Healthy': 0.6, 'Fever': 0.4}  初始状态概率
transition_probability = {
    'Healthy' : {'Healthy': 0.7, 'Fever': 0.3},
    'Fever' : {'Healthy': 0.4, 'Fever': 0.6}
}  状态转移概率 
emission_probability = {
    'Healthy' : {'normal': 0.5, 'cold': 0.4, 'dizzy': 0.1},
    'Fever' : {'normal': 0.1, 'cold': 0.3, 'dizzy': 0.6}
} 状态症状概率 


p_{1,k} = π_k * p(y_1|k)  #  终端状态为k，前 1个观察值对应的最可能状态序列的概率 
p_{t,k} = max(p_{t-1,i} * a_{i,k} * p(y_t|k))  # t时刻状态为k的概率


p_{t,k} 终端状态为k，前 t个观察值对应的最可能状态序列的概率 
p(y_t|k)  影藏状态为k，观察值为y_t的概率


dp[i][y] = dp[i-1][y_{i-1}] * a(y_{i-1},y) * emi(y,o(i))  for y_{i-1} for y{i}   #  贝叶斯公式
"""
def viterbi(states,observations,start_probability,transition_probability,emission_probability):
    V = [{}]
    path = {}

    # Initialize base cases (t == 0) 观测是固定的， 所以， 最大化概率， V[0][y]  在第0天，观测到 O(0) ，那么状态是啥样的？ 
    for y in states: # 假设状态是啥样的，分别概率是多少。  第i天，假设状态是y,观察是 O(i) ，最大概率是多少。 
        V[0][y] = start_probability[y] * emission_probability[y][observations[0]]
        path[y] = [y]

    # Run Viterbi for t > 0
    for t in range(1,len(observations)):
        V.append({})
        newpath = {}

        for y in states:
            (prob,state) = max([(V[t-1][y0] * transition_probability[y0][y] * emission_probability[y][observations[t]],y0) for y0 in states])
            V[t][y] = prob
            newpath[y] = path[state] + [y]

        # Don't need to remember the old paths
        path = newpath

    n = 0
    if len(observations) != 1:
        n = t

    (prob,state) = max([(V[n][y],y) for y in states])

    return (prob,path[state])