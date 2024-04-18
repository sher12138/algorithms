"""
dfs 维护连通块 
"""
# 

graph = 
n = 10 
ids  = [-1] * n # 每个节点所处的连通块 
cc_and = []  # 每个连通块的最小路径权 

def dfs(x):
    # and_ = -1 
    ids[x] = len(cc_and)
    for u,w in graph[x]:
        # 可能访问过，可能没访问过？
        # and_ &= w 
        if ids[u] <0: # 没访问过？ 
            # and_ &= dfs(u)
            dfs(u)

for i in range(n):
    if ids[i] < 0:
        cc_and.append(dfs(i))


"""
并查集维护连通块 
"""

fa = list(range(n))
and_ = [-1] * n 

def find(x):
    if fa[x] != x:
        fa[x] = find(fa[x])
    return fa[x] 

for x,y,w in edges:
    x = find(x) 
    y = find(y)
    and_[y] &= w 
    if x != y:
        and_[y] &=and_[x] 
        fa[x] = y # union 

