"""

graph = [
"...x..#",
"....x.#",
"...x..#",
"..#..x..",
"...x...",
]

图中 # 表示障碍物，x 表示可以中转点. 表示空地，求给定一个起点，到达所有中转点的最短运动步数
"""


def min_steps(graph, start):
    # 1. 找到所有中转点
    transfer_points = []
    for i in range(len(graph)):
        for j in range(len(graph[0])):
            if graph[i][j] == 'x':
                transfer_points.append((i, j))

    return dp