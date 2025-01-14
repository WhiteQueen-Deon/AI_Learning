import random
import math
import time
import tracemalloc
import CityGraph

# ACO 参数
ALPHA = 1       # 信息素重要性
BETA = 2        # 启发式信息重要性
RHO = 0.1       # 信息素挥发系数
Q = 100         # 信息素强度
NUM_ANTS = 10   # 蚂蚁数量
MAX_ITER = 100  # 最大迭代次数

# 初始化信息素矩阵
def initialize_pheromone(graph):
    n = len(graph)
    return [[1 for _ in range(n)] for _ in range(n)]

# 选择下一个城市
def select_next_city(current, visited, pheromone, graph):
    n = len(graph)
    probabilities = []
    denominator = 0
    for neighbor, distance in graph[current]:
        if neighbor not in visited:
            tau = pheromone[current][neighbor] ** ALPHA
            eta = (1 / distance) ** BETA
            probabilities.append((neighbor, tau * eta))
            denominator += tau * eta

    probabilities = [(city, prob / denominator) for city, prob in probabilities]
    next_city = random.choices([city for city, _ in probabilities],
                                weights=[prob for _, prob in probabilities])[0]
    return next_city

# 更新信息素
def update_pheromone(pheromone, ants_paths, graph):
    n = len(graph)
    for i in range(n):
        for j in range(n):
            pheromone[i][j] *= (1 - RHO)  # 信息素挥发

    for path, cost in ants_paths:
        for i in range(len(path) - 1):
            city1, city2 = path[i], path[i + 1]
            pheromone[city1][city2] += Q / cost
            pheromone[city2][city1] += Q / cost

# ACO 主函数
def aco_tsp(graph, start):
    tracemalloc.start()
    start_time = time.time()

    n = len(graph)
    pheromone = initialize_pheromone(graph)
    best_cost = float('inf')
    best_path = []

    for _ in range(MAX_ITER):
        ants_paths = []
        for _ in range(NUM_ANTS):
            visited = {start}
            path = [start]
            current = start
            total_cost = 0

            while len(visited) < n:
                next_city = select_next_city(current, visited, pheromone, graph)
                for neighbor, distance in graph[current]:
                    if neighbor == next_city:
                        total_cost += distance
                        break
                path.append(next_city)
                visited.add(next_city)
                current = next_city

            # 回到起点
            for neighbor, distance in graph[current]:
                if neighbor == start:
                    total_cost += distance
                    break
            path.append(start)

            ants_paths.append((path, total_cost))
            if total_cost < best_cost:
                best_cost = total_cost
                best_path = path

        update_pheromone(pheromone, ants_paths, graph)

    time_cost = time.time() - start_time
    peak_memory = tracemalloc.get_traced_memory()[1]
    tracemalloc.stop()

    return best_path, best_cost, time_cost, peak_memory

# 测试 ACO 算法
city_number = 5
cities = CityGraph.generate_cities(city_number)
graph = CityGraph.create_complete_graph(cities, True)

best_path, best_cost, time_cost, peak_memory = aco_tsp(graph, start=0)

print("ACO Result:")
print("Path:", best_path)
print("Total Cost:", best_cost)
print("Time Cost:", time_cost)
print("Peak Memory Usage:", peak_memory)