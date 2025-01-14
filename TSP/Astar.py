import heapq
import CityGraph
import time
import tracemalloc


# 启发式函数：可采纳（Admissible）启发式
def admissible_heuristic(current, start, visited, graph):
    # 找到从当前节点到未访问节点的最小边权
    min_edge = float('inf')
    for neighbor, weight in graph[current]:  # 遍历当前节点的邻居
        if not (visited & (1 << neighbor)):  # 如果节点未访问
            min_edge = min(min_edge, weight)
    return min_edge if min_edge < float('inf') else 0


# 启发式函数：不可采纳（Inadmissible）启发式
def inadmissible_heuristic(current, start, visited, graph):
    # 找到从当前节点到未访问节点的最大边权
    max_edge = 0
    for neighbor, weight in graph[current]:  # 遍历当前节点的邻居
        if not (visited & (1 << neighbor)):  # 如果节点未访问
            max_edge = max(max_edge, weight)
    return max_edge


# A* 算法求解 TSP
def a_star_tsp(graph, start, heuristic_func):
    tracemalloc.start()
    start_time = time.time()

    n = len(graph)
    visited_all = (1 << n) - 1  # 所有节点都访问的状态
    open_set = [(0, start, 1 << start, [start])]  # (f, current, visited, path)
    best_path = None
    best_cost = float('inf')

    while open_set:
        f, current, visited, path = heapq.heappop(open_set)

        # 如果所有节点都访问过且返回起点，更新最优解
        if visited == visited_all:
            for neighbor, weight in graph[current]:
                if neighbor == start:
                    total_cost = f + weight
                    if total_cost < best_cost:
                        best_cost = total_cost
                        best_path = path + [start]
                    break
            continue

        # 扩展当前节点的邻居
        for neighbor, weight in graph[current]:
            if not (visited & (1 << neighbor)):  # 邻居未访问
                new_visited = visited | (1 << neighbor)
                g = f + weight
                h = heuristic_func(neighbor, start, new_visited, graph)
                heapq.heappush(open_set, (g + h, neighbor, new_visited, path + [neighbor]))

    # 返回结果
    time_cost = time.time() - start_time
    peak_memory = tracemalloc.get_traced_memory()[1]
    tracemalloc.stop()
    return best_path, best_cost, time_cost, peak_memory

# 测试 A* 算法
print("####################################################################################")
print("City Number == 5")
city_number = 5
cities = CityGraph.generate_cities(city_number)
graph = CityGraph.create_complete_graph(cities, True)

# 使用可采纳启发式
admissible_path, admissible_cost, admissible_time, admissible_memory = a_star_tsp(
    graph, start=0, heuristic_func=admissible_heuristic
)

# 使用不可采纳启发式
inadmissible_path, inadmissible_cost, inadmissible_time, inadmissible_memory = a_star_tsp(
    graph, start=0, heuristic_func=inadmissible_heuristic
)

# 输出结果
print("Admissible Heuristic:")
print("Path:", admissible_path)
print("Total Cost:", admissible_cost)
print("Time Cost:", admissible_time)
print("Peak Memory Usage:", admissible_memory)

print("\nInadmissible Heuristic:")
print("Path:", inadmissible_path)
print("Total Cost:", inadmissible_cost)
print("Time Cost:", inadmissible_time)
print("Peak Memory Usage:", inadmissible_memory)