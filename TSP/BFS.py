import CityGraph
from collections import deque
import random
import math
import tracemalloc
import time

# Advantages:
# Shortest Path: Guarantees the shortest path in an unweighted graph when searching for a single solution.
# Complete: Always finds a solution if one exists.
# Optimal for Unweighted Graphs: Ensures optimality in terms of the number of edges traversed.
# Disadvantages:
# Memory-Intensive: Requires storing all nodes at the current level in the queue, which can consume a lot of memory for large graphs.
# Slower for Deep Graphs: Inefficient for graphs with many levels and a sparse solution near the end.

def bfs(graph, start):
    start_time = time.time()
    tracemalloc.start()

    initial_path = [start]
    queue = deque([initial_path])
    n = len(graph)
    all_paths = []

    while queue:
        path = queue.popleft()
        city = path[-1]

        if len(path) == n:
            path.append(start)
            cost = calculate_cost(path, graph)
            all_paths.append((path, cost))
            continue

        for neighbor, _ in graph[city]:
            if neighbor not in path:
                new_path = list(path)
                new_path.append(neighbor)
                queue.append(new_path)

# memory consumption
    current, peak = tracemalloc.get_traced_memory()
    tracemalloc.stop()

# time consumption
    end_time = time.time()
    time_cost = end_time - start_time

    if all_paths:
        min_cost_path, min_cost = min(all_paths, key=lambda x: x[1])
        return min_cost_path, min_cost, peak, time_cost
    else:
        return [], float("inf"), peak, time_cost

def calculate_cost(path, graph):
    total_cost = 0
    for i in range(len(path) - 1):
        for neighbor, cost in graph[path[i]]:
            if neighbor == path[i + 1]:
                total_cost += cost
                break
    return total_cost

city_number = 5
cities = CityGraph.generate_cities(city_number)
graph = CityGraph.create_complete_graph(cities, True)
min_cost_path, min_cost, peak_memory_consumption, time_cost = bfs(graph, 0)

# print("####################################################################################")
# print("####################################################################################")
# print("####################################################################################")
# print("City Number == 5")
# city_number = 5
# cities = CityGraph.generate_cities(city_number)
#
# # test bfs sym
# graph = CityGraph.create_complete_graph(cities, True)
# min_cost_path, min_cost, peak_memory_consumption, time_cost = bfs(graph, 0)
# print("Completed Symmetric Graph")
# print("Minimum cost path:", min_cost_path)
# print("Minimum cost:", min_cost)
# print("Minimum peak_memory_consumption:", peak_memory_consumption)
# print("Minimum time_cost:", time_cost)
# print("#######################################################################")
#
# # test bfs sym
# graph = CityGraph.create_complete_graph(cities, False)
# min_cost_path, min_cost, peak_memory_consumption, time_cost = bfs(graph, 0)
# print("Completed Asymmetric Graph")
# print("Minimum cost path:", min_cost_path)
# print("Minimum cost:", min_cost)
# print("Minimum peak_memory_consumption:", peak_memory_consumption)
# print("Minimum time_cost:", time_cost)
# print("#######################################################################")
#
# #reduce 20% edge sym
# graph = CityGraph.create_complete_graph(cities, True)
# edge_list = CityGraph.graph_to_edge_list(graph)
# reduced_edge_list = CityGraph.remove_20_percent_edges(edge_list)
# reduced_graph = CityGraph.edge_list_to_graph(reduced_edge_list, city_number)
# min_cost_path, min_cost, peak_memory_consumption, time_cost = bfs(reduced_graph, 0)
# print("Reduced Symmetric Graph")
# print("Minimum cost path:", min_cost_path)
# print("Minimum cost:", min_cost)
# print("Minimum peak_memory_consumption:", peak_memory_consumption)
# print("Minimum time_cost:", time_cost)
# print("#######################################################################")
#
# #reduce 20% edge asym
# graph = CityGraph.create_complete_graph(cities, False)
# edge_list = CityGraph.graph_to_edge_list(graph)
# reduced_edge_list = CityGraph.remove_20_percent_edges(edge_list)
# reduced_graph = CityGraph.edge_list_to_graph(reduced_edge_list, city_number)
# min_cost_path, min_cost, peak_memory_consumption, time_cost = bfs(reduced_graph, 0)
# print("Reduced Asymmetric Graph")
# print("Minimum cost path:", min_cost_path)
# print("Minimum cost:", min_cost)
# print("Minimum peak_memory_consumption:", peak_memory_consumption)
# print("Minimum time_cost:", time_cost)
#
# print("####################################################################################")
# print("####################################################################################")
# print("####################################################################################")
# print("City Number == 10 ")
# city_number = 10
# cities = CityGraph.generate_cities(city_number)
#
# # test bfs sym
# graph = CityGraph.create_complete_graph(cities, True)
# min_cost_path, min_cost, peak_memory_consumption, time_cost = bfs(graph, 0)
# print("Completed Symmetric Graph")
# print("Minimum cost path:", min_cost_path)
# print("Minimum cost:", min_cost)
# print("Minimum peak_memory_consumption:", peak_memory_consumption)
# print("Minimum time_cost:", time_cost)
# print("#######################################################################")
#
# # test bfs sym
# graph = CityGraph.create_complete_graph(cities, False)
# min_cost_path, min_cost, peak_memory_consumption, time_cost = bfs(graph, 0)
# print("Completed Asymmetric Graph")
# print("Minimum cost path:", min_cost_path)
# print("Minimum cost:", min_cost)
# print("Minimum peak_memory_consumption:", peak_memory_consumption)
# print("Minimum time_cost:", time_cost)
# print("#######################################################################")
#
# #reduce 20% edge sym
# graph = CityGraph.create_complete_graph(cities, True)
# edge_list = CityGraph.graph_to_edge_list(graph)
# reduced_edge_list = CityGraph.remove_20_percent_edges(edge_list)
# reduced_graph = CityGraph.edge_list_to_graph(reduced_edge_list, city_number)
# min_cost_path, min_cost, peak_memory_consumption, time_cost = bfs(reduced_graph, 0)
# print("Reduced Symmetric Graph")
# print("Minimum cost path:", min_cost_path)
# print("Minimum cost:", min_cost)
# print("Minimum peak_memory_consumption:", peak_memory_consumption)
# print("Minimum time_cost:", time_cost)
# print("#######################################################################")
#
# #reduce 20% edge asym
# graph = CityGraph.create_complete_graph(cities, False)
# edge_list = CityGraph.graph_to_edge_list(graph)
# reduced_edge_list = CityGraph.remove_20_percent_edges(edge_list)
# reduced_graph = CityGraph.edge_list_to_graph(reduced_edge_list, city_number)
# min_cost_path, min_cost, peak_memory_consumption, time_cost = bfs(reduced_graph, 0)
# print("Reduced Asymmetric Graph")
# print("Minimum cost path:", min_cost_path)
# print("Minimum cost:", min_cost)
# print("Minimum peak_memory_consumption:", peak_memory_consumption)
# print("Minimum time_cost:", time_cost)
#
# print("####################################################################################")
# print("####################################################################################")
# print("####################################################################################")
# print("City Number == 15")
# city_number = 15
# cities = CityGraph.generate_cities(city_number)
#
# # test bfs sym
# graph = CityGraph.create_complete_graph(cities, True)
# min_cost_path, min_cost, peak_memory_consumption, time_cost = bfs(graph, 0)
# print("Completed Symmetric Graph")
# print("Minimum cost path:", min_cost_path)
# print("Minimum cost:", min_cost)
# print("Minimum peak_memory_consumption:", peak_memory_consumption)
# print("Minimum time_cost:", time_cost)
# print("#######################################################################")
#
# # test bfs sym
# graph = CityGraph.create_complete_graph(cities, False)
# min_cost_path, min_cost, peak_memory_consumption, time_cost = bfs(graph, 0)
# print("Completed Asymmetric Graph")
# print("Minimum cost path:", min_cost_path)
# print("Minimum cost:", min_cost)
# print("Minimum peak_memory_consumption:", peak_memory_consumption)
# print("Minimum time_cost:", time_cost)
# print("#######################################################################")
#
# #reduce 20% edge sym
# graph = CityGraph.create_complete_graph(cities, True)
# edge_list = CityGraph.graph_to_edge_list(graph)
# reduced_edge_list = CityGraph.remove_20_percent_edges(edge_list)
# reduced_graph = CityGraph.edge_list_to_graph(reduced_edge_list, city_number)
# min_cost_path, min_cost, peak_memory_consumption, time_cost = bfs(reduced_graph, 0)
# print("Reduced Symmetric Graph")
# print("Minimum cost path:", min_cost_path)
# print("Minimum cost:", min_cost)
# print("Minimum peak_memory_consumption:", peak_memory_consumption)
# print("Minimum time_cost:", time_cost)
# print("#######################################################################")
#
# #reduce 20% edge asym
# graph = CityGraph.create_complete_graph(cities, False)
# edge_list = CityGraph.graph_to_edge_list(graph)
# reduced_edge_list = CityGraph.remove_20_percent_edges(edge_list)
# reduced_graph = CityGraph.edge_list_to_graph(reduced_edge_list, city_number)
# min_cost_path, min_cost, peak_memory_consumption, time_cost = bfs(reduced_graph, 0)
# print("Reduced Asymmetric Graph")
# print("Minimum cost path:", min_cost_path)
# print("Minimum cost:", min_cost)
# print("Minimum peak_memory_consumption:", peak_memory_consumption)
# print("Minimum time_cost:", time_cost)
#
# print("####################################################################################")
# print("####################################################################################")
# print("####################################################################################")
# print("City Number == 20")
# city_number = 20
# cities = CityGraph.generate_cities(city_number)
#
# # test bfs sym
# graph = CityGraph.create_complete_graph(cities, True)
# min_cost_path, min_cost, peak_memory_consumption, time_cost = bfs(graph, 0)
# print("Completed Symmetric Graph")
# print("Minimum cost path:", min_cost_path)
# print("Minimum cost:", min_cost)
# print("Minimum peak_memory_consumption:", peak_memory_consumption)
# print("Minimum time_cost:", time_cost)
# print("#######################################################################")
#
# # test bfs sym
# graph = CityGraph.create_complete_graph(cities, False)
# min_cost_path, min_cost, peak_memory_consumption, time_cost = bfs(graph, 0)
# print("Completed Asymmetric Graph")
# print("Minimum cost path:", min_cost_path)
# print("Minimum cost:", min_cost)
# print("Minimum peak_memory_consumption:", peak_memory_consumption)
# print("Minimum time_cost:", time_cost)
# print("#######################################################################")
#
# #reduce 20% edge sym
# graph = CityGraph.create_complete_graph(cities, True)
# edge_list = CityGraph.graph_to_edge_list(graph)
# reduced_edge_list = CityGraph.remove_20_percent_edges(edge_list)
# reduced_graph = CityGraph.edge_list_to_graph(reduced_edge_list, city_number)
# min_cost_path, min_cost, peak_memory_consumption, time_cost = bfs(reduced_graph, 0)
# print("Reduced Symmetric Graph")
# print("Minimum cost path:", min_cost_path)
# print("Minimum cost:", min_cost)
# print("Minimum peak_memory_consumption:", peak_memory_consumption)
# print("Minimum time_cost:", time_cost)
# print("#######################################################################")
#
# #reduce 20% edge asym
# graph = CityGraph.create_complete_graph(cities, False)
# edge_list = CityGraph.graph_to_edge_list(graph)
# reduced_edge_list = CityGraph.remove_20_percent_edges(edge_list)
# reduced_graph = CityGraph.edge_list_to_graph(reduced_edge_list, city_number)
# min_cost_path, min_cost, peak_memory_consumption, time_cost = bfs(reduced_graph, 0)
# print("Reduced Asymmetric Graph")
# print("Minimum cost path:", min_cost_path)
# print("Minimum cost:", min_cost)
# print("Minimum peak_memory_consumption:", peak_memory_consumption)
# print("Minimum time_cost:", time_cost)
