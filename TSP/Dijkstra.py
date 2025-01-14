import heapq
import itertools
import time
import tracemalloc
import CityGraph

# advantage: ensure the optimal solution for small to medium-sized graphs
# disadvantage：huge time complexity and storage complexity

# Dijkstra Algorithm: Calculate shortest paths from `start` to all other nodes
def dijkstra(graph, start):
    n = len(graph)
    distances = {i: float('inf') for i in range(n)}
    distances[start] = 0
    priority_queue = [(0, start)]
    while priority_queue:
        current_distance, current_node = heapq.heappop(priority_queue)
        if current_distance > distances[current_node]:
            continue
        for neighbor, weight in graph[current_node]:
            distance = current_distance + weight
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                heapq.heappush(priority_queue, (distance, neighbor))
    return distances


# Solve TSP using Dijkstra for pairwise shortest paths
def tsp_with_dijkstra(graph):
    tracemalloc.start()
    start_time = time.time()

    n = len(graph)

    # Step 1: Precompute all-pairs shortest paths using Dijkstra
    all_pairs_shortest = [[0] * n for _ in range(n)]
    for i in range(n):
        shortest_paths = dijkstra(graph, i)
        for j in range(n):
            all_pairs_shortest[i][j] = shortest_paths[j]

    # Step 2: Solve TSP using Dynamic Programming (State Compression)
    dp = [[float('inf')] * n for _ in range(1 << n)]
    dp[1][0] = 0  # Start from node 0 with only it visited

    for visited in range(1 << n):
        for current in range(n):
            if not (visited & (1 << current)):  # If current node is not visited
                continue
            for next_node in range(n):
                if visited & (1 << next_node):  # If next_node is already visited
                    continue
                next_visited = visited | (1 << next_node)
                dp[next_visited][next_node] = min(
                    dp[next_visited][next_node],
                    dp[visited][current] + all_pairs_shortest[current][next_node]
                )

    # Step 3: Find the minimum cost to return to the starting node
    min_cost = float('inf')
    for i in range(1, n):
        min_cost = min(min_cost, dp[(1 << n) - 1][i] + all_pairs_shortest[i][0])

    # Step 4: Reconstruct the path (optional)
    path = [0]
    visited = (1 << n) - 1
    current = 0
    while len(path) < n:
        for next_node in range(n):
            if next_node != current and (visited & (1 << next_node)):
                if dp[visited][current] == dp[visited ^ (1 << current)][next_node] + all_pairs_shortest[next_node][current]:
                    path.append(next_node)
                    visited ^= (1 << current)
                    current = next_node
                    break
    path.append(0)  # Return to start

    end_time = time.time()
    current_memory, peak_memory = tracemalloc.get_traced_memory()
    tracemalloc.stop()

    time_cost = end_time - start_time
    return path, min_cost, time_cost, peak_memory

# Test Dijkstra
print("####################################################################################")
print("City Number == 5")
city_number = 5
cities = CityGraph.generate_cities(city_number)

# test Dijkstra sym
graph = CityGraph.create_complete_graph(cities, True)
min_cost_path, min_cost, time_cost, peak_memory_consumption = tsp_with_dijkstra(graph)

print("Minimum cost path:", min_cost_path)
print("Minimum cost:", min_cost)
print("Time cost:", time_cost)
print("Peak memory consumption:", peak_memory_consumption)
print("#######################################################################")

# test Dijkstra asym
graph = CityGraph.create_complete_graph(cities, False)
min_cost_path, min_cost, time_cost, peak_memory_consumption = tsp_with_dijkstra(graph)

print("Minimum cost path:", min_cost_path)
print("Minimum cost:", min_cost)
print("Time cost:", time_cost)
print("Peak memory consumption:", peak_memory_consumption)
print("#######################################################################")

#reduce 20% edge sym
graph = CityGraph.create_complete_graph(cities, True)
edge_list = CityGraph.graph_to_edge_list(graph)
reduced_edge_list = CityGraph.remove_20_percent_edges(edge_list)
reduced_graph = CityGraph.edge_list_to_graph(reduced_edge_list, city_number)
min_cost_path, min_cost, time_cost, peak_memory_consumption = tsp_with_dijkstra(reduced_graph)
print("Minimum cost path:", min_cost_path)
print("Minimum cost:", min_cost)
print("Time cost:", time_cost)
print("Peak memory consumption:", peak_memory_consumption)
print("#######################################################################")

#reduce 20% edge sym
graph = CityGraph.create_complete_graph(cities, False)
edge_list = CityGraph.graph_to_edge_list(graph)
reduced_edge_list = CityGraph.remove_20_percent_edges(edge_list)
reduced_graph = CityGraph.edge_list_to_graph(reduced_edge_list, city_number)
min_cost_path, min_cost, time_cost, peak_memory_consumption = tsp_with_dijkstra(reduced_graph)
print("Minimum cost path:", min_cost_path)
print("Minimum cost:", min_cost)
print("Time cost (DFS):", time_cost)
print("Peak memory consumption:", peak_memory_consumption)