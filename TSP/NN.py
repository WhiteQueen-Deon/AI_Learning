import time
import tracemalloc
import CityGraph
def calculate_cost(path, graph):
    cost = 0
    for i in range(len(path) - 1):
        for neighbor, distance in graph[path[i]]:
            if neighbor == path[i + 1]:
                cost += distance
                break
    return cost


def nearest_neighbor_tsp(graph, start=0):
    tracemalloc.start()
    start_time = time.time()

    n = len(graph)
    visited = [False] * n
    path = [start]
    visited[start] = True
    current_city = start
    total_cost = 0

    while len(path) < n:
        neighbors = graph[current_city]
        next_city = None
        min_distance = float("inf")

        for neighbor, distance in neighbors:
            if not visited[neighbor] and distance < min_distance:
                next_city = neighbor
                min_distance = distance

        if next_city is not None:
            path.append(next_city)
            total_cost += min_distance
            visited[next_city] = True
            current_city = next_city

    if start in [neighbor for neighbor, _ in graph[current_city]]:
        for neighbor, distance in graph[current_city]:
            if neighbor == start:
                total_cost += distance
                break
    path.append(start)

    current, peak_memory = tracemalloc.get_traced_memory()
    tracemalloc.stop()
    time_cost = time.time() - start_time

    return path, total_cost, time_cost, peak_memory

# test DFS
print("####################################################################################")
print("####################################################################################")
print("####################################################################################")
print("City Number == 5")
city_number = 5
cities = CityGraph.generate_cities(city_number)

# test dfs sym
graph = CityGraph.create_complete_graph(cities, True)
min_cost_path, min_cost, time_cost, peak_memory_consumption = nearest_neighbor_tsp(graph, 0)

print("Minimum cost path(DFS):", min_cost_path)
print("Minimum cost (DFS):", min_cost)
print("Time cost (DFS):", time_cost)
print("Peak memory consumption (DFS):", peak_memory_consumption)
print("#######################################################################")

# test dfs asym
graph = CityGraph.create_complete_graph(cities, False)
min_cost_path, min_cost, time_cost, peak_memory_consumption = nearest_neighbor_tsp(graph, 0)

print("Minimum cost path(DFS):", min_cost_path)
print("Minimum cost (DFS):", min_cost)
print("Time cost (DFS):", time_cost)
print("Peak memory consumption (DFS):", peak_memory_consumption)
print("#######################################################################")

#reduce 20% edge sym
graph = CityGraph.create_complete_graph(cities, True)
edge_list = CityGraph.graph_to_edge_list(graph)
reduced_edge_list = CityGraph.remove_20_percent_edges(edge_list)
reduced_graph = CityGraph.edge_list_to_graph(reduced_edge_list, city_number)
min_cost_path, min_cost, time_cost, peak_memory_consumption = nearest_neighbor_tsp(reduced_graph, 0)
print("Minimum cost path(DFS):", min_cost_path)
print("Minimum cost (DFS):", min_cost)
print("Time cost (DFS):", time_cost)
print("Peak memory consumption (DFS):", peak_memory_consumption)
print("#######################################################################")

#reduce 20% edge sym
graph = CityGraph.create_complete_graph(cities, False)
edge_list = CityGraph.graph_to_edge_list(graph)
reduced_edge_list = CityGraph.remove_20_percent_edges(edge_list)
reduced_graph = CityGraph.edge_list_to_graph(reduced_edge_list, city_number)
min_cost_path, min_cost, time_cost, peak_memory_consumption = nearest_neighbor_tsp(reduced_graph, 0)
print("Minimum cost path(DFS):", min_cost_path)
print("Minimum cost (DFS):", min_cost)
print("Time cost (DFS):", time_cost)
print("Peak memory consumption (DFS):", peak_memory_consumption)


print("####################################################################################")
print("####################################################################################")
print("####################################################################################")
print("City Number == 10")
city_number = 10
cities = CityGraph.generate_cities(city_number)

# test dfs sym
graph = CityGraph.create_complete_graph(cities, True)
min_cost_path, min_cost, time_cost, peak_memory_consumption = nearest_neighbor_tsp(graph, 0)

print("Minimum cost path(DFS):", min_cost_path)
print("Minimum cost (DFS):", min_cost)
print("Time cost (DFS):", time_cost)
print("Peak memory consumption (DFS):", peak_memory_consumption)
print("#######################################################################")

# test dfs asym
graph = CityGraph.create_complete_graph(cities, False)
min_cost_path, min_cost, time_cost, peak_memory_consumption = nearest_neighbor_tsp(graph, 0)

print("Minimum cost path(DFS):", min_cost_path)
print("Minimum cost (DFS):", min_cost)
print("Time cost (DFS):", time_cost)
print("Peak memory consumption (DFS):", peak_memory_consumption)
print("#######################################################################")

#reduce 20% edge sym
graph = CityGraph.create_complete_graph(cities, True)
edge_list = CityGraph.graph_to_edge_list(graph)
reduced_edge_list = CityGraph.remove_20_percent_edges(edge_list)
reduced_graph = CityGraph.edge_list_to_graph(reduced_edge_list, city_number)
min_cost_path, min_cost, time_cost, peak_memory_consumption = nearest_neighbor_tsp(reduced_graph, 0)
print("Minimum cost path(DFS):", min_cost_path)
print("Minimum cost (DFS):", min_cost)
print("Time cost (DFS):", time_cost)
print("Peak memory consumption (DFS):", peak_memory_consumption)
print("#######################################################################")

#reduce 20% edge sym
graph = CityGraph.create_complete_graph(cities, False)
edge_list = CityGraph.graph_to_edge_list(graph)
reduced_edge_list = CityGraph.remove_20_percent_edges(edge_list)
reduced_graph = CityGraph.edge_list_to_graph(reduced_edge_list, city_number)
min_cost_path, min_cost, time_cost, peak_memory_consumption = nearest_neighbor_tsp(reduced_graph, 0)
print("Minimum cost path(DFS):", min_cost_path)
print("Minimum cost (DFS):", min_cost)
print("Time cost (DFS):", time_cost)
print("Peak memory consumption (DFS):", peak_memory_consumption)

print("####################################################################################")
print("####################################################################################")
print("####################################################################################")
print("City Number == 15")
city_number = 15
cities = CityGraph.generate_cities(city_number)

# test dfs sym
graph = CityGraph.create_complete_graph(cities, True)
min_cost_path, min_cost, time_cost, peak_memory_consumption = nearest_neighbor_tsp(graph, 0)

print("Minimum cost path(DFS):", min_cost_path)
print("Minimum cost (DFS):", min_cost)
print("Time cost (DFS):", time_cost)
print("Peak memory consumption (DFS):", peak_memory_consumption)
print("#######################################################################")

# test dfs asym
graph = CityGraph.create_complete_graph(cities, False)
min_cost_path, min_cost, time_cost, peak_memory_consumption = nearest_neighbor_tsp(graph, 0)

print("Minimum cost path(DFS):", min_cost_path)
print("Minimum cost (DFS):", min_cost)
print("Time cost (DFS):", time_cost)
print("Peak memory consumption (DFS):", peak_memory_consumption)
print("#######################################################################")

#reduce 20% edge sym
graph = CityGraph.create_complete_graph(cities, True)
edge_list = CityGraph.graph_to_edge_list(graph)
reduced_edge_list = CityGraph.remove_20_percent_edges(edge_list)
reduced_graph = CityGraph.edge_list_to_graph(reduced_edge_list, city_number)
min_cost_path, min_cost, time_cost, peak_memory_consumption = nearest_neighbor_tsp(reduced_graph, 0)
print("Minimum cost path(DFS):", min_cost_path)
print("Minimum cost (DFS):", min_cost)
print("Time cost (DFS):", time_cost)
print("Peak memory consumption (DFS):", peak_memory_consumption)
print("#######################################################################")

#reduce 20% edge sym
graph = CityGraph.create_complete_graph(cities, False)
edge_list = CityGraph.graph_to_edge_list(graph)
reduced_edge_list = CityGraph.remove_20_percent_edges(edge_list)
reduced_graph = CityGraph.edge_list_to_graph(reduced_edge_list, city_number)
min_cost_path, min_cost, time_cost, peak_memory_consumption = nearest_neighbor_tsp(reduced_graph, 0)
print("Minimum cost path(DFS):", min_cost_path)
print("Minimum cost (DFS):", min_cost)
print("Time cost (DFS):", time_cost)
print("Peak memory consumption (DFS):", peak_memory_consumption)

print("####################################################################################")
print("####################################################################################")
print("####################################################################################")
print("City Number == 20")
city_number = 20
cities = CityGraph.generate_cities(city_number)

# test dfs sym
graph = CityGraph.create_complete_graph(cities, True)
min_cost_path, min_cost, time_cost, peak_memory_consumption = nearest_neighbor_tsp(graph, 0)

print("Minimum cost path(DFS):", min_cost_path)
print("Minimum cost (DFS):", min_cost)
print("Time cost (DFS):", time_cost)
print("Peak memory consumption (DFS):", peak_memory_consumption)
print("#######################################################################")

# test dfs asym
graph = CityGraph.create_complete_graph(cities, False)
min_cost_path, min_cost, time_cost, peak_memory_consumption = nearest_neighbor_tsp(graph, 0)

print("Minimum cost path(DFS):", min_cost_path)
print("Minimum cost (DFS):", min_cost)
print("Time cost (DFS):", time_cost)
print("Peak memory consumption (DFS):", peak_memory_consumption)
print("#######################################################################")

#reduce 20% edge sym
graph = CityGraph.create_complete_graph(cities, True)
edge_list = CityGraph.graph_to_edge_list(graph)
reduced_edge_list = CityGraph.remove_20_percent_edges(edge_list)
reduced_graph = CityGraph.edge_list_to_graph(reduced_edge_list, city_number)
min_cost_path, min_cost, time_cost, peak_memory_consumption = nearest_neighbor_tsp(reduced_graph, 0)
print("Minimum cost path(DFS):", min_cost_path)
print("Minimum cost (DFS):", min_cost)
print("Time cost (DFS):", time_cost)
print("Peak memory consumption (DFS):", peak_memory_consumption)
print("#######################################################################")

#reduce 20% edge sym
graph = CityGraph.create_complete_graph(cities, False)
edge_list = CityGraph.graph_to_edge_list(graph)
reduced_edge_list = CityGraph.remove_20_percent_edges(edge_list)
reduced_graph = CityGraph.edge_list_to_graph(reduced_edge_list, city_number)
min_cost_path, min_cost, time_cost, peak_memory_consumption = nearest_neighbor_tsp(reduced_graph, 0)
print("Minimum cost path(DFS):", min_cost_path)
print("Minimum cost (DFS):", min_cost)
print("Time cost (DFS):", time_cost)
print("Peak memory consumption (DFS):", peak_memory_consumption)
print("#######################################################################")