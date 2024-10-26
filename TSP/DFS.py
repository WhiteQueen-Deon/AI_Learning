import tracemalloc
import time
import CityGraph

def calculate_cost(path, graph):
    cost = 0
    for i in range(len(path) - 1):
        city1, city2 = path[i], path[i + 1]
        for neighbor , distance in graph[city1]:
            if neighbor == city2:
                cost += distance
                break
    return cost

def dfs(graph, start):
    tracemalloc.start()
    start_time = time.time()

    n = len(graph)
    min_cost_path = None
    min_cost = float("inf")
    stack = [(start, [start])]

    while stack:
        city,path = stack.pop()

        if len(path) == n:
            path.append(start)
            cost = calculate_cost(path, graph)
            if cost < min_cost:
                min_cost = cost
                min_cost_path = path
            path.pop()
            continue

        for neighbor, _ in graph[city]:
            if neighbor not in path:
                new_path = list(path)
                new_path.append(neighbor)
                stack.append((neighbor, new_path))

    end_time = time.time()
    current, peak = tracemalloc.get_traced_memory()
    tracemalloc.stop()

    time_cost = end_time - start_time
    return min_cost_path, min_cost, time_cost, peak



# test DFS
print("####################################################################################")
print("####################################################################################")
print("####################################################################################")
print("City Number == 5")
city_number = 5
cities = CityGraph.generate_cities(city_number)

# test dfs sym
graph = CityGraph.create_complete_graph(cities, True)
min_cost_path, min_cost, time_cost, peak_memory_consumption = dfs(graph, 0)

print("Minimum cost path(DFS):", min_cost_path)
print("Minimum cost (DFS):", min_cost)
print("Time cost (DFS):", time_cost)
print("Peak memory consumption (DFS):", peak_memory_consumption)
print("#######################################################################")

# test dfs asym
graph = CityGraph.create_complete_graph(cities, False)
min_cost_path, min_cost, time_cost, peak_memory_consumption = dfs(graph, 0)

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
min_cost_path, min_cost, time_cost, peak_memory_consumption = dfs(reduced_graph, 0)
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
min_cost_path, min_cost, time_cost, peak_memory_consumption = dfs(reduced_graph, 0)
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
min_cost_path, min_cost, time_cost, peak_memory_consumption = dfs(graph, 0)

print("Minimum cost path(DFS):", min_cost_path)
print("Minimum cost (DFS):", min_cost)
print("Time cost (DFS):", time_cost)
print("Peak memory consumption (DFS):", peak_memory_consumption)
print("#######################################################################")

# test dfs asym
graph = CityGraph.create_complete_graph(cities, False)
min_cost_path, min_cost, time_cost, peak_memory_consumption = dfs(graph, 0)

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
min_cost_path, min_cost, time_cost, peak_memory_consumption = dfs(reduced_graph, 0)
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
min_cost_path, min_cost, time_cost, peak_memory_consumption = dfs(reduced_graph, 0)
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
min_cost_path, min_cost, time_cost, peak_memory_consumption = dfs(graph, 0)

print("Minimum cost path(DFS):", min_cost_path)
print("Minimum cost (DFS):", min_cost)
print("Time cost (DFS):", time_cost)
print("Peak memory consumption (DFS):", peak_memory_consumption)
print("#######################################################################")

# test dfs asym
graph = CityGraph.create_complete_graph(cities, False)
min_cost_path, min_cost, time_cost, peak_memory_consumption = dfs(graph, 0)

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
min_cost_path, min_cost, time_cost, peak_memory_consumption = dfs(reduced_graph, 0)
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
min_cost_path, min_cost, time_cost, peak_memory_consumption = dfs(reduced_graph, 0)
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
min_cost_path, min_cost, time_cost, peak_memory_consumption = dfs(graph, 0)

print("Minimum cost path(DFS):", min_cost_path)
print("Minimum cost (DFS):", min_cost)
print("Time cost (DFS):", time_cost)
print("Peak memory consumption (DFS):", peak_memory_consumption)
print("#######################################################################")

# test dfs asym
graph = CityGraph.create_complete_graph(cities, False)
min_cost_path, min_cost, time_cost, peak_memory_consumption = dfs(graph, 0)

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
min_cost_path, min_cost, time_cost, peak_memory_consumption = dfs(reduced_graph, 0)
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
min_cost_path, min_cost, time_cost, peak_memory_consumption = dfs(reduced_graph, 0)
print("Minimum cost path(DFS):", min_cost_path)
print("Minimum cost (DFS):", min_cost)
print("Time cost (DFS):", time_cost)
print("Peak memory consumption (DFS):", peak_memory_consumption)
print("#######################################################################")