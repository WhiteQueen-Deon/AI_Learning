import CityGraph
from collections import deque
import random
import math


def bfs(graph,start):
    initial_path = [start]
    queue = deque([initial_path])
    n = len(graph)

    while queue:
        path = queue.popleft()
        city = path[-1]

        if len(path) == n:
            path.append(start)
            return path

        for neighbor, _ in graph[city]:
            if neighbor not in path:
                new_path = list(path)
                new_path.append(neighbor)
                queue.append(new_path)

# test bfs
city_number = 5
cities_5 = CityGraph.generate_cities(city_number)
graph = CityGraph.create_complete_graph(cities_5, True)
path_5 = bfs(graph,0)
print(path_5)