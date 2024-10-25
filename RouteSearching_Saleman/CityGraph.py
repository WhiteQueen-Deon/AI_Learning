# Exercise 1 Route searching
# 1.	Create a set of cities (as points) with coordinates x, y on a plane with height as z coordinate.
# The cost of going from city A to city B is equal to the Euclidean distance between two cities,
# if there exists a road. You should define scenarios according to two criteria:
    # a.	There are all the direct connections / c.a. 80% of possible connections
    # b.	The problem is symmetrical / asymmetrical (in asymmetrical – going up is height +10%,
    # going down: -10%)
    # You should choose the coordinates randomly from the range <-100, 100> for x,y and <0, 50> for z.
# 2.	Represent the created map as a weighted (directed) graph, where cities are the nodes and roads
# are the edges of the graph.
# 3.	In the created scene, solve the traveling salesman problem: The salesman starts from a chosen city
# and has to visit every city exactly once before returning to the starting city.
# The goal is to find a path with the lowest cost.
# In the problem, we define state as a partial or full path from the starting city and the corresponding state.
# You should represent the search problem in a form of state tree.
    # a.	Implement a full search of the tree, using BFS and DFS methods.
    # b.	Approximate the solution using greedy search (NN and Dijkstra)
    # c.	Solve/approximate the solution using A* with inadmissible/admissible heuristics
    # d.	Approximate the solution using ACO algorithm
    # 4.	Test each algorithm, in each scenario, for n=5…20 cities, in terms of the found path cost,
    # time and memory consumption.

import itertools
import random
import math
from collections import deque

# create a set of cities
def generate_cities(n):
    cities = []
    for _ in range(n):
        x = random.uniform(-100, 100)
        y = random.uniform(-100, 100)
        z = random.uniform(0, 50)
        cities.append((x, y, z))
    return cities

# # city quantity
# cities = generate_cities(10)

# print(cities)

#calculate Euclidean distance
def euclidean_distance(city1,city2):
    x1,y1,z1 = city1
    x2,y2,z2 = city2
    return math.sqrt((x2-x1)**2+(y2-y1)**2+(z2-z1)**2)

def asymmetric_distance(city1,city2):
    base_distance =euclidean_distance(city1,city2)
    if city2[2] > city1[2]:  # up
        return base_distance * 1.1
    else:  # down
        return base_distance * 0.9

# all the direct connections
def create_complete_graph(cities, symmetric = True):
    graph = {i: [] for i in range(len(cities))}

    for city1, city2 in itertools.combinations(range(len(cities)),2):
        if symmetric:
            distance = euclidean_distance(cities[city1], cities[city2])
            graph[city1].append((city2, distance))
            graph[city2].append((city1, distance))
        else:
            distance = asymmetric_distance(cities[city1], cities[city2])
            graph[city1].append((city2, distance))

    return graph

# edges = create_complete_graph(cities, True)
# print(edges)

# 80% of possible connections
def remove_20_percent_edges(edges):
    num_edges = len(edges)
    num_edges_to_keep = int(num_edges * 0.8)
    random.shuffle(edges)
    return edges[:num_edges_to_keep]

# reduced_edges = remove_20_percent_edges(edges)
# print(reduced_edges)
