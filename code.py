import networkx as nx
import matplotlib.pyplot as plt
import sys

def create_weighted_graph():
    #Creates a weighted graph based on user input. Returns the created graph.

    graph={}
    n=int(input("Enter number of vertices:"))
    for i in range(n):
        print('For vertex',i+1,':')
        vertex=input("Enter name of the vertex:")
        edges=input('Enter neighbours of this vertex(neighbour,weight< >neighbour,weight and so on):').split()

        graph[vertex]={}
        for edge in edges:
            neighbour,weight=edge.split(',')
            graph[vertex][neighbour]=int(weight)

    return graph


def dijkstra(graph, start, destination):
     #Applies Dijkstra's algorithm to find the shortest path in a weighted graph.
    #Returns a tuple of distances and parents for each vertex.

    distances = {vertex: sys.maxsize for vertex in graph}
    distances[start] = 0
    visited = set()
    parent = {vertex: None for vertex in graph}

    while len(visited) < len(graph):
        # Find the vertex with the minimum distance
        min_distance = sys.maxsize
        min_vertex = None

        for vertex in graph:
            if vertex not in visited and distances[vertex] < min_distance:
                min_distance = distances[vertex]
                min_vertex = vertex

        # Mark the current vertex as visited
        visited.add(min_vertex)

        if min_vertex == destination:
            break

        # Update distances to neighboring vertices
        for neighbor, weight in graph[min_vertex].items():
            distance = distances[min_vertex] + weight

            if distance < distances[neighbor]:
                distances[neighbor] = distance
                parent[neighbor] = min_vertex

    return distances,parent


def construct_path(parent, destination):

    #Constructs the path from the destination vertex to the start vertex using the parent dictionary.
    #Returns the path as a list.
    path = []
    current = destination

    while current is not None:
        path.append(current)
        current = parent[current]

    return path[::-1]

def second_shortest_path(graph, start, destination, shortest_path):
    #Finds the second shortest path in a weighted graph.
    #Removes edges from the shortest path and runs Dijkstra's algorithm on the modified graph.
    #Returns the distance and path of the second shortest path
    
    for i in range(len(shortest_path) - 1):
        u = shortest_path[i]
        v = shortest_path[i + 1]
        if v in graph[u]:
            del graph[u][v]
    
    # Run Dijkstra's algorithm on the modified graph
    second_shortest_dis, second_shortest_parent = dijkstra(graph, start, destination)
    second_shortest_distance = second_shortest_dis[destination]
    second_shortest_path = construct_path(second_shortest_parent, destination)
    
    return second_shortest_distance, second_shortest_path

# create the weighted graph
weighted_graph = create_weighted_graph()

# Get user input for start and destination vertices
start = input("Enter start vertex:")
destination = input("Enter destination vertex:")

# Find the shortest path and shortest distance using Dijkstra's algorithm
shortest_dis, shortest_parent = dijkstra(weighted_graph, start, destination)
shortest_distance = shortest_dis[destination]
shortest_path = construct_path(shortest_parent, destination)

# Find the second shortest path and distance
second_shortest_distance, second_shortest_path = second_shortest_path(weighted_graph, start, destination, shortest_path)

# Print the results
print("Length of shortest distance between", start, "and", destination, ":", shortest_distance)
print("Shortest path:", shortest_path)
print("Length of second shortest distance between", start, "and", destination, ":", second_shortest_distance)
print("Second shortest path:", second_shortest_path)

# Create an empty graph
G = nx.Graph()

# Add vertices to the graph
G.add_nodes_from(weighted_graph.keys())

# Add edges to the graph
for node, edges in weighted_graph.items():
    for neighbor, weight in edges.items():
        G.add_edge(node, neighbor, weight=weight)

# Visualize the graph
pos = nx.spring_layout(G)  # Choose a layout algorithm
nx.draw(G, pos, with_labels=True, node_color='lightblue', node_size=500, font_size=10)
nx.draw_networkx_edge_labels(G, pos, edge_labels=nx.get_edge_attributes(G, 'weight'))
plt.show()
