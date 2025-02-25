import networkx as nx
import matplotlib.pyplot as plt

class Graph:
    def __init__(self):
        self.nodes = []
        self.edges = []

    def add_nodes_from(nodeArr):
        for node in nodeArr:
            nodes.append(node)
        
    def add_edges_from(edgeArr):
        for edge in edgeArr:
            edges.append(edge)
    
    def get_node_attributes(self, attr):
        return [node[1][attr] for node in self.nodes]
    
    def get_edge_attributes(self, attr):
        return [edge[2][attr] for edge in self.edges]
    
    def dijkstra(self, start_node):
        distances = {}
        visited = {}
        for node in nodes:
            distances.append((node, float('inf')))
            visited.append((node, False))
        distances[start_node] = 0

        for node in nodes:
            min_distance = float('inf')
            u = None
            for node1 in nodes:
                if not visited[node] and distances[node] < min_distance:
                    min_distance = distances[node]
                    u = node

            if u is None:
                break

            visited[u] = True

            for node2 in nodes:
                if self.edges[edges[0] == node1 and edges[1] == node2] != 0 and not visited[node2]:
                    alt = distances[u] + self.adj_matrix[u][v]
                    if alt < distances[v]:
                        distances[v] = alt

        return distances

map = nx.Graph()

nodes1 = [(1, (0,0)),
         (2, (0, 44)),
         (3, (-34, 44)),
         (4, (104, 44)),
         (5, (-105, 44)),
         (6, (-105, 131)),
         (7, (-1, 131)),
         (8, (33, 131)),
         (9, (104, 131)),
         (10, (-1, 168)),
         (11, (-1, 207)),
         (12, (38, 207)),
         (13, (-105, 207)),
         (14, (104, 207)),
         ('X1', (-34, 75)),
         ('X2', (33, 108)),
         ('X3', (-27, 168)),
         ('X4', (38, 184)),
         ('RY', (-105, 0)),
         ('BG', (104, 0))]         

edges1 = [(1, 2, {"weight" : 44}),
         (2, 3, {"weight" : 34}),
         (2, 4, {"weight" : 104}),
         (3, 'X1', {"weight" : 31}),
         (3, 5, {"weight" : 71}),
         (5, 6, {"weight" : 87}),
         (5, 'RY', {"weight" : 44}),
         (6, 7, {"weight" : 104}),
         (6, 13, {"weight" : 76}),
         (11, 13, {"weight" : 105}),
         (12, 14, {"weight" : 65}),
         (7, 10, {"weight" : 37}),
         (11, 10, {"weight" : 39}),
         (10, 'X3', {"weight" : 26}),
         (7, 8, {"weight" : 34}),
         (8, 'X2', {"weight" : 23}),
         (8, 9, {"weight" : 71}),
         (4, 9, {"weight" : 87}),
         (4, 'BG', {"weight" : 44}),
         (9, 14, {"weight" : 76}),
         (11, 12, {"weight" : 39}),
         (12, 'X4', {"weight" : 23})]

nodes = [(1, {'pos' : (0,0)}),
         (2, {'pos' : (0, 44)}),
         (3, {'pos' : (-34, 44)}),
         (4, {'pos' : (104, 44)}),
         (5, {'pos' : (-105, 44)}),
         (6, {'pos' : (-105, 131)}),
         (7, {'pos' : (-1, 131)}),
         (8, {'pos' : (33, 131)}),
         (9, {'pos' : (104, 131)}),
         (10, {'pos' : (-1, 168)}),
         (11, {'pos' : (-1, 207)}),
         (12, {'pos' : (38, 207)}),
         (13, {'pos' : (-105, 207)}),
         (14, {'pos' : (104, 207)}),
         ('X1', {'pos' : (-34, 75)}),
         ('X2', {'pos' : (33, 108)}),
         ('X3', {'pos' : (-27, 168)}),
         ('X4', {'pos' : (38, 184)}),
         ('RY', {'pos' : (-105, 0)}),
         ('BG', {'pos' : (104, 0)})]         

edges = [(1, 2, {"weight" : 44}),
         (2, 3, {"weight" : 34}),
         (2, 4, {"weight" : 104}),
         (3, 'X1', {"weight" : 31}),
         (3, 5, {"weight" : 71}),
         (5, 6, {"weight" : 87}),
         (5, 'RY', {"weight" : 44}),
         (6, 7, {"weight" : 104}),
         (6, 13, {"weight" : 76}),
         (11, 13, {"weight" : 105}),
         (12, 14, {"weight" : 65}),
         (7, 10, {"weight" : 37}),
         (11, 10, {"weight" : 39}),
         (10, 'X3', {"weight" : 26}),
         (7, 8, {"weight" : 34}),
         (8, 'X2', {"weight" : 23}),
         (8, 9, {"weight" : 71}),
         (4, 9, {"weight" : 87}),
         (4, 'BG', {"weight" : 44}),
         (9, 14, {"weight" : 76}),
         (11, 12, {"weight" : 39}),
         (12, 'X4', {"weight" : 23})]

map.add_nodes_from(nodes)
map.add_edges_from(edges)

nx.draw(map, nx.get_node_attributes(map, 'pos'), with_labels = True)
nx.draw_networkx_edge_labels(map, nx.get_node_attributes(map, 'pos'), nx.get_edge_attributes(map, "weight"))
plt.show()