import networkx as nx
import matplotlib.pyplot as plt

class Graph:
    def __init__(self):
        self.nodes = []
        self.edges = []

    def add_nodes_from(self, nodeArr):
        for node in nodeArr:
            self.nodes.append(node)
        
    def add_edges_from(self, edgeArr):
        for edge in edgeArr:
            self.edges.append(edge)
    
    def get_node_pos(self):
        return [node[1] for node in self.nodes]
    
    def get_edge_weight(self):
        return [edge[2] for edge in self.edges]
    
    def get_neighbours(self, node):
        neighbours = []
        for edge in edges:
            if edge[0] == node[0]:
                neighbours.append(edge[1])
            elif edge[1] == node[0]:
                neighbours.append(edge[0])
        return neighbours    

    def DFS(self, start, end, path = None, visited = None):
        if path is None:
            path = []
        if visited is None:
            visited = set()
        path.append(start)
        visited.add(start)
    
        if start == end:
            return path[:]

        for neighbour in self.get_neighbours(start):
            if neighbour not in visited:
                result = self.DFS(neighbour, end, path, visited)
                if result:  # Path found
                    return result
        path.pop() # Backtrack
        return

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

edges1 = [(1, 2, 44),
         (2, 3, 34),
         (2, 4, 104),
         (3, 'X1', 31),
         (3, 5, 71),
         (5, 6, 87),
         (5, 'RY', 44),
         (6, 7, 104),
         (6, 13, 76),
         (11, 13, 105),
         (12, 14, 65),
         (7, 10, 37),
         (11, 10, 39),
         (10, 'X3', 26),
         (7, 8, 34),
         (8, 'X2', 23),
         (8, 9, 71),
         (4, 9, 87),
         (4, 'BG', 44),
         (9, 14, 76),
         (11, 12, 39),
         (12, 'X4', 23)]

map = nx.Graph()

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

map1 = Graph()
map1.add_nodes_from(nodes1)
map1.add_edges_from(edges1)

#print(map1.DFS(1, 9))

nx.draw(map, nx.get_node_attributes(map, 'pos'), with_labels = True)
nx.draw_networkx_edge_labels(map, nx.get_node_attributes(map, 'pos'), nx.get_edge_attributes(map, "weight"))
plt.show()