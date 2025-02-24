import networkx as nx
import matplotlib.pyplot as plt

map = nx.Graph()

nodes = [(1, {'pos' : (0,0)}),
         (2, {'pos' : (0, 44)}),
         (3, {'pos' : (-34, 44)}),
         (4, {'pos' : (144, 44)}),
         (5, {'pos' : (-105, 44)}),
         (6, {'pos' : (-105, 131)}),
         (7, {'pos' : (-1, 131)}),
         (8, {'pos' : (33, 131)}),
         (9, {'pos' : (104, 101)}),
         (10, {'pos' : (-1, 168)}),
         (11, {'pos' : (-1, 207)}),
         (12, {'pos' : (38, 207)}),
         ('X1', {'pos' : (-34, 75)}),
         ('X2', {'pos' : (33, 108)}),
         ('X3', {'pos' : (-27, 168)}),
         ('X4', {'pos' : (38, 184)}),
         ('RY', {'pos' : (-105, 0)}),
         ('BG', {'pos' : (104, 0)})]         

edges = [(1, 2, {"weight" : 1}),
         (2, 3, {"weight" : 1}),
         (2, 4, {"weight" : 1}),
         (3, 'X1', {"weight" : 1}),
         (3, 5, {"weight" : 1}),
         (5, 6, {"weight" : 1}),
         (5, 'RY', {"weight" : 1}),
         (6, 7, {"weight" : 1}),
         (6, 11, {"weight" : 1}),
         (7, 10, {"weight" : 1}),
         (11, 10, {"weight" : 1}),
         (10, 'X3', {"weight" : 1}),
         (7, 8, {"weight" : 1}),
         (8, 'X2', {"weight" : 1}),
         (8, 9, {"weight" : 1}),
         (4, 9, {"weight" : 1}),
         (4, 'BG', {"weight" : 1}),
         (9, 12, {"weight" : 1}),
         (11, 12, {"weight" : 1}),
         (12, 'X4', {"weight" : 1})]

map.add_nodes_from(nodes)
map.add_edges_from(edges)
pos = {(x, y): (y, -x) for x, y in map.nodes()}

nx.draw(map, pos, with_labels = True)
plt.show()