import networkx as nx
import matplotlib.pyplot as plt

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
         (6, 11, {"weight" : 180}),
         (7, 10, {"weight" : 37}),
         (11, 10, {"weight" : 39}),
         (10, 'X3', {"weight" : 26}),
         (7, 8, {"weight" : 34}),
         (8, 'X2', {"weight" : 23}),
         (8, 9, {"weight" : 71}),
         (4, 9, {"weight" : 87}),
         (4, 'BG', {"weight" : 44}),
         (9, 12, {"weight" : 142}),
         (11, 12, {"weight" : 39}),
         (12, 'X4', {"weight" : 23})]

map.add_nodes_from(nodes)
map.add_edges_from(edges)

nx.draw(map, nx.get_node_attributes(map, 'pos'), with_labels = True)
nx.draw_networkx_edge_labels(map, nx.get_node_attributes(map, 'pos'), nx.get_edge_attributes(map, "weight"))
plt.show()