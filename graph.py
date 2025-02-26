# map.py
graph_nodes_cm = {
    1: (0, 0),
    2: (0, 44),
    3: (-34, 44),
    4: (104, 44),
    5: (-105, 44),
    6: (-105, 131),
    7: (-1, 131),
    8: (33, 131),
    9: (104, 131),
    10: (-1, 168),
    11: (-1, 207),
    12: (38, 207),
    13: (-105, 207),
    14: (104, 207),
    'X1': (-34, 75),
    'X2': (33, 108),
    'X3': (-27, 168),
    'X4': (38, 184),
    'RY': (-105, 0),
    'BG': (104, 0)
}
graph_nodes = {node: (pos[0] * 0.01, pos[1] * 0.01) for node, pos in graph_nodes_cm.items()}

edges = [
    (1, 2), (2, 3), (2, 4), (3, 'X1'), (3, 5),
    (5, 6), (5, 'RY'), (6, 7), (6, 13), (11, 13),
    (11, 14), (9, 14), (7, 10), (11, 10), (10, 'X3'),
    (7, 8), (8, 'X2'), (8, 9), (4, 9), (4, 'BG'),
    (11, 12), (12, 'X4')
]

graph_neighbors = {}
for node in graph_nodes:
    graph_neighbors[node] = set()
for (a, b) in edges:
    if a in graph_nodes and b in graph_nodes:
        graph_neighbors[a].add(b)
        graph_neighbors[b].add(a)
