import heapq

# Constant array to match an integer to cardinal direction
directionArr = ('N', 'E', 'S', 'W')

# Node Class
class Node:
    def __init__(self, value):
        '''
        Initialise each node with attributes:
            Own label
            Neighboring nodes: List of tuples containing (node object, weight, cardinal direction)
        '''
        self.value = value
        self.adjacent = []
        
    def add_neighbor(self, node, weight, dir12, dir21):
        '''
        Connects to nodes by updating Node.adjacent arrays associated with both nodes.
        Args:
            self
            node: Neighboring node (node object)
            weight: Distance to node (int)
            dir12: Cardinal direction from self to node (char)
            dir21: Cardinal direction from node to self (char)
        '''
        if not any(neighbor == node for neighbor, _, _ in self.adjacent):  # Avoid duplicates
            self.adjacent.append((node, weight, dir12))
            node.adjacent.append((self, weight, dir21))  # Ensure undirected connection

    def __repr__(self):
        return f"GraphNode({self.value})"

class Edge:
    def __init__(self, node1, node2, weight, dirInt12):
        '''
        Initialise each node with attributes:
            node1: The first node (node object)
            node2: The second node (node object)
            weight: Distance between the nodes (int)
            dirInt12: Index of node1->node2 cardinal direction from directionArr (int)
        '''
        self.node1 = node1
        self.node2 = node2
        self.weight = weight
        self.dir12 = directionArr[dirInt12]
        self.dir21 = directionArr[(dirInt12 + 2) % 4] # Find the reverse direction
        node1.add_neighbor(node2, weight, self.dir12, self.dir21)  # Connect the nodes
    
    def __repr__(self):
        return f"GraphEdge({self.node1.value}, {self.node2.value}) of Weight({self.weight})"

class Graph:
    def __init__(self):
        '''
        Initialise graph with a dictionary containing {node label: node object}
        '''
        self.nodes = {}

    def add_node(self, value):
        '''
        If a node is not already in the graph, add dictionary term {node label: node object}
        The node object is initialised in-function from node label
        Arg:
            value: Node label (Any basic type)
        '''
        if value not in self.nodes:
            self.nodes[value] = Node(value)

    def add_edge(self, value1, value2, weight, dirInt12):
        '''
        If the two end nodes are both in the graph, initialise an edge object connecting them
        with the required characteristics
        Args:
            value1: Label of node1 (Any basic type)
            value2: Label of node2 (Any basic type)
            weight: Distance between the nodes (int)
            dirInt12: Index of node1->node2 cardinal direction from directionArr (int)
        '''
        if value1 in self.nodes and value2 in self.nodes:
            Edge(self.nodes[value1], self.nodes[value2], weight, dirInt12)

    def add_nodes_from(self, values):
        '''
        Iterable implementation of Graph.add_node()
        Arg:
            values: Array of node labels (iterable containing basic types)
        '''
        for value in values:
            self.add_node(value)
    
    def add_edges_from(self, pairsWeightsDir):
        '''
        Iterable implementation of Graph.add_edge()
        Arg:
            pairsWeightsDir: List of tuples containing (value1, value2, weight, dirInt12) [defined in Graph.add_edge() method]
        '''
        for obj in pairsWeightsDir:
            self.add_edge(obj[0], obj[1], obj[2], obj[3])

    def display(self):
        '''
        Displays list of {node1 label: (node2 label, distance, node1->node2 cardinal direction), ...}
        '''
        for node in self.nodes.values():
            edges = ", ".join(f"({neighbor.value}, {weight}, {dir})" for neighbor, weight, dir in node.adjacent)
            print(f"{node.value}: {edges}")

    def dijkstra(self, start_value, end_value):
        '''
        Implements Dijkstra's algorithm to find the shortest path between any two given nodes
        Args:
            start_value: Start node label
            end_value: End node label
        Returns:
            path: List of node labels along shortest path from start node -> end node inclusive
            distance: Total traversed distance along path
        '''
        # Return None if nodes are not in the graph
        if start_value not in self.nodes or end_value not in self.nodes:
            return None, float('inf') 

        # Find start and end node objects
        start = self.nodes[start_value]
        end = self.nodes[end_value]

        # Min-heap priority queue: (distance, nodeID, node) - Minimum prioritises distance
        # nodeID exists in case two nodes share the same minimum distance
        nodeID = {list(self.nodes.values())[i]: i for i in range(len(self.nodes))}
        pq = [(0, nodeID[start], start)]
        # Initialise dictionary containing {node object: shortest found distance thus far to node}
        distances = {node: float('inf') for node in self.nodes.values()}
        distances[start] = 0
        # Initialise dictionary containing {node object: previous node in shortest path found thus far}
        previous_nodes = {node: None for node in self.nodes.values()}
        
        # Iterate through pq until the destination is reached (break occurs in loop)
        while pq:
            # Extract current_distance and current_node from the node with shortest distance from start
            current_distance, _, current_node = heapq.heappop(pq)

            # Stop early if we reach the destination
            if current_node == end:
                break
            
            # Iterate through adjacent nodes of current_node and calculate distances to those nodes via the current_node
            for neighbor, weight, _ in current_node.adjacent:
                distance = current_distance + weight

                # If a shorter path is found to any of the neighbor nodes, update the shortest distance and previous node
                if distance < distances[neighbor]:
                    distances[neighbor] = distance
                    previous_nodes[neighbor] = current_node
                    # Push this new distance to the priority queue
                    heapq.heappush(pq, (distance, nodeID[neighbor], neighbor))

        # Reconstruct the shortest path
        path = []
        current = end
        # previous_nodes[start] = None causes while loop to break
        while current:
            path.append(current.value)
            current = previous_nodes[current]
        path.reverse()

        return path, distances[end] if distances[end] != float('inf') else None

# Node label array - see map picture
nodes = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 'X1', 'X2', 'X3', 'X4', 'RY','BG']         

# Edges in format (Node1, Node2, Weight, Cardinal Direction)
edges = [(1, 2, 44, 0),
         (2, 3, 34, 3),
         (2, 4, 104, 1),
         (3, 'X1', 31, 0),
         (3, 5, 71, 3),
         (5, 6, 87, 0),
         (5, 'RY', 44, 2),
         (6, 7, 104, 1),
         (6, 13, 76, 0),
         (11, 13, 105, 3),
         (12, 14, 65, 1),
         (7, 10, 37, 0),
         (11, 10, 39, 2),
         (10, 'X3', 26, 3),
         (7, 8, 34, 1),
         (8, 'X2', 23, 2),
         (8, 9, 71, 1),
         (4, 9, 87, 0),
         (4, 'BG', 44, 2),
         (9, 14, 76, 0),
         (11, 12, 39, 1),
         (12, 'X4', 23, 2)]

# Initialising graph to contain IDP map information
graph = Graph()
graph.add_nodes_from(nodes)
graph.add_edges_from(edges)


# ------ Testing ------
#graph.display()
#path, distance = graph.dijkstra(1, 'X1')
#print(f"Shortest path: {path}, Distance: {distance}")