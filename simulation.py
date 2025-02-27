#!/usr/bin/env python3
import time
import heapq
import math
import matplotlib.pyplot as plt

# -----------------------------
# Map Data (from map.py)
# -----------------------------
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
# Convert coordinates to meters
graph_nodes = {node: (pos[0] * 0.01, pos[1] * 0.01) for node, pos in graph_nodes_cm.items()}

edges_list = [
    (1, 2), (2, 3), (2, 4), (3, 'X1'), (3, 5),
    (5, 6), (5, 'RY'), (6, 7), (6, 13), (11, 13),
    (11, 14), (9, 14), (7, 10), (11, 10), (10, 'X3'),
    (7, 8), (8, 'X2'), (8, 9), (4, 9), (4, 'BG'),
    (11, 12), (12, 'X4')
]

# -----------------------------
# Helper Functions for Map
# -----------------------------
def compute_weight(a, b):
    """Compute Euclidean distance between nodes a and b using graph_nodes."""
    x1, y1 = graph_nodes[a]
    x2, y2 = graph_nodes[b]
    return math.hypot(x2 - x1, y2 - y1)

def compute_cardinal_direction(a, b):
    """
    Compute a rough cardinal direction from node a to node b.
    Returns an index: 0: North, 1: East, 2: South, 3: West.
    This is based on the dominant difference.
    """
    x1, y1 = graph_nodes[a]
    x2, y2 = graph_nodes[b]
    dx = x2 - x1
    dy = y2 - y1
    if abs(dx) >= abs(dy):
        return 1 if dx > 0 else 3  # East or West
    else:
        return 0 if dy > 0 else 2  # North or South

# -----------------------------
# Graph, Node, and Dijkstra (adapted)
# -----------------------------
directionArr = ['N', 'E', 'S', 'W']

class Node:
    def __init__(self, value):
        self.value = value
        self.adjacent = []  # List of tuples: (neighbor, weight, direction)
        
    def add_neighbor(self, node, weight, dir12, dir21):
        # Avoid duplicate connections
        if not any(neighbor == node for neighbor, _, _ in self.adjacent):
            self.adjacent.append((node, weight, dir12))
            node.adjacent.append((self, weight, dir21))
            
    def __repr__(self):
        return f"GraphNode({self.value})"

class Edge:
    def __init__(self, node1, node2, weight, dirInt12):
        self.node1 = node1
        self.node2 = node2
        self.weight = weight
        self.dir12 = directionArr[dirInt12]
        self.dir21 = directionArr[(dirInt12 + 2) % 4]
        node1.add_neighbor(node2, weight, self.dir12, self.dir21)
    
    def __repr__(self):
        return f"GraphEdge({self.node1.value}, {self.node2.value}) weight {self.weight}"

class Graph:
    def __init__(self):
        self.nodes = {}
        
    def add_node(self, value):
        if value not in self.nodes:
            self.nodes[value] = Node(value)
            
    def add_edge(self, value1, value2, weight, dirInt12):
        if value1 in self.nodes and value2 in self.nodes:
            Edge(self.nodes[value1], self.nodes[value2], weight, dirInt12)
            
    def add_nodes_from(self, values):
        for value in values:
            self.add_node(value)
    
    def add_edges_from(self, edge_list):
        for a, b in edge_list:
            if a in graph_nodes and b in graph_nodes:
                weight = compute_weight(a, b)
                dirInt12 = compute_cardinal_direction(a, b)
                self.add_edge(a, b, weight, dirInt12)
    
    def display(self):
        for node in self.nodes.values():
            edges = ", ".join(f"({neighbor.value}, {weight:.2f}, {dir})" for neighbor, weight, dir in node.adjacent)
            print(f"{node.value}: {edges}")
            
    def dijkstra(self, start_value, end_value):
        if start_value not in self.nodes or end_value not in self.nodes:
            return None, float('inf')
        start = self.nodes[start_value]
        end = self.nodes[end_value]
        nodeID = {node: i for i, node in enumerate(self.nodes.values())}
        pq = [(0, nodeID[start], start)]
        distances = {node: float('inf') for node in self.nodes.values()}
        previous_nodes = {node: None for node in self.nodes.values()}
        distances[start] = 0
        
        while pq:
            current_distance, _, current_node = heapq.heappop(pq)
            if current_node == end:
                break
            for neighbor, weight, _ in current_node.adjacent:
                distance = current_distance + weight
                if distance < distances[neighbor]:
                    distances[neighbor] = distance
                    previous_nodes[neighbor] = current_node
                    heapq.heappush(pq, (distance, nodeID[neighbor], neighbor))
                    
        path = []
        current = end
        while current:
            path.append(current.value)
            current = previous_nodes[current]
        path.reverse()
        return path, distances[end] if distances[end] != float('inf') else None

# Build the graph using our map nodes and edges.
g = Graph()
g.add_nodes_from(graph_nodes.keys())
g.add_edges_from(edges_list)

# -----------------------------
# Fake Sensor Implementation
# -----------------------------
class FakeSensors:
    """
    Fake sensor class that always returns the ideal sensor patterns.
    When turning, we simulate perfect detection.
    """
    def __init__(self):
        self.mode = 'straight'
    
    def set_mode(self, mode):
        self.mode = mode

    def read_track_sensor(self):
        if self.mode == 'left_turn':
            return {'left': 1, 'front': 0, 'right': 0, 'rear': 0}
        elif self.mode == 'right_turn':
            return {'right': 1, 'front': 0, 'left': 0, 'rear': 0}
        else:
            return {'front': 1, 'left': 0, 'right': 0, 'rear': 0}

    def read_color_sensor(self):
        return {'color': 'none'}

    def read_ultrasonic_sensor(self):
        return {'distance': 100}

fake_sensor_instance = FakeSensors()

def get_track_sensor_pattern():
    return fake_sensor_instance.read_track_sensor()

# -----------------------------
# Odometry
# -----------------------------
DISTANCE_PER_MOVE = 0.22  # Not used in plotting, but for simulation

class Odometry:
    def __init__(self):
        self.current_position = (0.0, 0.0)
        self.vehicle_orientation = 0  # 0: North, 1: East, 2: South, 3: West
    
    def get_new_position(self, position, orientation, distance=DISTANCE_PER_MOVE):
        x, y = position
        if orientation == 0:
            return (x, y + distance)
        elif orientation == 1:
            return (x + distance, y)
        elif orientation == 2:
            return (x, y - distance)
        elif orientation == 3:
            return (x - distance, y)
        return position
    
    def update_position(self, move_type):
        if move_type == 'front':
            self.current_position = self.get_new_position(self.current_position, self.vehicle_orientation)
        elif move_type == 'rear':
            back_orientation = (self.vehicle_orientation + 2) % 4
            self.current_position = self.get_new_position(self.current_position, back_orientation)
        else:
            raise ValueError("Invalid move type. Use 'front' or 'rear'.")
    
    def update_orientation(self, turn_direction):
        if turn_direction == 'left':
            self.vehicle_orientation = (self.vehicle_orientation - 1) % 4
        elif turn_direction == 'right':
            self.vehicle_orientation = (self.vehicle_orientation + 1) % 4
        else:
            raise ValueError("Invalid turn direction. Use 'left' or 'right'.")
    
    def __str__(self):
        return f"Position: {self.current_position}, Orientation: {self.vehicle_orientation}"

# -----------------------------
# Fake Motor Implementation
# -----------------------------
class FakeMotor:
    def __init__(self, id):
        self.id = id
    
    def Forward(self):
        print(f"Motor {self.id}: Forward")
    
    def Reverse(self):
        print(f"Motor {self.id}: Reverse")
    
    def off(self):
        print(f"Motor {self.id}: Off")

class FakeMotorPair:
    def __init__(self):
        self.left = FakeMotor("Left")
        self.right = FakeMotor("Right")
    
    def move_forward(self, duration=0.5):
        print("MotorPair: move_forward")
        self.left.Forward()
        self.right.Forward()
        time.sleep(duration)
        self.left.off()
        self.right.off()
    
    def move_backward(self, duration=0.5):
        print("MotorPair: move_backward")
        self.left.Reverse()
        self.right.Reverse()
        time.sleep(duration)
        self.left.off()
        self.right.off()
    
    def turn_left(self, duration=0.5):
        print("MotorPair: turn_left")
        self.left.Reverse()
        self.right.Forward()
        time.sleep(duration)
        self.left.off()
        self.right.off()
    
    def turn_right(self, duration=0.5):
        print("MotorPair: turn_right")
        self.left.Forward()
        self.right.Reverse()
        time.sleep(duration)
        self.left.off()
        self.right.off()

# -----------------------------
# Navigation
# -----------------------------
TARGET_ROUTE = ['X1', 'X2', 'X3', 'X4', 'RY', 'BG']
direction_map = {'N': 0, 'E': 1, 'S': 2, 'W': 3}

def get_edge_direction(current_node, next_node):
    if current_node not in g.nodes:
        return None
    node_obj = g.nodes[current_node]
    for neighbor, weight, dir_value in node_obj.adjacent:
        if neighbor.value == next_node:
            return dir_value
    return None

def get_next_node(current_node, target):
    path, distance = g.dijkstra(current_node, target)
    if path is None or len(path) < 2:
        return current_node
    return path[1]

def compute_turn_type(current_orientation, desired_direction):
    diff = (desired_direction - current_orientation) % 4
    if diff == 0:
        return 'straight'
    elif diff == 1:
        return 'right'
    elif diff == 3:
        return 'left'
    elif diff == 2:
        return 'rear'
    return 'straight'

def check_node_sensor(node, current_orientation, candidate_turn_type):
    # For simulation, assume sensors always match perfectly.
    print(f"Sensors at node {node} are as expected.")

def turn_until_shift(motors, turn_type, increment=0.1, timeout=3):
    start_time = time.time()
    stable_time = 0.2
    pattern_stable_start = None

    if turn_type == 'left':
        fake_sensor_instance.set_mode('left_turn')
    elif turn_type == 'right':
        fake_sensor_instance.set_mode('right_turn')

    while time.time() - start_time < timeout:
        if turn_type == 'left':
            motors.turn_left(duration=increment)
        elif turn_type == 'right':
            motors.turn_right(duration=increment)
        else:
            raise ValueError("Invalid turn_type. Use 'left' or 'right'.")

        sensor_data = get_track_sensor_pattern()
        if turn_type == 'left':
            expected = sensor_data.get('left') == 1 and sensor_data.get('front') == 0
        else:
            expected = sensor_data.get('right') == 1 and sensor_data.get('front') == 0

        if expected:
            if pattern_stable_start is None:
                pattern_stable_start = time.time()
            elif time.time() - pattern_stable_start >= stable_time:
                print(f"{turn_type.capitalize()} turn complete based on stable sensor pattern.")
                fake_sensor_instance.set_mode('straight')
                return
        else:
            pattern_stable_start = None

        time.sleep(increment)
    print("Turn timeout reached without achieving a stable sensor pattern.")
    fake_sensor_instance.set_mode('straight')

def run_navigation(motors, odom):
    visited = [1]  # start at node 1
    current_node = 1
    target_index = 0
    num_targets = len(TARGET_ROUTE)
    
    while target_index < num_targets:
        target = TARGET_ROUTE[target_index]
        if current_node == target:
            print(f"Reached target node: {target}")
            visited.append(target)
            target_index += 1
            continue
        
        next_node = get_next_node(current_node, target)
        print(f"Current: {current_node}, Next: {next_node}, Target: {target}")
        edge_dir = get_edge_direction(current_node, next_node)
        if edge_dir is None:
            desired_direction = odom.vehicle_orientation
        else:
            desired_direction = direction_map[edge_dir]
        
        turn_type = compute_turn_type(odom.vehicle_orientation, desired_direction)
        print(f"Orientation: {odom.vehicle_orientation}, Desired: {desired_direction}, Turn: {turn_type}")
        check_node_sensor(current_node, odom.vehicle_orientation, turn_type)
        
        if turn_type == 'straight':
            motors.move_forward(duration=0.5)
        elif turn_type == 'left':
            turn_until_shift(motors, 'left', increment=0.1, timeout=3)
            odom.update_orientation('left')
            motors.move_forward(duration=0.5)
        elif turn_type == 'right':
            turn_until_shift(motors, 'right', increment=0.1, timeout=3)
            odom.update_orientation('right')
            motors.move_forward(duration=0.5)
        elif turn_type == 'rear':
            motors.move_backward(duration=0.5)
        # (For simulation, we skip odometry updates on position.)
        current_node = next_node
        visited.append(current_node)
        time.sleep(0.1)
    
    # Return to node 1 if not already there.
    if current_node != 1:
        print("Returning to node 1...")
        while current_node != 1:
            next_node = get_next_node(current_node, 1)
            print(f"Return: Current: {current_node}, Next: {next_node}")
            edge_dir = get_edge_direction(current_node, next_node)
            if edge_dir is None:
                desired_direction = odom.vehicle_orientation
            else:
                desired_direction = direction_map[edge_dir]
            turn_type = compute_turn_type(odom.vehicle_orientation, desired_direction)
            if turn_type == 'straight':
                motors.move_forward(duration=0.5)
            elif turn_type == 'left':
                turn_until_shift(motors, 'left', increment=0.1, timeout=3)
                odom.update_orientation('left')
                motors.move_forward(duration=0.5)
            elif turn_type == 'right':
                turn_until_shift(motors, 'right', increment=0.1, timeout=3)
                odom.update_orientation('right')
                motors.move_forward(duration=0.5)
            elif turn_type == 'rear':
                motors.move_backward(duration=0.5)
            current_node = next_node
            visited.append(current_node)
            time.sleep(0.1)
    
    print("Navigation complete. Route visited:", visited)
    return visited

# -----------------------------
# Main Simulation and Plotting
# -----------------------------
def main():
    print("Starting simulation with fake hardware and map.")
    motors = FakeMotorPair()
    odom = Odometry()
    
    # Display the graph (map)
    print("Graph:")
    g.display()
    path, dist = g.dijkstra(1, 'X1')
    print(f"Example path from 1 to X1: {path}, Distance: {dist:.2f}")
    
    # Run navigation simulation (visit targets and return to 1)
    visited_nodes = run_navigation(motors, odom)
    print("Visited nodes:", visited_nodes)
    
    # Plot the graph (all edges) and the visited route.
    plt.figure(figsize=(8, 6))
    
    # Plot all nodes and edges
    for a, b in edges_list:
        if a in graph_nodes and b in graph_nodes:
            x1, y1 = graph_nodes[a]
            x2, y2 = graph_nodes[b]
            plt.plot([x1, x2], [y1, y2], color='lightgray', linestyle='--', zorder=1)
    for node, (x, y) in graph_nodes.items():
        plt.plot(x, y, 'ko', markersize=6)
        plt.text(x, y, str(node), fontsize=10, ha='right', va='bottom')
    
    # Plot visited route
    route_x = []
    route_y = []
    for node in visited_nodes:
        if node in graph_nodes:
            x, y = graph_nodes[node]
            route_x.append(x)
            route_y.append(y)
    plt.plot(route_x, route_y, 'r-', marker='o', markersize=8, label='Route', zorder=2)
    
    plt.title("Simulated Navigation Route")
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.legend()
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    main()
