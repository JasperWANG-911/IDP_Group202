#!/usr/bin/env python3
"""
simulation.py

This simulation script runs a navigation routine purely in software.
It simulates the navigation logic (deciding when to turn, move forward, or reverse)
and logs the actions taken at each node, assuming perfect execution.
Finally, it plots the entire sequence of visited nodes (including repeats).
"""

import sys
import os
import time
import math
import matplotlib.pyplot as plt

# Add the parent directory to sys.path so that we can import our modules.
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

# Import required functions from our modules.
from Pathfinder import get_edge_direction, get_next_node, compute_turn_type

# -----------------------------
# Map Data and Graph Construction
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
# Convert centimeters to meters.
graph_nodes = {node: (pos[0]*0.01, pos[1]*0.01) for node, pos in graph_nodes_cm.items()}

edges_list = [
    (1, 2), (2, 3), (2, 4), (3, 'X1'), (3, 5),
    (5, 6), (5, 'RY'), (6, 7), (6, 13), (11, 13),
    (11, 14), (9, 14), (7, 10), (11, 10), (10, 'X3'),
    (7, 8), (8, 'X2'), (8, 9), (4, 9), (4, 'BG'),
    (11, 12), (12, 'X4')
]

def compute_weight(a, b):
    x1, y1 = graph_nodes[a]
    x2, y2 = graph_nodes[b]
    return math.hypot(x2 - x1, y2 - y1)

def compute_cardinal_direction(a, b):
    x1, y1 = graph_nodes[a]
    x2, y2 = graph_nodes[b]
    dx = x2 - x1
    dy = y2 - y1
    if abs(dx) >= abs(dy):
        return 1 if dx > 0 else 3
    else:
        return 0 if dy > 0 else 2

# -----------------------------
# Graph, Node, and Dijkstra (Simulation Graph)
# -----------------------------
directionArr = ['N', 'E', 'S', 'W']

class Node:
    def __init__(self, value):
        self.value = value
        self.adjacent = []  # List of tuples: (neighbor, weight, direction)
    def add_neighbor(self, node, weight, dir12, dir21):
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
        self.dir21 = directionArr[(dirInt12+2)%4]
        node1.add_neighbor(node2, weight, self.dir12, self.dir21)
    def __repr__(self):
        return f"GraphEdge({self.node1.value}, {self.node2.value}) weight {self.weight:.2f}"

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
        import heapq
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

# Build the simulation graph.
g = Graph()
g.add_nodes_from(graph_nodes.keys())
g.add_edges_from(edges_list)

# -----------------------------
# Simulated Navigation Logic
# -----------------------------
TARGET_ROUTE = ['X1', 'X2', 'X3', 'X4', 'RY', 'BG']

def simulate_navigation():
    """
    Simulate navigation logic:
      - Starting at node 1 with an assumed initial orientation (0: North).
      - For each target, compute the next node and required turn.
      - Log the action taken and update the simulated orientation.
      - If at a marking node, log a pause and reverse maneuver.
      - Finally, simulate returning to node 1.
    Returns:
      route_nodes (list): Full sequence of visited nodes (including duplicates).
      action_log (list): Log of text describing each navigation action.
    """
    current_node = 1
    current_orientation = 0  # 0: North, 1: East, 2: South, 3: West
    route_nodes = [current_node]  # We'll store *all* visits, even duplicates
    action_log = []
    
    target_index = 0
    num_targets = len(TARGET_ROUTE)
    
    while target_index < num_targets:
        target = TARGET_ROUTE[target_index]
        if current_node == target:
            action_log.append(f"Reached target node: {target}")
            target_index += 1
            continue
        
        next_node = get_next_node(current_node, target)
        action_log.append(f"Transition: {current_node} -> {next_node}")
        
        edge_dir = get_edge_direction(current_node, next_node)
        if edge_dir is None:
            desired_direction = current_orientation
        else:
            mapping = {'N': 0, 'E': 1, 'S': 2, 'W': 3}
            desired_direction = mapping.get(edge_dir, current_orientation)
        
        turn_type = compute_turn_type(current_orientation, desired_direction)
        action_log.append(f"At node {current_node}: current_orientation={current_orientation}, desired_direction={desired_direction}, computed turn: {turn_type}")
        
        # Simulate action based on turn type.
        if turn_type == 'straight':
            action_log.append("Action: Move forward")
        elif turn_type == 'left':
            action_log.append("Action: Turn left")
            current_orientation = (current_orientation - 1) % 4
        elif turn_type == 'right':
            action_log.append("Action: Turn right")
            current_orientation = (current_orientation + 1) % 4
        elif turn_type == 'rear':
            action_log.append("Action: Move backward")
        
        action_log.append(f"Arrived at node {next_node}")
        current_node = next_node
        # Always append, even if repeated:
        route_nodes.append(current_node)
        
        # For marking nodes, simulate a pause and reverse maneuver.
        if current_node in ['X1', 'X2', 'X3', 'X4', 'RY', 'BG']:
            action_log.append(f"At marking node {current_node}: Pause for 3 seconds")
            action_log.append("Execute reverse maneuver to leave marking node")
        
        time.sleep(0.05)
     
    # Simulate returning to node 1 if necessary.
    if current_node != 1:
        action_log.append("Returning to node 1")
        while current_node != 1:
            next_node = get_next_node(current_node, 1)
            edge_dir = get_edge_direction(current_node, next_node)
            if edge_dir is None:
                desired_direction = current_orientation
            else:
                mapping = {'N': 0, 'E': 1, 'S': 2, 'W': 3}
                desired_direction = mapping.get(edge_dir, current_orientation)
            turn_type = compute_turn_type(current_orientation, desired_direction)
            action_log.append(f"Return transition: {current_node} -> {next_node} with turn {turn_type}")
            if turn_type == 'straight':
                action_log.append("Action: Move forward")
            elif turn_type == 'left':
                action_log.append("Action: Turn left")
                current_orientation = (current_orientation - 1) % 4
            elif turn_type == 'right':
                action_log.append("Action: Turn right")
                current_orientation = (current_orientation + 1) % 4
            elif turn_type == 'rear':
                action_log.append("Action: Move backward")
            action_log.append(f"Arrived at node {next_node}")
            current_node = next_node
            route_nodes.append(current_node)  # Add repeated visits too
            time.sleep(0.05)
        action_log.append("Arrived at finish node 1, stopping.")
    
    return route_nodes, action_log

# -----------------------------
# Plotting the Route
# -----------------------------
def plot_route(route_nodes):
    """
    Plots the full route sequence (including repeated visits).
    """
    plt.figure(figsize=(8,6))
    # Plot all edges in light gray.
    for a, b in edges_list:
        if a in graph_nodes and b in graph_nodes:
            x1, y1 = graph_nodes[a]
            x2, y2 = graph_nodes[b]
            plt.plot([x1, x2], [y1, y2], color='lightgray', linestyle='--', zorder=1)
    # Plot nodes.
    for node, (x, y) in graph_nodes.items():
        plt.plot(x, y, 'ko', markersize=6)
        plt.text(x, y, str(node), fontsize=10, ha='right', va='bottom')
    
    # Now plot the entire route with duplicates in the order visited.
    route_x = []
    route_y = []
    for node in route_nodes:
        if node in graph_nodes:
            x, y = graph_nodes[node]
            route_x.append(x)
            route_y.append(y)
    plt.plot(route_x, route_y, 'r-', marker='o', markersize=8, label='Route', zorder=2)
    
    plt.title("Simulated Navigation Route (Full Sequence)")
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.legend()
    plt.grid(True)
    plt.show()

# -----------------------------
# Main Function
# -----------------------------
def main():
    print("Simulating navigation logic with full repeated route tracking...")
    route_nodes, action_log = simulate_navigation()
    
    print("\nAction Log:")
    for action in action_log:
        print(action)
    
    print("\nFull Route Sequence (including repeats):")
    print(route_nodes)
    
    plot_route(route_nodes)

if __name__ == "__main__":
    main()
