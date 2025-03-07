#!/usr/bin/env python3
"""
Pathfinder Module for IDP Navigation System

This module provides helper functions for:
  - Retrieving the edge direction between nodes.
  - Determining the next node on the shortest path.
  - Computing the turn type based on the current and desired orientations.
  - Checking sensor patterns at a node using only the side sensors.

For node checking:
  - The check is based solely on the side sensors (left_side and right_side).
  - It is accepted as a cross if at least one side sensor is active.
  - The two center sensors are completely ignored.
"""

def get_edge_direction(current_node, next_node):
    """
    Return the cardinal direction (letter) from current_node to next_node.
    Assumes that each node's 'adjacent' list contains tuples of (neighbor, weight, cardinal direction).
    """
    from graph_classes import graph  # Assuming graph_classes.py defines 'graph'
    if current_node not in graph.nodes:
        return None
    node_obj = graph.nodes[current_node]
    for neighbor, weight, dir_value in node_obj.adjacent:
        if neighbor.value == next_node:
            return dir_value
    return None

def get_next_node(current_node, target):
    """
    Use Dijkstra's algorithm (from the graph) to determine the next node along the shortest path.
    
    Args:
        current_node: The current node label.
        target: The target node label.
    
    Returns:
        The label of the next node along the path.
    """
    from graph_classes import graph
    path, distance = graph.dijkstra(current_node, target)
    if path is None or len(path) < 2:
        return current_node
    return path[1]

def compute_turn_type(current_orientation, desired_direction):
    """
    Determine turn type based on orientation difference.
    
    Args:
        current_orientation: Current numeric orientation (0: North, 1: East, 2: South, 3: West).
        desired_direction: Desired numeric orientation.
        
    Returns:
        A string: 'straight', 'right', 'left', or 'rear'.
    """
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

def expected_sensor_pattern(node):
    """
    For node checking, we use only the side sensors.
    Instead of returning a fixed pattern for each node, we now simply
    indicate that for a cross to be detected, at least one side sensor should be active.
    
    Returns:
        A marker value indicating that the check is: "at least one side sensor must be active".
    """
    return "atleast_one"

def check_node_sensor(sensor_instance, node):
    """
    Compare actual sensor readings (using only the side sensors) with the expected condition:
    At least one side sensor must be active.
    
    Args:
        sensor_instance: An instance of LineSensors.
        node: The node label.
    """
    expected = expected_sensor_pattern(node)
    actual = sensor_instance.read_all()
    # Extract only the side sensor readings.
    actual_side = {
        'left_side': actual.get('left_side'),
        'right_side': actual.get('right_side')
    }
    # The condition: at least one side sensor should be active.
    if actual_side['left_side'] == 1 or actual_side['right_side'] == 1:
        print(f"Sensor readings at node {node} are acceptable (at least one side sensor is active).")
    else:
        print(f"Warning: Sensor mismatch at node {node}.")
        print(f"Expected: At least one side sensor active; Actual: {actual_side}")

# For testing purposes, you can uncomment the following block:
# if __name__ == "__main__":
#     from line_sensor import LineSensors
#     sensors = LineSensors()
#     print("Testing sensor check for node 1:")
#     check_node_sensor(sensors, 1)
#     print("Testing sensor check for node X1:")
#     check_node_sensor(sensors, "X1")
