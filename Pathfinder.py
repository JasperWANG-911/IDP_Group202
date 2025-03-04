# Pathfinder.py
from graph_classes import graph

# Mapping from cardinal letters to numeric orientation.
direction_map = {'N': 0, 'E': 1, 'S': 2, 'W': 3}

def get_edge_direction(current_node, next_node):
    """
    Return the cardinal direction (letter) from current_node to next_node.
    """
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
    """
    path, distance = graph.dijkstra(current_node, target)
    if path is None or len(path) < 2:
        return current_node
    return path[1]

def compute_turn_type(current_orientation, desired_direction):
    """
    Determine turn type based on orientation difference.
      - 0: straight, 1: right, 3: left, 2: rear.
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
    Return the expected sensor pattern for a given node:
      - Node 1 (start/finish): all sensors off.
      - Marking nodes (X1, X2, X3, X4, RY, BG): all sensors active.
      - Otherwise: None.
    """
    if node == 1:
        return {'left_side': 0, 'center_left': 0, 'center_right': 0, 'right_side': 0}
    if node in ['X1', 'X2', 'X3', 'X4', 'RY', 'BG']:
        return {'left_side': 1, 'center_left': 1, 'center_right': 1, 'right_side': 1}
    return None

def check_node_sensor(sensor_instance, node):
    """
    Compare actual sensor readings with the expected pattern for a node.
    Print a warning if they do not match.
    """
    expected = expected_sensor_pattern(node)
    if expected is None:
        return
    actual = sensor_instance.read_all()
    if actual != expected:
        print("Warning: Sensor mismatch at node", node)
        print("Expected:", expected, "Actual:", actual)
    else:
        print("Sensor readings at node", node, "match expected pattern.")
