import time
import random
import sensor_for_main as sensor  # Import sensor module to read sensor data

# ---------------- Map Setup ----------------
# Define graph nodes in centimeters and convert them to meters
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

# Define the edges between nodes and build neighbor relationships
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

def get_node_direction(current_node, neighbor):
    """
    Determine the direction from the current node to a neighbor node.
    Returns:
      0 for North, 1 for East, 2 for South, 3 for West, or None if indeterminate.
    """
    current_pos = graph_nodes[current_node]
    neighbor_pos = graph_nodes[neighbor]
    dx = neighbor_pos[0] - current_pos[0]
    dy = neighbor_pos[1] - current_pos[1]
    tol = 1e-6
    if abs(dx) < tol and dy > 0:
        return 0
    elif abs(dy) < tol and dx > 0:
        return 1
    elif abs(dx) < tol and dy < 0:
        return 2
    elif abs(dy) < tol and dx < 0:
        return 3
    return None

def decide_next_node_random(current_node, previous_node, current_orientation):
    """
    Randomly decide the next node to navigate to while avoiding the previous node if possible.
    Returns a tuple: (candidate node, new orientation, turn type)
    """
    neighbors = list(graph_neighbors[current_node])
    if previous_node is not None and len(neighbors) > 1:
        if previous_node in neighbors:
            neighbors.remove(previous_node)
    candidate = random.choice(neighbors) if neighbors else previous_node
    candidate_direction = get_node_direction(current_node, candidate)
    if candidate_direction is None:
        turn_type = 'straight'
    else:
        diff = (candidate_direction - current_orientation) % 4
        if diff == 0:
            turn_type = 'straight'
        elif diff == 1:
            turn_type = 'right'
        elif diff == 3:
            turn_type = 'left'
        elif diff == 2:
            turn_type = 'rear'
        else:
            turn_type = 'straight'
    new_orientation = candidate_direction if candidate_direction is not None else current_orientation
    return candidate, new_orientation, turn_type

def expected_sensor_pattern(node, current_orientation, candidate_turn_type):
    """
    Determine the expected sensor pattern at a given node based on orientation and turn decision.
    Returns a dictionary with expected sensor values or None if no check is needed.
    """
    if node == 1:
        return {'front': 0, 'right': 0, 'rear': 0, 'left': 0}
    if node in ['RY', 'BG']:
        return None
    if node in ['X1', 'X2', 'X3', 'X4']:
        return {'front': 1, 'right': 1, 'rear': 1, 'left': 1}
    n_neighbors = len(graph_neighbors[node])
    mapping = {0: 'front', 1: 'right', 2: 'rear', 3: 'left'}
    if n_neighbors == 1:
        expected = {'front': 0, 'right': 0, 'rear': 0, 'left': 0}
        expected[mapping[current_orientation]] = 1
        return expected
    elif n_neighbors == 2:
        if candidate_turn_type == 'straight':
            return None
        rear_sensor = mapping[(current_orientation + 2) % 4]
        if candidate_turn_type == 'left':
            side_sensor = 'left'
        elif candidate_turn_type == 'right':
            side_sensor = 'right'
        else:
            side_sensor = mapping[current_orientation]
        expected = {'front': 0, 'right': 0, 'rear': 0, 'left': 0}
        expected[rear_sensor] = 1
        expected[side_sensor] = 1
        return expected
    else:
        return {'front': 1, 'right': 1, 'rear': 1, 'left': 1}

def check_node_sensor(node, current_orientation, candidate_turn_type):
    """
    Compare the actual sensor readings to the expected pattern at a node.
    """
    expected = expected_sensor_pattern(node, current_orientation, candidate_turn_type)
    if expected is None:
        return
    actual = sensor.get_track_sensor_pattern()
    if actual != expected:
        print("Warning: Sensor mismatch at node", node)
        print("Expected:", expected, "Actual:", actual)
    else:
        print("Sensor readings at node", node, "match expected pattern.")

def turn_until_shift(motors, turn_type, increment=0.1, timeout=3):
    """
    Turn in small increments until a 90° shift is detected based on sensor readings.
    This function uses the MotorPair instance and checks sensor data.
    """
    start_time = time.time()
    consecutive_count = 0
    while time.time() - start_time < timeout:
        if turn_type == 'left':
            motors.turn_left(duration=increment)
        elif turn_type == 'right':
            motors.turn_right(duration=increment)
        sensor_data = sensor.get_track_sensor_pattern()
        if turn_type == 'left':
            if sensor_data['left'] == 1 and sensor_data['front'] == 0:
                consecutive_count += 1
            else:
                consecutive_count = 0
        elif turn_type == 'right':
            if sensor_data['right'] == 1 and sensor_data['front'] == 0:
                consecutive_count += 1
            else:
                consecutive_count = 0
        if consecutive_count >= 3:
            print(f"{turn_type.capitalize()} turn complete based on sensor shift.")
            return
    print("Turn timeout reached without clear sensor shift.")
