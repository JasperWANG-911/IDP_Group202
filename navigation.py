import time
from graph_classes import graph  # Use the new graph structure :contentReference[oaicite:0]{index=0}
import sensor_for_main as sensor  # Sensor functions are provided by sensor_for_main.py
import odometry as odom

# Predetermined route: after starting at node 1, visit these nodes in order.
TARGET_ROUTE = ['X1', 'X2', 'X3', 'X4', 'RY', 'BG']

# Mapping from cardinal directions (letters) to numeric orientation (0: North, 1: East, 2: South, 3: West)
direction_map = {'N': 0, 'E': 1, 'S': 2, 'W': 3}

def get_edge_direction(current_node, next_node):
    """
    Retrieve the cardinal direction (as a letter) from current_node to next_node
    based on the graph edge information.
    Returns one of 'N', 'E', 'S', or 'W' or None if not found.
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
    Compute the shortest path from current_node to target using Dijkstra's algorithm,
    then return the immediate next node on that path.
    """
    path, distance = graph.dijkstra(current_node, target)
    if path is None or len(path) < 2:
        # Already at target or no path found
        return current_node
    # The path list begins with current_node; return the second element.
    return path[1]

def compute_turn_type(current_orientation, desired_direction):
    """
    Compare the vehicle's current numeric orientation with the desired direction (numeric)
    and determine the turn type needed.
    Returns one of: 'straight', 'left', 'right', or 'rear'.
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

def expected_sensor_pattern(node, current_orientation, candidate_turn_type):
    """
    Return the expected track sensor pattern for a given node.
    For simplicity, this function assumes:
      - Node 1: no sensor is active.
      - Special nodes (X1, X2, X3, X4): all sensors active.
      - For nodes 'RY' and 'BG', no sensor check is applied.
    Modify as needed for your hardware.
    """
    if node == 1:
        return {'front': 0, 'right': 0, 'rear': 0, 'left': 0}
    if node in ['RY', 'BG']:
        return None
    if node in ['X1', 'X2', 'X3', 'X4']:
        return {'front': 1, 'right': 1, 'rear': 1, 'left': 1}
    return None

def check_node_sensor(node, current_orientation, candidate_turn_type):
    """
    Compare the actual sensor readings with the expected pattern.
    Prints a warning if there is a mismatch.
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
    Turn in small increments until the front path is stably detected.
    If the front sensor is active at the start, the robot will turn until the
    front sensor reading is lost and then re-detected stably.

    Parameters:
      - motors: MotorPair instance controlling the robot.
      - turn_type: 'left' or 'right' indicating the direction to turn.
      - increment: Duration (in seconds) for each incremental motor command (used as the polling interval).
      - timeout: Maximum time (in seconds) to attempt the turn before giving up.
    """
    import time
    start_time = time.time()
    stable_time = 0.2  # Duration the sensor pattern must remain stable
    pattern_stable_start = None

    # Get the initial sensor reading for the front sensor
    initial_sensor_data = sensor.get_track_sensor_pattern()
    initial_front = initial_sensor_data.get('front') == 1

    # This flag indicates whether the front sensor has been lost (only needed if it was initially detected)
    lost_front = False

    while time.time() - start_time < timeout:
        # Execute a small incremental turn based on the requested direction.
        if turn_type == 'left':
            motors.turn_left(duration=increment)
        elif turn_type == 'right':
            motors.turn_right(duration=increment)
        else:
            raise ValueError("Invalid turn_type. Use 'left' or 'right'.")

        # Read the current sensor pattern after the incremental turn.
        sensor_data = sensor.get_track_sensor_pattern()
        current_front = sensor_data.get('front') == 1

        # If the front sensor was not detected initially, complete turn when front is stably detected.
        if not initial_front:
            if current_front:
                if pattern_stable_start is None:
                    pattern_stable_start = time.time()
                elif time.time() - pattern_stable_start >= stable_time:
                    print("Turn complete: front path detected.")
                    return
            else:
                # Reset the stability timer if the front sensor reading is lost.
                pattern_stable_start = None
        else:
            # If the front sensor was detected initially, first wait for it to be lost.
            if not lost_front:
                if not current_front:
                    lost_front = True  # Front sensor is lost, now waiting for re-detection.
            else:
                # After front sensor is lost, wait for a stable re-detection.
                if current_front:
                    if pattern_stable_start is None:
                        pattern_stable_start = time.time()
                    elif time.time() - pattern_stable_start >= stable_time:
                        print("Turn complete: front path re-detected after being lost.")
                        return
                else:
                    # Reset the stability timer if the front sensor reading is lost again.
                    pattern_stable_start = None

        time.sleep(increment)

    print("Turn timeout reached without achieving a stable front sensor detection.")



def run_navigation(motors, odom):
    """
    Main navigation loop using the predetermined route.
    'motors' is an instance of MotorPair.
    'odom' is an instance managing odometry (vehicle position and orientation).
    The vehicle begins at node 1 and proceeds to each target in TARGET_ROUTE.
    """
    current_node = 1
    target_index = 0
    num_targets = len(TARGET_ROUTE)
    
    while target_index < num_targets:
        target = TARGET_ROUTE[target_index]
        
        # If the current node is the target, update to the next target.
        if current_node == target:
            print(f"Reached target node: {target}")
            target_index += 1
            continue
        
        # Compute the next node on the shortest path from current_node to target.
        next_node = get_next_node(current_node, target)
        print(f"Current node: {current_node}, Next node: {next_node}, Target: {target}")
        
        # Determine desired turn direction from graph edge info.
        edge_dir = get_edge_direction(current_node, next_node)
        if edge_dir is None:
            print(f"Could not determine direction from {current_node} to {next_node}. Moving straight.")
            desired_direction = odom.vehicle_orientation
        else:
            desired_direction = direction_map[edge_dir]
        
        # Compute the required turn type.
        turn_type = compute_turn_type(odom.vehicle_orientation, desired_direction)
        print(f"Vehicle orientation: {odom.vehicle_orientation}, Desired direction: {desired_direction}, Turn: {turn_type}")
        
        # Perform sensor check at the node.
        check_node_sensor(current_node, odom.vehicle_orientation, turn_type)
        
        # Execute movement based on the turn type.
        if turn_type == 'straight':
            motors.move_forward(duration=0.5)
            odom.update_position('front')
        elif turn_type == 'left':
            turn_until_shift(motors, 'left', increment=0.1, timeout=3)
            # Update orientation immediately after turn, before moving forward.
            odom.update_orientation('left')
            motors.move_forward(duration=0.5)
            odom.update_position('front')
        elif turn_type == 'right':
            turn_until_shift(motors, 'right', increment=0.1, timeout=3)
            # Update orientation immediately after turn, before moving forward.
            odom.update_orientation('right')
            motors.move_forward(duration=0.5)
            odom.update_position('front')
        elif turn_type == 'rear':
            motors.move_backward(duration=0.5)
            odom.update_position('rear')
        
        # Update the current node after executing movement.
        current_node = next_node
        time.sleep(0.1)
    
    print("Navigation complete. All target nodes reached.")
