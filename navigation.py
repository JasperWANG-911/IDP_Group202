import time
from graph_classes import graph  # Uses our graph structure
import sensor_for_main as sensor  # Sensor functions are provided by sensor_for_main.py
import odometry as odom
from PID_concept import Control  # Import our PID controller

# Predetermined route: after starting at node 1, visit these nodes in order.
TARGET_ROUTE = ['X1', 'X2', 'X3', 'X4', 'RY', 'BG']

# Mapping from cardinal directions (letters) to numeric orientation (0: North, 1: East, 2: South, 3: West)
direction_map = {'N': 0, 'E': 1, 'S': 2, 'W': 3}

def get_edge_direction(current_node, next_node):
    if current_node not in graph.nodes:
        return None
    node_obj = graph.nodes[current_node]
    for neighbor, weight, dir_value in node_obj.adjacent:
        if neighbor.value == next_node:
            return dir_value
    return None

def get_next_node(current_node, target):
    path, distance = graph.dijkstra(current_node, target)
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

def expected_sensor_pattern(node, current_orientation, candidate_turn_type):
    if node == 1:
        return {'front': 0, 'right': 0, 'rear': 0, 'left': 0}
    if node in ['RY', 'BG']:
        return None
    if node in ['X1', 'X2', 'X3', 'X4']:
        return {'front': 1, 'right': 1, 'rear': 1, 'left': 1}
    return None

def check_node_sensor(node, current_orientation, candidate_turn_type):
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
    start_time = time.time()
    stable_time = 0.2
    pattern_stable_start = None
    initial_sensor_data = sensor.get_track_sensor_pattern()
    initial_front = initial_sensor_data.get('front') == 1
    lost_front = False

    while time.time() - start_time < timeout:
        if turn_type == 'left':
            motors.turn_left(duration=increment)
        elif turn_type == 'right':
            motors.turn_right(duration=increment)
        else:
            raise ValueError("Invalid turn_type. Use 'left' or 'right'.")
        sensor_data = sensor.get_track_sensor_pattern()
        current_front = sensor_data.get('front') == 1
        if not initial_front:
            if current_front:
                if pattern_stable_start is None:
                    pattern_stable_start = time.time()
                elif time.time() - pattern_stable_start >= stable_time:
                    print("Turn complete: front path detected.")
                    return
            else:
                pattern_stable_start = None
        else:
            if not lost_front:
                if not current_front:
                    lost_front = True
            else:
                if current_front:
                    if pattern_stable_start is None:
                        pattern_stable_start = time.time()
                    elif time.time() - pattern_stable_start >= stable_time:
                        print("Turn complete: front path re-detected after being lost.")
                        return
                else:
                    pattern_stable_start = None
        time.sleep(increment)
    print("Turn timeout reached without achieving a stable front sensor detection.")

def correct_orientation(motors, pid_control, correction_duration=0.5, threshold=1):
    """
    Use the PID controller to correct the robot's orientation.
    Over a short correction period, read the COM x-error from the PID controller.
    If the error exceeds the threshold:
      - If error > threshold, the robot is too far to the right so turn left.
      - If error < -threshold, the robot is too far to the left so turn right.
    Otherwise, proceed forward in small increments.
    """
    start_time = time.time()
    while time.time() - start_time < correction_duration:
        pos = pid_control.get_pos()
        if pos is None:
            break  # No sensor data available
        error = pid_control.get_error(pos)
        if error > threshold:
            print("PID correction: turning left (error:", error,")")
            motors.turn_left(duration=0.05)
        elif error < -threshold:
            print("PID correction: turning right (error:", error,")")
            motors.turn_right(duration=0.05)
        else:
            # Within threshold; move forward a short bit
            motors.move_forward(duration=0.05)
        time.sleep(0.05)

def run_navigation(motors, odom):
    """
    Main navigation loop using the predetermined route.
    The vehicle begins at node 1 and proceeds to each target in TARGET_ROUTE.
    Now incorporates an orientation correction phase using a PID controller.
    """
    # Instantiate a PID controller using our sensor interface.
    pid_control = Control(sensor)
    
    current_node = 1
    visited = [current_node]
    target_index = 0
    num_targets = len(TARGET_ROUTE)
    
    while target_index < num_targets:
        target = TARGET_ROUTE[target_index]
        if current_node == target:
            print(f"Reached target node: {target}")
            if visited[-1] != target:
                visited.append(target)
            target_index += 1
            continue
        
        next_node = get_next_node(current_node, target)
        print(f"Current node: {current_node}, Next node: {next_node}, Target: {target}")
        
        edge_dir = get_edge_direction(current_node, next_node)
        if edge_dir is None:
            print(f"Could not determine direction from {current_node} to {next_node}. Moving straight.")
            desired_direction = odom.vehicle_orientation
        else:
            desired_direction = direction_map[edge_dir]
        
        turn_type = compute_turn_type(odom.vehicle_orientation, desired_direction)
        print(f"Vehicle orientation: {odom.vehicle_orientation}, Desired: {desired_direction}, Turn: {turn_type}")
        
        check_node_sensor(current_node, odom.vehicle_orientation, turn_type)
        
        if turn_type == 'straight':
            # Instead of a simple forward move, incorporate a PID-based orientation correction.
            correct_orientation(motors, pid_control, correction_duration=0.5, threshold=1)
            motors.move_forward(duration=0.5)
            odom.update_position('front')
        elif turn_type == 'left':
            turn_until_shift(motors, 'left', increment=0.1, timeout=3)
            odom.update_orientation('left')
            motors.move_forward(duration=0.5)
            odom.update_position('front')
        elif turn_type == 'right':
            turn_until_shift(motors, 'right', increment=0.1, timeout=3)
            odom.update_orientation('right')
            motors.move_forward(duration=0.5)
            odom.update_position('front')
        elif turn_type == 'rear':
            motors.move_backward(duration=0.5)
            odom.update_position('rear')
        
        if next_node != current_node:
            current_node = next_node
            if visited[-1] != current_node:
                visited.append(current_node)
        time.sleep(0.1)
    
    print("Navigation complete. All target nodes reached.")
