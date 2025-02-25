import machine
import time
import math
import random
from motor import Motor  # Import the Motor class from motor.py

# ---------------- Motor Pair Class ----------------
class MotorPair:
    def __init__(self, left_motor, right_motor):
        self.left = left_motor
        self.right = right_motor

    def move_forward(self, speed=60, duration=0.5):
        """
        Drive both motors forward for the given duration.
        Speed is given as a percentage (0-100).
        """
        self.left.forward(speed)
        self.right.forward(speed)
        time.sleep(duration)
        self.left.off()
        self.right.off()

    def move_backward(self, speed=60, duration=0.5):
        """
        Drive both motors in reverse for the given duration.
        Speed is given as a percentage (0-100).
        """
        self.left.reverse(speed)
        self.right.reverse(speed)
        time.sleep(duration)
        self.left.off()
        self.right.off()

    def turn_left(self, speed=60, duration=0.5):
        """
        Turn in place to the left by stopping the left motor
        and driving the right motor forward.
        """
        self.left.off()
        self.right.forward(speed)
        time.sleep(duration)
        self.left.off()
        self.right.off()

    def turn_right(self, speed=60, duration=0.5):
        """
        Turn in place to the right by driving the left motor forward
        and stopping the right motor.
        """
        self.left.forward(speed)
        self.right.off()
        time.sleep(duration)
        self.left.off()
        self.right.off()

# ---------------- Global Variables for Navigation ----------------
# Current position in meters (updated by odometry)
current_position = (0.0, 0.0)
# Current node (starting at node 1)
current_node = 1
# Previous node (None at the start)
previous_node = None
# Vehicle orientation: 0 = North (+y), 1 = East (+x), 2 = South (-y), 3 = West (-x)
vehicle_orientation = 0

# ---------------- Map Setup ----------------
# Define map nodes with positions in centimeters (as provided)
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
# Convert positions from centimeters to meters for consistency
graph_nodes = {node: (pos[0] * 0.01, pos[1] * 0.01) for node, pos in graph_nodes_cm.items()}

# Define graph edges (neighbors) as undirected connections.
edges = [
    (1, 2),
    (2, 3),
    (2, 4),
    (3, 'X1'),
    (3, 5),
    (5, 6),
    (5, 'RY'),
    (6, 7),
    (6, 13),
    (11, 13),
    (11, 14),
    (9, 14),
    (7, 10),
    (11, 10),
    (10, 'X3'),
    (7, 8),
    (8, 'X2'),
    (8, 9),
    (4, 9),
    (4, 'BG'),
    (11, 12),
    (12, 'X4')
]
# Build a mapping from each node to its set of neighbors.
graph_neighbors = {}
for node in graph_nodes:
    graph_neighbors[node] = set()
for (a, b) in edges:
    if a in graph_nodes and b in graph_nodes:
        graph_neighbors[a].add(b)
        graph_neighbors[b].add(a)

# ---------------- Motor Tuning Parameter ----------------
# Each movement command drives the robot one grid cell (approx. 0.22 m)
DISTANCE_PER_MOVE = 0.22

# ---------------- Instantiate Motors and MotorPair ----------------
# For integration we reassign motor pins as needed.
# Here we assume:
#   - Left motor: direction controlled by pin 3, PWM on pin 2.
#   - Right motor: direction controlled by pin 5, PWM on pin 4.
left_motor = Motor(dir_pin=3, pwm_pin=2)
right_motor = Motor(dir_pin=5, pwm_pin=4)
motors = MotorPair(left_motor, right_motor)

# ---------------- Odometry Helper Functions ----------------
def update_position(move_type):
    """
    Update the global current_position.
    'front' moves in the current vehicle orientation.
    'rear' moves in the opposite direction.
    """
    global current_position
    if move_type == 'front':
        current_position = get_new_position(current_position, vehicle_orientation)
    elif move_type == 'rear':
        back_orientation = (vehicle_orientation + 2) % 4
        current_position = get_new_position(current_position, back_orientation)

def get_new_position(position, orientation, distance=DISTANCE_PER_MOVE):
    """
    Calculate new (x, y) position given current position, orientation, and movement distance.
    Orientation: 0 = North, 1 = East, 2 = South, 3 = West.
    """
    x, y = position
    if orientation == 0:      # North (+y)
        return (x, y + distance)
    elif orientation == 1:    # East (+x)
        return (x + distance, y)
    elif orientation == 2:    # South (-y)
        return (x, y - distance)
    elif orientation == 3:    # West (-x)
        return (x - distance, y)
    return position

def update_orientation(turn_direction):
    """
    Update the global vehicle_orientation based on a turn.
    'left' rotates counterclockwise; 'right' rotates clockwise.
    """
    global vehicle_orientation
    if turn_direction == 'left':
        vehicle_orientation = (vehicle_orientation - 1) % 4
    elif turn_direction == 'right':
        vehicle_orientation = (vehicle_orientation + 1) % 4

# ---------------- Alternative Navigation: Random Turn Decision ----------------
def get_node_direction(current_node, neighbor):
    """
    Determine the cardinal direction from current_node to neighbor based on their positions.
    Returns: 0 (North), 1 (East), 2 (South), or 3 (West) if the edge is exactly cardinal.
    Otherwise, returns None.
    """
    current_pos = graph_nodes[current_node]
    neighbor_pos = graph_nodes[neighbor]
    dx = neighbor_pos[0] - current_pos[0]
    dy = neighbor_pos[1] - current_pos[1]
    tol = 1e-6  # Tolerance for floating-point comparison
    if abs(dx) < tol and dy > 0:
        return 0  # North
    elif abs(dy) < tol and dx > 0:
        return 1  # East
    elif abs(dx) < tol and dy < 0:
        return 2  # South
    elif abs(dy) < tol and dx < 0:
        return 3  # West
    return None

def decide_next_node_random(current_node, previous_node, current_orientation):
    """
    At the current node, randomly choose one of the available neighbors.
    If a previous node exists and other choices are available, do not select it.
    Returns a tuple: (next_node, new_orientation, turn_type)
    where turn_type is one of 'straight', 'left', 'right', or 'rear'.
    """
    neighbors = list(graph_neighbors[current_node])
    # Exclude the previous node if possible.
    if previous_node is not None and len(neighbors) > 1:
        if previous_node in neighbors:
            neighbors.remove(previous_node)
    # Randomly choose a candidate neighbor.
    if not neighbors:
        candidate = previous_node
    else:
        candidate = random.choice(neighbors)
    
    candidate_direction = get_node_direction(current_node, candidate)
    # Determine turn type relative to current_orientation.
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

# ---------------- Sensor and Node Verification Functions ----------------
def get_sensor_pattern():
    """
    Read path sensor states and return a dictionary with keys:
      'front', 'right', 'rear', and 'left'.
    A value of 1 indicates path detection.
    """
    return {
        'front': machine.Pin(8, machine.Pin.IN).value(),
        'right': machine.Pin(11, machine.Pin.IN).value(),
        'rear': machine.Pin(9, machine.Pin.IN).value(),
        'left': machine.Pin(10, machine.Pin.IN).value()
    }

def expected_sensor_pattern(node, current_orientation, candidate_turn_type):
    """
    Determine the expected sensor pattern at a node given the vehicle's orientation
    and the candidate turn decision.
    
    Special handling for node 1: expected pattern is all sensors off.
    
    - For nodes RY and BG: returns None (verification not applied).
    - For nodes X1/X2/X3/X4: all sensors should be active.
    - For other nodes:
       * If the node has only one neighbor, only the sensor matching current_orientation is expected.
       * If the node has exactly 2 neighbors:
             - If the candidate turn is 'straight', then the sensor pattern is the typical straight road
               (front and rear active), which should not be treated as a node; so return None.
             - Otherwise, it is assumed the vehicle came from behind and the new exit is off to one side.
               In that case, the expected active sensors are:
                   - the sensor opposite to current_orientation ("rear"), and
                   - the sensor corresponding to the candidate turn ('left' or 'right').
       * For nodes with 3 or more neighbors, assume an intersection where all sensors are active.
    """
    if node == 1:
        # For node 1, all sensors should be off.
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
        # For a straight road, candidate_turn_type would be 'straight'.
        if candidate_turn_type == 'straight':
            # Do not apply sensor verification for a straight road.
            return None
        # Otherwise, expected: the sensor opposite to the current orientation (rear)
        # and the sensor corresponding to the candidate turn (left or right).
        rear_sensor = mapping[(current_orientation + 2) % 4]
        if candidate_turn_type == 'left':
            side_sensor = 'left'
        elif candidate_turn_type == 'right':
            side_sensor = 'right'
        else:
            side_sensor = mapping[current_orientation]  # fallback
        expected = {'front': 0, 'right': 0, 'rear': 0, 'left': 0}
        expected[rear_sensor] = 1
        expected[side_sensor] = 1
        return expected
    else:
        # For nodes with 3 or more neighbors, assume all sensors active.
        return {'front': 1, 'right': 1, 'rear': 1, 'left': 1}

def check_node_sensor(node, current_orientation, candidate_turn_type):
    """
    Check if the actual sensor readings match the expected pattern for the node.
    If there is a mismatch, print a warning. (You may uncomment an assertion to enforce failure.)
    """
    expected = expected_sensor_pattern(node, current_orientation, candidate_turn_type)
    if expected is None:
        # No sensor verification applied.
        return
    actual = get_sensor_pattern()
    if actual != expected:
        print("Warning: Sensor mismatch at node", node)
        print("Expected:", expected, "Actual:", actual)
        # Uncomment the following line to enforce an assertion:
        # assert actual == expected, "Sensor mismatch at node " + str(node)
    else:
        print("Sensor readings at node", node, "match expected pattern.")

# ---------------- Main Loop: Navigation with Random Turn and Sensor Check using Sensor Pattern ----------------
while True:
    sensor_pattern = get_sensor_pattern()
    sensor_count = sum(sensor_pattern.values())
    
    # Special handling for node 1:
    # Node 1 is considered reached if all sensors are off.
    if current_node == 1:
        node_reached = (sensor_count == 0)
    else:
        # Define a condition to ignore a pattern where front, left, and right are active but rear is off.
        ignore_pattern = (sensor_pattern['front'] == 1 and 
                          sensor_pattern['left'] == 1 and 
                          sensor_pattern['right'] == 1 and 
                          sensor_pattern['rear'] == 0)
        node_reached = ((sensor_count >= 3 or (sensor_count == 2 and (sensor_pattern['left'] == 1 or sensor_pattern['right'] == 1)))
                        and not ignore_pattern)
    
    if node_reached:
        print("Vehicle reached node:", current_node,
              "with sensor pattern:", sensor_pattern,
              "and orientation:", vehicle_orientation)
        
        candidate, new_orientation, turn_type = decide_next_node_random(current_node, previous_node, vehicle_orientation)
        print("Random decision: turning", turn_type, "to node", candidate)
        
        check_node_sensor(current_node, vehicle_orientation, turn_type)
        
        if turn_type == 'straight':
            motors.move_forward(duration=0.5)
            update_position('front')
        elif turn_type == 'left':
            motors.turn_left(duration=0.5)
            motors.move_forward(duration=0.5)
            update_position('front')
            update_orientation('left')
        elif turn_type == 'right':
            motors.turn_right(duration=0.5)
            motors.move_forward(duration=0.5)
            update_position('front')
            update_orientation('right')
        elif turn_type == 'rear':
            motors.move_backward(duration=0.5)
            update_position('rear')
        
        previous_node = current_node
        current_node = candidate
        vehicle_orientation = new_orientation
    else:
        motors.move_forward(duration=0.2)
    
    time.sleep(0.1)
