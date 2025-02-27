import machine
import time
import math
import random
import sensor             # Import the sensor module from sensor.py
from motor import Motor1, Motor2   # Import Motor1 and Motor2 classes from motor.py

# ---------------- Motor Pair Class ----------------
class MotorPair:
    def __init__(self, left_motor, right_motor):
        self.left = left_motor
        self.right = right_motor

    def move_forward(self, duration=0.5):
        """
        Drive both motors forward for the given duration.
        """
        self.left.Forward()   # Call Forward method (fixed speed)
        self.right.Forward()
        time.sleep(duration)
        self.left.off()
        self.right.off()

    def move_backward(self, duration=0.5):
        """
        Drive both motors in reverse for the given duration.
        """
        self.left.Reverse()   # Call Reverse method (fixed speed)
        self.right.Reverse()
        time.sleep(duration)
        self.left.off()
        self.right.off()

    def turn_left(self, duration=0.5):
        """
        Turn on the spot to the left by running the left motor in reverse
        and the right motor forward.
        """
        self.left.Reverse()
        self.right.Forward()
        time.sleep(duration)
        self.left.off()
        self.right.off()

    def turn_right(self, duration=0.5):
        """
        Turn on the spot to the right by running the left motor forward
        and the right motor in reverse.
        """
        self.left.Forward()
        self.right.Reverse()
        time.sleep(duration)
        self.left.off()
        self.right.off()

# ---------------- Global Variables for Navigation ----------------
current_position = (0.0, 0.0)      # in meters
current_node = 1                 # starting at node 1
previous_node = None             # no previous node at start
vehicle_orientation = 0          # 0 = North, 1 = East, 2 = South, 3 = West

# ---------------- Map Setup ----------------
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

# ---------------- Motor Tuning Parameter ----------------
DISTANCE_PER_MOVE = 0.22   # Each move command drives one grid cell (~0.22 m)

# ---------------- Instantiate Motors and MotorPair ----------------
left_motor = Motor1()   # Instantiated with fixed pins defined in motor.py
right_motor = Motor2()
motors = MotorPair(left_motor, right_motor)

# ---------------- Odometry Helper Functions ----------------
def update_position(move_type):
    global current_position
    if move_type == 'front':
        current_position = get_new_position(current_position, vehicle_orientation)
    elif move_type == 'rear':
        back_orientation = (vehicle_orientation + 2) % 4
        current_position = get_new_position(current_position, back_orientation)

def get_new_position(position, orientation, distance=DISTANCE_PER_MOVE):
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
    global vehicle_orientation
    if turn_direction == 'left':
        vehicle_orientation = (vehicle_orientation - 1) % 4
    elif turn_direction == 'right':
        vehicle_orientation = (vehicle_orientation + 1) % 4

# ---------------- Turning Helper Function ----------------
def turn_until_shift(turn_type, increment=0.1, timeout=3):
    """
    Turn in small increments until a 90° shift is detected.
    For a left turn, we require that the left sensor is active and the front sensor is off for 3 consecutive readings.
    For a right turn, we require that the right sensor is active and the front sensor is off for 3 consecutive readings.
    A timeout prevents endless turning.
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

# ---------------- Alternative Navigation: Random Turn Decision ----------------
def get_node_direction(current_node, neighbor):
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

# ---------------- Sensor and Node Verification Functions ----------------
def expected_sensor_pattern(node, current_orientation, candidate_turn_type):
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
    expected = expected_sensor_pattern(node, current_orientation, candidate_turn_type)
    if expected is None:
        return
    actual = sensor.get_track_sensor_pattern()
    if actual != expected:
        print("Warning: Sensor mismatch at node", node)
        print("Expected:", expected, "Actual:", actual)
    else:
        print("Sensor readings at node", node, "match expected pattern.")

# ---------------- Beginning of Task: Wait for Start and Flash LED ----------------
print("Waiting to set off from starting node 1...")
led = machine.Pin(25, machine.Pin.OUT)  # LED on Pin 25 (adjust as needed)
while True:
    sp = sensor.get_track_sensor_pattern()
    if sp['front'] == 1 and sp['rear'] == 1:
        print("Vehicle has set off from starting point.")
        for i in range(3):
            led.value(1)
            time.sleep(0.2)
            led.value(0)
            time.sleep(0.2)
        break
    time.sleep(0.1)

# ---------------- Main Loop: Navigation with Random Turn and Sensor Check ----------------
while True:
    sensor_pattern = sensor.get_track_sensor_pattern()
    sensor_count = sum(sensor_pattern.values())
    
    if current_node == 1:
        node_reached = (sensor_count == 0)
    else:
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
            turn_until_shift('left', increment=0.1, timeout=3)
            motors.move_forward(duration=0.5)
            update_position('front')
            update_orientation('left')
        elif turn_type == 'right':
            turn_until_shift('right', increment=0.1, timeout=3)
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
