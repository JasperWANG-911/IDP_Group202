import machine
import time

# -------------- Motor Setup --------------
# Define GPIO pins for motor control (adjust pin numbers as needed)
left_motor_forward_pin = machine.Pin(2, machine.Pin.OUT)
left_motor_backward_pin = machine.Pin(3, machine.Pin.OUT)
right_motor_forward_pin = machine.Pin(4, machine.Pin.OUT)
right_motor_backward_pin = machine.Pin(5, machine.Pin.OUT)

# Define PWM channels for speed control on each motor
left_motor_pwm = machine.PWM(machine.Pin(6))
right_motor_pwm = machine.PWM(machine.Pin(7))
left_motor_pwm.freq(1000)   # Set PWM frequency to 1000 Hz
right_motor_pwm.freq(1000)

# -------------- Sensor Setup --------------
# Define digital input pins for track sensors: front, rear, left, right
sensor_front = machine.Pin(8, machine.Pin.IN)
sensor_rear = machine.Pin(9, machine.Pin.IN)
sensor_left = machine.Pin(10, machine.Pin.IN)
sensor_right = machine.Pin(11, machine.Pin.IN)

# -------------- Global Variables for Odometry and Graph --------------
# Track the current grid position (x, y); starting at (0.0, 0.0)
current_position = (0.0, 0.0)
# Track the current direction: 0 = North, 1 = East, 2 = South, 3 = West
current_direction = 0
# A dictionary to map intersections (nodes) to the exits (edges) that have been used.
intersections_map = {}

# -------------- Motor Tuning Parameter --------------
# Based on a typical small robot:
# Assuming a wheel diameter of ~70 mm gives a wheel circumference of about 220 mm (0.22 m)
# If one command equals one wheel revolution, then one grid cell is ~0.22 meters.
DISTANCE_PER_MOVE = 0.22

# -------------- Movement Functions --------------
def stop_motors():
    # Stop all motors and reset PWM duty cycles.
    left_motor_forward_pin.value(0)
    left_motor_backward_pin.value(0)
    right_motor_forward_pin.value(0)
    right_motor_backward_pin.value(0)
    left_motor_pwm.duty(0)
    right_motor_pwm.duty(0)

def move_forward(speed=600, duration=0.5):
    # Move forward: assume this advances the vehicle by one calibrated grid cell.
    left_motor_pwm.duty(speed)
    right_motor_pwm.duty(speed)
    left_motor_forward_pin.value(1)
    left_motor_backward_pin.value(0)
    right_motor_forward_pin.value(1)
    right_motor_backward_pin.value(0)
    time.sleep(duration)
    stop_motors()

def move_backward(speed=600, duration=0.5):
    # Move backward: assume this retreats the vehicle by one calibrated grid cell.
    left_motor_pwm.duty(speed)
    right_motor_pwm.duty(speed)
    left_motor_forward_pin.value(0)
    left_motor_backward_pin.value(1)
    right_motor_forward_pin.value(0)
    right_motor_backward_pin.value(1)
    time.sleep(duration)
    stop_motors()

def turn_left(speed=600, duration=0.5):
    # Turn left in place (or with minimal forward movement).
    left_motor_pwm.duty(speed)
    right_motor_pwm.duty(speed)
    left_motor_forward_pin.value(0)
    left_motor_backward_pin.value(0)  # left motor stops for turning
    right_motor_forward_pin.value(1)
    right_motor_backward_pin.value(0)
    time.sleep(duration)
    stop_motors()

def turn_right(speed=600, duration=0.5):
    # Turn right in place (or with minimal forward movement).
    left_motor_pwm.duty(speed)
    right_motor_pwm.duty(speed)
    left_motor_forward_pin.value(1)
    left_motor_backward_pin.value(0)
    right_motor_forward_pin.value(0)
    right_motor_backward_pin.value(0)  # right motor stops for turning
    time.sleep(duration)
    stop_motors()

# -------------- Odometry Helper Functions --------------
def get_new_position(position, heading, distance=DISTANCE_PER_MOVE):
    """
    Compute the new grid position based on the current position, heading, and a calibrated distance.
    """
    x, y = position
    if heading == 0:      # North
        return (x, y + distance)
    elif heading == 1:    # East
        return (x + distance, y)
    elif heading == 2:    # South
        return (x, y - distance)
    elif heading == 3:    # West
        return (x - distance, y)
    return position

def update_odometry(chosen_exit):
    """
    Update the global current_position and current_direction based on the motor commands
    and the calibrated distance per movement.
    """
    global current_position, current_direction
    if chosen_exit == 'front':
        current_position = get_new_position(current_position, current_direction, DISTANCE_PER_MOVE)
    elif chosen_exit == 'left':
        current_direction = (current_direction - 1) % 4
        current_position = get_new_position(current_position, current_direction, DISTANCE_PER_MOVE)
    elif chosen_exit == 'right':
        current_direction = (current_direction + 1) % 4
        current_position = get_new_position(current_position, current_direction, DISTANCE_PER_MOVE)
    elif chosen_exit == 'rear':
        back_direction = (current_direction + 2) % 4
        current_position = get_new_position(current_position, back_direction, DISTANCE_PER_MOVE)

# -------------- Sensor and Intersection Helper Functions --------------
def get_sensor_pattern():
    """
    Read the current state of the track sensors.
    Returns a dictionary with keys 'front', 'rear', 'left', 'right'
    where a value of 1 indicates that the sensor detects the track.
    """
    return {
        'front': sensor_front.value(),
        'right': sensor_right.value(),
        'rear': sensor_rear.value(),
        'left': sensor_left.value()
    }

def generate_intersection_id(pattern):
    """
    Generate a unique ID for an intersection using the current estimated position
    (tracked from motor commands with tuning) and the sensor pattern.
    """
    return (current_position, tuple(sorted(pattern.items())))

def choose_direction(pattern, visited_exits):
    """
    Decide which exit to take at an intersection.
    Available exits are those where the sensor indicates track detection.
    Prioritize unvisited exits in the order: front, left, right, then rear.
    """
    available_exits = [exit_name for exit_name, val in pattern.items() if val == 1]
    if 'front' in available_exits and 'front' not in visited_exits:
        return 'front'
    if 'left' in available_exits and 'left' not in visited_exits:
        return 'left'
    if 'right' in available_exits and 'right' not in visited_exits:
        return 'right'
    if 'rear' in available_exits and 'rear' not in visited_exits:
        return 'rear'
    return None

def execute_movement(chosen_exit):
    """
    Execute the movement corresponding to the chosen exit.
    For left/right turns, the robot first rotates and then moves forward.
    After movement, update the odometry.
    """
    if chosen_exit == 'front':
        move_forward()
    elif chosen_exit == 'left':
        turn_left()
        move_forward()
    elif chosen_exit == 'right':
        turn_right()
        move_forward()
    elif chosen_exit == 'rear':
        move_backward()
    update_odometry(chosen_exit)
    time.sleep(0.2)

# -------------- Main Loop: Graph Building with Tuned Odometry --------------
while True:
    # Read the sensor pattern from track sensors
    sensor_pattern = get_sensor_pattern()
    sensor_count = sum(sensor_pattern.values())
    
    # When three or more sensors detect the track, assume we're at an intersection.
    if sensor_count >= 3:
        # Generate a unique intersection ID using the current position and sensor pattern.
        intersection_id = generate_intersection_id(sensor_pattern)
        
        # If this intersection is new, initialize its record.
        if intersection_id not in intersections_map:
            intersections_map[intersection_id] = set()
        
        # Choose an exit from the intersection that has not yet been used.
        chosen_exit = choose_direction(sensor_pattern, intersections_map[intersection_id])
        
        if chosen_exit is not None:
            # Mark the chosen exit as visited.
            intersections_map[intersection_id].add(chosen_exit)
            # Execute the movement and update odometry (position and heading).
            execute_movement(chosen_exit)
        else:
            # If all available exits have been visited, move forward by default.
            move_forward()
            update_odometry('front')
    else:
        # When not at an intersection, follow the track by moving forward.
        move_forward(duration=0.2)
        update_odometry('front')
    
    # Small delay to allow sensor readings to stabilize.
    time.sleep(0.1)
