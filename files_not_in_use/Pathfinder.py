#Edit to test Lisa's Git setup, DELETE

import machine
import time

# break down into modules: no need for motor setup, sensor setup...
# we just do path tracking here

# # -------------- Motor Setup --------------
# # Left motor control pins
# left_motor_forward_pin = machine.Pin(2, machine.Pin.OUT)
# left_motor_backward_pin = machine.Pin(3, machine.Pin.OUT)
# # Right motor control pins
# right_motor_forward_pin = machine.Pin(4, machine.Pin.OUT)
# right_motor_backward_pin = machine.Pin(5, machine.Pin.OUT)

# # PWM channels for speed control
# left_motor_pwm = machine.PWM(machine.Pin(6))
# right_motor_pwm = machine.PWM(machine.Pin(7))
# left_motor_pwm.freq(1000)    # Set PWM frequency to 1000 Hz
# right_motor_pwm.freq(1000)

# -------------- Sensor Setup --------------
# Track sensors: front, rear, left, right
sensor_front = machine.Pin(8, machine.Pin.IN)
sensor_rear  = machine.Pin(9, machine.Pin.IN)
sensor_left  = machine.Pin(10, machine.Pin.IN)
sensor_right = machine.Pin(11, machine.Pin.IN)

# -------------- Global Variables for Odometry and Graph --------------
current_position = (0.0, 0.0)    # Starting at (0.0, 0.0)
current_direction = 0          # 0=North, 1=East, 2=South, 3=West
intersections_map = {}         # To store visited intersection exits

# -------------- Motor Tuning Parameter --------------
# With a ~70 mm wheel (circumference ~220 mm = 0.22 m), we assume each move command
# drives the robot one "grid cell" of ~0.22 m.
DISTANCE_PER_MOVE = 0.22

# Use circumference * number of turns


# -------------- Movement Functions --------------
def stop_motors():
    """Stop both motors and reset PWM duty cycle to 0."""
    left_motor_forward_pin.value(0)
    left_motor_backward_pin.value(0)
    right_motor_forward_pin.value(0)
    right_motor_backward_pin.value(0)
    left_motor_pwm.duty_u16(0)
    right_motor_pwm.duty_u16(0)

def move_forward(speed=600, duration=0.5):
    """
    Move both motors forward at a given PWM duty (0–65535)
    and for a specified duration (in seconds).
    """
    left_motor_pwm.duty_u16(speed)
    right_motor_pwm.duty_u16(speed)
    left_motor_forward_pin.value(1)
    left_motor_backward_pin.value(0)
    right_motor_forward_pin.value(1)
    right_motor_backward_pin.value(0)
    time.sleep(duration)
    stop_motors()

def move_backward(speed=600, duration=0.5):
    """
    Move both motors backward.
    """
    left_motor_pwm.duty_u16(speed)
    right_motor_pwm.duty_u16(speed)
    left_motor_forward_pin.value(0)
    left_motor_backward_pin.value(1)
    right_motor_forward_pin.value(0)
    right_motor_backward_pin.value(1)
    time.sleep(duration)
    stop_motors()

def turn_left(speed=600, duration=0.5):
    """
    Turn in place to the left:
      - Left motor is stopped (or slowed) while the right motor moves forward.
    """
    left_motor_pwm.duty_u16(speed)
    right_motor_pwm.duty_u16(speed)
    # Stop left motor
    left_motor_forward_pin.value(0)
    left_motor_backward_pin.value(0)
    # Right motor moves forward
    right_motor_forward_pin.value(1)
    right_motor_backward_pin.value(0)
    time.sleep(duration)
    stop_motors()

def turn_right(speed=600, duration=0.5):
    """
    Turn in place to the right:
      - Right motor is stopped while the left motor moves forward.
    """
    left_motor_pwm.duty_u16(speed)
    right_motor_pwm.duty_u16(speed)
    # Left motor moves forward
    left_motor_forward_pin.value(1)
    left_motor_backward_pin.value(0)
    # Stop right motor
    right_motor_forward_pin.value(0)
    right_motor_backward_pin.value(0)
    time.sleep(duration)
    stop_motors()

# -------------- Odometry Helper Functions --------------
def get_new_position(position, heading, distance=DISTANCE_PER_MOVE):
    """
    Calculate new (x, y) position given current position, heading, and movement distance.
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
    Update the global current_position and current_direction based on the chosen exit.
    """
    global current_position, current_direction
    if chosen_exit == 'front':
        current_position = get_new_position(current_position, current_direction)
    elif chosen_exit == 'left':
        current_direction = (current_direction - 1) % 4
        current_position = get_new_position(current_position, current_direction)
    elif chosen_exit == 'right':
        current_direction = (current_direction + 1) % 4
        current_position = get_new_position(current_position, current_direction)
    elif chosen_exit == 'rear':
        # 'Rear' is treated as moving backward relative to current direction.
        back_direction = (current_direction + 2) % 4
        current_position = get_new_position(current_position, back_direction)

# -------------- Sensor and Intersection Helper Functions --------------
def get_sensor_pattern():
    """
    Read track sensor states and return a dictionary with keys:
      'front', 'right', 'rear', and 'left'.
    A value of 1 indicates track detection.
    """
    return {
        'front': sensor_front.value(),
        'right': sensor_right.value(),
        'rear': sensor_rear.value(),
        'left': sensor_left.value()
    }

def generate_intersection_id(pattern):
    """
    Create a unique intersection ID based on the current position and sensor pattern.
    """
    return (current_position, tuple(sorted(pattern.items())))

def choose_direction(pattern, visited_exits):
    """
    Decide which direction to take at an intersection.
    Prioritize unvisited exits in this order: front, left, right, rear.
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
    Perform the movement based on the chosen exit:
      - 'front' moves forward.
      - 'left' or 'right' turn then move forward.
      - 'rear' moves backward.
    Then update odometry.
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

# -------------- Main Loop: Navigation and Graph Building --------------
while True:
    # Read current sensor pattern from track sensors.
    sensor_pattern = get_sensor_pattern()
    sensor_count = sum(sensor_pattern.values())
    
    # Detect intersection when three or more sensors are active.
    if sensor_count >= 3:
        intersection_id = generate_intersection_id(sensor_pattern)
        if intersection_id not in intersections_map:
            intersections_map[intersection_id] = set()
        chosen_exit = choose_direction(sensor_pattern, intersections_map[intersection_id])
        if chosen_exit is not None:
            intersections_map[intersection_id].add(chosen_exit)
            execute_movement(chosen_exit)
        else:
            # If all exits at this intersection have been visited, continue moving forward.
            move_forward()
            update_odometry('front')
    else:
        # When not at an intersection, follow the track.
        move_forward(duration=0.2)
        update_odometry('front')
    
    # Small delay for sensor stabilization.
    time.sleep(0.1)
