# main.py
import machine
import time
import sensor
from motor import Motor
from motor import MotorPair
import odometry
import turning
import navigation
from graph import graph_nodes

# Global navigation variables
current_position = (0.0, 0.0)
current_node = 1
previous_node = None
vehicle_orientation = 0

# Instantiate motors and motor pair
left_motor = Motor(dir_pin=3, pwm_pin=2)
right_motor = Motor(dir_pin=5, pwm_pin=4)
motors = MotorPair(left_motor, right_motor)

# LED for start signal
led = machine.Pin(25, machine.Pin.OUT)

# Beginning of task: wait at node 1 until the vehicle sets off
print("Waiting to set off from starting node 1...")
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

# Main loop (simplified example)
while True:
    sp = sensor.get_track_sensor_pattern()
    sensor_count = sum(sp.values())
    if current_node == 1:
        node_reached = (sensor_count == 0)
    else:
        ignore_pattern = (sp['front'] == 1 and sp['left'] == 1 and sp['right'] == 1 and sp['rear'] == 0)
        node_reached = ((sensor_count >= 3 or (sensor_count == 2 and (sp['left'] == 1 or sp['right'] == 1))) and not ignore_pattern)
    
    if node_reached:
        print("Vehicle reached node:", current_node, "with sensor pattern:", sp)
        candidate, new_orientation, turn_type = navigation.decide_next_node_random(current_node, previous_node, vehicle_orientation)
        print("Random decision: turning", turn_type, "to node", candidate)
        # Here, call turning.turn_until_shift() for left/right turns if needed.
        if turn_type == 'straight':
            motors.move_forward(duration=0.5)
            current_position = odometry.get_new_position(current_position, vehicle_orientation)
        elif turn_type == 'left':
            turning.turn_until_shift('left', motors, speed=60, increment=0.1, timeout=3)
            motors.move_forward(duration=0.5)
            current_position = odometry.get_new_position(current_position, vehicle_orientation)
            vehicle_orientation = (vehicle_orientation - 1) % 4
        elif turn_type == 'right':
            turning.turn_until_shift('right', motors, speed=60, increment=0.1, timeout=3)
            motors.move_forward(duration=0.5)
            current_position = odometry.get_new_position(current_position, vehicle_orientation)
            vehicle_orientation = (vehicle_orientation + 1) % 4
        elif turn_type == 'rear':
            motors.move_backward(duration=0.5)
            current_position = odometry.get_new_position(current_position, (vehicle_orientation + 2) % 4)
        
        previous_node = current_node
        current_node = candidate
    else:
        motors.move_forward(duration=0.2)
    time.sleep(0.1)
