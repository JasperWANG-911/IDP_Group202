import machine
import time
import sensor_for_main as sensor
from motor import Motor1, Motor2
from motor_pair import MotorPair
from odometry import Odometry
import navigation

def main():
    # Initialize motor pair and odometry state
    left_motor = Motor1()
    right_motor = Motor2()
    motors = MotorPair(left_motor, right_motor)
    odom = Odometry()

    # Set starting node and previous node variables
    current_node = 1
    previous_node = None

    # Set up LED on Pin 25 for visual indication
    led = machine.Pin(25, machine.Pin.OUT)

    # Wait for the start condition: both front and rear sensors active
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

    # Main navigation loop
    while True:
        sensor_pattern = sensor.get_track_sensor_pattern()
        sensor_count = sum(sensor_pattern.values())

        # Determine if a node has been reached based on sensor readings
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
                  "and orientation:", odom.vehicle_orientation)

            # Decide next node and corresponding turn based on current state
            candidate, new_orientation, turn_type = navigation.decide_next_node_random(
                current_node, previous_node, odom.vehicle_orientation)
            print("Random decision: turning", turn_type, "to node", candidate)

            # Check sensor readings at the current node
            navigation.check_node_sensor(current_node, odom.vehicle_orientation, turn_type)

            # Execute movement based on decision
            if turn_type == 'straight':
                motors.move_forward(duration=0.5)
                odom.update_position('front')
            elif turn_type == 'left':
                navigation.turn_until_shift(motors, 'left', increment=0.1, timeout=3)
                motors.move_forward(duration=0.5)
                odom.update_position('front')
                odom.update_orientation('left')
            elif turn_type == 'right':
                navigation.turn_until_shift(motors, 'right', increment=0.1, timeout=3)
                motors.move_forward(duration=0.5)
                odom.update_position('front')
                odom.update_orientation('right')
            elif turn_type == 'rear':
                motors.move_backward(duration=0.5)
                odom.update_position('rear')

            # Update navigation state for next iteration
            previous_node = current_node
            current_node = candidate
            odom.vehicle_orientation = new_orientation
        else:
            motors.move_forward(duration=0.2)

        time.sleep(0.1)

if __name__ == "__main__":
    main()
