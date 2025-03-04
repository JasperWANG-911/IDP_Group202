import machine
import time
import sensor_for_main as sensor
from motor import Motor1, Motor2
from motor_pair import MotorPair
from odometry import Odometry
import navigation

def main():
    # Initialize motor pair and odometry state
    left_motor = Motor2()  # Using swapped motors as per previous configuration
    right_motor = Motor1()
    motors = MotorPair(left_motor, right_motor)
    odom = Odometry()

    # Set up LED on Pin 25 for visual indication
    led = machine.Pin(25, machine.Pin.OUT)

    # Move forward until the starting condition is met:
    # Both front and rear sensors must be active
    print("Moving forward until starting condition is met (both front and rear sensors active)...")
    while True:
        # Move forward a short duration
        motors.move_forward(duration=0.1)
        # Read sensor pattern after moving
        sp = sensor.get_track_sensor_pattern()
        # Check if starting condition is met
        if sp['front'] == 1 and sp['rear'] == 1:
            print("Starting condition met: Vehicle has set off from starting point.")
            # Flash LED to indicate start
            for i in range(3):
                led.value(1)
                time.sleep(0.2)
                led.value(0)
                time.sleep(0.2)
            break
        time.sleep(0.1)

    # Run the navigation routine using the predetermined route logic.
    # The vehicle starts at node 1 and will visit X1, X2, X3, X4, RY, BG in order.
    navigation.run_navigation(motors, odom)

if __name__ == "__main__":
    main()
