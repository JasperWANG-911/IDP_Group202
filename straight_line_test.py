#!/usr/bin/env python3
import time


from motor import MotorPair, Motor1, Motor2
from orientation_control import OrientationController

def test_straight_line():
    """
    Test to simulate the vehicle following a straight line.
    The vehicle should drive forward continuously while the PID-based
    orientation control (via OrientationController.update()) is active.
    
    This test runs for a fixed duration and prints PID output.
    """
    # Instantiate motors: Motor2 is used as left, Motor1 as right.
    left_motor = Motor2()
    right_motor = Motor1()
    motors = MotorPair(left_motor, right_motor)
    
    # Create an OrientationController with a base speed and PID parameters.
    # These parameters can be tuned for your hardware.
    orientation_controller = OrientationController(base_speed=75, k_p=15, k_i=0.05, k_d=0)
    
    print("Starting straight line test. Vehicle should drive forward with continuous orientation control.")
    
    # Duration for the test in seconds.
    duration = 60  
    start_time = time.time()
    
    # Instead of a single blocking move_forward command,
    # run a loop that continuously updates orientation control.
    while time.time() - start_time < duration:
        orientation_controller.update()  # PID update for forward motion
        time.sleep(0.1)  # update interval
    
    print("Straight line test complete.")

if __name__ == "__main__":
    test_straight_line()
