#!/usr/bin/env python3
import time
import machine
from motor import MotorPair, Motor1, Motor2  # Motor2 is left, Motor1 is right
from orientation_control import OrientationController

def wait_for_button_release(button):
    """
    Wait until the button is released.
    """
    while button.value() == 0:
        time.sleep(0.05)
    time.sleep(0.2)  # debounce delay

def test_straight_line():
    """
    Test to simulate the vehicle following a straight line.
    The vehicle should drive forward continuously while the PID-based
    orientation control (via OrientationController.update()) is active.
    
    The test starts when you press and release the button on pin 20,
    and stops when you press and release it again. At the stop event,
    the motors are explicitly stopped.
    """
    # Instantiate motors: Motor2 is used as left, Motor1 as right.
    left_motor = Motor2()
    right_motor = Motor1()
    motors = MotorPair(left_motor, right_motor)
    
    # Create an OrientationController with a base speed and tuned PID parameters.
    orientation_controller = OrientationController(base_speed=75, k_p=10, k_i=0.2, k_d=8, deriv_window=5, integral_window=100)
    
    # Configure button on pin 20 (active low, with internal pull-up)
    button = machine.Pin(20, machine.Pin.IN, machine.Pin.PULL_UP)
    
    print("Press and release the button on pin 20 to start the straight line test.")
    # Wait for a complete press-release cycle to start.
    while button.value() == 0:
        time.sleep(0.05)  # Ensure button is released before starting
    while button.value() == 1:
        time.sleep(0.05)
    time.sleep(0.2)  # Debounce delay
    
    print("Test started. Vehicle should follow a straight line with continuous PID updates.")
    
    running = True
    prev_button_state = button.value()  # Likely 1 (not pressed)
    
    while running:
        # Update orientation control continuously.
        orientation_controller.update()  
        time.sleep(0.1)
        
        # Edge detection: check if button has been pressed (transition from high to low)
        current_state = button.value()
        if prev_button_state == 1 and current_state == 0:
            # Wait for the button to be released to confirm the press-release cycle.
            wait_for_button_release(button)
            print("Stop button detected. Stopping the straight line test.")
            running = False
        prev_button_state = current_state

    # Stop the orientation control and motors explicitly.
    orientation_controller.stop()
    motors.left.off()
    motors.right.off()
    
    print("Straight line test complete.")

if __name__ == "__main__":
    test_straight_line()
