#!/usr/bin/env python3
import time
import machine
from motor import MotorPair, Motor1, Motor2  # Motor2 is left, Motor1 is right
from orientation_control import OrientationController
from line_sensor import LineSensors

def wait_for_button_release(button):
    """
    Wait until the button is released.
    """
    while button.value() == 0:
        time.sleep(0.05)
    time.sleep(0.2)  # debounce delay

def test_reverse_with_sensor_stop():
    """
    Test to move backward continuously using update_reverse().
    Whenever a sensor pattern is detected (either left_side or right_side is active),
    the vehicle stops for 3 seconds before resuming reverse motion.
    Button control is similar to the straight line test: a press-release cycle starts the test,
    and another press-release cycle stops the test.
    """
    # Instantiate motors, controller, and sensors.
    left_motor = Motor2()    # Left motor
    right_motor = Motor1()   # Right motor
    motors = MotorPair(left_motor, right_motor)
    
    # Create an OrientationController with desired PID parameters.
    orientation_controller = OrientationController(
        base_speed=75, k_p=10, k_i=0.2, k_d=8, deriv_window=5, integral_window=100
    )
    sensors = LineSensors()
    
    # Configure button on pin 20 (active low, with internal pull-up)
    button = machine.Pin(20, machine.Pin.IN, machine.Pin.PULL_UP)
    
    print("Press and release the button on pin 20 to start the reverse sensor test.")
    # Wait for a complete press-release cycle to start.
    while button.value() == 0:
        time.sleep(0.05)
    while button.value() == 1:
        time.sleep(0.05)
    time.sleep(0.2)  # debounce delay
    
    print("Reverse sensor test started. Press the button to stop the test.")
    running = True
    prev_button_state = button.value()  # Expected to be 1 when not pressed.
    
    while running:
        # Continuously update reverse control.
        orientation_controller.update_reverse()
        sensor_data = sensors.read_all()
        print("Sensor data:", sensor_data)
        
        # If a sensor pattern is detected, stop and wait 3 seconds.
        if sensor_data.get('left_side') == 1 or sensor_data.get('right_side') == 1:
            print("Sensor pattern detected! Stopping for 3 seconds...")
            orientation_controller.stop()  # Stop the motors.
            time.sleep(3)                  # Wait for 3 seconds.
            print("Resuming reverse motion...")
        
        # Button control: detect press-release to stop the test.
        current_state = button.value()
        if prev_button_state == 1 and current_state == 0:
            wait_for_button_release(button)
            print("Stop button detected. Exiting reverse sensor test.")
            running = False
        prev_button_state = current_state
        time.sleep(0.05)  # Short delay between iterations.
    
    # Ensure motors are turned off.
    orientation_controller.stop()
    motors.left.off()
    motors.right.off()
    print("Reverse sensor test complete.")

if __name__ == "__main__":
    test_reverse_with_sensor_stop()
