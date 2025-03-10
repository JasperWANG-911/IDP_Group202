#!/usr/bin/env python3
import time
import machine
from motor import MotorPair, Motor1, Motor2  # Motor2 is left, Motor1 is right
from orientation_control import OrientationController
from line_sensor import LineSensors
from turning import turn_until_shift  # Import the updated turn_until_shift

def wait_for_button_release(button):
    """
    Wait until the button is released.
    """
    while button.value() == 0:
        time.sleep(0.05)
    time.sleep(0.2)  # debounce delay

def test_turning():
    """
    Test to simulate the vehicle following a straight line until a T-cross is detected
    (defined by side sensors active for a stable period), then executing a left turn using a single orientation controller.
    
    The PID controller remains active during the entire motion, including during the turn.
    The test starts when you press and release the button on pin 20,
    and you can also press the button to stop the test.
    """
    # Instantiate motors: Motor2 is used as left, Motor1 as right.
    left_motor = Motor2()
    right_motor = Motor1()
    motors = MotorPair(left_motor, right_motor)
    
    # Create a single orientation controller for both straight driving and turning.
    orientation_controller = OrientationController(
        base_speed=75, k_p=10, k_i=0.2, k_d=8, deriv_window=10, integral_window=10
    )
    
    # Instantiate the line sensors.
    sensors = LineSensors()
    
    # Configure button on pin 20 (active low, with internal pull-up).
    button = machine.Pin(20, machine.Pin.IN, machine.Pin.PULL_UP)
    
    print("Press and release the button on pin 20 to start the turning test.")
    # Wait for a complete press-release cycle to start.
    while button.value() == 0:
        time.sleep(0.05)
    while button.value() == 1:
        time.sleep(0.05)
    time.sleep(0.2)  # debounce delay
    
    print("Test started. Driving forward with continuous PID updates until T-cross detected.")
    
    running = True
    # Define the T-cross condition based solely on the side sensors.
    def t_cross_detected(sensor_data):
        return sensor_data.get('left_side') == 1 or sensor_data.get('right_side') == 1

    pattern_stable_start = None  # Timestamp when T-cross pattern was first seen

    # Drive forward until the T-cross condition is met stably.
    while running:
        orientation_controller.update()  # continuously update straight-line PID control
        time.sleep(0.05)  # increased sampling frequency
        sensor_data = sensors.read_all()
        print("Sensor readings:", sensor_data)
        
        if t_cross_detected(sensor_data):
            # If this is the first time we see a T-cross pattern, record the time.
            if pattern_stable_start is None:
                pattern_stable_start = time.time()
            # If the pattern has been stable for at least 0.1 seconds, trigger the turn.
            elif time.time() - pattern_stable_start >= 0.1:
                print("T-cross detected based on stable sensor pattern. Preparing to turn left.")
                break
        else:
            # If the pattern is lost, reset the stable timer.
            pattern_stable_start = None

        # Check if the button is pressed to stop the test.
        if button.value() == 0:
            wait_for_button_release(button)
            print("Stop button pressed. Ending test.")
            running = False
            orientation_controller.stop()
            motors.left.off()
            motors.right.off()
            return

    # Execute a left turn using the single orientation controller.
    # The integrated turn_until_shift function updates controller parameters for turning and reverts them afterward.
    turn_until_shift(orientation_controller, sensors, turn_type='left', base_increment=0.1, timeout=5, initial_delay=1.5)
    
    # After the turn, continue forward for a short period.
    print("Turn complete. Driving forward for 2 seconds after turn.")
    start_time = time.time()
    while time.time() - start_time < 2:
        orientation_controller.update()
        time.sleep(0.1)
    
    # Stop everything.
    orientation_controller.stop()
    motors.left.off()
    motors.right.off()
    
    print("Turning test complete.")

if __name__ == "__main__":
    test_turning()
