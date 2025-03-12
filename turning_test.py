#!/usr/bin/env python3
import time
import machine
from motor import MotorPair, Motor1, Motor2  # Motor2 is left, Motor1 is right
from orientation_control import OrientationController
from line_sensor import LineSensors
from turning import turn_until_shift, turn_90

def wait_for_button_release(button):
    """
    Wait until the button is released.
    """
    while button.value() == 0:
        time.sleep(0.05)
    time.sleep(0.2)  # debounce delay

def test_turning_loop():
    """
    Test to simulate the vehicle continuously updating its PID control and sensor readings.
    The loop prints sensor data and waits for a stable T-cross condition 
    (either side sensor active for at least 0.1 seconds) to trigger a left turn.
    The test starts when you press and release the button on pin 20,
    and it stops either when the T-cross condition is met (and a turn is executed)
    or when you press the button to stop the test.
    """
    # Instantiate motors.
    left_motor = Motor2()
    right_motor = Motor1()
    motors = MotorPair(left_motor, right_motor)
    
    # Create an OrientationController with tuned PID parameters.
    orientation_controller = OrientationController(
        base_speed=75, k_p=10, k_i=0.2, k_d=8, deriv_window=10, integral_window=10
    )
    
    # Instantiate the line sensors.
    sensors = LineSensors()
    
    # Configure the button on pin 20 (active low, with internal pull-up).
    button = machine.Pin(20, machine.Pin.IN, machine.Pin.PULL_UP)
    
    print("Press and release the button on pin 20 to start the turning test.")
    # Wait for a complete press-release cycle to start.
    while button.value() == 0:
        time.sleep(0.05)
    while button.value() == 1:
        time.sleep(0.05)
    time.sleep(0.2)
    
    print("Turning test started. Continuously updating PID and sensor readings.")
    
    running = True
    pattern_stable_start = None  # Timestamp when T-cross pattern is first detected
    
    # Function to detect T-cross based on side sensors.
    def t_cross_detected(sensor_data):
        return sensor_data.get('left_side') == 1 or sensor_data.get('right_side') == 1
    
    prev_button_state = button.value()  # Typically 1 (not pressed)
    
    while running:
        # Update PID control and read sensor data.
        orientation_controller.update()
        sensor_data = sensors.read_all()
        print("Sensor data:", sensor_data)
        time.sleep(0.05)  # High sampling frequency
        
        # Check for T-cross detection.
        if t_cross_detected(sensor_data):
            if pattern_stable_start is None:
                pattern_stable_start = time.time()
            elif time.time() - pattern_stable_start >= 0:
                print("Stable T-cross detected. Preparing to turn left.")
                running = False  # Exit the loop to trigger turning.
                break
        else:
            pattern_stable_start = None  # Reset if the pattern is lost.
        
        # Also, check if the button is pressed to stop the test.
        current_state = button.value()
        if prev_button_state == 1 and current_state == 0:
            wait_for_button_release(button)
            print("Stop button detected. Ending turning test.")
            running = False
            break
        prev_button_state = current_state

    # If a stable T-cross was detected, execute a left turn.
    if pattern_stable_start is not None:
        turn_90(orientation_controller, sensors, turn_type='left',turn_time=2.5)
        print("Turn executed.")
    else:
        print("Test stopped without triggering a turn.")
        
        
    
    # Stop the controller and motors.
    orientation_controller.stop()
    motors.left.off()
    motors.right.off()
    
    print("Turning test complete.")

if __name__ == "__main__":
    test_turning_loop()
