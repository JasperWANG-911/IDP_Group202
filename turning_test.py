#!/usr/bin/env python3
import time
import machine
from motor import MotorPair, Motor1, Motor2  # Motor2 is left, Motor1 is right
from orientation_control import OrientationController
from line_sensor import LineSensors
from turning import turn_until_shift

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
    (defined by both side sensors active), then executing a left turn.
    
    The PID controller remains active during the entire motion, including during the turn.
    The test starts when you press and release the button on pin 20,
    and you can also press the button to stop the test.
    """
    # Instantiate motors: Motor2 is used as left, Motor1 as right.
    left_motor = Motor2()
    right_motor = Motor1()
    motors = MotorPair(left_motor, right_motor)
    
    # Create the "straight" orientation controller (for driving straight)
    orientation_controller = OrientationController(
        base_speed=75, k_p=15, k_i=0.05, k_d=5, deriv_window=10, integral_window=10
    )
    # Create a separate orientation controller for turning.
    # Here, we set a lower base speed (50), sensitivity factor 2, and action_type "left".
    orientation_controller_turning = OrientationController(
        base_speed=50, k_p=15, k_i=0.05, k_d=5, deriv_window=10, integral_window=10, 
        sensitivity=2, action_type="left"
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
        return sensor_data.get('left_side') == 1 and sensor_data.get('right_side') == 1

    # Drive forward until the T-cross condition is met.
    while running:
        orientation_controller.update()  # continuously update straight-line PID control
        time.sleep(0.1)
        sensor_data = sensors.read_all()
        print("Sensor readings:", sensor_data)
        if t_cross_detected(sensor_data):
            print("T-cross detected based on side sensors. Preparing to turn left.")
            break
        # Check if the button is pressed to stop the test.
        if button.value() == 0:
            wait_for_button_release(button)
            print("Stop button pressed. Ending test.")
            running = False
            orientation_controller.stop()
            motors.left.off()
            motors.right.off()
            return

    # Execute a left turn. The turning orientation controller is active during the turn.
    # Note: initial_delay parameter gives a pulse delay after turning starts before we check the sensor pattern.
    turn_until_shift(motors, sensors, orientation_controller_turning, turn_type='left', increment=0.1, timeout=5, initial_delay=1.5)
    
    # After the turn, continue forward for a short period.
    print("Turn complete. Driving forward for 2 seconds after turn.")
    start_time = time.time()
    while time.time() - start_time < 2:
        orientation_controller.update()
        time.sleep(0.1)
    
    # Stop everything.
    orientation_controller.stop()
    orientation_controller_turning.stop()
    motors.left.off()
    motors.right.off()
    
    print("Turning test complete.")

if __name__ == "__main__":
    test_turning()
