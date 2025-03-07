# Turning.py
import time

def turn_until_shift(motors, sensor_instance, orientation_controller, turn_type, increment=0.1, timeout=5, initial_delay=1.5):
    """
    Turn the robot (left or right) until the sensor readings indicate proper alignment.
    During the turning maneuver, the orientation controller is updated periodically so that the
    center sensors (the "nose") continuously try to stick to the path.
    
    Alignment is defined as:
      - center_left = 1, center_right = 1, and side sensors off (0).
    
    Args:
        motors: The MotorPair object controlling the robot.
        sensor_instance: The sensor instance (providing read_all() method).
        orientation_controller: The OrientationController instance (with update() method).
        turn_type: 'left' or 'right'.
        increment: Time in seconds for each turn step.
        timeout: Maximum time in seconds to try achieving the desired sensor pattern.
        initial_delay: Time to wait before starting to check sensor pattern (in seconds).
    """
    start_time = time.time()
    stable_time = 0.2  # Required stable duration for confirmation.
    pattern_stable_start = None
    desired_pattern = {'center_left': 1, 'center_right': 1, 'left_side': 0, 'right_side': 0}
    
    while time.time() - start_time < timeout:
        if turn_type == 'left':
            motors.turn_left(duration=increment, speed=50)
        elif turn_type == 'right':
            motors.turn_right(duration=increment, speed=50)
        else:
            raise ValueError("Invalid turn_type. Use 'left' or 'right'.")
        
        # Update orientation control during turning so that the "nose" remains on track.
        orientation_controller.update()
        
        # Only start checking the sensor pattern after the initial delay.
        if time.time() - start_time >= initial_delay:
            sensor_data = sensor_instance.read_all()
            if (sensor_data.get('center_left') == desired_pattern['center_left'] and
                sensor_data.get('center_right') == desired_pattern['center_right']):
                if pattern_stable_start is None:
                    pattern_stable_start = time.time()
                elif time.time() - pattern_stable_start >= stable_time:
                    print("Turn complete: Desired sensor pattern achieved.")
                    return
            else:
                pattern_stable_start = None
        
        time.sleep(increment)
    
    print("Turn timeout reached without achieving desired sensor pattern.")
