# Turning.py
import time

def turn_until_shift(motors, sensor_instance, orientation_controller, turn_type, increment=0.1, timeout=3):
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
    """
    start_time = time.time()
    stable_time = 0.2  # Required stable duration for confirmation.
    pattern_stable_start = None
    desired_pattern = {'center_left': 1, 'center_right': 1, 'left_side': 0, 'right_side': 0}
    
    while time.time() - start_time < timeout:
        if turn_type == 'left':
            motors.turn_left(duration=increment)
        elif turn_type == 'right':
            motors.turn_right(duration=increment)
        else:
            raise ValueError("Invalid turn_type. Use 'left' or 'right'.")
        
        # Update orientation control during turning so that the "nose" remains on track.
        orientation_controller.update()
        
        sensor_data = sensor_instance.read_all()
        if (sensor_data.get('center_left') == desired_pattern['center_left'] and
            sensor_data.get('center_right') == desired_pattern['center_right'] and
            sensor_data.get('left_side') == desired_pattern['left_side'] and
            sensor_data.get('right_side') == desired_pattern['right_side']):
            if pattern_stable_start is None:
                pattern_stable_start = time.time()
            elif time.time() - pattern_stable_start >= stable_time:
                print("Turn complete: Desired sensor pattern achieved.")
                return
        else:
            pattern_stable_start = None
        
        time.sleep(increment)
    
    print("Turn timeout reached without achieving desired sensor pattern.")

def perform_reverse_turn(motors, orientation_controller, duration=0.5):
    """
    Execute a reverse maneuver with orientation correction.
    Uses the orientation controller's update_reverse method for the specified duration.
    """
    start_time = time.time()
    while time.time() - start_time < duration:
        orientation_controller.update_reverse()
        time.sleep(0.05)
    motors.left.off()
    motors.right.off()
