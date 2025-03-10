import time

def turn_until_shift(orientation_controller, sensor_instance, turn_type, base_increment=0.1, timeout=5, initial_delay=1.5, turning_base_speed=50, turning_sensitivity=2):
    """
    Turn the robot by toggling the action type in the orientation controller until the sensor readings 
    indicate proper alignment. This method automatically updates the orientation controller parameters for turning 
    (using a lower base_speed and higher sensitivity) before executing the turn, and reverts them back after the turn.
    
    When the action type changes, the PID speed correction (i.e., the PID's internal error and integral history) 
    is zeroed by clearing the corresponding windows. Once turning is done, the action type is reset to "straight" 
    and the PID state is cleared.
    
    Args:
        orientation_controller: The OrientationController instance (with update() method and action_type property).
        sensor_instance: The sensor instance providing the read_all() method.
        turn_type: 'left' or 'right', used to set the action type for turning.
        base_increment: The constant time delay (in seconds) for each update cycle.
        timeout: Maximum time (in seconds) to try achieving the desired sensor pattern.
        initial_delay: Time (in seconds) to wait before starting to check the sensor pattern.
        turning_base_speed: The base speed to use during turning.
        turning_sensitivity: The sensitivity to use during turning.
    """
    # Save the original orientation controller parameters.
    original_base_speed = orientation_controller.base_speed
    original_sensitivity = orientation_controller.sensitivity
    
    # Update orientation controller parameters for turning.
    orientation_controller.base_speed = turning_base_speed
    orientation_controller.sensitivity = turning_sensitivity

    # Stop the vehicle using the controller's stop method before turning.
    orientation_controller.stop()
    time.sleep(0.2)  # Allow time for the vehicle to come to a complete stop.

    # Set action type to desired turn type if not already set, and reset the PID state.
    if orientation_controller.action_type != turn_type:
        orientation_controller.action_type = turn_type
        orientation_controller.pid.error_window.clear()      # Reset derivative history.
        orientation_controller.pid.integral_window.clear()   # Reset integral history.

    start_time = time.time()
    stable_time = 0.2  # Duration (in seconds) the sensor pattern must be stable for confirmation.
    pattern_stable_start = None
    desired_pattern = {'center_left': 1, 'center_right': 1, 'left_side': 0, 'right_side': 0}

    while time.time() - start_time < timeout:
        # If the action type has unexpectedly changed, reset it and clear the PID state.
        if orientation_controller.action_type != turn_type:
            orientation_controller.action_type = turn_type
            orientation_controller.pid.error_window.clear()
            orientation_controller.pid.integral_window.clear()

        # Update the orientation controller so that the PID adjusts motor speeds based on the current action type.
        orientation_controller.update()

        # After the initial delay, check if the sensor readings match the desired alignment pattern.
        if time.time() - start_time >= initial_delay:
            sensor_data = sensor_instance.read_all()
            if (sensor_data.get('center_left') == desired_pattern['center_left'] and
                sensor_data.get('center_right') == desired_pattern['center_right']):
                if pattern_stable_start is None:
                    pattern_stable_start = time.time()
                elif time.time() - pattern_stable_start >= stable_time:
                    print("Turn complete: Desired sensor pattern achieved.")
                    # Reset the action type to "straight" and clear PID state after turning is done.
                    orientation_controller.action_type = "straight"
                    orientation_controller.pid.error_window.clear()
                    orientation_controller.pid.integral_window.clear()
                    
                    # Revert to the original parameters.
                    orientation_controller.base_speed = original_base_speed
                    orientation_controller.sensitivity = original_sensitivity
                    return
            else:
                pattern_stable_start = None

        time.sleep(base_increment)

    print("Turn timeout reached without achieving desired sensor pattern.")
    # In case of a timeout, reset the action type, clear the PID state, and revert parameters.
    orientation_controller.action_type = "straight"
    orientation_controller.pid.error_window.clear()
    orientation_controller.pid.integral_window.clear()
    orientation_controller.base_speed = original_base_speed
    orientation_controller.sensitivity = original_sensitivity
