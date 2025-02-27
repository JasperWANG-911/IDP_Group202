# turning.py
import time
import sensor_for_main as sensor  # Use the sensor module used in main

def turn_until_shift(turn_type, motors, speed=60, increment=0.1, required_count=3, timeout=3):
    """
    Turn in small increments until a 90° sensor shift is detected.
    For a left turn, the function requires that the left sensor is active and the front sensor is off 
    for a number of consecutive readings (default 3).
    For a right turn, it requires that the right sensor is active and the front sensor is off for the same
    number of consecutive readings.
    
    Both wheels spin in opposite directions during the turn.
    
    Parameters:
      turn_type: 'left' or 'right'
      motors: an instance of MotorPair (with both wheels configured for opposing spin during turns)
      speed: turning speed (percentage 0-100)
      increment: duration (in seconds) for each incremental turn command
      required_count: number of consecutive readings required to confirm a 90° shift
      timeout: maximum time (in seconds) to attempt turning before giving up
      
    Returns when the consecutive_count threshold is reached or after timeout.
    """
    start_time = time.time()
    consecutive_count = 0
    while time.time() - start_time < timeout:
        if turn_type == 'left':
            motors.turn_left(speed=speed, duration=increment)
        elif turn_type == 'right':
            motors.turn_right(speed=speed, duration=increment)
        sensor_data = sensor.get_track_sensor_pattern()
        if turn_type == 'left':
            if sensor_data['left'] == 1 and sensor_data['front'] == 0:
                consecutive_count += 1
            else:
                consecutive_count = 0
        elif turn_type == 'right':
            if sensor_data['right'] == 1 and sensor_data['front'] == 0:
                consecutive_count += 1
            else:
                consecutive_count = 0
        if consecutive_count >= required_count:
            print(f"{turn_type.capitalize()} turn complete based on sensor shift.")
            return
    print("Turn timeout reached without clear sensor shift.")
