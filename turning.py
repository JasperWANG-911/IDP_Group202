# turning.py
import time
import sensor
from navigation import log  # Optionally a logging function

def turn_until_shift(turn_type, motors, speed=60, increment=0.1, timeout=3):
    """
    Turn in small increments until a 90° shift is detected.
    For a left turn, require left sensor active and front sensor off for 3 consecutive readings.
    For a right turn, require right sensor active and front sensor off for 3 consecutive readings.
    """
    start_time = time.time()
    consecutive_count = 0
    while time.time() - start_time < timeout:
        if turn_type == 'left':
            motors.turn_left(speed=speed, duration=increment)
        elif turn_type == 'right':
            motors.turn_right(speed=speed, duration=increment)
        sensor_data = sensor.get_track_sensor_pattern()
        if turn_type == 'left' and sensor_data['left'] == 1 and sensor_data['front'] == 0:
            consecutive_count += 1
        elif turn_type == 'right' and sensor_data['right'] == 1 and sensor_data['front'] == 0:
            consecutive_count += 1
        else:
            consecutive_count = 0
        if consecutive_count >= 3:
            print(f"{turn_type.capitalize()} turn complete based on sensor shift.")
            return
    print("Turn timeout reached without clear sensor shift.")
