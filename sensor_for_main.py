from sensor import Sensors  # Import the proven sensor class from sensor.py

# Instantiate a single Sensors object to be used throughout the program
_sensor_instance = Sensors()

def get_track_sensor_pattern():
    """
    Read track sensor states using the Sensors class.
    Returns a dictionary with keys: 'front', 'right', 'rear', and 'left'.
    """
    return _sensor_instance.read_track_sensor()

def get_color_sensor_pattern():
    """
    Read color sensor data using the Sensors class.
    Returns a dictionary with key 'color'.
    """
    return _sensor_instance.read_color_sensor()

def get_ultrasonic_sensor_reading():
    """
    Read ultrasonic sensor data using the Sensors class.
    Returns a dictionary with key 'distance'.
    """
    return _sensor_instance.read_ultrasonic_sensor()

if __name__ == "__main__":
    import time
    print("Starting sensor test. Ensure your sensors are correctly connected.")
    while True:
        track_pattern = get_track_sensor_pattern()
        color_pattern = get_color_sensor_pattern()
        ultrasonic_reading = get_ultrasonic_sensor_reading()
        
        print("Track Sensor Pattern:", track_pattern)
        print("Color Sensor Pattern:", color_pattern)
        print("Ultrasonic Sensor Reading:", ultrasonic_reading)
        time.sleep(0.5)
