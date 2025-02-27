import machine
import time

def get_sensor_pattern():
    """
    Read path sensor states and return a dictionary with keys:
      'front', 'right', 'rear', and 'left'.
    A value of 1 indicates that a path is detected.
    """
    return {
        'front': machine.Pin(8, machine.Pin.IN).value(),
        'right': machine.Pin(11, machine.Pin.IN).value(),
        'rear': machine.Pin(9, machine.Pin.IN).value(),
        'left': machine.Pin(10, machine.Pin.IN).value()
    }

if __name__ == "__main__":
    print("Starting sensor test.")
    print("Ensure the sensors are exposed to appropriate path signals (or simulate them).")
    while True:
        # Read the current sensor pattern.
        pattern = get_sensor_pattern()
        sensor_count = sum(pattern.values())
        # Print the sensor pattern and the number of active sensors.
        print("Sensor Pattern:", pattern, "Count:", sensor_count)
        time.sleep(0.5)
