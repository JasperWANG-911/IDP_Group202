# File: line_sensor.py
import machine
import time

class LineSensors:
    """
    Class for configuring and reading the four line sensors.
    The robot uses four sensors: left_side, center_left, center_right, and right_side.
    Adjust the pin numbers based on your hardware configuration.
    """
    def __init__(self):
        # Configure sensor pins (update pin numbers as needed)
        self.left_side = machine.Pin(10, machine.Pin.IN)
        self.center_left = machine.Pin(8, machine.Pin.IN)
        self.center_right = machine.Pin(9, machine.Pin.IN)
        self.right_side = machine.Pin(11, machine.Pin.IN)

    def read_all(self):
        """
        Reads the state of all four sensors.
        Returns:
            dict: Keys 'left_side', 'center_left', 'center_right', 'right_side'
        """
        return {
            'left_side': self.left_side.value(),
            'center_left': self.center_left.value(),
            'center_right': self.center_right.value(),
            'right_side': self.right_side.value()
        }

    def read_center(self):
        """
        Reads the state of the two center sensors.
        Returns:
            dict: Keys 'center_left' and 'center_right'
        """
        return {
            'center_left': self.center_left.value(),
            'center_right': self.center_right.value()
        }

if __name__ == "__main__":
    sensors = LineSensors()
    while True:
        print(sensors.read_all())
        time.sleep(0.5)
