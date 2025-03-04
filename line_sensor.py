import machine
import time

class LineSensors:
    """
    Class for configuring and reading the two middle line sensors.
    Adjust the pin numbers based on your hardware configuration.
    """
    def __init__(self):
        # Configure the two middle sensors (only these participate in orientation control)
        self.center_left = machine.Pin(8, machine.Pin.IN)
        self.center_right = machine.Pin(9, machine.Pin.IN)

    def read(self):
        """
        Reads the state of the two middle sensors.
        Returns:
            dict: A dictionary with keys 'center_left' and 'center_right' containing sensor states.
        """
        return {
            'center_left': self.center_left.value(),
            'center_right': self.center_right.value()
        }

if __name__ == "__main__":
    sensors = LineSensors()
    while True:
        print(sensors.read())
        time.sleep(0.5)
