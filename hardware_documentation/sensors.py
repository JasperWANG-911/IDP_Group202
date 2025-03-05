import time
import machine

class Sensors:
    """
    Sensor class for reading data from track, color, and ultrasonic sensors.
    """
    def __init__(self):
        # Initialize track sensor pins
        self.track_pins = {
            'front': machine.Pin(8, machine.Pin.IN),
            'right': machine.Pin(11, machine.Pin.IN),
            'rear': machine.Pin(9, machine.Pin.IN),
            'left': machine.Pin(10, machine.Pin.IN)
        }

        # Initialize the ultrasonic sensor pin (adjust the pin number based on your hardware)
        self.ultrasonic_pin = 1 # to be completed

    def read_track_sensor(self):
        return {key: pin.value() for key, pin in self.track_pins.items()}

    def read_ultrasonic_sensor(self):
        return {'distance': self.ultrasonic_pin.value()}

if __name__ == "__main__":
    sensor_test = Sensors()
    print("Starting sensor test. Press Ctrl-C to exit.")
    
    try:
        while True:
            # Read data from each sensor
            track_data = sensor_test.read_track_sensor()
            ultrasonic_data = sensor_test.read_ultrasonic_sensor()

            # Print the sensor data
            print("Track Sensor:", track_data)
            print("Ultrasonic Sensor:", ultrasonic_data)
            print("-" * 40)
            
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("Sensor test finished.")