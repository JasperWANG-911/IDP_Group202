from machine import Pin, PWM
from utime import sleep
import hardware_documentation.TOF_sensor
from hardware_documentation.tcs34725 import html_rgb

# Define actuator class
class Actuator:
    def __init__(self):
        self.m1Dir = Pin(0, Pin.OUT)  # set motor direction
        self.pwm1 = PWM(Pin(1))       # set speed
        self.pwm1.freq(1000)          # set max frequency 
        self.pwm1.duty_u16(0)         # set duty cycle

    def off(self):
        self.pwm1.duty_u16(0)

    def Forward(self):
        self.m1Dir.value(0)  # forward = 0, reverse = 1 for motor 1
        self.pwm1.duty_u16(int(65535 * 100 / 100))  # speed range 0-100 for motor 1

    def Reverse(self):
        self.m1Dir.value(1)
        self.pwm1.duty_u16(int(65535 * 30 / 100))

    def collection(self, motor_pair, TOF_sensor, colour_sensor) -> tuple[float, float, float, float]:
        """
        Grabs the box by performing the following steps:
        1. Stop the vehicle.
        2. Extend the actuator to engage the box.
        3. Move the vehicle forward to position the actuator.
        4. Retract the actuator to grab the box.
        5. Move the vehicle backward to complete the operation.
        """
        # Stop the vehicle in front of the box
        motor_pair.off()

        # Extend the actuator
        self.Forward()
        sleep(5)  # Adjust the extension time as needed
        self.off()

        # Move the vehicle forward to reach the box
        while TOF_sensor.read() > 5: # Adjust this for how far forward we need to travel
            motor_pair.move_forward()

        # Retract the actuator to grab the box
        self.Reverse()
        sleep(1)  # Adjust the retraction time as needed
        self.off()

        # Move the vehicle backward to clear the box
        motor_pair.move_backward(duration=5)  # Adjust the backward movement duration as needed
        rgb = html_rgb(colour_sensor.read(raw = True))
        return rgb # Ideally will return a string corresponding to colour, but need values for conditionals.
    


    def dropoff(self, motor_pair):
        """
        Reverse the process in grab_the_box() to drop the box.
        """
        # Stop the vehicle in front of the box
        motor_pair.off()

        # Extend the actuator
        self.Forward()
        sleep(1)  # Adjust the extension time as needed
        self.off()

        # Retract the actuator to set the extension length to default
        self.Reverse()
        sleep(5)  # Adjust the retraction time as needed
        self.off()

        # Leave the box-dropping by moving the vehicle backward
        motor_pair.move_backward(duration=5)
