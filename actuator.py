from machine import Pin, PWM
from utime import sleep
import Other_Sensor
import 

# Define actuator class
class Actuator:
    def __init__(self):
        self.m1Dir = Pin(0 , Pin.OUT) # set motor direction
        self.pwm1 = PWM(Pin(1)) # set speed
        self.pwm1.freq(1000) # set max frequency 
        self.pwm1.duty_u16(0) # set duty cycle
    def off(self):
        self.pwm1.duty_u16(0)
    def Forward(self):
        self.m1Dir.value(0) # forward = 0 reverse = 1 motor 1
        self.pwm1.duty_u16(int(65535*100/100)) # speed range 0-100 motor 1
    def Reverse(self):
        self.m1Dir.value(1)
        self.pwm1.duty_u16(int(65535*30/100))
        
# once a color detected, control the actuator
actuator=Actuator()
sensor = Other_Sensor.Sensors()

def grab_the_box():
    """
    Grabs the box by performing the following steps:
    1. Stop the vehicle.
    2. Extend the actuator to engage the box.
    3. Move the vehicle forward to position the actuator.
    4. Retract the actuator to grab the box.
    5. Move the vehicle backward to complete the operation.
    """
    # Stop the vehicle in front of the box
    motor.off()

    # Extend the actuator
    actuator.Forward()
    sleep(5)  # Adjust the extension time as needed
    actuator.off()

    # Move the vehicle forward to reach the box
    motor.move_forward(duration=5)  # Adjust the forward movement duration as needed

    # Retract the actuator to grab the box
    actuator.Reverse()
    sleep(1)  # Adjust the retraction time as needed
    actuator.off()

    # Move the vehicle backward to clear the box
    motor.move_backward(duration=5)  # Adjust the backward movement duration as needed


