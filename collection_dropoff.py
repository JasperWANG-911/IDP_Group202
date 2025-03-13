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

def collection(motor_pair, actuator, TOF_sensor, colour_sensor) -> str:
    """
    Checks box distance.
    When box is within a threshold distance, grab_the_box function is triggered.
    After triggering grab_the_box, the colour of the box is determined and
    corresponding colour string is returned.
    """
    while TOF_sensor.ping() > 150:  
        print(TOF_sensor.ping(), 'safe')

    else:
        # Grab the box when it is detected by ToF sensor
        print('box detected')
        actuator.grab_the_box(motor_pair)
        data = colour_sensor.read(True)
        colour_sensor.gain(60)
        # read the color by R_value
        R_value = html_rgb(data)[0]
        if R_value < 7: 
            colour = 'BG' # box A/B
        else:
            colour = 'RY' # box C/D
    return colour

# To drop box, call actuator.drop_the_box

def drop_off(motor_pair, actuator, TOF_sensor):
    actuator.drop_the_box(motor_pair)
    if TOF_sensor.ping > 20:
        return True

