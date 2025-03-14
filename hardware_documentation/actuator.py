from machine import Pin, PWM
from utime import sleep
import time
import motor
from machine import Pin, I2C
from vl53l0x import VL53L0X
from machine import Pin
from machine import I2C
import tcs34725 as tc
from orientation_control import OrientationController
from line_sensor import LineSensors
# Define actuator class
class Actuator:
    def __init__(self):
        self.m1Dir = Pin(0, Pin.OUT)  # set motor direction
        self.pwm1 = PWM(Pin(1))       # set speed
        self.pwm1.freq(1000)          # set max frequency 
        self.pwm1.duty_u16(0)         # set duty cycle
        self.orientation_controller = OrientationController(base_speed=75, k_p=10, k_i=0.2, k_d=8)

    def off(self):
        self.pwm1.duty_u16(0)

    def Forward(self):
        self.m1Dir.value(0)  # forward = 0, reverse = 1 for motor 1
        self.pwm1.duty_u16(int(65535 * 100 / 100))  # speed range 0-100 for motor 1

    def Reverse(self):
        self.m1Dir.value(1)
        self.pwm1.duty_u16(int(65535 * 100 / 100))
        
    def controlled_move_forward(self, duration, update_interval=0.1):
        """Drive forward for the given duration while updating the controller."""
        start_time = time.time()
        while time.time() - start_time < duration:
            self.orientation_controller.update()
            time.sleep(update_interval)

    def controlled_move_backward(self, duration, update_interval=0.1):
        """Drive backward for the given duration while updating reverse control."""
        start_time = time.time()
        while time.time() - start_time < duration:
            self.orientation_controller.update_reverse()
            time.sleep(update_interval)
    def grab_the_box(self, motor_pair):
        # Stop the vehicle in front of the box
        motor_pair.off()
        
        # extend the actuator
        self.Forward()
        sleep(3.3) 
        self.off()



        # Retract the actuator to grab the box
        self.Reverse()
        sleep(3.3)
        self.off()


    def drop_the_box(self, motor_pair):
        # Stop the vehicle in front of the box
        motor_pair.off()

        # Extend the actuator
        self.Forward()
        sleep(3.3) 
        self.off()
    

        # Retract the actuator to set the extension length to default
        self.Reverse()
        sleep(3.3)
        self.off()
        
        
        
        
        

            
    
