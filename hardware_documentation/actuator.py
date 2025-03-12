from machine import Pin, PWM
from utime import sleep
import motor
from machine import Pin, I2C
from vl53l0x import VL53L0X
from machine import Pin
from machine import I2C
from tcs34725 import TCS34725
import tcs34725 as tc

i2c_bus = I2C(0, sda=Pin(16), scl=Pin(17))
tcs = TCS34725(i2c_bus)

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

    def grab_the_box(self, motor_pair):
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
        sleep(2)  # Adjust the extension time as needed
        self.off()

        # Move the vehicle forward to reach the box
        motor_pair.move_forward(duration=1)  # Adjust the forward movement duration as needed

        # Retract the actuator to grab the box
        self.Reverse()
        sleep(0.5)  # Adjust the retraction time as needed
        self.off()

        # Move the vehicle backward to clear the box
        motor_pair.move_backward(duration=1)  # Adjust the backward movement duration as needed

    def drop_the_box(self, motor_pair):
        """
        Reverse the process in grab_the_box() to drop the box.
        """
        # Stop the vehicle in front of the box
        motor_pair.off()

        # Extend the actuator
        self.Forward()
        sleep(0.5)  # Adjust the extension time as needed
        self.off()

        # Retract the actuator to set the extension length to default
        self.Reverse()
        sleep(2)  # Adjust the retraction time as needed
        self.off()

        # Leave the box-dropping by moving the vehicle backward
        motor_pair.move_backward(duration=1)


# Setting up ToF sensor
i2c = I2C(id=1, sda=Pin(18), scl=Pin(19))
tof = VL53L0X(i2c)

budget = tof.measurement_timing_budget_us
print("Budget was:", budget)
tof.set_measurement_timing_budget(4000000000)

tof.set_Vcsel_pulse_period(tof.vcsel_period_type[0], 12) # numbers 12 and 8(below) could be changed, see Moodle tof_test.py
tof.set_Vcsel_pulse_period(tof.vcsel_period_type[1], 8)

# Setting up color sensor
i2c_bus = I2C(0, sda=Pin(16), scl=Pin(17), freq = 400000)
tcs = tc.TCS34725(i2c_bus)


# Example use
if __name__ == "__main__":
    # Set up motors and sensors
    actuator = Actuator()
    motor1 = motor.Motor1()
    motor2 = motor.Motor2()
    motor_pair = motor.MotorPair(motor1, motor2)
    
    while tof.ping() > 120:
        print('safe')
    else:
        # Grab the box when it is detected by ToF sensor
        print('box detected')
        Actuator.grab_the_box(actuator, motor_pair)
        data = tcs.read(True)
        tcs.gain(60)
        # read the color
        R_value = tc.html_rgb(data) '''find tc.html_rgb(data) type'''
        if R_value < 5: '''calibrate this number'''
            box_index = 1 # box A/B
        else:
            box_index = 2 # box C/D
            
    # now go back to the box-dropping zone (add code here)
    # ....
    
    # drop the box once it arrrives box-dropping zone
    motor_pair.move_forward(duration=2) # go a bit further to enter the zone
    Actuator.drop_the_box(actuator, motor_pair)
        
        
        
        

            
    
