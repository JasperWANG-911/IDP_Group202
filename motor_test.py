#!/usr/bin/env python3
import time
from machine import Pin, PWM
from utime import sleep

# -----------------------------
# Motor Implementation
# -----------------------------
class Motor1:
    """
    Controls Motor1 using direction on Pin 7 and PWM on Pin 6.
    """
    def __init__(self):
        self.m1Dir = Pin(7, Pin.OUT)  # Motor direction pin
        self.pwm1 = PWM(Pin(6))       # PWM control pin
        self.pwm1.freq(1000)          # Set PWM frequency to 1 kHz
        self.pwm1.duty_u16(0)         # Initially off

    def off(self):
        self.pwm1.duty_u16(0)

    def Forward(self):
        # Forward direction: set direction pin to 0
        self.m1Dir.value(0)
        # Full speed: 100% duty cycle
        self.pwm1.duty_u16(65535)

    def Reverse(self):
        # Reverse direction: set direction pin to 1
        self.m1Dir.value(1)
        # Reverse at 30% duty cycle for safer backward motion
        self.pwm1.duty_u16(int(65535 * 0.3))


class Motor2:
    """
    Controls Motor2 using direction on Pin 4 and PWM on Pin 5.
    """
    def __init__(self):
        self.m1Dir = Pin(4, Pin.OUT)  # Motor direction pin
        self.pwm1 = PWM(Pin(5))       # PWM control pin
        self.pwm1.freq(1000)          # Set PWM frequency to 1 kHz
        self.pwm1.duty_u16(0)         # Initially off

    def off(self):
        self.pwm1.duty_u16(0)

    def Forward(self):
        self.m1Dir.value(0)
        self.pwm1.duty_u16(65535)

    def Reverse(self):
        self.m1Dir.value(1)
        self.pwm1.duty_u16(int(65535 * 0.3))


class MotorPair:
    """
    Combines two motors to allow coordinated movement.
    """
    def __init__(self, left_motor, right_motor):
        self.left = left_motor
        self.right = right_motor

    def move_forward(self, duration=0.5):
        print("MotorPair: Moving forward")
        self.left.Forward()
        self.right.Forward()
        time.sleep(duration)
        self.left.off()
        self.right.off()

    def move_backward(self, duration=0.5):
        print("MotorPair: Moving backward")
        self.left.Reverse()
        self.right.Reverse()
        time.sleep(duration)
        self.left.off()
        self.right.off()

    def turn_left(self, duration=0.5):
        print("MotorPair: Turning left")
        # Left motor in reverse, right motor forward for an in-place turn
        self.left.Reverse()
        self.right.Forward()
        time.sleep(duration)
        self.left.off()
        self.right.off()

    def turn_right(self, duration=0.5):
        print("MotorPair: Turning right")
        # Left motor forward, right motor in reverse for an in-place turn
        self.left.Forward()
        self.right.Reverse()
        time.sleep(duration)
        self.left.off()
        self.right.off()


# -----------------------------
# Main Hardware Test Routine
# -----------------------------
def main():
    print("Starting hardware motor test.")
    
    # Initialize motors and create a motor pair
    left_motor = Motor1()
    right_motor = Motor2()
    motors = MotorPair(left_motor, right_motor)
    
    # Test move forward
    print("Test: Move forward for 1 second.")
    motors.move_forward(duration=5)
    time.sleep(1)
    
    # Test move backward
    print("Test: Move backward for 1 second.")
    motors.move_backward(duration=5)
    time.sleep(1)
    
    # Test turn left
    print("Test: Turn left for 1 second.")
    motors.turn_left(duration=5)
    time.sleep(1)
    
    # Test turn right
    print("Test: Turn right for 1 second.")
    motors.turn_right(duration=5)
    time.sleep(1)
    
    print("Hardware motor test complete.")

if __name__ == "__main__":
    main()

