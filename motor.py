from machine import Pin, PWM
from utime import sleep

# Motor1 controls one motor (to be used as the right motor)
class Motor1:
    def __init__(self):
        # Initialize motor direction pin and PWM for speed control
        self.m1Dir = Pin(7, Pin.OUT)   # Motor 1 direction control pin
        self.pwm1 = PWM(Pin(6))        # Motor 1 PWM pin for speed control
        self.pwm1.freq(1000)           # Set PWM frequency to 1000 Hz
        self.pwm1.duty_u16(0)          # Start with 0 duty cycle (motor off)

    def off(self):
        """Turn off the motor."""
        self.pwm1.duty_u16(0)

    def forward(self, speed=100):
        """
        Run motor forward at a given speed.
        :param speed: Speed as a percentage (0-100)
        """
        self.m1Dir.value(0)  # Set direction to forward
        duty = int(65535 * speed / 100)
        self.pwm1.duty_u16(duty)

    def reverse(self, speed=100):
        """
        Run motor in reverse at a given speed.
        :param speed: Speed as a percentage (0-100)
        """
        self.m1Dir.value(1)  # Set direction to reverse
        duty = int(65535 * speed / 100)
        self.pwm1.duty_u16(duty)


# Motor2 controls the other motor (to be used as the left motor)
class Motor2:
    def __init__(self):
        # Initialize motor direction pin and PWM for speed control
        self.m1Dir = Pin(4, Pin.OUT)   # Motor 2 direction control pin
        self.pwm1 = PWM(Pin(5))        # Motor 2 PWM pin for speed control
        self.pwm1.freq(1000)           # Set PWM frequency to 1000 Hz
        self.pwm1.duty_u16(0)          # Start with 0 duty cycle (motor off)

    def off(self):
        """Turn off the motor."""
        self.pwm1.duty_u16(0)

    def forward(self, speed=100):
        """
        Run motor forward at a given speed.
        :param speed: Speed as a percentage (0-100)
        """
        self.m1Dir.value(0)  # Set direction to forward
        duty = int(65535 * speed / 100)
        self.pwm1.duty_u16(duty)

    def reverse(self, speed=100):
        """
        Run motor in reverse at a given speed.
        :param speed: Speed as a percentage (0-100)
        """
        self.m1Dir.value(1)  # Set direction to reverse
        duty = int(65535 * speed / 100)
        self.pwm1.duty_u16(duty)


# MotorPair class encapsulates coordinated control of left and right motors.
# Note: Motor2 is used as the left motor and Motor1 as the right motor.
class MotorPair:
    def __init__(self, left_motor, right_motor):
        self.left = left_motor   # Left motor (instance of Motor2)
        self.right = right_motor # Right motor (instance of Motor1)

    def move_forward(self, duration=0.5, speed=100):
        """Drive both motors forward at the specified speed."""
        self.left.forward(speed)
        self.right.forward(speed)
        sleep(duration)
        self.left.off()
        self.right.off()

    def move_backward(self, duration=0.5, speed=100):
        """Drive both motors in reverse at the specified speed."""
        self.left.reverse(speed)
        self.right.reverse(speed)
        sleep(duration)
        self.left.off()
        self.right.off()

    def turn_left(self, duration=0.5, speed=100):
        """
        Turn on the spot by running the left motor forward and the right motor in reverse.
        This results in a right turn.
        """
        self.left.forward(speed)
        self.right.reverse(speed)
        sleep(duration)
        self.left.off()
        self.right.off()

    def turn_right(self, duration=0.5, speed=100):
        """
        Turn on the spot by running the left motor in reverse and the right motor forward.
        This results in a left turn.
        """
        self.left.reverse(speed)
        self.right.forward(speed)
        sleep(duration)
        self.left.off()
        self.right.off()


if __name__ == "__main__":
    # Instantiate motors based on new hardware design:
    # Motor2 is used as the left motor, Motor1 is used as the right motor.
    left_motor = Motor2()
    right_motor = Motor1()
    
    # Create a MotorPair object for coordinated control
    car = MotorPair(left_motor, right_motor)
    
    # List of speeds (as percentages) to test
    test_speeds = [50, 75, 100]
    
    try:
        for speed in test_speeds:
            print(f"\n--- Testing at {speed}% speed ---")
            print("Moving forward")
            car.move_forward(duration=3, speed=speed)
            print("Turning left")
            car.turn_left(duration=3, speed=speed)
            print("Moving backward")
            car.move_backward(duration=3, speed=speed)
            print("Turning right")
            car.turn_right(duration=3, speed=speed)
    except KeyboardInterrupt:
        # Allow user to exit the program using Ctrl+C
        pass
