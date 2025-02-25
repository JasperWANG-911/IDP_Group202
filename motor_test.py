import time
from motor import Motor  # Import the Motor class from motor.py

# ---------------- MotorPair Class ----------------
class MotorPair:
    def __init__(self, left_motor, right_motor):
        self.left = left_motor
        self.right = right_motor

    def move_forward(self, speed=60, duration=1.0):
        """
        Drive both motors forward.
        Speed is a percentage (0-100). Duration is in seconds.
        """
        self.left.forward(speed)
        self.right.forward(speed)
        time.sleep(duration)
        self.left.off()
        self.right.off()

    def move_backward(self, speed=60, duration=1.0):
        """
        Drive both motors in reverse.
        """
        self.left.reverse(speed)
        self.right.reverse(speed)
        time.sleep(duration)
        self.left.off()
        self.right.off()

    def turn_left(self, speed=60, duration=1.0):
        """
        Turn in place to the left by stopping the left motor
        and driving the right motor forward.
        """
        self.left.off()
        self.right.forward(speed)
        time.sleep(duration)
        self.left.off()
        self.right.off()

    def turn_right(self, speed=60, duration=1.0):
        """
        Turn in place to the right by driving the left motor forward
        and stopping the right motor.
        """
        self.left.forward(speed)
        self.right.off()
        time.sleep(duration)
        self.left.off()
        self.right.off()

# ---------------- Main Motor Test ----------------
if __name__ == "__main__":
    # Instantiate motors.
    # For integration we assume:
    #   - Left motor: direction controlled by pin 3, PWM on pin 2.
    #   - Right motor: direction controlled by pin 5, PWM on pin 4.
    left_motor = Motor(dir_pin=3, pwm_pin=2)
    right_motor = Motor(dir_pin=5, pwm_pin=4)
    motors = MotorPair(left_motor, right_motor)

    print("Testing motor forward")
    motors.move_forward(speed=60, duration=1.0)
    time.sleep(1)

    print("Testing motor reverse")
    motors.move_backward(speed=60, duration=1.0)
    time.sleep(1)

    print("Testing motor turn left")
    motors.turn_left(speed=60, duration=1.0)
    time.sleep(1)

    print("Testing motor turn right")
    motors.turn_right(speed=60, duration=1.0)
    time.sleep(1)

    print("Motor test complete")
