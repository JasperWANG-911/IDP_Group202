import time
from motor import Motor1, Motor2  # Import the modified motor classes

# ---------------- MotorPair Class ----------------
class MotorPair:
    def __init__(self, left_motor, right_motor):
        self.left = left_motor
        self.right = right_motor

    def move_forward(self, duration=1.0):
        """
        Drive both motors forward.
        """
        self.left.Forward()
        self.right.Forward()
        time.sleep(duration)
        self.left.off()
        self.right.off()

    def move_backward(self, duration=1.0):
        """
        Drive both motors in reverse.
        """
        self.left.Reverse()
        self.right.Reverse()
        time.sleep(duration)
        self.left.off()
        self.right.off()

    def turn_left(self, duration=1.0):
        """
        Turn on the spot to the left by driving the left motor in reverse
        and the right motor forward.
        """
        self.left.Reverse()
        self.right.Forward()
        time.sleep(duration)
        self.left.off()
        self.right.off()

    def turn_right(self, duration=1.0):
        """
        Turn on the spot to the right by driving the left motor forward
        and the right motor in reverse.
        """
        self.left.Forward()
        self.right.Reverse()
        time.sleep(duration)
        self.left.off()
        self.right.off()

# ---------------- Main Motor Test ----------------
if __name__ == "__main__":
    # Instantiate the motors.
    # Motor1 is used for the left motor and Motor2 for the right motor.
    left_motor = Motor1()
    right_motor = Motor2()
    motors = MotorPair(left_motor, right_motor)

    print("Testing motor forward")
    motors.move_forward(duration=1.0)
    time.sleep(1)

    print("Testing motor reverse")
    motors.move_backward(duration=1.0)
    time.sleep(1)

    print("Testing motor turn left (spinning on the spot)")
    motors.turn_left(duration=1.0)
    time.sleep(1)

    print("Testing motor turn right (spinning on the spot)")
    motors.turn_right(duration=1.0)
    time.sleep(1)

    print("Motor test complete")
