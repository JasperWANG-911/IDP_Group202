import time
from motor import Motor1, Motor2

# MotorPair class encapsulates the left and right motors
class MotorPair:
    def __init__(self, left_motor, right_motor):
        self.left = left_motor
        self.right = right_motor

    def move_forward(self, duration=0.5):
        # Drive both motors forward for the given duration
        self.left.Forward()
        self.right.Forward()
        time.sleep(duration)
        self.left.off()
        self.right.off()

    def move_backward(self, duration=0.5):
        # Drive both motors in reverse for the given duration
        self.left.Reverse()
        self.right.Reverse()
        time.sleep(duration)
        self.left.off()
        self.right.off()

    def turn_right(self, duration=0.5):
        # Turn on the spot to the left (left motor reverse, right motor forward)
        self.left.Reverse()
        self.right.Forward()
        time.sleep(duration)
        self.left.off()
        self.right.off()

    def turn_left(self, duration=0.5):
        # Turn on the spot to the right (left motor forward, right motor reverse)
        self.left.Forward()
        self.right.Reverse()
        time.sleep(duration)
        self.left.off()
        self.right.off()
