#!/usr/bin/env python3
import time
from motor import MotorPair, Motor1, Motor2  # Motor2 is left, Motor1 is right
from line_sensor import LineSensors

class PIDController:
    def __init__(self, k_p, k_i, k_d):
        # Initialize PID constants and internal state variables.
        self.k_p = k_p
        self.k_i = k_i
        self.k_d = k_d
        self.integral = 0.0
        self.last_error = 0.0

    def compute(self, error):
        """
        Compute PID correction based on the current error.
        :param error: The current error value.
        :return: The PID correction value.
        """
        self.integral += error
        derivative = error - self.last_error
        self.last_error = error
        return self.k_p * error + self.k_i * self.integral + self.k_d * derivative

def clamp_speed(speed):
    """
    Clamp the motor speed within the range [-100, 100].
    :param speed: Proposed motor speed.
    :return: Clamped motor speed.
    """
    return max(-100, min(100, speed))

def set_motor_speed(motor, speed):
    """
    Set motor speed. If speed is positive, use forward;
    if negative, use reverse (with absolute value); if zero, stop.
    """
    if speed > 0:
        motor.forward(speed)
    elif speed < 0:
        motor.reverse(abs(speed))
    else:
        motor.off()

class OrientationController:
    """
    OrientationController integrates sensor readings, PID control, and motor commands
    to adjust the robot's heading along a line. It uses two center sensors for error calculation.
    It supports both forward and reverse control by allowing negative speeds.
    """
    def __init__(self, base_speed=75, k_p=20, k_i=0.5, k_d=10):
        self.sensors = LineSensors()
        self.left_motor = Motor2()   # Left motor (Motor2)
        self.right_motor = Motor1()  # Right motor (Motor1)
        self.car = MotorPair(self.left_motor, self.right_motor)
        self.pid = PIDController(k_p, k_i, k_d)
        self.base_speed = base_speed
        # Last valid error to use when sensor data is absent.
        self.last_valid_error = 0.0

    def get_line_error(self):
        """
        Compute the line error using only the two center sensors.
        Error rules:
          - Both active: 0.0 (aligned)
          - Only center_left active: -1.0 (deviated left)
          - Only center_right active: 1.0 (deviated right)
          - Both inactive: use last valid error.
        """
        sensor_values = self.sensors.read_center()
        left_active = sensor_values.get('center_left', 0)
        right_active = sensor_values.get('center_right', 0)

        if left_active and right_active:
            error = 0.0
        elif left_active and not right_active:
            error = -1.0
        elif right_active and not left_active:
            error = 1.0
        else:
            error = self.last_valid_error

        if left_active or right_active:
            self.last_valid_error = error

        return error

    def update(self):
        """
        Perform one forward control update cycle:
          - Read sensor data.
          - Compute PID correction.
          - Adjust motor speeds (using positive speeds for forward movement).
        """
        error = self.get_line_error()
        correction = self.pid.compute(error)
        # Calculate new speeds based on base_speed and correction.
        left_speed = clamp_speed(self.base_speed + correction)
        right_speed = clamp_speed(self.base_speed - correction)
        print("Forward Update -> Error: {:.2f}, Correction: {:.2f}, Left Speed: {:.2f}, Right Speed: {:.2f}".format(
            error, correction, left_speed, right_speed))
        set_motor_speed(self.left_motor, left_speed)
        set_motor_speed(self.right_motor, right_speed)

    def update_reverse(self):
        """
        Perform one reverse control update cycle:
          - Read sensor data.
          - Compute PID correction.
          - Adjust motor speeds for reverse movement (base_speed is considered negative).
        """
        error = self.get_line_error()
        correction = self.pid.compute(error)
        # For reverse, subtract correction from base_speed.
        left_speed = clamp_speed(self.base_speed - correction)
        right_speed = clamp_speed(self.base_speed + correction)
        print("Reverse Update -> Error: {:.2f}, Correction: {:.2f}, Left Speed: {:.2f}, Right Speed: {:.2f}".format(
            error, correction, left_speed, right_speed))
        set_motor_speed(self.left_motor, left_speed)
        set_motor_speed(self.right_motor, right_speed)

    def run(self):
        """
        Continuously run the forward control update loop until interrupted.
        """
        try:
            while True:
                self.update()
                time.sleep(0.1)
        except KeyboardInterrupt:
            self.stop()
            print("Orientation control terminated.")

    def stop(self):
        """
        Stop both motors.
        """
        self.left_motor.off()
        self.right_motor.off()

if __name__ == "__main__":
    controller = OrientationController()
    controller.run()
