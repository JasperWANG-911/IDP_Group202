#!/usr/bin/env python3
import time
from motor import MotorPair, Motor1, Motor2  # Motor2 is left, Motor1 is right
from line_sensor import LineSensors

class PIDController:
    def __init__(self, k_p, k_i, k_d, deriv_window=10, integral_window=10):
        # Initialize PID constants and internal state variables.
        self.k_p = k_p
        self.k_i = k_i
        self.k_d = k_d
        # Lists for storing recent error values for smoothing derivative and integral
        self.error_window = []         # For derivative smoothing
        self.integral_window = []      # For a moving integral sum
        self.deriv_window_size = deriv_window
        self.integral_window_size = integral_window

    def compute(self, error):
        """
        Compute PID correction based on the current error using moving average smoothing for the derivative 
        and a moving window for the integral term.
        
        Args:
            error: The current error value.
        Returns:
            The PID correction value.
        """
        # --- Update derivative window ---
        self.error_window.append(error)
        if len(self.error_window) > self.deriv_window_size:
            self.error_window.pop(0)
        
        if len(self.error_window) > 1:
            # Use the average of all previous errors in the window for smoothing.
            avg_prev = sum(self.error_window[:-1]) / (len(self.error_window) - 1)
            derivative = error - avg_prev
        else:
            derivative = 0.0

        # --- Update integral window ---
        self.integral_window.append(error)
        if len(self.integral_window) > self.integral_window_size:
            self.integral_window.pop(0)
        integral = sum(self.integral_window)

        # Compute PID output.
        output = self.k_p * error + self.k_i * integral + self.k_d * derivative
        return output

def clamp_speed(speed):
    """
    Clamp the motor speed within the range [-100, 100].
    
    Args:
        speed: Proposed motor speed.
    Returns:
        Clamped motor speed.
    """
    return max(-100, min(100, speed))

def set_motor_speed(motor, speed):
    """
    Set motor speed. If speed is positive, use forward; if negative, use reverse (with absolute value); if zero, stop.
    
    Args:
        motor: The motor instance.
        speed: Desired speed (as a percentage).
    """
    if speed > 0:
        motor.forward(speed)
    elif speed < 0:
        motor.reverse(abs(speed))
    else:
        motor.off()

class OrientationController:
    """
    OrientationController integrates sensor readings, a PID controller, and motor commands
    to adjust the robot's heading along a line. It uses the two center sensors for error calculation.
    This implementation uses moving windows for both derivative and integral terms to smooth the PID response.
    
    New parameters:
      - sensitivity: scales the PID output before it is added to the base speed (default: 1).
      - action_type: determines the motor command pattern.
          "straight" (default): left_speed = base_speed + sensitivity*PID,
                                right_speed = base_speed - sensitivity*PID.
          "left": left_speed = -base_speed + sensitivity*PID,
                  right_speed = base_speed - sensitivity*PID.
          "right": left_speed = base_speed + sensitivity*PID,
                   right_speed = -base_speed - sensitivity*PID.
    """
    def __init__(self, base_speed=75, k_p=20, k_i=0.5, k_d=10, deriv_window=10, integral_window=10, sensitivity=1, action_type="straight"):
        self.sensors = LineSensors()
        self.left_motor = Motor2()   # Left motor (Motor2)
        self.right_motor = Motor1()  # Right motor (Motor1)
        self.car = MotorPair(self.left_motor, self.right_motor)
        self.pid = PIDController(k_p, k_i, k_d, deriv_window, integral_window)
        self.base_speed = base_speed
        self.sensitivity = sensitivity
        self.action_type = action_type

    def get_line_error(self):
        """
        Compute the line error using only the two center sensors.
        Error rules:
          - Both active: 0.0 (aligned)
          - Only center_left active: -1.0 (deviated left)
          - Only center_right active: 1.0 (deviated right)
          - Both inactive: use last computed error (implicitly via the PID windows)
        Returns:
            The error value.
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
            error = self.pid.error_window[-1] if self.pid.error_window else 0.0

        return error

    def update(self):
        """
        Perform one forward control update cycle:
          - Read sensor data.
          - Compute PID correction.
          - Adjust motor speeds based on the current action type.
        """
        error = self.get_line_error()
        correction = self.pid.compute(error)
        # Apply sensitivity factor.
        scaled_correction = self.sensitivity * correction
        
        if self.action_type == "straight":
            left_speed = clamp_speed(self.base_speed + scaled_correction)
            right_speed = clamp_speed(self.base_speed - scaled_correction)
        elif self.action_type == "left":
            # When turning left: left motor runs at negative base speed plus correction, right runs at positive base speed minus correction.
            left_speed = clamp_speed(-self.base_speed + scaled_correction)
            right_speed = clamp_speed(self.base_speed - scaled_correction)
        elif self.action_type == "right":
            # When turning right: left motor runs at positive base speed plus correction, right runs at negative base speed minus correction.
            left_speed = clamp_speed(self.base_speed + scaled_correction)
            right_speed = clamp_speed(-self.base_speed - scaled_correction)
        else:
            # Default to straight if unknown action type.
            left_speed = clamp_speed(self.base_speed + scaled_correction)
            right_speed = clamp_speed(self.base_speed - scaled_correction)
            
        #print("Forward Update -> Error: {:.2f}, Correction: {:.2f}, Left Speed: {:.2f}, Right Speed: {:.2f}".format(
            #error, scaled_correction, left_speed, right_speed))
        set_motor_speed(self.left_motor, left_speed)
        set_motor_speed(self.right_motor, right_speed)

    def update_reverse(self):
        """
        Perform one reverse control update cycle:
          - Read sensor data.
          - Compute PID correction.
          - Adjust motor speeds for reverse movement (base_speed is considered negative) based on the action type.
        """
        error = self.get_line_error()
        correction = self.pid.compute(error)
        scaled_correction = self.sensitivity * correction
        
        if self.action_type == "straight":
            left_speed = -clamp_speed(self.base_speed + scaled_correction)
            right_speed = -clamp_speed(self.base_speed - scaled_correction)
        elif self.action_type == "left":
            left_speed = clamp_speed(-self.base_speed - scaled_correction)
            right_speed = clamp_speed(self.base_speed + scaled_correction)
        elif self.action_type == "right":
            left_speed = clamp_speed(self.base_speed - scaled_correction)
            right_speed = clamp_speed(-self.base_speed + scaled_correction)
        else:
            left_speed = clamp_speed(self.base_speed - scaled_correction)
            right_speed = clamp_speed(self.base_speed + scaled_correction)
        
        #print("Reverse Update -> Error: {:.2f}, Correction: {:.2f}, Left Speed: {:.2f}, Right Speed: {:.2f}".format(
            #error, scaled_correction, left_speed, right_speed))
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
