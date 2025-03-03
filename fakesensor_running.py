#!/usr/bin/env python3
"""
Standalone simulation script to test navigation with fake sensor inputs,
updated odometry parameters, and modified backward speed.

Updates:
  - Odometry uses a wheel diameter of 60 mm (≈ 0.1885 m per move).
  - A 90° turn now uses a tuning period of 2.5 seconds.
  - The motor reverse functions are updated to run at the same speed as forward.
  - The fake sensor randomly issues turning or move-back commands every ~15 s.
"""

import time
import random
import math
import navigation
from motor import Motor1, Motor2
from motor_pair import MotorPair
from odometry import Odometry
import sensor_for_main
import odometry

# ---------------------------
# Update Motor Reverse Methods to Match Forward Speed
# ---------------------------
# For Motor1:
original_Motor1_Reverse = Motor1.Reverse
def new_Motor1_Reverse(self):
    self.m1Dir.value(1)
    # Updated reverse: use 100% speed instead of 30%
    self.pwm1.duty_u16(int(65535*100/100))
Motor1.Reverse = new_Motor1_Reverse

# For Motor2:
original_Motor2_Reverse = Motor2.Reverse
def new_Motor2_Reverse(self):
    self.m1Dir.value(1)
    # Updated reverse: use 100% speed instead of 30%
    self.pwm1.duty_u16(int(65535*100/100))
Motor2.Reverse = new_Motor2_Reverse

# ---------------------------
# Update Odometry based on 60 mm Wheel Diameter
# ---------------------------
wheel_diam = 0.06  # in meters (60 mm)
odometry.DISTANCE_PER_MOVE = math.pi * wheel_diam  # ≈ 0.1885 m per move
print(f"Updated distance per move: {odometry.DISTANCE_PER_MOVE:.4f} m (using wheel diameter of {wheel_diam*1000:.0f} mm)")

# ---------------------------
# Fake Sensors Implementation with Random Commands
# ---------------------------
class FakeSensors:
    """
    Fake sensor class that simulates track sensor outputs.
    
    - Startup Phase: For the first few calls, returns no sensor activity,
      then returns a start condition (both front and rear active).
    - Normal Operation: Returns a default forward pattern (front active).
      Every ~15 seconds, it randomly switches to one of:
          left_turn (left sensor active),
          right_turn (right sensor active),
          or rear (rear sensor active)
      for about 2 seconds, then reverts to the forward pattern.
    """
    def __init__(self):
        self.startup_count = 0
        self.startup_complete = False
        
        self.last_command_time = time.time()
        self.command_mode = None  # None means default forward pattern
        self.command_start_time = None

    def read_track_sensor(self):
        now = time.time()
        # ----- Startup Phase -----
        if not self.startup_complete:
            if self.startup_count < 5:
                self.startup_count += 1
                return {'front': 0, 'right': 0, 'rear': 0, 'left': 0}
            else:
                self.startup_complete = True
                self.last_command_time = now
                return {'front': 1, 'right': 0, 'rear': 1, 'left': 0}
        
        # ----- Normal Operation -----
        if self.command_mode is not None:
            if now - self.command_start_time >= 2:
                self.command_mode = None
        
        if self.command_mode is None and now - self.last_command_time >= 15:
            self.command_mode = random.choice(['left_turn', 'right_turn', 'rear'])
            self.command_start_time = now
            self.last_command_time = now
            print(f"FakeSensors: New command mode activated: {self.command_mode}")

        if self.command_mode == 'left_turn':
            return {'front': 0, 'right': 0, 'rear': 0, 'left': 1}
        elif self.command_mode == 'right_turn':
            return {'front': 0, 'right': 1, 'rear': 0, 'left': 0}
        elif self.command_mode == 'rear':
            return {'front': 0, 'right': 0, 'rear': 1, 'left': 0}
        else:
            return {'front': 1, 'right': 0, 'rear': 0, 'left': 0}

# Instantiate the fake sensor.
fake_sensors = FakeSensors()
sensor_for_main.get_track_sensor_pattern = fake_sensors.read_track_sensor

# If navigation has a fake sensor instance, update it.
if hasattr(navigation, 'fake_sensor_instance'):
    navigation.fake_sensor_instance = fake_sensors

# ---------------------------
# Override Turning Routine for 90° Turns with 2.5 s Timeout
# ---------------------------
original_turn_until_shift = navigation.turn_until_shift
def turn_until_shift_fixed(motors, turn_type, increment=0.1, timeout=3):
    # Force a timeout of 2.5 seconds.
    return original_turn_until_shift(motors, turn_type, increment, timeout=2.5)
navigation.turn_until_shift = turn_until_shift_fixed

# ---------------------------
# Main Simulation Script
# ---------------------------
def main():
    print("Starting simulation with updated backward speed, odometry, and turning parameters...")

    left_motor = Motor1()
    right_motor = Motor2()
    motors = MotorPair(left_motor, right_motor)
    odom_obj = Odometry()

    print("Waiting to set off from starting node 1...")
    while True:
        sp = sensor_for_main.get_track_sensor_pattern()
        print("Sensor pattern:", sp)
        if sp['front'] == 1 and sp['rear'] == 1:
            print("Vehicle has set off from starting point.")
            for _ in range(3):
                print("LED ON")
                time.sleep(0.2)
                print("LED OFF")
                time.sleep(0.2)
            break
        time.sleep(0.1)

    # Run the navigation routine (using the predetermined route from the graph)
    navigation.run_navigation(motors, odom_obj)

if __name__ == "__main__":
    main()
