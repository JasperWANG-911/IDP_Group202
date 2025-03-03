#!/usr/bin/env python3
"""
Standalone simulation script to test navigation with fake sensor inputs.

Updates from the previous version:
  - Odometry is updated to use a wheel diameter of 60 mm (circumference ≈ 0.1885 m)
    so that a move command advances the robot by ~0.1885 m.
  - The turning routine is modified so that a 90° turn (tuning) now uses a 2.5s timeout.
  - The navigation task is executed over the predetermined route using graph coordinates.
"""

import time
import random
import navigation
from motor import Motor1, Motor2
from motor_pair import MotorPair
from odometry import Odometry
import sensor_for_main
import odometry

# ---------------------------
# Update Odometry based on wheel diameter
# ---------------------------
# For a wheel diameter of 60mm, the circumference is:
import math
wheel_diam = 0.06  # in meters (60mm)
odometry.DISTANCE_PER_MOVE = math.pi * wheel_diam  # ~0.1885 m
print(f"Updated distance per move: {odometry.DISTANCE_PER_MOVE:.4f} m (using wheel diameter of {wheel_diam*1000:.0f} mm)")

# ---------------------------
# Fake Sensors Implementation with Random Commands
# ---------------------------
class FakeSensors:
    """
    Fake sensor class that simulates track sensor outputs.
    
    The sensor operates in two phases:
      - Startup Phase: For the first few calls, returns no sensor activity.
        Then returns a start condition (both front and rear active) to trigger startup.
      - Normal Operation: Normally returns the default forward pattern (only front sensor active).
        However, every ~15 seconds it randomly switches to one of the following command patterns:
            * left_turn (simulate left turn: left sensor active, front inactive)
            * right_turn (simulate right turn: right sensor active, front inactive)
            * rear (simulate move backward: rear sensor active, front inactive)
        These command patterns persist for about 2 seconds before reverting to default.
    """
    def __init__(self):
        # Startup phase
        self.startup_count = 0
        self.startup_complete = False
        
        # Command generation
        self.last_command_time = time.time()
        self.command_mode = None  # None means default forward pattern
        self.command_start_time = None

    def read_track_sensor(self):
        """
        Simulate reading the track sensor.
        Returns a dictionary with binary outputs for keys: 'front', 'right', 'rear', 'left'.
        """
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
        
        # ----- Normal Operation Phase -----
        # If currently in a command mode, check if 2 seconds have elapsed.
        if self.command_mode is not None:
            if now - self.command_start_time >= 2:
                self.command_mode = None  # End command mode
        
        # If not in command mode and 15 seconds have passed, choose a new random command.
        if self.command_mode is None and now - self.last_command_time >= 15:
            self.command_mode = random.choice(['left_turn', 'right_turn', 'rear'])
            self.command_start_time = now
            self.last_command_time = now
            print(f"FakeSensors: New command mode activated: {self.command_mode}")

        # Return sensor pattern based on current command mode.
        if self.command_mode == 'left_turn':
            return {'front': 0, 'right': 0, 'rear': 0, 'left': 1}
        elif self.command_mode == 'right_turn':
            return {'front': 0, 'right': 1, 'rear': 0, 'left': 0}
        elif self.command_mode == 'rear':
            return {'front': 0, 'right': 0, 'rear': 1, 'left': 0}
        else:
            # Default forward pattern: only front sensor active.
            return {'front': 1, 'right': 0, 'rear': 0, 'left': 0}

# Instantiate the fake sensor object.
fake_sensors = FakeSensors()

# Override the sensor_for_main module's get_track_sensor_pattern function.
sensor_for_main.get_track_sensor_pattern = fake_sensors.read_track_sensor

# ---------------------------
# Override Turning Routine for 90° Turns
# ---------------------------
# Save original turn_until_shift from navigation.
original_turn_until_shift = navigation.turn_until_shift

def turn_until_shift_fixed(motors, turn_type, increment=0.1, timeout=3):
    """
    Wrapper for turn_until_shift that enforces a timeout of 2.5 seconds
    for a 90-degree turn.
    """
    # Force timeout to 2.5 seconds regardless of caller's parameter.
    return original_turn_until_shift(motors, turn_type, increment, timeout=2.5)

# Override the navigation turn function with our fixed version.
navigation.turn_until_shift = turn_until_shift_fixed

# ---------------------------
# Main Simulation Script
# ---------------------------
def main():
    """
    Main function that initializes motors, odometry, waits for start condition,
    and runs the navigation routine over the predetermined route.
    """
    print("Starting simulation with updated fake sensor input and odometry parameters...")

    # Initialize motor pair and odometry.
    left_motor = Motor1()
    right_motor = Motor2()
    motors = MotorPair(left_motor, right_motor)
    odom_obj = Odometry()

    # Simulated LED indication via console output.
    print("Waiting to set off from starting node 1...")
    while True:
        sp = sensor_for_main.get_track_sensor_pattern()
        print("Sensor pattern:", sp)
        # Check for the start condition: both front and rear active.
        if sp['front'] == 1 and sp['rear'] == 1:
            print("Vehicle has set off from starting point.")
            for _ in range(3):
                print("LED ON")
                time.sleep(0.2)
                print("LED OFF")
                time.sleep(0.2)
            break
        time.sleep(0.1)

    # Run the navigation routine using the predetermined route (from the graph).
    navigation.run_navigation(motors, odom_obj)

if __name__ == "__main__":
    main()
