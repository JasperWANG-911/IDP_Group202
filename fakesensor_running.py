#!/usr/bin/env python3
"""
Standalone test script to simulate vehicle navigation using a randomized fake sensor.

This script mimics the behavior of your main.py but overrides sensor input with a fake
sensor implementation. The fake sensor works in two phases:
  1. Startup Phase: For the first few readings, it returns no sensor activity,
     then returns a start condition (both front and rear active) to trigger vehicle start.
  2. Normal Operation: It normally returns a forward pattern (only front active).
     However, every ~15 seconds it randomly returns a command pattern for either a left turn,
     right turn, or move back for 2 seconds, then reverts to forward.
  
The rest of the vehicle logic (navigation, odometry, motor control) is exercised using these simulated inputs.
"""

import time
import random
import navigation
from motor import Motor1, Motor2
from motor_pair import MotorPair
from odometry import Odometry
import sensor_for_main

# ---------------------------------------------------------------------------
# Fake Sensors Implementation with Random Commands
# ---------------------------------------------------------------------------
class FakeSensors:
    """
    Fake sensor class that simulates track sensor outputs.
    
    The sensor works in two phases:
      - Startup Phase: For the first few calls, returns no sensor activity.
        Then returns a start condition (both front and rear active) once to trigger startup.
      - Normal Operation: Returns a default forward pattern (front active only)
        most of the time. However, every ~15 seconds it randomly switches to one of:
            - left_turn (simulate left turn: left sensor active, front inactive),
            - right_turn (simulate right turn: right sensor active, front inactive),
            - rear (simulate move back: rear sensor active, front inactive)
        for about 2 seconds before reverting to default.
    """
    def __init__(self):
        # Startup phase counters
        self.startup_count = 0
        self.startup_complete = False
        
        # For random command generation
        self.last_command_time = time.time()
        self.command_mode = None  # None means default (forward) pattern
        self.command_start_time = None

    def read_track_sensor(self):
        """
        Simulate reading the track sensor.
        
        Returns:
          A dictionary with keys: 'front', 'right', 'rear', and 'left' (0 = inactive, 1 = active).
        """
        now = time.time()
        # ----- Startup Phase -----
        if not self.startup_complete:
            if self.startup_count < 5:
                self.startup_count += 1
                return {'front': 0, 'right': 0, 'rear': 0, 'left': 0}
            else:
                # Once startup counter is done, simulate a start condition (both front and rear active)
                self.startup_complete = True
                # Also reset command timing after startup
                self.last_command_time = now
                return {'front': 1, 'right': 0, 'rear': 1, 'left': 0}
        
        # ----- Normal Operation Phase -----
        # If currently in a command mode, check if the 2-second duration has elapsed.
        if self.command_mode is not None:
            if now - self.command_start_time >= 2:
                # End command mode, revert to default.
                self.command_mode = None
        
        # If not in command mode and it's been 15 seconds since the last command, generate a new command.
        if self.command_mode is None and now - self.last_command_time >= 15:
            self.command_mode = random.choice(['left_turn', 'right_turn', 'rear'])
            self.command_start_time = now
            self.last_command_time = now  # Reset last command time
            print(f"FakeSensors: New command mode activated: {self.command_mode}")

        # Return sensor pattern based on current command mode.
        if self.command_mode == 'left_turn':
            # Simulate a left turn: left sensor active, front inactive.
            return {'front': 0, 'right': 0, 'rear': 0, 'left': 1}
        elif self.command_mode == 'right_turn':
            # Simulate a right turn: right sensor active, front inactive.
            return {'front': 0, 'right': 1, 'rear': 0, 'left': 0}
        elif self.command_mode == 'rear':
            # Simulate a move back: rear sensor active, front inactive.
            return {'front': 0, 'right': 0, 'rear': 1, 'left': 0}
        else:
            # Default forward pattern: only front sensor active.
            return {'front': 1, 'right': 0, 'rear': 0, 'left': 0}

# Instantiate the fake sensor object.
fake_sensors = FakeSensors()

# Override the sensor_for_main module's get_track_sensor_pattern to use our fake sensor.
sensor_for_main.get_track_sensor_pattern = fake_sensors.read_track_sensor

# Also update any fake sensor instance in navigation if present.
if hasattr(navigation, 'fake_sensor_instance'):
    navigation.fake_sensor_instance = fake_sensors

# ---------------------------------------------------------------------------
# Main Simulation Script
# ---------------------------------------------------------------------------
def main():
    """
    Main function that initializes the hardware abstractions, waits for the start condition,
    and then executes the navigation routine.
    """
    print("Starting simulation with randomized fake sensor input...")

    # Initialize motor pair and odometry state.
    left_motor = Motor1()
    right_motor = Motor2()
    motors = MotorPair(left_motor, right_motor)
    odom = Odometry()

    # Simulate LED indication (via console output) as in the original main.py.
    print("Waiting to set off from starting node 1...")
    while True:
        sp = sensor_for_main.get_track_sensor_pattern()
        print("Sensor pattern:", sp)
        # Check start condition: both front and rear sensors active.
        if sp['front'] == 1 and sp['rear'] == 1:
            print("Vehicle has set off from starting point.")
            for i in range(3):
                print("LED ON")
                time.sleep(0.2)
                print("LED OFF")
                time.sleep(0.2)
            break
        time.sleep(0.1)

    # Run the navigation routine with the predetermined route.
    navigation.run_navigation(motors, odom)

if __name__ == "__main__":
    main()
