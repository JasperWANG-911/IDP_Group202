#!/usr/bin/env python3
import machine
from machine import Pin, I2C
import time
from motor import Motor1, Motor2, MotorPair
from navigation import Navigation
from collection_dropoff import collection, drop_off
from TOF_sensor import VL53L0X
import tcs34725 as tc
from actuator import Actuator
from vl53l0x import VL53L0X



def wait_for_button_press(button):
    """
    Wait until the button is pressed and then released.
    Returns after a complete press-release cycle is detected.
    """
    # Wait for button press (active low)
    while button.value() == 1:
        time.sleep(0.05)
    time.sleep(0.2)  # debounce delay
    # Wait for button release
    while button.value() == 0:
        time.sleep(0.05)
    time.sleep(0.2)  # debounce delay


def main():
    # Tunable parameters:
    target_route = ['X1', 'X2', 'X3', 'X4', 1]  # Change target route if needed.
    base_speed = 75  # Default speed.
    pid_params = (10, 0.2, 18)  # PID parameters: (k_p, k_i, k_d).
    deriv_window = 10  # Size of the derivative moving average window.
    integral_window = 10  # Size of the integral accumulation window.

    # Initialize motors (Motor2 is left, Motor1 is right).
    left_motor = Motor2()
    right_motor = Motor1()
    motors = MotorPair(left_motor, right_motor)

    # Initialze actuator
    actuator = Actuator()
    actuator.Reverse()
    time.sleep(5)
    actuator.off()
    
    actuator.Forward()
    time.sleep(5)
    actuator.off()

    # Initialze ToF
    i2c = I2C(id=1, sda=Pin(18), scl=Pin(19))
    tof = VL53L0X(i2c)
    budget = tof.measurement_timing_budget_us
    tof.set_measurement_timing_budget(4000000000)
    tof.set_Vcsel_pulse_period(tof.vcsel_period_type[0], 12)
    tof.set_Vcsel_pulse_period(tof.vcsel_period_type[1], 8)

    # Initialze color sensor
    i2c_bus = I2C(0, sda=Pin(16), scl=Pin(17), freq=400000)
    tcs = tc.TCS34725(i2c_bus)

    # Initialize button
    button = machine.Pin(20, machine.Pin.IN, machine.Pin.PULL_UP)

    # Create a Navigation instance with all tunable parameters.
    nav = Navigation(
        motors,
        target_route=target_route,
        base_speed=base_speed,
        pid_params=pid_params,
        # deriv_window=deriv_window,
        # integral_window=integral_window
    )

    print("Press and release the button on pin 20 to start the navigation sequence.")
    wait_for_button_press(button)

    print("Starting main navigation sequence...")
    visited_nodes = nav.run(actuator, tof, tcs)
    print("Navigation sequence completed. Visited nodes:", visited_nodes)


    # Wait for a final button press to end the program (optional).
    print("Press and release the button on pin 20 to exit.")
    wait_for_button_press(button)
    print("Exiting program.")


if __name__ == "__main__":
    main()
