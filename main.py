#!/usr/bin/env python3
import machine
import time
from motor import Motor1, Motor2, MotorPair
from navigation import Navigation

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
    target_route = ['X1', 'RY',1]  # Change target route if needed.
    base_speed = 75                                       # Default speed.
    pid_params = (10, 0.2, 8)                           # PID parameters: (k_p, k_i, k_d).
    deriv_window = 10                                     # Size of the derivative moving average window.
    integral_window = 10                                  # Size of the integral accumulation window.
    
    # Initialize motors (Motor2 is left, Motor1 is right).
    left_motor = Motor2()
    right_motor = Motor1()
    motors = MotorPair(left_motor, right_motor)
    
    # Create a Navigation instance with all tunable parameters.
    nav = Navigation(
        motors,
        target_route=target_route,
        base_speed=base_speed,
        pid_params=pid_params,
        #deriv_window=deriv_window,
        #integral_window=integral_window
    )
    
    # Configure the start/stop button on pin 20 (active low, internal pull-up)
    button = machine.Pin(20, machine.Pin.IN, machine.Pin.PULL_UP)
    
    print("Press and release the button on pin 20 to start the navigation sequence.")
    wait_for_button_press(button)
    
    print("Starting main navigation sequence...")
    visited_nodes = nav.run()
    print("Navigation sequence completed. Visited nodes:", visited_nodes)
    
    # Flash LED on pin 25 to indicate completion.
    led = machine.Pin(25, machine.Pin.OUT)
    for i in range(3):
        led.value(1)
        time.sleep(0.2)
        led.value(0)
        time.sleep(0.2)
    
    # Wait for a final button press to end the program (optional).
    print("Press and release the button on pin 20 to exit.")
    wait_for_button_press(button)
    print("Exiting program.")

if __name__ == "__main__":
    main()
    