# main.py
import machine
import time
from motor import Motor1, Motor2, MotorPair
from navigation import Navigation

def main():
    # Tunable parameters:
    target_route = ['X1', 'X2', 'X3', 'X4', 'RY', 'BG']  # Change target route if needed.
    base_speed = 75                                       # Default speed.
    pid_params = (20, 0.5, 10)                            # PID parameters: (k_p, k_i, k_d).
    deriv_window = 10                                    # Size of the derivative moving average window.
    integral_window = 10                                # Size of the integral accumulation window.
    
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
        deriv_window=deriv_window,
        integral_window=integral_window
    )
    
    print("Starting main navigation sequence...")
    visited_nodes = nav.run()
    print("Navigation sequence completed. Visited nodes:", visited_nodes)
    
    # Set up an LED on Pin 25 for visual indication.
    led = machine.Pin(25, machine.Pin.OUT)
    for i in range(3):
        led.value(1)
        time.sleep(0.2)
        led.value(0)
        time.sleep(0.2)

if __name__ == "__main__":
    main()
