from machine import Pin
import time

# Define the control pins for extending and retracting
extend_pin = Pin(15, Pin.OUT)   # Change 15 to your extend control pin number
retract_pin = Pin(14, Pin.OUT)  # Change 14 to your retract control pin number

def extend_actuator(duration):
    print("Extending actuator for", duration, "seconds.")
    extend_pin.value(1)    # Activate extension
    retract_pin.value(0)   # Ensure retraction is off
    time.sleep(duration)
    stop_actuator()

def retract_actuator(duration):
    print("Retracting actuator for", duration, "seconds.")
    extend_pin.value(0)    # Ensure extension is off
    retract_pin.value(1)   # Activate retraction
    time.sleep(duration)
    stop_actuator()

def stop_actuator():
    print("Stopping actuator.")
    extend_pin.value(0)
    retract_pin.value(0)
    time.sleep(1)  # Pause briefly before the next action

# Main loop to continuously extend and retract the actuator
while True:
    extend_actuator(3)   # Extend for 3 seconds
    retract_actuator(3)  # Retract for 3 seconds
