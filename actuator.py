import RPi.GPIO as GPIO
import time

# Define the GPIO pins (using BCM numbering)
EXTEND_PIN = 18   # GPIO pin to extend the actuator
RETRACT_PIN = 23  # GPIO pin to retract the actuator

# Set up the GPIO mode and pins
GPIO.setmode(GPIO.BCM)
GPIO.setup(EXTEND_PIN, GPIO.OUT)
GPIO.setup(RETRACT_PIN, GPIO.OUT)

def extend_actuator(duration):
    print("Extending actuator")
    GPIO.output(EXTEND_PIN, GPIO.HIGH)
    GPIO.output(RETRACT_PIN, GPIO.LOW)
    time.sleep(duration)
    stop_actuator()

def retract_actuator(duration):
    print("Retracting actuator")
    GPIO.output(EXTEND_PIN, GPIO.LOW)
    GPIO.output(RETRACT_PIN, GPIO.HIGH)
    time.sleep(duration)
    stop_actuator()

def stop_actuator():
    # Set both pins LOW to stop any motion
    GPIO.output(EXTEND_PIN, GPIO.LOW)
    GPIO.output(RETRACT_PIN, GPIO.LOW)
    print("Actuator stopped")
    time.sleep(1)  # Pause for a moment before next action

try:
    while True:
        extend_actuator(3)  # Extend for 3 seconds
        retract_actuator(3) # Retract for 3 seconds

except KeyboardInterrupt:
    print("Program interrupted by user")

finally:
    GPIO.cleanup()  # Reset the GPIO settings
