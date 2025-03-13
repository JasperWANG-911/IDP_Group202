import time

def wait_for_button_press(button):
    # Wait for button press (active low)
    while button.value() == 1:
        time.sleep(0.05)
    time.sleep(0.2)  # debounce delay

    # Wait for button release
    while button.value() == 0:
        time.sleep(0.05)
    time.sleep(0.2)  # debounce delay