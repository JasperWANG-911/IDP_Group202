import TOF_sensor
from tcs34725 import html_rgb
from orientation_control import OrientationController

def collection(motor_pair, actuator, TOF_sensor, colour_sensor) -> str:
    # orientation control

    while TOF_sensor.ping() > 150:
        print(TOF_sensor.ping(), 'safe')

    else:
        # Grab the box when it is detected by ToF sensor
        print('box detected')
        actuator.grab_the_box(motor_pair)
        colour_sensor.gain(60)
        data = colour_sensor.read(True)
        
        # read the color by R_value
        R_value = html_rgb(data)[0]
        if R_value < 4: 
            colour = 'BG' # box A/B
        else:
            colour = 'RY' # box C/D
    return colour

# To drop box, call actuator.drop_the_box

def drop_off(motor_pair, actuator, TOF_sensor):

    actuator.drop_the_box(motor_pair)
    
    return True

