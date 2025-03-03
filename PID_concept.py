'''This file is a PID implementation, assuming trackers are arranged in front-back-left-right config.

Let position be 0 along the centre-line of the robot.'''

import sensor_for_main as sfm
import motor_pair

# Positions of tracker sensors (mm)

posf = (0, 10)
posb = (0, -10)
posl = (-10, 0)
posr = (10, 0)

pattern = sfm.get_track_sensor_pattern()

class Control:
    def __init__(self, sensors):
        '''
        Initialises object with associated sensors,
        kp, kd and ki control constants,
        accumulated integral error
        and previous absolute error (derivative control)
        '''
        self.sensors = sensors
        self.k_p = 5
        self.k_i = 5
        self.k_d = 5
        self.errI = 0
        self.prevPos = 0

    def get_pos(self) -> float | None:
        # Get state of sensors and calculate "COM" of readings.
        '''
        Calculates the x position of 'COM' of error.
        Returns:
            x : x position of 'COM' of error
        '''
        state = self.sensors.get_track_sensor_pattern()
        if list(state.values()).sum() == 0:
            return None
        posG = state['front'] * posf + state['back'] * posb + state['left'] * posl + state['right'] * posr
        return posG[0]
        
    def get_error(self, x : float):
        '''
        Calculates the error, based on PID control.
        Arg:
            x : x position of 'COM' of error
        Returns:
            error : calibrated error to add to one motor and subtract from the other
        '''
        errP = self.k_p * x
        errD = self.k_d * (x - self.prevPos)
        self.errI += self.k_i * x
        self.prevPos = x
        return errP + errD + self.errI
    
    def at_junction(self) -> bool:
        '''
        Detects if it is at a junction. This operates by detecting if there are more than 2 sensors on at any time.
        Error is also reset.
        Arg:
            sensors : Sensors object
        Returns:
            bool : at junction or not
        '''
        state = self.sensors.get_track_sensor_patter()
        self.reset()
        return (list(state.values()).sum() >= 3)

    def reset(self):
        self.errI = 0
    




