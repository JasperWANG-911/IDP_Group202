'''This file is a PID implementation, assuming trackers are arranged in front-back-left-right config.

Let position be 0 along the centre-line of the robot.
'''

import sensor_for_main as sfm

# Positions of tracker sensors (mm)
posf = (0, 10)
posb = (0, -10)
posl = (-10, 0)
posr = (10, 0)

# Get an initial pattern (for testing, not used further)
pattern = sfm.get_track_sensor_pattern()

class Control:
    def __init__(self, sensors):
        '''
        Initialise object with associated sensors,
        kp, kd and ki control constants,
        accumulated integral error
        and previous absolute error (for derivative control)
        '''
        self.sensors = sensors
        self.k_p = 5
        self.k_i = 5
        self.k_d = 5
        self.errI = 0
        self.prevPos = 0

    def get_pos(self) -> float | None:
        # Get state of sensors and calculate "COM" of readings.
        state = self.sensors.get_track_sensor_pattern()
        if sum(state.values()) == 0:
            return None
        # Calculate x position of COM (only x component is used for lateral correction)
        posG_x = (state['front'] * posf[0] +
                  state['rear'] * posb[0] +
                  state['left'] * posl[0] +
                  state['right'] * posr[0])
        return posG_x
        
    def get_error(self, x: float):
        '''
        Calculates the error based on PID control.
        Arg:
            x : x position of 'COM' of error
        Returns:
            error : calibration to add to one motor and subtract from the other.
        '''
        errP = self.k_p * x
        errD = self.k_d * (x - self.prevPos)
        self.errI += self.k_i * x
        self.prevPos = x
        return errP + errD + self.errI
    
    def at_junction(self) -> bool:
        '''
        Detects if the robot is at a junction by checking if three or more sensors are active.
        Resets the integral term when a junction is detected.
        Returns:
            bool : True if at junction, False otherwise.
        '''
        state = self.sensors.get_track_sensor_pattern()  # fixed function call name
        self.reset()
        return (sum(state.values()) >= 3)

    def reset(self):
        self.errI = 0
