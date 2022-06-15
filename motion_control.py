import numpy as np
import math
from protocol import Protocol, Arduino

# unit: second, centimeter
# positions in row vector convention
# [x, y]
# state = [position, orientation]

"""
Motion Control will generate motion commands,
and the main program will execute it.

Motion Control module 'should'(I think) be separated from 
the actual Arduino controling interface
"""

class MotionCmd:
    def __init__(self, dir, dist) -> None:
        self.dir = dir
        self.dist = dist
    
    def timeout4speed(self, speed):
        assert speed > 0 and speed < 76.5
        return 0.1+self.dist/speed

    def timeout4distance(self):
        return 0.1+self.dist/MotionControl.DISTANCE_MODE_SPEED

class MotionControl:

    WHEEL_SPACING_FACTOR = 28 # distance btw wheels (53cm)/2
    DISTANCE_MODE_SPEED = 15 # 50*0.3cm/s
    NOMINAL_SPEED = 35 #26.6 # nominal speed of the motor at 11.1V

    def __init__(self) -> None:
        pass

    @staticmethod
    def _angle2dist(angle):
        return angle * MotionControl.WHEEL_SPACING_FACTOR

    @staticmethod
    def _cart2polar(vector):
        dist = np.linalg.norm(vector)
        angle = np.arctan2(vector[1], vector[0])
        return dist, angle

    @staticmethod
    def _get_angle_to_turn(angle_target, state_angle): # target_angle, robot_angle in deg
        angle_target = math.degrees(angle_target)
        state_angle  = math.degrees(state_angle)
        phi = (state_angle-angle_target) % 360
        sign = -1
        # used to calculate sign
        if not ((phi >= 0 and phi <= 180) or (phi <= -180 and phi >= -360)):
            sign = 1
        if phi > 180:
            result = 360-phi
        else:
            result = phi
        return math.radians(result*sign)
