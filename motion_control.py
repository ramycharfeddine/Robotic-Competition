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
        assert speed > 0 and speed < 0.765
        return 0.1+self.dist/speed

    def timeout4distance(self):
        return 0.1+self.dist/MotionControl.DISTANCE_MODE_SPEED

class MotionControl:
    INITIAL_PATH = np.array([[1,1], [7,1],[5,5], [0,5], [0.5,0.5]])
    WHEEL_SPACING_FACTOR = 26.5 # distance btw wheels (53cm)/2
    EPS_ANGLE = math.radians(10) # ~10 degree
    ERROR_POS_X = 20 # cm
    ERROR_POS_Y = 20 # cm
    EPS_DISTANCE = np.sqrt(ERROR_POS_X**2 + ERROR_POS_Y**2)
    DISTANCE_MODE_SPEED = 15 # 50*0.3cm/s
    NOMINAL_SPEED = 26.6 # nominal speed of the motor at 11.1V

    def __init__(self) -> None:
        self.waypoint_list = self.INITIAL_PATH
        self.BOTTLE_TOO_CLOSE = False
        self.GOING_BACK = False

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

    def _go_to_waypoint(self, state, waypoint):
        """generate command to next waypoint
        """
        cmd = []

        dist_target, angle_target = self._cart2polar(waypoint - state[0])
        #angle_rotation = self._get_angle_to_turn(angle_target, state[1])
        angle_rotation = angle_target - state[1]
        print(angle_rotation)

        if dist_target > self.EPS_DISTANCE:
            # we don't need to turn if we have reached the waypoint
            if not self.BOTTLE_TOO_CLOSE: 
                cmd.append(MotionCmd(3, dist_target))
            else:
                cmd.append(MotionCmd(0, dist_target))
            if abs(angle_rotation) > self.EPS_ANGLE:
                cmd.insert(0, MotionCmd(1+1*(angle_rotation>0), 
                                        self._angle2dist(angle_rotation)))

        return cmd

    def go_to_next_waypoint(self, state) -> list: 
        wp = self.waypoint_list[0] #self.next_waypoint(state)
        cmd = self._go_to_waypoint(self, state, wp)
        if len(cmd) == 0: # reached
            self.waypoint_list = self.waypoint_list[1:]
            # return an empty cmd list

        return cmd

    def insert_waypoint(self, wp):
        self.waypoint_list = np.insert(self.waypoint_list,0, wp, axis=0)
