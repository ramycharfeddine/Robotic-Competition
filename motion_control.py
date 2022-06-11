import numpy as np
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
    WHEEL_SPACING_FACTOR = 53
    EPS_ANGLE = 0.017 # ~1 degree
    EPS_DISTANCE = 5
    DISTANCE_MODE_SPEED = 15 # 50*0.3cm/s
    NOMINAL_SPEED = 26.6 # nominal speed of the motor at 11.1V

    def __init__(self) -> None:
        self.waypoint_list = self.INITIAL_PATH

    @staticmethod
    def _angle2dist(angle):
        return angle * MotionControl.WHEEL_SPACING_FACTOR

    @staticmethod
    def _cart2polar(vector):
        dist = np.linalg.norm(vector)
        angle = np.arctan2(vector[1], vector[0])
        return dist, angle

    def _go_to_waypoint(self, state, waypoint):
        """generate command to next waypoint
        """
        cmd = []

        dist_target, angle_target = self._cart2polar(waypoint - state[0])
        angle_rotation = angle_target - state[1]

        if dist_target > self.EPS_DISTANCET:
            cmd.append(MotionCmd(3, dist_target))
            # we don't need to turn if we have reached the waypoint
            if abs(angle_rotation) > self.EPS_ANGLE:
                cmd.insert(0, MotionCmd(1+1*(angle_rotation>0), 
                                        self._angle2dist(angle_rotation)))
        
        return cmd

    def go_to_next_waypoint(self, state) -> list: 
        wp = self.next_waypoint(state)
        cmd = self._go_to_waypoint(self, state, wp)
        if len(cmd) == 0: # reached
            self.waypoint_list = self.waypoint_list[:-1]
            # return an empty cmd list
            
        return cmd

    def next_waypoint(self, state): # maybe more info
        """decide the next waypoint"""
        # TODO
        return self.waypoint_list[-1]
