from turtle import speed
from protocol import Protocol, Arduino
from Localisation import Localization
from detector import Detector
from motion_control import MotionControl, MotionCmd
import numpy as np
import time

class Walle:
    INITIAL_PATH = np.array([[200,200], [100,250], [200,350], [100,450], [75,700], [200,750], [150,600], [350,600],
                             [350,700], [400,750], [450,700], [450,500], [300,450], [400,350], [550,500], [750,400],
                             [750,350], [400,300], [350,250], [750,250], [700,50], [500,125], [500,50], [425,50], 
                             [375,150], [300,50], [300,200], [200,200], [75,75]])
    WAYPOINT_TORLERANCE_DIST = 50 #cm 40width
    WAYPOINT_TORLERANCE_ANGLE = 0.2 # ~11.5 degree

    RECYCLING_TORLERANCE_DIST = 50 #cm 40width
    RECYCLING_ANGLE = np.pi/4
    RECYCLING_ZONE = np.array([75, 75])
    STORAGE = 10

    PRECISE_TORLERANCE_ANGLE = 0.035 # ~2 degree
    COLLECTION_DISTANCE = 15
    COLLECTION_APPROACH_DISTANCE = 50
    DETECTION_INTERVAL = 5 # TODO tune

    LOCAL_AVOID_MEMORY = 10 # s

    def __init__(self, portName, bottle_cam, 
                    beacon_cam, 
                    bottle_model = "arena.tflite") -> None:
        print("-- Initializing the robot...")

        print("Initializing the serial communication with Arduino...")
        self.ard = Arduino(portName=portName)
        if not self.ard.start():
            print("Initialization Failed: Check the connection to Arduino")
        
        print("Initializing the beacon camera...")
        self.localizer = Localization(camera_id=beacon_cam)
        self.localizer.start()
        print("Localization test...")
        for i in range(3):
            pose = self.localizer.get_pose()
            if not self._valid_pose(pose):
                print("Initialization Failed: ", i)
            else:
                break
        if not self._valid_pose(pose):
            pose = [np.array([100, 100]), np.pi/2]
        self.pose = pose
        #elif (pose[0][0] - 100) ** 2 + (pose[0][0] - 100) ** 2 > 28.29 ** 2 or abs(pose[1] - np.pi/2) > 0.2:
        #    print("[Localization] Wrong initial pose")
        #    pose = [[100, 100], np.pi/2]

        print("Initializing the bottle detection camera...")
        self.detector = Detector(
            model_path = bottle_model,
            camera_id= bottle_cam
        )
        ret = self.detector.get_nearest_bottle()
        while True:
            if ret is None:
                print("No bottle detected, please place one in 2m to test!")
                print("Another check will be executed in 5s")
                time.sleep(5)
            elif ret is False:
                print("Error in bottle detection camera")
                break
            else:
                print(f"the nearest bottle is in {ret[0]:.1f}cm away, {ret[1]/np.pi*180:.1f} degree.")
                break
            
        self.mc = MotionControl()

        self.bottle_collected = 0
        self.pose = pose
        self.__show_pose(self.pose)
        self.detection_timer = 0
        self.state = 0 # 0 Path follow, 1 Bottle detection 2 Bottle approaching 3 Bottle collection, 4 Obstacle avoidance
        self.target_bottle = None
        self.wps = self.INITIAL_PATH.copy()
        self.cmd = None
        self.cmd_timer_start = time.time()
        self.cmd_timeout = 0
        self.starter = time.time()
        self.local_avoid_timer = 0
        self.avoid_direction = False
        print("-- Initialization Done!")

    @staticmethod
    def _valid_pose(pose):
        if pose is None: return False
        if  pose[0][0] > 0 and pose[0][0] < 800 and pose[0][1] > 0 and pose[0][1] < 800:
            return True
        return False

    def get_pose(self):
        pose = self.localizer.get_pose()
        if self._valid_pose(pose):
            self.pose = pose
            self.ard.read()
            self.ard.get_displacements() # clear the displacements buffer        
        # use the last pose and odometry
        self.odometry(self.ard.get_displacements())

    @staticmethod
    def __show_pose(pose):
        return print(f"[Position] {pose[0][0]:.1f}, {pose[0][1]:.1f}, {pose[1]/np.pi*180.0:.1f}")
        
    def odometry(self, displacements):
        for dl, dr in displacements:
            p = (dl + dr)/2.0
            a = np.arctan2(dr - dl, 2*self.WHEEL_DISTANCE*0.8)
            t = self.pose[1] + a/2
            self.pose[0] = self.pose[0] + p*np.array([np.cos(t), np.sin(t)])
            self.pose[1] = (self.pose[1] + a) % 2*np.pi
            # print(f"[Odomtry] disp {dl}, {dr}; new pose {self.__show_pose(self.pose)}")


    def run_cmd(self, cmd: MotionCmd):
        # self.ard.stop() # TODO smoother change
        # execution
        if(cmd.dist > 50):
            # fast motion by speed mode
            speed = MotionControl.NOMINAL_SPEED 
            timeout = cmd.timeout4speed(speed)
            self.ard.move(cmd.dir, speed = int(speed/Protocol.MOTION_SPEED_FACTOR), timeout=int(timeout/0.1))
        else:
            # accurate motion by distance
            timeout = cmd.timeout4distance()
            self.ard.move(cmd.dir, distance=int(cmd.dist), timeout=int(timeout/0.1))

        # remeber it
        self.cmd = cmd
        self.cmd_timer_start = time.time()
        self.cmd_timeout = timeout
    
    def clear_cmd(self):
        self.cmd = None
        self.cmd_timeout = 0

    def should_detect_bottle(self) -> bool: 
        return self.detection_timer + self.DETECTION_INTERVAL > time.time()

    def angle2turn(self, target):
        return ((target - self.pose[1]) + np.pi) %(2*np.pi) - np.pi

    def local_avoidance(self):
        # todo
        # simple strategy: let the arduino do everything!
        # decide the direction
        cur_time = time.time()
        if cur_time > self.local_avoid_timer + self.LOCAL_AVOID_MEMORY:
            wp = self.mc.waypoint_list[0]
            dist_target, angle_target = self.mc._cart2polar(wp - self.pose[0])
            angletoturn = self.mc._get_angle_to_turn(self.pose[1], angle_target)
            self.avoid_direction = angletoturn < 0
        self.local_avoid_timer = cur_time
        self.ard.do_local_avoidance(self.avoid_direction)

    def start(self, debug = False):
        """Start the whole program!"""
        finished = False
        while not finished:
            starter = time.time()
            if debug:
                a = input("Debug mode, enter to continue")

            # read from arduino
            self.ard.read()

            if self.state == -1:
                if not self.ard.TASK_UNDERGOING:
                    print("Task finished")
                    self.state = 0
                else:
                    # using this waiting time to do localization
                    self.get_pose()
            elif self.state == 0: # path following
                # 
                if self.ard.OBS_WARN: #obstacle too close
                    self.state = 4
                    self.clear_cmd()
                elif self.should_detect_bottle():
                    self.state = 1
                    self.clear_cmd()
                else: # do path following
                    wp = self.wps[0]
                    # if we have reached
                    self.get_pose()
                    dist_target, angle_target = self.mc._cart2polar(wp - self.state[0])
                    if dist_target < self.WAYPOINT_TORLERANCE_DIST:
                        # ok, reached
                        self.ard.stop()
                        self.clear_cmd()
                        print("Waypoint Reached", wp)
                        self.wps = self.wps[1:]
                    else:
                        # if not, we check the angle
                        if self.cmd is None or self.cmd.dir == 3:
                            angle_to_turn = self.angle2turn(angle_target)
                            if abs(angle_to_turn) >  self.WAYPOINT_TORLERANCE_ANGLE:
                                # turn towards it
                                self.run_cmd(MotionCmd(1+1*(angle_to_turn<0)), \
                                    self.mc._angle2dist(abs(angle_to_turn)))
                            # go to it
                            elif self.cmd is None: 
                                self.run_cmd(MotionCmd(3, dist_target))
                        # else we finish turning first
            elif self.state == 1: # bottle detection
                if self.cmd is None:
                    b1 = time.time()
                    ret = self.detector.get_nearest_bottle()
                    b2 = time.time()
                    print(f"Bottle Detection takes {b2-b1:.2f}s")
                    if ret is None:
                        print("No bottle detected")
                        self.state = 0
                    elif ret == False:
                        print("[Error] Bottle Camera fails to get frame")
                        self.state = 0
                    else:
                        rd, ra = ret
                        print(f"Bottle dist: {rd:.2f}, angle:{ra/np.pi*180.0:.2f}")
                        if rd < 200:
                            self.target_bottle = [rd, ra]
                            if rd > self.COLLECTION_APPROACH_DISTANCE + 10:
                                self.state = 2 # bottle collection mode
                            elif rd < self.COLLECTION_DISTANCE:
                                # go back 
                                # the angle should be fine
                                self.run_cmd(MotionCmd(0, self.COLLECTION_DISTANCE - rd))
                                self.state = 3 # pick it
                            else:
                                if abs(ra) > self.PRECISE_TORLERANCE_ANGLE:
                                    self.run_cmd(MotionCmd(1+1*(ra<0)), \
                                        self.mc._angle2dist(abs(ra)))
                                    self.target_bottle[1] = 0 # trust in arduino
                                else:
                                    self.run_cmd(MotionCmd(3, rd - self.COLLECTION_DISTANCE))
                                    self.target_bottle[0] = self.COLLECTION_DISTANCE - 1
                                self.state = 3 # pick it
                        else:
                            # as if we didn't see it
                            print("too far")
                            self.state = 0
                    # reset should_detect_bottle
                    self.detection_timer = starter
                # else wait for execution
            elif self.state == 2: # appoach to it
                if self.ard.OBS_WARN: #obstacle too close
                    self.state = 4
                    self.clear_cmd()
                    self.target_bottle = None
                elif self.cmd is None:
                    # angle first              
                    angle_to_turn = self.target_bottle[1]
                    if abs(angle_to_turn) > self.PRECISE_TORLERANCE_ANGLE:
                        self.run_cmd(MotionCmd(1+1*(angle_to_turn<0)), \
                            self.mc._angle2dist(abs(angle_to_turn)))
                        self.target_bottle[1] = 0 # trust in arduino  
                    else:
                        self.run_cmd(MotionCmd(3, self.target_bottle[0] - self.COLLECTION_APPROACH_DISTANCE))
                        self.target_bottle[0] = self.COLLECTION_APPROACH_DISTANCE
                        self.state = 1 # re-detect the bottle
                # else wait for execution
            elif self.state == 3: # going to pick it
                if self.ard.OBS_WARN: #obstacle too close
                    self.state = 4
                    self.clear_cmd()
                    self.target_bottle = None
                elif self.cmd is None:
                    # check the distance
                    if self.target_bottle[0] > self.COLLECTION_DISTANCE:
                        self.run_cmd(MotionCmd(3, self.target_bottle[0]- self.COLLECTION_DISTANCE))
                        self.target_bottle[0] = self.COLLECTION_DISTANCE - 1
                    else: # ok to pick
                        self.ard.pick_up()
                        self.bottle_collected += 1
                        self.state = -1
                # else wait for the cmd execution
            elif self.state == 4:
                # local avoidance
                self.local_avoidance()
                self.state = -1 # wait for the task
            elif self.state == 5:
                # going back
                if self.ard.OBS_WARN: #obstacle too close
                    self.state = 4
                    self.clear_cmd()
                else:
                    wp = self.RECYCLING_ZONE
                    # path following                
                    self.get_pose()
                    dist_target, angle_target = self.mc._cart2polar(wp - self.state[0])
                    if dist_target < self.WAYPOINT_TORLERANCE_DIST:
                        # ok, reached
                        print("Recycling Reached", wp)
                        # turn to first
                        if self.cmd is None:
                            angle_to_turn = self.angle2turn(self.RECYCLING_ANGLE)
                            if abs(angle_to_turn) >  self.PRECISE_TORLERANCE_ANGLE:
                                self.run_cmd(MotionCmd(1+1*(angle_to_turn<0)), \
                                    self.mc._angle2dist(abs(angle_to_turn)))
                            else: # we can!
                                print("Emptying the storage...", wp)
                                self.ard.stop()
                                self.clear_cmd()
                                self.ard.empty_storage()
                                self.bottle_collected = 0
                                self.state = -1
                        # else wait for execution
                    else:
                        # if not, we check the angle
                        if self.cmd is None or self.cmd.dir == 3:
                            angle_to_turn = self.angle2turn(angle_target)
                            if abs(angle_to_turn) >  self.WAYPOINT_TORLERANCE_ANGLE:
                                # turn towards it
                                self.run_cmd(MotionCmd(1+1*(angle_to_turn<0)), \
                                    self.mc._angle2dist(abs(angle_to_turn)))
                            # go to it
                            elif self.cmd is None: 
                                self.run_cmd(MotionCmd(3, dist_target))
                            # else do not need modification
                        # else we finish turning first
            else:
                self.state = 0 # deal with the error state

            # check current cmd
            if self.cmd is not None and starter > self.cmd_timeout + self.cmd_timer_start:
                self.ard.stop()
                self.clear_cmd()

            # check if we should go back
            if not self.state == 5:
                if self.bottle_collected >= self.STORAGE:
                    print("so many bottles, let's go back")
                    self.state = 5
                else:
                    # running out of time! let's go back
                    cur_time = time.time()
                    if cur_time - self.starter > 8 * 60:
                        print("curtime", cur_time)
                        print("running out of time! let's go back")
                        self.state = 5

            ender = time.time()
            print(f"[Debug] One Loop takes {ender - starter:.2f}s")

if __name__ == "__main__":
    walle = Walle(beacon_cam=1, bottle_cam=0, port_name="/dev/ttyACM0")
    walle.start(debug=False)