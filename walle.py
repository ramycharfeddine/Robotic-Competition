from protocol import Protocol, Arduino
from Localisation import Localization
# from detector import Detector
from motion_control import MotionControl, MotionCmd
import numpy as np
import time

"""
unit: cm, s
"""

class Walle:

    NOMINAL_SPEED_VALUE = MotionControl.NOMINAL_SPEED/Protocol.MOTION_SPEED_FACTOR
    DETECTION_DISTANCE_INTERVAL = 80
    DIST_BOTTLE_HIGH = 50
    DIST_BOTTLE_COLLECT = 20
    EPS_BOTTLE_COLLECT = 2
    STORAGE = 10
    RECYCLING_ZONE = [0.5, 0.5]
    WHEEL_DISTANCE = MotionControl.WHEEL_SPACING_FACTOR # TODO

    def __init__(self, 
                    port_name = "/dev/ttyACM0", 
                    localize_camid = 1,
                    bottle_camid = 0, 
                    model_path = "/home/pi/rc/detect/bottle_efficientdet0_anr.tflite") -> None:
        print("-- Initializing the robot...")

        print("Initializing the serial communication with Arduino...")
        self.ard = Arduino(portName=port_name)
        if not self.ard.start():
            print("Initialization Failed: Check the connection to Arduino")
        
        print("Initializing the localization camera...")
        self.localizer = Localization(camera_id=localize_camid)
        self.localizer.start()
        print("Localization test...")
        pose = self.localizer.get_pose()
        if not self._valid_pose(pose):
            print("Initialization Failed: Check the localization")
            pose = [[100, 100], np.pi/2]
        #elif (pose[0][0] - 100) ** 2 + (pose[0][0] - 100) ** 2 > 28.29 ** 2 or abs(pose[1] - np.pi/2) > 0.2:
        #    print("[Localization] Wrong initial pose")
        #    pose = [[100, 100], np.pi/2]

   
        
        """
        print("Initializing the bottle detection camera...")
        self.detector = Detector(
            model_path = model_path,
            camera_id=bottle_camid
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
        """
        self.mc = MotionControl()

        self.bottle_collected = 0
        self.pose = pose
        self.distance_from_last_detection = 0 # the counter for bottle detection
        self.state = 0 # Path follow, Bottle collection, Obstacle avoidance
        self.wpthings = 0 # noting - 0; collect bottle - 1 ; 2 - check the bottle; 3 - going back
        # self.check_bottle_waypoint_finished = False
        # self.collect_waypoint_finished = False
        # self.waypoint_going_back = False
        self.PAUSE = False
        print("-- Initialization Done!")

    @staticmethod
    def _valid_pose(pose):
        if pose is None: return False
        if  pose[0][0] > 0 and pose[0][0] < 800 and pose[0][1] > 0 and pose[0][1] < 800:
            return True
        return False
    

    def __del__(self):
        self.end()

    def end(self):
        self.ard.stop()
        self.ard.end()
        self.localizer.end()
        # self.detector.end()

    def get_pose(self):
        pose = self.localizer.get_pose()
        if self._valid_pose(pose):
            self.pose = pose
            self.ard.read()
            self.ard.get_displacements() # clear the displacements buffer
            return self.pose
        
        # use the last pose and odometry
        self.odometry(self.ard.get_displacements())
        return self.pose

    @staticmethod
    def __show_pose(pose):
        return f"{pose[0][0]:.1f}, {pose[0][1]:.1f}, {pose[1]/np.pi*180.0:.1f}"
        
    def odometry(self, displacements):
        for dl, dr in displacements:
            p = (dl + dr)/2.0
            a = np.arctan2(dr - dl, self.WHEEL_DISTANCE)
            t = self.pose[1] + a/2
            self.pose[0] = self.pose[0] + p*np.array([np.cos(t), np.sin(t)])
            self.pose[1] += a
            print(f"[Odomtry] disp {dl}, {dr}; new pose {self.__show_pose(self.pose)}")

    def run_motion_cmd(self, cmd:MotionCmd, fast = None) -> None:
        if fast is None:
            # decide it by distance
            fast = cmd.dist > 50
        if fast:
            # fast motion by speed mode
            speed = MotionControl.NOMINAL_SPEED 
            timeout = cmd.timeout4speed(speed)
            self.ard.move(cmd.dir, speed = int(speed/Protocol.MOTION_SPEED_FACTOR), timeout=int(timeout/0.1))
        else:
            # accurate motion by distance
            timeout = cmd.timeout4distance()
            self.ard.move(cmd.dir, distance=int(cmd.dist), timeout=int(timeout/0.1))
        time.sleep(timeout)


    def should_detect_bottles(self):
        if self.wpthings == 0:
            return self.distance_from_last_detection > self.DETECTION_DISTANCE_INTERVAL
        else:
            return False
    
    def special_waypoint(self):
        if self.wpthings == 0:
            return
        elif self.wpthings == 1:
            self.distance_from_last_detection = self.DETECTION_DISTANCE_INTERVAL + 1
            self.wpthings = 0
        elif self.wpthings == 2:
            # collection
            self.ard.pick_up()
            self.state = 2 #
            self.wpthings = 0
            self.bottle_collected += 1
        elif self.wpthings == 3:
            # empty the storage
            # turn to 45 deg
            angle_rotation = np.pi/4 - self.pose[1]
            self.ard.move(1+1*(angle_rotation>0), 
                                    distance = self.mc._angle2dist(angle_rotation))
            self.ard.empty_storage() # TODO in Arduino: shake?
            self.state = 3 # empty
            self.bottle_collected = 0
            self.wpthings = 0
        # else: it's not special

    def follow_path(self):
        # Localization
        # should we stop first to get a better image?

        # follow the path
        cmds = self.mc.go_to_next_waypoint(self.pose) # if reached, the list is empty
        for cmd in cmds: 
            print("cmd:", cmd.dir, cmd.dist)
            self.distance_from_last_detection += cmd.dist
            self.run_motion_cmd(cmd)
        
        if len(cmds) == 0:
            self.special_waypoint()

    def local_avoidance(self):
        # todo
        # simple strategy: let the arduino do everything!
        self.ard.do_local_avoidance()

    def pause(self):
        self.ard.stop()
        self.PAUSE = True

    def start(self, state = None, debug = False):
        """Start the whole program!

        Strategy:
        """
        if not (state is None):
            self.state = state # maybe useful for resume
        finished = False
        starter = time.time()
        while not finished: 
            if debug:
                a = input("Debug mode, enter to continue")
            # TODO: check pause

            self.ard.read() # deal with the info from Arduino
            if self.ard.OBS_WARN and not self.state == 1:
                # local avoidance
                self.local_avoidance()
                # Motion stopped because of the obstacle in front
                self.state = 1
            
            if self.ard.TASK_UNDERGOING:
                continue
            else:
                self.state = 0

            # update the displacement
            # self.odometry(self.ard.get_displacements())

            # localization
            self.pose = self.get_pose() 
            #if pose is None: # some error in localization                
            #    print("[Error] top Camera fails to get frame")
            #    self.ard.move(Protocol.FORWARD, distance=10)
            #    continue
            # self.pose = pose
            print(f"current pose: {self.pose[0]}, ori: {self.pose[1]*180.0/np.pi}")

            if self.state == 0: 
                # main task
                if self.should_detect_bottles():
                    # detect the bottles
                    """
                    ret = self.detector.get_nearest_bottle()
                    self.distance_from_last_detection = 0
                    if ret is None:
                        # No bottle detected
                        pass
                    elif ret == False:
                        # errors in camera         
                        print("[Error] front Camera fails to get frame")
                    else:
                        rd, ra = ret
                        if rd > self.DIST_BOTTLE_HIGH:
                            dist_to_bottle = rd-self.DIST_BOTTLE_HIGH
                            x_bottle = self.pose[0][0] + dist_to_bottle*np.sin(ra)
                            y_bottle = self.pose[0][1] + dist_to_bottle*np.sin(ra)
                            self.mc.insert_waypoint([x_bottle,y_bottle])
                            
                            self.wpthings = 2
                            # self.check_bottle_waypoint_finished = True
                        elif rd > self.DIST_BOTTLE_COLLECT - self.EPS_BOTTLE_COLLECT:
                            dist_to_bottle = rd-self.DIST_BOTTLE_COLLECT
                            x_bottle = self.pose[0][0] + dist_to_bottle*np.sin(ra)
                            y_bottle = self.pose[0][1] + dist_to_bottle*np.sin(ra)
                            self.mc.insert_waypoint([x_bottle,y_bottle])
                            
                            self.wpthings = 1
                            #self.collect_waypoint_finished = True
                        else: # too close
                            self.ard.move(Protocol.BACKWARD, distance= self.DIST_BOTTLE_COLLECT - rd + 10)
                            # self.check_bottle_waypoint_finished = True 
                            self.wpthings = 2                           
                    """    
                else:
                    # path following
                    self.follow_path()
            elif self.state == 1:# local avoidance
                continue

            # decide next waypoint
            # TODO
            if self.bottle_collected >= self.STORAGE:
                self.mc.insert_waypoint(self.RECYCLING_ZONE)
                self.wpthings = 3
                #self.waypoint_going_back = True
            else:
                # running out of time! let's go back
                cur_time = time.time()
                if cur_time - starter < 8 * 60:
                    self.mc.insert_waypoint(self.RECYCLING_ZONE)
                    self.wpthings = 3
                    #self.waypoint_going_back = True
                    self.check_bottle_waypoint_finished = False
                    self.collect_waypoint_finished = False

if __name__ == "__main__":
    walle = Walle(localize_camid=0, port_name="COM4")
    walle.start(debug=True)