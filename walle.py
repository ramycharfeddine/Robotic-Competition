from protocol import Protocol, Arduino
from Localisation import Localization
from detector import Detector
from motion_control import MotionControl, MotionCmd
import numpy as np
import time

"""
unit: cm, s
"""

class Walle:
    NOMINAL_SPEED_VALUE = MotionControl.NOMINAL_SPEED/Protocol.MOTION_SPEED_FACTOR
    BOTTLE_DISTANCE_STAGE1 = 0.3
    DETECTION_DISTANCE_INTERVAL = 80
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
        
        print("Initializing the localizetion camera...")
        self.localizer = Localization(camera_id=localize_camid)
        self.localizer.start()
        print("Localization test...")
        pose = self.localizer.get_pose()
        if pose is None:
            print("Initialization Failed: Check the localization")
        
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
        
        self.mc = MotionControl()

        self.bottle_collected = 0
        self.pose = pose
        self.distance_from_last_detection = 0 # the counter for bottle detection
        self.state = 'P' # Path follow, Bottle collection, Obstacle avoidance
        print("-- Initialization Done!")

    def __del__(self):
        self.end()

    def end(self):
        self.ard.stop()
        self.ard.end()
        self.localizer.end()
        self.detector.end()

    def run_motion_cmd(self, cmd:MotionCmd, fast = None) -> None:
        if fast is None:
            # decide it by distance
            fast = cmd.dist > 0.5
        if fast:
            # fast motion by speed mode
            speed = self.NOMINAL_SPEED_VALUE
            timeout = cmd.timeout4speed(speed)
            self.ard.move(cmd.dir, speed = speed, timeout=timeout)
        else:
            # accurate motion by distance
            self.ard.move(cmd.dir, cmd.dist, cmd.timeout4distance())

    def should_detect_bottles(self):
        return self.distance_from_last_detection > self.DETECTION_DISTANCE_INTERVAL
    
    def follow_path(self):
        # Localization
        # should we stop first to get a better image?
        pose = self.localizer.get_pose() 
        if pose is None: # some error in localization                
            print("[Error] top Camera fails to get frame")
            self.follow_path() # redo it TODO:what if kept failing
        self.pose = pose

        # follow the path
        cmds = self.mc.go_to_next_waypoint(self.pose) # if reached, the list is empty
        for cmd in cmds: 
            self.distance_from_last_detection += cmd.dist
            self.run_motion_cmd(cmd)

    def local_avoidance(self):
        # TODO: 
        self.ard.do_local_avoidance()

    def start(self, state = None):
        """Start the whole program!

        Strategy:
        """
        if not (state is None):
            self.state = state # maybe useful for resume
        finished = False
        while not finished: 
            self.ard.read() # deal with the info from Arduino
            if self.ard.OBS_WARN:
                # Motion stopped because of the obstacle in front
                self.state

            # Check the state
            if self.state == "B1":
                # bottle detection stage 1
                # we should go to the next waypoint to fetch the bottle
                pass

            if self.should_detect_bottles():
                # bottle detection
                ret = self.detector.get_nearest_bottle()
                if ret is None:
                    # No bottle detected
                    pass
                elif ret == False:
                    # errors in camera         
                    print("[Error] front Camera fails to get frame")
                else:
                    rr, rt = ret
                    # do bottle collection
                    # go to stage 1

            pass
