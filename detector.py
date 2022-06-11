import sys
import time
import numpy as np

import cv2
from object_detector import ObjectDetector
from object_detector import ObjectDetectorOptions
import utils

from protocol import *

# in mm
detection_img_pts2 = np.float32([[-80, 60], [80, 60],
                [-220, 300], [220, 300]])
detection_img_pts1 = np.float32([[0, 480], [640, 480],
                [0, 200], [640, 200]])

class Detector:
    MAXIMUM_DISTANCE_VALIDE = 3.0 # m
    def __init__(self, 
                model_path = "/home/pi/rc/detect/bottle_efficientdet0_anr.tflite",
                camera_id = 0,
                img_width = 640,
                img_height = 480,
                max_results = 5,
                score_threshold = 0.4):

        # Start capturing video input from the camera
        self.cap = cv2.VideoCapture(camera_id)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, img_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, img_height)

        # Initialize the object detection model
        options = ObjectDetectorOptions(
            num_threads=4,
            score_threshold=score_threshold,
            max_results=max_results,
            enable_edgetpu=False)
        self.detector = ObjectDetector(model_path=model_path, options=options)
        
        # warp matrix
        self.M = cv2.getPerspectiveTransform(detection_img_pts1, detection_img_pts2)

    def __del__(self):
        self.end()

    def get_nearest_bottle(self,
                           repeat_read = 3 # to clear the buffer
                           ):
        """the distance and theta to the nearest bottle
        return: 
        - False: error in camera
        - None: no bottle detected
        - distance, theta
        """
        # read image
        for i in range(repeat_read):
            success, image = self.cap.read()
        if not success: return False # error in camera!
        
        # predict
        detections = self.detector.detect(image)

        # get bottles' real position            
        nr = 1000
        nt = 0
        for b in self._get_bottles(detections):
            center = self._bottle_ground(b)
            #area = bottle_area(b)
            real_pos, size = self._get_real_pos(center)
            #print(real_pos)
            r, theta = self._cart2polar(real_pos)
            #print(r/10, theta/np.pi*180)
            if r < nr:
                nr = r
                nt = theta
        
        if nr == 1000: return None
        
        return nr, nt 

    def end(self):
        self.cap.release()

    def _get_real_pos(self, pixel):
        p = np.append(pixel, 1).reshape((3,1))
        r = np.matmul(self.M, p).flatten()
        return r[:2]/r[2], r[2]

    @staticmethod
    def _get_bottles(detect):
        return [d.bounding_box for d in detect if d.categories[0].index == 0]

    @staticmethod
    def _bottle_area(box):
        length = box.right - box.left
        width = box.bottom - box.top
        return length*width

    @staticmethod
    def _bottle_center(box):
        return [(box.left+box.right)/2, (box.top+box.bottom)/2]

    @staticmethod
    def _bottle_ground(box):
        return [(box.left+box.right)/2, box.bottom]

    @staticmethod
    def _cart2polar(pos):
        r = np.sqrt(pos[0]**2 + pos[1]**2)
        theta = np.arctan2(pos[1], pos[0])
        return r/10, theta-np.pi/2

if __name__ == '__main__':
    from protocol import *

    ard = Arduino(portName = "/dev/ttyACM0")
    ard.start()
    detector = Detector()
    print("Detectior initialized")

    try:
        time.sleep(1)
        ard.read()

        ret = detector.get_nearest_bottle()
        if ret is None:
            print("No bottle detected")
        elif ret == False:
            print("Camera Error!")
        else:
            nr, nt = ret
            print(nr, nt)
            if nr > 200:
                print("No bottles in 2m")
            elif nr < 15:
                print("The bottle is too close")
            else:
                time.sleep(1)
                if abs(nt) > 0.1: # 5.7deg
                    if nt < 0:
                        ard.move(Protocol.RIGHT,distance = int(-nt/np.pi*90))
                    else:
                        ard.move(Protocol.LEFT, distance = int(nt/np.pi*90))
                    print('turn', int(nt/np.pi*90))
                time.sleep(1)
                ard.move(Protocol.FORWARD, distance = int(nr-15))
    finally:
        detector.close()
        ard.stop()