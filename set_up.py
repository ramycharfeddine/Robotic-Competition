import sys
import os
import math
import cv2
import time
import numpy as np
import matplotlib.pyplot as plt
from statistics import mean
from numpy.linalg import norm 
from mpmath import cot
from threading import Timer

# Define arena dimensions
W = 8
H = 8

# Define robot's dimensions

LENGTH = 20*10**(-2)   # [m]  # half width of the robot
LENGTH_WHEELS = 26.5*10**(-2) #[m]  # half distance between wheels
RADIUS = 60*10**(-3)   # [m]  # wheel radius of the robot

# Initialize the camera for localisation: 
# In test room set exposure as luminosity = 0% and Intensity of colors = 12%
ERROR_CENTER_Y = 15
ERROR_CENTER_X = -5
cam_port = 0
cam = cv2.VideoCapture(cam_port,cv2.CAP_DSHOW)
cam.set(cv2.CAP_PROP_EXPOSURE,-7)

# Parameters to set up navigation
ERROR_POS_X = 60*10**(-2) 
ERROR_POS_Y = 60*10**(-2)
EPS_DIST  = np.sqrt(ERROR_POS_X**2 + ERROR_POS_Y**2)  # [m] # Accepted error between real and expected position 
EPS_THETA = math.radians(10)                  # [rad] # Accepted error between real and expected orientation

# Threading
SAMPLING_TIME_MOTION = 0.1 # update the motion each 0.1s
SAMPLING_TIME_GOAL = 0.01  # check if goal is reached every 0.01s


class RepeatedTimer(object):
    def __init__(self, interval, function, *args, **kwargs):
        self._timer     = None
        self.interval   = interval
        self.function   = function
        self.args       = args
        self.kwargs     = kwargs
        self.is_running = False
        self.start()
    
    def _run(self):
        self.is_running = False
        self.start()
        self.function(*self.args, **self.kwargs)
    
    def start(self):
        if not self.is_running:
            self._timer = Timer(self.interval, self._run)
            self._timer.start()
            self.is_running = True
    
    def stop(self):
        self._timer.cancel()
        self.is_running = False