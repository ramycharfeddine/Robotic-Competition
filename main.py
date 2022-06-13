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

from protocol import*
from set_up import*
from Localisation import*
from path_planning import*
from path_following import*

# create an instance of Arduino Controller specifying the port name.
ard = Arduino(portName = "COM4")  
ard.start()

try:
    # Initialize the setup
    robot_state = localisation()
    path = init_path(robot_state[0], robot_state[1])
finally:
    ard.end()
