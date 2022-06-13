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

# create an instance of Arduino Controller specifying the port name.
ard = Arduino(portName = "COM4")  
ard.start()

from set_up import*
from Localisation import*
from path_planning import*
from path_following import*

# Get initial state of the robot [x,y,orientation]
robot_state = localisation() 
# Define initial path and target to follow
path = init_path(robot_state[0], robot_state[1])
target = path[0,:]
visited_points = np.empty((0, 2))

# Loop over desired way points

#rt = RepeatedTimer(SAMPLING_TIME_MOTION, follow_path,robot_state, target) #call follow_path function each SAMPLING_TIME sec

#while not (path.size == 0):
#    time.sleep(SAMPLING_TIME_GOAL) # check path each SAMPLING_TIME_GOAL sec
    
    ##### Obstacle avoidance #####
    #if obstacle():
    #    rt.stop()
    #    while obstacle() :
    #        avoid_obstacle()
    #        time.sleep(SAMPLING_TIME_GOAL)
    #    robot_state = localisation() 
    #    target = update_target_obst()
    #    rt = RepeatedTimer(SAMPLING_TIME_MOTION, follow_path)
        
    ##### Bottle grasping ##### 
    #if bottle():
    #    rt.stop()
    #    while bottle() :
    #        time.sleep(SAMPLING_TIME_GOAL)
    #        while not bottle_ready():
    #            go_to_bottle()
    #        grasp_bottle() 
    #       
    #     robot_state = localisation() 
    #     target = update_target_bottle()
    #     rt = RepeatedTimer(SAMPLING_TIME_MOTION, follow_path)
    
    ##### Simple Motion #####
    
    #robot_state = localisation() 

#    if goal_reached(robot_state, target):
#        visited_points = np.append(visited_points, [target], axis=0)
#        path = np.delete(path, 0, axis=0)
#        target = path[0,:]
        
#rt.stop()
    #target = path[0,:]
    #while not goal_reached(robot_state[0:2],target):
    #    robot_state = localisation()
    #    follow_path(ard,robot_state,target)

    #visited_points = np.append(visited_points, [target],axis=0)
    #path = np.delete(path, 0, axis=0)

print(robot_state)
follow_path(ard,robot_state,[robot_state[0]-2,robot_state[1]])
for i in range(3):
    time.sleep(1)
    ard.read()
print(robot_state)
#ard.stop()

