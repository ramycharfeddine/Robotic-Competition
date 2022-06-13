import math
from tkinter import image_types
import cv2
import numpy as np
import matplotlib.pyplot as plt
from mpmath import cot

from set_up import*
project_path = "C:\\Users\\User\\Documents\\EPFL Master semestre 2\\Robotic Competition\\Project_Code\\images\\"

def frame():   
    
    # reading the input using the camera
    cam_check, img_original = cam.read()
       
    if not cam_check:
        return "Failed to grab frame"
    
    # define the frame properties
    Y_center_frame = img_original.shape[0]/2 + ERROR_CENTER_Y
    X_center_frame = img_original.shape[1]/2 + ERROR_CENTER_X
    center_frame = np.array([[X_center_frame],[Y_center_frame]])
        
    # convert original image to hsv image 
    img_hsv = cv2.cvtColor(img_original, cv2.COLOR_BGR2HSV)
    
    # Visualize center for set up
    #cv2.circle(img_original,(int(X_center_frame), int(Y_center_frame)), 10, (0,255,0))
    #cv2.line(img_original, (int(X_center_frame), int(Y_center_frame)), (img_original.shape[1], int(Y_center_frame)), (0, 255, 0), 3) 
    
    # Blackout all what is outside of a circle
    mask = np.zeros_like(img_original)
    mask = cv2.circle(mask, (int(X_center_frame), int(Y_center_frame)), 160, (255,255,255), -1)
    img_original = cv2.bitwise_and(img_original, mask)

    # saving image in local storage
    cv2.imwrite(project_path + "Localisation_test.png", img_original)
    cv2.imwrite(project_path + "Localisation_hsv_test.png", img_hsv)
    
    return img_original,img_hsv,X_center_frame,Y_center_frame,center_frame

global tempi
tempi = 0
def led_position(img_hsv, img_original, hsv_thr_lower, hsv_thr_upper, thr):
    
    mask_led = cv2.inRange(img_hsv, hsv_thr_lower, hsv_thr_upper)
    led      = cv2.bitwise_and(img_original, img_original, mask=mask_led)
    led      = cv2.cvtColor(led, cv2.COLOR_BGR2GRAY)
    (thresh, led) = cv2.threshold(led,thr, 255, cv2.THRESH_BINARY)
    led = cv2.morphologyEx(led, cv2.MORPH_CLOSE, np.ones((5,5),np.uint8))
    led = cv2.morphologyEx(led, cv2.MORPH_OPEN, np.ones((2,2),np.uint8))
    global tempi
    cv2.imwrite(project_path + "Ledwc_test" + str(tempi)+".png", led)
    moment_led    = cv2.moments(led)
    if moment_led["m00"]!=0:
        x = int(moment_led["m10"] / moment_led["m00"])
        y = int(moment_led["m01"] / moment_led["m00"])
    else:
        x = 0
        y = 0
    #cv2.imwrite(project_path + "Led" + str(tempi)+".png", led)
    tempi = tempi+1
    return np.array([[x],[y]])


def angle(vector_1, vector_2):
    #unit_vector_1 = vector_1 / np.linalg.norm(vector_1)
    #unit_vector_2 = vector_2 / np.linalg.norm(vector_2)
    #dot_product = np.dot(unit_vector_1, unit_vector_2)
    #angle = np.arccos(dot_product)
    angle = np.arctan2(vector_2[1],vector_2[0]) - np.arctan2(vector_1[1],vector_1[0])
    return angle


def circle(L, angle):
    circle = np.array([[L/2], [L/2*np.tan(np.pi/2 - angle)], [(𝐿/2)/np.cos(np.pi/2 - angle)]])
    return circle


def pos(x1,y1,r1,x2,y2,r2):
    a = 2*(x1-x2)
    b = 2*(y1-y2)
    c = (r2**2)-(r1**2)+(x1**2)-(x2**2)+(y1**2)-(y2**2)    
    d = (y1**2)-(r1**2)+((c**2)/(a**2))-((2*x1*c)/a)+(x1**2)
    e = ((2*b*x1)/a)-((2*c*b)/(a**2))-2*y1
    f = 1 + ((b**2)/(a**2))
    
    ysol1 = (-e+np.sqrt((e**2)-4*d*f))/(2*f)
    ysol2 = (-e-np.sqrt((e**2)-4*d*f))/(2*f)
    
    xsol1 = (c-b*ysol1)/a
    xsol2 = (c-b*ysol2)/a
    
    return np.array([[xsol1,xsol2],[ysol1,ysol2]])


def Total_algo (x1, y1, x2, y2, x3, y3, theta12, theta23):
    
    # Compute the modified beacon coordinates
    x1_p = x1 - x2
    y1_p = y1 - y2
    x3_p = x3 - x2
    y3_p = y3 - y2
    
    T12 = cot(theta12)
    T23 = cot(theta23)
    T31 = (1 - (T12*T23))/(T12 + T23)

    # Compute the modified circle center coordinates
    
    x12_p = x1_p + T12*y1_p
    y12_p = y1_p - T12*x1_p
    x23_p = x3_p - T23*y3_p
    y23_p = y3_p + T23*x3_p
    x31_p = (x3_p + x1_p) + T31*(y3_p - y1_p)
    y31_p = (y3_p + y1_p) - T31*(x3_p - x1_p)
    
    k31_p = x1_p*x3_p + y1_p*y3_p + T31*(x1_p*y3_p - x3_p*y1_p)
    
    # Compute D (if D = 0, return with an error)
    
    D = (x12_p - x23_p)*(y23_p - y31_p) - (y12_p - y23_p)*(x23_p - x31_p)
    
    if (D == 0):
        return print("error in position with camera")
    
    # Compute the robot position {xR, yR}
    
    xr = float(x2 + (k31_p*(y12_p-y23_p))/D)
    yr = float(y2 + (k31_p*(x23_p - x12_p))/D)

    return np.array([xr,yr])

def get_angle_to_turn(target_angle, robot_angle): # target_angle, robot_angle in deg
    phi = (robot_angle-target_angle) % 360
    sign = -1
    # used to calculate sign
    if not ((phi >= 0 and phi <= 180) or (
            phi <= -180 and phi >= -360)):
        sign = 1
    if phi > 180:
        result = 360-phi
    else:
        result = phi
    return result*sign

def localisation():
    
    # Read frame
    img_original,img_hsv,X_center_frame,Y_center_frame,center_frame = frame()

    #img_original = cv2.imread("images/Localisation0.png")
    #img_hsv = cv2.imread("images/Localisation_hsv.png")

     # define the frame properties
    Y_center_frame = img_original.shape[0]/2 + ERROR_CENTER_Y
    X_center_frame = img_original.shape[1]/2 + ERROR_CENTER_X
    center_frame = np.array([[X_center_frame],[Y_center_frame]])
    
    # Find the position of the beacons on the image
    LED = np.zeros((2, 4))
    LED[:,0:1] = led_position(img_hsv, img_original, np.array([128, 114, 77]), np.array([162, 255, 255]), 10)  # Purple Beacon
    LED[:,1:2] = led_position(img_hsv, img_original, np.array([77, 148, 148]), np.array([120, 255, 255]), 10) # Blue Beacon
    LED[:,2:3] = led_position(img_hsv, img_original, np.array([49, 46,116]), np.array([88, 255, 255]), 10)    # Green Beacon
    LED[:,3:4] = led_position(img_hsv, img_original, np.array([0, 160, 0]), np.array([7, 255, 255]), 10)  # Red Beacon
    
    idx_led = np.argwhere(np.all(LED[..., :] == 0, axis=0))
    LED = np.delete(LED, idx_led, axis=1)

    # Find angles between beacons
    LED_vectors = LED - center_frame
    LED_angles = []
    for i in range(LED_vectors.shape[1]-1):
        LED_angles = np.concatenate((LED_angles,[angle(LED_vectors[:,i],LED_vectors[:,i+1])])) 
    LED_angles = np.concatenate((LED_angles,[angle(LED_vectors[:,(LED_vectors.shape[1]-1)],LED_vectors[:,0])]))

    #LED_angles = np.array([angle(LED_vectors[:,0],LED_vectors[:,1]), angle(LED_vectors[:,1],LED_vectors[:,2]), angle(LED_vectors[:,2],LED_vectors[:,3]), angle(LED_vectors[:,3],LED_vectors[:,0])])
    LED_pos = np.array([[0, W, W, 0], [0, 0, H, H]])
    LED_pos = np.delete(LED_pos, idx_led, axis=1)

    # Determine actual position of the robot
    robot_pos = Total_algo(LED_pos[0][0], LED_pos[1][0], LED_pos[0][1],LED_pos[1][1], LED_pos[0][2], LED_pos[1][2], LED_angles[0], LED_angles[1])
    
    # Determine the robot orientation
    Front_vector = (np.array([[img_original.shape[1]],[Y_center_frame]]) - center_frame).flatten()
    LED_angles_to_front = []
    for i in range(LED_vectors.shape[1]):
        LED_angles_to_front = np.concatenate((LED_angles_to_front,[angle(Front_vector,LED_vectors[:,i])]))

    #LED_angles_to_front = np.array([angle(Front_vector,LED_vectors[:,0]), angle(Front_vector,LED_vectors[:,1]), angle(Front_vector,LED_vectors[:,2]), angle(Front_vector,LED_vectors[:,3])])
    idx = np.where(LED_angles_to_front < 0)
    print(idx)
    LED_angles_to_front = [LED_angles_to_front[index] for index in idx[0]]
    #robot_to_LED = math.degrees(np.arctan2(LED_pos[1,1] - robot_pos[1], LED_pos[0,1] - robot_pos[0]))
    #theta_robot = math.radians(get_angle_to_turn(math.degrees(LED_angles_to_front[0]),robot_to_LED))
    robot_to_LED = []
    for i in range(np.size(idx)):
        robot_to_LED = np.concatenate((robot_to_LED,[np.arctan2(LED_pos[1,idx[0][i]] - robot_pos[1], LED_pos[0,idx[0][i]] - robot_pos[0])]))

    #robot_to_LED = np.array([np.arctan2(LED_pos[1,idx[0][0]] - robot_pos[1], LED_pos[0,idx[0][0]] - robot_pos[0]),np.arctan2(LED_pos[1,idx[0][1]] - robot_pos[1], LED_pos[0,idx[0][1]] - robot_pos[0])])
    #robot_to_LED = np.array([np.arctan2(LED_pos[1,0] - robot_pos[1], LED_pos[0,0] - robot_pos[0]),np.arctan2(LED_pos[1,1] - robot_pos[1], LED_pos[0,1] - robot_pos[0]),np.arctan2(LED_pos[1,2] - robot_pos[1], LED_pos[0,2] - robot_pos[0]),np.arctan2(LED_pos[1,3] - robot_pos[1], LED_pos[0,3] - robot_pos[0])])
    #theta_robot = LED_angles_to_front[0] - np.arctan2(LED_pos[1,0] - robot_pos[1], LED_pos[0,0] - robot_pos[0]),
    theta_robot = np.mean(robot_to_LED)+np.mean(np.absolute(LED_angles_to_front))
    
    if (0 in idx_led):
        print("d")
        if (2 in idx[0][:]):
            theta_robot = theta_robot-2*np.pi
    elif (3 in idx_led):
        print("a")
        #if (0 in idx[0][:]):
        #    theta_robot = theta_robot-np.pi
    elif (0 in idx[0][:]) and (3 in idx[0][:]):
        print("e")
        theta_robot = theta_robot-np.pi

    return np.array([robot_pos[0], robot_pos[1], theta_robot])

print(localisation())