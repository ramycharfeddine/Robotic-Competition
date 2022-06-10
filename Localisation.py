import math
import cv2
from matplotlib.style import available
import numpy as np
import matplotlib.pyplot as plt

class Localization:
    # camera settings
    FRAME_WIDTH = 640
    FRAME_HEIGHT = 480
    # colors: Purple, Blue, Green, Red
    COLOR_HSV_RANGE = (
        ((128, 114, 77), (162, 255, 255)),
        ((77, 148, 148), (120, 255, 255)),
        ((49, 46, 116), (88, 255, 255)),
        ((0, 160, 0), (7, 255, 255)))
    COLOR_HSV_THRESHOLD = (10, 10, 10, 10)
    FRONT_VECTOR = np.array((FRAME_WIDTH, 0))

    # Define arena dimensions
    W = 8
    H = 8
    LED_POS = np.array([[0, W, W, 0], [0, 0, H, H]])

    def __init__(self, camera_id = 1, 
                    cam_error_center_x = 10, 
                    cam_error_center_y = 5,
                    camera_pos_bias = (0, -0.05) #m
                    ):
        self.cam_id = camera_id
        self.cam_error_center_x = cam_error_center_x
        self.cam_error_center_y = cam_error_center_y
        self.camera_pos_bias = camera_pos_bias

    def start(self):
        self.cam = cv2.VideoCapture(self.cam_id)
        self.cam.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)
        self.cam.set(cv2.CAP_PROP_EXPOSURE, 20)
        self.cam.set(cv2.CV_CAP_PROP_BUFFERSIZE, 3)
        self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, self.FRAME_WIDTH)
        self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, self.FRAME_HEIGHT)

    def frame(self, clear_buffer = True):
        ret, img = self.cam.read()
        if clear_buffer:
            for i in range(3):
                ret, img = self.cam.read()
        
        if not ret:
            print("[Error] Camera Error")
            return None

        return img
        
        
        # convert original image to hsv image 
        img_hsv = cv2.cvtColor(img_original, cv2.COLOR_BGR2HSV)
        
        # Blackout all what is outside of a circle
        mask = np.zeros_like(img_original)
        mask = cv2.circle(mask, (int(X_center_frame), int(Y_center_frame)), 160, (255,255,255), -1)
        img_original = cv2.bitwise_and(img_original, mask)


    def calibrate(self, img_original = None):
        if img_original is None:
            img_original = self.frame(False)
            if img_original is None:
                return
        
        # define the frame properties
        X_center_frame = self.FRAME_WIDTH/2 + self.cam_error_center_x
        Y_center_frame = self.FRAME_HEIGHT/2 + self.cam_error_center_y
        # center_frame = np.array([[X_center_frame],[Y_center_frame]])


        img_hsv = cv2.cvtColor(img_original, cv2.COLOR_BGR2HSV)
        
        for i in range(4):
            led_mask = self.color_filter(img_hsv, 
                self.COLOR_HSV_RANGE[i][0], 
                self.COLOR_HSV_RANGE[i][1],
                self.COLOR_HSV_THRESHOLD[i])
            img_led = cv2.bitwise_and(img_original, img_original, mask=led_mask)
            cv2.imshow('led'+str(i), img_led)

        # Visualize center for set up
        cv2.circle(img_original,(int(X_center_frame), int(Y_center_frame)), 10, (0,255,0))
        cv2.line(img_original, (int(X_center_frame), int(Y_center_frame)), (img_original.shape[1], int(Y_center_frame)), (0, 255, 0), 3) 
        cv2.imshow("calibration", img_original)

    @staticmethod
    def color_filter(img_hsv, hsv_thr_lower, hsv_thr_upper, thr):
        mask_led = cv2.inRange(img_hsv, hsv_thr_lower, hsv_thr_upper)
        # led      = cv2.bitwise_and(img_original, img_original, mask=mask_led)
        led      = cv2.bitwise_and(img_hsv, img_hsv, mask=mask_led)
        led      = cv2.cvtColor(led, cv2.COLOR_BGR2GRAY)
        (thresh, led) = cv2.threshold(led,thr, 255, cv2.THRESH_BINARY)
        led = cv2.morphologyEx(led, cv2.MORPH_CLOSE,
                                cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5)))
        led = cv2.morphologyEx(led, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3)))
        return led

    def get_leds(self, img_hsv):
        led_positions = np.zeros((2, 4))
        available = np.ones((4,1), dtype=bool)
        for i in range(4):
            try:
                led_mask = self.color_filter(img_hsv, 
                    self.COLOR_HSV_RANGE[i][0], 
                    self.COLOR_HSV_RANGE[i][1],
                    self.COLOR_HSV_THRESHOLD[i])
                moment_led = cv2.moments(led_mask)
                led_positions[0, i] = int(moment_led["m10"] / moment_led["m00"])
                led_positions[1, i] = int(moment_led["m01"] / moment_led["m00"])
            except:
                print(f"fail to detect color {i}")
                available[i] = 0
        return led_positions, available
    
    @staticmethod
    def angle(vector_1, vector_2):
        """from v1 to v2
        """
        #unit_vector_1 = vector_1 / np.linalg.norm(vector_1)
        #unit_vector_2 = vector_2 / np.linalg.norm(vector_2)
        #dot_product = np.dot(unit_vector_1, unit_vector_2)
        #angle = np.arccos(dot_product)
        #angle = np.arctan2(vector_2[1],vector_2[0]) \
        #            - np.arctan2(vector_1[1],vector_1[0])
        return np.arctan2(vector_2[1],vector_2[0]) \
                   - np.arctan2(vector_1[1],vector_1[0])
    
    @staticmethod
    def beacon_localize(led_vectors, indices):
        x1 = Localization.LED_POS[0, indices[0]]
        y1 = Localization.LED_POS[1, indices[0]]
        x2 = Localization.LED_POS[0, indices[1]]
        y2 = Localization.LED_POS[1, indices[1]]
        x3 = Localization.LED_POS[0, indices[2]]
        y3 = Localization.LED_POS[1, indices[2]]

        # Find angles between beacons        
        theta12 = Localization.angle(
            led_vectors[:,indices[0]],led_vectors[:,indices[1]])
        theta23 = Localization.angle(
            led_vectors[:,indices[1]],led_vectors[:,indices[2]])

        # Compute the modified beacon coordinates
        x1_p = x1 - x2
        y1_p = y1 - y2
        x3_p = x3 - x2
        y3_p = y3 - y2
        
        T12 = 1.0/np.tan(theta12)
        T23 = 1.0/np.tan(theta23)
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
            print("error in position with camera")
            return None
        
        # Compute the robot position {xR, yR}        
        xr = float(x2 + (k31_p*(y12_p-y23_p))/D)
        yr = float(y2 + (k31_p*(x23_p - x12_p))/D)

        return xr,yr

    @staticmethod
    def orientation_by_beacon(index, led_vector, position):
        tobeacon = Localization.angle(Localization.FRONT_VECTOR, led_vector)
        beacon_angle = np.arctan2(position[1] - Localization.LED_POS[1,index], position[0] - Localization.LED_POS[0,index])
        return np.pi + beacon_angle - tobeacon

    def localize(self, img):
        # Find the leds by color filtering
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        led_positions, available = self.get_leds(img_hsv)        
        if np.sum(available) < 3:
            print("No enough beacon for localization")
            return None

        # the center of the image
        X_center_frame = self.FRAME_WIDTH/2 + self.cam_error_center_x
        Y_center_frame = self.FRAME_HEIGHT/2 + self.cam_error_center_y
        center_frame = np.array([[X_center_frame],[Y_center_frame]])
        
        # calculate the position
        indices = np.where(available == True)[0] # Choose 3 beacons
        led_vectors = led_positions - center_frame
        position = self.beacon_localize(led_vectors, indices)
        if position is None:
            return None
            
        # calculate the orientation
        oris = np.zeros((len(indices),1))
        for i in indices:
            oris[i] = self.orientation_by_beacon(indices[i], led_vectors[:, indices[i]], position) % (2*np.pi)
        ori = np.mean(oris)
        print(oris)

        # camera position to center position


        return position, ori
    
    def get_state(self):
        
        img_original = self.frame()
        if img_original is None: # error
            return None        
        
        return self.localize(img_original)

if __name__ == "__main__":
    localizer = Localization()
    img = cv2.imread("Localisation90.png")
    state = localizer.localize(img)
    print(f'pos ({state[0][0]:.2f},{state[0][1]:.2f}), ori: {state[1]/np.pi*180:.1f}')
    localizer.calibrate(img_original=img)
    cv2.waitKey(0)


"""
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
    
    # Blackout all what is outside of a circle
    mask = np.zeros_like(img_original)
    mask = cv2.circle(mask, (int(X_center_frame), int(Y_center_frame)), 160, (255,255,255), -1)
    img_original = cv2.bitwise_and(img_original, mask)

    # saving image in local storage
    cv2.imwrite(project_path + "Localisation.png", img_original)
    cv2.imwrite(project_path + "Localisation_hsv.png", img_hsv)
    
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
    cv2.imwrite(project_path + "Ledwc" + str(tempi)+".png", led)
    moment_led    = cv2.moments(led)
    x = int(moment_led["m10"] / moment_led["m00"])
    y = int(moment_led["m01"] / moment_led["m00"])
    cv2.imwrite(project_path + "Led" + str(tempi)+".png", led)
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

    #img_original = cv2.imread("images/Localisation.png")
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
    
    # Find angles between beacons
    LED_vectors = LED - center_frame
    LED_angles = np.array([
        angle(LED_vectors[:,0],LED_vectors[:,1]), 
        angle(LED_vectors[:,1],LED_vectors[:,2]), 
        angle(LED_vectors[:,2],LED_vectors[:,3]), 
        angle(LED_vectors[:,3],LED_vectors[:,0])])
    LED_pos = np.array([[0, W, W, 0], [0, 0, H, H]])

    # Determine actual position of the robot
    robot_pos = Total_algo(LED_pos[0][0], LED_pos[1][0], LED_pos[0][1],LED_pos[1][1], LED_pos[0][2], LED_pos[1][2], LED_angles[0], LED_angles[1])
    
    # Determine the robot orientation
    Front_vector = (
        np.array([[img_original.shape[1]],[Y_center_frame]])\
         - center_frame).flatten()
    LED_angles_to_front = np.array([
        angle(Front_vector,LED_vectors[:,0]), 
        angle(Front_vector,LED_vectors[:,1]), 
        angle(Front_vector,LED_vectors[:,2]), 
        angle(Front_vector,LED_vectors[:,3])])
    print(LED_angles_to_front)
    idx = np.where(LED_angles_to_front < 0)
    print(idx)
    LED_angles_to_front = [ LED_angles_to_front[index] for index in idx[0]]
    print(LED_angles_to_front)
    #robot_to_LED = math.degrees(np.arctan2(LED_pos[1,1] - robot_pos[1], LED_pos[0,1] - robot_pos[0]))
    #theta_robot = math.radians(get_angle_to_turn(math.degrees(LED_angles_to_front[0]),robot_to_LED))
    robot_to_LED = np.array([
        np.arctan2(LED_pos[1,idx[0][0]] - robot_pos[1], LED_pos[0,idx[0][0]] - robot_pos[0]),
        np.arctan2(LED_pos[1,idx[0][1]] - robot_pos[1], LED_pos[0,idx[0][1]] - robot_pos[0])])
    #robot_to_LED = np.array([np.arctan2(LED_pos[1,0] - robot_pos[1], LED_pos[0,0] - robot_pos[0]),np.arctan2(LED_pos[1,1] - robot_pos[1], LED_pos[0,1] - robot_pos[0]),np.arctan2(LED_pos[1,2] - robot_pos[1], LED_pos[0,2] - robot_pos[0]),np.arctan2(LED_pos[1,3] - robot_pos[1], LED_pos[0,3] - robot_pos[0])])
    #theta_robot = LED_angles_to_front[0] - np.arctan2(LED_pos[1,0] - robot_pos[1], LED_pos[0,0] - robot_pos[0]),
    theta_robot = robot_to_LED+np.absolute(LED_angles_to_front)
    #print(theta_robot)
    theta_robot = np.mean(theta_robot)

    return np.array([robot_pos[0], robot_pos[1], theta_robot])

print(localisation())
"""