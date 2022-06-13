import time
import cv2
import numpy as np

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
    CAMERA_EXPOSURE = -7

    # Define arena dimensions
    W = 800
    H = 800
    LED_POS = np.array([[0, W, W, 0], [0, 0, H, H]])

    def __init__(self, camera_id = 1, 
                    cam_error_center_x = 10, 
                    cam_error_center_y = 5,
                    camera_pos_bias = (0, -5) 
                    ):
        self.cam_id = camera_id
        self.X_center_frame = int(self.FRAME_WIDTH/2 + cam_error_center_x)
        self.Y_center_frame = int(self.FRAME_HEIGHT/2 + cam_error_center_y)
        self.camera_pos_bias = camera_pos_bias
    
    def __del__(self):
        self.end()

    def start(self):
        self.cam = cv2.VideoCapture(self.cam_id)
        self.cam.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)
        self.cam.set(cv2.CAP_PROP_EXPOSURE, self.CAMERA_EXPOSURE)
        self.cam.set(cv2.CAP_PROP_BUFFERSIZE, 3)
        self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, self.FRAME_WIDTH)
        self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, self.FRAME_HEIGHT)
    
    def end(self):
        cv2.destroyAllWindows()
        print("[Localization] Closing camera...")
        self.cam.release()

    def _frame(self, clear_buffer = True):
        self.cam.set(cv2.CAP_PROP_EXPOSURE, self.CAMERA_EXPOSURE)
        ret, img = self.cam.read()
        if clear_buffer:
            for i in range(3):
                ret, img = self.cam.read()
        
        if not ret:
            return None

        mask = np.zeros_like(img)
        cv2.circle(mask, (self.X_center_frame, self.Y_center_frame), 165, (255, 255, 255), -1)
        img = cv2.bitwise_and(img, mask)

        return img

    def calibrate(self, img_original = None):
        if img_original is None:
            img_original = self._frame(False)
            if img_original is None:
                return
        
        img = img_original.copy()        
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        
        for i in range(4):
            led_mask = self._color_filter(img_hsv, 
                self.COLOR_HSV_RANGE[i][0], 
                self.COLOR_HSV_RANGE[i][1],
                self.COLOR_HSV_THRESHOLD[i])
            img_led = cv2.bitwise_and(img, img, mask=led_mask)
            
            moment_led = cv2.moments(img_led)
            x = int(moment_led["m10"] / moment_led["m00"])
            y = int(moment_led["m01"] / moment_led["m00"])
            cv2.circle(img_led, (x,y), 10, (255,255,255), 2)

            cv2.imshow('led'+str(i), img_led)

        # Visualize center for set up
        cv2.circle(img,(int(self.X_center_frame), int(self.Y_center_frame)), 10, (0,255,0))
        cv2.line(img, (int(self.X_center_frame), int(self.Y_center_frame)), (img.shape[1], int(self.Y_center_frame)), (0, 255, 0), 3) 
        cv2.imshow("calibration", img)

    @staticmethod
    def _color_filter(img_hsv, hsv_thr_lower, hsv_thr_upper, thr):
        mask_led = cv2.inRange(img_hsv, hsv_thr_lower, hsv_thr_upper)
        # led      = cv2.bitwise_and(img_original, img_original, mask=mask_led)
        led      = cv2.bitwise_and(img_hsv, img_hsv, mask=mask_led)
        led      = cv2.cvtColor(led, cv2.COLOR_BGR2GRAY)
        (thresh, led) = cv2.threshold(led,thr, 255, cv2.THRESH_BINARY)
        led = cv2.morphologyEx(led, cv2.MORPH_CLOSE,
                                cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5)))
        led = cv2.morphologyEx(led, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3)))
        return led

    def _get_leds(self, img_hsv):
        led_positions = np.zeros((2, 4))
        available = np.ones((4,1), dtype=bool)
        for i in range(4):
            try:
                led_mask = self._color_filter(img_hsv, 
                    self.COLOR_HSV_RANGE[i][0], 
                    self.COLOR_HSV_RANGE[i][1],
                    self.COLOR_HSV_THRESHOLD[i])
                moment_led = cv2.moments(led_mask)
                led_positions[0, i] = int(moment_led["m10"] / moment_led["m00"])
                led_positions[1, i] = int(moment_led["m01"] / moment_led["m00"])
            except:
                print(f"[Localization] fail to detect color {i}")
                available[i] = 0
        return led_positions, available
    
    @staticmethod
    def _angle(vector_1, vector_2):
        """from v1 to v2
        """
        return np.arctan2(vector_2[1],vector_2[0]) \
                   - np.arctan2(vector_1[1],vector_1[0])
    
    @staticmethod
    def _beacon_localize(led_vectors, indices):
        x1 = Localization.LED_POS[0, indices[0]]
        y1 = Localization.LED_POS[1, indices[0]]
        x2 = Localization.LED_POS[0, indices[1]]
        y2 = Localization.LED_POS[1, indices[1]]
        x3 = Localization.LED_POS[0, indices[2]]
        y3 = Localization.LED_POS[1, indices[2]]

        # Find angles between beacons        
        theta12 = Localization._angle(
            led_vectors[:,indices[0]],led_vectors[:,indices[1]])
        theta23 = Localization._angle(
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
            print("[Localization] error in position with camera")
            return None
        
        # Compute the robot position {xR, yR}        
        xr = float(x2 + (k31_p*(y12_p-y23_p))/D)
        yr = float(y2 + (k31_p*(x23_p - x12_p))/D)

        return xr,yr

    @staticmethod
    def _orientation_by_beacon(index, led_vector, position):
        front_to_beacon = Localization._angle(Localization.FRONT_VECTOR, led_vector)
        beacon_to_zero = Localization._angle(Localization.LED_POS[:, index] - position, [100, 0])
        return -(front_to_beacon + beacon_to_zero)
        #tobeacon = Localization._angle(Localization.FRONT_VECTOR, led_vector)
        #beacon_angle = np.arctan2(position[1] - Localization.LED_POS[1,index], position[0] - Localization.LED_POS[0,index])
        #return np.pi + beacon_angle - tobeacon

    def _localize(self, img):
        # Find the leds by color filtering
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        led_positions, available = self._get_leds(img_hsv)        
        if np.sum(available) < 3:
            print(f"[Localization] No enough beacon for localization, unavailable beacon ids: {np.where(available == False)[0]}")
            return None

        # the center of the image
        center_frame = np.array([[self.X_center_frame],[self.Y_center_frame]])
        
        # calculate the position
        indices = np.where(available == True)[0] # Choose 3 beacons
        led_vectors = led_positions - center_frame
        position = self._beacon_localize(led_vectors, indices)
        if position is None:
            return None
            
        # calculate the orientation
        oris = np.zeros((len(indices),1))
        for i in range(len(indices)):
            oris[i] = self._orientation_by_beacon(indices[i], led_vectors[:, indices[i]], position) % (2*np.pi)
        ori = np.median(oris)
        # print(oris)

        # camera position to center position
        # TODO

        return position, ori
    
    def get_pose(self): # rename it because I want to use 'state' for more general things
        # TODO: should we use odometry for faster speed?
        img_original = self._frame()
        if img_original is None: # error
            return None        
        
        return self._localize(img_original)

if __name__ == "__main__":
    localizer = Localization(camera_id=1)
    # img = cv2.imread("Localisation90.png")
    localizer.start()
    while True:
        img = localizer._frame()
        cv2.imshow("frame", img)
        cv2.waitKey(0)
        starter = time.time()
        state = localizer._localize(img)
        ender = time.time()
        print(f'it takes {ender-starter}s.')
        if state is None:
            continue
        print(f'pos ({state[0][0]:.1f},{state[0][1]:.1f}), ori: {state[1]/np.pi*180:.1f}')
        try:
            localizer.calibrate(img_original=img)
            cv2.waitKey(0)
        except:
            print("No result.")
            time.sleep(5)
