#!/usr/bin/env python3
"""
partialy refer to [Kai's work](https://gitlab.epfl.ch/create-lab/lab-systems/arduino_python_communication/-/blob/main/comms_wrapper.py)
with our own protocol
"""
from base64 import decode
import serial
import time
from threading import Thread
import copy
import numpy as np

# odometry test
#from localization import *

class SerialComm:
    def __init__(self, portName, baudrate = 115200, seperator = b'\x00') -> None:
        """
        Open a Serial port and write, listen to it
        """
        self.portName = portName
        self.baudrate = baudrate

        self.ser = None
        self.connected = False

        self.seperator = seperator
        self.receivedMessages = [] # the latest is appended to the end
        self.newMsgReceived = False

        self.__thread_read = None
        self.__reading = False

        self.lock = False

    def _serial_read(self):
        buffer = bytearray()
        while 1:
            try:
                b = self.ser.read()
                #print(b)
                if b == self.seperator:
                    while self.lock:
                        time.sleep(1)
                    self.lock = True
                    self.receivedMessages.append(copy.deepcopy(buffer))
                    self.newMsgReceived = True
                    self.lock = False
                    buffer.clear()
                else:
                    buffer.extend(b)
                if not self.__reading:
                    break
            except Exception as e:
                print(e)

    def read_msg(self):
        """
        read the message from arduino, list of bytearray; 
        note: you need to decode them
        """
        if self.newMsgReceived:
            while self.lock:
                time.sleep(1)
            self.lock = True
            cp = copy.deepcopy(self.receivedMessages)
            self.newMsgReceived = False
            self.receivedMessages = []
            self.lock = False
            return cp
        else:
            return None

    # msg: byte array
    def _serial_write(self, msg: bytes):
        if self.ser is not None:
            self.ser.write(msg)
        else:
            raise Exception("Please connect to the serial first!")
    
    def start(self):
        # 1. Connect the Serial
        try: 
            self.ser = serial.Serial(
                port=self.portName, 
                baudrate=self.baudrate,
                )
            self.connected = True
            # # toggle dtr to reset the arduino
            # self.ser.dtr = True
            # time.sleep(1)
            # self.ser.dtr = False
        
            print("Successfully connected to Arduino.")
        except Exception as e:
            print(e)
            print("!! Cannot connect to Arduino!!")
            self.connected = False
            return False
        # 2. Start Reading Thread
        self.__reading = True
        self.__thread_read = Thread(target=self._serial_read)
        self.__thread_read.daemon = True
        self.__thread_read.start()

    def end(self):
        self.__reading = False # close background thread
        self.connected = False
        self.receivedMessages = [] 
        self.newMsgReceived = False
        self.ser.close()

    def send_message(self, msg: bytearray):
        print("Sending Messega:" + str(list(msg)))
        self._serial_write(self.seperator + msg)

class Protocol:
    """
    Protocols for communication with Arduino.
    """
    # Default settings
    CMD_MOTION_SPEED_TIMEOUT = 10 #*100ms
    CMD_MOTION_DISTANCE_TIMEOUT = 100 #*100ms

    # Commands
    CMD_MOTION = 0
    CMD_SERVO = 1
    CMD_ADVFUN = 2
    CMD_REQUEST = 3

    MOTION_MODE_SPEED = 0
    MOTION_MODE_DISTANCE = 1

    FORWARD = 3
    LEFT = 1
    RIGHT = 2
    BACKWARD = 0

    SERVO_PICKUP = 0
    SERVO_EMPTY = 1

    ADVFUN_LOCAL_AVOID = 0

    REQUEST_ORIENTATION = 0
    REQUEST_DECLINATION = 1

    # report
    RPT_WARN = 0
    RPT_DISPLACEMENT = 1
    RPT_OBSTACLES_FRONT = 2
    RPT_OBSTACLES_BACK = 3
    RPT_DONE_TASK = 4
    RPT_ANSWER = 6

    OBS_FRONT2 = 0
    OBS_SIDE = 1
    OBS_BACK = 2

    DONE_PICKUP = 0
    DONE_EMPTY = 1
    DONE_AVOIDANCE = 2

    ANS_ORIENTATION = 0
    ANS_DECLINATION = 1

    @staticmethod
    def decode(msg: bytearray):
        """return report_type, added_info, <data1>, ..."""
        assert len(msg) > 0
        header = msg[0] #int.from_bytes(msg[0], 'little')
        length = (header & 0b11000000) >> 6
        assert length == len(msg) - 1, f"msg length incorrect, len: {len(msg)}, decoded: {length}"
        report_type = (header & 0b00111000) >> 3
        added_info = header & 0b00000111
        return [report_type, added_info] + list(msg[1:])

    @staticmethod
    def encode(cmd, added_info1 = None, added_info2 = None, data = None):
        if cmd == Protocol.CMD_MOTION:
            assert added_info1 is not None # Mode
            assert added_info2 is not None # Direction
            assert data[0] <= 255 and data[0] >= 0  # value
            assert data[1] <= 255 and data[1] >= 0  # timeout
            length = 2
            header = (length << 6) | (cmd << 3) | (added_info1 << 2) | added_info2
            return bytearray([header] + data)
        elif cmd == Protocol.CMD_SERVO:
            assert (added_info1 == Protocol.SERVO_PICKUP) or (added_info1 == Protocol.SERVO_EMPTY)
            header = (0 << 6) | (cmd << 3) | added_info1
            return bytearray(header)
        elif cmd == Protocol.CMD_ADVFUN:
            assert added_info1 == Protocol.ADVFUN_LOCAL_AVOID
            header = (0 << 6) | (cmd << 3) | added_info1
            return bytearray(header)
        elif cmd == Protocol.CMD_REQUEST:
            assert added_info1 is not None # Content
            header = (0 << 6) | (cmd << 3) | added_info1
            return bytearray(header)
        else:
            raise Exception(f"Unknow Command {cmd}")
        
    @staticmethod
    def signedbyte(data):
        return (data-128)/10.0


class Arduino:
    """
    Arduino Controller via Serial port
    I'm trying to hide everything from high-level program,
    so just check the available function not start with _, which means they are somehow private function
    """

    def __init__(self, portName : str) -> None:
        self.sercom = SerialComm(portName=portName)

        self.TASK_UNDERGOING = False # pick bottle/ empty the storage/ local avoidance

    def whereami(self):
        pos, theta = self.locaizer.get()
        return f'{pos[0]:.2f}, {pos[1]:.2f}, {theta/np.pi*180:.0f} deg'

    def start(self):
        #self.locaizer = Localization()
        self.sercom.start()

    def end(self):
        self.sercom.end()

    def read(self):
        buffer = self.sercom.read_msg()
        if buffer is not None:
            # decode the messages
            for msg in buffer:
                try: 
                    # type, added info, datas, ...
                    decoded = Protocol.decode(msg)
                    msgtype = decoded[0]
                    addedinfo = decoded[1]
                    data = decoded[2:]
                    if msgtype == Protocol.RPT_WARN:
                        # less than 30cm in front, the robot has stopped
                        warn_left = (addedinfo & 0b100)  >> 2
                        warn_middle = (addedinfo & 0b10) >> 1
                        warn_right = addedinfo & 0b1
                        # some obs are too close
                        # how to deal with it?
                        # TODO: identify if it's obstacle or the wall.

                        print(f"[Warning] Obstacle in front too close! {data}")
                        
                    elif msgtype == Protocol.RPT_OBSTACLES_FRONT:
                        # addedinfo indicates if there's obstacle in 80cm(for avoidance)
                        warn_left = (addedinfo & 0b100)  >> 2
                        warn_middle = (addedinfo & 0b10) >> 1
                        warn_right = addedinfo & 0b1
                        # TODO
                        print(f'[Obstacle] Front: {data[0], data[1], data[2]}')
                    elif msgtype == Protocol.RPT_OBSTACLES_BACK:
                        # not implemented yet
                        warn_left = (addedinfo & 0b100)  >> 2
                        warn_right = addedinfo & 0b1
                        # TODO
                        print(f'[Obstacle]  Back: {data[0], data[1]}')
                    elif msgtype == Protocol.RPT_DISPLACEMENT:
                        dl = Protocol.signedbyte(data[0])
                        dr = Protocol.signedbyte(data[1])
                        #self.locaizer.update_displacement(dl, dr)
                        print(f"Displacement:{dl, dr}")
                    elif msgtype == Protocol.RPT_DONE_TASK:
                        if addedinfo == Protocol.DONE_PICKUP:
                            # todo
                            print("[Done] Bottle collected")
                        elif addedinfo == Protocol.DONE_EMPTY:
                            # todo
                            print("[Done] Storage emptyed")
                        elif addedinfo == Protocol.DONE_AVOIDANCE:
                            # todo
                            print("[Done] Obstacle free")
                        else: raise Exception("Unknown task type.")
                        self.TASK_UNDERGOING = False #TODO: a timeout may needed

                    elif msgtype == Protocol.RPT_ANSWER:
                        # TODO
                        print("anwser")
                    else: raise Exception("Unknown report type.")
                except:
                    # fail to decode - it may be debug info.
                    try:
                        print("[debug]", msg.decode('utf-8'))
                    except:
                        print("[debug]", msg)

    def _move_at_speed(self, dir, speed, timeout = None):
        if timeout is None:
            timeout = Protocol.CMD_MOTION_SPEED_TIMEOUT
        self.sercom.send_message(Protocol.encode(
                Protocol.CMD_MOTION, Protocol.MOTION_MODE_SPEED, dir, [int(speed), timeout]))

    def _move_to_distance(self, dir, distance, timeout = None):
        if timeout is None:
            timeout = Protocol.CMD_MOTION_DISTANCE_TIMEOUT
        self.sercom.send_message(Protocol.encode(
                Protocol.CMD_MOTION, Protocol.MOTION_MODE_DISTANCE, dir, [int(distance), timeout]))

    """
    def go_forward(self, speed = None, distance = None, timeout = None):
        if speed is None and distance is None:
            raise Exception("Please assign at least one")
        if speed is not None:
            self._move_at_speed(Protocol.FORWARD, speed, timeout)
            if distance is not None:
                print("Only speed are applied")
        elif distance is not None:
            self._move_to_distance(Protocol.FORWARD, distance, timeout)

    def go_backward(self, speed = None, distance = None, timeout = None):        
        if speed is None and distance is None:
            raise Exception("Please assign at least one")
        if speed is not None:
            self._move_at_speed(Protocol.BACKWARD, speed, timeout)
            if distance is not None:
                print("Only speed are applied")
        elif distance is not None:
            self._move_to_distance(Protocol.BACKWARD, distance, timeout)

    def turn_left(self, speed = None, distance = None, timeout = None):        
        if speed is None and distance is None:
            raise Exception("Please assign at least one")
        if speed is not None:
            self._move_at_speed(Protocol.LEFT, speed, timeout)
            if distance is not None:
                print("Only speed are applied")
        elif distance is not None:
            self._move_to_distance(Protocol.LEFT, distance, timeout)

    def turn_right(self, speed = None, distance = None, timeout = None):        
        if speed is None and distance is None:
            raise Exception("Please assign at least one")
        if speed is not None:
            if timeout is None:
                timeout = 10
            self._move_at_speed(Protocol.RIGHT, speed, timeout)
            if distance is not None:
                print("Only speed are applied")
        elif distance is not None:
            self._move_to_distance(Protocol.RIGHT, distance, timeout)
    """

    def move(self, dir, speed = None, distance = None, timeout = None):
        """
        dir: Protocol.[LEFT/RIGHT/FORWARD/BACKWARD]
        speed: 1 ~ 0.3cm/s -- 255 ~ 76.5cm/s
        distance: cm
        """
        assert dir >= 0 and dir <=3
        if speed is None and distance is None:
            raise Exception("Please assign at least one")
        if speed is not None:
            self._move_at_speed(dir, speed, timeout)
            if distance is not None:
                print("[Warning] Only speed is applied")
        elif distance is not None:
            self._move_to_distance(dir, distance, timeout)

    def pick_up(self):
        self.TASK_UNDERGOING = True
        self.sercom.send_message(Protocol.encode(Protocol.CMD_SERVO, Protocol.SERVO_PICKUP))
    
    def empty_storage(self):
        self.TASK_UNDERGOING = True
        self.sercom.send_message(Protocol.encode(Protocol.CMD_SERVO, Protocol.SERVO_EMPTY))
    
    def do_local_avoidance(self):
        self.TASK_UNDERGOING = True
        self.sercom.send_message(Protocol.encode(Protocol.CMD_ADVFUN, Protocol.ADVFUN_LOCAL_AVOID))

    def stop(self):
        self.move(0, 0)


if __name__ == '__main__':
    # On raspberry pi, you can use command
    # ls /dev/ttyACM*
    # to find the available port name
    # !ls /dev/ttyACM* # for jupyter notebook
    # normally it will be "/dev/ttyACM0"

    # create an instance of Arduino Controller specifying the port name.
    ard = Arduino(portName = "COM4")
    ard.start()

    try:
        time.sleep(1)
        # read msg from arduino
        # please add code inside 'read' function to deal with these info
        ard.read()
        
        #print('[whereami]',ard.whereami())
        ard.move(Protocol.FORWARD,distance=20, timeout=50)
        for i in range(3):
            time.sleep(1) # wait for response from arduino
            ard.read()
        #print('[whereami]',ard.whereami())
        
        ard.move(Protocol.BACKWARD, distance=10, timeout=50)
        #print('[whereami]',ard.whereami())
        for i in range(3):
            time.sleep(1) # wait for response from arduino
            ard.read()
        # stop the robot. it will stop motion immediately (ofc, after receving this message)
        ard.stop()
            
        # turn left in speed of 100
        # real speed can be calculated as 100/255*2060(min^-1)/60*12cm*pi
        # rpm = 2060, reduction of gearbox: 60, diameter of the wheel: 12cm
        # max speed corresponds to 255(the highest value for 1 byte)
        ard.move(dir=Protocol.LEFT, speed=50, timeout=10) 
        # the coding of the direction is actually the direction of the motor --
        # the higher digit for left, the lower one for right, so
        #           left    right  | value
        # Forward   1       1      | 3
        # Backward  0       0      | 0 
        # Left      0       1      | 1
        # Right     1       0      | 2 
        # this might be easier for your code
        #print('[whereami]',ard.whereami())
        for i in range(3):
            time.sleep(1) # wait for response from arduino
            ard.read()
        #print('[whereami]',ard.whereami())
    finally:
        ard.end() # close the serial communitaion


