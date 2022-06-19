#ifndef PROTOCOL_H
#define PROTOCOL_H
/* All the commands and reports

RPi -> Arduino
* locomotion command
    * go forward  -- control mode
        * SPEED CONTROL 
          * (e.g. normal speed -- for exploration/ fast speed -- it's clear in front / slow speed -- carefully)
          * default timeout 1s ?
        * DISTANCE CONTROL 
    * turning
    * backward
* Action Command
    * Pick up the bottle (Arm down, gripper close, arm up, gripper open)
    * Empty the storage (open the door *and shake)
* Ask Question
    * *Bottle confirmation

Arduino -> RPi
* Routine
    * Displacement
    * Obstacles distances
    * Warning, if there is
* On request
    * *Bottle Detection

*/
/*
Protocol
!!!!!0x00 as seperator!!!!!

Rpi -> Arduino
Header [Data]
* Header
    * 0-1: length of the data (0-3)
    * 2-4: Command type
        * 0: Motion
        * 1: Servo Motion
        * 2: Adavanced functions
        * 3: Request info
        * 4: Direct Motion Control
    * 5-7: 
        * For motion
            * 5: Mode
                * 0: Speed Mode
                * 1: Distance Mode
            * 6-7: 
                * 0: Forward
                * 1: Left
                * 2: Right
                * 3: Backward
        * For request
            * 0: Orientation
            * 1: Declination
        * For Servos
            * 0: Pick up
            * 1: Empty the storage
        * Advanced Functions
            * 0: Local avoidance left
            * 1: Local avoidance right
            * //1: Disable obstacle detection
            * //2: Enable obstacle detection
* Data
    * for Motion
        * Data[0]
            * Speed(1byte) 0-255 will be mapped to 0-145
            * Distance(1byte) in cm
        * Data[1]
            * timeout (unit: 100ms)
    * for Direct Motion
        * Data[0]: Speed Left
        * Data[1]: Speed Right

Arduino -> Rpi
Header [Data]
Header - Length - type - addedinfo
* 0-1: length of the data (0-3)
* 2-4: Report type
    * 0: Warning! We stopped because of obstacle detected
    * 1: Displacement - O
    * 2: Obstacles Front - O
    * 3: Obstacles Back
    * 4: task done 
    * ~~6: Answer the request
* 5-7: For Obstacle info(also for warning)
    * the index of the sensor having minimum distance, from left to right
* 5-7: Task done:
    * pick up: 0
    * empty: 1
    * local avoidance: 2
* 5-7: For Answer
    * 0: Orientation
    * 1: Declination
Data
//* Obstacle(2/3byte): distance in cm (bigger than 255cm will not be reported)
* Obstacle (1 byte): the minimum distance in cm
* Displacement(2bytes): -127~127 map to 1~255
*/
#define MSG_SEPARATOR 0x00


#define ROUND_2_INT(f) ((int)(f >= 0.0 ? (f + 0.5) : (f - 0.5)))
#define SIGNED_BYTE(data) ((byte)(ROUND_2_INT(data)) + 128)

#define MSG_TYPE_RPI_MOTION 0
#define MSG_TYPE_RPI_SERVO 1
#define MSG_TYPE_RPI_ADAFUN 2
#define MSG_TYPE_RPI_REQUEST 3
#define MSG_TYPE_RPI_DIRECT_MOTION 4

#define MSG_TYPE_RPI_MODE_SPEED 0
#define MSG_TYPE_RPI_MODE_DISTANCE 1
#define MSG_TYPE_RPI_DIR_FORWARD 3
#define MSG_TYPE_RPI_DIR_LEFT 1
#define MSG_TYPE_RPI_DIR_RIGHT 2
#define MSG_TYPE_RPI_DIR_BACKWARD 0

#define MSG_TYPE_SERVO_PICKUP 0
#define MSG_TYPE_SERVO_EMPTY 1
#define MSG_TYPE_ADAFUN_LOCAL_AVOID_LEFT 0
#define MSG_TYPE_ADAFUN_LOCAL_AVOID_RIGHT 1

#define MSG_TYPE_REQUEST_ORIENTATION 0
#define MSG_TYPE_REQUEST_DECLINATION 1

#define MSG_RPI_DATA_LENGTH(header) (header >> 6)
#define MSG_RPI_CMD_TYPE(header) ((header & 0b00111000) >> 3)
#define MSG_RPI_MOTION_MODE(header) ((header & 0b00000100) >> 2)
#define MSG_RPI_MOTION_DIR(header) (header & 0b00000011)
#define MSG_RPI_SERVO_TYPE(header) (header & 0b00000111)
#define MSG_RPI_ADAFUN_TYPE(header) (header & 0b00000111)

// Arduino -> RPI
#define MSG_TYPE_ARD_WARNING 0
#define MSG_TYPE_ARD_DISPLACEMENT 1
#define MSG_TYPE_ARD_OBSTACLES_FRONT 2
#define MSG_TYPE_ARD_OBSTACLES_BACK 3
#define MSG_TYPE_ARD_TASK_DONE 4
#define MSG_TYPE_ARD_ANSWER 6

#define MSG_ARD_DONE_PICKUP 0
#define MSG_ARD_DONE_EMPTY 1
#define MSG_ARD_DONE_LOCALAVOID 2

#define MSG_ARD_REPORT_WARN(state, data1) {(1 << 6) | (MSG_TYPE_ARD_WARNING << 3) | state, data1}
#define MSG_ARD_REPORT_OBS_FRONT(state, data1) {(1 << 6) | (MSG_TYPE_ARD_OBSTACLES_FRONT << 3) | state, data1}
//#define MSG_ARD_REPORT_OBS_SIDE(data1, data2) {(2 << 6) | (MSG_TYPE_ARD_OBSTACLES << 3) | MSG_TYPE_ARD_OBS_SIDE, data1, data2}
#define MSG_ARD_REPORT_OBS_BACK(state, data1, data2) {(3 << 6) | (MSG_TYPE_ARD_OBSTACLES_BACK << 3) | state, data1, data2}

//#define MSG_ARD_REPORT_WARN_FRONT(data1, data2) {(2 << 6) | (MSG_TYPE_ARD_WARNING << 3) | MSG_TYPE_ARD_OBS_FRONT, data1, data2}
//#define MSG_ARD_REPORT_WARN_SIDE(data1, data2) {(2 << 6) | (MSG_TYPE_ARD_WARNING << 3) | MSG_TYPE_ARD_OBS_SIDE, data1, data2}
//#define MSG_ARD_REPORT_WARN_BACK(data) {(1 << 6) | (MSG_TYPE_ARD_WARNING << 3) | MSG_TYPE_ARD_OBS_BACK, data1}

#define MSG_ARD_REPORT_DISPLACEMENT(data1, data2) {(2 << 6) | (MSG_TYPE_ARD_DISPLACEMENT << 3), data1, data2}

#define MSG_ARD_REPORT_DONE_PICKUP (0 << 6) | (MSG_TYPE_ARD_TASK_DONE << 3) | MSG_ARD_DONE_PICKUP
#define MSG_ARD_REPORT_DONE_EMPTY (0 << 6) | (MSG_TYPE_ARD_TASK_DONE << 3) | MSG_ARD_DONE_EMPTY
#define MSG_ARD_REPORT_DONE_LOCALAVOID (0 << 6) | (MSG_TYPE_ARD_TASK_DONE << 3) | MSG_ARD_DONE_LOCALAVOID

//#define MSG_ARD_REPORT_ORIENTATION(data_low, data_high) {(2 << 6) | (MSG_TYPE_ARD_ANSWER << 3) | MSG_TYPE_REQUEST_ORIENTATION, data_low, data_high}
//#define MSG_ARD_REPORT_DECLINATION(data_low, data_high) {(2 << 6) | (MSG_TYPE_ARD_ANSWER << 3) | MSG_TYPE_REQUEST_DECLINATION, data_low, data_high}

#endif
