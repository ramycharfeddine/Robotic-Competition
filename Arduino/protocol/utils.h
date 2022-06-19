#ifndef _WALLE_UTILS_H
#define _WALLE_UTILS_H

// ----------- Parameters ---------------

// Obstacale Avoidance
#define OBS_AVOID_THRESHOLD 30
#define OBS_STOP_THRESHOLD 20 //76.5cm/s*5*5ms ~= 2cm 
#define OBS_TURN_SPEED 30 //
#define OBS_TURN_TIME 250 //ms 0.2s*20*0.5cm/s = 2cm ~ 4degree
#define OBS_AVOID_FORWARD_DISTANCE 10 //cm go forward after turning
#define OBS_AVOID_TURN_DISTANCE 4 // ~6degree
#define OBS_REPORT_THRESHOLD 80 // cm
#define OBS_REPORT_MIN_PERIOD 1000 //ms

// ----------- PIN DEFINITATION ---------
// PWM for Mega: 2-13, 44-46
#define PIN_MOTOR_L_PWM 7
#define PIN_MOTOR_L_EN 50
#define PIN_MOTOR_L_DIR 52
#define PIN_MOTOR_LF_CURSPD A13
#define PIN_MOTOR_LH_CURSPD A12
//#define PIN_MOTOR_L_AVGSPD A14
#define PIN_MOTOR_R_PWM 6
#define PIN_MOTOR_R_EN 51
#define PIN_MOTOR_R_DIR 53
#define PIN_MOTOR_RF_CURSPD A11
#define PIN_MOTOR_RH_CURSPD A10
//#define PIN_MOTOR_R_AVGSPD A12

// Servo Channels
#define SERVO_CHANNEL_FINGER 0
#define SERVO_CHANNEL_HAND 1
#define SERVO_CHANNEL_ARM_LEFT 2
#define SERVO_CHANNEL_ARM_RIGHT 3
#define SERVO_CHANNEL_GATE 4

// Range Sensors
#define PIN_IR_1 A0
#define PIN_IR_2 A1
#define PIN_IR_3 A2
#define PIN_IR_4 A3
#define PIN_IR_5 A4


#endif
