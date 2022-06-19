#ifndef _SERVOS_H
#define _SERVOS_H
#include "utils.h"

#define FINGER_RELEASE 800
#define FINGER_GRASP 1900

#define HAND_DOWN 2200
#define HAND_NEU 1500
#define HAND_BACK 800

#define ARM_LEFT_DOWN 2060
#define ARM_LEFT_NEU 1300
#define ARM_LEFT_BACK 1080

#define ARM_RIGHT_DOWN 800
#define ARM_RIGHT_NEU 1900
#define ARM_RIGHT_BACK 2240

#define GATE_OPEN 1250
#define GATE_CLOSE 1450 //1300

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

class Gripper{
  Adafruit_PWMServoDriver pwm_driver = Adafruit_PWMServoDriver();

  unsigned int positions[5] = {FINGER_RELEASE, 
    HAND_BACK, ARM_LEFT_BACK, ARM_RIGHT_BACK, GATE_CLOSE};
  
  void move_to(int channel, int value);
  void move_in(int channel, int value);
  
  void move_one(int channel, int value);
  void move_together(int v_hand, int v_left, int v_right, int steps = 20, int waittime = 2000);

public:
  Gripper(){    }
  void init();  
  void _resume();
  void rest();
  void collect();
  void open_gate();
  void close_gate();
  void _go_neutral();
  void _grasp();
  void _release();
  void _arm_back();
  void _arm_down();
  void _arm_neu();
};

#endif
// result:
// Tower Pro
// 550 - 2400 -- slightly smaller than 180 degree
// Parallax (without label)
// 555 - 2250 -- slightly smaller than 180 degree
// Parallax (#1)
// 550 - 2380 -- slightly bigger than 180 degree
