#ifndef _MOTION_H
#define _MOTION_H

#define WHEEL_PERIMETER (12*3.14159)
#define MAX_MAXON_RPM (2060)
#define MAXON_REDUCTION (60)
#define DISTANCE_MODE_SPEED (50)
#define DISTANCE_MODE_TIMEOUT (10000)
#define SPEED_MODE_TIMEOUT (1000)

#define MOTOR_LOW_PWM_BOUND ((byte)(0.1*255))
#define MOTOR_HIGH_PWM_BOUND ((byte)(0.9*255))
#define MOTOR_SPEED_MEASURE_MAX_RPM 10000
#define MOTOR_SPEED_MEASURE_MAX_VOLTAGE 4.0

#include <Arduino.h>
#include "utils.h"

#include <TimerThree.h>

class Motor{
    const int pin_pwm;
    const int pin_en;
    const int pin_dir;      //HIGH - FORWARD
    const int pin_cur_spd_front;  // 0-10000rpm -> 0-2-4V
    const int pin_cur_spd_hind;
//    const int pin_avg_spd;


    /*float value2speed(byte value){
        // byte -> cm/s
        return value/1024.0*MAX_MAXON_RPM/MAXON_REDUCTION/60.0*WHEEL_PERIMETER;
    }
    byte speed2value(float speed){
        if(speed < 0) return 0;
        int value = (int)(speed * 60.0 / WHEEL_PERIMETER / (MAX_MAXON_RPM/MAXON_REDUCTION) * 255);
        if(value > 255) value = 255;
        return value;
    }*/

    inline byte truncat(byte value){
        return map(value, 0, 255, MOTOR_LOW_PWM_BOUND, MOTOR_HIGH_PWM_BOUND);
    }

public:
    bool isStill = true;
    bool direction = true;
    
    //Motor(){}
    //~Motor(){}
    inline Motor(const int pwm, const int en, const int dir,
            const int curspdf, const int curspdh):
            pin_pwm(pwm),pin_en(en), pin_dir(dir),
            pin_cur_spd_front(curspdf), pin_cur_spd_hind(curspdh){

            }
    
    void init();

    void set_speed(byte value, bool dir = true);
    void stop();
    float current_speed(byte filter_threshold = 5);
    /*
    float avg_speed(){
        int value = analogRead(pin_avg_spd);
        float rpm = speed_measure_max_rpm/2.0 * (value/1024.0*5.0 - speed_measure_max_voltage/2.0);
        return rpm / MAXON_REDUCTION * WHEEL_PERIMETER / 60;
    }*/
};

enum DIRECTION {FORWARD = 0b11, BACKWARD = 0b00, LEFT=0b01, RIGHT=0b10};
#define SIGN_DIRECTION(d) (2*d-1)

struct Motion_Command{
  bool valid = false;
  bool speed_control = true; // or false -- distance control
  DIRECTION dir = FORWARD;
  byte speed = 0;
  double distance = 0; // in cm (for turning, please pre-transform the angle into distance)
  double disp_l = 0, disp_r = 0;
  unsigned long timer_start = 0;
  unsigned long timeout = 0;

  inline bool isTimeOut(){
    return millis() - timer_start > timeout;
    //if(ret) valid = false;
    //return ret;
  }
};

class Motion
{
private:
    Motor motor_left;
    Motor motor_right;
    
    double *p_cnt_left, *p_cnt_right;
    void (*p_int_func)();
    unsigned long period = 10000; // 10ms
    double displacement_left_last = 0;
    double displacement_right_last = 0;

    Motion_Command cmd;
    const double torlerance = 0.5;
    const byte distance_mode_speed = DISTANCE_MODE_SPEED;
    
    void start_speed_monitor();

    // maybe you should never call this
    void end_speed_monitor();
    
public:
  Motion(const int pwm_l, const int en_l, const int dir_l,
          const int curspd_l, const int avgspd_l,
          const int pwm_r, const int en_r, const int dir_r,
          const int curspd_r, const int avgspd_r,
          double* cnt_left, double* cnt_right):
          motor_left(pwm_l, en_l, dir_l, curspd_l, avgspd_l),
          motor_right(pwm_r, en_r, dir_r, curspd_r, avgspd_r),
          p_cnt_left(cnt_left), p_cnt_right(cnt_right)
          {};

  void init(void (*itrpt)(), unsigned long _period = 10000);
  inline void move(byte value, DIRECTION dir){
      motor_left.set_speed(value, dir >> 1);
      motor_right.set_speed(value, dir & 1);
  }
    

  void set_cmd_speed(byte value, DIRECTION dir, unsigned long timeout = SPEED_MODE_TIMEOUT);
  
  // directly control the motors speed
  // do not stop old cmd
  void set_cmd_speed2(int left, int right, unsigned long timeout = 1000);
  void set_cmd_dis(byte dis, DIRECTION dir, unsigned long timeout = 0);
  
  // run this function as much as possible...
  // return status of the execution
  bool sync();

  void stop();

  void get_cur_speed(double* left, double* right);
  //void get_avg_speed(double* left, double* right);

  void get_displacement(double* left, double* right);
  
  // get the displacement difference from last time call this function
  void get_delta_displacement(double* left, double* right);
};


#endif

/*
PWM AND TIMERS
https://playground.arduino.cc/Main/TimerPWMCheatsheet/
timer 0 (controls pin 13, 4)
timer 1 (controls pin 12, 11)
timer 2 (controls pin 10, 9)
timer 3 (controls pin 5, 3, 2)
timer 4 (controls pin 8, 7, 6)

Arduino Pin   Register
2 OCR3B
3 OCR3C
4 OCR0C
5 OCR3A
6 OCR4A
7 OCR4B
8 OCR4C
9 OCR2B
10  OCR2A
11  OCR1A
12  OCR1B
13  OCR0A
44  OCR5C
45  OCR5B
46  OCR5A

*/
