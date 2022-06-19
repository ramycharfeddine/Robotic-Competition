#include "servos.h"

  
void Gripper::move_to(int channel, int value){
  pwm_driver.writeMicroseconds(channel, value);
  positions[channel] = value;
}
void Gripper::move_in(int channel, int value){
  int pos = positions[channel];
  move_to(channel, pos + value);
}
void Gripper::move_one(int channel, int value){
  int pos = positions[channel];
  int sign = (value > pos)? 1 : -1;
  for(int i = 0; i < abs(value - pos); i += 1){
    pwm_driver.writeMicroseconds(channel, pos + sign*i);
    delay(1); 
  }
  positions[channel] = value;
}  
void Gripper::move_together(int v_hand, int v_left, int v_right, int steps, int waittime){
  // do them with 10(+1) steps
  // total spent time = 2000
  int pos_hand = positions[SERVO_CHANNEL_HAND];
  int pos_arm_left = positions[SERVO_CHANNEL_ARM_LEFT];
  int pos_arm_right = positions[SERVO_CHANNEL_ARM_RIGHT];
  int step_h = (v_hand - pos_hand)/steps;
  int step_l = (v_left - pos_arm_left)/steps;
  int step_r = (v_right - pos_arm_right)/steps;
  while(true){
    bool finished = true;
    if(abs(pos_hand - v_hand) > abs(step_h)) {
      move_in(SERVO_CHANNEL_HAND, step_h);
      finished = false;
    }
    if(abs(pos_arm_left - v_left) > abs(step_l)) {
      move_in(SERVO_CHANNEL_ARM_LEFT, step_l);
      finished = false;
    }
    if(abs(pos_arm_right - v_right) > abs(step_r)){
      move_in(SERVO_CHANNEL_ARM_RIGHT, step_r);
      finished = false;
    }
    pos_hand += step_h;
    pos_arm_left += step_l;
    pos_arm_right += step_r;
    if(finished) break;
    delay(waittime/steps);
  }
  
  move_to(SERVO_CHANNEL_HAND,  v_hand);
  move_to(SERVO_CHANNEL_ARM_LEFT,  v_left);
  move_to(SERVO_CHANNEL_ARM_RIGHT,  v_right);
  positions[SERVO_CHANNEL_HAND] = pos_hand;
  positions[SERVO_CHANNEL_ARM_LEFT] = pos_arm_left;
  positions[SERVO_CHANNEL_ARM_RIGHT] = pos_arm_right;
}

void Gripper::init(){
  
  pwm_driver.begin();
  pwm_driver.setOscillatorFrequency(27000000); // todo
  pwm_driver.setPWMFreq(50);  // Analog servos run at ~50 Hz updates
    
  _resume();
}

void Gripper::_resume(){
  close_gate();
  _arm_back();
  _release();
}
void Gripper::rest(){    
  close_gate();
  _arm_back();
  _release();
}
void Gripper::collect(){
  _go_neutral();
  delay(100);
  _arm_down();
  delay(200);
  _grasp();
  delay(500);
  _arm_back();
  _release();
  delay(200);
}
void Gripper::_go_neutral(){
  _arm_neu();
  _release();
}
void Gripper::_grasp(){
  move_one(SERVO_CHANNEL_FINGER, FINGER_GRASP);
}
void Gripper::_release(){
  move_one(SERVO_CHANNEL_FINGER, FINGER_RELEASE);
}
void Gripper::_arm_back(){
  move_together(HAND_BACK, ARM_LEFT_BACK, ARM_RIGHT_BACK);
}
void Gripper::_arm_down(){
  move_together(HAND_DOWN, ARM_LEFT_DOWN, ARM_RIGHT_DOWN);
} 
void Gripper::_arm_neu(){
  move_together(HAND_NEU, ARM_LEFT_NEU, ARM_RIGHT_NEU);
}
void Gripper::open_gate(){
  move_one(SERVO_CHANNEL_GATE, GATE_OPEN);
}
void Gripper::close_gate(){
  move_one(SERVO_CHANNEL_GATE, GATE_CLOSE);
}
