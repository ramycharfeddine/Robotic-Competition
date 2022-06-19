#include <Arduino.h>

//#define DEBUG
#include "protocol.h"
#include "motion.h" // Note: You must connect the ground to make the speed monitor work!
#include "servos.h" // Please put the gripper to the rest pose.
#include "obstacle.h"

#define DISPLACEMENT_REPORT_PERIOD 50 //ms; 10 will be a good idea?

byte robot_state = 0;
// ------ Motion -----
// Speed monitor
const unsigned long GB_period = 10000; //us - motor speed sampling period
double GB_displacement_left = 0;
double GB_displacement_right = 0;
Motion motion = Motion(
                  PIN_MOTOR_L_PWM, PIN_MOTOR_L_EN, PIN_MOTOR_L_DIR, PIN_MOTOR_LF_CURSPD, PIN_MOTOR_LH_CURSPD,
                  PIN_MOTOR_R_PWM, PIN_MOTOR_R_EN, PIN_MOTOR_R_DIR, PIN_MOTOR_RF_CURSPD, PIN_MOTOR_RH_CURSPD,
                  &GB_displacement_left, &GB_displacement_right
                );
unsigned long timer_displacement = 0;
void motion_sync() {
  motion.sync(); // check the motion command
  unsigned long cur_time = millis();
  if (cur_time - timer_displacement > DISPLACEMENT_REPORT_PERIOD) {
#ifdef DEBUG
    display_displacement();
    Serial.write(0x00);
#endif
    send_displacement(); // report displacement
    timer_displacement = cur_time;
  }
}
// ----- Servos -----
Gripper gripper;

// ----- Obstacles Detection -----
unsigned long timer_last_report_obs = 0;
Obstacle_detection obs;
void obstacle_check() {
  byte min_dis = 255, min_index = 0;
  byte status = obs.obstacle_check(&min_dis, &min_index);

  if(status == 0){
    // no obstacle detected
    return;
  }
  else if(status == 1){
    robot_state = 1;
    warn_obs(min_index, min_dis);
  }
  else if(status == 2){
    if (millis() > OBS_REPORT_MIN_PERIOD + timer_last_report_obs){
      report_obs(min_index, min_dis);
      timer_last_report_obs = millis();
    }    
  }
}

// ---- debug functions ----
#ifdef DEBUG
  void display_displacement() {
    double dis_l, dis_r;
    motion.get_displacement(&dis_l, &dis_r);
    Serial.print("Displacement: ");
    Serial.print(dis_l);
    Serial.print(", ");
    Serial.print(dis_r);
    Serial.println();
    //Serial.write(0x00);
  }
#endif

// 0 - Ready
// 1 - Stopped because of the obstacle in front
// 2 - local avoidancing
// 3 - Picking the bottle
// 4 - Empty the storage

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  motion.init(&update_displacement, GB_period);

  gripper.init();
  gripper.rest();
  delay(1000);
  gripper.open_gate();
  delay(1000);
  gripper.close_gate();
  
  obs.init(&motion);
  
  Serial.write(0x00); // clear buffer
  Serial.println("Hello from Arduino!");
  Serial.write(0x00);
#ifdef DEBUG
  display_displacement();
#endif
}

void empty_storage(){  
    motion.move(20, DIRECTION::BACKWARD); // 6cm/s
    delay(4000); // 24cm
    motion.stop();
    gripper.open_gate();
    for(int i = 0; i < 3; i++){
      motion.move(80, DIRECTION::FORWARD); // 24cm/s
      delay(1000);
      motion.stop();
      delay(500);
      motion.move(40, DIRECTION::BACKWARD); // 12cm/s
      delay(2000);
      motion.stop();
    }
    delay(1000);
    // maybe shake?
    gripper.close_gate();
}

void loop() {
  unsigned long starter = millis(), ender;

  motion_sync();
  obstacle_check();
  read_command();

  motion_sync();
  switch (robot_state)
  {
    case 0:
      /* ready */
      break;
    case 1:
      // obstacles!
      break;
    case 2:
      // local avoidance
      motion.stop();
      obs.local_avoidance();
      _avoidance_done();
      robot_state = 0;
      // TODO
      break;
    case 3:
      // pick up
      motion.stop();
      gripper.collect(); // it takes ~2s
      _pick_done();
      robot_state = 0;
      break;
    case 4:
      // empty the storage
      motion.stop();
      // TODO
      empty_storage();
      _empty_done();
      robot_state = 0;
      break;

    default:
      break;
  }
  /*
    ender = millis();
    Serial.print("Loop time: ");
    Serial.print(ender - starter);
    Serial.println(".");
    Serial.write(0x00);
  */
  //delay(20); // do something
  gripper.rest();
}

// Protocol - Read
void read_command() {
  byte len = Serial.available();
  if (len > 0) {
    byte header = Serial.read();
    while (header == 0x00) {
      // Seperator
      header = Serial.read();
    }
    byte data_length = MSG_RPI_DATA_LENGTH(header);
    byte data[data_length];
    for (int i = 0; i < data_length; i++) {
      int re = Serial.read();
      unsigned long cnt = millis();
      while (re == -1) { // -1 if unavaliable
        re = Serial.read();
        // wait for max. 10ms
        if (millis() - cnt > 10) {
          Serial.println("Error Reading: Data arrived not long enough");
          break;
        }
        return;
      }
      data[i] = re;
    }
    // decode the msg
    switch (MSG_RPI_CMD_TYPE(header))
    {
      case MSG_TYPE_RPI_MOTION:
        if ((data_length != 2) && (data_length != 1)) {
          Serial.print("Error Reading: Data length should be 1 or 2 for motion command");
          Serial.println(data_length);
          Serial.write(0x00);
          break;
        }
        if (data[0] == 0) {
#ifdef DEBUG
          Serial.println("Stop");
#endif
          motion.stop();
          break;
        }
        if (robot_state != 0) {
          Serial.print("Should not move! Current state is ");
          Serial.println(robot_state);
          Serial.write(0x00);
          //break;
        }
#ifdef DEBUG
        Serial.println("Move");
#endif
        if (MSG_RPI_MOTION_MODE(header) == MSG_TYPE_RPI_MODE_SPEED) {
#ifdef DEBUG
          Serial.print(" in speed ");
          Serial.print(data[0]);
          //Serial.print(" dir:");
          //Serial.print(static_cast<DIRECTION>(MSG_RPI_MOTION_DIR(header)));
          Serial.print(" timeout ");
          Serial.println(100 * (int)(data[1]));
#endif
          if (data_length < 2) motion.set_cmd_speed(data[0], static_cast<DIRECTION>(MSG_RPI_MOTION_DIR(header)));
          else motion.set_cmd_speed(data[0], static_cast<DIRECTION>(MSG_RPI_MOTION_DIR(header)), 100 * (int)(data[1]));
        }
        else if (MSG_RPI_MOTION_MODE(header) == MSG_TYPE_RPI_MODE_DISTANCE) {
#ifdef DEBUG
          Serial.print(" to distance ");
          Serial.print(data[0]);
          //Serial.print(" dir:");
          //Serial.println(static_cast<DIRECTION>(MSG_RPI_MOTION_DIR(header)));
          Serial.print(" timeout ");
          Serial.println(100 * (int)(data[1]));
#endif
          // using default timeout
          if (data_length < 2) motion.set_cmd_dis(data[0], static_cast<DIRECTION>(MSG_RPI_MOTION_DIR(header)));
          else motion.set_cmd_dis(data[0], static_cast<DIRECTION>(MSG_RPI_MOTION_DIR(header)), 100 * (int)(data[1]));
        }
        else {
          Serial.println("Error Type for motion mode");
        }
        break;
      case MSG_TYPE_RPI_DIRECT_MOTION:
        if(data_length == 2) motion.set_cmd_speed2(data[0], data[1]);
        else if (data_length == 3) motion.set_cmd_speed2(data[0], data[1], data[2]*100);
        else{
          Serial.print("Error Reading: Data length should be 2 or 3 for direct motion command");
          Serial.println(data_length);
          Serial.write(0x00);
        }

        break;
      case MSG_TYPE_RPI_SERVO:
        // assert robot_state == 0
        if (MSG_RPI_SERVO_TYPE(header) == MSG_TYPE_SERVO_PICKUP) {
          Serial.println("Pickup");
          robot_state = 3;
        }
        else if (MSG_RPI_SERVO_TYPE(header) == MSG_TYPE_SERVO_EMPTY) {
          Serial.println("Empty the storage");
          robot_state = 4;
        }
        else Serial.println("!ERROR type for servo motion cmd.");
        break;
      case MSG_TYPE_RPI_ADAFUN:
        // assert robot_state == 1 (or 0)
        if (MSG_RPI_ADAFUN_TYPE(header) == MSG_TYPE_ADAFUN_LOCAL_AVOID_LEFT) {
          Serial.println("Doing local avoidance left.");
          obs.set_direction(true);
          robot_state = 2;
        }
        else if(MSG_RPI_ADAFUN_TYPE(header) == MSG_TYPE_ADAFUN_LOCAL_AVOID_RIGHT) {
          Serial.println("Doing local avoidance right.");
          obs.set_direction(false);
          robot_state = 2;
        }
        break;
      default:
        break;
    }
#ifdef DEBUG
    display_displacement();
#endif
    Serial.write(0x00);


    // grasp
    // remind to check if it's grasping

  }
  // check if another command
  if (Serial.available() > 0) {
    Serial.println("NEW COMMAND ARRIES TOO QUICK");
    Serial.write(0x00);
  }
  //report_error('R'); // Overflow ? transmission is too slow? Buffer needed?
  // else no info from rpi
}

// Protocol - Write
void _send_to_rpi_mul(byte msg[], byte length) {
  Serial.write(msg, length);
  Serial.write(MSG_SEPARATOR);
}
void _send_to_rpi(byte msg) {
  Serial.write(msg);
  Serial.write(MSG_SEPARATOR);
}

void _pick_done() {
  _send_to_rpi(MSG_ARD_REPORT_DONE_PICKUP);
}
void _empty_done() {
  _send_to_rpi(MSG_ARD_REPORT_DONE_EMPTY);
}
void _avoidance_done() {
  _send_to_rpi(MSG_ARD_REPORT_DONE_LOCALAVOID);
}

void warn_obs(byte index, byte dis) {
  byte msg[] = MSG_ARD_REPORT_WARN(index, dis+1);
  _send_to_rpi_mul(msg, 2);
}

void report_obs(byte index, byte dis) {
  byte msg[] = MSG_ARD_REPORT_OBS_FRONT(index, dis+1);
  _send_to_rpi_mul(msg, 2);
}

void send_displacement() {
  double left = 0, right = 0;
  motion.get_delta_displacement(&left, &right);
  if (abs(left) + abs(right) < 0.1) return;
#ifdef DEBUG
  Serial.print("delta disp:");
  Serial.print(left);
  Serial.print(",");
  Serial.print(right);
  Serial.println("");
  Serial.write(0x00);
#endif
  byte msg[] = MSG_ARD_REPORT_DISPLACEMENT(SIGNED_BYTE(left * 10), SIGNED_BYTE(right * 10));
  _send_to_rpi_mul(msg, 3);
}

// Speed Monitor
// This function must be written here rather than in the included header files.
void update_displacement() {
  double l, r;
  motion.get_cur_speed(&l, &r);
  GB_displacement_left += l * GB_period / 1000000.0;
  GB_displacement_right += r * GB_period / 1000000.0;
}
