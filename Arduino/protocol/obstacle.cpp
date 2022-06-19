#include "obstacle.h"

Ir_range::Ir_range(const int _pin): pin_value(_pin){ }

void Ir_range::init(){ pinMode(pin_value, INPUT);}

int Ir_range::_get_raw(){ return analogRead(pin_value);}

int Ir_range::_median_value(){
  int sort[IR_FILTER_LENGTH];
  for(int i = 0; i< IR_FILTER_LENGTH; i++){
    sort[i] = _get_raw(); 
  }

  // sort - bubble      
  for(byte i = 0; i< IR_FILTER_LENGTH-1; i++){
    bool flag = true;
    for(byte j = 0; j<IR_FILTER_LENGTH-i-1; j++){
      if(sort[j] > sort[j+1]){
        int temp = sort[j];
        sort[j] = sort[j+1];
        sort[j+1] = temp;
        flag = false;
      }
    }
    if(flag) break;
  }
  return sort[IR_FILTER_LENGTH/2];
  //if (IR_FILTER_LENGTH%2 == 1) return sort[IR_FILTER_LENGTH/2];
  //else return ((int)sort[IR_FILTER_LENGTH/2-1]+(int)sort[IR_FILTER_LENGTH/2])/2;
}

int Ir_range::get_distance(){
  int median = _median_value();
  // formula used by SharpIR library
  return 27.728 * pow(map(median, 0, 1023, 0, 5000)/1000.0, -1.2045);
}    


void Obstacle_detection::init(Motion *m){
  mo = m;
  for(int i = 0; i < 5; i++){
    ir_list[i].init();
  }
}
bool Obstacle_detection::obstacle_dis(byte distance[5]){
  // return true if there's obstacle
  bool obs = false;
  for(int i = 0; i < 5; i++){
    int dis = (ir_list[i].get_distance());
    // irregular value
    if(dis > OBS_REPORT_THRESHOLD || dis < 3) dis = OBS_REPORT_THRESHOLD + 20;
    else obs = true;
    distance[i] = (byte)dis;
  }
  return obs;
}

// Obstacle Status
// return
// 0 -- No obstacle
// 1 -- Stopped!
// 2 -- Detected 
byte Obstacle_detection::obstacle_check(byte *min_dis, byte *min_index) {
  byte distances[5];
  bool ret = obstacle_dis(distances);
  if (!ret) return 0;

  // Find the minimum
  *min_dis = 255;
  *min_index = 0;
  for(int i=0; i< 5; i++){
    if(distances[i] < *min_dis){
      *min_dis = distances[i];
      *min_index = i;
    }
  }

  if(*min_dis < OBS_STOP_THRESHOLD){
    mo->stop();
    return 1;
    //warn_obs(min_index, min_dis);
  }

  unsigned long timer_now = millis();
  if ((timer_now - timer_last_report_obs > OBS_REPORT_MIN_PERIOD) &&
      (*min_dis < OBS_REPORT_THRESHOLD ))
  {
    //report_obs(min_index, min_dis);
    timer_last_report_obs = timer_now;
    return 2;
  }
  return 0;
}

// Local avoidance
// return clear or not
bool Obstacle_detection::_detectNturn(bool turn_started){
  byte distances[5];
  bool ret = obstacle_dis(distances);
  if(!ret) return true; // clear

#ifdef DEBUG
    Serial.print("Distances: ");
    for(int i = 0; i<5; i++){
      Serial.print(distances[i]);
      Serial.print(", ");
    }
    Serial.println("");
    Serial.write(0x00);
#endif

  // find the closest obstacle
  byte min_dis = 255, min_index = 0;
  for(int i=0; i< 5; i++){
    if(distances[i] < min_dis){
      min_dis = distances[i];
      min_index = i;
    }
  }
  if(min_dis > OBS_AVOID_THRESHOLD) return true;

  DIRECTION dir = dir_default;
  /*if(!turn_started){
    int left = distances[0] + distances[1];
    int right = distances[3] + distances[4];
    //if(min_index < 2) dir = DIRECTION::RIGHT;
    //else if(min_index > 2) dir = DIRECTION::LEFT;
    if(left > right) dir = DIRECTION::LEFT;
    else dir = DIRECTION::RIGHT;
    dir_default = dir;
  }*/
  
  mo->set_cmd_speed(OBS_TURN_SPEED, dir_default);
  delay(OBS_TURN_TIME);
  mo->stop();
  return false;
}

void Obstacle_detection::local_avoidance() {
  bool clear = _detectNturn(false);
  while (!clear) {
    clear = _detectNturn(true);
#ifdef DEBUG
    Serial.println("One turn");
    Serial.write(0x00);
#endif
  }
#ifdef DEBUG
  Serial.println("Clear!");
  Serial.write(0x00);
#endif
  mo->set_cmd_dis(OBS_AVOID_TURN_DISTANCE, dir_default);
  delay(OBS_AVOID_TURN_DISTANCE/15.0*1000 +100);
  //while(mo->sync()){    delay(1);  }
  mo->set_cmd_dis(OBS_AVOID_FORWARD_DISTANCE, DIRECTION::FORWARD);
  delay(OBS_AVOID_FORWARD_DISTANCE/15.0*1000 +100);
}
void Obstacle_detection::set_direction(bool left){
  if(left)dir_default = DIRECTION::LEFT;
  else dir_default = DIRECTION::RIGHT;
}
