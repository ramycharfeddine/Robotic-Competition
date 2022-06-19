#ifndef _IR_RANGE_H
#define _IR_RANGE_H
#include "utils.h"
#include "motion.h"
#include "Arduino.h"

#define IR_FILTER_LENGTH 25

class Ir_range{
    const int pin_value;

public:
    Ir_range(const int _pin);
    void init();    
    int _get_raw();
    int _median_value();
    int get_distance();
};

class Obstacle_detection{
    Ir_range ir_list[5] = {
      Ir_range(PIN_IR_1),
      Ir_range(PIN_IR_2),
      Ir_range(PIN_IR_3),
      Ir_range(PIN_IR_4),
      Ir_range(PIN_IR_5)};
    Motion *mo;
    DIRECTION dir_default = DIRECTION::LEFT;
    unsigned long timer_last_report_obs = 0;
public:
    void init(Motion *m);
    bool obstacle_dis(byte distance[5]);
    byte obstacle_check(byte *min_dis, byte *min_index);
    bool _detectNturn(bool turn_started);
    void local_avoidance();
    void set_direction(bool left);
};

#endif
