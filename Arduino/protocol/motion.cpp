#include "motion.h"

void Motor::init(){
    // maybe todo: check if connected; check the speed if is expected
    pinMode(pin_pwm, OUTPUT);
    pinMode(pin_en, OUTPUT);
    pinMode(pin_dir, OUTPUT);
    pinMode(pin_cur_spd_front, INPUT);
    pinMode(pin_cur_spd_hind, INPUT);

    digitalWrite(pin_dir, 1);
    digitalWrite(pin_pwm, 0);
    digitalWrite(pin_en, 0);
}
void Motor::set_speed(byte value, bool dir){
    direction = dir;
    digitalWrite(pin_dir, dir);
    digitalWrite(pin_en, HIGH);
    analogWrite(pin_pwm, truncat(value));
    if(value > 0) isStill = false;
}
void Motor::stop(){
    digitalWrite(pin_pwm, LOW);
    digitalWrite(pin_en, LOW);
    isStill = true;
}
float Motor::current_speed(byte filter_threshold){
    int valuef = analogRead(pin_cur_spd_front);
    int valueh = analogRead(pin_cur_spd_hind);
    int value = (valuef + valueh)/2.0;
    // if no speed command, then small speed values should be treat as the noise
    if(isStill && abs(value - 409.6) < filter_threshold)return 0;
    float rpm = MOTOR_SPEED_MEASURE_MAX_RPM/2.0 * (value/1024.0*5.0 - MOTOR_SPEED_MEASURE_MAX_VOLTAGE/2.0);
    return rpm * WHEEL_PERIMETER / MAXON_REDUCTION / 60;
}

void Motion::start_speed_monitor(){
    Timer3.initialize(period); // 1ms
    Timer3.attachInterrupt(*p_int_func);
}

// maybe you should never call this
void Motion::end_speed_monitor(){
    Timer3.detachInterrupt();
    *p_cnt_left = 0;
    *p_cnt_right = 0;
    displacement_left_last = 0;
    displacement_right_last = 0;
}

void Motion::init(void (*itrpt)(), unsigned long _period){
    motor_left.init();
    motor_right.init();
    
    period = _period;
    p_int_func = itrpt;
    start_speed_monitor();
}
void Motion::set_cmd_speed(byte value, DIRECTION dir,unsigned long timeout/* = SPEED_MODE_TIMEOUT*/){
    // old cmd is still valid
    if(cmd.valid && !cmd.isTimeOut()){
        // the same direction, so we just update the cmd
        if(cmd.speed_control && cmd.dir == dir){
            cmd.speed = value;
            cmd.timeout += timeout;
            return;
        }
        else{//stop first
            stop();
        }
    }
    // set new cmd
    cmd.speed_control = true;
    cmd.speed = value;
    cmd.dir = dir;
    cmd.timeout = timeout;
    cmd.timer_start = millis();
    cmd.valid = true;
    move(value, dir);
}
void Motion::set_cmd_speed2(int left, int right, unsigned long timeout/* = 1000*/){
    // do not stop old cmd
    // set new cmd
    cmd.speed_control = true;
    cmd.timeout = timeout;
    cmd.timer_start = millis();
    cmd.valid = true;
    
    bool dir_left = left > 0;
    bool dir_right = right > 0;
    motor_left.set_speed(abs(left), dir_left);
    motor_right.set_speed(abs(right), dir_right);
}
void Motion::set_cmd_dis(byte dis, DIRECTION dir, unsigned long timeout/* = 0*/){
    // old cmd is still valid
    if(cmd.valid && !cmd.isTimeOut()){
    /*// the same direction, so we just update the cmd
        if(!cmd.speed_control && cmd.dir == dir){
            cmd.timeout += timeout;
            cmd.distance += dis;
            return;
        }*/
        //else{//stop first
            stop();
        //}
    }
    // set new cmd
    cmd.speed_control = false;
    cmd.distance = dis;
    get_displacement(&cmd.disp_l, &cmd.disp_r);
    cmd.dir = dir;
    if(timeout == 0) timeout = (int)((float)(dis)/(DISTANCE_MODE_SPEED*0.0003)) + 1000;
    cmd.timeout = timeout;
    cmd.timer_start = millis();
    cmd.valid = true;  
    move(distance_mode_speed, dir);    
}
    
// run this function as much as possible...
bool Motion::sync(){
    if(cmd.valid){
        // we are moving!
        if(cmd.isTimeOut()){
            // Ohoh, we can't move anymore
#ifdef DEBUG
            Serial.println("Command Timeout ");
#endif
            stop();
            return false;;
        }
        // Speed mode: let controller board do the speed close-loop control
        if(cmd.speed_control) return true;
        // Distance mode: update the distance travelled
        double l, r;
        get_displacement(&l, &r);   
        double travelled = (SIGN_DIRECTION((cmd.dir >> 1))*(l-cmd.disp_l) + 
                            SIGN_DIRECTION((cmd.dir & 1))*(r-cmd.disp_r))/2;
        if(cmd.distance - travelled < torlerance ){
#ifdef DEBUG
            Serial.print("Travelled: ");
            Serial.println(travelled);
#endif
            // OK, we finished our task
            stop();
            return false;
        }
        return true;
    }
    //else stop();
    return false;
}

void Motion::stop(){
    motor_left.stop();
    motor_right.stop();
    cmd.valid = false; // no more cmd should be executed
#ifdef DEBUG
    Serial.println("Stop");
#endif
}

void Motion::get_cur_speed(double* left, double* right){
    *left = motor_left.current_speed();
    *right = -motor_right.current_speed();
}
/* void Motion::get_avg_speed(double* left, double* right){
    *left = motor_left.avg_speed();
    *right = -motor_right.avg_speed();
}*/

void Motion::get_displacement(double* left, double* right){
    noInterrupts();
    *left = *p_cnt_left;
    *right = *p_cnt_right;
    interrupts();
}
    
void Motion::get_delta_displacement(double* left, double* right){
    double l, r;
    get_displacement(&l, &r);
    double dl = l - displacement_left_last;
    double dr = r - displacement_right_last;
    // reserve the displacement less than 0.1cm
    // todo: 10.1?
    if(abs(dl) < 0.1){
    *left = 0;
    }
    else{
    *left = dl;
    displacement_left_last = l;
    }
    if(abs(dr) < 0.1){
    *right = 0;
    }
    else{
    *right = dr;
    displacement_right_last = r;
    }
}
