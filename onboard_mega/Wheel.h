#ifndef WHEEL_H
#define WHEEL_H

#include <Arduino.h>

class Wheel {
public:

  Wheel() {
    this->set_speed_pin = 0;
    this->enable_pin = 0;
    this->dir_pin = 0;
    this->actual_speed_pin = 0;
    this->error_pin = 0;
    
    this->set_speed = 0;
    this->actual_speed = 0;
    this->dir = true;
    this->error = false;

    this->is_right_side_wheel = true;
  }
  
  void init(uint8_t set_speed_pin, uint8_t enable_pin, uint8_t dir_pin, uint8_t actual_speed_pin, uint8_t error_pin, bool is_right_side_wheel){
    this->set_speed_pin = set_speed_pin;
    this->enable_pin = enable_pin;
    this->dir_pin = dir_pin;
    this->actual_speed_pin = actual_speed_pin;
    this->error_pin = error_pin;
    this->is_right_side_wheel = is_right_side_wheel;
    
    digitalWrite(enable_pin, true);
  }
  
  ~Wheel(){}
  
  void updateFeedbackData() {
//    this->actual_speed = convertToRpm(analogRead(actual_speed_pin));
    this->actual_speed = map(analogRead(actual_speed_pin),0,1023,0,255);
    this->error = digitalRead(this->error_pin);
  }

  uint16_t convertToRpm(uint8_t rawSpeed) {
    // escon output ranges from 0-4V while MEGA input ranges from 0-5V and we want to convert a range [0,1023] to [ESCON_RPM_RANGE_MIN, ESCON_RPM_RANGE_MAX]. 818.4 is 4/5 of 1023.
    // this equation is the equation of a line on a graph representing the output range on the y axis and the input range on th x axis. y = mx + b
    return ((((ESCON_RPM_RANGE_MAX - ESCON_RPM_RANGE_MIN) / 818.4) * rawSpeed) + ESCON_RPM_RANGE_MAX); 
  }

  void writeParams() {
    analogWrite(set_speed_pin, map(set_speed,0,255,(255*.1),(255*.9)));
    digitalWrite(dir_pin, is_right_side_wheel == dir);
  }

  uint8_t set_speed_pin;
  uint8_t dir_pin;
  uint8_t enable_pin;
  uint8_t actual_speed_pin;
  uint8_t error_pin;
  
  byte set_speed;
  byte actual_speed;
  
  bool dir;
  bool error;

  bool is_right_side_wheel;
};


#endif
