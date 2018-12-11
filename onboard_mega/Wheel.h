#ifndef WHEEL_H
#define WHEEL_H

#include <Arduino.h>

class Wheel {
public:

  Wheel(){}
  
  Wheel(uint8_t set_speed_pin, uint8_t enable_pin, uint8_t dir_pin, uint8_t actual_speed_pin, uint8_t error_pin){
    this->set_speed_pin = set_speed_pin;
    this->enable_pin = enable_pin;
    this->dir_pin = dir_pin;
    this->actual_speed_pin = actual_speed_pin;
    this->error_pin = error_pin;
    
    this->set_speed = 0;
    this->actual_speed = 0;
    this->dir = true;
    this->error = false;
    
    digitalWrite(enable_pin, true);
    writeParams();
  }
  
  ~Wheel(){}
  
  void updateFeedbackData() {
    this->actual_speed = convertToRpm(analogRead(actual_speed_pin));
    this->error = digitalRead(error_pin);
  }

  double convertToRpm(double rawSpeed) {
    // escon output ranges from 0-4V while MEGA input ranges from 0-5V and we want to convert a range [0,1023] to [ESCON_RPM_RANGE_MIN, ESCON_RPM_RANGE_MAX]. 818.4 is 4/5 of 1023.
    // this equation is the equation of a line on a graph representing the output range on the y axis and the input range on th x axis. y = mx + b
    return ((((ESCON_RPM_RANGE_MAX - ESCON_RPM_RANGE_MIN) / 818.4) * rawSpeed) + ESCON_RPM_RANGE_MAX); 
  }

  void writeParams() {
    analogWrite(set_speed_pin, set_speed);
    digitalWrite(dir_pin, dir);
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
};


#endif
