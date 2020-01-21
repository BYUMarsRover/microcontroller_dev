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
    this->acceleration_pin = 0;
    this->deceleration_pin = 0;
    
    this->set_speed = 0;
    this->actual_speed = 0;
    this->dir = true;
    this->error = false;

    this->is_right_side_wheel = true;
  }
  
  void init(uint8_t set_speed_pin, uint8_t enable_pin, uint8_t dir_pin, uint8_t actual_speed_pin, uint8_t error_pin, uint8_t acceleration_pin, uint8_t deceleration_pin, bool is_right_side_wheel){
    this->set_speed_pin = set_speed_pin;
    this->enable_pin = enable_pin;
    this->dir_pin = dir_pin;
    this->actual_speed_pin = actual_speed_pin;
    this->error_pin = error_pin;
    this->acceleration_pin = acceleration_pin;
    this->deceleration_pin = deceleration_pin;
    this->is_right_side_wheel = is_right_side_wheel;
    
    digitalWrite(enable_pin, true);
  }
  
  ~Wheel(){}
  
  void updateFeedbackData() {
    
    this->error = digitalRead(this->error_pin);
    /*
     * the actual_speed is reported by the motor controller as a voltage ranging from 0-4V (representing -5400rpm to +5400rpm).
     * the arduino receives this through analogRead() which expects a 0-5V input range. This is where the value 818.4 (1023 * (4/5) 
     * comes from. analogRead() will report the 0-5V input as a value in the range 0-1023 (because it has a 10 bit analog-to-digital converter.
     * But because our input will range from 0-v volts we map the input from 0 to (1023 * (4/5) = 818.4) to a new range from 
     * -255 to 255. This adjusts for the 4V to 5V difference. In addition, we take the absolute value of this reading and report.
     * The direction of the motor, CW or CCW, determines the polarity of the reading.
     */
    if (!this->error) this->actual_speed = abs((map(analogRead(actual_speed_pin),0,818.4,-255,255)));
    else this->actual_speed = 0;
    
  }

  uint16_t convertToRpm(uint8_t rawSpeed) {
    // escon output ranges from 0-4V while MEGA input ranges from 0-5V and we want to convert a range [0,1023] to [ESCON_RPM_RANGE_MIN, ESCON_RPM_RANGE_MAX]. 818.4 is 4/5 of 1023.
    // this equation is the equation of a line on a graph representing the output range on the y axis and the input range on th x axis. y = mx + b
    return ((((ESCON_RPM_RANGE_MAX - ESCON_RPM_RANGE_MIN) / 818.4) * rawSpeed) + ESCON_RPM_RANGE_MAX); 
  }

  void writeParams() {
    /*
    * The Escon Drive Controllers require a minuimum-maximum range for the duty cycle (pwm) of 10% - 90%. So the desired speed is mapped from
    *  a range of [0,255] to [255*.1,255*.9].
    */
    analogWrite(set_speed_pin, map(set_speed,0,255,(255 * .1),(255 * .9)));
    digitalWrite(dir_pin, is_right_side_wheel == dir);
    analogWrite(acceleration_pin, acceleration);
    analogWrite(deceleration_pin, deceleration);
  }

  uint8_t set_speed_pin;
  uint8_t dir_pin;
  uint8_t enable_pin;
  uint8_t actual_speed_pin;
  uint8_t error_pin;
  uint8_t acceleration_pin;
  uint8_t deceleration_pin;
  
  byte acceleration;
  byte deceleration;
  byte set_speed;
  byte actual_speed;
  
  bool dir;
  bool error;

  bool is_right_side_wheel;
};


#endif
