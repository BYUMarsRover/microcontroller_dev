#ifndef WHEELS_H
#define WHEELS_H

#include "Globals.h"
#include <Arduino.h>

using namespace std;

class Wheels {
public:

  Wheels(){  
    right_front_speed = 0;
    right_front_dir = 0;
    right_front_enable = 0;
    
    right_middle_speed = 0;
    right_middle_dir = 0;
    right_middle_enable = 0;
    
    right_rear_speed = 0;
    right_rear_dir = 0;
    right_rear_enable = 0;
  
    left_front_speed = 0;
    left_front_dir = 0;
    left_front_enable = 0;
    
    left_middle_speed = 0;
    left_middle_dir = 0;
    left_middle_enable = 0;
    
    left_rear_speed = 0;
    left_rear_dir = 0;
    left_rear_enable = 0;
  }
  
  ~Wheels(){}

  void applyParams() {
    if (left_front_speed < (255 * (MIN_SPEED_PERCENTAGE / 100))) left_front_enable = false;
    if (left_middle_speed < (255 * (MIN_SPEED_PERCENTAGE / 100))) left_middle_enable = false;
    if (left_rear_speed < (255 * (MIN_SPEED_PERCENTAGE / 100))) left_rear_enable = false;
    if (right_front_speed < (255 * (MIN_SPEED_PERCENTAGE / 100))) right_front_enable = false;
    if (right_middle_speed < (255 * (MIN_SPEED_PERCENTAGE / 100))) right_middle_enable = false;
    if (right_rear_speed < (255 * (MIN_SPEED_PERCENTAGE / 100))) right_rear_enable = false;
    writePins();
  }

  void writePins() {
    analogWrite(RIGHT_FRONT_WHEEL_SPEED, right_front_speed);
    digitalWrite(RIGHT_FRONT_WHEEL_DIR, right_front_dir);
    digitalWrite(RIGHT_FRONT_WHEEL_ENABLE, right_front_enable);

    analogWrite(RIGHT_MIDDLE_WHEEL_SPEED, right_middle_speed);
    digitalWrite(RIGHT_MIDDLE_WHEEL_DIR, right_middle_dir);
    digitalWrite(RIGHT_MIDDLE_WHEEL_ENABLE, right_middle_enable);

    analogWrite(RIGHT_REAR_WHEEL_SPEED, right_rear_speed);
    digitalWrite(RIGHT_REAR_WHEEL_DIR, right_rear_dir);
    digitalWrite(RIGHT_REAR_WHEEL_ENABLE, right_rear_enable);

    analogWrite(LEFT_FRONT_WHEEL_SPEED, left_front_speed);
    digitalWrite(LEFT_FRONT_WHEEL_DIR, left_front_dir);
    digitalWrite(LEFT_FRONT_WHEEL_ENABLE, left_front_enable);

    analogWrite(LEFT_MIDDLE_WHEEL_SPEED, left_middle_speed);
    digitalWrite(LEFT_MIDDLE_WHEEL_DIR, left_middle_dir);
    digitalWrite(LEFT_MIDDLE_WHEEL_ENABLE, left_middle_enable);

    analogWrite(LEFT_REAR_WHEEL_SPEED, left_rear_speed);
    digitalWrite(LEFT_REAR_WHEEL_DIR, left_rear_dir);
    digitalWrite(LEFT_REAR_WHEEL_ENABLE, left_rear_enable);
    Serial.println(this->toString());
  }

  char* toString() {
    return "hello world";
  }

  byte right_front_speed;
  byte right_front_dir;
  byte right_front_enable;
  
  byte right_middle_speed;
  byte right_middle_dir;
  byte right_middle_enable;
  
  byte right_rear_speed;
  byte right_rear_dir;
  byte right_rear_enable;

  byte left_front_speed;
  byte left_front_dir;
  byte left_front_enable;
  
  byte left_middle_speed;
  byte left_middle_dir;
  byte left_middle_enable;
  
  byte left_rear_speed;
  byte left_rear_dir;
  byte left_rear_enable;
  
};

#endif
