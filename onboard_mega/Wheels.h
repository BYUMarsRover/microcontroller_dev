#ifndef WHEELS_H
#define WHEELS_H

#include "Globals.h"
#include <Arduino.h>

using namespace std;

class Wheels {
public:

  Wheels(){  
    right_front_set_speed = 125;
    right_front_dir = true;
    right_front_enable = true;
    right_front_actual_speed = 0;
    right_front_amps = 0;
    right_front_error = false;
    
    right_middle_set_speed = 0;
    right_middle_dir = true;
    right_middle_enable = true;
    right_middle_actual_speed = 0;
    right_middle_amps = 0;
    right_middle_error = false;
    
    right_rear_set_speed = 0;
    right_rear_dir = true;
    right_rear_enable = true;
    right_rear_actual_speed = 0;
    right_rear_amps = 0;
    right_rear_error = false;
  
    left_front_set_speed = 125;
    left_front_dir = true;
    left_front_enable = true;
    left_front_actual_speed = 0;
    left_front_amps = 0;
    left_front_error = false;
    
    left_middle_set_speed = 0;
    left_middle_dir = true;
    left_middle_enable = true;
    left_middle_actual_speed = 0;
    left_middle_amps = 0;
    left_middle_error = false;
    
    left_rear_set_speed = 0;
    left_rear_dir = true;
    left_rear_enable = true;
    left_rear_actual_speed = 0;
    left_rear_amps = 0;
    left_rear_error = false;
  }
  
  ~Wheels(){}

  void determineEnableValues() {
    if (left_front_set_speed < (255 * (MIN_SET_SPEED_PERCENTAGE / 100))) left_front_enable = false;
    if (left_middle_set_speed < (255 * (MIN_SET_SPEED_PERCENTAGE / 100))) left_middle_enable = false;
    if (left_rear_set_speed < (255 * (MIN_SET_SPEED_PERCENTAGE / 100))) left_rear_enable = false;
    if (right_front_set_speed < (255 * (MIN_SET_SPEED_PERCENTAGE / 100))) right_front_enable = false;
    if (right_middle_set_speed < (255 * (MIN_SET_SPEED_PERCENTAGE / 100))) right_middle_enable = false;
    if (right_rear_set_speed < (255 * (MIN_SET_SPEED_PERCENTAGE / 100))) right_rear_enable = false;
  }

  void updateFeedbackData() {
    this->right_front_actual_speed = map(analogRead(RIGHT_FRONT_WHEEL_ACTUAL_SPEED),0,1023,-6000,9000);
//    this->right_front_actual_speed = map(analogRead(RIGHT_FRONT_WHEEL_ACTUAL_SPEED),0,1023,0,255);
    this->right_middle_actual_speed = map(analogRead(RIGHT_MIDDLE_WHEEL_ACTUAL_SPEED),0,1023,0,255);
    this->right_rear_actual_speed = map(analogRead(RIGHT_REAR_WHEEL_ACTUAL_SPEED),0,1023,0,255);
    this->left_front_actual_speed = map(analogRead(LEFT_FRONT_WHEEL_ACTUAL_SPEED),0,1023,0,255);
    this->left_middle_actual_speed = map(analogRead(LEFT_MIDDLE_WHEEL_ACTUAL_SPEED),0,1023,0,255);
    this->left_rear_actual_speed = map(analogRead(LEFT_REAR_WHEEL_ACTUAL_SPEED),0,1023,0,255);

    this->right_front_amps = map(analogRead(RIGHT_FRONT_WHEEL_AMPS),0,1023,0,255);
    this->right_middle_amps = map(analogRead(RIGHT_MIDDLE_WHEEL_AMPS),0,1023,0,255);
    this->right_rear_amps = map(analogRead(RIGHT_REAR_WHEEL_AMPS),0,1023,0,255);
    this->left_front_amps = map(analogRead(LEFT_FRONT_WHEEL_AMPS),0,1023,0,255);
    this->left_middle_amps = map(analogRead(LEFT_MIDDLE_WHEEL_AMPS),0,1023,0,255);
    this->left_middle_amps = map(analogRead(LEFT_MIDDLE_WHEEL_AMPS),0,1023,0,255);

  }

  void writeParams() {
    analogWrite(RIGHT_FRONT_WHEEL_SET_SPEED, right_front_set_speed);
    digitalWrite(RIGHT_FRONT_WHEEL_DIR, right_front_dir);

    analogWrite(RIGHT_MIDDLE_WHEEL_SET_SPEED, right_middle_set_speed);
    digitalWrite(RIGHT_MIDDLE_WHEEL_DIR, right_middle_dir);

    analogWrite(RIGHT_REAR_WHEEL_SET_SPEED, right_rear_set_speed);
    digitalWrite(RIGHT_REAR_WHEEL_DIR, right_rear_dir);

    analogWrite(LEFT_FRONT_WHEEL_SET_SPEED, left_front_set_speed);
    digitalWrite(LEFT_FRONT_WHEEL_DIR, left_front_dir);

    analogWrite(LEFT_MIDDLE_WHEEL_SET_SPEED, left_middle_set_speed);
    digitalWrite(LEFT_MIDDLE_WHEEL_DIR, left_middle_dir);

    analogWrite(LEFT_REAR_WHEEL_SET_SPEED, left_rear_set_speed);
    digitalWrite(LEFT_REAR_WHEEL_DIR, left_rear_dir);

    digitalWrite(RIGHT_FRONT_WHEEL_ENABLE, right_front_enable);
    digitalWrite(RIGHT_MIDDLE_WHEEL_ENABLE, right_middle_enable);
    digitalWrite(RIGHT_REAR_WHEEL_ENABLE, right_rear_enable);
    digitalWrite(LEFT_FRONT_WHEEL_ENABLE, left_front_enable);
    digitalWrite(LEFT_MIDDLE_WHEEL_ENABLE, left_middle_enable);
    digitalWrite(LEFT_REAR_WHEEL_ENABLE, left_rear_enable);
  }

  String toString() {
    String result;
    result += "right front speed: ";
    result += right_front_set_speed;
    result += "\nright front dir: ";
    result += right_front_dir;
    result += "\nright_front_enable: ";
    result += right_front_enable;

    result += "\nleft front speed: ";
    result += left_front_set_speed;
    result += "\nleft front dir: ";
    result += left_front_dir;
    result += "\nleft_front_enable: ";
    result += left_front_enable;

    result += "\nright middle speed: ";
    result += right_middle_set_speed;
    result += "\nright middle dir: ";
    result += right_middle_dir;
    result += "\nright_middle_enable: ";
    result += right_middle_enable;

    result += "\nleft middle speed: ";
    result += left_middle_set_speed;
    result += "\nleft middle dir: ";
    result += left_middle_dir;
    result += "\nleft_middle_enable: ";
    result += left_middle_enable;

    result += "\nright rear speed: ";
    result += right_rear_set_speed;
    result += "\nright rear dir: ";
    result += right_rear_dir;
    result += "\nright_rear_enable: ";
    result += right_rear_enable;

    result += "\nleft rear speed: ";
    result += left_rear_set_speed;
    result += "\nleft rear dir: ";
    result += left_rear_dir;
    result += "\nleft_rear_enable: ";
    result += left_rear_enable;
    
    return result;
  }

  byte right_front_set_speed;
  bool right_front_dir;
  bool right_front_enable;
  byte right_front_actual_speed;
  byte right_front_amps;
  bool right_front_error;
  
  byte right_middle_set_speed;
  bool right_middle_dir;
  bool right_middle_enable;
  byte right_middle_actual_speed;
  byte right_middle_amps;
  bool right_middle_error;
  
  byte right_rear_set_speed;
  bool right_rear_dir;
  bool right_rear_enable;
  byte right_rear_actual_speed;
  byte right_rear_amps;
  bool right_rear_error;

  byte left_front_set_speed;
  bool left_front_dir;
  bool left_front_enable;
  byte left_front_actual_speed;
  byte left_front_amps;
  bool left_front_error;
  
  byte left_middle_set_speed;
  bool left_middle_dir;
  bool left_middle_enable;
  byte left_middle_actual_speed;
  byte left_middle_amps;
  bool left_middle_error;
  
  byte left_rear_set_speed;
  bool left_rear_dir;
  bool left_rear_enable;
  byte left_rear_actual_speed;
  byte left_rear_amps;
  bool left_rear_error;
  
};

#endif
