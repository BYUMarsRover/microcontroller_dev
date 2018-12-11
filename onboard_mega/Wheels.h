#ifndef WHEELS_H
#define WHEELS_H

#include "Globals.h"
#include <Arduino.h>
#include "Wheel.h"

using namespace std;

class Wheels {
public:
//uint8_t set_speed_pin, uint8_t dir_pin, uint8_t actual_speed_pin, uint8_t error_pin
  Wheels(){  
    right_front.init(RIGHT_FRONT_WHEEL_SET_SPEED, RIGHT_FRONT_WHEEL_ENABLE, RIGHT_FRONT_WHEEL_DIR, RIGHT_FRONT_WHEEL_ACTUAL_SPEED, RIGHT_FRONT_WHEEL_ERROR);
    right_middle.init(RIGHT_MIDDLE_WHEEL_SET_SPEED, RIGHT_MIDDLE_WHEEL_ENABLE, RIGHT_MIDDLE_WHEEL_DIR, RIGHT_MIDDLE_WHEEL_ACTUAL_SPEED, RIGHT_MIDDLE_WHEEL_ERROR);
    right_rear.init(RIGHT_REAR_WHEEL_SET_SPEED, RIGHT_REAR_WHEEL_ENABLE, RIGHT_REAR_WHEEL_DIR, RIGHT_REAR_WHEEL_ACTUAL_SPEED, RIGHT_REAR_WHEEL_ERROR);
    left_front.init(LEFT_FRONT_WHEEL_SET_SPEED, LEFT_FRONT_WHEEL_ENABLE, LEFT_FRONT_WHEEL_DIR, LEFT_FRONT_WHEEL_ACTUAL_SPEED, LEFT_FRONT_WHEEL_ERROR);
    left_middle.init(LEFT_MIDDLE_WHEEL_SET_SPEED, LEFT_MIDDLE_WHEEL_ENABLE, LEFT_MIDDLE_WHEEL_DIR, LEFT_MIDDLE_WHEEL_ACTUAL_SPEED, LEFT_MIDDLE_WHEEL_ERROR);
    left_rear.init(LEFT_REAR_WHEEL_SET_SPEED, LEFT_REAR_WHEEL_ENABLE, LEFT_REAR_WHEEL_DIR, LEFT_REAR_WHEEL_ACTUAL_SPEED, LEFT_REAR_WHEEL_ERROR);
  }
  
  ~Wheels(){}

  void updateFeedbackData() {
    right_front.updateFeedbackData();
    right_middle.updateFeedbackData();
    right_rear.updateFeedbackData();
    left_front.updateFeedbackData();
    left_middle.updateFeedbackData();
    left_rear.updateFeedbackData();
  }

  void writeParams() {
    right_front.writeParams();
    right_middle.writeParams();
    right_rear.writeParams();
    left_front.writeParams();
    left_middle.writeParams();
    left_rear.writeParams();    
  }

  String toString() {
    String result;

    result += right_front.toString("right front") + '\n';
    result += right_middle.toString("right middle") + '\n';
    result += right_rear.toString("right rear") + '\n';
    result += left_front.toString("left front") + '\n';
    result += left_middle.toString("left middle") + '\n';
    result += left_rear.toString("left rear");
    
    return result;
  }

  void clearErrorStates() {
    right_front.clearErrorState();
    right_middle.clearErrorState();
    right_rear.clearErrorState();
    left_front.clearErrorState();
    left_middle.clearErrorState();
    left_rear.clearErrorState();
  }

  Wheel right_front;
  Wheel right_middle;
  Wheel right_rear;
  Wheel left_front;
  Wheel left_middle;
  Wheel left_rear;

};

#endif
