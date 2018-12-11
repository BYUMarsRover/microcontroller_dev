#ifndef WHEELS_H
#define WHEELS_H

#include "Globals.h"
#include <Arduino.h>
#include "Wheel.h"

using namespace std;

class Wheels {
public:
  Wheels(){
    wheelList[0] = Wheel(RIGHT_FRONT_WHEEL_SET_SPEED, RIGHT_FRONT_WHEEL_ENABLE, RIGHT_FRONT_WHEEL_DIR, RIGHT_FRONT_WHEEL_ACTUAL_SPEED, RIGHT_FRONT_WHEEL_ERROR);
    wheelList[1] = Wheel(RIGHT_MIDDLE_WHEEL_SET_SPEED, RIGHT_MIDDLE_WHEEL_ENABLE, RIGHT_MIDDLE_WHEEL_DIR, RIGHT_MIDDLE_WHEEL_ACTUAL_SPEED, RIGHT_MIDDLE_WHEEL_ERROR);
    wheelList[2] = Wheel(RIGHT_REAR_WHEEL_SET_SPEED, RIGHT_REAR_WHEEL_ENABLE, RIGHT_REAR_WHEEL_DIR, RIGHT_REAR_WHEEL_ACTUAL_SPEED, RIGHT_REAR_WHEEL_ERROR);
    wheelList[3] = Wheel(LEFT_FRONT_WHEEL_SET_SPEED, LEFT_FRONT_WHEEL_ENABLE, LEFT_FRONT_WHEEL_DIR, LEFT_FRONT_WHEEL_ACTUAL_SPEED, LEFT_FRONT_WHEEL_ERROR);
    wheelList[4] = Wheel(LEFT_MIDDLE_WHEEL_SET_SPEED, LEFT_MIDDLE_WHEEL_ENABLE, LEFT_MIDDLE_WHEEL_DIR, LEFT_MIDDLE_WHEEL_ACTUAL_SPEED, LEFT_MIDDLE_WHEEL_ERROR);
    wheelList[5] = Wheel(LEFT_REAR_WHEEL_SET_SPEED, LEFT_REAR_WHEEL_ENABLE, LEFT_REAR_WHEEL_DIR, LEFT_REAR_WHEEL_ACTUAL_SPEED, LEFT_REAR_WHEEL_ERROR);
  }
  
  ~Wheels(){}

  void updateFeedbackData() {
    for (int i = 0; i < NUM_WHEELS; i++) {
      wheelList[i].updateFeedbackData();
    }
  }

  void writeParams() {
    for (int i = 0; i < NUM_WHEELS; i++) {
      wheelList[i].writeParams();
    }
  }

  void setWheelParams(int i, byte set_speed, byte dir) {
    wheelList[i].set_speed = set_speed;
    wheelList[i].dir = dir;
  }

  Wheel wheelList[NUM_WHEELS];
};

#endif
