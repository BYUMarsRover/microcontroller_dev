#ifndef WHEELS_H
#define WHEELS_H

#include "Globals.h"
#include <Arduino.h>
#include "Wheel.h"

using namespace std;

class Wheels {
public:
  Wheels(){
    wheelList[0].init(LEFT_FRONT_WHEEL_SET_SPEED,  LEFT_FRONT_WHEEL_ENABLE,  LEFT_FRONT_WHEEL_DIR,  LEFT_FRONT_WHEEL_ACTUAL_SPEED, LEFT_FRONT_WHEEL_ERROR, false);
    wheelList[1].init(LEFT_MIDDLE_WHEEL_SET_SPEED, LEFT_MIDDLE_WHEEL_ENABLE, LEFT_MIDDLE_WHEEL_DIR, LEFT_MIDDLE_WHEEL_ACTUAL_SPEED, LEFT_MIDDLE_WHEEL_ERROR, false);
    wheelList[2].init(LEFT_REAR_WHEEL_SET_SPEED,   LEFT_REAR_WHEEL_ENABLE,   LEFT_REAR_WHEEL_DIR,   LEFT_REAR_WHEEL_ACTUAL_SPEED, LEFT_REAR_WHEEL_ERROR, false);
    wheelList[3].init(RIGHT_REAR_WHEEL_SET_SPEED,  RIGHT_REAR_WHEEL_ENABLE,  RIGHT_REAR_WHEEL_DIR,  RIGHT_REAR_WHEEL_ACTUAL_SPEED, RIGHT_REAR_WHEEL_ERROR, true);
    wheelList[4].init(RIGHT_MIDDLE_WHEEL_SET_SPEED,RIGHT_MIDDLE_WHEEL_ENABLE,RIGHT_MIDDLE_WHEEL_DIR,RIGHT_MIDDLE_WHEEL_ACTUAL_SPEED, RIGHT_MIDDLE_WHEEL_ERROR, true);
    wheelList[5].init(RIGHT_FRONT_WHEEL_SET_SPEED, RIGHT_FRONT_WHEEL_ENABLE, RIGHT_FRONT_WHEEL_DIR, RIGHT_FRONT_WHEEL_ACTUAL_SPEED, RIGHT_FRONT_WHEEL_ERROR, true);
  }
  
  ~Wheels(){}

  void printVals() {
    for (int i = 0; i < NUM_WHEELS; i++) {
      Serial.print(i);
      Serial.print(". set_speed: ");
      Serial.println(wheelList[i].set_speed);
      
      Serial.print("\tdir: ");
      Serial.println(wheelList[i].dir);
    }
    Serial.println("");
  }

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

  Wheel wheelList[NUM_WHEELS];
};

#endif
