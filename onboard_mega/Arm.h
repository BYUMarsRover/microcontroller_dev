#ifndef ARM_H
#define ARM_H
#include "Globals.h"
#include <Arduino.h>
#include <JrkG2.h>

JrkG2I2C shoulder(SHOULDER_ADDRESS);
JrkG2I2C elbow(ELBOW_ADDRESS);
JrkG2I2C wrist(WRIST_ADDRESS);

using namespace std;

class Arm {
public:
  Arm(){
    turret_high = 0;
    turret_low = 0;
    shoulder_high = 0;
    shoulder_low = 0;
    elbow_high = 0;
    elbow_low = 0;
    wrist_speed = 0;
    wrist_dir = 0;
    hand_dir = 0;
  }
  ~Arm(){}

  void write_params() {
    write_turret_params();
    write_shoulder_params();
    write_elbow_params();
    write_wrist_params();
    write_hand_params();
  }

  void write_turret_params() {
    
  }
  
  void write_shoulder_params() {
    uint16_t val = (shoulder_high << 8) | shoulder_low;
    shoulder.setTarget(val);
  }
  
  void write_elbow_params() {
    uint16_t val = (elbow_high << 8) | elbow_low;
    elbow.setTarget(val);
  }

  void write_wrist_params() {
    int val = map(wrist_speed,0,255,0,2048);
    if (wrist_dir == 0) {
      val = -val + 2048;
    } else {
      val += 2047;
    }
    Serial.println(val);
    wrist.setTarget(val);
  }

//  #define LEFT_HAND_PWM 9
//  #define RIGHT_HAND_PWM 10
//  #define LEFT_HAND_LN_A 40
//  #define LEFT_HAND_LN_B 41
//  #define RIGHT_HAND_LN_A 42
//  #define RIGHT_HAND_LN_B 43

  void write_hand_params() {
    
    digitalWrite(LEFT_HAND_LN_A, hand_dir);
    digitalWrite(LEFT_HAND_LN_B, !hand_dir);
    digitalWrite(RIGHT_HAND_LN_A, hand_dir);
    digitalWrite(RIGHT_HAND_LN_B, !hand_dir);

    analogWrite(LEFT_HAND_PWM, hand_speed);
    analogWrite(RIGHT_HAND_PWM, hand_speed);
  }

  byte turret_high;
  byte turret_low;
  byte shoulder_high;
  byte shoulder_low;
  byte elbow_high;
  byte elbow_low;
  byte wrist_speed;
  byte wrist_dir;
  byte hand_speed;
  byte hand_dir;

};

#endif
