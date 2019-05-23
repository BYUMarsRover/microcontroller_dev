#ifndef ARM_H
#define ARM_H
#include "Globals.h"
#include <Arduino.h>
#include <JrkG2.h>
#define FINGER_MAX_COUNT 12
#define IN HIGH
#define OUT LOW

#define MAX_ELBOW 2469
#define MIN_ELBOW 0
#define MAX_SHOULDER 2965
#define MIN_SHOULDER 0

JrkG2I2C shoulder(SHOULDER_ADDRESS);
JrkG2I2C elbow(ELBOW_ADDRESS);
JrkG2I2C wrist(WRIST_ADDRESS);

using namespace std;

static enum state{wait, runningOut, runningIn} fingerState;//state for PD controller

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
    hand_speed = 0;
    finger_enable = 0;
    fingerSM_init();
  }
  ~Arm(){}

  void printVals() {
    Serial.println(turret_high);
    Serial.println(turret_low);
    Serial.println(shoulder_high);
    Serial.println(shoulder_low);
    Serial.println(elbow_high);
    Serial.println(elbow_low);
    Serial.println(wrist_speed);
    Serial.println(wrist_dir);
    Serial.println(hand_speed);
    Serial.println(hand_dir);
    Serial.println(finger_enable);
  }

  void write_params() {
    write_turret_params();
    write_shoulder_params();
    write_elbow_params();
    write_wrist_params();
    write_hand_params();
  }

  void write_turret_params() {
    Wire.beginTransmission(TURRET_ADDRESS);
    Wire.write(turret_high);
    Wire.write(turret_low);
    Wire.endTransmission();
  }
  
  void write_shoulder_params() {
    uint16_t val = (shoulder_high << 8) | shoulder_low;
    if (val > MAX_SHOULDER) {
      val = MAX_SHOULDER;
    }
    else if (val < MIN_SHOULDER) {
      val = MIN_SHOULDER;
    }
    shoulder.setTarget(val);
  }
  
  void write_elbow_params() {
    uint16_t val = (elbow_high << 8) | elbow_low;
    if (val > MAX_ELBOW) {
      val = MAX_ELBOW;
    }
    else if (val < MIN_ELBOW) {
      val = MIN_ELBOW;
    }
    elbow.setTarget(val);
  }

  void write_wrist_params() {
    int val = map(wrist_speed,0,255,0,2048);
    if (wrist_dir == 0) {
      val = -val + 2048;
    } else {
      val += 2047;
    }
    wrist.setTarget(val);
  }

  void write_hand_params() {
    digitalWrite(LEFT_HAND_LN_A, hand_dir);
    digitalWrite(LEFT_HAND_LN_B, !hand_dir);
    //digitalWrite(RIGHT_HAND_LN_A, hand_dir);
    //digitalWrite(RIGHT_HAND_LN_B, !hand_dir);

    analogWrite(LEFT_HAND_PWM, hand_speed);
    //analogWrite(RIGHT_HAND_PWM, hand_speed);
  }

  
  void write_finger_params(byte finger_dir, byte speed) {
    //set the direction
    digitalWrite(FINGER_LN_A, finger_dir);
    digitalWrite(FINGER_LN_B, !finger_dir);
    
    //set the speed
    analogWrite(FINGER_PWM, speed);
  }

  void fingerSM_init() {
    fingerState = runningIn;
    fingerCounter = 0;
  }

  void fingerSM_tick() {
    switch (fingerState) {//state transitions
      case wait:
        if (finger_enable) {//start running the finger
          fingerCounter = 0;//reset counter
          fingerState = runningOut;
          write_finger_params(OUT, 255);//push the finger out          
        }
      break;

      case runningOut:
        if (fingerCounter >= FINGER_MAX_COUNT) {
          fingerState = runningIn;
          fingerCounter = 0;//reset counter
          write_finger_params(IN, 255);//pull the finger in          
        }
      break;

      case runningIn:
        if (fingerCounter >= FINGER_MAX_COUNT) {
          fingerState = wait;
          finger_enable = 0;
          fingerCounter = 0;//reset counter          
          write_finger_params(OUT, 0);//stop the finger          
        }
      break;
    }

    switch (fingerState) {//state actions
      case runningOut:
      case runningIn:
        fingerCounter++;
      break;      
    }
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
  byte finger_enable;
  long fingerCounter;//counter for how long to move the finger in/out
};

#endif
