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
  }

  // finish this later... the intention is to init the arm turret target to the current location at startup to prevent the arm from thrashing on startup
  void init_turret() {
    float sensorValue = analogRead(ARM_TURRET_FB);             // gives number from 100-917
    sensorValue = (sensorValue - 100) / 2.269444;           // changes to number from 0-360
  }

  void write_params() {
    write_shoulder_params();
    write_elbow_params();
    write_wrist_params();
    write_hand_params();
  }

  int oneCheck = 1;
  int twoCheck = 2;
  int threeCheck = 3;
  const int TURRET_BUFFER = 12; // degrees in wiggle room :)
  const int TURRET_TURN_RIGHT = 200;
  const int TURRET_TURN_LEFT = 54;
  const int TURRET_STOP = 127;

  
  void write_turret_params() {
    Wire.beginTransmission(TURRET_NANO_ADDRESS);
    Wire.write(turret_high);
    Wire.write(turret_low);
    Wire.endTransmission();
  }
  
  void write_shoulder_params() {
    uint16_t val = (shoulder_high << 8) | shoulder_low;
    shoulder.setTarget(val);
//    Serial.println(val);
  }
  
  void write_elbow_params() {
    uint16_t val = (elbow_high << 8) | elbow_low;
    elbow.setTarget(val);
//    Serial.println(val);
  }

  void write_wrist_params() {
    int val = map(wrist_speed,0,255,0,2048);
    if (wrist_dir == 0) {
      val = -val + 2048;
    } else {
      val += 2047;
    }
    //Serial.println(val);
    wrist.setTarget(val);
  }

//  #define LEFT_HAND_PWM 9
//  #define RIGHT_HAND_PWM 10
//  #define LEFT_HAND_LN_A 40
//  #define LEFT_HAND_LN_B 41
//  #define RIGHT_HAND_LN_A 42
//  #define RIGHT_HAND_LN_B 43

  void write_hand_params() {

//    Serial.println("writing gripper params");
    digitalWrite(LEFT_HAND_LN_A, hand_dir);
    digitalWrite(LEFT_HAND_LN_B, !hand_dir);
    digitalWrite(RIGHT_HAND_LN_A, hand_dir);
    digitalWrite(RIGHT_HAND_LN_B, !hand_dir);

    analogWrite(LEFT_HAND_PWM, hand_speed);
    analogWrite(RIGHT_HAND_PWM, hand_speed);
//    Serial.print("done. dir: ");
//    Serial.print(hand_dir);
//    Serial.print("___ speed: ");
//    Serial.println(hand_speed);
    
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
