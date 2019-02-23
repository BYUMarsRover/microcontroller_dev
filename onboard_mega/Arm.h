#ifndef ARM_H
#define ARM_H
#include "Globals.h"
#include <Arduino.h>
#include <JrkG2.h>

JrkG2I2C shoulder(SHOULDER_ADDRESS);
JrkG2I2C elbow(ELBOW_ADDRESS);

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
    //do this
  }

  void write_hand_params() {
    //do this!
  }

  byte turret_high;
  byte turret_low;
  byte shoulder_high;
  byte shoulder_low;
  byte elbow_high;
  byte elbow_low;
  byte wrist_speed;
  byte wrist_dir;
  byte hand_dir;

};

#endif
