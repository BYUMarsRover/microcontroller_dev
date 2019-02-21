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
  Arm(){}
  ~Arm(){}

  void set_turret_params(byte arg1, byte arg2) {
    //
    
  }
  
  void set_shoulder_params(byte arg1, byte arg2) {
    shoulder.setTarget((arg1 << 8) | arg2);
  }
  
  void set_elbow_params(byte arg1, byte arg2) {
    //do this!
  }

  void set_wrist_params(byte arg1, byte arg2) {
    //do this
  }

  void set_hand_params(byte arg1, byte arg2) {
    //do this!
  }
};

#endif
