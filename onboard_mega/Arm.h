#ifndef ARM_H
#define ARM_H
#include "Turret.h"
#include "Fergelli.h"
#include "Wrist.h"
#include "Hand.h"

using namespace std;

class Arm {
public:
  Arm(){}
  ~Arm(){}
  
  Turret turret();
  Fergelli shoulder(SHOULDER_ADDRESS);
  Fergelli elbow(ELBOW_ADDRESS);
  Wrist wrist(WRIST_ADDRESS;
  Hand hand();
};

