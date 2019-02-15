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
  
  Turret turret;
  Fergelli shoulder;
  Fergelli elbow;
  Wrist wrist;
  Hand hand;
};

