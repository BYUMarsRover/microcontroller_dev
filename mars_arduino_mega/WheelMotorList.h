#ifndef WHEEL_MOTOR_LIST
#define WHEEL_MOTOR_LIST
#include "WheelMotor.h"

using namespace std;

class WheelMotorList {
public:
  WheelMotorList(){}
  WheelMotorList(const int capacity) {
    this->motorList = new WheelMotor[capacity];
    this->capacity = capacity;
    this->size_ = 0;
  }
  ~WheelMotorList() {
    delete []motorList;
  }
  void append(WheelMotor wheelMotor) {
    motorList[size_] = wheelMotor;
    size_++;
  }
  WheelMotor at(const int index) {
    return motorList[index];
  }
private:
  WheelMotor* motorList;
  int capacity;
  int size_;
};

#endif
