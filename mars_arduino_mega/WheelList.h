#ifndef WHEEL_LIST_H
#define WHEEL_LIST_H
#include "Wheel.h"

using namespace std;

class WheelList {
public:
  WheelList(){}
  WheelList(const int capacity) {
    this->motorList = new Wheel[capacity];
    this->capacity = capacity;
    this->size_ = 0;
  }
  ~WheelList() {
    delete []motorList;
  }
  void append(Wheel wheel) {
    motorList[size_] = wheel;
    size_++;
  }
  Wheel at(const int index) {
    return motorList[index];
  }
  void pushNewParams(WheelParamsList newParams) {
    
  }
private:
  Wheel* motorList;
  int capacity;
  int size_;
};

#endif
