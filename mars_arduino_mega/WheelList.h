#ifndef WHEEL_LIST_H
#define WHEEL_LIST_H

#include "Wheel.h"
#include "Pinout.h"
#include "Globals.h"

using namespace std;

class WheelList {
public:
  WheelList(){}
  WheelList(const int capacity) {
    this->wheelList = new Wheel[capacity];
    this->capacity = capacity;
    this->size_ = 0;
    initWheels();
  }
  ~WheelList() {
    delete []wheelList;
  }

  void initWheels() {
    for (int i = 0; i < capacity; i++) {
      this->wheelList[i] = Wheel(pinout.ithWheelPWMPin(i), pinout.ithWheelDirectionPin(i), pinout.ithWheelEnablePin(i));
    }
  }
  
  void append(Wheel wheel) {
    wheelList[size_] = wheel;
    size_++;
  }
  Wheel at(const int index) {
    return wheelList[index];
  }
  void pushNewParams(WheelParamsList newParams) {
    for (int i = 0; i < NUM_WHEELS; i++) {
      wheelList[i].pushWheelParams(newParams.at(i));
    }
  }
private:
  Pinout pinout;
  Wheel* wheelList;
  int capacity;
  int size_;
};

#endif
