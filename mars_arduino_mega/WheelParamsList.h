#ifndef WHEEL_PARAMS_LIST_H
#define WHEEL_PARAMS_LIST_H
#include "WheelParams.h"

using namespace std;

class WheelParamsList {
public:
  WheelParamsList(const int capacity) {
    this->parameters = new WheelParams[size_];
    this->capacity = capacity;
    this->size_ = 0;
  }
  ~WheelParamsList() {
    delete []parameters;
  }
  void append(WheelParams newParams) {
    parameters[size_] = newParams;
    size_++;
  }
  char* toString() {
    return "hello world";
  }
private:
  int capacity;
  int size_;
  WheelParams* parameters;
};

#endif
