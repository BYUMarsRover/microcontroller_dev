#ifndef ALL_WHEEL_MOTORS_PARAMETERS
#define ALL_WHEEL_MOTORS_PARAMETERS
#include "WheelMotorParameters.h"

using namespace std;

class AllWheelMotorsParameters {
public:
  AllWheelMotorsParameters(const int capacity) {
    this->parameters = new WheelMotorParameters[size_];
    this->capacity = capacity;
    this->size_ = 0;
  }
  ~AllWheelMotorsParameters() {
    delete []parameters;
  }
  void append(WheelMotorParameters newParams) {
    parameters[size_] = newParams;
    size_++;
  }
  char* toString() {
    return "hello world";
  }
private:
  int capacity;
  int size_;
  WheelMotorParameters* parameters;
};

#endif
