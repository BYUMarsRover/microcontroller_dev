#ifndef WHEEL_PARAMS_H
#define WHEEL_PARAMS_H

using namespace std;

class WheelParams {
public:
  WheelParams(){}
  ~WheelParams(){}
  WheelParams(byte speed_, byte direction_) {
    this->speed_ = speed_;
    this->direction_ = direction_; 
  }
private:
  byte speed_;
  byte direction_;
};

#endif
