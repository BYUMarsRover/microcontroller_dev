#ifndef WHEEL_MOTOR_PARAMETERS
#define WHEEL_MOTOR_PARAMETERS

using namespace std;

class WheelMotorParameters {
public:
  WheelMotorParameters(){}
  ~WheelMotorParameters(){}
  WheelMotorParameters(byte speed_, byte direction_) {
    this->speed_ = speed_;
    this->direction_ = direction_; 
  }
private:
  byte speed_;
  byte direction_;
};

#endif
