#ifndef WHEEL_MOTOR
#define WHEEL_MOTOR
#include <Arduino.h>
using namespace std;

class WheelMotor {
public:
  WheelMotor(const int PWM_PIN, const int DIRECTION_PIN, const int ENABLE_PIN) {
    this->PWM_PIN = PWM_PIN;
    this->DIRECTION_PIN = DIRECTION_PIN;
    this->ENABLE_PIN = ENABLE_PIN;
  }
  ~WheelMotor(){}
  void setSpeed_() {
    analogWrite(
  }
  void setDirection() {
    
  }
private:
  int PWM_PIN;
  int DIRECTION_PIN;
  int ENABLE_PIN;
  byte speed_;
  byte direction_;
  byte enable
};

#endif 
