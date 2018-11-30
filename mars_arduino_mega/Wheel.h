#ifndef WHEEL_H
#define WHEEL_H
#include <Arduino.h>
using namespace std;

class Wheel {
public:
  Wheel(){}
  Wheel(const int PWM_PIN, const int DIRECTION_PIN, const int ENABLE_PIN) {
    this->PWM_PIN = PWM_PIN;
    this->DIRECTION_PIN = DIRECTION_PIN;
    this->ENABLE_PIN = ENABLE_PIN;
  }
  ~Wheel(){}
  void pushWheelParams(WheelParams wheelParams) {
    this->speed_ = wheelParams.speed_;
    Serial.println("pushed a speed of: " + speed_);
    this->direction_ = wheelParams.direction_;
    Serial.println("pushed a direction of: " + direction_);
    writeParams();
  }
  void writeParams() {
    if (speed_ < 120) {
      //
    } else if (speed_ > 134) {
      
    } else {
      
    }
  }
private:
  int PWM_PIN;
  int DIRECTION_PIN;
  int ENABLE_PIN;
  byte speed_;
  byte direction_;
  byte enable;
};

#endif 
