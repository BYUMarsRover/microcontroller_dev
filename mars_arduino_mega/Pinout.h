#ifndef PINOUT_H
#define PINOUT_H

class Pinout {
public:
  Pinout(){}
  ~Pinout(){}
  int ithWheelPWMPin(const int ithWheel) {
    switch(ithWheel) {
    case 1: return RIGHT_FRONT_WHEEL_PWM; break;
    case 2: return RIGHT_MIDDLE_WHEEL_PWM; break;
    case 3: return RIGHT_BACK_WHEEL_PWM; break;
    case 4: return LEFT_FRONT_WHEEL_PWM; break;
    case 5: return LEFT_MIDDLE_WHEEL_PWM; break;
    case 6: return LEFT_BACK_WHEEL_PWM; break;
    }
  }
  int ithWheelDirectionPin(const int ithWheel) {
    switch(ithWheel) {
    case 1: return RIGHT_FRONT_WHEEL_DIRECTION;
    case 2: return RIGHT_MIDDLE_WHEEL_DIRECTION;
    case 3: return RIGHT_BACK_WHEEL_DIRECTION;
    case 4: return LEFT_FRONT_WHEEL_DIRECTION;
    case 5: return LEFT_MIDDLE_WHEEL_DIRECTION;
    case 6: return LEFT_BACK_WHEEL_DIRECTION;
    }
  }
  int ithWheelEnablePin(const int ithWheel) {
    switch(ithWheel) {
    case 1: return RIGHT_FRONT_WHEEL_ENABLE;
    case 2: return RIGHT_MIDDLE_WHEEL_ENABLE;
    case 3: return RIGHT_BACK_WHEEL_ENABLE;
    case 4: return LEFT_FRONT_WHEEL_ENABLE;
    case 5: return LEFT_MIDDLE_WHEEL_ENABLE;
    case 6: return LEFT_BACK_WHEEL_ENABLE;
    }
  }
private:
  const int RIGHT_FRONT_WHEEL_PWM = 2;
  const int RIGHT_FRONT_WHEEL_DIRECTION = 22;
  const int RIGHT_FRONT_WHEEL_ENABLE = 23;
  const int RIGHT_MIDDLE_WHEEL_PWM = 3;
  const int RIGHT_MIDDLE_WHEEL_DIRECTION = 24;
  const int RIGHT_MIDDLE_WHEEL_ENABLE = 5;
  const int RIGHT_BACK_WHEEL_PWM = 4;
  const int RIGHT_BACK_WHEEL_DIRECTION = 26;
  const int RIGHT_BACK_WHEEL_ENABLE = 27;
  const int LEFT_FRONT_WHEEL_PWM = 5;
  const int LEFT_FRONT_WHEEL_DIRECTION = 28;
  const int LEFT_FRONT_WHEEL_ENABLE = 29;
  const int LEFT_MIDDLE_WHEEL_PWM = 6;
  const int LEFT_MIDDLE_WHEEL_DIRECTION = 30;
  const int LEFT_MIDDLE_WHEEL_ENABLE = 31;
  const int LEFT_BACK_WHEEL_PWM = 7;
  const int LEFT_BACK_WHEEL_DIRECTION = 32;
  const int LEFT_BACK_WHEEL_ENABLE = 33;
};

#endif
