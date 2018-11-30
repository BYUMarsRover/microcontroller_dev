#ifndef PINOUT_H
#define PINOUT_H

class Pinout {
public:
  Pinout(){}
  ~Pinout(){}
  int ithWheelMotorPWMPin(const int ithMotor) {
    switch(ithMotor) {
    case 1: return RIGHT_FRONT_WHEEL_MOTOR_PWM; break;
    case 2: return RIGHT_MIDDLE_WHEEL_MOTOR_PWM; break;
    case 3: return RIGHT_BACK_WHEEL_MOTOR_PWM; break;
    case 4: return LEFT_FRONT_WHEEL_MOTOR_PWM; break;
    case 5: return LEFT_MIDDLE_WHEEL_MOTOR_PWM; break;
    case 6: return LEFT_BACK_WHEEL_MOTOR_PWM; break;
    }
  }
  int ithWheelMotorDirectionPin(const int ithMotor) {
    switch(ithMotor) {
    case 1: return RIGHT_FRONT_WHEEL_MOTOR_DIRECTION;
    case 2: return RIGHT_MIDDLE_WHEEL_MOTOR_DIRECTION;
    case 3: return RIGHT_BACK_WHEEL_MOTOR_DIRECTION;
    case 4: return LEFT_FRONT_WHEEL_MOTOR_DIRECTION;
    case 5: return LEFT_MIDDLE_WHEEL_MOTOR_DIRECTION;
    case 6: return LEFT_BACK_WHEEL_MOTOR_DIRECTION;
    }
  }
  int ithWheelMotorEnablePin(const int ithMotor) {
    switch(ithMotor) {
    case 1: return RIGHT_FRONT_WHEEL_MOTOR_ENABLE;
    case 2: return RIGHT_MIDDLE_WHEEL_MOTOR_ENABLE;
    case 3: return RIGHT_BACK_WHEEL_MOTOR_ENABLE;
    case 4: return LEFT_FRONT_WHEEL_MOTOR_ENABLE;
    case 5: return LEFT_MIDDLE_WHEEL_MOTOR_ENABLE;
    case 6: return LEFT_BACK_WHEEL_MOTOR_ENABLE;
    }
  }
private:
  const int RIGHT_FRONT_WHEEL_MOTOR_PWM = 2;
  const int RIGHT_FRONT_WHEEL_MOTOR_DIRECTION = 22;
  const int RIGHT_FRONT_WHEEL_MOTOR_ENABLE = 23;
  const int RIGHT_MIDDLE_WHEEL_MOTOR_PWM = 3;
  const int RIGHT_MIDDLE_WHEEL_MOTOR_DIRECTION = 24;
  const int RIGHT_MIDDLE_WHEEL_MOTOR_ENABLE = 5;
  const int RIGHT_BACK_WHEEL_MOTOR_PWM = 4;
  const int RIGHT_BACK_WHEEL_MOTOR_DIRECTION = 26;
  const int RIGHT_BACK_WHEEL_MOTOR_ENABLE = 27;
  const int LEFT_FRONT_WHEEL_MOTOR_PWM = 5;
  const int LEFT_FRONT_WHEEL_MOTOR_DIRECTION = 28;
  const int LEFT_FRONT_WHEEL_MOTOR_ENABLE = 29;
  const int LEFT_MIDDLE_WHEEL_MOTOR_PWM = 6;
  const int LEFT_MIDDLE_WHEEL_MOTOR_DIRECTION = 30;
  const int LEFT_MIDDLE_WHEEL_MOTOR_ENABLE = 31;
  const int LEFT_BACK_WHEEL_MOTOR_PWM = 7;
  const int LEFT_BACK_WHEEL_MOTOR_DIRECTION = 32;
  const int LEFT_BACK_WHEEL_MOTOR_ENABLE = 33;
};

#endif
