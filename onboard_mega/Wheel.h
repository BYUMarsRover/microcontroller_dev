#ifndef WHEEL_H
#define WHEEL_H

class Wheel {
public:
  Wheel(int){
    this->set_speed = 0;
    this->actual_speed = 0;
    this->dir = true;
    this->error = false;
    this->enable = false;
  }
  ~Wheel(){}
  
  void updateFeedbackData() {
    this->actual_speed = convertToRpm(analogRead(RIGHT_FRONT_WHEEL_ACTUAL_SPEED));
    this->error = digitalRead(RIGHT_FRONT_WHEEL_ERROR);
  }





  byte set_speed;
  byte actual_speed
  bool dir;
  bool error;
  bool enable;
};


#endif
