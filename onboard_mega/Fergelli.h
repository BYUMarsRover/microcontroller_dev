#ifndef FERGELLI_H
#define FERGELLI_H

// Jacob

//fergelli is the brand of linear actuator we are using for the shoulder and elbow

using namespace std;

class Fergelli {
public:
  Fergelli() {
      pos_high_byte = 0;
      pos_low_byte = 0;
  }
  ~Fergelli(){}
  void set_params(byte pos_high_byte, byte pos_low_byte) {
    this->pos_high_byte = pos_high_byte;
    this->pos_low_byte = pos_low_byte;
    //push params to linear actuator
  }
private:
  byte pos_high_byte;
  byte pos_low_byte;
};

