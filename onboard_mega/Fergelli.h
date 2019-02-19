#ifndef FERGELLI_H
#define FERGELLI_H

//fergelli is the brand of linear actuator we are using for the shoulder and elbow
#include <JrkG2.h>

using namespace std;

class Fergelli {
public:
  Fergelli(const int& i2cAddress) {
      jrk(i2cAddress);
      pos_high_byte = 0;
      pos_low_byte = 0;
  }
  ~Fergelli(){}
  void set_params(byte pos_high_byte, byte pos_low_byte) {
    //push params to linear actuator
    pos_full = (pos_high_byte << 8) | pos_low_byte; 
    jrk.setTarget(pos_full);    
  }
private:
  JrkG2I2C jrk;
};

