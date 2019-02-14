#ifndef WRIST_H
#define WRIST_H

using namespace std;

class Wrist {
public:
  Wrist(){
    this->speed = 0;
    this->dir = 0;  
  }
  ~Wrist(){}
  void set_params(byte speed, byte dir) {
    this->speed = speed;
    this->dir = dir;
    //push params to wrist!
  }
private:
  byte speed;
  byte dir;
};

