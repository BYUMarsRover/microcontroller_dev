#ifndef HAND_H
#define HAND_H

using namespace std;

class Hand {
public:
  Hand(){
    this->speed = 0;
    this->dir = 0;  
  }
  ~Hand(){}
  void set_params(byte speed, byte dir) {
    this->speed = speed;
    this->dir = dir;
    //push params to hand 
  }
  byte get_speed() { return this->speed }
  byte get_dir() { return this->dir }
private:
  byte speed;
  byte dir;
};

