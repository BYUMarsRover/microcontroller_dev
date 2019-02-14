#ifndef TURRET_H
#define TURRET_H

using namespace std;

class Turret {
public:
  Turret(){}
  ~Turret(){}
  void set_motor_params(byte dir, byte speed);
private:
  byte dir;
  byte speed;
};

