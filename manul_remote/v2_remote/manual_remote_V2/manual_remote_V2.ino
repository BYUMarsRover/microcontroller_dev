#include <Wire.h>

const int OFF = 0;
const int ON = 1;
const int FORWARD = 1;
const int BACKWARD = 0;
const int STOP = 0;

const int RIGHT_FORWARD_SWITCH = 2;
const int RIGHT_BACK_SWITCH = 5;
const int LEFT_FORWARD_SWITCH = 4;
const int LEFT_BACK_SWITCH = 3;
const int SLOW_MODE_SWITCH = 6;
const int HIGH_SPEED = 180;
const int LOW_SPEED = 115;

class WheelParams {
public:
  byte leftFrontSpeed = 0x0;
  byte leftFrontDir = 0x0;
  byte leftMiddleSpeed = 0x0;
  byte leftMiddleDir = 0x0;
  byte leftBackSpeed = 0x0;
  byte leftBackDir = 0x0;
  byte rightBackSpeed = 0x0;
  byte rightBackDir = 0x0;
  byte rightMiddleSpeed = 0x0;
  byte rightMiddleDir = 0x0;
  byte rightFrontSpeed = 0x0;
  byte rightFrontDir = 0x0;
};

int set_speed = 0;

int RIGHT_FORWARD_STATE = OFF;
int RIGHT_BACK_STATE = OFF;
int LEFT_FORWARD_STATE = OFF;
int LEFT_BACK_STATE = OFF;
int SLOW_MODE = OFF;

int PREV_RIGHT_FORWARD_STATE = OFF;
int PREV_RIGHT_BACK_STATE = OFF;
int PREV_LEFT_FORWARD_STATE = OFF;
int PREV_LEFT_BACK_STATE = OFF;
int PREV_SLOW_MODE = OFF;

WheelParams wheelParams;

void setup() {
  Wire.begin();
//  Serial.begin(9600);
  pinMode(RIGHT_FORWARD_SWITCH, INPUT);
  pinMode(RIGHT_BACK_SWITCH, INPUT);
  pinMode(LEFT_FORWARD_SWITCH, INPUT);
  pinMode(LEFT_BACK_SWITCH, INPUT);
  pinMode(SLOW_MODE_SWITCH, INPUT);
}

void loop() {

  PREV_RIGHT_FORWARD_STATE = RIGHT_FORWARD_STATE;
  PREV_RIGHT_BACK_STATE = RIGHT_BACK_STATE;
  PREV_LEFT_FORWARD_STATE = LEFT_FORWARD_STATE;
  PREV_LEFT_BACK_STATE = LEFT_BACK_STATE;
  PREV_SLOW_MODE = SLOW_MODE;

  RIGHT_FORWARD_STATE = digitalRead(RIGHT_FORWARD_SWITCH);
  RIGHT_BACK_STATE = digitalRead(RIGHT_BACK_SWITCH);
  LEFT_FORWARD_STATE = digitalRead(LEFT_FORWARD_SWITCH);
  LEFT_BACK_STATE = digitalRead(LEFT_BACK_SWITCH);
  SLOW_MODE = digitalRead(SLOW_MODE_SWITCH);
  
  if (isStateChanged()) {

    if (SLOW_MODE == ON) {
      set_speed = LOW_SPEED;
    } else {
      set_speed = HIGH_SPEED;
    }
  
    if (RIGHT_FORWARD_STATE == ON) {
      setWheelsSpeed('r', set_speed);
      setWheelsDir('r', FORWARD);
    } else if (RIGHT_BACK_STATE == ON) {
      setWheelsSpeed('r', set_speed);
      setWheelsDir('r', BACKWARD);
    } else {
      setWheelsSpeed('r', STOP);
    }

    if (LEFT_FORWARD_STATE == ON) {
      setWheelsSpeed('l', set_speed);
      setWheelsDir('l', FORWARD);
    } else if (LEFT_BACK_STATE == ON) {
      setWheelsSpeed('l', set_speed);
      setWheelsDir('l', BACKWARD);
    } else {
      setWheelsSpeed('l', STOP);
    }

    writeParams();
    
  }
  delay(10);
}

bool isStateChanged() {
  return (PREV_RIGHT_FORWARD_STATE != RIGHT_FORWARD_STATE ||
          PREV_RIGHT_BACK_STATE != RIGHT_BACK_STATE ||
          PREV_LEFT_FORWARD_STATE != LEFT_FORWARD_STATE ||
          PREV_LEFT_BACK_STATE != LEFT_BACK_STATE ||
          PREV_SLOW_MODE != SLOW_MODE);
}

void setWheelsSpeed(const char& side, const int& speed_) {
  switch (side) {
  case 'l' :  {
    wheelParams.leftFrontSpeed = speed_;
    wheelParams.leftMiddleSpeed = speed_;
    wheelParams.leftBackSpeed = speed_;
  }; break;
  case 'r' : {
    wheelParams.rightFrontSpeed = speed_;
    wheelParams.rightMiddleSpeed = speed_;
    wheelParams.rightBackSpeed = speed_;
  }; break;
  }
}

void setWheelsDir(const char& side, const int& dir) {
  switch (side) {
  case 'l' :  {
    wheelParams.leftFrontDir = dir;
    wheelParams.leftMiddleDir = dir;
    wheelParams.leftBackDir = dir;
  }; break;
  case 'r' : {
    wheelParams.rightFrontDir = dir;
    wheelParams.rightMiddleDir = dir;
    wheelParams.rightBackDir = dir;
  }; break;
  }
}

void writeParams() {
  Wire.beginTransmission(8);
  Wire.write(1);
  Wire.write(wheelParams.leftFrontSpeed);
  Wire.write(wheelParams.leftFrontDir);
  Wire.write(wheelParams.leftMiddleSpeed);
  Wire.write(wheelParams.leftMiddleDir);
  Wire.write(wheelParams.leftBackSpeed);
  Wire.write(wheelParams.leftBackDir);
  Wire.write(wheelParams.rightBackSpeed);
  Wire.write(wheelParams.rightBackDir);
  Wire.write(wheelParams.rightMiddleSpeed);
  Wire.write(wheelParams.rightMiddleDir);
  Wire.write(wheelParams.rightFrontSpeed);
  Wire.write(wheelParams.rightFrontDir);
  Wire.endTransmission();
}
