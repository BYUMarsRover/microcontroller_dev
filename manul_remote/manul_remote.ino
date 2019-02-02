#include <Wire.h>

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

// constants won't change. They're used here to set pin numbers:
const int LB = 3;     // the number of the pushbutton pin
const int RB = 2;

// variables will change:
int LBState = 0;         // variable for reading the pushbutton status
int RBState = 0;
int speed_ = 155;
int prevLB = 0;
int prevRB = 0;
WheelParams wheelParams;

void setup() {
  pinMode(LB, INPUT);
  pinMode(RB, INPUT);
  Wire.begin();
  Serial.begin(9600);
}

void loop() {
  prevLB = LBState;
  prevRB = RBState;
  LBState = digitalRead(LB);
  RBState = digitalRead(RB);

  if (prevLB != LBState || prevRB != RBState) {
  
    if (LBState == 1) {
      setWheelsSpeed('l', speed_);
    } else {
      setWheelsSpeed('l', 0);
    }
    if (RBState == 1) {
      setWheelsSpeed('r', speed_);
    } else {
      setWheelsSpeed('r', 0);
    }
    writeParams();
    
  }
  delay(10);
}

void setWheelsSpeed(const char& side, const int& speed_) {
  switch (side) {
  case 'l' :  {
    wheelParams.leftFrontSpeed = speed_;
    wheelParams.leftFrontDir = 0x1;
    wheelParams.leftMiddleSpeed = speed_;
    wheelParams.leftMiddleDir = 0x1;
    wheelParams.leftBackSpeed = speed_;
    wheelParams.leftBackDir = 0x1;
  }; break;
  case 'r' : {
    wheelParams.rightFrontSpeed = speed_;
    wheelParams.rightFrontDir = 0x1;
    wheelParams.rightMiddleSpeed = speed_;
    wheelParams.rightMiddleDir = 0x1;
    wheelParams.rightBackSpeed = speed_;
    wheelParams.rightBackDir = 0x1;
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
