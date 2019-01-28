#include <Wire.h>

class WheelParams {
public:
  byte preamble = 0x1;
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
const int LB = 2;     // the number of the pushbutton pin
const int RB = 3;
const int ledPin =  13;      // the number of the LED pin

// variables will change:
int LBState = 0;         // variable for reading the pushbutton status
int RBState = 0;
WheelParams wheelParams;

void setup() {
  // initialize the LED pin as an output:
  pinMode(ledPin, OUTPUT);
  // initialize the pushbutton pin as an input:
  pinMode(LB, INPUT);
  pinMode(RB, INPUT);
  Wire.begin(); // join i2c bus (address optional for master)
}

void loop() {
  // read the state of the pushbutton value:
  LBState = digitalRead(LB);
  RBState = digitalRead(RB);

  // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
  if (LBState == HIGH) {
    setWheelsDrive('l');
  } else {
    setWheelsStop('l');
  }
  if (RBState == HIGH) {
    setWheelsDrive('r');
  } else {
    setWheelsStop('r');
  }
  writeParams();
  delay(10);
}

void setWheelsDrive(char side) {
  switch (side) {
  case 'l' :  {
    wheelParams.leftFrontSpeed = 0x125;
    wheelParams.leftFrontDir = 0x1;
    wheelParams.leftMiddleSpeed = 0x125;
    wheelParams.leftMiddleDir = 0x1;
    wheelParams.leftBackSpeed = 0x125;
    wheelParams.leftBackDir = 0x1;
    ;break;
  }
  case 'r' : {
    wheelParams.rightFrontSpeed = 0x125;
    wheelParams.rightFrontDir = 0x1;
    wheelParams.rightMiddleSpeed = 0x125;
    wheelParams.rightMiddleDir = 0x1;
    wheelParams.rightBackSpeed = 0x125;
    wheelParams.rightBackDir = 0x1;
    break;
  }
  default :;
  }
  
}

void setWheelsStop(char side) {
  switch (side) {
  case 'l' : {
    wheelParams.leftFrontSpeed = 0x0;
    wheelParams.leftFrontDir = 0x1;
    wheelParams.leftMiddleSpeed = 0x0;
    wheelParams.leftMiddleDir = 0x1;
    wheelParams.leftBackSpeed = 0x0;
    wheelParams.leftBackDir = 0x1;
    break;
  }
  case 'r' : {
    wheelParams.rightFrontSpeed = 0x0;
    wheelParams.rightFrontDir = 0x1;
    wheelParams.rightMiddleSpeed = 0x0;
    wheelParams.rightMiddleDir = 0x1;
    wheelParams.rightBackSpeed = 0x0;
    wheelParams.rightBackDir = 0x1;
    ; break;
  }
  default : ; 
  }
}

void writeParams() {
  Wire.beginTransmission(8);
  Wire.write(wheelParams.preamble);
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
