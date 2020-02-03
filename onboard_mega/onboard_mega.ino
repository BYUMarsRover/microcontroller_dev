//COM 5

#include <Wire.h>
#include "Globals.h"
#include "Wheels.h"
//#include "Arm.h"

using namespace std;

Wheels wheels;

bool writeWheelParams = true;

void setup() {
  setPinModes();
  Wire.begin(I2C_ADDRESS);
  Wire.onReceive(receiveHandler);
  Wire.onRequest(requestHandler);
//  wheels.updateFeedbackData();
//  checkClearErrorStates();  
  
//  powerJetso/n();//turn on the jetson  
  digitalWrite(POWER_INDICATOR, HIGH);
}

void loop() {
  if (writeWheelParams) {
    wheels.writeParams();
    writeWheelParams = false;
  }
  checkClearErrorStates();
//  wheels.updateFeedbackData();
}

void checkClearErrorStates() {
  for (int i = 0; i < NUM_WHEELS; i++) {
    if (wheels.wheelList[i].error) {
      digitalWrite(wheels.wheelList[i].enable_pin, false);
      delay(10);
      digitalWrite(wheels.wheelList[i].enable_pin, true);
      delay(10);
    }
  }
}

void receiveHandler(int byteCount) {
  switch(Wire.read()) {
    case 1: setWheelParams(); break;
    default: flushWire(); break;
  }
}

void flushWire() {
  while(Wire.available()) {
    Wire.read();
  }
}

void requestHandler() {
  for (int i = 0; i < NUM_WHEELS; i++) {
    Wire.write(wheels.wheelList[i].actual_speed);
    Wire.write(wheels.wheelList[i].error);
  }
}

void setWheelParams() {
  if (Wire.available() == 12) { 
    for (int i = 0; i < NUM_WHEELS; i++) {
      wheels.wheelList[i].set_speed = Wire.read();
      wheels.wheelList[i].dir = Wire.read();
    }
    writeWheelParams = true;
  } else {
    flushWire();
  }  
}

//Warning: this function is blocking 
//void powerJetson() {
//  digitalWrite(JETSON_POWER_ON, HIGH);
//  delay(40);
//  digitalWrite(JETSON_POWER_ON,LOW);
//}


void setPinModes() {
  //misc
  pinMode(POWER_INDICATOR, OUTPUT);
//  pinMode(JETSON_POWER_ON, OUTPUT);

  //wheels
  pinMode(RIGHT_FRONT_WHEEL_SET_SPEED, OUTPUT);
  pinMode(RIGHT_FRONT_WHEEL_DIR, OUTPUT);
  pinMode(RIGHT_FRONT_WHEEL_ENABLE, OUTPUT);
  pinMode(RIGHT_FRONT_WHEEL_ERROR, INPUT_PULLUP);
  
  pinMode(RIGHT_MIDDLE_WHEEL_SET_SPEED, OUTPUT);
  pinMode(RIGHT_MIDDLE_WHEEL_DIR, OUTPUT);
  pinMode(RIGHT_MIDDLE_WHEEL_ENABLE, OUTPUT);
  pinMode(RIGHT_MIDDLE_WHEEL_ERROR, INPUT_PULLUP);
  
  pinMode(RIGHT_REAR_WHEEL_SET_SPEED, OUTPUT);
  pinMode(RIGHT_REAR_WHEEL_DIR, OUTPUT);
  pinMode(RIGHT_REAR_WHEEL_ENABLE, OUTPUT);
  pinMode(RIGHT_REAR_WHEEL_ERROR, INPUT_PULLUP);
  
  pinMode(LEFT_FRONT_WHEEL_SET_SPEED, OUTPUT);
  pinMode(LEFT_FRONT_WHEEL_DIR, OUTPUT);
  pinMode(LEFT_FRONT_WHEEL_ENABLE, OUTPUT);
  pinMode(LEFT_FRONT_WHEEL_ERROR, INPUT_PULLUP);
  
  pinMode(LEFT_MIDDLE_WHEEL_SET_SPEED, OUTPUT);
  pinMode(LEFT_MIDDLE_WHEEL_DIR, OUTPUT);
  pinMode(LEFT_MIDDLE_WHEEL_ENABLE, OUTPUT);
  pinMode(LEFT_MIDDLE_WHEEL_ERROR, INPUT_PULLUP);
  
  pinMode(LEFT_REAR_WHEEL_SET_SPEED, OUTPUT);
  pinMode(LEFT_REAR_WHEEL_DIR, OUTPUT);
  pinMode(LEFT_REAR_WHEEL_ENABLE, OUTPUT);
  pinMode(LEFT_REAR_WHEEL_ERROR, INPUT_PULLUP);  
}
