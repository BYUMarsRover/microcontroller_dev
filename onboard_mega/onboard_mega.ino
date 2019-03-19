//COM 5

#include <Wire.h>
#include "Globals.h"
#include "Wheels.h"
#include "Arm.h"

using namespace std;

Wheels wheels;
Arm arm;
bool writeWheelParams = true;
bool write_arm_params = true;
bool receivedi2c = false; 

void setup() {
  setPinModes();
  Wire.begin(I2C_ADDRESS);
  Wire.onReceive(receiveHandler);
  Wire.onRequest(requestHandler);
//  Serial.begin(9600);
//  arm.init_turret();
  delay(500);
  digitalWrite(POWER_INDICATOR, HIGH); // so we can tell the Mega is powered from breakoutboard.
//  Serial.println("starting...");
}

void loop() {
  if (write_arm_params) {
    arm.write_params();
    write_arm_params = false;
  }
  if (writeWheelParams) {
    wheels.writeParams();
    writeWheelParams = false;
  }

  checkClearErrorStates();
  wheels.updateFeedbackData();
}

void checkClearErrorStates() {
  for (int i = 0; i < NUM_WHEELS; i++) {
    if (wheels.wheelList[i].error) {
      digitalWrite(wheels.wheelList[i].enable_pin, false);
      delay(10);
      digitalWrite(wheels.wheelList[i].enable_pin, true);
    }
  }
}

void receiveHandler(int byteCount) {
  switch(Wire.read()) {
    case 1: setWheelParams(); break;
    case 2: setArmParams(); break;
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
//    Wire.write((uint8_t*)&wheels.wheelList[i].actual_speed, 2);
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

void setArmParams() {
//  Serial.println("setArm");
//  Serial.println(Wire.available());
  if (Wire.available() == 10) {
    arm.turret_high = Wire.read();
    arm.turret_low = Wire.read();
    arm.shoulder_high = Wire.read();
    arm.shoulder_low = Wire.read();
    arm.elbow_high = Wire.read();
    arm.elbow_low = Wire.read();
    arm.wrist_speed = Wire.read();
    arm.wrist_dir = Wire.read();
    arm.hand_speed = Wire.read();
    arm.hand_dir = Wire.read();
    write_arm_params = true; 
//    arm.printVals();
  }
  else {
    //maybe notify someone that this failed????
    flushWire();
  }
}


void setPinModes() {
  pinMode(POWER_INDICATOR, OUTPUT);
  
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
