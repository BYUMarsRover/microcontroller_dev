#include <Wire.h>
#include "Globals.h"
#include "Wheels.h"

using namespace std;

Wheels wheels;
bool writeWheelParams = true;

void setup() {
  setPins();
  Wire.begin(I2C_ADDRESS);
  Wire.onReceive(receiveHandler);
  Wire.onRequest(requestHandler);
  Serial.begin(9600);
}

void loop() {
  wheels.updateFeedbackData();
  if (writeWheelParams) {
    wheels.writeParams();
    writeWheelParams = false;
  }
  wheels.clearErrorStates();
  delay(125);
}

void writeWheelFeedback() {
  Wire.write(wheels.right_front_actual_speed);
  Wire.write(wheels.right_front_error);

  Wire.write(wheels.right_middle_actual_speed);
  Wire.write(wheels.right_middle_error);

  Wire.write(wheels.right_rear_actual_speed);
  Wire.write(wheels.right_rear_error);

  Wire.write(wheels.left_front_actual_speed);
  Wire.write(wheels.left_front_error);

  Wire.write(wheels.left_middle_actual_speed);
  Wire.write(wheels.left_middle_error);

  Wire.write(wheels.left_rear_actual_speed);
  Wire.write(wheels.left_rear_error);
}

void receiveHandler(int byteCount) {
  byte preambleByte = Wire.read();
  switch(preambleByte) {
    case 1: setWheelParams(); break;
    case 2: setArmParams(); break;
    case I2C_ADDRESS: flushWire(); break;
    default: flushWire(); break;
  }
}

void flushWire() {
  while(Wire.available()) {
    Wire.read();
  }
}

void requestHandler() {
  writeWheelFeedback();
}

void setWheelParams() {
  if (Wire.available() == 12) { 
    wheels.right_front_set_speed = Wire.read();
    wheels.right_front_dir = Wire.read();
    wheels.right_middle_set_speed = Wire.read();
    wheels.right_middle_dir = Wire.read();
    wheels.right_rear_set_speed = Wire.read();
    wheels.right_rear_dir = Wire.read();
    
    wheels.left_front_set_speed = Wire.read();
    wheels.left_front_dir = Wire.read();
    wheels.left_middle_set_speed = Wire.read();
    wheels.left_middle_dir = Wire.read();
    wheels.left_rear_set_speed = Wire.read();
    wheels.left_rear_dir = Wire.read();
  
    writeWheelParams = true;
  } else {
    Serial.println("wrong number of bytes in setWheelParams()");
    exit(1);
  }
  
}

void setArmParams() {
  
}


void setPins() {
  pinMode(LEFT_FRONT_WHEEL_SET_SPEED, OUTPUT);
  pinMode(LEFT_FRONT_WHEEL_DIR, OUTPUT);
  pinMode(LEFT_FRONT_WHEEL_ENABLE, OUTPUT);
  pinMode(LEFT_FRONT_WHEEL_ACTUAL_SPEED, INPUT);
  pinMode(LEFT_FRONT_WHEEL_ERROR, INPUT);
  
  pinMode(LEFT_MIDDLE_WHEEL_SET_SPEED, OUTPUT);
  pinMode(LEFT_MIDDLE_WHEEL_DIR, OUTPUT);
  pinMode(LEFT_MIDDLE_WHEEL_ENABLE, OUTPUT);
  pinMode(LEFT_MIDDLE_WHEEL_ACTUAL_SPEED, INPUT);
  pinMode(LEFT_MIDDLE_WHEEL_ERROR, INPUT);
  
  pinMode(LEFT_REAR_WHEEL_SET_SPEED, OUTPUT);
  pinMode(LEFT_REAR_WHEEL_DIR, OUTPUT);
  pinMode(LEFT_REAR_WHEEL_ENABLE, OUTPUT);
  pinMode(LEFT_REAR_WHEEL_ACTUAL_SPEED, INPUT);
  pinMode(LEFT_REAR_WHEEL_ERROR, INPUT);
  
  pinMode(LEFT_FRONT_WHEEL_SET_SPEED, OUTPUT);
  pinMode(LEFT_FRONT_WHEEL_DIR, OUTPUT);
  pinMode(LEFT_FRONT_WHEEL_ENABLE, OUTPUT);
  pinMode(LEFT_FRONT_WHEEL_ACTUAL_SPEED, INPUT);
  pinMode(LEFT_FRONT_WHEEL_ERROR, INPUT);
  
  pinMode(LEFT_MIDDLE_WHEEL_SET_SPEED, OUTPUT);
  pinMode(LEFT_MIDDLE_WHEEL_DIR, OUTPUT);
  pinMode(LEFT_MIDDLE_WHEEL_ENABLE, OUTPUT);
  pinMode(LEFT_MIDDLE_WHEEL_ACTUAL_SPEED, INPUT);
  pinMode(LEFT_MIDDLE_WHEEL_ERROR, INPUT);
  
  pinMode(LEFT_REAR_WHEEL_SET_SPEED, OUTPUT);
  pinMode(LEFT_REAR_WHEEL_DIR, OUTPUT);
  pinMode(LEFT_REAR_WHEEL_ENABLE, OUTPUT);
  pinMode(LEFT_REAR_WHEEL_ACTUAL_SPEED, INPUT);
  pinMode(LEFT_REAR_WHEEL_ERROR, INPUT);
}
