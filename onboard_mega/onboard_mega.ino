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

void setup() {
  setPinModes();
  Wire.begin(I2C_ADDRESS);
  Wire.onReceive(receiveHandler);
  Wire.onRequest(requestHandler);
  Serial.begin(9600);
  delay(500);
  Serial.println("starting...");
}

void loop() {
  if (writeWheelParams) {
    wheels.writeParams();
    writeWheelParams = false;
  }
  if (write_arm_params) {
    arm.write_params();
    write_arm_params = false;
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
  digitalWrite(13, HIGH);
  int preamble = Wire.read();
  Serial.println(preamble);
  delay(500);
  switch(preamble) {
    case 1: setWheelParams(); break;
    case 2: setArmParams(); flushWire(); break;
    default: flushWire(); break;
  }
  digitalWrite(13, LOW);
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
  Serial.println("setWheelParams called");
  if (Wire.available() == 12) { 
    for (int i = 0; i < NUM_WHEELS; i++) {
      wheels.wheelList[i].set_speed = Wire.read();
      wheels.wheelList[i].dir = Wire.read();
    }
    writeWheelParams = true;
  } else {
    flushWire();
    Serial.println("wrong number of bytes sent in: setWheelParams");
  }
}

void setArmParams() {
  digitalWrite(LED_BUILTIN, HIGH);
  if (Wire.available() == 9) {
    arm.turret_high = Wire.read();
    arm.turret_low = Wire.read();
    arm.shoulder_high = Wire.read();
    arm.shoulder_low = Wire.read();
    arm.elbow_high = Wire.read();
    arm.elbow_low = Wire.read();
    arm.wrist_speed = Wire.read();
    arm.wrist_dir = Wire.read();
    arm.hand_dir = Wire.read();
    write_arm_params = true; 
  }
  else {
    //maybe notify someone that this failed????
    flushWire();
  }
  digitalWrite(LED_BUILTIN, LOW);
}


void setPinModes() {
  pinMode(RIGHT_FRONT_WHEEL_SET_SPEED, OUTPUT);
  pinMode(RIGHT_FRONT_WHEEL_DIR, OUTPUT);
  pinMode(RIGHT_FRONT_WHEEL_ENABLE, OUTPUT);
  pinMode(RIGHT_FRONT_WHEEL_ACTUAL_SPEED, INPUT);
  pinMode(RIGHT_FRONT_WHEEL_ERROR, INPUT_PULLUP);
  
  pinMode(RIGHT_MIDDLE_WHEEL_SET_SPEED, OUTPUT);
  pinMode(RIGHT_MIDDLE_WHEEL_DIR, OUTPUT);
  pinMode(RIGHT_MIDDLE_WHEEL_ENABLE, OUTPUT);
  pinMode(RIGHT_MIDDLE_WHEEL_ACTUAL_SPEED, INPUT);
  pinMode(RIGHT_MIDDLE_WHEEL_ERROR, INPUT_PULLUP);
  
  pinMode(RIGHT_REAR_WHEEL_SET_SPEED, OUTPUT);
  pinMode(RIGHT_REAR_WHEEL_DIR, OUTPUT);
  pinMode(RIGHT_REAR_WHEEL_ENABLE, OUTPUT);
  pinMode(RIGHT_REAR_WHEEL_ACTUAL_SPEED, INPUT);
  pinMode(RIGHT_REAR_WHEEL_ERROR, INPUT_PULLUP);
  
  pinMode(LEFT_FRONT_WHEEL_SET_SPEED, OUTPUT);
  pinMode(LEFT_FRONT_WHEEL_DIR, OUTPUT);
  pinMode(LEFT_FRONT_WHEEL_ENABLE, OUTPUT);
  pinMode(LEFT_FRONT_WHEEL_ACTUAL_SPEED, INPUT);
  pinMode(LEFT_FRONT_WHEEL_ERROR, INPUT_PULLUP);
  
  pinMode(LEFT_MIDDLE_WHEEL_SET_SPEED, OUTPUT);
  pinMode(LEFT_MIDDLE_WHEEL_DIR, OUTPUT);
  pinMode(LEFT_MIDDLE_WHEEL_ENABLE, OUTPUT);
  pinMode(LEFT_MIDDLE_WHEEL_ACTUAL_SPEED, INPUT);
  pinMode(LEFT_MIDDLE_WHEEL_ERROR, INPUT_PULLUP);
  
  pinMode(LEFT_REAR_WHEEL_SET_SPEED, OUTPUT);
  pinMode(LEFT_REAR_WHEEL_DIR, OUTPUT);
  pinMode(LEFT_REAR_WHEEL_ENABLE, OUTPUT);
  pinMode(LEFT_REAR_WHEEL_ACTUAL_SPEED, INPUT);
  pinMode(LEFT_REAR_WHEEL_ERROR, INPUT_PULLUP);
}
