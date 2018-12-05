#include <Wire.h>
#include "Globals.h"
#include "Wheels.h"

using namespace std;

Wheels wheels;
byte T = 0;

void setup() {
  Wire.begin(I2C_ADDRESS);
  Serial.begin(9600);
  Wire.onReceive(receiveHandler);
  Wire.onRequest(requestHandler);
}

void loop() {
}

void receiveHandler(int byteCount) {
  switch(Wire.read()) {
  case 1: setWheelParams(); break;
  case 2: setArmParams(); break;
  case I2C_ADDRESS: Serial.println("address read"); Wire.read(); break;
  default: Serial.println("default called" + Wire.read()); flushWire(); break;
  }
}

void flushWire() {
  Serial.println("flushing wire");
  while(Wire.available()) {
    Serial.println(Wire.read());
  }
}

void requestHandler() {
  Wire.write(T);
  Wire.write(T);
  Wire.write(T);
  Wire.write(T);
  Wire.write(T);
  Wire.write(T);
  Wire.write(T);
  Wire.write(T);
  Wire.write(T);
  Wire.write(T);
  Wire.write(T);
  Wire.write(T);
  Wire.write(T);
  Wire.write(T);
  Wire.write(T);
  Wire.write(T);
  Wire.write(T);
  Wire.write(T);
}

void setWheelParams() {
  if (Wire.available() != 12) errorOccurred("in readWheelParams(): wrong number of bytes sent!");
  
  wheels.right_front_speed = Wire.read();
  wheels.right_front_dir = Wire.read();
  wheels.right_middle_speed = Wire.read();
  wheels.right_middle_dir = Wire.read();
  wheels.right_rear_speed = Wire.read();
  wheels.right_rear_dir = Wire.read();
  
  wheels.left_front_speed = Wire.read();
  wheels.left_front_dir = Wire.read();
  wheels.left_middle_speed = Wire.read();
  wheels.left_middle_dir = Wire.read();
  wheels.left_rear_speed = Wire.read();
  wheels.left_rear_dir = Wire.read();

  wheels.applyParams();
  
}

void setArmParams() {
  
}

void errorOccurred(char* errorMessage) {
  Serial.print("error: ");
  Serial.println(errorMessage);
//  exit(1);
}
