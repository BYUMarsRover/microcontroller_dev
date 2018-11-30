#include <Wire.h>
#include "AllWheelMotorsParameters.h"
#include "WheelMotorParameters.h"
#include "WheelControlHandler.h"
#include "FeedbackHandler.h"
#include "Pinout.h"

const int I2C_ADDRESS = 8;
const int NUM_WHEEL_MOTORS = 6;

WheelControlHandler wheelControlHandler;
FeedbackHandler feedbackHandler;

void setup() {
  Serial.begin(9600);
  Wire.begin(I2C_ADDRESS);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
}

void loop() {
}

void receiveEvent() {
  if (Wire.available()) {
    switch(Wire.read()) { // reads the preamble and switches accordingly
    case 1: wheelControlHandler.pushAllWheelMotorsParameters(getAllWheelMotorsParameters()); break;
    case 2: /*add arm controller here */; break;
    default: /* throw error here */; break;
    }
  }
}

AllWheelMotorsParameters getAllWheelMotorsParameters() {
  AllWheelMotorsParameters result(NUM_WHEEL_MOTORS);
  for (int i = 0; i < NUM_WHEEL_MOTORS; i++) {
    result.append(WheelMotorParameters(Wire.read(), Wire.read()));
  }
  return result;
}

void requestEvent() {
  
}
