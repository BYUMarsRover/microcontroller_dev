#include <Wire.h>
#include "WheelParamsList.h"
#include "WheelParams.h"
#include "Pinout.h"
#include "Globals.h"
#include "WheelList.h"
#include "Wheel.h"

WheelList wheelList(NUM_WHEELS);

void setup() {
  Serial.begin(9600);
  Wire.begin(I2C_ADDRESS);
  Wire.onReceive(jetsonSentBytes);
  Wire.onRequest(jetsonRequestedBytes);
}

void loop() {
}

void jetsonSentBytes() {
  if (Wire.available()) {
    switch(Wire.read()) { // reads the preamble and switches accordingly
    case 1: wheelList.pushNewParams(readWheelParamList()); break;
    case 2: /*add arm controller here */; break;
    default: /* throw error here */; break;
    }
  }
}

WheelParamsList readWheelParamList() {
  WheelParamsList result(NUM_WHEELS);
  for (int i = 0; i < NUM_WHEELS; i++) {
    result.append(WheelParams(Wire.read(), Wire.read()));
  }
  return result;
}

void jetsonRequestedBytes() {
  
}
