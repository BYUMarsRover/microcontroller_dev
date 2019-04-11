#include <Wire.h>
#include "PD_SM.h"
#include "VEL_SM.h"

//define preambles to decide which state we are in
#define ARM_PREAMBLE 2
#define SCIENCE_PREAMBLE 3

static enum masterState{POSITION, VELOCITY} currState;//master state for position or velocity control

void setup() {
  Serial.begin (9600);          
  pinMode (LED_BUILTIN, OUTPUT);
  Wire.begin(TURRET_ADDRESS);
  Wire.onReceive(onReceive);
  pinMode(ARM_TURRET_FB, INPUT);

  PD_init();
}  
  
void loop() {
  PD_tick();
  delay(50);
}
  
void onReceive(int howMany) 
{
  digitalWrite(LED_BUILTIN,HIGH);
  byte preamble = Wire.read();
  if (preamble == ARM_PREAMBLE) {//read two more bytes for turret position high and low
    turret_high = Wire.read();
    turret_low = Wire.read();
  }
  else if (preamble == SCIENCE_PREAMBLE) {//read one more byte for turret direction
    direction = Wire.read();
  }
  else {//flush wire
    while(Wire.available()) {
      Wire.read();
    } 
  }
  digitalWrite(LED_BUILTIN,LOW);
}
  
