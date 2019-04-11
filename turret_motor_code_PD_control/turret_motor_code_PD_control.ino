#include <Wire.h>
#include "PD_SM.h"
#include "VEL_SM.h"

//define preambles to decide which state we are in
#define ARM_PREAMBLE 2
#define SCIENCE_PREAMBLE 3

#define MAX_TIME 2000
#define PERIOD 50

static enum masterState{WAIT, POSITION, VELOCITY} currState;//master state for position or velocity control
long prevTime = 0;//used to kill the turret if we don't receive a command within x seconds
long tickCount = 0;

void setup() {
  Serial.begin (9600);          
  pinMode (LED_BUILTIN, OUTPUT);
  Wire.begin(TURRET_ADDRESS);
  Wire.onReceive(onReceive);
  pinMode(ARM_TURRET_FB, INPUT);
  prevTime = millis();

  PD_init();
  VEL_init();
  masterSM_init();
}  
  
void loop() {
  if (millis() >= tickCount+PERIOD) {//tick the SM every <period> milli seconds
    masterSM_tick();
    tickCount = millis();
  }
}

void masterSM_init() {
  currState = WAIT;
}

void masterSM_tick() {
  if (currState == POSITION) {
    PD_tick();
  }
  else if (currState == VELOCITY) {
    if (millis() < prevTime+MAX_TIME) {//only tick if we've received a message recently
     VEL_tick(); 
    }
    else {//otherwise stop the turret
      stop_turret();
    }
  }
}
  
void onReceive(int howMany) 
{
  digitalWrite(LED_BUILTIN,HIGH);
  byte preamble = Wire.read();
  if (preamble == ARM_PREAMBLE) {//read two more bytes for turret position high and low
    turret_high = Wire.read();
    turret_low = Wire.read();
    currState = POSITION;
  }
  else if (preamble == SCIENCE_PREAMBLE) {//read one more byte for turret direction
    direction = Wire.read();
    currState = VELOCITY;
    prevTime = millis();//save the last time we got a message for turret velocity
  }
  else {//flush wire
    while(Wire.available()) {
      Wire.read();
    } 
  }
  digitalWrite(LED_BUILTIN,LOW);
}
  
