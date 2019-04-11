#include "Arduino.h"
#ifndef __PD_CONTROL__
#define __PD_CONTROL__
static enum PD_state{PD_STOP, PD_CW, PD_CCW} pdState;//state for PD controller

void PD_init();
void PD_tick();

byte turret_high=0;
byte turret_low=40;

static uint16_t DesiredAngle;
static float SensorValue;
static int Speed = 40;                                   //sets speed for motor 127-0, 127 max speed, 0 min speed
static float error;
static float AngVelocity;
static float PrevValue;

static const int kp=1;
static const int kd=.717;
static const int Max=60;
static const int Min=30;
static const int BUFFER = 3;                             // degrees in wiggle room BUFFER*2
#define ARM_TURRET_FB A0                    // Absolute Encoder pin
#define TURRET_ADDRESS 15

int saturate_speed(int speed);
void turret_cw(int speed);
void turret_ccw(int speed);
void stop_turret();

//init function
void PD_init() {
  SensorValue = analogRead(ARM_TURRET_FB);          // gives number from 100-917 
  SensorValue = SensorValue*.43-61.51;              // changes to number from 0-360 
  PrevValue=SensorValue;
  Speed = 40;
}

int saturate_speed(int speed)
{  
  if (speed > Max) {
    speed=Max;
  }
  else if (speed < Min) {
    speed = Min;
  }
}

void PD_tick()
{
  DesiredAngle = (turret_high << 8) | turret_low;    //reading in from I2C the desired angle
  SensorValue = analogRead(ARM_TURRET_FB);          // gives number from 100-917 
  SensorValue = SensorValue*.43-61.51;              // changes to number from 0-360 
  error = DesiredAngle-SensorValue;                 // error will determine needed speed for motor
  AngVelocity= (SensorValue-PrevValue)/2;           // used in Speed calculation
  PrevValue=SensorValue;

  Speed=error*kp-AngVelocity*kd;//update speed of motor with the PD control
  
  switch(pdState) {
    case PD_STOP:
      if (error < -BUFFER) {            
        turret_cw(Speed);
        pdState=PD_CW;
        digitalWrite(13,HIGH);
      }
      else if (error > BUFFER) {
        turret_ccw(Speed); 
        pdState=PD_CCW;
        digitalWrite(13,HIGH);
      }
    break;
    
    case PD_CW:
      if (error > -BUFFER) {
        stop_turret();
        pdState=PD_STOP;
        digitalWrite(13,LOW);
      }
      else {
        turret_cw(Speed);
      }
    break;
    
    case PD_CCW:
      if (error < BUFFER) {
        stop_turret();
        pdState=PD_STOP;
        digitalWrite(13,LOW);
      } 
      else {
        turret_ccw(Speed);
      }
    break;
  }
}

void stop_turret()
{
  Serial.write(0xFF); 
}
  
void turret_ccw(int speed)
{
  //turn the clock-wise 
  speed = saturate_speed(Speed);
  unsigned char turnCCW[] = {0xE0, speed};              
  Serial.write(turnCCW, sizeof(turnCCW));                // Might need to switch "CCW" and "CW" code depending on which way the motor is facing (right is current orientation)
}
  
void turret_cw(int speed)
{
  speed = saturate_speed(Speed);
  unsigned char turnCW[] = {0xE1,speed};        // 0=stop, 127=fullspeed
  Serial.write(turnCW, sizeof(turnCW));
}
#endif
