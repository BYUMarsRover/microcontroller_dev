#include <Wire.h>

  enum state{STOP, CW, CCW} currState;
  void state_machine_run();
  void stop_turret();
  void turret_cw();
  void turret_ccw();
  void motor_speed();
  
  byte turret_high=0;
  byte turret_low=40;

  uint16_t DesiredAngle;
  float SensorValue;
  int Speed = 40;                                   //sets speed for motor 127-0, 127 max speed, 0 min speed
  float error;
  float AngVelocity;
  float PrevValue;

  const int kp=1;
  const int kd=.717;
  const int Max=60;
  const int Min=30;
  const int BUFFER = 3;                             // degrees in wiggle room BUFFER*2
  const int ARM_TURRET_FB = A0;                     // Absolute Encoder pin
  const int TURRET_ADDRESS = 15;

void setup() {
  // put your setup code here, to run once:
  Serial.begin (9600);          
  pinMode (13, OUTPUT);
  Wire.begin(TURRET_ADDRESS);
  Wire.onReceive(getTurretParams);
  pinMode(ARM_TURRET_FB, INPUT);

  SensorValue = analogRead(ARM_TURRET_FB);          // gives number from 100-917 
  SensorValue = SensorValue*.43-61.51;              // changes to number from 0-360 
  PrevValue=SensorValue;
}  
  
void loop() {
  // put your main code here, to run repeatedly:
  DesiredAngle = (turret_high << 8) | turret_low;    //reading in from I2C the desired angle
  SensorValue = analogRead(ARM_TURRET_FB);          // gives number from 100-917 
  SensorValue = SensorValue*.43-61.51;              // changes to number from 0-360 
  error = DesiredAngle-SensorValue;                 // error will determine needed speed for motor
  AngVelocity= (SensorValue-PrevValue)/2;           // used in Speed calculation
  PrevValue=SensorValue;
  state_machine_run();
  delay(50);
}

void motor_speed()
{
  Speed=error*kp-AngVelocity*kd;
  if (Speed > Max)
  {
    Speed=Max;
  }
  else if (Speed < Min)
  {
    Speed = Min;
  }
}

void state_machine_run()
{
  switch(currState)
  {
    case STOP:
      if (error < -BUFFER) 
      {                  
      turret_cw();
      currState=CW;
      digitalWrite(13,HIGH);
      }
      else if (error > BUFFER)
      {
      turret_ccw(); 
      currState=CCW;
      digitalWrite(13,HIGH);
      }
    break;
    
    case CW:
      if (error > -BUFFER) 
      {
      stop_turret();
      currState=STOP;
      digitalWrite(13,LOW);
      }
      else
      {
      turret_cw();
      }
    break;
    
    case CCW:
      if (error < BUFFER)
      {
      stop_turret();
      currState=STOP;
      digitalWrite(13,LOW);
      } 
      else
      {
      turret_ccw();
      }
    break;
  }
}
  
void stop_turret()
{
  Serial.write(0xFF); 
}
  
void turret_ccw()
{
  //turn the clock-wise 
  motor_speed();
  unsigned char turnCCW[] = {0xE0, Speed};              
  Serial.write(turnCCW, sizeof(turnCCW));                // Might need to switch "CCW" and "CW" code depending on which way the motor is facing (right is current orientation)
}
  
void turret_cw()
{
  motor_speed();
  unsigned char turnCW[] = {0xE1,Speed};        // 0=stop, 127=fullspeed
  Serial.write(turnCW, sizeof(turnCW));
}
  
void getTurretParams(int howMany) 
{
  digitalWrite(13,HIGH);
  if(Wire.available()>= 2) 
  {
    turret_high = Wire.read();
    turret_low = Wire.read();
  }
  digitalWrite(13,LOW);
}
  
