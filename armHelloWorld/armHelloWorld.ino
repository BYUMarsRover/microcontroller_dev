// This example shows how to control the Jrk G2 over I2C by
// sending Set Target commands.
//
// The Jrk's input mode must be set to "Serial/I2C/USB".  The
// Jrk's device number must be set to its default value of 11.
//
// Please see https://github.com/pololu/jrk-g2-arduino for
// details on how to make the connections between the Arduino and
// the Jrk G2.

#include <JrkG2.h>

JrkG2I2C wrist(12);
JrkG2I2C elbow(13);
JrkG2I2C shoulder(11);

//function to drive the speed and dir of the wrist
//true = fwd
void setWristSpeed(byte speed, bool dir) {
  if (dir) {//go forward
    wrist.setTargetLowResFwd(speed);
  }
  else {
    wrist.setTargetLowResRev(speed);
  }
}

void stopWrist() {
  wrist.stopMotor();
}

void setup()
{
  // Set up I2C.
  Wire.begin();
  Serial.begin(9600);
  elbow.setTarget(1000);
  shoulder.setTarget(1000);
  
  setWristSpeed(50,true);
  delay(1000);
  setWristSpeed(50,false);
  delay(1000);
  stopWrist();
  delay(1000);
}

void loop()
{  
}
