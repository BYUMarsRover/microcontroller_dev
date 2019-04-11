//COM 6

#define ON_BOARD_MEGA 8
#define POT A0
#define CONTROLLER 8
#include <Wire.h>

const bool USE_POT = false;
uint8_t set_speed = 125;

void setup() {
  pinMode(CONTROLLER, OUTPUT);
  Wire.begin();
  Serial.begin(9600);
}

void loop() {
  if (USE_POT) {
    set_speed = map(analogRead(POT),0,1023,0,255);
  }

  Wire.beginTransmission(8);
  Wire.write(1); // preamble
  Wire.write(set_speed);
  Wire.write(false);
//  Wire.write(set_speed);
//  Wire.write(false);
//  Wire.write(0);
//  Wire.write(true);
//  Wire.write(0);
//  Wire.write(true);
//  Wire.write(0);
//  Wire.write(true);
//  Wire.write(0);
//  Wire.write(true);
  Wire.endTransmission();
  delay(20);

//  Wire.requestFrom(8, 12);
//  char message[100];
//  uint8_t count = 0;
//  while(Wire.available()) {
//    uint8_t actual_speed = Wire.read();
//    bool error = Wire.read();
//    if (count == 0) {
//      digitalWrite(CONTROLLER, error ? HIGH : LOW);
//    }
//    sprintf(message, "motor %d: set_speed = %d, actual_speed = %d, error = %s", count, set_speed, actual_speed, error ? "True" : "False");
//    Serial.println(message); 
//    count++;
//  }
//  Serial.print('\n');

  delay(20);
}
