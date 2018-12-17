//COM 6

#define ON_BOARD_MEGA 8
#include <Wire.h>

void setup() {
  Wire.begin();
  Serial.begin(9600);
}

void loop() {
  Wire.beginTransmission(8);
  Wire.write(1); // preamble
  Wire.write(255);
  Wire.write(true);
  Wire.write(0);
  Wire.write(true);
  Wire.write(0);
  Wire.write(true);
  Wire.write(0);
  Wire.write(true);
  Wire.write(0);
  Wire.write(true);
  Wire.write(0);
  Wire.write(true);
  Wire.endTransmission();
  delay(250);

  Wire.requestFrom(8, 12);
  while(Wire.available()) {
    char c = Wire.read();
    Serial.print(c);
  }
  Serial.print('\n');
}
