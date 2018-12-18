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
  Wire.write(100);
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
  int count = 0;
  char message[600];
  while(Wire.available()) {
    char message[100];
    uint8_t actual_speed = Wire.read();
    bool error = Wire.read();
    sprintf(message, "count:%d -> actual_speed = %d, error = %s\n", count, actual_speed, error ? "True" : "False");
    Serial.print(message);
    count++;
  }
  Serial.print('\n');
}
