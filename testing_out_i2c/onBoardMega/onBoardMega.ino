#include <Wire.h>

void setup() {
  Wire.begin(8);
  Serial.begin(9600);
  Wire.onReceive(receiveBytes);
}

void loop() {
  

}

void receiveBytes() {
  while (Wire.available()) {
    Serial.println((int)Wire.read());
  }
  Serial.println("------------------\n");
}
