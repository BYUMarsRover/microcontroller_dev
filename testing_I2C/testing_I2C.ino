#include <Wire.h>

#define WIRE 8

byte T = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin(WIRE);
  Wire.onReceive(gettingBytes);
  Wire.onRequest(sendingBytes);
}

void loop() {

}

void gettingBytes() {
  Serial.println("getting bytes!");
  while (Wire.available()) {
    Serial.print("from jetson: ");
    Serial.println(Wire.read());
  }
  Serial.println("end of transmission.\n");
}

void sendingBytes() {
//  Wire.write(T);
//  Wire.write(T);
//  Wire.write(T);
//  Wire.write(T);
//  Wire.write(T);
//  Wire.write(T);
//  Wire.write(T);
//  Wire.write(T);
//  Wire.write(T);
//  Wire.write(T);
//  Wire.write(T);
//  Wire.write(T);
//  Wire.write(T);
//  Wire.write(T);
//  Wire.write(T);
//  Wire.write(T);
//  Wire.write(T);
//  Wire.write(T);
}
