#include <Wire.h>

byte T = 0;
void setup() {
  Wire.begin(9);
  Serial.begin(9600);
  Wire.onReceive(onReceive_);
  Wire.onRequest(onRequest_);

}

void onReceive_() {
//  Serial.println("\nReceive\n");
  if (Wire.available() ==1) {
    Wire.read();
    return;
  }
  while(Wire.available()) {
    Serial.println(Wire.read());
  }
  Serial.println("\n");
}

void onRequest_() {
//  Serial.println("\nRequest\n");
  /*while(Wire.available()) {
    Serial.println(Wire.read());
  }*/
//  Serial.println("\nend of transmission\n");
  Wire.write(T);
  Wire.write(T);
  Wire.write(T);
  Wire.write(T);
  Wire.write(T);
  Wire.write(T);
  Wire.write(T);
  Wire.write(T);
  Wire.write(T);
  Wire.write(T);
  Wire.write(T);
  Wire.write(T);
  Wire.write(T);
  Wire.write(T);
  Wire.write(T);
  Wire.write(T);
  Wire.write(T);
  Wire.write(T);
}

void loop() {
  // put your main code here, to run repeatedly:

}
