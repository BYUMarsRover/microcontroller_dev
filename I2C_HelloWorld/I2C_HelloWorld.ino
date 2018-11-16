#include <Wire.h>

void setup() {
  Wire.begin(8);                // join i2c bus with address #8
  Wire.onReceive(receiveEvent); // register event
  Serial.begin(9600);           // start serial for output
}

void loop() {
  delay(100);
}

void receiveEvent(int howMany) {
  while (Wire.available()) { // loop through all but the last    
    byte c = Wire.read(); // receive byte as a character    
    Serial.print(c);    

    if (c == 25) {
      //do something      
    }
  }  
}
