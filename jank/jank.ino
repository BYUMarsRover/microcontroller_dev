char PIN = A4;
#define PIN_ A4

void setup() {
  pinMode(PIN_, INPUT);
  Serial.begin(9600);
}

void loop() {
  Serial.println(digitalRead(PIN));
  delay(250);
}
