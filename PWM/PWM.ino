#define PWM 6
#define DIR 8
#define BLINK 13

void setup() {
  pinMode(PWM, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(BLINK, OUTPUT);
  
  digitalWrite(DIR, HIGH);
  analogWrite(PWM,255);

}

void loop() {
  digitalWrite(BLINK,HIGH);
  delay(1000);
  digitalWrite(BLINK,LOW);
  delay(1000);
}
