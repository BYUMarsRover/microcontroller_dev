void setup() {
pinMode(2,OUTPUT);
pinMode(4,OUTPUT);
pinMode(7,OUTPUT);
digitalWrite(7,HIGH);
}

void loop() {
  // put your main code here, to run repeatedly:
digitalWrite(2,HIGH);
digitalWrite(4,LOW);
delay(3000);
digitalWrite(4,HIGH);
digitalWrite(2,LOW);
delay(3000);

}
