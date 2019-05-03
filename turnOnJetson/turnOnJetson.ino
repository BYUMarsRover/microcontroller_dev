void setup() {
  pinMode(12,OUTPUT);
  pinMode(11,OUTPUT);
  pinMode(10,OUTPUT);
  
  digitalWrite(12,HIGH);
  digitalWrite(11,LOW);
  digitalWrite(10,LOW);

  delay(2000);  
  digitalWrite(10,HIGH);
  delay(40);
  digitalWrite(10,LOW);
}

void loop() {
}
