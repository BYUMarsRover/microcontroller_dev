void setup() {
  // put your setup code here, to run once:
  pinMode(4, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(5, INPUT_PULLUP);
  digitalWrite(4, HIGH);
  analogWrite(3, 250);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (!digitalRead(5)) {
    digitalWrite(4, LOW);
    Serial.println("Button Pressed");
    
  }
  else {
    digitalWrite(4, HIGH);
    //analogWrite(4,0);
    Serial.println("Button Not Pressed");
  }
}
