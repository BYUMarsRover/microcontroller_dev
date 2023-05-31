// These constants won't change. They're used to give names to the pins used:

#define laser_pin 22
#define clicker_direction_pin 34
#define clicker_enable_pin 36
#define clicker_limit_pin 20


void setup() {
  pinMode(laser_pin, OUTPUT);
  pinMode(clicker_direction_pin, OUTPUT);
  pinMode(clicker_enable_pin, OUTPUT);
  pinMode(clicker_limit_pin, INPUT);
  Serial.begin(9600);
}

void loop() {
  digitalWrite(clicker_direction_pin, HIGH);
  digitalWrite(clicker_enable_pin, HIGH);
//  delay(1000);
  int start_time = millis();
  while(digitalRead(clicker_limit_pin) == LOW);
  digitalWrite(clicker_direction_pin, LOW);
  digitalWrite(clicker_enable_pin, HIGH);
//  delay(1000);
  delay(min(millis() - start_time + 10, 2500));
//  Serial.println(digitalRead(clicker_limit_pin));
//  delay(1000);
}
