

const int OFF = 0;
const int ON = 1;

const int RIGHT_FORWARD_SWITCH = 2;
const int RIGHT_BACK_SWITCH = 5;
const int LEFT_FORWARD_SWITCH = 4;
const int LEFT_BACK_SWITCH = 3;
const int SLOW_MODE_SWITCH = 6;


void setup() {
  Serial.begin(9600);
  pinMode(RIGHT_FORWARD_SWITCH, INPUT);
  pinMode(RIGHT_BACK_SWITCH, INPUT);
  pinMode(LEFT_FORWARD_SWITCH, INPUT);
  pinMode(LEFT_BACK_SWITCH, INPUT);
  pinMode(SLOW_MODE_SWITCH, INPUT);
}

int RIGHT_FORWARD_STATE = OFF;
int RIGHT_BACK_STATE = OFF;
int LEFT_FORWARD_STATE = OFF;
int LEFT_BACK_STATE = OFF;
int SLOW_MODE = OFF;

void loop() {

  RIGHT_FORWARD_STATE = digitalRead(RIGHT_FORWARD_SWITCH);
  RIGHT_BACK_STATE = digitalRead(RIGHT_BACK_SWITCH);
  LEFT_FORWARD_STATE = digitalRead(LEFT_FORWARD_SWITCH);
  LEFT_BACK_STATE = digitalRead(LEFT_BACK_SWITCH);
  SLOW_MODE = digitalRead(SLOW_MODE_SWITCH);

  Serial.println(RIGHT_FORWARD_STATE);
  Serial.println(RIGHT_BACK_STATE);
  Serial.println(LEFT_FORWARD_STATE);
  Serial.println(LEFT_BACK_STATE);
  Serial.println(SLOW_MODE);
  Serial.println("-------------------------\n");
  
  

//  if (RIGHT_FORWARD_STATE == HIGH || RIGHT_BACK_STATE == HIGH || B3State == HIGH || LEFT_BACK_STATE == HIGH) {
//    digitalWrite(LED_BUILTIN, HIGH);
//  } else {
//    digitalWrite(LED_BUILTIN, LOW);
//  }
  
  delay(500);
}
