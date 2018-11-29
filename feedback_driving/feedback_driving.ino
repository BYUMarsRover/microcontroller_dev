#define RIGHT_PWM 6
#define LEFT_PWM 5
#define RIGHT_ENABLE 52
#define LEFT_ENABLE 50
#define RIGHT_DIRECTION 46
#define LEFT_DIRECTION 44
#define RIGHT_RPM_FB A3 //fb = feedback
#define LEFT_RPM_FB A1 //fb = feedback
#define RIGHT_AMPS_FB A4 //fb = feedback
#define LEFT_AMPS_FB A0 //fb = feedback
#define CONTROLLER_POT A8
#define CW HIGH //clockwise
#define CCW LOW //counter-clockwise
#define ERROR_STATE 40

int control_pot_value = 0;
bool in_error_state = false;
bool right_enable = false;
bool left_enable = false;

void setup() {
  set_pin_modes();
  Serial.begin(9600);
}

void loop() {
  if (in_error_state) cycle_enable_pin();
  update_control_pot_value();
  drive_motors();
  delay(10);
  check_for_error_state();
}

void check_for_error_state() {
  if (left_enable && get_abs_RPM(LEFT_RPM_FB) < 40) in_error_state = true;
  else if (right_enable && get_abs_RPM(RIGHT_RPM_FB) < 40) in_error_state = true;
  else Serial.println("no error detected");
}

void cycle_enable_pin() {
  Serial.println("error detected");
  digitalWrite(RIGHT_ENABLE,LOW);
  digitalWrite(LEFT_ENABLE,LOW);
  delay(250);
  digitalWrite(RIGHT_ENABLE,HIGH);
  digitalWrite(LEFT_ENABLE,HIGH);
  delay(250);
  in_error_state = false;
  Serial.println("error state cleared");
}

float get_abs_RPM(int pin) {
  float result = map(analogRead(pin),0,1023,-6000,9000);
  if (result >= 0) return result;
  return result * -1;
}



void drive_motors() {
  if (control_pot_value < 500) {
    digitalWrite(RIGHT_DIRECTION, CCW);
    digitalWrite(LEFT_DIRECTION, CW);
    analogWrite(RIGHT_PWM, map(control_pot_value,0,499,227,30));
    analogWrite(LEFT_PWM, map(control_pot_value,0,499,227,30));
    digitalWrite(RIGHT_ENABLE, HIGH);
    digitalWrite(LEFT_ENABLE, HIGH);
    right_enable = true;
    left_enable = true;
  } else if (control_pot_value > 525) {
    digitalWrite(RIGHT_DIRECTION, CW);
    digitalWrite(LEFT_DIRECTION, CCW);
    analogWrite(RIGHT_PWM, map(control_pot_value,526,1023,30,227));
    analogWrite(LEFT_PWM, map(control_pot_value,526,1023,30,227));
    digitalWrite(RIGHT_ENABLE, HIGH);
    digitalWrite(LEFT_ENABLE, HIGH);
    right_enable = true;
    left_enable = true;
  } else {
    digitalWrite(RIGHT_ENABLE, LOW);
    digitalWrite(LEFT_ENABLE, LOW);
    right_enable = false;
    left_enable = false;
    delay(200);//this delay prevents the motors reversing direction too quickly and prevents damage. 
  }
}

void update_control_pot_value() {
  control_pot_value = analogRead(CONTROLLER_POT); //0-1023
}

void set_pin_modes() {
  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(LEFT_PWM, OUTPUT);
  pinMode(RIGHT_ENABLE, OUTPUT);
  pinMode(LEFT_ENABLE, OUTPUT);
  pinMode(RIGHT_DIRECTION, OUTPUT);
  pinMode(LEFT_DIRECTION, OUTPUT);
  pinMode(RIGHT_RPM_FB, INPUT);
  pinMode(LEFT_RPM_FB, INPUT);
  pinMode(RIGHT_AMPS_FB, INPUT);
  pinMode(LEFT_AMPS_FB, INPUT);
  pinMode(CONTROLLER_POT, INPUT);
  pinMode(ERROR_STATE, INPUT);
}
