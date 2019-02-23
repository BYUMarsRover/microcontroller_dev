#include "Shoe.h"
#include <Wire.h>

const uint16_t SHOULDER_MAX = 2260;
const uint16_t SHOULDER_MIN = 150;
const uint16_t ELBOW_MAX = 2250;
const uint16_t ELBOW_MIN = 190;
 
using namespace std;

class Arm_Params {
public:
  ArmParams(int waste) {
    turret_speed = 0;
    turret_dir = 0;
    shoulder_high = 0;
    shoulder_low = 0;
    elbow_high = 0;
    elbow_low = 0;
    wrist_speed = 0;
    wrist_dir = 0;
    hand_dir = 0;
    empty = 0;
  }
  uint8_t turret_speed;
  uint8_t turret_dir;
  uint8_t shoulder_high;
  uint8_t shoulder_low;
  uint8_t elbow_high;
  uint8_t elbow_low;
  uint8_t wrist_speed;
  uint8_t wrist_dir;
  uint8_t hand_dir;
  uint8_t empty;
};

Arm_Params arm_p_max;
Arm_Params arm_p_min;


void setup() {

  arm_p_max.shoulder_high = SHOULDER_MAX >> 8;
  arm_p_max.shoulder_low = SHOULDER_MAX;
  arm_p_max.elbow_high = ELBOW_MAX >> 8;
  arm_p_max.elbow_low = ELBOW_MAX;

  arm_p_min.shoulder_high = SHOULDER_MIN >> 8;
  arm_p_min.shoulder_low = SHOULDER_MIN;
  arm_p_min.elbow_high = ELBOW_MIN >> 8;
  arm_p_min.elbow_low = ELBOW_MIN;
  Wire.begin(); // join i2c bus (address optional for master)
  delay(100);
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  Wire.beginTransmission(8); // transmit to device #8
  Wire.write(2);
  Wire.write(arm_p_max.turret_speed);
  Wire.write(arm_p_max.turret_dir);
  Wire.write(arm_p_max.shoulder_high);
  Wire.write(arm_p_max.shoulder_low);
  Wire.write(arm_p_max.elbow_high);
  Wire.write(arm_p_max.elbow_low);
  Wire.write(arm_p_max.wrist_speed);
  Wire.write(arm_p_max.wrist_dir);
  Wire.write(arm_p_max.hand_dir);
  Wire.endTransmission(); 
  digitalWrite(LED_BUILTIN, LOW);
  delay(20000);

    digitalWrite(LED_BUILTIN, HIGH);
  Wire.beginTransmission(8); // transmit to device #8
  Wire.write(2);
  Wire.write(arm_p_min.turret_speed);
  Wire.write(arm_p_min.turret_dir);
  Wire.write(arm_p_min.shoulder_high);
  Wire.write(arm_p_min.shoulder_low);
  Wire.write(arm_p_min.elbow_high);
  Wire.write(arm_p_min.elbow_low);
  Wire.write(arm_p_min.wrist_speed);
  Wire.write(arm_p_min.wrist_dir);
  Wire.write(arm_p_min.hand_dir);
  Wire.endTransmission(); 
  digitalWrite(LED_BUILTIN, LOW);
  delay(20000);
}
