/*
 * Written by Zack Alger zack@a7ger.com (918) 418-7982
 * February 25, 2019
 *     
 *     
 * INSTRUCTIONS FOR SWITCHING MODES FOUND HERE!
 *      
 *      
 * The controller has arm mode and wheel mode.
 * Each has 2 sub modes (for a total of 4 modes):
 * 
 * Arm Modes:
 * - shoulder/elbow
 * - forearm/hand
 * 
 * Wheel Modes:
 * - wheels slow
 * - wheels fast
 * 
 * The controller always starts in wheel mode.
 * To toggle between arm mode and wheel mode toggle the mode switch 2 times within 
 * the below set MODE_TOGGLE_INTERVAL (some value in milliseconds - recommend 1000 ms or 1 second)
 * 
 * Once in either arm or wheel mode the state of the mode switch (UP or DOWN) will determine which
 * sub-mode the controller will enter:
 * 
 * Arm Modes:
 * - shoulder/elbow   (DOWN)
 * - forearm/hand     (UP)
 * 
 * Wheel Modes:
 * - wheels slow      (DOWN)
 * - wheels fast      (UP)
 * 
 * Example: You turn on the rover and the controller mode switch is in the up position. The controller defaults to wheel mode and since the switch
 * is in the up position, the controller enters the wheels fast mode. If you want to slow down, toggle the mode switch to the down position 
 * and the controller will enter the wheels slow mode. If you want to speed up again toggle the switch back to up and you'll re-enter wheels
 * fast. If you are now ready to control the arm toggle the mode switch 2 times within 1 second (if the MODE_TOGGLE_INTERVAL is set to a value other 
 * than 1000 then adjust accordingly) and the controller will enter arm mode. At this point the mode xwitch would be in the up position as that's 
 * where it started before toggling twice to switch to arm mode and therefore you will be in forearm/elbow mode specifically (which is 1 of the 
 * sub-modes for arm mode). Then if you want to switch modes to control the shoulder/elbow just toggle the switch 1 time to the down position.
 * 
 * NOTE: if you toggle the switch 3 times and each of the toggle events are within the MODE_TOGGLE_INTERVAL of each other you will actually 
 * switch modes twice. This is likely undesireable and therefore I recommend being careful with how fast you switch between submodes within 
 * a super-mode.
 */




#include <Wire.h>

// These are used throughout the code in general to imporove readability
const int ON = 1;
const int OFF = 0;
const int FORWARD = 1;
const int BACKWARD = 0;
const int STOP = 0;
const int UP = 0;
const int DOWN = 1;

//the different modes the remote can be in
enum Mode {SHOULDER_ELBOW, FOREARM_HAND, WHEELS_FAST, WHEELS_SLOW};

// defaults to shoulder_elbow mode
Mode mode = SHOULDER_ELBOW;

// the interval given for switching modes. see instruction for switching modes below
const int MODE_TOGGLE_INTERVAL = 1000; // in millisecods

// used to detect the time elapsed between mode toggles, used for entering different modes
int last_mode_toggle_time = 0;


const int RIGHT_FORWARD_SWITCH = 2;
const int RIGHT_BACK_SWITCH = 5;
const int LEFT_FORWARD_SWITCH = 4;
const int LEFT_BACK_SWITCH = 3;
const int MODE_SWITCH = 6; 

//hard coded values for the fast and slow speeds toggeled by the mode switch when in wheel mode 0 is 0 and 255 is max speed
const int HIGH_SPEED = 180;
const int LOW_SPEED = 115;

// this remote tank drives the rover meaning all left wheels spin together and all right wheels spin together
// temporarily stores the calculated params to be written then writes them at the same time for smoother changes in params
class WheelParams {
public:
  uint8_t left_speed = 0;
  uint8_t left_dir = 0;
  uint8_t right_speed = 0;
  uint8_t right_dir = 0;
};

// this will be assigned dynamically based on the mode. The two choices are HIGH_SPEED and LOW_SPEED declared above
int desired_speed = 0;

// the differnet buttons and switches on the controller have the following states
int RIGHT_FORWARD_STATE = OFF;
int RIGHT_BACK_STATE = OFF;
int LEFT_FORWARD_STATE = OFF;
int LEFT_BACK_STATE = OFF;
int MODE_STATE = DOWN;

/*
* keeping track of previous states so as to only write new params when they have changed 
* helps reduce I2C bus traffic. However, if controller is unplugged accidentally during operation, the last
* written params will persist, aka the rover will continue driving if it was driving when the controller 
* disconnected.
*/
int PREV_RIGHT_FORWARD_STATE = OFF;
int PREV_RIGHT_BACK_STATE = OFF;
int PREV_LEFT_FORWARD_STATE = OFF;
int PREV_LEFT_BACK_STATE = OFF;
int PREV_MODE_STATE = DOWN;

WheelParams wheelParams;

void setup() {
  Wire.begin();
  Serial.begin(9600);
  set_pin_modes();
  delay(1000); // to allow all i2c to come online and prevent mode switching errors
}

void loop() {

  remember_previous_states();

  update_state();

  if (is_state_changed()) {
    update_mode();
    apply_state(); 
  }

//  
//  if (isStateChanged()) {
//
//    if (MODE_STATE == DOWN) {
//      desired_speed = LOW_SPEED;
//    } else {
//      desired_speed = HIGH_SPEED;
//    }
//  
//    if (RIGHT_FORWARD_STATE == ON) {
//      wheel_params.right_speed = desired_speed;
//      wheel_params.right_dir = FORWARD;
//    } else if (RIGHT_BACK_STATE == ON) {
//      wheel_params.right_speed = desired_speed;
//      wheel_params.right_dir = BACKWARD;
//    } else {
//      wheel_params.right_speed = STOP;
//    }
//
//    if (LEFT_FORWARD_STATE == ON) {
//      wheel_params.left_speed = desired_speed;
//      wheel_params.left_dir = FORWARD;
//    } else if (LEFT_BACK_STATE == ON) {
//      wheel_params.left_speed = desired_speed;
//      wheel_params.left_dir = BACKWARD;
//    } else {
//      wheel_params.left_speed = STOP;
//    }
//
//    writeParams();
//    
//  }
  delay(10);
}

void remember_previous_states() {
  PREV_RIGHT_FORWARD_STATE = RIGHT_FORWARD_STATE;
  PREV_RIGHT_BACK_STATE = RIGHT_BACK_STATE;
  PREV_LEFT_FORWARD_STATE = LEFT_FORWARD_STATE;
  PREV_LEFT_BACK_STATE = LEFT_BACK_STATE;
  PREV_MODE_STATE = MODE_STATE;
}

void update_state() {
  RIGHT_FORWARD_STATE = digitalRead(RIGHT_FORWARD_SWITCH);
  RIGHT_BACK_STATE = digitalRead(RIGHT_BACK_SWITCH);
  LEFT_FORWARD_STATE = digitalRead(LEFT_FORWARD_SWITCH);
  LEFT_BACK_STATE = digitalRead(LEFT_BACK_SWITCH);
  MODE_STATE = digitalRead(MODE_SWITCH);
}

int current_time = 0;
int time_elapsed_between_mode_toggles = 0;

void update_mode() {
  
  current_time = millis();
  
  if (MODE_STATE != PREV_MODE_STATE) {
    time_elapsed_between_mode_toggles = (current_time - last_mode_toggle_time);
    last_mode_toggle_time = current_time;

    // if time elapsed is bewteen 0 and 1 toggle modes
    if ( (time_elapsed_between_mode_toggles > 0) && (time_elapsed_between_mode_toggles < MODE_TOGGLE_INTERVAL) ) {
      if ( (mode == SHOULDER_ELBOW) || (mode == FOREARM_HAND) ) {
        if (MODE_STATE == UP) {
          mode = WHEELS_FAST;
        } else {
          mode = WHEELS_SLOW;
        }
      } else {
        if (MODE_STATE == UP) {
          mode = FOREARM_HAND;
        } else {
          mode = SHOULDER_ELBOW;
        }
      }
    } else {
      if ( (mode == SHOULDER_ELBOW) || (mode == FOREARM_HAND) ) {
        if (MODE_STATE == UP) {
          mode = FOREARM_HAND;
        } else {
          mode = SHOULDER_ELBOW;
        }
      } else {
        if (MODE_STATE == UP) {
          mode = WHEELS_FAST;
        } else {
          mode = WHEELS_SLOW;
        }
      }
    }
  } else {
    last_mode_toggle_time = current_time;
  }
}

bool is_state_changed() {
  return (PREV_RIGHT_FORWARD_STATE != RIGHT_FORWARD_STATE ||
          PREV_RIGHT_BACK_STATE != RIGHT_BACK_STATE ||
          PREV_LEFT_FORWARD_STATE != LEFT_FORWARD_STATE ||
          PREV_LEFT_BACK_STATE != LEFT_BACK_STATE ||
          PREV_MODE_STATE != MODE_STATE);
}

void write_params() {

  //8 is the address of the onboard Rugged Mega.
  Wire.beginTransmission(8);
  Wire.write(1); //preamble to indicated wheel params vs arm params

  // all three left wheels are driven together (tank dirve)
  Wire.write(wheelParams.left_speed);
  Wire.write(wheelParams.left_dir);
  Wire.write(wheelParams.left_speed);
  Wire.write(wheelParams.left_dir);
  Wire.write(wheelParams.left_speed);
  Wire.write(wheelParams.left_dir);

  // all three right wheels are driven together (tank dirve)
  Wire.write(wheelParams.right_speed);
  Wire.write(wheelParams.right_dir);
  Wire.write(wheelParams.right_speed);
  Wire.write(wheelParams.right_dir);
  Wire.write(wheelParams.right_speed);
  Wire.write(wheelParams.right_dir);
  
  Wire.endTransmission();
}

void apply_state() {
  switch(mode) {
  case SHOULDER_ELBOW: Serial.println("SE"); break;
  case FOREARM_HAND: Serial.println("FH"); break;
  case WHEELS_SLOW: Serial.println("WS"); break;
  case WHEELS_FAST: Serial.println("WF"); break;
  }
}

void set_pin_modes() {
  pinMode(RIGHT_FORWARD_SWITCH, INPUT);
  pinMode(RIGHT_BACK_SWITCH, INPUT);
  pinMode(LEFT_FORWARD_SWITCH, INPUT);
  pinMode(LEFT_BACK_SWITCH, INPUT);
  pinMode(MODE_SWITCH, INPUT);
}
