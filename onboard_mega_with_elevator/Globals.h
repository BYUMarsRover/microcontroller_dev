//changes that need to be made in this code to implement the elevator. This is the code that sets the pins correspoonding with each of the wheels. 

#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>

const int I2C_ADDRESS = 10;
const int ESCON_RPM_RANGE_MIN = -6000;
const int ESCON_RPM_RANGE_MAX = 6000;
//CHANGE THE NUMBER OF WHEELS BEING USED TO SEVEN FOR USE IN THE OTHER FUNCTIONS
const int NUM_WHEELS = 7;



//ADDED SECTION ASSIGING PINS FOR THE ELEVATOR MOTOR/////////
const int ELEVATOR_SET_SPEED = 9;
const int ELEVATOR_ENABLE = 30;
const int ELEVATOR_DIR = 31;
const int ELEVATOR_ERROR = 32;  //This is what is called ready on the pcb. not sure which one it is actually doing
const int ELEVATOR_ACTUAL_SPEED = A5;   //
//END ADDED SECTION 
const int LEFT_FRONT_WHEEL_SET_SPEED = 45;
const int LEFT_FRONT_WHEEL_ENABLE = 49;
const int LEFT_FRONT_WHEEL_DIR = 53;
const int LEFT_FRONT_WHEEL_ERROR = 51;
const int LEFT_FRONT_WHEEL_ACTUAL_SPEED = A15;

const int LEFT_MIDDLE_WHEEL_SET_SPEED = 46;
const int LEFT_MIDDLE_WHEEL_ENABLE = 48;
const int LEFT_MIDDLE_WHEEL_DIR = 50;
const int LEFT_MIDDLE_WHEEL_ERROR = 52;
const int LEFT_MIDDLE_WHEEL_ACTUAL_SPEED = A9;

const int LEFT_REAR_WHEEL_SET_SPEED = 44;
const int LEFT_REAR_WHEEL_ENABLE = 38;
const int LEFT_REAR_WHEEL_DIR = 40;
const int LEFT_REAR_WHEEL_ERROR = 42;
const int LEFT_REAR_WHEEL_ACTUAL_SPEED = A3;

const int RIGHT_REAR_WHEEL_SET_SPEED = 2;
const int RIGHT_REAR_WHEEL_ENABLE = 27;
const int RIGHT_REAR_WHEEL_DIR = 25;
const int RIGHT_REAR_WHEEL_ERROR = 23;
const int RIGHT_REAR_WHEEL_ACTUAL_SPEED = A6;

const int RIGHT_MIDDLE_WHEEL_SET_SPEED = 3;
const int RIGHT_MIDDLE_WHEEL_ENABLE = 24;
const int RIGHT_MIDDLE_WHEEL_DIR = 26;
const int RIGHT_MIDDLE_WHEEL_ERROR = 28;
const int RIGHT_MIDDLE_WHEEL_ACTUAL_SPEED = A12;

const int RIGHT_FRONT_WHEEL_SET_SPEED = 4;
const int RIGHT_FRONT_WHEEL_ENABLE = 5;
const int RIGHT_FRONT_WHEEL_DIR = 6;
const int RIGHT_FRONT_WHEEL_ERROR = 7;
const int RIGHT_FRONT_WHEEL_ACTUAL_SPEED = A10;

//const int JETSON_POWER_ON = 44;

const int POWER_INDICATOR = 13;

#endif
