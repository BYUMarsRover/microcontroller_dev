#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>

const int I2C_ADDRESS = 8;
const int ESCON_RPM_RANGE_MIN = -6000;
const int ESCON_RPM_RANGE_MAX = 6000;
const int NUM_WHEELS = 6;

const int LEFT_FRONT_WHEEL_SET_SPEED = 2;
const int LEFT_FRONT_WHEEL_ENABLE = 22;
const int LEFT_FRONT_WHEEL_DIR = 28;
const int LEFT_FRONT_WHEEL_ERROR = 34;
const int LEFT_FRONT_WHEEL_ACTUAL_SPEED = A8;

const int LEFT_MIDDLE_WHEEL_SET_SPEED = 3;
const int LEFT_MIDDLE_WHEEL_ENABLE = 23;
const int LEFT_MIDDLE_WHEEL_DIR = 29;
const int LEFT_MIDDLE_WHEEL_ERROR = 35;
const int LEFT_MIDDLE_WHEEL_ACTUAL_SPEED = A9;

const int LEFT_REAR_WHEEL_SET_SPEED = 4;
const int LEFT_REAR_WHEEL_ENABLE = 24;
const int LEFT_REAR_WHEEL_DIR = 30;
const int LEFT_REAR_WHEEL_ERROR = 36;
const int LEFT_REAR_WHEEL_ACTUAL_SPEED = A10;

const int RIGHT_REAR_WHEEL_SET_SPEED = 5;
const int RIGHT_REAR_WHEEL_ENABLE = 25;
const int RIGHT_REAR_WHEEL_DIR = 31;
const int RIGHT_REAR_WHEEL_ERROR = 37;
const int RIGHT_REAR_WHEEL_ACTUAL_SPEED = A11;

const int RIGHT_MIDDLE_WHEEL_SET_SPEED = 6;
const int RIGHT_MIDDLE_WHEEL_ENABLE = 26;
const int RIGHT_MIDDLE_WHEEL_DIR = 32;
const int RIGHT_MIDDLE_WHEEL_ERROR = 38;
const int RIGHT_MIDDLE_WHEEL_ACTUAL_SPEED = A12;

const int RIGHT_FRONT_WHEEL_SET_SPEED = 7;
const int RIGHT_FRONT_WHEEL_ENABLE = 27;
const int RIGHT_FRONT_WHEEL_DIR = 33;
const int RIGHT_FRONT_WHEEL_ERROR = 39;
const int RIGHT_FRONT_WHEEL_ACTUAL_SPEED = A13;

const int POWER_INDICATOR = 13;

#endif