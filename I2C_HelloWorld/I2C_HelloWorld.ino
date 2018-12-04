/*
 * @Author: Tucker Wilkes
 * @Description: The purpose of this program is to demonstrate
 * various I2C functions. 
 * 
 * The motorControl structure is where the bytes sent from the Jetson
 * get parsed into - this will allow us to easily set the speed and dir
 * of each individual wheel.
 * 
 * The structure motorFeedback is for the puspose of reading feedback
 * from the motor drivers. Currently we have it set up to read
 * speed, amps, and one byte available for any needed error messages
*/
#include <Wire.h>

#define I2C_ADDRESS 8//the address used to register on the I2C bus

struct motorControl {
  byte right_front_speed;
  byte right_front_dir;
  byte right_middle_speed;
  byte right_middle_dir;
  byte right_rear_speed;
  byte right_rear_dir;

  byte left_front_speed;
  byte left_front_dir;
  byte left_middle_speed;
  byte left_middle_dir;
  byte left_rear_speed;
  byte left_rear_dir;
};

//this struct is used to communicate back to the jetson
//the actual state of the motors
//for each wheel we will send, its actual speed, 
//amps, and any error state
struct motorFeedback {
  byte right_front_speed;
  byte right_front_amps;
  byte right_front_error;
  
  byte right_middle_speed;
  byte right_middle_amps;
  byte right_middle_error;
  
  byte right_rear_speed;
  byte right_rear_amps;
  byte right_rear_error;

  byte left_front_speed;
  byte left_front_amps;
  byte left_front_error;

  byte left_middle_speed;
  byte left_middle_amps;
  byte left_middle_error;

  byte left_rear_speed;
  byte left_rear_amps;
  byte left_rear_error;
};

//init a struct for the bytes to control the motor
struct motorControl motorCommands = {0,0,0,0,0,0,0,0,0,0,0,0};

//struct to contain feedback of motor
struct motorFeedback feedback = {97,98,99,100,101,0,0,0,0,0,0,0,0,0,0,0,0,0};

void setup() {
  Wire.begin(I2C_ADDRESS);                // join i2c bus with address #8
  Wire.onReceive(receiveEvent); // register the receive event
  Wire.onRequest(requestEvent);//register the request event
  Serial.begin(9600);           // start serial for output
}

//This function runs continuously
//In here we print out the state of the motor
void loop() {
  delay(500);
  //print status of right motors
  Serial.print(motorCommands.right_front_speed);
  Serial.print(' ');
  Serial.print(motorCommands.right_front_dir);
  Serial.print(' ');
  Serial.print(motorCommands.right_middle_speed);
  Serial.print(' ');
  Serial.print(motorCommands.right_middle_dir);
  Serial.print(' ');
  Serial.print(motorCommands.right_rear_speed);
  Serial.print(' ');
  Serial.print(motorCommands.right_rear_dir);
  Serial.print(' ');

  //print status of left motors
  Serial.print(motorCommands.left_front_speed);
  Serial.print(' ');
  Serial.print(motorCommands.left_front_dir);
  Serial.print(' ');
  Serial.print(motorCommands.left_middle_speed);
  Serial.print(' ');
  Serial.print(motorCommands.left_middle_dir);
  Serial.print(' ');
  Serial.print(motorCommands.left_rear_speed);
  Serial.print(' ');
  Serial.print(motorCommands.left_rear_dir);
  Serial.print('\n');
}

//function that is called when the jetson reads bytes
//here is where we will send it the feedback
void requestEvent() {
  Wire.write(feedback.right_front_speed);
  Wire.write(feedback.right_front_amps);
  Wire.write(feedback.right_front_error);

  Wire.write(feedback.right_middle_speed);
  Wire.write(feedback.right_middle_amps);
  Wire.write(feedback.right_middle_error);

  Wire.write(feedback.right_rear_speed);
  Wire.write(feedback.right_rear_amps);
  Wire.write(feedback.right_rear_error);

  Wire.write(feedback.left_front_speed);
  Wire.write(feedback.left_front_amps);
  Wire.write(feedback.left_front_error);

  Wire.write(feedback.left_middle_speed);
  Wire.write(feedback.left_middle_amps);
  Wire.write(feedback.left_middle_error);

  Wire.write(feedback.left_rear_speed);
  Wire.write(feedback.left_rear_amps);
  Wire.write(feedback.left_rear_error);
}

//this function is called when we receive bytes from the i2c bus
void receiveEvent(int howMany) {
  //the python i2c library has a preammble byte that we can set
  //as of the time of this writing we are using the preamble to 
  //define what we are going to control
  //0x1 - drive wheels
  //0x2 - arm
  //0x3 - science
  
  for (int index = 0; Wire.available(); index++) {//if we have a message   
    byte c = Wire.read(); // receive byte as a character      

    //consume a byte and place it into the appropiate 
    //data field in the motor control struct
    switch (index) {
      case 0:
        motorCommands.left_rear_speed = c;
      break;
      
      case 1:
        motorCommands.right_front_speed = c;
      break;

      case 2:
        motorCommands.right_front_dir = c;
      break;

      case 3:
        motorCommands.right_middle_speed = c;
      break;

      case 4:
        motorCommands.right_middle_dir = c;
      break;

      case 5:
        motorCommands.right_rear_speed = c;
      break;

      case 6:
        motorCommands.right_rear_dir = c;
      break;

      case 7:
        motorCommands.left_front_speed = c;
      break;

      case 8:
        motorCommands.left_front_dir = c;
      break;

      case 9:
        motorCommands.left_middle_speed = c;
      break;

      case 10:
        motorCommands.left_middle_dir = c;
      break;

      case 11:
        motorCommands.left_rear_speed = c;
      break;

      case 12:
        motorCommands.left_rear_dir = c;
      break;
    }
  } 
}
