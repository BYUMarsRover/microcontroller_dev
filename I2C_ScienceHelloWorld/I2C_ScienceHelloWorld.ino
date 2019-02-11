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

#define I2C_ADDRESS 11//the address used to register on the I2C bus
#define NUM_CO2_SENSORS 5

struct scienceControl {
  byte led_status;//true for on, false for off
  byte vacuum_status;//true for on
  byte rot_chamber_pos;//values 1-6 for the different positions  
};

//this struct is used to communicate back to the jetson
//the actual state of the motors
//for each wheel we will send, its actual speed, 
//amps, and any error state
struct scienceFeedback {
  byte led_status;//true for on, false for off
  byte vacuum_status;//true for on
  byte rot_chamber_pos;//values 1-6 for the different positions 

  byte co2Bytes[NUM_CO2_SENSORS*2];//we need two bytes for each co2 sensor
};

//init a struct for the bytes to control the motor
struct scienceControl scienceCommands = {0,0,0};

//struct to contain feedback of motor
struct scienceFeedback feedback = {1,2,3};

void setup() {
  Wire.begin(I2C_ADDRESS);                // join i2c bus with address #8
  Wire.onReceive(receiveEvent); // register the receive event
  Wire.onRequest(requestEvent);//register the request event
  Serial.begin(9600);           // start serial for output

  for (int i = 0; i < i < NUM_CO2_SENSORS*2; i++) {
    feedback.co2Bytes[i] = i;
  }
}

//This function runs continuously
//In here we print out the state of the motor
void loop() {
  delay(500);
  
}

//function that is called when the jetson reads bytes
//here is where we will send it the feedback
void requestEvent() {
  Wire.write(feedback.led_status);
  Wire.write(feedback.vacuum_status);
  Wire.write(feedback.rot_chamber_pos);

  //write each co2 byte
  for (int i = 0; i < NUM_CO2_SENSORS*2; i++) {
    Wire.write(feedback.co2Bytes[i]);
  }
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
    //data field in the control struct
    switch (index) {      //ignore the first byte cause it's just the preamble
      case 1:
        scienceCommands.led_status = c;
      break;

      case 2:
        scienceCommands.vacuum_status = c;
      break;

      case 3:
        scienceCommands.rot_chamber_pos = c;
      break;      
    }
  } 
}
