//AUTHORS: David Hill (based on code by Joseph Cannon and Spencer Stowell)
//DATE: 4/11/2023
//PURPOSE: Add ROS functionality to base antenna servo

//Topic for servo is: "base_antenna_angle"

//include ROS libraries
#include <ros.h>
#include <std_msgs/UInt8.h>

//this is the header file for running a servos
#include <Servo.h>

using namespace std;

const int SERVO_PIN = 9;
volatile uint8_t angleCommand = 0;
Servo antennaServo;  // create servo object to control a servo

//set up ROS node handle.
ros::NodeHandle nh;

//ros node callback (gets called when something gets published to the base_antenna_angle topic)
//Expects object of type Uint16 from the rover_msgs package
void servoCb(const std_msgs::UInt8 &angleCmd) {
  angleCommand = angleCmd.data;
}


//subscribe to Ros topic for servo commands
//The topic is in quotes "" and the callback fn follows the &
ros::Subscriber<std_msgs::UInt8> subservo("base_antenna_angle", &servoCb);

void setup() {

  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(subservo);

  //initialize servo with correct pin
  antennaServo.attach(SERVO_PIN);

}

void loop() {
  nh.spinOnce();
  antennaServo.write(angleCommand); // tell servo to go to position in variable 'pos'
  delay(10);
}
