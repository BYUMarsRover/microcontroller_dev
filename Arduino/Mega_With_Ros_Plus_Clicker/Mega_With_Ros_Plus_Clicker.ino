//AUTHORS: Joseph Cannon & Spencer Stowell
//DATE: 3/22/23
//PURPOSE: Added ROS functionality to Rover Code

//Topic for elevator commands is: "grip_elev", (located in the IKController.py script)

//include ROS libraries
#include <ros.h>
#include <std_msgs/UInt16MultiArray.h>
//#include <std_msgs/UInt16.h>
#include <std_msgs/Bool.h>

//this is the header file that was generated to use our custom message to publish motor speeds and dirs.
#include "IWC_motors.h"

//this is custom header file for the elevator message that we will need to read from.
#include "GripperElevatorControl.h"

//include custom header files
#include "Globals.h"
#include "Wheels.h"
using namespace std;

const int MAX_CLICK_TIME = 2100;
const int CLICKER_DIR = 34;
const int CLICKER_ENABLE = 36;
const int LIMIT_SWITCH = 47;
const int LASER_CTRL = 22;
volatile unsigned long lastClickForward = 0;
volatile unsigned long lastClickBackward = 0;
volatile unsigned long lastClickDuration = 0;
volatile bool clicking = false;

//set up ROS node handle.
ros::NodeHandle nh;

//ROS serial setup

//Configure message for publishing IR sensor data.
std_msgs::UInt16MultiArray sensorArray;

//Initialize IR publisher
ros::Publisher IRpub("IR", &sensorArray);

//Create object of type "Wheels" names "wheels"
Wheels wheels;

//Initialize variables used to protect against lost connection

//Variable to store the current timestamp for communication
unsigned long currentTime = 0;

//Variable to store the timestamp of the previous command
unsigned long oldCommandTime = 0;

//Constant variable that sets the maximum delay between motor commands
//If this value is exceeded, it implies that communication has been lost between
//the Xavier and the Mega and the Mega will shut down all motion in the wheels
const int TIMEDIFF = 2000000;

//Constant variable used to shut down the wheels
const byte STOP_WHEELS = byte(0);

//ros node callback (gets called when something gets published to the IWC_motorControl topic)
//Expects object of type IWC_motors from the rover_msgs package
//Each command following the motorArray object relates to the IWC_motors.msg
void wheelCb(const rover_msgs::IWC_motors & motorArray) {
  wheels.wheelList[0].set_speed = motorArray.left_front_speed;
  wheels.wheelList[0].dir = motorArray.left_front_dir;
  wheels.wheelList[1].set_speed = motorArray.left_middle_speed;
  wheels.wheelList[1].dir = motorArray.left_middle_dir;
  wheels.wheelList[2].set_speed = motorArray.left_rear_speed;
  wheels.wheelList[2].dir = motorArray.left_rear_dir;
  wheels.wheelList[3].set_speed = motorArray.right_rear_speed;
  wheels.wheelList[3].dir = motorArray.right_rear_dir;
  wheels.wheelList[4].set_speed = motorArray.right_middle_speed;
  wheels.wheelList[4].dir = motorArray.right_middle_dir;
  wheels.wheelList[5].set_speed = motorArray.right_front_speed;
  wheels.wheelList[5].dir = motorArray.right_front_dir;

  //Update the time stamp for the most recent command
  oldCommandTime = millis();
}

//ros node callback (gets called when something gets published to the grip_elev topic)
//Expects object of type GripperElevatorControl from the rover_msgs package
void elevatorCb(const rover_msgs::GripperElevatorControl & elevatorArray) {
  wheels.wheelList[6].set_speed = elevatorArray.elevator_speed;
  wheels.wheelList[6].dir = elevatorArray.elevator_direction;
}

//ros node callback (gets called when something gets published to the "" topic)
//Expects object of type UInt16 from the std_msgs package
void clickCb(const std_msgs::Bool & clickCmd) {
  //Only actuate the clicker if the corresponding X-box button is pressed
  if (clickCmd.data && !clicking) {
    digitalWrite(CLICKER_DIR, HIGH);
    digitalWrite(CLICKER_ENABLE, HIGH);
    lastClickForward = millis();
    clicking = true;
  }
}

void laserCb(const std_msgs::Bool & laserCmd) {
  //Only actuate the clicker if the corresponding X-box button is pressed
  if (laserCmd.data) {
    digitalWrite(13, HIGH);
    digitalWrite(LASER_CTRL, HIGH);
  }
  else{
    digitalWrite(13, LOW);
    digitalWrite(LASER_CTRL, LOW);
  }
}

//subscribe to Ros topic for wheel commands
//The topic is in quotes "" and the callback fn follows the &
ros::Subscriber<rover_msgs::IWC_motors> subwheels("IWC_motorControl", &wheelCb);

//subscribe to Ros topic for elevator commands
//The topic is in quotes "" and the callback fn follows the &
ros::Subscriber<rover_msgs::GripperElevatorControl> subelev("grip_elev", &elevatorCb);

//subscribe to Ros topic for clicker commands
//The topic is in quotes "" and the callback fn follows the &
ros::Subscriber<std_msgs::Bool> subclick("arm_clicker", &clickCb);

//subscribe to Ros topic for clicker commands
//The topic is in quotes "" and the callback fn follows the &
ros::Subscriber<std_msgs::Bool> sublaser("arm_laser", &laserCb);

//create variables for the two IR analog reading variables.
int reading_IR1;
int reading_IR2;

//Initialize empty array to use for memory allocation with the IR publisher object
int array0[2];

void setup() {

  //sets all pins as inputs or outputs
  set_pin_modes();

  // Write the default (initialization) parameters contained in the Wheels class to the wheels
  wheels.writeParams();

  // Ros setup stuff
  // Configure the baud rate. Must match between ROS and here
  nh.getHardware()->setBaud(115200);

  //Initialize the arduino as a ROS node
  nh.initNode();

  //Tell the publisher object what data length to expect
  sensorArray.data_length = 2;

  //Initialize the memory using the empty array (see above)
  sensorArray.data = array0;

  //Configure the subscribers for wheels and elevator
  nh.subscribe(subwheels);
  nh.subscribe(subelev);
  nh.subscribe(subclick);
  nh.subscribe(sublaser);

  //Begin advertising the publisher
  nh.advertise(IRpub);

  //Set the first command time
  oldCommandTime = millis();
  
  pinMode(LIMIT_SWITCH, INPUT);
  pinMode(CLICKER_DIR, OUTPUT);
  pinMode(CLICKER_ENABLE, OUTPUT);
  pinMode(LASER_CTRL, OUTPUT);
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  digitalWrite(CLICKER_ENABLE, LOW);
  digitalWrite(CLICKER_DIR, LOW);
  digitalWrite(LASER_CTRL, LOW);

}

void loop() {

  //Read from IR sensors
  sensorArray.data[0] = analogRead(GRIP_IR1);
  sensorArray.data[1] = analogRead(GRIP_IR2);

  //publish data on the first IR sensor
  IRpub.publish(&sensorArray);

  //Reads/Writes all data from/to relevant topics via buffers (512 bytes default)
  //Without this command, no publisher would ever send data and no subscriber
  //would ever receive data.
  nh.spinOnce();

  wheels.writeParams(); // Write the values of the wheels data members onto the arduino pins

  if(clicking && ((millis() > lastClickForward + MAX_CLICK_TIME) || (digitalRead(LIMIT_SWITCH) == HIGH))){
    lastClickBackward = millis();
    lastClickDuration = lastClickBackward - lastClickForward;
    clicking = false;
    digitalWrite(CLICKER_ENABLE, HIGH);
    digitalWrite(CLICKER_DIR, LOW);
  }
  else if(!clicking && (millis() > lastClickBackward + min(lastClickDuration,MAX_CLICK_TIME))){
    digitalWrite(CLICKER_ENABLE, LOW);
  }

  //delay to keep computer and arduino in sync (publishing and reading at same time)
  //Frequency does not need to be exact, but should be close
  delay(17);  //Approx 60 Hz

  //Code to check if elapsed time between motor commands exceeds the value set in
  //TIMEDIFF
  currentTime = millis();
  if ((currentTime - oldCommandTime) > TIMEDIFF) {
    for (int i = 0; i < NUM_WHEELS; i++) {
      wheels.wheelList[i].set_speed = STOP_WHEELS;
      wheels.wheelList[i].dir = STOP_WHEELS;
    }

    wheels.writeParams();
  }
}

// This method sets the pins for each motor
void set_pin_modes() {
  //wheels
  pinMode(RIGHT_FRONT_WHEEL_SET_SPEED, OUTPUT);
  pinMode(RIGHT_FRONT_WHEEL_DIR, OUTPUT);
  pinMode(RIGHT_FRONT_WHEEL_ENABLE, OUTPUT);
  pinMode(RIGHT_FRONT_WHEEL_ERROR, INPUT_PULLUP);

  pinMode(RIGHT_MIDDLE_WHEEL_SET_SPEED, OUTPUT);
  pinMode(RIGHT_MIDDLE_WHEEL_DIR, OUTPUT);
  pinMode(RIGHT_MIDDLE_WHEEL_ENABLE, OUTPUT);
  pinMode(RIGHT_MIDDLE_WHEEL_ERROR, INPUT_PULLUP);

  pinMode(RIGHT_REAR_WHEEL_SET_SPEED, OUTPUT);
  pinMode(RIGHT_REAR_WHEEL_DIR, OUTPUT);
  pinMode(RIGHT_REAR_WHEEL_ENABLE, OUTPUT);
  pinMode(RIGHT_REAR_WHEEL_ERROR, INPUT_PULLUP);

  pinMode(LEFT_FRONT_WHEEL_SET_SPEED, OUTPUT);
  pinMode(LEFT_FRONT_WHEEL_DIR, OUTPUT);
  pinMode(LEFT_FRONT_WHEEL_ENABLE, OUTPUT);
  pinMode(LEFT_FRONT_WHEEL_ERROR, INPUT_PULLUP);

  pinMode(LEFT_MIDDLE_WHEEL_SET_SPEED, OUTPUT);
  pinMode(LEFT_MIDDLE_WHEEL_DIR, OUTPUT);
  pinMode(LEFT_MIDDLE_WHEEL_ENABLE, OUTPUT);
  pinMode(LEFT_MIDDLE_WHEEL_ERROR, INPUT_PULLUP);

  pinMode(LEFT_REAR_WHEEL_SET_SPEED, OUTPUT);
  pinMode(LEFT_REAR_WHEEL_DIR, OUTPUT);
  pinMode(LEFT_REAR_WHEEL_ENABLE, OUTPUT);
  pinMode(LEFT_REAR_WHEEL_ERROR, INPUT_PULLUP);

  //elevator
  pinMode(ELEVATOR_SET_SPEED, OUTPUT);
  pinMode(ELEVATOR_DIR, OUTPUT);
  pinMode(ELEVATOR_ENABLE, OUTPUT);
  pinMode(ELEVATOR_ERROR, INPUT_PULLUP);

  //IR sensors
  pinMode(GRIP_IR1, INPUT);
  pinMode(GRIP_IR2, INPUT);
}

//This method reads ir sensors.
//void read_ir() {
//  reading_IR1 = analogRead(GRIP_IR1);
//  sensor.data = analogRead(GRIP_IR1);
//  reading_IR2 = analogRead(GRIP_IR2);
//}
