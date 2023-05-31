//AUTHOR: Joseph Cannon
//DATE: 2/23/23
//PURPOSE: Added ROS functionality to Rover Code

//include ROS libraries
#include <ros.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/UInt16.h>
//this is the header file that was generated to use our custom message to publish motor speeds and dirs.
#include "IWC_motors.h"

//include custom libraries
#include "Globals.h"
#include "Wheels.h"
using namespace std;

//set up ROS node handle. 
ros::NodeHandle nh;

//ROS serial setup
std_msgs::UInt16 sensor;

//Initialize IR publisher
ros::Publisher IRpub("IR", &sensor);

Wheels wheels;

//ros node callback (gets called when something gets published) 
void messageCb(const rover_msgs::IWC_motors & motorArray) {
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
}

//subscribe to Ros message for wheel commands
ros::Subscriber<rover_msgs::IWC_motors> sub("IWC_motorControl", &messageCb);



//create variables for the two IR analog reading variables.   
int reading_IR1;
int reading_IR2;

void setup() {

    //sets all pins as inputs or outputs
    set_pin_modes();

    // Write the default (initialization) parameters contained in the Wheels class to the wheels
    wheels.writeParams();

    // Ros setup stuff. 
    nh.getHardware()->setBaud(115200);
    nh.initNode();
    nh.subscribe(sub);
    nh.advertise(IRpub);
}

void loop() {       
        
        //Read from IR sensors
        sensor.data = analogRead(GRIP_IR1);

        //publish data on the first IR sensor
        IRpub.publish(&sensor);

        //calls ros node.
        nh.spinOnce();   
          
        wheels.writeParams(); // Write the values of the wheels data members onto the arduino pins   
             
        //delay to keep computer and arduino in sync (publishing and reading at same time)
        delay(17);
    
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
