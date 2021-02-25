#include <ros.h>
#include <rover_msgs/GripperElevatorControl.h>
#include <Tic.h>

TicI2C tic;

#define FLT 2  // D2
#define INA 3   // D3
#define INB 4  // D4
#define PWM 5   // D5
int counter = 0;
bool engage = false;

ros::NodeHandle_<ArduinoHardware, 2, 1, 160, 210> node_handle;

rover_msgs::GripperElevatorControl control_msg;

void control_Callback(const rover_msgs::GripperElevatorControl& control_msg) {

  if (control_msg.gripper == 0) {

    analogWrite(PWM, 0);
  } else if (control_msg.gripper > 0) {

    digitalWrite(INA, HIGH);
    digitalWrite(INB, LOW);
    analogWrite(PWM, control_msg.gripper);
  } else if (control_msg.gripper < 0) {

    digitalWrite(INA, LOW);
    digitalWrite(INB, HIGH);
    analogWrite(PWM, -255 - control_msg.gripper);
  }
  
  if (engage == true) {

    if (control_msg.elevator == 12) {
      
      tic.deenergize();
      engage = false;
    } else {
      
      tic.setTargetVelocity(control_msg.elevator * 2000000);
    }
  } else if ((engage == false) && (control_msg.elevator != 1)) {

    engage = true;
    // Set up I2C.
    Wire.begin();
    // Give the Tic some time to start up.
    delay(20);
    tic.exitSafeStart();
    tic.setTargetVelocity(0);
    tic.energize();
  }
}

// Sends a "Reset command timeout" command to the Tic.  We must
// call this at least once per second, or else a command timeout
// error will happen.  The Tic's default command timeout period
// is 1000 ms, but it can be changed or disabled in the Tic
// Control Center.
void resetCommandTimeout()
{
  tic.resetCommandTimeout();
}

// Delays for the specified number of milliseconds while
// resetting the Tic's command timeout so that its movement does
// not get interrupted by errors.
void delayWhileResettingCommandTimeout(uint32_t ms)
{
  uint32_t start = millis();
  do
  {
    resetCommandTimeout();
  } while ((uint32_t)(millis() - start) <= ms);
}

ros::Subscriber<rover_msgs::GripperElevatorControl> control_subscriber("grip_elev", &control_Callback);

void setup()
{
  // !! Set pin functionality

  // FAULT - LOW: Overcurrent condition or thermal shutdown
  //         HIGH: Normal Operation
  pinMode(FLT, INPUT);

  // ENABLE - LOW: Enable driver outputs
  //          HIGH: Enable Tri-State ??????????????
  pinMode(INA, OUTPUT);

  // DIRECTION - LOW: Reverse
  //             HIGH: Forward
  pinMode(INB, OUTPUT);

  // PULSE WIDTH MODULATION - 0: 0% Duty Cycle
  //                          255: 100% Duty Cycle
  pinMode(PWM, OUTPUT);

  // !! Set pin states !!
  
  // Sets ENABLE to enable driver outputs
  digitalWrite(INA, LOW);

  // Sets DIRECTION to forward
  digitalWrite(INB, HIGH);

  // Sets PULSE WIDTH MODULATION to 0% duty cycle
  analogWrite(PWM, 0);
  
  node_handle.initNode();
  node_handle.subscribe(control_subscriber);
}

void loop()
{

  node_handle.spinOnce();
  delay(1);
}
