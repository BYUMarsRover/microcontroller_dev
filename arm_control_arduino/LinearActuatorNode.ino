/* 
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */

#include <ros.h>
#include <std_msgs/Empty.h>

ros::NodeHandle  nh;

void callback( const std_msgs::Empty& toggle_msg){
  // take out linear actuator command and gripper command (velocity: slow, med, fast)
  // DEBUG: print out velocity for both
  // Change delay for bit banging method to match velocity for whichever is being commanded. 
}

ros::Subscriber<std_msgs::Empty> sub("joint_states", callback);

void setup()
{ 
  pinMode(LED_BUILTIN, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{  
  nh.spinOnce();
  delay(1);
}
