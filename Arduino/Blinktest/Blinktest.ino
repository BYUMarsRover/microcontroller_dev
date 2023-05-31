/* 
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */

#include <ros.h>
#include <std_msgs/UInt16.h>

ros::NodeHandle  nh;

void messageCb( const std_msgs::UInt16& LED){
  digitalWrite(LED_BUILTIN, LED.data);   // blink the led
}

ros::Subscriber<std_msgs::UInt16> sub("chatter", &messageCb );

void setup()
{ 
  pinMode(LED_BUILTIN, OUTPUT);
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);
  
}

void loop()
{  
  nh.spinOnce();
  delay(20);
}
