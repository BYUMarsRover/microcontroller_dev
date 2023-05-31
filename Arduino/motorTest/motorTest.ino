#include <ros.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/UInt16.h>

ros::NodeHandle nh;

std_msgs::UInt16 sensor;
ros::Publisher IRpub("IR", &sensor);

void messageCb( const std_msgs::UInt16MultiArray& motorArray) {
  for (int i = 0; i < 14; i++) {
    analogWrite(i, motorArray.data[i]);
  }
  digitalWrite(LED_BUILTIN,HIGH-digitalRead(LED_BUILTIN));
}

ros::Subscriber<std_msgs::UInt16MultiArray> sub("motors", &messageCb);

void setup() {
  // put your setup code here, to run once:
  pinMode(A3, INPUT);

  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(IRpub);
}

void loop() {
  // put your main code here, to run repeatedly:
  sensor.data = analogRead(A3);

  IRpub.publish(&sensor);
  nh.spinOnce();
  delay(100);
}
