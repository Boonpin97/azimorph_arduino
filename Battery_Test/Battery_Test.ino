#include <ros.h>
#include <std_msgs/UInt16.h>

ros::NodeHandle nh;

std_msgs::UInt16 str_msg;
ros::Publisher battery("battery", &str_msg);

void setup()
{
  nh.initNode();
  nh.advertise(battery);
}

void loop()
{
  str_msg.data = analogRead(A9);
  battery.publish( &str_msg );
  nh.spinOnce();
  delay(1000);
}
