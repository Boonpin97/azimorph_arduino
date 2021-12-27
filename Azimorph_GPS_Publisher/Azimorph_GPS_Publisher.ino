// ROS Test
// Tests reading and publishing of GPS and IMU
#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/NavSatFix.h>
#include <TinyGPS.h>
#define GPS_PUBLISH_RATE 5 //hz// ROS messages
sensor_msgs::NavSatFix navSat_msg;
ros::NodeHandle nh;
ros::Publisher gpsPub("gps", &navSat_msg);
TinyGPS gps;
float flat, flon;
unsigned long age;
uint32_t publish_gps_time = 0;

void setup() {
  // initialize ROS node
  nh.initNode();
  nh.getHardware()->setBaud(57600);
  nh.advertise(gpsPub);
  Serial1.begin(9600);
  while (!nh.connected()) {
    nh.spinOnce();
  }
}

void loop() {

  while (Serial1.available())
  {
    char c = Serial1.read();
    gps.encode(c);
  }
  if ((millis() - publish_gps_time) >= (1000 / GPS_PUBLISH_RATE))
  {
    gps.f_get_position(&flat, &flon, &age);

    //publishVelocities();
    navSat_msg.latitude = String(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6).toFloat();
    navSat_msg.longitude = String(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6).toFloat();
    navSat_msg.altitude = 0;
    navSat_msg.header.stamp = nh.now();
    gpsPub.publish(&navSat_msg);
    // velocity
    publish_gps_time = millis();
  }
}
