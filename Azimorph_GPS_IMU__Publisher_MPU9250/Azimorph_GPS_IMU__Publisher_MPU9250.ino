// ROS Test
// Tests reading and publishing of GPS and IMU
#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/NavSatFix.h>
#include <TinyGPS.h>
#include <Wire.h>
#include "MPU9250.h"
#include "eeprom_utils.h"

MPU9250 mpu;
#define GPS_PUBLISH_RATE 5 //hz// ROS messages
sensor_msgs::Imu imu_msg;
ros::Publisher imu("imu", &imu_msg);
sensor_msgs::NavSatFix navSat_msg;
ros::NodeHandle nh;
ros::Publisher gpsPub("gps", &navSat_msg);
TinyGPS gps;
float flat, flon;
unsigned long age;
uint32_t publish_gps_time = 0;
unsigned long timer = 0;
float timeStep = 0.01;
float deg2rad = 0.0174528; // 3.142/180
// Pitch, Roll and Yaw values
float pitch = 0;
float roll = 0;
float yaw = 0;

void setup() {
  // initialize ROS node
  nh.initNode();
  nh.getHardware()->setBaud(57600);
  nh.advertise(gpsPub);
  nh.advertise(imu);
  Wire.begin();
  Serial1.begin(9600);
  if (!mpu.setup(0x68)) {  // change to your own address
    while (1) {
      Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
      delay(5000);
    }
  }
  mpu.verbose(false);
  loadCalibration();
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
  if (mpu.update()) {
    if ((millis() - publish_gps_time) >= (1000 / GPS_PUBLISH_RATE) && mpu.update())
    {
      imu_msg.header.frame_id = 0;
      imu_msg.orientation.x = mpu.getQuaternionX();
      imu_msg.orientation.y = mpu.getQuaternionY();
      imu_msg.orientation.z = mpu.getQuaternionZ();
      imu_msg.orientation.w = mpu.getQuaternionW();
      imu_msg.linear_acceleration.x = mpu.getLinearAccX();
      imu_msg.linear_acceleration.y = mpu.getLinearAccY();
      imu_msg.linear_acceleration.z = mpu.getLinearAccZ();
      imu_msg.header.stamp = nh.now();
      gps.f_get_position(&flat, &flon, &age);

      //publishVelocities();
      navSat_msg.latitude = String(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6).toFloat();
      navSat_msg.longitude = String(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6).toFloat();
      navSat_msg.altitude = 0;
      gpsPub.publish(&navSat_msg);
      // velocity
      publish_gps_time = millis();
      imu.publish(&imu_msg);
      nh.spinOnce();
    }
  }
}
