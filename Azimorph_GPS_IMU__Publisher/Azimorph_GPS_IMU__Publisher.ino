// ROS Test
// Tests reading and publishing of GPS and IMU
#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <TinyGPS.h>
#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;
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

  Serial1.begin(9600);
  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  while (!nh.connected()) {
    nh.spinOnce();
  }
  mpu.calibrateGyro();

  // Set threshold sensivty. Default 3.
  // If you don't want use threshold, comment this line or set 0.
  mpu.setThreshold(3);
}

void loop() {

  while (Serial1.available())
  {
    char c = Serial1.read();
    gps.encode(c);
  }
  if ((millis() - publish_gps_time) >= (1000 / GPS_PUBLISH_RATE))
  {
    Vector normGyro = mpu.readNormalizeGyro();
    Vector normAccel = mpu.readNormalizeAccel();
    pitch = pitch + normGyro.YAxis * timeStep;
    roll = roll + normGyro.XAxis * timeStep;
    yaw = yaw + normGyro.ZAxis * timeStep;
    double cy = cos(yaw * 0.5 * deg2rad);
    double sy = sin(yaw * 0.5 * deg2rad);
    double cp = cos(pitch * 0.5 * deg2rad);
    double sp = sin(pitch * 0.5 * deg2rad);
    double cr = cos(roll * 0.5 * deg2rad);
    double sr = sin(roll * 0.5 * deg2rad);

    imu_msg.header.frame_id = 0;
    imu_msg.orientation.x = sr * cp * cy - cr * sp * sy;
    imu_msg.orientation.y = cr * sp * cy + sr * cp * sy;
    imu_msg.orientation.z = cr * cp * sy - sr * sp * cy;
    imu_msg.orientation.w = cr * cp * cy + sr * sp * sy;
    imu_msg.linear_acceleration.x = normAccel.XAxis;
    imu_msg.linear_acceleration.y = normAccel.YAxis;
    imu_msg.linear_acceleration.z = normAccel.ZAxis;
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
