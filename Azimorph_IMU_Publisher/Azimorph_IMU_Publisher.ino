#include <ros.h>
#include <sensor_msgs/Imu.h>#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

// Timers
unsigned long timer = 0;
float timeStep = 0.01;
float deg2rad = 0.0174528; // 3.142/180
// Pitch, Roll and Yaw values
float pitch = 0;
float roll = 0;
float yaw = 0;

//Set up the ros node and publisher
sensor_msgs::Imu imu_msg;
ros::Publisher imu("imu", &imu_msg);
ros::NodeHandle nh;



void setup()
{

  nh.initNode();
  nh.getHardware()->setBaud(57600);
  nh.advertise(imu);

  //Serial.begin(115200);

  // Initialize MPU6050
  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }

  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want calibrate, comment this line.
  mpu.calibrateGyro();

  // Set threshold sensivty. Default 3.
  // If you don't want use threshold, comment this line or set 0.
  mpu.setThreshold(3);
}

long publisher_timer;

void loop()
{
  Vector normGyro = mpu.readNormalizeGyro();
  Vector normAccel = mpu.readNormalizeAccel();

  // Calculate Pitch, Roll and Yaw
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


  if (millis() > publisher_timer) {
    // step 1: request reading from sensor
    imu.publish(&imu_msg);
    publisher_timer = millis() + 100; //publish ten times a second
    nh.spinOnce();
  }


}
