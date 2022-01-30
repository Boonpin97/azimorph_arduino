/*
  Name:    getVescValues.ino
  Created: 19-08-2018
  Author:  SolidGeek
  Description:  This example is made using a Arduino Micro (Atmega32u4) that has a HardwareSerial port (Serial1) seperated from the Serial port.
                A Arduino Nano or Uno that only has one Serial port will not be able to display the data returned.
*/

#include <VescUart.h>
#include <ros.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <math.h>
//#include <tf/tf.h>
//#include <tf/transform_broadcaster.h>

#define Wheel_Distance 0.415
#define Wheel_Circumference 0.61
#define RPM_Click 20
#define Encoder_Click 128

ros::NodeHandle nh;

//geometry_msgs::TransformStamped t;
std_msgs::Float64 batt_msg;
std_msgs::Float64MultiArray motortemp_msg;
nav_msgs::Odometry odom_msg;
char baseFrame[] = "base_link";
char odomFrame[] = "odom";

/** Initiate VescUart class */
VescUart MotorFR;
VescUart MotorFL;
VescUart MotorBL;
VescUart MotorBR;

int spd = 500;
int turn = 1000;
float cmd_vx = 0, cmd_vy = 0, cmd_vz = 0;
int32_t current_FR_enc;
int32_t current_FL_enc;
int32_t current_BR_enc;
int32_t current_BL_enc;
float lasttime = millis();
float THETA;
float delta_x;
float delta_y;
float x;
float y;
int32_t  prev_FL_enc = 0;
int32_t  prev_FR_enc = 0;
int32_t  prev_BL_enc = 0;
int32_t  prev_BR_enc = 0;
float FR_temp = 0.0;
float BR_temp = 0.0;
float FL_temp = 0.0;
float BL_temp = 0.0;

void cmdVelCb(const geometry_msgs::Twist& msg) {
  cmd_vx = msg.linear.x; // m/s
  cmd_vy = 0.0; // m/s
  cmd_vz = msg.angular.z; // rad/s
  Serial.println("Recieve cmd vel");
}

ros::Subscriber<geometry_msgs::Twist> cmdVelSub("/cmd_vel", &cmdVelCb);
ros::Publisher battery("battery", &batt_msg);
ros::Publisher motortemp("motor_temp", &motortemp_msg);
ros::Publisher odomPub("odom_raw", &odom_msg);

void setup() {

  /** Setup Serial port to display data */
  Serial.begin(57600);
  Serial2.begin(115200);
  Serial3.begin(115200);
  Serial4.begin(115200);
  Serial5.begin(115200);
  nh.getHardware()->setBaud(57600);

  //broadcaster.init(nh);
  nh.initNode();
  nh.subscribe(cmdVelSub);
  nh.advertise(battery);
  nh.advertise(motortemp);
  nh.advertise(odomPub);
  motortemp_msg.data = (float*)malloc(sizeof(float) * 4);
  motortemp_msg.data_length = 4;
  while (!Serial2) {
    ;
    Serial2.println("Front Right Serial Fail");
  }
  while (!Serial3) {
    ;
    Serial3.println("Front Left Serial Fail");
  }
  while (!Serial4) {
    ;
    Serial4.println("Back Left Serial Fail");
  }
  while (!Serial5) {
    ;
    Serial5.println("Back Right Serial Fail");
  }

  /** Define which ports to use as UART */
  MotorFR.setSerialPort(&Serial2);
  MotorFL.setSerialPort(&Serial3);
  MotorBL.setSerialPort(&Serial4);
  MotorBR.setSerialPort(&Serial5);
  delay(3000);

  if ( MotorFR.getVescValues() ) {
    prev_FR_enc = MotorFR.data.tachometer;
  }
  if ( MotorFL.getVescValues() ) {
    prev_FL_enc = MotorFL.data.tachometer;
  }
  if ( MotorBR.getVescValues() ) {
    prev_BR_enc = MotorBR.data.tachometer;
  }
  if ( MotorBL.getVescValues() ) {
    prev_BL_enc = MotorBL.data.tachometer;
  }
}

void loop() {

  getVESCValue();
  batt_msg.data = analogRead(A9) / 35.2;
  Serial.println(batt_msg.data);
  battery.publish( &batt_msg);
  //Serial.println(FL_temp);
  motortemp_msg.data[0] = FL_temp;
  motortemp_msg.data[1] = FR_temp;
  motortemp_msg.data[2] = BL_temp;
  motortemp_msg.data[3] = BR_temp;

  motortemp.publish( &motortemp_msg);
  cmd_velToVESC();
  //    Serial.print("VX:");
  //    Serial.print(cmd_vx);
  //    Serial.print(" |VY:");
  //    Serial.print(cmd_vy);
  //    Serial.print(" |VZ:");
  //    Serial.println(cmd_vz);
  Odometry();

  nh.spinOnce();
}

void cmd_velToVESC() {
  float right_speed_ms = (cmd_vz * Wheel_Distance) / 2 + cmd_vx;
  float left_speed_ms = (cmd_vx * 2) - right_speed_ms;
  float right_rpm = ( right_speed_ms / Wheel_Circumference ) * 60 * RPM_Click;
  float left_rpm = ( left_speed_ms / Wheel_Circumference ) * 60 * RPM_Click;
  if (right_rpm == 0) {
    MotorBR.setBrakeCurrent(30);
    MotorFR.setBrakeCurrent(30);
    //MotorBR.setRPM(0);
    //MotorFR.setRPM(0);
  }
  else {
    MotorBR.setRPM(right_rpm);
    MotorFR.setRPM(right_rpm);
  }
  if (left_rpm == 0) {
    MotorFL.setBrakeCurrent(30);
    MotorBL.setBrakeCurrent(30);
    //MotorFL.setRPM(0);
    //MotorBL.setRPM(0);
  }
  else {
    MotorFL.setRPM(left_rpm);
    MotorBL.setRPM(left_rpm);
  }
}

void getVESCValue() {
  if ( MotorFR.getVescValues() ) {
    current_FR_enc = MotorFR.data.tachometer;
    FR_temp = MotorFR.data.tempMotor;
  }
  if ( MotorFL.getVescValues() ) {
    current_FL_enc = MotorFL.data.tachometer;
    FL_temp = MotorFL.data.tempMotor;
  }
  if ( MotorBR.getVescValues() ) {
    current_BR_enc = MotorBR.data.tachometer;
    BR_temp = MotorBR.data.tempMotor;
  }
  if ( MotorBL.getVescValues() ) {
    current_BL_enc = MotorBL.data.tachometer;
    BL_temp = MotorBL.data.tempMotor;
  }
}

void Odometry() {
  float FL_delta_enc = (current_FL_enc - prev_FL_enc);
  float FR_delta_enc = (current_FR_enc - prev_FR_enc);
  float BL_delta_enc = (current_BL_enc - prev_BL_enc);
  float BR_delta_enc = (current_BR_enc - prev_BR_enc);

  prev_FL_enc = current_FL_enc;
  prev_FR_enc = current_FR_enc;
  prev_BL_enc = current_BL_enc;
  prev_BR_enc = current_BR_enc;

  float left_delta_metre = (((FL_delta_enc + BL_delta_enc) / 2) / Encoder_Click ) * Wheel_Circumference;
  float right_delta_metre = (((FR_delta_enc + BR_delta_enc) / 2) / Encoder_Click ) * Wheel_Circumference;

  float delta_r = (left_delta_metre + right_delta_metre) / 2.0;
  float delta_theta = (right_delta_metre - left_delta_metre) / Wheel_Distance; //angular velocity*/

  THETA = delta_theta + THETA;

  if (THETA >= 6.28) {
    THETA = THETA - 6.28;
  }
  if (THETA <= -6.28) {
    THETA = THETA + 6.28;
  }

  delta_x = delta_r * cos(THETA);
  delta_y = delta_r * sin(THETA);

  x = x + delta_x; //linear x  ??dk if need refer to diffposition.h
  y = y + delta_y; //linear y

  //    Serial.print("vx");
  //    Serial.print(vx);
  //    Serial.print(" theta:");
  //    Serial.println(vtheta);

//      Serial.print("FR");
//      Serial.print(current_FR_enc);
//      Serial.print(" BR:");
//      Serial.print(current_BR_enc);
//      Serial.print(" FL:");
//      Serial.print(current_FL_enc);
//      Serial.print(" BL");
//      Serial.println(current_BL_enc);
  //  Serial.print("left_ms:");
  //  Serial.print(left_speed_ms);
  //  Serial.print(" |right_ms:");
  //  Serial.print(right_speed_ms);

  Serial.print(" |x:");
  Serial.print(x);
  Serial.print(" |y:");
  Serial.print(y);
  Serial.print(" |theta:");
  Serial.println(THETA);

  geometry_msgs::Quaternion quaternion;
  quaternion.x = 0.0;
  quaternion.y = 0.0;
  quaternion.z = sin(THETA / 2.0);
  quaternion.w = cos(THETA / 2.0);

  /* Publish the distances and speeds on the odom topic. Set the timestamp
      to the last encoder time. */
  odom_msg.header.frame_id = odomFrame;
  odom_msg.child_frame_id = baseFrame;
  odom_msg.header.stamp = nh.now();
  odom_msg.pose.pose.position.x = x;
  odom_msg.pose.pose.position.y = y;
  odom_msg.pose.pose.position.z = 0;
  odom_msg.pose.pose.orientation = quaternion;
  odom_msg.twist.twist.linear.x = delta_r;
  odom_msg.twist.twist.linear.y = delta_theta;
  odom_msg.twist.twist.linear.z = 0;
  odom_msg.twist.twist.angular.x = 0;
  odom_msg.twist.twist.angular.y = 0;
  odom_msg.twist.twist.angular.z = THETA;

  odomPub.publish(&odom_msg);
}
