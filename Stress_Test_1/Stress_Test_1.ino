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
#include <std_msgs/Float64.h>
#include <std_msgs/Float32MultiArray.h>
#include <math.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#define Wheel_Distance 0.36
#define Wheel_Circumference 0.67
#define Encoder_Click 20

ros::NodeHandle nh;

geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;
std_msgs::Float64 str_msg;

/** Initiate VescUart class */
VescUart MotorFR;
VescUart MotorFL;
VescUart MotorBL;
VescUart MotorBR;

int spd = 500;
int turn = 1000;
float cmd_vx = 0, cmd_vy = 0, cmd_vz = 0;

void cmdVelCb(const geometry_msgs::Twist& msg) {
  cmd_vx = msg.linear.x; // m/s
  cmd_vy = 0.0; // m/s
  cmd_vz = msg.angular.z; // rad/s
  Serial.println("Recieve cmd vel");
}

ros::Subscriber<geometry_msgs::Twist> cmdVelSub("/cmd_vel", &cmdVelCb);
ros::Publisher battery("battery", &str_msg);

void setup() {

  /** Setup Serial port to display data */
  Serial.begin(9600);
  Serial2.begin(115200);
  Serial3.begin(115200);
  Serial4.begin(115200);
  Serial5.begin(115200);

  broadcaster.init(nh);
  nh.initNode();
  nh.subscribe(cmdVelSub);
  nh.advertise(battery);
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
  //UART.nunchuck.lowerButton = true;
}

void loop() {

  /** Call the function getVescValues() to acquire data from VESC */

  //  MotorBR.setRPM(100);
  //  MotorFR.setRPM(100);
  //  MotorFL.setRPM(spd);
  //  MotorBL.setRPM(spd);
  //cmd_vx = 0;
  //cmd_vz = 3;
  str_msg.data = analogRead(A9) / 30.4;
  battery.publish( &str_msg);
  //MotorFR.setHandBrake(30);
  //MotorFL.setHandBrake(30);
  //MotorBR.setHandBrake(30);
  //MotorBL.setHandBrake(30);
  //  MotorFR.setRPM(0);
  //  MotorFL.setRPM(0);
  //  MotorBR.setRPM(0);
  //  MotorBL.setRPM(0);
  MotorFR.setBrakeCurrent(30);
  MotorFL.setBrakeCurrent(30);
  MotorBR.setBrakeCurrent(30);
  MotorBL.setBrakeCurrent(30);
  nh.spinOnce();
}

void cmd_velToVESC() {
  float right_speed_ms = (cmd_vz * Wheel_Distance) / 2 + cmd_vx;
  float left_speed_ms = (cmd_vx * 2) - right_speed_ms;
  float right_rpm = ( right_speed_ms / Wheel_Circumference ) * 60 * Encoder_Click;
  float left_rpm = ( left_speed_ms / Wheel_Circumference ) * 60 * Encoder_Click;
  if (right_rpm == 0) {
    MotorBR.setBrakeCurrent(30);
    MotorFL.setBrakeCurrent(30);
    MotorFR.setBrakeCurrent(30);
    MotorBL.setBrakeCurrent(30);
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
