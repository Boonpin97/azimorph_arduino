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
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32MultiArray.h>
#include <math.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#define Wheel_Distance 0.36
#define Wheel_Circumference 0.67
#define Encoder_Click 128

ros::NodeHandle nh;

geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;
std_msgs::UInt16 str_msg;

char base_link[] = "/base_link";
char odom[] = "/odom";

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

  delay(3000);
  //UART.nunchuck.lowerButton = true;
}

void loop() {
//  MotorFR.setRPM(1500);
//  MotorFL.setRPM(1500);
//  MotorBL.setRPM(1500);
//  MotorBR.setRPM(1500);

  MotorFR.setRPM(0);
  MotorFL.setRPM(0);
  MotorBL.setRPM(0);
  MotorBR.setRPM(0);
  /** Call the function getVescValues() to acquire data from VESC */
  getEncoderValue();
  Odometry();

  nh.spinOnce();
}

void getEncoderValue() {
  if ( MotorFR.getVescValues() ) {
    current_FR_enc = MotorFR.data.tachometer;
  }
  if ( MotorFL.getVescValues() ) {
    current_FL_enc = MotorFL.data.tachometer;
  }
  if ( MotorBR.getVescValues() ) {
    current_BR_enc = MotorBR.data.tachometer;
  }
  if ( MotorBL.getVescValues() ) {
    current_BL_enc = MotorBL.data.tachometer;
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

  //    Serial.print("FR");
  //    Serial.print(FR_enc_speed);
  //    Serial.print(" BR:");
  //    Serial.print(BR_enc_speed);
  //    Serial.print(" FL:");
  //    Serial.print(FL_enc_speed);
  //    Serial.print(" BL");
  //    Serial.println(BL_enc_speed);
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
    t.header.frame_id = odom;
    t.child_frame_id = base_link;
  
    t.transform.translation.x = x;
    t.transform.translation.y = y;
  
    t.transform.rotation = tf::createQuaternionFromYaw(THETA);
    t.header.stamp = nh.now();
  
    broadcaster.sendTransform(t);
    nh.spinOnce();
}
