//See BareMinimum example for a list of library functions

//Includes required to use Roboclaw library
#include <SoftwareSerial.h>
#include "RoboClaw.h"
#include <ros.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32MultiArray.h>
#include <math.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

ros::NodeHandle nh;

geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;

SoftwareSerial serial(10, 11);
RoboClaw roboclaw(&serial, 10000);

#define address 0x80
#define MAX_SPEED 1.5
#define Wheel_Distance 0.4      //29cm 
double Wheel_Circumference = 0.59; //38cm
#define ODOM_RATE 30
//#define SERIALTIMEOUT 25

char base_link[] = "/base_link";
char odom[] = "/odom";

unsigned long nextOdom = 0;
const float ODOM_INTERVAL = 1000.0 / ODOM_RATE;

float cmd_vx = 0, cmd_vy, cmd_vz = 0;
float vx = 0, vy = 0, vz = 0, vtheta;
double X = 0.0;
double Y = 0.0;
double THETA = 0.0;

uint8_t status1, status2;
bool valid1, valid2;
int32_t left_enc;
int32_t left_speed;
int32_t right_enc;
int32_t right_speed;
double right_speed_ms;
double left_speed_ms;
float lasttime ;
float currenttime ;

void cmdVelCb(const geometry_msgs::Twist& msg) {

  cmd_vx = msg.linear.x; // m/s
  cmd_vy = 0.0; // m/s
  cmd_vz = msg.angular.z; // rad/s
}

ros::Subscriber<geometry_msgs::Twist> cmdVelSub("/cmd_vel", &cmdVelCb);

void setup() {
  pinMode(13, OUTPUT);
  // put your setup code here, to run once:
  Serial.begin(57600);
  roboclaw.begin(38400);
  broadcaster.init(nh);
  nh.initNode();

  nh.subscribe(cmdVelSub);
  //while (!nh.connected()) nh.spinOnce();
  lasttime = millis();
}

void loop() {
  // put your main code here, to run repeatedly:


  cmd_velToRoboclaw();

  nh.spinOnce();
  delay(10);
}



void cmd_velToRoboclaw() {
  right_speed_ms = (cmd_vz * Wheel_Distance) / 2 + cmd_vx;
  left_speed_ms = (cmd_vx * 2) - right_speed_ms;

  right_speed = -MsToEncSpeed(right_speed_ms);
  left_speed = -MsToEncSpeed(left_speed_ms);
  roboclaw.SpeedM1(address, right_speed); //right
  roboclaw.SpeedM2(address, left_speed); //left
}
