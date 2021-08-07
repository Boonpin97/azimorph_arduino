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
  if (cmd_vx >= MAX_SPEED) {
    cmd_vx = MAX_SPEED;
  }
  else if (cmd_vx <= -MAX_SPEED) {
    cmd_vx = -MAX_SPEED;
  }
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
  displayspeed();
  Odometry();

  nh.spinOnce();
  delay(10);
}

void cmd_velToRoboclaw() {
  right_speed_ms = (cmd_vz * Wheel_Distance) / 2 + cmd_vx;
  left_speed_ms = (cmd_vx * 2) - right_speed_ms;

  right_speed = MsToEncSpeed(right_speed_ms);
  left_speed = MsToEncSpeed(left_speed_ms);
  if (right_speed > 0) {
    roboclaw.ForwardM2(address, abs(right_speed));
  }
  else {
    roboclaw.BackwardM2(address, abs(right_speed));
  }
  if (left_speed > 0) {
    roboclaw.ForwardM1(address, abs(left_speed));
  }
  else {
    roboclaw.BackwardM1(address, abs(left_speed));
  }
  //roboclaw.SpeedM1(address, right_speed); //right
  //roboclaw.SpeedM2(address, left_speed); //left
}


double cal_x(double leftspeed, double rightspeed) {
  double x = (leftspeed + rightspeed) / 2.00; //m/s
  return -x;
}

double cal_theta(double leftspeed, double rightspeed) {
  double Q = (rightspeed - leftspeed) / (Wheel_Distance); //ticks/s
  return Q;
}

int32_t MsToEncSpeed(double x) {

  int32_t EncSpeed = ((x / Wheel_Circumference) * 176128.0) / 3094;
  if (EncSpeed > 127) {
    EncSpeed = 127;
  }
  else if (EncSpeed < -127) {
    EncSpeed = -127;
  }
  return EncSpeed;
}

int32_t MsToRoboSpeed(double x) {

  int32_t RoboSpeed = x / 3800; //divide 3800 to get the robospeed
  return RoboSpeed;
}

double EncSpeedToMs(double x) {

  double SpeedInMs = (x / 176128.0) * Wheel_Circumference; //speed/20000 = rps(m/s)
  return SpeedInMs;
}

void displayspeed(void) {

  left_enc = roboclaw.ReadEncM2(address, &status1, &valid1);
  left_speed = roboclaw.ReadSpeedM2(address, &status2, &valid2);
  right_enc = roboclaw.ReadEncM1(address, &status1, &valid1);
  right_speed = roboclaw.ReadSpeedM1(address, &status2, &valid2);
  //
  //  Serial.print("Left:");
  //  Serial.print(EncSpeedToMs(left_speed));
  //  Serial.print("  ");
  //
  //
  //  Serial.print("Right:");
  //  Serial.print(EncSpeedToMs(right_speed));
  //  Serial.print("  ");
  //
  //  Serial.print("vx:");
  //  Serial.print(vx);
  //  Serial.print("  ");
  //
  //  Serial.print("theta:");
  //  Serial.print(vtheta);
  //  Serial.print("  ");
  //
  //  Serial.print("Z:");
  //  Serial.print(vz);
  //  Serial.print("  ");


  Serial.println(" ");
}



void Odometry () {
  left_speed_ms = EncSpeedToMs(left_speed); //ticks/s to m/s
  right_speed_ms = EncSpeedToMs(right_speed); //ticks/s to m/s

  vx = cal_x(left_speed_ms, right_speed_ms); //linear x velocity
  vtheta = cal_theta(left_speed_ms, right_speed_ms); //angular velocity*/

  currenttime = millis();
  double dt = (currenttime - lasttime) / 1000;
  double delta_th = vtheta * dt;
  THETA = delta_th + THETA;

  if (THETA >= 6.28) {
    THETA = THETA - 6.28;
  }
  if (THETA <= -6.28) {
    THETA = THETA + 6.28;
  }


  double delta_x = vx * cos(THETA) * dt;
  double delta_y = vx * sin(THETA) * dt;

  X = X + delta_x; //linear x  ??dk if need refer to diffposition.h
  Y = Y + delta_y; //linear y

  lasttime = currenttime;

  t.header.frame_id = odom;
  t.child_frame_id = base_link;

  t.transform.translation.x = X;
  t.transform.translation.y = Y;

  t.transform.rotation = tf::createQuaternionFromYaw(THETA);
  t.header.stamp = nh.now();

  broadcaster.sendTransform(t);
  nh.spinOnce();
}
