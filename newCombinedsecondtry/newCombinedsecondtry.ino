//See BareMinimum example for a list of library functions

//Includes required to use Roboclaw library
#include <SoftwareSerial.h>
#include "RoboClaw.h"
#include <ros.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <roboclaw_base/OdometryLite.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32MultiArray.h>
//#include <std_msgs/String.h>
//#include <std_msgs/Empty.h>

/* We need sin and cos for odometry calcuations */
#include <math.h>

/* The base and odometry frames */
char baseFrame[] = "/base_link";
char odomFrame[] = "/odom";

ros::NodeHandle nh;
//ros::Publisher chatter("Odometry", &float64_odomX);
//std_msgs::OdometryLite odom_msg;

roboclaw_base::OdometryLite odom_msg;
ros::Publisher odomPub("odometry_lite", &odom_msg);

//ros::Rate loop_rate(10);
//ros::Publisher odomPub = nh.advertise<nav_msgs::Odometry>("odom", &odom_msg);
//tf::TransformBroadcaster odom_broadcaster;


//See limitations of Arduino SoftwareSerial
SoftwareSerial serial(10,11);  
RoboClaw roboclaw(&serial,10000);

#define address 0x80
#define MAX_SPEED 1.5
#define Wheel_Distance 0.29     //29cm 
#define Wheel_Circumference 0.38 //38cm
#define Radius 0.06 / 2.00 //6cm

//Velocity PID coefficients
#define Kp 1.0 //0.5
#define Ki 0.5  //0.0
#define Kd 0.25  //0.0
#define qpps 44000  //50000

#define ODOM_RATE 30
const float ODOM_INTERVAL = 1000.0 / ODOM_RATE;

double X = 0.0;
double Y = 0.0;
double THETA = 0.0;

uint8_t status1,status2;
bool valid1,valid2;
int32_t left_enc;
int32_t left_speed;
int32_t right_enc;
int32_t right_speed;

double right_speed_ms;
double left_speed_ms;

double right_speed_robo;
double left_speed_robo;
/*double vx;     //this part suscribe to cmd_vel
double vtheta;*/

unsigned long nextOdom = 0;
#define SERIALTIMEOUT 25

float lasttime ;
float currenttime ;

float vx, vy, vz, vtheta;
void cmdVelCb(const geometry_msgs::Twist& msg){
  
  
  vx = msg.linear.x; // m/s
  vy = 0.0; // m/s
  vz = msg.angular.z; // rad/s

  if(vx >= MAX_SPEED) {
    vx = MAX_SPEED;
    }
  else if (vx <= -MAX_SPEED) {
    vx = -MAX_SPEED;
    }

  
  }

  /* A subscriber for the /cmd_vel topic */
ros::Subscriber<geometry_msgs::Twist> cmdVelSub("/cmd_vel", &cmdVelCb);


//This is the first function arduino runs on reset/power up
void setup() {
  //Open Serial and roboclaw at 38400bps
  Serial.begin(57600);
  roboclaw.begin(19200);

  //Set PID Coefficients
  roboclaw.SetM1VelocityPID(address,Kd,Kp,Ki,qpps);
  roboclaw.SetM2VelocityPID(address,Kd,Kp,Ki,qpps); 
  
  //Serial.println("Starting...");
  lasttime=millis();
  nextOdom = ODOM_INTERVAL;

  nh.initNode();
  nh.advertise(odomPub);
  
  nh.subscribe(cmdVelSub);

  //wait until you are actually connected
  while (!nh.connected()) nh.spinOnce();


  
}

void loop() {
  /*if(serial.available()>0){
    updateOdom(); //update and publish the odom
    nextOdom += ODOM_INTERVAL;
  }
    //roboclaw.SpeedM1(address, 12000);
    //roboclaw.SpeedM2(address, 12000);

    //chatter.publish( X );
    
    //nh.spinOnce();
    delay(100);
  }*/
  cmdConvert(); //convert cmd values into left right wheel velocity that robot can understand
     
  moveRobot(); //convert into robospeed and then publish to robot

  //displayspeed(); //display the speed 

  updateOdom(); //update and publish the odom
    //updateOdom(); //update and publish the odom
   /*if (millis() >= nextOdom){
    
    
    updateOdom(); //update and publish the odom
    nextOdom += ODOM_INTERVAL;
  }*/
    //roboclaw.SpeedM1(address, 12000);
    //roboclaw.SpeedM2(address, 12000);

    //chatter.publish( X );
    
    nh.spinOnce();
    delay(1000);
    //loop_rate.sleep();
    

}

int32_t MsToEncSpeed(double x){
  
  int32_t EncSpeed=((x/Wheel_Circumference)*20000.0); //do not divide by 3800 to get the robospeed
  return EncSpeed;
  }

int32_t MsToRoboSpeed(double x){
  
  int32_t RoboSpeed= x/3800; //divide 3800 to get the robospeed
  return RoboSpeed;
  }

double EncSpeedToMs(int x){
  
  double SpeedInMs=(x/20000.0)*Wheel_Circumference; //speed/20000 = rps(m/s)
  return SpeedInMs;
  }



//Display Encoder and Speed for Motor 1
void displayspeed(void){
  

  left_enc = roboclaw.ReadEncM1(address, &status1, &valid1);
  left_speed = roboclaw.ReadSpeedM1(address, &status2, &valid2);
  right_enc = roboclaw.ReadEncM2(address, &status1, &valid1);
  right_speed = roboclaw.ReadSpeedM2(address, &status2, &valid2);


    /*Serial.print("Left:");
    left_speed_ms=EncSpeedToMs(left_speed);
    Serial.print(left_speed_ms);
    Serial.print("  ");


    Serial.print("Right:");
    right_speed_ms=EncSpeedToMs(right_speed);
    Serial.print(right_speed_ms);
    Serial.print("  ");

    Serial.print("X:");
    Serial.print(X);
    Serial.print("  ");

    Serial.print("Y:");
    Serial.print(Y);
    Serial.print("  ");

    Serial.print("THETA:");
    Serial.print(THETA);
    Serial.print("  ");


Serial.println(" ");*/
}


double cal_x(double leftspeed, double rightspeed){

  double x = (leftspeed+rightspeed)* Radius /2.00; //m/s
  return x;
}

double cal_theta(double leftspeed, double rightspeed){

  double Q = (rightspeed-leftspeed)* Radius / Wheel_Distance; //ticks/s
  return Q;

}
void cmdConvert() {

  //calculate left and right wheel velocity
  //right_speed_ms = (vx + vz * Wheel_Distance) / 2 * Radius;
  //left_speed_ms = (vx - vz * Wheel_Distance) / 2 * Radius;
  
  right_speed_ms = (vz * Wheel_Distance)/2 + vx;  //get rid of 180/3.142 because in rad/s
  left_speed_ms = (vx * 2) - right_speed_ms;

  right_speed = MsToEncSpeed(right_speed_ms); //ticks/s
  left_speed = MsToEncSpeed(left_speed_ms); //ticks/s
  
  }  

void moveRobot() {

  //right_speed_robo = MsToRoboSpeed(right_speed); //robospeed  ms --> ticks/s ---> robospeed
  //left_speed_robo = MsToRoboSpeed(left_speed); //robospeed  ms --> ticks/s ---> robospeed
  
  roboclaw.SpeedM1(address, left_speed); //right
  roboclaw.SpeedM2(address, right_speed); //left
  return;
  
  }

void updateOdom() {
  

  //cmdConvert(); //convert cmd values into left right wheel velocity that robot can understand
     
  //moveRobot(); //convert into robospeed and then publish to robot

  //displayspeed(); //display the speed

  
  od

  /*left_speed_ms=EncSpeedToMs(left_speed_robo); //ticks/s to m/s
  right_speed_ms=EncSpeedToMs(right_speed_robo); //ticks/s to m/s
  vx = cal_x(left_speed_ms, right_speed_ms); //linear x velocity
  vtheta = cal_theta(left_speed_ms, right_speed_ms); //angular velocity*/
  
  Odometry(vx, vtheta);       //ticks/s --> m/s                       //Hi Keith, this function calculates the od0m. Please publish the variable; X,Y and THETA to ros.

  }



void Odometry ( double speedx, double speedtheta){
   left_speed_ms=EncSpeedToMs(left_speed); //ticks/s to m/s
   right_speed_ms=EncSpeedToMs(right_speed); //ticks/s to m/s
   vx = cal_x(left_speed_ms, right_speed_ms); //linear x velocity
   vtheta = cal_theta(left_speed_ms, right_speed_ms); //angular velocity*/
  
   currenttime = millis();
   double dt = (currenttime-lasttime)/1000;
   double delta_th = speedtheta * dt;
   THETA = delta_th + THETA;

   if (THETA >= 6.28){
     THETA = THETA - 6.28;
   }
   if (THETA <= -6.28){
     THETA = THETA + 6.28;
   }
   
   double delta_x = speedx * cos(THETA) * dt;
   double delta_y = speedx * sin(THETA) * dt;
   X = X + delta_x; //linear x  ??dk if need refer to diffposition.h
   Y = Y + delta_y; //linear y

   lasttime = currenttime;


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
   odom_msg.pose.position.x = X;
   odom_msg.pose.position.y = Y;
   odom_msg.pose.position.z = 0;
   odom_msg.pose.orientation = quaternion;
   odom_msg.twist.linear.x = vx;
   odom_msg.twist.linear.y = vy;
   odom_msg.twist.linear.z = 0;
   odom_msg.twist.angular.x = 0;
   odom_msg.twist.angular.y = 0;
   odom_msg.twist.angular.z = THETA;

  odomPub.publish(&odom_msg);
}

  
