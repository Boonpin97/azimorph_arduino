// ROS Test 
// Tests reading and publishing of GPS and IMU
#if ARDUINO>=100
  #include <Arduino.h>  // Arduino 1.0
#else
  #include <WProgram.h>  // Arduino 0022
#endif// ROS includes
#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/NavSatFix.h>
#include <SoftwareSerial.h>
#include <Adafruit_GPS.h>
#define GPS_PUBLISH_RATE 5 //hz// ROS messages
sensor_msgs::NavSatFix navSat_msg;
ros::NodeHandle nh;
ros::Publisher gpsPub("gps", &navSat_msg);
SoftwareSerial GPSSerial(10, 11);
Adafruit_GPS GPS(&GPSSerial);
// Set to 'true' to debug and listen to the raw GPS sentences
#define GPSECHO falseuint32_t timer = millis();// blink
const byte ledPin = 13;
boolean output = HIGH;// general variables 
boolean bAutoMode = false;
boolean bSensorInit = false;

uint32_t publish_imu_time = 0;
uint32_t publish_gps_time = 0;void setup() {
  // initialize ROS node
  nh.initNode();
  nh.getHardware()->setBaud(57600);
  nh.advertise(gpsPub);
  
  // setup LED blink
  pinMode(ledPin, OUTPUT);
  GPS.begin(9600);
  // uncomment this line to turn on RMC 
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but
  // either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
     
  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);  
  while (!nh.connected()) {
    nh.spinOnce();
  }
}void loop() {

  // read data from the GPS in the 'main loop'
  char c = GPS.read();  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA())) 
      return; 
  }  //this block publishes velocity based on defined rate
  if ((millis() - publish_gps_time) >= (1000 / GPS_PUBLISH_RATE))
  {
    //publishVelocities();
    navSat_msg.latitude = GPS.latitude;
    navSat_msg.longitude = GPS.longitude;
    navSat_msg.altitude = GPS.altitude;
    gpsPub.publish(&navSat_msg);
    // velocity
    publish_gps_time = millis();
  }
}
