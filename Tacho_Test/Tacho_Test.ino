/*
  Name:    getVescValues.ino
  Created: 19-08-2018
  Author:  SolidGeek
  Description:  This example is made using a Arduino Micro (Atmega32u4) that has a HardwareSerial port (Serial1) seperated from the Serial port.
                A Arduino Nano or Uno that only has one Serial port will not be able to display the data returned.
*/

#include <VescUart.h>

/** Initiate VescUart class */
VescUart MotorFR;
VescUart MotorFL;
VescUart MotorBL;
VescUart MotorBR;

int FR_init;
int FL_init;
int BR_init;
int BL_init;

void setup() {

  /** Setup Serial port to display data */
  Serial.begin(9600);

  Serial2.begin(115200);
  Serial3.begin(115200);
  Serial4.begin(115200);
  Serial5.begin(115200);

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

  MotorFR.setSerialPort(&Serial2);
  MotorFL.setSerialPort(&Serial3);
  MotorBL.setSerialPort(&Serial4);
  MotorBR.setSerialPort(&Serial5);
  MotorFR.getVescValues();
  FR_init = MotorFR.data.tachometerAbs;
  MotorFL.getVescValues();
  FL_init = MotorFL.data.tachometerAbs;
  MotorBR.getVescValues();
  BR_init = MotorBR.data.tachometerAbs;
  MotorBL.getVescValues();
  BL_init = MotorBL.data.tachometerAbs;
}

void loop() {
  MotorFR.setRPM(0);
  MotorFL.setRPM(0);
  MotorBL.setRPM(0);
  MotorBR.setRPM(0);
  /** Call the function getVescValues() to acquire data from VESC */
  if ( MotorFR.getVescValues() ) {
    Serial.print(" FR:");
    Serial.print(MotorFR.data.tachometerAbs - FR_init);
  }
  else
  {
    Serial.print("Failed to get data!");
  }

  if ( MotorFL.getVescValues() ) {
    Serial.print(" FL:");
    Serial.print(MotorFL.data.tachometerAbs - FL_init);
  }
  else
  {
    Serial.print("Failed to get data!");
  }

  if ( MotorBR.getVescValues() ) {
    Serial.print(" BR:");
    Serial.print(MotorBR.data.tachometerAbs  - BR_init);
  }
  else
  {
    Serial.print("Failed to get data!");
  }

  if ( MotorBL.getVescValues() ) {
    Serial.print(" BL:");
    Serial.println(MotorBL.data.tachometerAbs  - BL_init);
  }
  else
  {
    Serial.println("Failed to get data!");
  }

  delay(50);
}
