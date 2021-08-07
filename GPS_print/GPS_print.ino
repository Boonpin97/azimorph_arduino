#include <SoftwareSerial.h>

SoftwareSerial mySerial(10, 11); // RX, TX

void setup() {
  // put your setup code here, to run once:
     // put your setup code here, to run once:
  mySerial.begin(9600);
  Serial.begin(38400);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (mySerial.available()) {
    //Serial.print((char)Serial1.read());
    Serial.print((char)mySerial.read()); // read each character
  }
  //Serial.println(" ");
}
