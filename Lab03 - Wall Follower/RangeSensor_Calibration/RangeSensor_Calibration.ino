/*
GP2D120/GP2D12 Calibration Sketch
Used for calibrating range sensors
LCD will read analog outputs measured by sensor so that they may be plotted and curve-fitted

Written: Pranav Maroli 
Date: 4/24/2017
ME179L
 */

#include <SoftwareSerial.h>

const int rxPin = 13;
const int txPin = 13;
SoftwareSerial mySerial =  SoftwareSerial(rxPin,txPin);

const int analogInPin = A0;  // Analog input pin that the range sensor is attached to

int sensorValue = 0;        // value read from the first sensor

void setup() {
  // initialize serial communications at 9600 bps:
  pinMode( txPin, OUTPUT);
  mySerial.begin(9600); 
}

void loop() {
  // read the analog in value:
  sensorValue = analogRead(analogInPin);
  
  // print the results to the serial monitor:
  mySerial.print("?x00?y0");     //Sets Cursor to x00,y0 
  mySerial.print("Sensor 1 = ");
  mySerial.print(sensorValue);
  mySerial.print("    ");
 
  // wait 2 milliseconds before the next loop
  // for the analog-to-digital converter to settle
  // after the last reading:
  delay(2);
}
