//Need to figure out how to get over range sensor peak to achieve P Control

#include <AFMotor.h>
#include <SoftwareSerial.h>
#define SwitchPin_A 1   // switch A is connected to pin 1
#define SwitchPin_B 2   // switch B is connected to pin 2

const int rxPin = 13;
const int txPin = 13;
const int analogInPin = A0;  // Analog input pin that the SHORT range sensor is attached to
const int longPin = A1;  // Analog input pin that the LONG range sensor is attached to
const int KpL = 1000;
const int KpR = 1000;
int sensorValue = 0; //Short range sensor value
int longValue = 0; //Long range sensor value

int oldSpeedR = 200;
int oldSpeedL = 255;
int countS = 0;
int diff = 0;
int maxdiff = 30;

SoftwareSerial mySerial =  SoftwareSerial(rxPin,txPin);
AF_DCMotor right_motor(3, MOTOR12_64KHZ);// create motor #1 64 KHZ pwm
AF_DCMotor left_motor(4, MOTOR12_64KHZ); // create motor #2, 64KHz pwm

void setup() {
pinMode( txPin, OUTPUT);
mySerial.begin(9600); // initialize serial communications at 9600 bps:

left_motor.setSpeed(oldSpeedL); // Speeds are set differently to compensate for difference in motor power
right_motor.setSpeed(oldSpeedR); 
}

int setpoint = 340; //Setpoint distance from the wall for the short range sensor

void loop(){ 

  sensorValue = analogRead(analogInPin);
  longValue = analogRead(longPin);
  diff = sensorValue - setpoint;

  // print the results to the serial monitor:
  mySerial.print("?x00?y0");     //Sets Cursor to x00,y0 
  mySerial.print("Short: ");
  mySerial.print(sensorValue);
  mySerial.print("    ");
  mySerial.print("?x00?y1");     //Sets Cursor to x00,y0 
  mySerial.print("Long: ");
  mySerial.print(longValue);
  mySerial.print("    ");
  mySerial.print("?x10?y1");     //Sets Cursor to x00,y0 
  mySerial.print("D:");
  mySerial.print(diff);  
  mySerial.print("    ");

  if(longValue > 370) {
    countS++;
    if(countS > 2){
      left_motor.setSpeed(255); 
      right_motor.setSpeed(200);
      right_motor.run(FORWARD);
      left_motor.run(BACKWARD);
      delay(750);
      countS = 0;
      left_motor.run(FORWARD);  
    }
//  }else if(diff < maxdiff && diff > -maxdiff){
//    left_motor.setSpeed(255); 
//    right_motor.setSpeed(200);
//    countS = 0;
  }else {
    left_motor.setSpeed(newSpeedL(diff, oldSpeedL, KpL));
    right_motor.setSpeed(newSpeedR(diff, oldSpeedR, KpR));
    countS = 0;
  }

 // delay(2);
}

int newSpeedR(int diff,int oldSpeedR,int KpR){
  int newSpeed = oldSpeedR + KpR*diff;
  if(newSpeed > 255){
    oldSpeedR = 255;
    return 255;
  }else if(newSpeed < 5){
    oldSpeedR = 0;
    return 0;
  }else {
    oldSpeedR = newSpeed;
    return newSpeed;
  }
}

int newSpeedL(int diff,int oldSpeedL,int KpL){
  int newSpeed = oldSpeedL - KpL*diff;
  if(newSpeed > 255){
    oldSpeedL = 255;
    return 255;
  }else if(newSpeed < 5){
    oldSpeedL = 0;
    return 0;
  }else {
    oldSpeedL = newSpeed;
    return newSpeed;
  }
}


