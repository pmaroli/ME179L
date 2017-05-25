#include <AFMotor.h>
#include <SoftwareSerial.h>
#include <math.h>
 
// Define constants:
#define switchPin 11   
#define txPin 13   // LCD tx pin.
#define rxPin 13   // LCD rx pin (not really used).
#define encoderPinR 2   // Encoder i.e. break-beam sensor (2 or 3 only, to allow hardware interrupt).
#define encoderPinL 3
#define motorTerminalL 3
#define motorTerminalR 4   // (1-4 only).
#define potPin 2
 
// Define (and initialize) global variables:
volatile int encoderCountL;
volatile int encoderCountR;
// Use "volatile" for faster updating of value during hardware interrupts.
 
const int SRsensor = A0;  // Analog input from left reflector
const int LRsensor = A1; // Analog input from right reflector
const int reflectorL = A3;
const int rightTrigger = A4;
const int leftTrigger = A5;
 
float distSR = 0;        // value read on left
float distLR = 0;        // value read on right
float distRef = 14;              // reference distance
float distLref = 824;             //reference distance to Left Reflector
float distL = 0;
int turnDist = 500;             // distance at which robot turns 180 degrees
float error = 0;
int motorSpeedLR = 160;
int motorSpeedRR = 160;
float kp = 0.7;
int turnCount = 0;
int refreshInt = 2000;
int lastRefresh = 0;
int rightTurn = 0;        // Consecutive counter for right turn line following
int leftTurn = 0;         // Consecutive counter for left turn line following
int rightTriggerThreshold = 800;          //Value reflective sensor reads before initiating a turn
int leftTriggerThreshold = 800;          //Value reflective sensor reads before initiating a turn
unsigned long currentMillis = 0;
unsigned long oldMillis = 0;
unsigned long newMillis = 0;
//int kpR = 1.8;
 
// Define serial display and motor objects:
SoftwareSerial mySerial =  SoftwareSerial( rxPin, txPin);  
AF_DCMotor myMotorL( motorTerminalL, MOTOR34_1KHZ); 
AF_DCMotor myMotorR( motorTerminalR, MOTOR34_1KHZ);
 
void setup() {
  
  int interruptPinR = encoderPinR - 2;   // Hardware interrupt pin (0 or 1 only, to refer to digital pin 2 or 3, respectively).
  attachInterrupt( interruptPinR, IncrementAndDisplayR, RISING);   // Attach interrupt pin, name of function to be called 
        // during interrupt, and whether to run interrupt upon voltage FALLING from high to low or ...
  int interruptPinL = encoderPinL - 2;   // Hardware interrupt pin (0 or 1 only, to refer to digital pin 2 or 3, respectively).
  attachInterrupt( interruptPinL, IncrementAndDisplayL, RISING);   // Attach interrupt pin, name of function to be called 
        // during interrupt, and whether to run interrupt upon voltage FALLING from high to low or ...
  
  // Setup encoder i.e. break-beam:
  pinMode( encoderPinR, INPUT); 
  digitalWrite( encoderPinR, HIGH); 
  pinMode( encoderPinL, INPUT); 
  digitalWrite( encoderPinL, HIGH);
 
  // Setup Light Reflectors
  digitalWrite(reflectorL, HIGH);   //Enables the pull-up resistor
  digitalWrite(rightTrigger, HIGH);   //Enables the pull-up resistor
  digitalWrite(leftTrigger, HIGH);   //Enables the pull-up resistor
 
  //Setup potPin:
  pinMode( potPin,INPUT );
  analogRead( potPin );
 
  // Setup switch:
  pinMode( switchPin, INPUT); 
  digitalWrite( switchPin, HIGH);
 
  // Setup serial display:
  pinMode( txPin, OUTPUT);
  mySerial.begin(9600); 
  mySerial.print("?f");
  
  // Set motor speed:
  myMotorL.setSpeed(motorSpeedLR);
  myMotorR.setSpeed(motorSpeedRR);
 
  myMotorL.run(FORWARD);
  myMotorR.run(FORWARD);
}
 
 
void loop() {
  // start motors forward
  // read distance from both sensors and establish error reading
  distL = getDistL();
  //distR = getDistR();
  error = distLref-distL;
  myMotorL.setSpeed(motorSpeedLR);
  myMotorR.setSpeed(motorSpeedRR);
  myMotorL.run(FORWARD);
  myMotorR.run(FORWARD);
 
  if (analogRead(LRsensor) > turnDist) {
    // if robot is too close to front wall stop and turn
    myMotorL.setSpeed(motorSpeedLR);
    myMotorR.setSpeed(motorSpeedRR);
    myMotorL.run(FORWARD);
    myMotorR.run(BACKWARD);
    delay(200);
    while(analogRead(leftTrigger) < leftTriggerThreshold) {
      //wait to stop turning
    }
    myMotorL.run(FORWARD);
    myMotorR.run(FORWARD);
    delay(200);
    /*newMillis = millis();
    currentMillis = newMillis - oldMillis;
    if (currentMillis >= refreshInt){
    turnCount = ++turnCount;
    oldMillis = newMillis; 
    }*/
  } else if(analogRead(rightTrigger) > rightTriggerThreshold) {
    rightTurn++;
    if(rightTurn>0){
      delay(300);
      myMotorL.setSpeed(motorSpeedLR);
      myMotorR.setSpeed(motorSpeedRR);
      myMotorL.run(FORWARD);
      myMotorR.run(BACKWARD);
      while(analogRead(reflectorL) < distLref) {
        //wait to stop
      }
      rightTurn = 0;
      leftTurn = 0;
    } else {
      rightTurn = 0;
      leftTurn = 0;
    }
  } else if(analogRead(leftTrigger) > leftTriggerThreshold) {
    leftTurn++;
    if(leftTurn>0){
      delay(300);
      myMotorL.setSpeed(motorSpeedLR);
      myMotorR.setSpeed(motorSpeedRR);
      myMotorL.run(BACKWARD);
      myMotorR.run(FORWARD);
      while(analogRead(rightTrigger) < rightTriggerThreshold) {
        //wait to stop
      }
      rightTurn = 0;
      leftTurn = 0;
    } else {
      rightTurn = 0;
      leftTurn = 0;
    }
  } else {
    // correct based on error via proportional gain
    int motorSpeedL = max(0, min(250, motorSpeedLR + error * kp));
    int motorSpeedR = max(0, min(250, motorSpeedRR - error * kp));
    myMotorL.setSpeed(motorSpeedL);
    myMotorR.setSpeed(motorSpeedR);
    myMotorL.run(FORWARD);
    myMotorR.run(FORWARD);
 
    /*mySerial.print("?f");
    mySerial.print("?x00?y0");
    mySerial.print(error);
    mySerial.print("        ");
    mySerial.print("?x00?y1");
    mySerial.print(motorSpeedL);
    mySerial.print("   ");
    mySerial.print(motorSpeedR);*/
 
    mySerial.print("?f");
    mySerial.print("        ");
    mySerial.print("?x00?y1");
    mySerial.print(analogRead(leftTrigger));
    mySerial.print("   ");
    mySerial.print(analogRead(rightTrigger));
    mySerial.print("   ");
    mySerial.print(analogRead(reflectorL));
    mySerial.print("   ");
    mySerial.print("?x00?y0");
    mySerial.print(analogRead(LRsensor));
    mySerial.print("   ");
  }
 
  /*if (turnCount == 8){
    delay(500);
    myMotorL.run(FORWARD);
    myMotorR.run(RELEASE);
    delay(500);
    myMotorL.run(RELEASE);
    myMotorR.run(RELEASE);
    delay(5000);
  }*/
}
 
float getDistSR() {
  // reads SR sensor and converts to cm
  return -8.905*log(analogRead(SRsensor)) + 59.9;//- log(834.5);
}
 
float getDistLR() {
  //reads LR sensor
  return analogRead(LRsensor);
}
 
float getDistL() {
  //reads left reflector
  return analogRead(reflectorL);
  //return -3*pow(10.0,-7.0)*pow(x,3.0)+0.0008*pow(x,2.0)-0.6647*x+185.19;
}
 
/*float getDistR() {
  //reads right reflector
  float x = analogRead(reflectorR);
  return -2*pow(10.0,-7.0)*pow(x,3.0)+0.0005*pow(x,2.0)-0.3806*x+100.04;
}*/
 
//encoder interrupt functions
void IncrementAndDisplayR() {
  ++encoderCountR;
}
void IncrementAndDisplayL() {
  ++encoderCountL;
}
 
 
 
 

