#include <AFMotor.h>
#include <SoftwareSerial.h>
#define SwitchPin_A 1   // switch A is connected to pin 1
#define SwitchPin_B 2   // switch B is connected to pin 2

const int rxPin = 13;
const int txPin = 13;
const int analogInPin = A0;  // Analog input pin that the SHORT range sensor is attached to
const int longPin = A1;  // Analog input pin that the LONG range sensor is attached to
int sensorValue = 0; //Short range sensor value
int longValue = 0; //Long range sensor value

int countR = 0;
int countL = 0;
int countS = 0;
int diff = 0;
int maxdiff = 30;

SoftwareSerial mySerial =  SoftwareSerial(rxPin,txPin);
AF_DCMotor right_motor(3, MOTOR12_64KHZ);// create motor #1 64 KHZ pwm
AF_DCMotor left_motor(4, MOTOR12_64KHZ); // create motor #2, 64KHz pwm

void setup() {
pinMode( txPin, OUTPUT);
mySerial.begin(9600); // initialize serial communications at 9600 bps:

left_motor.setSpeed(255); // Speeds are set differently to compensate for difference in motor power
right_motor.setSpeed(200); 
}

int setpoint = 415; //Setpoint distance from the wall for the short range sensor

void loop(){ 

  sensorValue = analogRead(analogInPin);
  longValue = analogRead(longPin);

  // print the results to the serial monitor:
  mySerial.print("?x00?y0");     //Sets Cursor to x00,y0 
  mySerial.print("Short = ");
  mySerial.print(sensorValue);
  mySerial.print("    ");
  mySerial.print("?x00?y1");     //Sets Cursor to x00,y0 
  mySerial.print("Long = ");
  mySerial.print(longValue);
  mySerial.print("    ");
  
  diff = sensorValue - setpoint;

  if(longValue > 370) {
    countS++;
    if(countS > 2){
      right_motor.run(FORWARD);
      left_motor.run(BACKWARD);
      delay(750);
      countR = 0; countL = 0; countS = 0;
      left_motor.run(FORWARD);  
    }
  }else if(diff < maxdiff && diff > -maxdiff){
    left_motor.run(FORWARD);
    right_motor.run(FORWARD);
    countR = 0; countL = 0;
  }else if(diff < -maxdiff) {
  countR++;
    if(countR > 5){
      right_motor.run(RELEASE); 
      left_motor.run(FORWARD); 
    }
  }else if(diff > maxdiff) {
    countL++;
    if(countL > 5){
      left_motor.run(RELEASE);
      right_motor.run(FORWARD);  
    }
  }else {
    countR = 0; countL = 0; countS = 0;
  }

 // delay(2);
}
