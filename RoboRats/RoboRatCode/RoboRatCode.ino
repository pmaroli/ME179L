//Include the needed libraries
#include <AFMotor.h>
#include <SoftwareSerial.h>
//Define Constants
#define DEBUG 1  //By setting DEBUG to either 0 or 1, the program will exclude or include the print statements when it compiles
#define LCDTxPin 13   // LCD connected to this pin (14 is analog 0)  
#define SPEED 250     // Set Speed to be used for motors
 
SoftwareSerial mySerial =  SoftwareSerial(4, LCDTxPin);  //Change Tx and Rx Pins to pins of our choosing
AF_DCMotor Right_Motor(3, MOTOR34_1KHZ); 
AF_DCMotor Left_Motor(4, MOTOR34_1KHZ); 
AF_DCMotor CheeseGrabber(1, MOTOR12_1KHZ); //Run backwards is nominal direction for grabbing cheese
 
void setup(){
  pinMode(LCDTxPin, OUTPUT);
  mySerial.begin(9600);  //Set baud rate for the LCD serial communication
  mySerial.print("?f"); //Sends clear screen command to LCD
  Right_Motor.setSpeed(SPEED);
  Left_Motor.setSpeed(SPEED);
  CheeseGrabber.setSpeed(SPEED);
  
}
 
void loop(){

  CheeseGrabber.run(BACKWARD);



}
