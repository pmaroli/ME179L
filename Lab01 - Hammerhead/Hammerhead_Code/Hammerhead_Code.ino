//Hammerhead Code
//When 'bumped' Hammerhead will move forward until an object is hit, then back up and hit it again... and again...

//Include the needed libraries
#include <AFMotor.h>
#include <SoftwareSerial.h>
//Define Constants
#define DEBUG 1  //By setting DEBUG to either 0 or 1, the program will exclude or include the print statements when it compiles
#define SwitchRPin 0   //Right switch is connected to this pin
#define SwitchLPin 1  //Left switch is connected to this pin 
#define LCDTxPin 13   // LCD connected to this pin (14 is analog 0)
#define DummyRxPin 4  // Not used by LCD Board, can be any unused pin  
#define SPEED 150     // Set Speed to be used for motors

//Additionally defined variables
#define turnTime 1500  //Amount of time that the robot should execute the "turn" command for
bool right; //Boolean variable that determines if the right switch was triggered

SoftwareSerial mySerial =  SoftwareSerial(DummyRxPin, LCDTxPin);  //Change Tx and Rx Pins to pins of our choosing
AF_DCMotor Right_Motor(3, MOTOR34_1KHZ); // create right motor on port 3, 1KHz pwm
AF_DCMotor Left_Motor(4, MOTOR34_1KHZ); // create left motor on port 4, 1KHz pwm
//Options for PWM, from quiet but power hungry to loud but lower power cost: MOTOR12_64KHZ, MOTOR12_8KHZ, MOTOR12_2KHZ, or MOTOR12_1KHZ
//(Only for ports 1 and 2, ports 3 and 4 are(and can only be) set to MOTOR34_1KHZ

void setup(){
  pinMode(LCDTxPin, OUTPUT);
  mySerial.begin(9600);  //Set baud rate for the LCD serial communication
  mySerial.print("?f"); //Sends clear screen command to LCD
  pinMode(SwitchRPin, INPUT); //Makes Switch Pin an input
  pinMode(SwitchLPin, INPUT); //Makes Switch Pin an input
  digitalWrite(SwitchLPin, HIGH);   //Enables the pull-up resistor on Switch Pin
  digitalWrite(SwitchRPin, HIGH);   //Enables the pull-up resistor on Switch Pin
  Right_Motor.setSpeed(SPEED);
  Left_Motor.setSpeed(SPEED);
  
  mySerial.print("?x00?y0");  //Move cursor to position x=0 and y=0 on the LCD display
  mySerial.print("Bump to begin...");
  while(digitalRead(SwitchLPin) && digitalRead(SwitchRPin))  {
   //Wait for switch to be pressed 
  }
  mySerial.print("?x00?y1");  //Move cursor to position x=0 and y=1 on the LCD display
  mySerial.print("Starting!");
  delay(1000);
  mySerial.print("?f"); //Sends clear screen command to LCD
  
}

void loop(){
  
  #ifdef DEBUG
    mySerial.print("?f"); //Sends clear screen command to LCD
    mySerial.print("?x00?y0");  //Move cursor to position x=0 and y=0 on the LCD display
    mySerial.print("Onward!");
  #endif
  
  DriveForward();

  while(true){  //Within this while loop, the switch that is triggered is determined
    if(!digitalRead(SwitchLPin)){ //Check if the left switch was triggered
      
      #ifdef DEBUG
        mySerial.print("?x00?y0");  //Move cursor to position x=0 and y=0 on the LCD display
        mySerial.print("Left Switch Hit!"); //Debuggin on LCD
      #endif

      right = false; //Right switch was NOT triggered
      break; //break out of the while loop if a switch is triggered
      
    }else if(!digitalRead(SwitchRPin)){  //else check if the right pin was triggered

      #ifdef DEBUG
        mySerial.print("?x00?y0");  //Move cursor to position x=0 and y=0 on the LCD display
        mySerial.print("Right Switch Hit!"); //Debugging on LCD
      #endif

      right = true; //Right switch WAS triggered
      break; //break out of the while loop if a switch is triggered
    }
  }

  DriveBackward(); //Drive backwards a little bit before turning in a "smart" direction
  delay(1000);

  //Now turn in a smart direction depending on which switch was triggered
  if(right){

    #ifdef DEBUG
        mySerial.print("?f"); //Sends clear screen command to LCD
        mySerial.print("?x00?y0");  //Move cursor to position x=0 and y=0 on the LCD display
        mySerial.print("Turning Left!"); //Debugging on LCD
    #endif
    TurnLeft(); //If the RIGHT switch was triggered, then turn LEFT
    delay(turnTime); //turnTime is a variable determining how long the motors should turn for
    
  }else {

    #ifdef DEBUG
        mySerial.print("?f"); //Sends clear screen command to LCD
        mySerial.print("?x00?y0");  //Move cursor to position x=0 and y=0 on the LCD display
        mySerial.print("Turning Right!"); //Debugging on LCD
    #endif
    TurnRight(); //If the LEFT switch was triggered, then turn RIGHT
    delay(turnTime);
    
  }
}


//Function definitions
void DriveForward(){
  Right_Motor.run(FORWARD);
  Left_Motor.run(FORWARD);
}

void DriveBackward(){
  Right_Motor.run(BACKWARD);
  Left_Motor.run(BACKWARD);
}

void TurnRight(){
  Right_Motor.run(BACKWARD);
  Left_Motor.run(FORWARD);
}

void TurnLeft(){
  Right_Motor.run(FORWARD);
  Left_Motor.run(BACKWARD);
}


