/* Encoder Test
*** Description ***
Demonstrates how to count turns of an encoder (Lego pulley) wheel. In addition to the wheel you will need:
--A DC Motor.
--A micro switch.
--A break-beam sensor.
--An LCD screen (optional).
The wheel should be attached to a rotating axle, directly or indirectly attached to the motor, and placed
near the break-beam sensor so that as it turns it alternately blocks and allows the beam through.
*** History ***
--10/4/12:  Modified by Blane for readability and simpler logic.
*/

#include <AFMotor.h>
#include <SoftwareSerial.h>

// Define constants:
#define DEBUG 1;
#define switchPinR 9 //Right switch
#define switchPinL 10 //Right switch  
#define txPin 13   // LCD tx pin.
#define rxPin 13   // LCD rx pin (not really used).
#define encoderPin 2   // Encoder i.e. break-beam sensor (2 or 3 only, to allow hardware interrupt).
#define encoderPin1 3  //CANT USE THIS IF MOTOR PIN 2 IS USED
#define right_Motor 1   // (1-4 only).
#define left_Motor 2   // (1-4 only).

// Define (and initialize) global variables:
volatile int encoderCount;   // Use "volatile" for faster updating of value during hardware interrupts.
volatile int encoderCount1;
int encoderCountGoal = 1000;
int distance = encoderCountGoal/6/25*6.85;

//User defined variables
int speedCount = 0;
int fastCount = 0;
int slowCount = 0;
int medCount = 0;

int fastSpeed = 255;
int medSpeed = 200;
int slowSpeed = 150;
int smallD = 12; //Distance to travel in INCHES
int medD = 24;
int bigD = 36;

// Define serial display and motor objects:
SoftwareSerial mySerial =  SoftwareSerial( rxPin, txPin);  
//AF_DCMotor rightMotor(right_Motor, MOTOR12_64KHZ); 
//AF_DCMotor leftMotor(left_Motor, MOTOR12_64KHZ);

AF_DCMotor right_motor(3, MOTOR12_1KHZ);// create motor #1 64 KHZ pwm
AF_DCMotor left_motor(4, MOTOR12_1KHZ); // create motor #2, 64KHz pwm



void setup() {
  
  // Setup hardware interrupt:
  int interruptPin = encoderPin - 2;   // Hardware interrupt pin (0 or 1 only, to refer to digital pin 2 or 3, respectively).
  attachInterrupt( interruptPin, IncrementAndDisplay, FALLING);   // Attach interrupt pin, name of function to be called 
        // during interrupt, and whether to run interrupt upon voltage FALLING from high to low or ...

  int interruptPin1 = encoderPin1 - 2;   // Hardware interrupt pin (0 or 1 only, to refer to digital pin 2 or 3, respectively).
  attachInterrupt( interruptPin1, IncrementAndDisplay1, FALLING);
  
  // Setup encoder i.e. break-beam:
  pinMode( encoderPin, INPUT); 
  digitalWrite( encoderPin, HIGH);  

  pinMode( encoderPin1, INPUT); 
  digitalWrite( encoderPin1, HIGH);
  
  // Setup switch:
  pinMode( switchPinR, INPUT); 
  digitalWrite( switchPinR, HIGH);

  pinMode( switchPinL, INPUT); 
  digitalWrite( switchPinL, HIGH);
  
  // Setup serial display:
  pinMode( txPin, OUTPUT);
  mySerial.begin(9600); 

  right_motor.setSpeed(255);
  left_motor.setSpeed(255);
        
  mySerial.print("?f");          //Clears LCD screen
  mySerial.print("?x00?y0");     //Sets Cursor to x00,y0
  mySerial.print("Press to Begin...");    //Displays "Press to Begin..."


}


void loop() {
  mySerial.print("?f");
  mySerial.print("?x00?y0");
  mySerial.print("S      M       F");
  
  while (speedCount < 3) { // Wait until switch is pressed.

    if(!digitalRead(switchPinL) && !digitalRead(switchPinR)){
      medCount++;
      speedCount++;
      delay(500);
    }else if(!digitalRead(switchPinR)){
      fastCount++;
      speedCount++;
      delay(500);
    }else if(!digitalRead(switchPinL)){
      slowCount++;
      speedCount++;
      delay(500);
    }else {
      //nothing
    }
  
  }

  speedCount = 0; //Reset speed count so that a distance can be selected
  fastCount = 0;
  slowCount = 0;
  medCount = 0;

  right_motor.setSpeed(returnSpeed(fastCount,medCount,slowCount)); //Sets the speed of myMotor
  left_motor.setSpeed(returnSpeed(fastCount,medCount,slowCount));
  delay(1000);


  mySerial.print("?f");
  mySerial.print("?x00?y0");
  mySerial.print("SMALL       LONG");
  mySerial.print("?x00?y1");
  mySerial.print("     MEDIUM      ");
  
  while (speedCount < 3) { // Wait until switch is pressed.

    if(!digitalRead(switchPinL) && !digitalRead(switchPinR)){
      medCount++;
      speedCount++;
      delay(500);
    }else if(!digitalRead(switchPinR)){
      fastCount++;
      speedCount++;
      delay(500);
    }else if(!digitalRead(switchPinL)){
      slowCount++;
      speedCount++;
      delay(500);
    }else {
      //nothing
    }
  }

  encoderCountGoal = returnGoal(fastCount,medCount,slowCount);

  delay(1000);
  
  mySerial.print("?f");
  mySerial.print("?x00?y0");
  mySerial.print("Counts:");  

  left_motor.run(FORWARD);
  right_motor.run(FORWARD);

  
  encoderCount = 0; 
  encoderCount1 = 0;  
  while (encoderCount < encoderCountGoal || encoderCount1 < encoderCountGoal) {
    if(encoderCount > encoderCountGoal) {
      right_motor.run(RELEASE);
    }
    if(encoderCount1 > encoderCountGoal){
      left_motor.run(RELEASE);
    }
    if(!digitalRead(switchPinL) || !digitalRead(switchPinR)){
      right_motor.run(BACKWARD);
      left_motor.run(BACKWARD);
      delay(1000);
      break;
    }
    mySerial.print("?x11?y1");
    mySerial.print(encoderCount);
    mySerial.print("?x01?y1");
    mySerial.print(encoderCount1);
    mySerial.print("?x06?y1");
    mySerial.print(encoderCountGoal);
  }   
  
  right_motor.run(RELEASE);
  left_motor.run(RELEASE);
  
  speedCount = 0; //Reset speed count so that a speed can be selected again
  fastCount = 0;
  slowCount = 0;
  medCount = 0;
}


void IncrementAndDisplay() {
  ++encoderCount; 
}

void IncrementAndDisplay1() {
  ++encoderCount1; 
}

int returnGoal(int fastCount, int medCount, int slowCount) {
  int maximum;
  maximum = max(fastCount,medCount);
  maximum = max(maximum,slowCount);

  if(maximum == fastCount){
    #ifdef DEBUG
      mySerial.print("?f");          //Clears LCD screen
      mySerial.print("?x00?y0");     //Sets Cursor to x00,y0
      mySerial.print("Long Distance");    //Displays "Press to Begin..."
    #endif
    return bigD*22;
  }else if(maximum == medCount){
    #ifdef DEBUG
      mySerial.print("?f");          //Clears LCD screen
      mySerial.print("?x00?y0");     //Sets Cursor to x00,y0
      mySerial.print("Medium Distance");    //Displays "Press to Begin..."
    #endif
    return medD*22;
  }else {
    #ifdef DEBUG
      mySerial.print("?f");          //Clears LCD screen
      mySerial.print("?x00?y0");     //Sets Cursor to x00,y0
      mySerial.print("Small Distance");    //Displays "Press to Begin..."
    #endif
    return smallD*22;
  }
}

int returnSpeed(int fastCount, int medCount, int slowCount) {
  int maximum;
  maximum = max(fastCount,medCount);
  maximum = max(maximum,slowCount);

  if(maximum == fastCount){
    #ifdef DEBUG
      mySerial.print("?f");          //Clears LCD screen
      mySerial.print("?x00?y0");     //Sets Cursor to x00,y0
      mySerial.print("Fast Speed!");    //Displays "Press to Begin..."
    #endif
    return fastSpeed;
  }else if(maximum == medCount){
    #ifdef DEBUG
      mySerial.print("?f");          //Clears LCD screen
      mySerial.print("?x00?y0");     //Sets Cursor to x00,y0
      mySerial.print("Medium Speed!");    //Displays "Press to Begin..."
    #endif
    return medSpeed;
  }else {
    #ifdef DEBUG
      mySerial.print("?f");          //Clears LCD screen
      mySerial.print("?x00?y0");     //Sets Cursor to x00,y0
      mySerial.print("Slow Speed!");    //Displays "Press to Begin..."
    #endif
    return slowSpeed;
  }
}



