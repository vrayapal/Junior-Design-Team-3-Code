#include <Servo.h>
#include <ezButton.h>
#include <SpeedyStepper.h>
#include "SparkFun_OPT4048.h"
#include <Wire.h>

SparkFun_OPT4048 myColor;

ezButton limitSwitch1(23); // Limit 1
ezButton limitSwitch2(25); // Limit 2
ezButton limitSwitch3(27); // Limit 3

ezButton limitSwitchL(29); // Button 1 Left  // G-> Digital; Y-> 5V; S-> GND
ezButton limitSwitchR(33); // Button 2 Right // G-> Digital; Y-> 5V; S-> GND

int d=0;

/*//Stepper
int stepPin = 5; 
int dirPin = 4; 
int enPin = 6;
int StepperSpeed = 500;
*/

//Limit Switch Variables
bool limit1 = false;
bool limit2 = false;
bool limit3 = false;

// Button Switch Varaiables
bool LeftButton = false;
bool RightButton = false;

// Gripper Variables
Servo gripper_servo;
int gripper_pos = 80;        // variable for servo position
bool gripper_state = true; // Initializes in the open state
int gripper_closed = 1;     // servo value at which the servo is closed 
int speed = 8; // smaller is faster
int gripper_open = 120;      // servo value at which the servo is closed
int color = 0;              // Initialises variable to hold the color value fromt eh color sensor

// Joystick 1 Variables
int x1Pin = A1;      // variable for joystick pin  (digital)
int y1Pin = A0;      // variable for joystick pin  (digital)
int buttonPin1 = 2;  // variable for button pin (analog in) SW

double xPosition1 = 0;      //position in the x for the joystick
double yPosition1 = 0;      //position in the y for the joystick
int buttonState1 = 0;    //position of the button for the joystick

// Joystick 2 Variables
int x2Pin = A3;      // variable for joystick pin  (digital)
int y2Pin = A2;      // variable for joystick pin  (digital)
int buttonPin2 = 3;  // variable for button pin (analog in) SW

double xPosition2 = 0;      //position in the x for the joystick
double yPosition2 = 0;      //position in the y for the joystick
int buttonState2 = 0;    //position of the button for the joystick

//Stepper Pins
const int MOTORLA_STEP_PIN = 9; // Orange
const int MOTORLA_DIRECTION_PIN = 8; // Yellow

const int MOTOR1_STEP_PIN = 5; // Orange
const int MOTOR1_DIRECTION_PIN = 4; // Yellow

const int MOTOR2_STEP_PIN = 7; // Purple
const int MOTOR2_DIRECTION_PIN = 6; // Yellow
const int ENA_PIN = 38; // Brown

float position = 100.0;

bool grip = true; // true is closed // false is closed 

SpeedyStepper stepperLA;
SpeedyStepper stepper1;
SpeedyStepper stepper2;

void joystick() {
  xPosition1 = analogRead(x1Pin)-511;   // normalized x position from -1 to 1 for joystick 1
  yPosition1 = analogRead(y1Pin)-511;   // normalized y position from -1 to 1 for joystick 1
  buttonState1 = digitalRead(buttonPin1);

  xPosition2 = (analogRead(x2Pin)-511);   // normalized x position from -1 to 1 for joystick 2
  yPosition2 = (analogRead(y2Pin)-511);   // normalized y position from -1 to 1 for joystick 2
  buttonState2 = digitalRead(buttonPin2);
  
  // Debug Stuff
  if(buttonState1 == 0){
    Serial.print("X1: ");
    Serial.print(xPosition1);
    Serial.print(" | Y1: ");
    Serial.print(yPosition1);
    Serial.print(" | ButtonL: ");
    Serial.println(buttonState1);
  }
  if(buttonState2 == 0){
    Serial.print("X2: ");
    Serial.print(xPosition2);
    Serial.print(" | Y2: ");
    Serial.print(yPosition2);
    Serial.print(" | ButtonR: ");
    Serial.println(buttonState2);
  }
}

void buttons() {
  limitSwitchL.loop();
  int L = limitSwitchL.getState();
  if(L == LOW)
    LeftButton = false;
    //Serial.print(LeftButton);
  if (L == HIGH)
     LeftButton = true;
  
  limitSwitchR.loop();
  int R = limitSwitchR.getState();
  if(R == LOW)
    RightButton = false;
    //Serial.println(RightButton);
  if (R == HIGH)
     RightButton = true;
}

void limit(){               // limit switch function for limit switches. To add more add another limit do the following: add a block in this function; intialize 'limit4' above; set debounce time in setup

  limitSwitch1.loop();
  limit1 = limitSwitch1.getState();
    if(limit1 == true){
      limit1 = true;
    }
    else if(limit1 == false){
      limit1 = false;
      Serial.println("Linear Axis 1 Limit Switch Triggered");
      
      //delay(100);
    }  
  limitSwitch2.loop();
  limit2 = limitSwitch2.getState();
    if(limit2 == true ){
      limit2 = true;
      //code to scoot robot away from limit switch 
      //Serial.println("RA1 Limit Switch Triggered");
    }
    else if(limit2 == false){
      limit2 = false;
      Serial.println("RA1 Limit Switch Triggered");
    } 

  limitSwitch3.loop();
  limit3 = limitSwitch3.getState();
    if(limit3 == true ){
      limit3 = true;
      //Serial.println("RA2 Limit Switch Triggered");
      //code scoot robot away from limit switch 
    }
    else if(limit3 == false){
      limit3 = false;
      Serial.println("RA2 Limit Switch Triggered");
    }
}

void gripper(){   
  //delay(2000);                    // function to actuate gripper from open to closed and vice versa. Requires values to be plugged in from 
  //gripper_state = !gripper_state;
       // False is open; True is closed
  if (gripper_state == true && grip == false){          // If gripper is going to close
  //Actuates the gripper from open to close position
    for (gripper_pos = gripper_closed; gripper_pos <= gripper_open; gripper_pos += 1) {  //change pos += 1 depending on if we are opening or closing servo
      gripper_servo.write(gripper_pos);
      //Serial.println(gripper_pos);
      delay(speed);                      // increase if servo moves too fast and vice versa
      //check grip
      //find color of block
    }
    grip = true;
  }
  else if (gripper_state == false && grip == true){    //If gripper is going to open
  //Actuates the gripper from closed to open position
    for (gripper_pos = gripper_open; gripper_pos >= gripper_closed; gripper_pos -= 1) { //change pos -= 1 depending on if we are opening or closing servo
      gripper_servo.write(gripper_pos);
      //Serial.println(gripper_pos);
      delay(speed);                      // increase if servo moves too fast and vice versa
    }
    grip = false;
  }
}

/*
void color (){
  if ((colorsensorvalue>= 0) && (colorsensorvalue<= 1))
    color=0;
    Serial.println("Block is Blue");
  else if ((colorsensorvalue>= 2) && (colorsensorvalue<= 3))
    color=1;
    Serial.println("Block is Green");
  else if ((colorsensorvalue>= 4) && (colorsensorvalue<= 5))
    color=2;
    Serial.println("Block is Red");
  else 
      Serial.println("Color not detected");
}
*/

void homeLA (){
  const float homingSpeedInMMPerSec = 25.0*4;
  const float maxHomingDistanceInMM = 720*4;   // since my lead-screw is 38cm long, should never move more than that
  const int directionTowardHome = 1;        // direction to move toward limit switch: 1 goes positive direction, -1 backward

  stepperLA.moveToHomeInMillimeters(directionTowardHome, homingSpeedInMMPerSec, maxHomingDistanceInMM, 23);

  stepperLA.setStepsPerMillimeter(40 * 4);
  stepperLA.setSpeedInMillimetersPerSecond(26);
  stepperLA.setAccelerationInMillimetersPerSecondPerSecond(50.0);
  stepperLA.moveToPositionInMillimeters(-2.0*10);
  //delay(5000);
}

void home1 (){
  float homingSpeedInStepsPerSec = 50.0;
  float maxHomingDistanceInMM = 720;   // since my lead-screw is 38cm long, should never move more than that
  int directionTowardHome = 1;        // direction to move toward limit switch: 1 goes positive direction, -1 backward
  
  digitalWrite(ENA_PIN, HIGH);
  
  Serial.println("Stepper 1");
  stepper1.moveToHomeInSteps(directionTowardHome, homingSpeedInStepsPerSec, maxHomingDistanceInMM, 25);
  
  digitalWrite(ENA_PIN, LOW);

  homingSpeedInStepsPerSec = 50.0*2;
  maxHomingDistanceInMM = 720;
  directionTowardHome = 1;
  Serial.println("Stepper 2");
  stepper2.moveToHomeInSteps(directionTowardHome, homingSpeedInStepsPerSec, maxHomingDistanceInMM, 27);
  
  stepper1.setSpeedInStepsPerSecond(400*4);
  stepper1.setAccelerationInStepsPerSecondPerSecond(800*2);
  stepper1.moveRelativeInSteps(-20*4);
  

  stepper2.setSpeedInStepsPerSecond(400*4);
  stepper2.setAccelerationInStepsPerSecondPerSecond(800*2);
  stepper2.moveRelativeInSteps(-20*4);
  //delay(5000);
}

void setup() {
  Serial.begin(9600);
  limitSwitch1.setDebounceTime(50);
  limitSwitch2.setDebounceTime(50);
  limitSwitch3.setDebounceTime(50);

  pinMode(x1Pin, INPUT);
  pinMode(y1Pin, INPUT);
  pinMode(buttonPin1, INPUT_PULLUP); 
  
  pinMode(x2Pin, INPUT);
  pinMode(y2Pin, INPUT);
  pinMode(buttonPin2, INPUT_PULLUP); 

  limitSwitchL.setDebounceTime(50);
  limitSwitchR.setDebounceTime(50);
  
  gripper_servo.attach(12); //PWM
  // Brown -> Gnd // Orange -> PWM // Red -> Vcc

  stepperLA.connectToPins(MOTORLA_STEP_PIN, MOTORLA_DIRECTION_PIN);
  stepper1.connectToPins(MOTOR1_STEP_PIN, MOTOR1_DIRECTION_PIN);
  stepper2.connectToPins(MOTOR2_STEP_PIN, MOTOR2_DIRECTION_PIN);
  pinMode(ENA_PIN, OUTPUT);


  //stepper23.setStepsPerMillimeter(40 * 1);
  //stepper23.setSpeedInMillimetersPerSecond(30.0);
  //stepper23.setAccelerationInMillimetersPerSecondPerSecond(25.0);
  /*Wire.begin();
    if (!myColor.begin()) {
        Serial.println("OPT4048 not detected- check wiring or that your I2C address is correct!");
        while (1) ;
    }
    myColor.setBasicSetup();*/
}

void loop() {
  limit();
  /* Debug for limit switchs
  Serial.print(limit1);
  Serial.print(limit2);
  Serial.println(limit3);
  */
  joystick();
  buttons();

  stepperLA.setSpeedInMillimetersPerSecond(10.0);
  stepperLA.setAccelerationInMillimetersPerSecondPerSecond(10.0);
  stepper1.setStepsPerRevolution(800);
  stepper2.setStepsPerRevolution(1600);
  Serial.println("Hello World");

  
  //homeLA();

  //home1();

  //stepperLA.moveToPositionInMillimeters(-135);
  //stepper1.moveToPositionInRevolutions(-0.74);
  //stepper2.moveToPositionInRevolutions(-1.04);
  
  while(true){
    limit();
    joystick();
    buttons();
    if (LeftButton == false){
      Serial.println("Left Button");
      gripper_state=false;
      gripper();
    }
    else if (RightButton == false){
      Serial.println("Right Button");
      gripper_state=true;
      gripper();
    }
    if (yPosition1 >=200){
      stepperLA.setSpeedInStepsPerSecond(8000);
      stepperLA.setAccelerationInStepsPerSecondPerSecond(10000);
      stepperLA.moveRelativeInSteps(-100);
      position = stepperLA.getCurrentPositionInMillimeters();
      Serial.print("Position PA1: ");
      Serial.println(position);
    }
    else if (yPosition1 <=-200){
      stepperLA.setSpeedInStepsPerSecond(8000);
      stepperLA.setAccelerationInStepsPerSecondPerSecond(10000);
      stepperLA.moveRelativeInSteps(100);
      position = stepperLA.getCurrentPositionInMillimeters();
      Serial.print("Position PA1: ");
      Serial.println(position);
    }
    if (xPosition2 <=-200){ // RA1 4x microstepping
        stepper1.setSpeedInStepsPerSecond(80*4);
        stepper1.setAccelerationInStepsPerSecondPerSecond(1000*2);
        stepper1.moveRelativeInSteps(-15*2);
        position = stepper1.getCurrentPositionInRevolutions();
        Serial.print("Position RA1: ");
        Serial.println(position);
        joystick();
      }
    else if (xPosition2 >=200){ // RA1 4x microstepping
      stepper1.setSpeedInStepsPerSecond(80*4);
      stepper1.setAccelerationInStepsPerSecondPerSecond(1000*2);
      stepper1.moveRelativeInSteps(15*2);
      position = stepper1.getCurrentPositionInRevolutions();
      Serial.print("Position RA1: ");
      Serial.println(position);
      joystick();
    }
    if (yPosition2 >=200){ //RA2 8x microstepping
      stepper2.setSpeedInStepsPerSecond(80*8);
      stepper2.setAccelerationInStepsPerSecondPerSecond(1000*2);
      stepper2.moveRelativeInSteps(-15*2);
      position = stepper2.getCurrentPositionInRevolutions();
      Serial.print("Position RA2: ");
      Serial.println(position);
      joystick();
    }
    else if (yPosition2 <=-200){ //RA2 8x microstepping
      stepper2.setSpeedInStepsPerSecond(80*8);
      stepper2.setAccelerationInStepsPerSecondPerSecond(1000*2);
      stepper2.moveRelativeInSteps(15*2);
      position = stepper2.getCurrentPositionInRevolutions();
      Serial.print("Position RA2: ");
      Serial.println(position);
      joystick();
    }
  }
}


